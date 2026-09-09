#include "StrengthTransient.hpp"
#include "TriangleMeshSlicer.hpp"
#include <Eigen/SparseCholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <map>
#include <queue>
#include <stdexcept>

namespace Slic3r::StrengthAnalysis {
namespace {
struct Cancelled {};
struct Cell { Vec3d p; Vec3d size; Vec3d u{Vec3d::Zero()}; int layer; double solid; };
struct Bond {
    size_t a, b;
    Vec3d n;
    double length, area, e, g, yield, ultimate, shear;
    double plastic{0.0}, accumulated{0.0}, shear_accumulated{0.0};
    Vec3d plastic_shear{Vec3d::Zero()};
    bool failed{false};
};
struct Trial {
    double plastic, accumulated, shear_accumulated;
    Vec3d plastic_shear, force;
    Matrix3d tangent;
    double normal, shear;
};
// Backward-Euler return mapping with isotropic hardening. Trial variables are
// committed only after global equilibrium converges, never during Newton iterations.
Trial constitutive(const Bond &b, const Vec3d &relative, const TransientSettings &settings)
{
    const Matrix3d nn = b.n * b.n.transpose();
    const Matrix3d transverse = Matrix3d::Identity() - nn;
    const double strain = relative.dot(b.n) / b.length;
    double normal = b.e * (strain - b.plastic);
    double tangent_e = b.e;
    Trial t{b.plastic, b.accumulated, b.shear_accumulated, b.plastic_shear, Vec3d::Zero(), Matrix3d::Zero(), 0.0, 0.0};
    const double h = b.e * settings.hardening_ratio;
    if (settings.plasticity && std::abs(normal) > b.yield + h * b.accumulated) {
        const double delta = (std::abs(normal) - b.yield - h * b.accumulated) / (b.e + h);
        t.plastic += std::copysign(delta, normal);
        t.accumulated += delta;
        normal = b.e * (strain - t.plastic);
        tangent_e = b.e * h / (b.e + h);
    }
    const Vec3d trial_shear = b.g * (transverse * relative / b.length - b.plastic_shear);
    Vec3d shear = trial_shear;
    Matrix3d tangent_g = b.g * transverse;
    const double hs = b.g * settings.hardening_ratio;
    const double radius = b.shear + hs * b.shear_accumulated;
    if (settings.plasticity && trial_shear.norm() > radius) {
        const Vec3d direction = trial_shear.normalized();
        const double delta = (trial_shear.norm() - radius) / (b.g + hs);
        t.plastic_shear += delta * direction;
        t.shear_accumulated += delta;
        shear -= b.g * delta * direction;
        const double radial = b.g * hs / (b.g + hs);
        tangent_g = b.g * (shear.norm() / trial_shear.norm()) * (transverse - direction * direction.transpose()) +
                    radial * direction * direction.transpose();
    }
    t.normal = normal;
    t.shear = shear.norm();
    t.force = b.area * (normal * b.n + shear);
    t.tangent = b.area / b.length * (tangent_e * nn + tangent_g);
    return t;
}
}

TransientResult analyze_transient(const indexed_triangle_set &mesh, const Setup &setup, const TransientSettings &s,
                                  const CancelPredicate &cancel, const std::function<void(int)> &progress)
{
    TransientResult out;
    out.settings = s;
    out.geometry_scale = setup.geometry_scale;
    const auto check = [&] { if (cancel && cancel()) throw Cancelled{}; };
    const auto fail = [&](const std::string &message) {
        out.status = AnalysisStatus::InvalidInput;
        out.message = message;
        out.frames.clear();
        return out;
    };
    const auto errors = validate(mesh, setup);
    if (!errors.empty()) return fail(errors.front());
    for (double value : {s.duration_s, s.layer_height_mm, s.first_layer_height_mm, s.cell_width_mm, s.line_width_mm,
                         s.cooling_time_s, s.layer_time_s, s.bonding_scale, s.hardening_ratio, s.failure_plastic_strain})
        if (!std::isfinite(value) || value <= 0.0) return fail("Time, dimensions and constitutive parameters must be finite and positive.");
    for (double value : {s.nozzle_temperature_c, s.bed_temperature_c, s.chamber_temperature_c, s.reference_nozzle_temperature_c})
        if (!std::isfinite(value)) return fail("Temperatures must be finite.");
    if (s.increments < 4 || s.increments > 200 || s.maximum_cells < 8 || s.maximum_cells > 10000 || s.wall_loops < 0 ||
        s.hardening_ratio > 1.0 || s.bonding_scale > 1.0 ||
        (s.thermal_bonding && (s.nozzle_temperature_c <= s.chamber_temperature_c ||
                              s.reference_nozzle_temperature_c <= s.chamber_temperature_c)))
        return fail("Invalid sampling, cell budget, hardening, bonding or thermal settings.");
    try {
        check();
        const Vec3d z = setup.print_layer_axis.cwiseQuotient(setup.geometry_scale).normalized();
        const Vec3d x = z.unitOrthogonal();
        const Vec3d y = z.cross(x);
        Matrix3d basis;
        basis.col(0) = x; basis.col(1) = y; basis.col(2) = z;
        indexed_triangle_set aligned = mesh;
        Vec3d low = Vec3d::Constant(std::numeric_limits<double>::infinity()), high = -low;
        for (Vec3f &v : aligned.vertices) {
            v = (basis.transpose() * v.cast<double>().cwiseProduct(setup.geometry_scale)).cast<float>();
            low = low.cwiseMin(v.cast<double>()); high = high.cwiseMax(v.cast<double>());
        }
        std::vector<float> heights;
        std::vector<double> thickness;
        double bottom = low.z();
        while (bottom < high.z() - 1e-6) {
            const double h = std::min(high.z() - bottom, s.layers ?
                (heights.empty() ? s.first_layer_height_mm : s.layer_height_mm) : s.cell_width_mm);
            heights.push_back(float(bottom + h * 0.5)); thickness.push_back(h); bottom += h;
            if (heights.size() > 2000) return fail("More than 2000 physical layers; choose a smaller part or disable layer resolution.");
        }
        out.layer_count = heights.size();
        const auto slices = slice_mesh_ex(aligned, heights, check);
        const auto inside = [&](int layer, double px, double py) {
            const Point p(scale_(px), scale_(py));
            for (const ExPolygon &poly : slices[size_t(layer)]) if (poly.contains(p)) return true;
            return false;
        };
        const int nx = std::max(1, int(std::ceil((high.x() - low.x()) / s.cell_width_mm)));
        const int ny = std::max(1, int(std::ceil((high.y() - low.y()) / s.cell_width_mm)));
        if (double(nx) * ny * heights.size() > 2000000.0) return fail("Cell grid exceeds the work limit. Increase in-plane cell width.");
        const double dx = (high.x() - low.x()) / nx, dy = (high.y() - low.y()) / ny;
        std::vector<Cell> cells;
        std::map<std::array<int, 3>, size_t> grid;
        for (int k = 0; k < int(heights.size()); ++k) {
            check();
            // Newton cooling of the previous layer before contact with a new bead.
            // Dimensionless weld proxy normalized to a user-supplied reference process.
            const double ambient = s.chamber_temperature_c + (s.bed_temperature_c - s.chamber_temperature_c) *
                std::exp(-(heights[size_t(k)] - low.z()) / 2.0);
            const double previous = ambient + (s.nozzle_temperature_c - ambient) * std::exp(-s.layer_time_s / (s.cooling_time_s * std::pow(thickness[size_t(k)] / 0.2, 2.0)));
            const double interface_t = 0.5 * (s.nozzle_temperature_c + previous);
            const double reference_previous = 25.0 + (s.reference_nozzle_temperature_c - 25.0) * std::exp(-1.0);
            const double reference_t = 0.5 * (s.reference_nozzle_temperature_c + reference_previous);
            const double thermal = s.thermal_bonding ? std::clamp(
                (interface_t - s.chamber_temperature_c) / std::max(1.0, reference_t - 25.0), 0.05, 1.5) : 1.0;
            out.layer_bond_factors.push_back(std::clamp(s.bonding_scale * thermal, 0.01, 1.0));
            out.layer_interface_temperature_c.push_back(interface_t);
            for (int j = 0; j < ny; ++j) for (int i = 0; i < nx; ++i) {
                const double px = low.x() + (i + 0.5) * dx, py = low.y() + (j + 0.5) * dy;
                if (!inside(k, px, py)) continue;
                const double wall = s.wall_loops * s.line_width_mm;
                const bool shell = wall > 0.0 && (!inside(k, px-wall, py) || !inside(k, px+wall, py) ||
                                                   !inside(k, px, py-wall) || !inside(k, px, py+wall));
                const double solid = shell ? 1.0 : std::clamp(setup.infill.background_density, 0.01, 1.0);
                grid[{i,j,k}] = cells.size();
                cells.push_back({Vec3d(px, py, heights[size_t(k)]), Vec3d(dx, dy, thickness[size_t(k)]), Vec3d::Zero(), k, solid});
                if (cells.size() > s.maximum_cells || cells.size() * (s.increments + 1) > 400000)
                    return fail("Layer grid exceeds the memory budget. Increase in-plane cell width; physical layers are never silently merged.");
            }
        }
        if (cells.size() < 2) return fail("Grid does not resolve this part. Reduce in-plane cell width.");
        const Material material = setup.material.calibrated();
        const std::array<double,6> pattern_stiffness{{0.82,0.94,1.05,0.98,1.03,1.0}};
        const std::array<double,6> pattern_strength{{0.82,0.91,1.02,0.96,1.0,1.0}};
        const size_t pattern = std::min(size_t(setup.infill.background_pattern), size_t(5));
        std::vector<Bond> bonds;
        std::vector<std::vector<size_t>> adjacency(cells.size());
        for (const auto &[index, a] : grid) {
            check();
            for (const auto &offset : {std::array<int,3>{1,0,0}, {0,1,0}, {0,0,1}}) {
                const auto it = grid.find({index[0]+offset[0], index[1]+offset[1], index[2]+offset[2]});
                if (it == grid.end()) continue;
                const size_t b = it->second;
                const bool interlayer = cells[a].layer != cells[b].layer;
                const Vec3d delta = (cells[b].p - cells[a].p) * 1e-3;
                const Vec3d mid = (cells[b].p + cells[a].p) * 0.5;
                if (!inside(cells[a].layer, mid.x(), mid.y()) || !inside(cells[b].layer, mid.x(), mid.y())) continue;
                const double factor = interlayer ? out.layer_bond_factors[size_t(cells[b].layer)] : 1.0;
                const double density = std::min(cells[a].solid, cells[b].solid);
                const double stiffness_factor = density + (1.0-density)*pattern_stiffness[pattern];
                const double strength_factor = density + (1.0-density)*pattern_strength[pattern];
                const double area = (offset[0] ? dy * cells[a].size.z() : offset[1] ? dx * cells[a].size.z() : dx * dy) *
                    1e-6 * std::min(cells[a].solid, cells[b].solid);
                bonds.push_back({a,b, basis * delta.normalized(),delta.norm(),area,
                    (interlayer ? material.elastic_modulus_z_pa : material.elastic_modulus_xy_pa) * factor * stiffness_factor,
                    (interlayer ? material.shear_modulus_xz_pa : material.shear_modulus_xy_pa) * factor * stiffness_factor,
                    (interlayer ? material.yield_strength_z_pa : material.yield_strength_xy_pa) * factor * strength_factor,
                    (interlayer ? material.ultimate_strength_z_pa : material.ultimate_strength_xy_pa) * factor * strength_factor,
                    (interlayer ? material.shear_strength_xz_pa : material.shear_strength_xy_pa) * factor * strength_factor});
                adjacency[a].push_back(bonds.size()-1); adjacency[b].push_back(bonds.size()-1);
            }
        }
        if (bonds.empty()) return fail("No connected cells. Refine the in-plane grid.");
        const auto raw = [&](const Vec3d &p) -> Vec3d { return (basis * p).cwiseQuotient(setup.geometry_scale); };
        // Geometric regions select cell centers; surface regions transfer their exact
        // source vertices to nearest cells. No load is reassigned after detachment.
        const auto selection = [&](const Load &load) {
            std::vector<size_t> selected;
            if (load.type == LoadType::GlobalForce || load.region.whole_model) {
                for (size_t i=0;i<cells.size();++i) selected.push_back(i);
            } else if (load.region.shape != RegionShape::Surface) {
                for (size_t i=0;i<cells.size();++i) if (load.region.contains(raw(cells[i].p))) selected.push_back(i);
            }
            if (selected.empty()) {
                const auto vertices = vertices_in_region(mesh, load.region);
                for (size_t vertex : vertices) {
                    const Vec3d p = mesh.vertices[vertex].cast<double>().cwiseProduct(setup.geometry_scale);
                    size_t nearest=0; double distance=std::numeric_limits<double>::infinity();
                    for (size_t i=0;i<cells.size();++i) {
                        const double d=(basis*cells[i].p-p).squaredNorm();
                        if(d<distance) { distance=d;nearest=i; }
                    }
                    selected.push_back(nearest);
                }
                std::sort(selected.begin(),selected.end());
                selected.erase(std::unique(selected.begin(),selected.end()),selected.end());
            }
            return selected;
        };
        std::vector<bool> fixed(cells.size(),false);
        for(const Load &load:setup.loads) if(load.active && load.type==LoadType::Fixed)
            for(size_t i:selection(load)) fixed[i]=true;
        struct Force { std::vector<size_t> cells; Vec3d vector; };
        std::vector<Force> forces;
        for(const Load &load:setup.loads) {
            if(!load.active || load.type==LoadType::Fixed) continue;
            auto selected=selection(load);
            selected.erase(std::remove_if(selected.begin(),selected.end(),[&](size_t i){return fixed[i];}),selected.end());
            if(selected.empty()) return fail("A force maps only to fixed cells. Refine the grid or separate the load and support regions.");
            const double magnitude=load.magnitude_n*(load.type==LoadType::ImpactForce?load.impact_factor:1.0);
            forces.push_back({selected,load.direction.normalized()*magnitude});
            out.probes.push_back({load.name,selected.front(),magnitude,0.0});
        }
        if(setup.gravity.enabled) {
            std::vector<size_t> all;
            for(size_t i=0;i<cells.size();++i) if(!fixed[i]) all.push_back(i);
            // Gravity is accumulated separately below by physical cell mass.
            out.probes.push_back({"Gravity", all.empty()?0:all.front(),0.0,0.0});
        }
        const auto supported = [&] {
            std::vector<bool> reached=fixed;
            std::queue<size_t> queue;
            for(size_t i=0;i<fixed.size();++i) if(fixed[i]) queue.push(i);
            while(!queue.empty()) {
                size_t i=queue.front();queue.pop();
                for(size_t edge:adjacency[i]) if(!bonds[edge].failed) {
                    size_t j=bonds[edge].a==i?bonds[edge].b:bonds[edge].a;
                    if(!reached[j]) {reached[j]=true;queue.push(j);}
                }
            }
            return reached;
        };
        auto attached=supported();
        if(std::find(attached.begin(),attached.end(),false)!=attached.end())
            return fail("The discretized part contains unsupported cells. Refine the grid or constrain each component.");
        // Render each finite cell independently: damaged interfaces can visibly open.
        const std::array<Vec3i32,12> faces{{{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},
                                          {1,2,6},{1,6,5},{2,3,7},{2,7,6},{3,0,4},{3,4,7}}};
        for(size_t i=0;i<cells.size();++i) {
            const int base=int(out.display_mesh.vertices.size());
            for(const Vec3d &corner : {Vec3d(-1,-1,-1),Vec3d(1,-1,-1),Vec3d(1,1,-1),Vec3d(-1,1,-1),
                                       Vec3d(-1,-1,1),Vec3d(1,-1,1),Vec3d(1,1,1),Vec3d(-1,1,1)}) {
                out.display_mesh.vertices.push_back(raw(cells[i].p+0.5*corner.cwiseProduct(cells[i].size)).cast<float>());
                out.display_cells.push_back(i);
            }
            for(const auto &face:faces) out.display_mesh.indices.push_back(face+Vec3i32::Constant(base));
        }
        const size_t count=cells.size();
        std::vector<Vec3d> u(count,Vec3d::Zero());
        for(size_t step=0;step<=s.increments;++step) {
            check();
            const double phase=double(step)/s.increments;
            const double fraction=s.unload ? (phase<=0.5?2.0*phase:2.0*(1.0-phase)) : phase;
            const double time=phase*s.duration_s;
            std::vector<Trial> trials(bonds.size());
            std::vector<Vec3d> external(count,Vec3d::Zero());
            bool equilibrated=false;
            // Each failure pass removes bonds permanently, recomputes support connectivity
            // and solves equilibrium again. Detached cells retain their last position.
            for(size_t damage_pass=0;damage_pass<=bonds.size();++damage_pass) {
                check();
                attached=supported();
                std::fill(external.begin(),external.end(),Vec3d::Zero());
                for(const Force &force:forces) for(size_t i:force.cells) if(attached[i])
                    external[i]+=fraction*force.vector/double(force.cells.size());
                if(setup.gravity.enabled) for(size_t i=0;i<count;++i) if(attached[i] && !fixed[i])
                    external[i]+=fraction*setup.gravity.acceleration_m_s2*material.density_kg_m3*cells[i].size.prod()*1e-9*cells[i].solid;
                std::vector<int> dof(count,-1); int free_count=0;
                for(size_t i=0;i<count;++i) if(attached[i] && !fixed[i]) {dof[i]=free_count;free_count+=3;}
                bool converged=free_count==0;
                double reference=0.0;
                for(const Vec3d &f:external) reference+=f.squaredNorm();
                double last_residual = 0.0;
                for(int iteration=0;iteration<60 && !converged;++iteration) {
                    check();
                    Eigen::VectorXd residual=Eigen::VectorXd::Zero(free_count);
                    std::vector<Eigen::Triplet<double>> entries;
                    entries.reserve(bonds.size()*36);
                    for(size_t i=0;i<count;++i) if(dof[i]>=0) residual.segment<3>(dof[i])=external[i];
                    for(size_t j=0;j<bonds.size();++j) {
                        const Bond &b=bonds[j];
                        if(b.failed || !attached[b.a] || !attached[b.b]) continue;
                        const Trial trial=constitutive(b,u[b.b]-u[b.a],s);
                        trials[j]=trial;
                        const int a=dof[b.a], c=dof[b.b];
                        if(a>=0) residual.segment<3>(a)+=trial.force;
                        if(c>=0) residual.segment<3>(c)-=trial.force;
                        for(int r=0;r<3;++r) for(int col=0;col<3;++col) {
                            const double k=trial.tangent(r,col);
                            if(a>=0) entries.emplace_back(a+r,a+col,k);
                            if(c>=0) entries.emplace_back(c+r,c+col,k);
                            if(a>=0 && c>=0) {entries.emplace_back(a+r,c+col,-k);entries.emplace_back(c+r,a+col,-k);}
                        }
                    }
                    last_residual = residual.norm();
                    if(residual.norm()<=1e-7*std::max(1.0,std::sqrt(reference))) {converged=true;break;}
                    Eigen::SparseMatrix<double> matrix(free_count,free_count);
                    matrix.setFromTriplets(entries.begin(),entries.end());
                    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(matrix);
                    if(solver.info()!=Eigen::Success) break;
                    const Eigen::VectorXd delta=solver.solve(residual);
                    if(solver.info()!=Eigen::Success || !delta.allFinite()) break;
                    // At load reversal the current tangent may still be plastic.
                    // Backtracking prevents that soft tangent from overshooting into
                    // reverse yield, which otherwise causes a Newton two-cycle.
                    double alpha = 1.0;
                    bool accepted = false;
                    for (int search = 0; search < 16; ++search, alpha *= 0.5) {
                        check();
                        auto candidate = u;
                        for (size_t i = 0; i < count; ++i)
                            if (dof[i] >= 0) candidate[i] += alpha * delta.segment<3>(dof[i]);
                        Eigen::VectorXd trial_residual = Eigen::VectorXd::Zero(free_count);
                        for (size_t i = 0; i < count; ++i)
                            if (dof[i] >= 0) trial_residual.segment<3>(dof[i]) = external[i];
                        for (const Bond &bond : bonds) {
                            if (bond.failed || !attached[bond.a] || !attached[bond.b]) continue;
                            const Vec3d force = constitutive(bond, candidate[bond.b] - candidate[bond.a], s).force;
                            if (dof[bond.a] >= 0) trial_residual.segment<3>(dof[bond.a]) += force;
                            if (dof[bond.b] >= 0) trial_residual.segment<3>(dof[bond.b]) -= force;
                        }
                        if (trial_residual.norm() < residual.norm() * (1.0 - 1e-4 * alpha)) {
                            u = std::move(candidate);
                            accepted = true;
                            break;
                        }
                    }
                    if (!accepted) break;
                }
                if(!converged) {
                    out.status=AnalysisStatus::NumericalFailure;
                    out.message="Nonlinear equilibrium did not converge at increment " + std::to_string(step) +
                        "; residual " + std::to_string(last_residual) + " N. Reduce loading or refine the grid/increments.";
                    out.frames.clear();return out;
                }
                bool changed=false;
                for(size_t j=0;j<bonds.size();++j) {
                    Bond &b=bonds[j];
                    if(b.failed || !attached[b.a] || !attached[b.b]) continue;
                    const Trial t=constitutive(b,u[b.b]-u[b.a],s);
                    trials[j]=t;
                    // Equilibrium is accepted before any damage cascade. Preserve
                    // plastic history even if subsequent redistribution unloads this bond.
                    b.plastic=t.plastic; b.accumulated=t.accumulated;
                    b.plastic_shear=t.plastic_shear; b.shear_accumulated=t.shear_accumulated;
                    if (std::hypot(t.accumulated,t.shear_accumulated)>1e-12)
                        out.first_yield_time_s=std::min(out.first_yield_time_s,time);
                    if(s.fracture && (std::abs(t.normal)>b.ultimate ||
                        (!s.plasticity && t.shear>b.shear) ||
                        std::hypot(t.accumulated,t.shear_accumulated)>s.failure_plastic_strain)) {
                        b.failed=true;changed=true;
                        out.first_failure_time_s=std::min(out.first_failure_time_s,time);
                    }
                }
                if(!changed) {equilibrated=true;break;}
            }
            if(!equilibrated) return fail("Damage iteration limit exceeded.");
            TransientFrame frame;
            frame.time_s=time;frame.load_fraction=fraction;frame.cells.resize(count);
            std::vector<std::array<Matrix3d,3>> stress(count);
            std::vector<std::array<double,3>> weights(count, {0.0,0.0,0.0});
            for (auto &directions : stress) for (auto &tensor : directions) tensor.setZero();
            for(size_t j=0;j<bonds.size();++j) {
                Bond &b=bonds[j];
                frame.maximum_plastic_strain=std::max(frame.maximum_plastic_strain,
                    std::hypot(b.accumulated,b.shear_accumulated));
                if(b.failed) {++frame.failed_bonds;continue;}
                if(!attached[b.a] || !attached[b.b]) continue;
                const Trial &t=trials[j];
                b.plastic=t.plastic;b.accumulated=t.accumulated;b.plastic_shear=t.plastic_shear;b.shear_accumulated=t.shear_accumulated;
                const double plastic=std::hypot(b.accumulated,b.shear_accumulated);
                if(plastic>1e-12) out.first_yield_time_s=std::min(out.first_yield_time_s,time);
                frame.maximum_plastic_strain=std::max(frame.maximum_plastic_strain,plastic);
                const Vec3d traction=t.force/b.area;
                const Vec3d shear_traction = traction - traction.dot(b.n) * b.n;
                const Matrix3d tensor = traction.dot(b.n) * (b.n * b.n.transpose()) +
                    shear_traction * b.n.transpose() + b.n * shear_traction.transpose();
                Eigen::Index direction = 0;
                (basis.transpose()*b.n).cwiseAbs().maxCoeff(&direction);
                for(size_t i:{b.a,b.b}) {
                    stress[i][size_t(direction)]+=tensor*b.area;
                    weights[i][size_t(direction)]+=b.area;
                }
            }
            for(size_t i=0;i<count;++i) {
                if(!attached[i]) ++frame.detached_cells;
                VertexResult &v=frame.cells[i];v.position_mm=raw(cells[i].p);v.displacement_m=u[i];
                // Average opposing faces within each direction, then sum the
                // directional tensors. Averaging all six faces dilutes axial stress.
                Matrix3d tensor = Matrix3d::Zero();
                for (size_t direction=0; direction<3; ++direction)
                    tensor += stress[i][direction]/std::max(weights[i][direction],1e-30);
                v.normal_stress_pa=tensor.diagonal();v.shear_stress_pa=Vec3d(tensor(0,1),tensor(0,2),tensor(1,2));
                const Matrix3d deviator=tensor-Matrix3d::Identity()*tensor.trace()/3.0;
                v.von_mises_pa=std::sqrt(1.5*deviator.squaredNorm());
                Eigen::SelfAdjointEigenSolver<Matrix3d> eigen(tensor,Eigen::EigenvaluesOnly);
                v.maximum_shear_pa=0.5*(eigen.eigenvalues().maxCoeff()-eigen.eigenvalues().minCoeff());
            }
            for(size_t j=0;j<forces.size();++j) {
                const auto &force=forces[j];
                double applied=0.0,displacement=0.0;
                for(size_t i:force.cells) {
                    if(attached[i]) applied+=fraction*force.vector.norm()/force.cells.size();
                    displacement=std::max(displacement,u[i].norm()*1000.0);
                }
                frame.applied_forces_n.push_back(applied);frame.probe_displacements_mm.push_back(displacement);
            }
            if(setup.gravity.enabled) {
                double applied=0.0,displacement=0.0;
                for(size_t i=0;i<count;++i) if(!fixed[i]) {
                    if(attached[i]) applied+=fraction*setup.gravity.acceleration_m_s2.norm()*material.density_kg_m3*cells[i].size.prod()*1e-9*cells[i].solid;
                    displacement=std::max(displacement,u[i].norm()*1000.0);
                }
                frame.applied_forces_n.push_back(applied);frame.probe_displacements_mm.push_back(displacement);
            }
            out.frames.push_back(std::move(frame));
            if(progress) progress(int(100*step/s.increments));
        }
        out.status=AnalysisStatus::Success;
        out.message="Layer-resolved lattice estimate; cell-centered geometry and homogenized infill. Detached fragments retain their last position; free flight is not simulated.";
    } catch(const Cancelled &) {out.status=AnalysisStatus::Cancelled;out.message="Animation analysis cancelled.";out.frames.clear();}
      catch(const std::exception &e) {out.status=AnalysisStatus::NumericalFailure;out.message=e.what();out.frames.clear();}
    return out;
}

Result transient_display_frame(const TransientResult &history,size_t index)
{
    Result result;
    if(!history.succeeded() || index>=history.frames.size()) return result;
    result.status=AnalysisStatus::Success;result.geometry_scale=history.geometry_scale;
    const auto &frame=history.frames[index];
    result.vertices.reserve(history.display_cells.size());
    for(size_t i=0;i<history.display_cells.size();++i) {
        VertexResult vertex=frame.cells[history.display_cells[i]];
        vertex.position_mm=history.display_mesh.vertices[i].cast<double>();
        result.maximum_displacement_m=std::max(result.maximum_displacement_m,vertex.displacement_m.norm());
        result.maximum_von_mises_pa=std::max(result.maximum_von_mises_pa,vertex.von_mises_pa);
        result.vertices.push_back(vertex);
    }
    return result;
}
}
