#include "StrengthAnalysis.hpp"

#include "nlohmann/json.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <string_view>

namespace Slic3r::StrengthAnalysis {

namespace {

constexpr double MM_TO_M = 1e-3;
constexpr double MM3_TO_M3 = 1e-9;
constexpr double NUMERIC_EPSILON = 1e-12;

struct PatternFactors {
    double stiffness;
    double strength;
    double directional;
    double time;
};

PatternFactors pattern_factors(InfillPattern pattern)
{
    // Offline, normalized engineering-estimate factors. These are deliberately conservative and
    // are labelled as model estimates in all returned comparison rows.
    switch (pattern) {
    case InfillPattern::Rectilinear: return {0.82, 0.82, 1.12, 0.90};
    case InfillPattern::Grid:        return {0.94, 0.91, 0.96, 1.04};
    case InfillPattern::Triangles:   return {1.05, 1.02, 1.00, 1.17};
    case InfillPattern::Honeycomb:   return {0.98, 0.96, 1.00, 1.20};
    case InfillPattern::Cubic:       return {1.03, 1.00, 1.02, 1.08};
    case InfillPattern::Gyroid:      return {1.00, 1.00, 1.00, 1.00};
    }
    return {1.0, 1.0, 1.0, 1.0};
}

double clamp_density(double density) { return std::clamp(density, 0.01, 1.0); }

double effective_solid_fraction(double density)
{
    // A conservative shell allowance prevents a sparse printed part from being treated as a bare
    // infill lattice. It is an estimate until actual sliced extrusion volume is available.
    return std::clamp(0.18 + 0.82 * clamp_density(density), 0.0, 1.0);
}

bool finite_positive(double value) { return std::isfinite(value) && value > 0.0; }

Vec3d normalized_or_zero(const Vec3d &v)
{
    const double norm = v.norm();
    if (norm > NUMERIC_EPSILON && std::isfinite(norm))
        return v / norm;
    return Vec3d::Zero();
}

double anisotropic_mix(double xy, double z, const Vec3d &direction, const Vec3d &layer_axis)
{
    const Vec3d n = normalized_or_zero(direction);
    const Vec3d a = normalized_or_zero(layer_axis);
    const double z_weight = std::clamp(std::pow(std::abs(n.dot(a)), 2.0), 0.0, 1.0);
    return xy * (1.0 - z_weight) + z * z_weight;
}

std::vector<size_t> region_vertices(const indexed_triangle_set &mesh, const SphericalRegion &region, bool select_nearest = true)
{
    std::vector<size_t> selected;
    if (region.whole_model) {
        selected.resize(mesh.vertices.size());
        std::iota(selected.begin(), selected.end(), size_t(0));
        return selected;
    }

    double nearest_distance = std::numeric_limits<double>::infinity();
    size_t nearest = 0;
    for (size_t index = 0; index < mesh.vertices.size(); ++index) {
        const Vec3d position = mesh.vertices[index].cast<double>();
        const double distance = (position - region.center_mm).norm();
        if (distance <= region.radius_mm)
            selected.push_back(index);
        if (distance < nearest_distance) {
            nearest_distance = distance;
            nearest = index;
        }
    }
    if (selected.empty() && select_nearest && !mesh.vertices.empty())
        selected.push_back(nearest);
    return selected;
}

std::array<double, 3> vec_to_array(const Vec3d &v) { return {v.x(), v.y(), v.z()}; }

Vec3d array_to_vec(const nlohmann::json &j, const Vec3d &fallback)
{
    if (!j.is_array() || j.size() != 3)
        return fallback;
    return Vec3d(j[0].get<double>(), j[1].get<double>(), j[2].get<double>());
}

template<class Enum> Enum enum_from_string(const std::string &value, const std::map<std::string, Enum> &values, Enum fallback)
{
    auto found = values.find(value);
    return found == values.end() ? fallback : found->second;
}

nlohmann::json region_json(const SphericalRegion &region)
{
    return {{"center_mm", vec_to_array(region.center_mm)}, {"radius_mm", region.radius_mm}, {"whole_model", region.whole_model}};
}

SphericalRegion region_from_json(const nlohmann::json &j)
{
    SphericalRegion region;
    if (!j.is_object())
        return region;
    if (j.contains("center_mm"))
        region.center_mm = array_to_vec(j["center_mm"], region.center_mm);
    region.radius_mm = j.value("radius_mm", region.radius_mm);
    region.whole_model = j.value("whole_model", region.whole_model);
    return region;
}

double support_score(const indexed_triangle_set &mesh, const Vec3d &layer_axis)
{
    const Vec3d up = normalized_or_zero(layer_axis);
    if (up.isZero(NUMERIC_EPSILON) || mesh.indices.empty())
        return 1.0;
    double minimum_height = std::numeric_limits<double>::infinity();
    double maximum_height = -std::numeric_limits<double>::infinity();
    for (const Vec3f &vertex : mesh.vertices) {
        const double height = vertex.cast<double>().dot(up);
        minimum_height = std::min(minimum_height, height);
        maximum_height = std::max(maximum_height, height);
    }
    const double bed_tolerance = std::max(1e-6, (maximum_height - minimum_height) * 1e-4);
    double downward_area = 0.0;
    double total_area = 0.0;
    for (const Vec3i32 &face : mesh.indices) {
        const Vec3d a = mesh.vertices[face.x()].cast<double>();
        const Vec3d b = mesh.vertices[face.y()].cast<double>();
        const Vec3d c = mesh.vertices[face.z()].cast<double>();
        const Vec3d cross = (b - a).cross(c - a);
        const double area = 0.5 * cross.norm();
        if (area <= NUMERIC_EPSILON)
            continue;
        total_area += area;
        const Vec3d normal = cross.normalized();
        const double centroid_height = ((a + b + c) / 3.0).dot(up);
        if (normal.dot(up) < -0.5 && centroid_height > minimum_height + bed_tolerance)
            downward_area += area * (-normal.dot(up));
    }
    return total_area > NUMERIC_EPSILON ? downward_area / total_area : 1.0;
}

double required_safety_factor(const Setup &setup)
{
    double required = setup.criteria.minimum_safety_factor;
    for (const Load &load : setup.loads) {
        if (load.active && load.type != LoadType::Fixed)
            required = std::max(required, load.target_safety_factor);
    }
    return required;
}

std::pair<double, double> strength_for_basis(const Setup &setup, StrengthBasis basis, double layer_fraction)
{
    const Material raw = setup.material;
    const Material calibrated = setup.material.calibrated();
    const Material &material = basis == StrengthBasis::CalibratedYield ? calibrated : raw;
    const double normal_xy = basis == StrengthBasis::Ultimate ? material.ultimate_strength_xy_pa : material.yield_strength_xy_pa;
    const double normal_z = basis == StrengthBasis::Ultimate ? material.ultimate_strength_z_pa : material.yield_strength_z_pa;
    const double normal = normal_xy * (1.0 - layer_fraction) + normal_z * layer_fraction;
    const double shear = material.shear_strength_xy_pa * (1.0 - layer_fraction) + material.shear_strength_xz_pa * layer_fraction;
    return {normal, shear};
}

std::pair<double, double> governing_strength(const Setup &setup, double layer_fraction)
{
    std::pair<double, double> allowable{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    bool found = false;
    for (const Load &load : setup.loads) {
        if (!load.active || load.type == LoadType::Fixed)
            continue;
        const auto candidate = strength_for_basis(setup, load.strength_basis, layer_fraction);
        allowable.first = std::min(allowable.first, candidate.first);
        allowable.second = std::min(allowable.second, candidate.second);
        found = true;
    }
    if (!found)
        allowable = strength_for_basis(setup, StrengthBasis::Yield, layer_fraction);
    return allowable;
}

double directional_strength(const Setup &setup, StrengthBasis basis, const Vec3d &direction, const Vec3d &layer_axis)
{
    const Vec3d n = normalized_or_zero(direction);
    const Vec3d a = normalized_or_zero(layer_axis);
    const double layer_fraction = std::clamp(std::pow(std::abs(n.dot(a)), 2.0), 0.0, 1.0);
    return strength_for_basis(setup, basis, layer_fraction).first;
}

} // namespace

bool SphericalRegion::contains(const Vec3d &position_mm) const
{
    return whole_model || (finite_positive(radius_mm) && (position_mm - center_mm).squaredNorm() <= radius_mm * radius_mm);
}

std::vector<std::string> Material::validate() const
{
    std::vector<std::string> errors;
    auto require = [&errors](double value, const char *name) {
        if (!finite_positive(value))
            errors.emplace_back(std::string(name) + " must be finite and greater than zero.");
    };
    if (key.empty()) errors.emplace_back("Material key is required.");
    if (name.empty()) errors.emplace_back("Material name is required.");
    require(density_kg_m3, "Density");
    require(elastic_modulus_xy_pa, "XY elastic modulus");
    require(elastic_modulus_z_pa, "Z elastic modulus");
    require(shear_modulus_xy_pa, "XY shear modulus");
    require(shear_modulus_xz_pa, "XZ shear modulus");
    require(yield_strength_xy_pa, "XY yield strength");
    require(yield_strength_z_pa, "Z yield strength");
    require(ultimate_strength_xy_pa, "XY ultimate strength");
    require(ultimate_strength_z_pa, "Z ultimate strength");
    require(shear_strength_xy_pa, "XY shear strength");
    require(shear_strength_xz_pa, "XZ shear strength");
    if (!std::isfinite(poisson_xy) || poisson_xy <= -1.0 || poisson_xy >= 0.5)
        errors.emplace_back("Poisson ratio must be finite and between -1 and 0.5.");
    require(calibration.modulus_xy_scale, "XY modulus calibration scale");
    require(calibration.modulus_z_scale, "Z modulus calibration scale");
    require(calibration.strength_xy_scale, "XY strength calibration scale");
    require(calibration.strength_z_scale, "Z strength calibration scale");
    require(calibration.shear_scale, "Shear calibration scale");
    if (ultimate_strength_xy_pa < yield_strength_xy_pa)
        errors.emplace_back("XY ultimate strength cannot be lower than XY yield strength.");
    if (ultimate_strength_z_pa < yield_strength_z_pa)
        errors.emplace_back("Z ultimate strength cannot be lower than Z yield strength.");
    return errors;
}

Material Material::calibrated() const
{
    Material output = *this;
    output.elastic_modulus_xy_pa *= calibration.modulus_xy_scale;
    output.elastic_modulus_z_pa *= calibration.modulus_z_scale;
    output.shear_modulus_xy_pa *= calibration.shear_scale;
    output.shear_modulus_xz_pa *= calibration.shear_scale;
    output.yield_strength_xy_pa *= calibration.strength_xy_scale;
    output.ultimate_strength_xy_pa *= calibration.strength_xy_scale;
    output.yield_strength_z_pa *= calibration.strength_z_scale;
    output.ultimate_strength_z_pa *= calibration.strength_z_scale;
    output.shear_strength_xy_pa *= calibration.shear_scale;
    output.shear_strength_xz_pa *= calibration.shear_scale;
    output.calibration.modulus_xy_scale = 1.0;
    output.calibration.modulus_z_scale = 1.0;
    output.calibration.strength_xy_scale = 1.0;
    output.calibration.strength_z_scale = 1.0;
    output.calibration.shear_scale = 1.0;
    return output;
}

const std::vector<Material> &builtin_materials()
{
    static const std::string provenance =
        "Generic starting estimate compiled from common filament manufacturer ranges; verify against the specific filament datasheet and printed test coupons.";
    static const std::vector<Material> materials = [&] {
        std::vector<Material> values;
        Material pla;
        pla.provenance = provenance;
        values.push_back(pla);

        Material petg = pla;
        petg.key = "petg_generic"; petg.name = "Generic PETG"; petg.density_kg_m3 = 1270.0;
        petg.elastic_modulus_xy_pa = 2.1e9; petg.elastic_modulus_z_pa = 1.25e9;
        petg.shear_modulus_xy_pa = 0.78e9; petg.shear_modulus_xz_pa = 0.46e9;
        petg.yield_strength_xy_pa = 43e6; petg.yield_strength_z_pa = 23e6;
        petg.ultimate_strength_xy_pa = 50e6; petg.ultimate_strength_z_pa = 29e6;
        petg.shear_strength_xy_pa = 27e6; petg.shear_strength_xz_pa = 16e6;
        values.push_back(petg);

        Material abs = pla;
        abs.key = "abs_generic"; abs.name = "Generic ABS"; abs.density_kg_m3 = 1040.0;
        abs.elastic_modulus_xy_pa = 1.8e9; abs.elastic_modulus_z_pa = 0.95e9;
        abs.shear_modulus_xy_pa = 0.67e9; abs.shear_modulus_xz_pa = 0.35e9;
        abs.yield_strength_xy_pa = 38e6; abs.yield_strength_z_pa = 19e6;
        abs.ultimate_strength_xy_pa = 44e6; abs.ultimate_strength_z_pa = 24e6;
        abs.shear_strength_xy_pa = 24e6; abs.shear_strength_xz_pa = 13e6;
        values.push_back(abs);

        Material pa = pla;
        pa.key = "pa_generic"; pa.name = "Generic Nylon (PA)"; pa.density_kg_m3 = 1140.0;
        pa.elastic_modulus_xy_pa = 1.6e9; pa.elastic_modulus_z_pa = 0.85e9;
        pa.shear_modulus_xy_pa = 0.59e9; pa.shear_modulus_xz_pa = 0.31e9;
        pa.yield_strength_xy_pa = 45e6; pa.yield_strength_z_pa = 22e6;
        pa.ultimate_strength_xy_pa = 58e6; pa.ultimate_strength_z_pa = 29e6;
        pa.shear_strength_xy_pa = 29e6; pa.shear_strength_xz_pa = 15e6;
        values.push_back(pa);

        Material pc = pla;
        pc.key = "pc_generic"; pc.name = "Generic Polycarbonate"; pc.density_kg_m3 = 1200.0;
        pc.elastic_modulus_xy_pa = 2.25e9; pc.elastic_modulus_z_pa = 1.25e9;
        pc.shear_modulus_xy_pa = 0.83e9; pc.shear_modulus_xz_pa = 0.46e9;
        pc.yield_strength_xy_pa = 55e6; pc.yield_strength_z_pa = 27e6;
        pc.ultimate_strength_xy_pa = 65e6; pc.ultimate_strength_z_pa = 34e6;
        pc.shear_strength_xy_pa = 35e6; pc.shear_strength_xz_pa = 19e6;
        values.push_back(pc);

        Material tpu = pla;
        tpu.key = "tpu_generic"; tpu.name = "Generic TPU 95A"; tpu.density_kg_m3 = 1210.0;
        tpu.elastic_modulus_xy_pa = 32e6; tpu.elastic_modulus_z_pa = 18e6;
        tpu.poisson_xy = 0.45; tpu.shear_modulus_xy_pa = 11e6; tpu.shear_modulus_xz_pa = 6e6;
        tpu.yield_strength_xy_pa = 8e6; tpu.yield_strength_z_pa = 5e6;
        tpu.ultimate_strength_xy_pa = 30e6; tpu.ultimate_strength_z_pa = 18e6;
        tpu.shear_strength_xy_pa = 10e6; tpu.shear_strength_xz_pa = 6e6;
        values.push_back(tpu);
        return values;
    }();
    return materials;
}

const Material *find_builtin_material(const std::string &key)
{
    const auto &materials = builtin_materials();
    auto found = std::find_if(materials.begin(), materials.end(), [&key](const Material &material) { return material.key == key; });
    return found == materials.end() ? nullptr : &*found;
}

std::string to_string(LoadType type)
{
    switch (type) {
    case LoadType::Fixed: return "fixed";
    case LoadType::LocalForce: return "local_force";
    case LoadType::DirectionalForce: return "directional_force";
    case LoadType::BearingForce: return "bearing_force";
    case LoadType::ImpactForce: return "impact_force";
    case LoadType::GlobalForce: return "global_force";
    }
    return "local_force";
}

std::string to_string(StrengthBasis basis)
{
    switch (basis) {
    case StrengthBasis::Yield: return "yield";
    case StrengthBasis::Ultimate: return "ultimate";
    case StrengthBasis::CalibratedYield: return "calibrated_yield";
    }
    return "yield";
}

std::string to_string(InfillPattern pattern)
{
    switch (pattern) {
    case InfillPattern::Rectilinear: return "rectilinear";
    case InfillPattern::Grid: return "grid";
    case InfillPattern::Triangles: return "triangles";
    case InfillPattern::Honeycomb: return "honeycomb";
    case InfillPattern::Cubic: return "cubic";
    case InfillPattern::Gyroid: return "gyroid";
    }
    return "gyroid";
}

std::string to_string(AnalysisStatus status)
{
    switch (status) {
    case AnalysisStatus::NotRun: return "not_run";
    case AnalysisStatus::Success: return "success";
    case AnalysisStatus::InvalidInput: return "invalid_input";
    case AnalysisStatus::Singular: return "singular";
    case AnalysisStatus::Cancelled: return "cancelled";
    case AnalysisStatus::NumericalFailure: return "numerical_failure";
    }
    return "not_run";
}

std::vector<std::string> validate(const indexed_triangle_set &mesh, const Setup &setup)
{
    std::vector<std::string> errors = setup.material.validate();
    if (mesh.vertices.empty() || mesh.indices.empty())
        errors.emplace_back("A non-empty triangle mesh is required.");
    if (mesh.vertices.size() > setup.solver.maximum_vertices)
        errors.emplace_back("Mesh exceeds the configured analysis vertex limit.");
    if (normalized_or_zero(setup.print_layer_axis).isZero(NUMERIC_EPSILON))
        errors.emplace_back("Print layer axis must be a non-zero vector.");
    if (!(setup.infill.background_density > 0.0 && setup.infill.background_density <= 1.0))
        errors.emplace_back("Background infill density must be in (0, 1].");
    if (!(setup.infill.dense_density > 0.0 && setup.infill.dense_density <= 1.0))
        errors.emplace_back("Dense infill density must be in (0, 1].");
    if (setup.infill.dense_density < setup.infill.background_density)
        errors.emplace_back("Dense infill density cannot be lower than the background density.");
    if (!std::isfinite(setup.infill.dense_stress_threshold) || setup.infill.dense_stress_threshold <= 0.0 ||
        setup.infill.dense_stress_threshold > 1.0)
        errors.emplace_back("Dense stress threshold must be in (0, 1].");
    if (!finite_positive(setup.criteria.minimum_safety_factor))
        errors.emplace_back("Minimum safety factor must be finite and greater than zero.");
    if (!std::isfinite(setup.criteria.maximum_displacement_mm) || setup.criteria.maximum_displacement_mm < 0.0)
        errors.emplace_back("Maximum displacement must be finite and non-negative.");
    const std::array<double, 4> weights{setup.criteria.mass_weight, setup.criteria.stiffness_weight,
        setup.criteria.support_weight, setup.criteria.print_time_weight};
    if (std::any_of(weights.begin(), weights.end(), [](double value) { return !std::isfinite(value) || value < 0.0; }))
        errors.emplace_back("Optimization weights must be finite and non-negative.");
    if (std::all_of(weights.begin(), weights.end(), [](double value) { return value == 0.0; }))
        errors.emplace_back("At least one optimization weight must be greater than zero.");
    if (setup.solver.maximum_vertices < 4)
        errors.emplace_back("Analysis vertex limit must be at least four.");
    if (!std::isfinite(setup.solver.transverse_stiffness_ratio) || setup.solver.transverse_stiffness_ratio <= 0.0 ||
        setup.solver.transverse_stiffness_ratio > 1.0)
        errors.emplace_back("Transverse stiffness ratio must be in (0, 1].");
    if (!std::isfinite(setup.solver.regularization_ratio) || setup.solver.regularization_ratio <= 0.0 ||
        setup.solver.regularization_ratio > 1.0)
        errors.emplace_back("Regularization ratio must be in (0, 1].");

    for (const SphericalRegion &region : setup.preserve_regions) {
        if (!region.whole_model && !finite_positive(region.radius_mm))
            errors.emplace_back("Preserve-region radius must be finite and greater than zero.");
        if (!region.center_mm.allFinite())
            errors.emplace_back("Preserve-region center must be finite.");
    }

    bool has_fixed = false;
    bool has_force = setup.gravity.enabled;
    std::set<size_t> constrained_vertices;
    for (const Load &load : setup.loads) {
        if (!load.active)
            continue;
        if (!load.region.center_mm.allFinite())
            errors.emplace_back("Load-region center must be finite.");
        if (load.type == LoadType::Fixed) {
            has_fixed = true;
            if (load.region.whole_model) {
                errors.emplace_back("A fixed constraint cannot cover the whole model.");
            } else if (!finite_positive(load.region.radius_mm)) {
                errors.emplace_back("Fixed-region radius must be finite and greater than zero.");
            } else {
                const std::vector<size_t> selected = region_vertices(mesh, load.region, false);
                constrained_vertices.insert(selected.begin(), selected.end());
            }
        } else {
            has_force = true;
            if (!load.region.whole_model && !finite_positive(load.region.radius_mm))
                errors.emplace_back("Load-region radius must be finite and greater than zero.");
            if (!finite_positive(load.magnitude_n))
                errors.emplace_back("Active load magnitude must be finite and greater than zero.");
            if (normalized_or_zero(load.direction).isZero(NUMERIC_EPSILON))
                errors.emplace_back("Active load direction must be non-zero.");
            if (load.type == LoadType::ImpactForce && (!std::isfinite(load.impact_factor) || load.impact_factor < 1.0))
                errors.emplace_back("Equivalent-static impact factor must be at least one.");
            if (!finite_positive(load.target_safety_factor))
                errors.emplace_back("Load target safety factor must be greater than zero.");
        }
    }
    if (!has_fixed)
        errors.emplace_back("At least one active fixed constraint is required.");
    else if (constrained_vertices.size() < 3)
        errors.emplace_back("Fixed regions must contain at least three mesh vertices.");
    else {
        const Vec3d origin = mesh.vertices[*constrained_vertices.begin()].cast<double>();
        bool non_collinear = false;
        for (auto first = std::next(constrained_vertices.begin()); first != constrained_vertices.end() && !non_collinear; ++first) {
            const Vec3d first_axis = mesh.vertices[*first].cast<double>() - origin;
            for (auto second = std::next(first); second != constrained_vertices.end(); ++second) {
                const Vec3d second_axis = mesh.vertices[*second].cast<double>() - origin;
                if (first_axis.cross(second_axis).squaredNorm() > NUMERIC_EPSILON) {
                    non_collinear = true;
                    break;
                }
            }
        }
        if (!non_collinear)
            errors.emplace_back("Fixed-region vertices must not all be collinear.");
    }
    if (!has_force)
        errors.emplace_back("At least one active force or gravity load is required.");
    if (setup.gravity.enabled && (!setup.gravity.acceleration_m_s2.allFinite() ||
                                  setup.gravity.acceleration_m_s2.isZero(NUMERIC_EPSILON)))
        errors.emplace_back("Gravity acceleration must be finite and non-zero.");
    return errors;
}

std::vector<InfillComparison> compare_infill_patterns(double solid_volume_m3, const Setup &setup, double reference_safety_factor)
{
    std::vector<InfillPattern> patterns = setup.criteria.candidate_patterns;
    if (patterns.empty())
        patterns = {InfillPattern::Rectilinear, InfillPattern::Grid, InfillPattern::Triangles,
                    InfillPattern::Honeycomb, InfillPattern::Cubic, InfillPattern::Gyroid};
    std::sort(patterns.begin(), patterns.end(), [](InfillPattern lhs, InfillPattern rhs) { return int(lhs) < int(rhs); });
    patterns.erase(std::unique(patterns.begin(), patterns.end()), patterns.end());
    std::vector<InfillComparison> output;
    const double density = clamp_density(setup.infill.background_density);
    const double mass = std::max(0.0, solid_volume_m3) * effective_solid_fraction(density) * setup.material.density_kg_m3;
    for (InfillPattern pattern : patterns) {
        const PatternFactors factors = pattern_factors(pattern);
        output.push_back({pattern, density, factors.stiffness, factors.strength * factors.directional, mass,
                          std::max(0.0, reference_safety_factor) * factors.strength,
                          "Bundled normalized engineering-estimate model; calibrate with printed coupons."});
    }
    return output;
}

std::vector<MassStrengthPoint> estimate_mass_strength_curve(double solid_volume_m3, const Setup &setup, double reference_safety_factor)
{
    std::vector<double> targets{1.0, 1.25, 1.5, 2.0, 2.5, 3.0};
    if (setup.criteria.minimum_safety_factor > 0.0)
        targets.push_back(setup.criteria.minimum_safety_factor);
    for (const Load &load : setup.loads) {
        if (load.active && load.type != LoadType::Fixed && load.target_safety_factor > 0.0)
            targets.push_back(load.target_safety_factor);
    }
    std::sort(targets.begin(), targets.end());
    targets.erase(std::unique(targets.begin(), targets.end()), targets.end());

    std::vector<double> densities = setup.criteria.candidate_densities;
    densities.push_back(setup.infill.background_density);
    densities.push_back(1.0);
    std::sort(densities.begin(), densities.end());
    densities.erase(std::unique(densities.begin(), densities.end()), densities.end());
    std::vector<InfillPattern> patterns = setup.criteria.candidate_patterns;
    if (patterns.empty()) patterns.push_back(setup.infill.background_pattern);

    const double baseline_fraction = effective_solid_fraction(setup.infill.background_density);
    std::vector<MassStrengthPoint> output;
    double previous_mass = 0.0;
    for (double target : targets) {
        MassStrengthPoint best;
        best.target_safety_factor = target;
        best.estimated_mass_kg = std::numeric_limits<double>::infinity();
        for (double density : densities) {
            if (!(density > 0.0 && density <= 1.0)) continue;
            for (InfillPattern pattern : patterns) {
                const PatternFactors factors = pattern_factors(pattern);
                const double predicted = std::max(0.0, reference_safety_factor) * factors.strength *
                                         effective_solid_fraction(density) / baseline_fraction;
                const double mass = std::max(0.0, solid_volume_m3) * effective_solid_fraction(density) * setup.material.density_kg_m3;
                if (predicted + 1e-9 >= target && mass < best.estimated_mass_kg) {
                    best.feasible = true;
                    best.estimated_mass_kg = mass;
                    best.density = density;
                    best.pattern = pattern;
                    best.predicted_safety_factor = predicted;
                }
            }
        }
        if (!best.feasible) {
            best.estimated_mass_kg = std::max(previous_mass, std::max(0.0, solid_volume_m3) * setup.material.density_kg_m3);
            best.density = 1.0;
            best.pattern = InfillPattern::Triangles;
            best.predicted_safety_factor = std::max(0.0, reference_safety_factor) * pattern_factors(best.pattern).strength / baseline_fraction;
        }
        best.estimated_mass_kg = std::max(best.estimated_mass_kg, previous_mass);
        previous_mass = best.estimated_mass_kg;
        output.push_back(best);
    }
    return output;
}

std::vector<OrientationRecommendation> recommend_orientations(const indexed_triangle_set &mesh, const Setup &setup,
                                                               double reference_safety_factor)
{
    const std::array<Vec3d, 6> axes{Vec3d::UnitX(), -Vec3d::UnitX(), Vec3d::UnitY(), -Vec3d::UnitY(), Vec3d::UnitZ(), -Vec3d::UnitZ()};
    std::vector<OrientationRecommendation> output;
    for (const Vec3d &axis : axes) {
        double governing_ratio = std::numeric_limits<double>::infinity();
        bool found_load = false;
        for (const Load &load : setup.loads) {
            if (load.active && load.type != LoadType::Fixed) {
                const double current = directional_strength(setup, load.strength_basis, load.direction, setup.print_layer_axis);
                const double candidate = directional_strength(setup, load.strength_basis, load.direction, axis);
                governing_ratio = std::min(governing_ratio, candidate / std::max(current, NUMERIC_EPSILON));
                found_load = true;
            }
        }
        if (setup.gravity.enabled) {
            const double current = directional_strength(setup, StrengthBasis::Yield, setup.gravity.acceleration_m_s2,
                                                        setup.print_layer_axis);
            const double candidate = directional_strength(setup, StrengthBasis::Yield, setup.gravity.acceleration_m_s2, axis);
            governing_ratio = std::min(governing_ratio, candidate / std::max(current, NUMERIC_EPSILON));
            found_load = true;
        }
        if (!found_load)
            governing_ratio = 1.0;
        const double predicted = std::max(0.0, reference_safety_factor) * governing_ratio;
        const double support = support_score(mesh, axis);
        const double combined = setup.criteria.stiffness_weight * std::min(predicted, 10.0) -
                                setup.criteria.support_weight * support;
        output.push_back({axis, predicted, support, combined});
    }
    std::stable_sort(output.begin(), output.end(), [](const auto &lhs, const auto &rhs) { return lhs.combined_score > rhs.combined_score; });
    return output;
}

std::vector<PrintSettingsCandidate> recommend_print_settings(double solid_volume_m3, const Setup &setup,
                                                              double reference_safety_factor,
                                                              double reference_displacement_mm)
{
    std::vector<double> densities = setup.criteria.candidate_densities;
    if (densities.empty()) densities = {0.15, 0.25, 0.40, 0.60, 0.80};
    std::vector<InfillPattern> patterns = setup.criteria.candidate_patterns;
    if (patterns.empty()) patterns = {InfillPattern::Rectilinear, InfillPattern::Cubic, InfillPattern::Gyroid};
    const std::array<int, 3> walls{2, 3, 4};
    const std::array<double, 3> layers{0.12, 0.20, 0.28};
    const double baseline_fraction = effective_solid_fraction(setup.infill.background_density);
    std::vector<PrintSettingsCandidate> output;
    for (double density : densities) {
        if (!(density > 0.0 && density <= 1.0)) continue;
        for (InfillPattern pattern : patterns) {
            const PatternFactors factors = pattern_factors(pattern);
            for (int wall_count : walls) {
                for (double layer_height : layers) {
                    const double wall_factor = 1.0 + 0.12 * (wall_count - 2);
                    const double layer_factor = std::clamp(1.0 + (0.20 - layer_height) * 0.6, 0.85, 1.10);
                    const double density_factor = effective_solid_fraction(density) / baseline_fraction;
                    const double predicted = std::max(0.0, reference_safety_factor) * factors.strength * wall_factor * layer_factor *
                                             density_factor;
                    const double stiffness_multiplier = factors.stiffness * wall_factor * layer_factor * density_factor;
                    const double predicted_displacement = std::max(0.0, reference_displacement_mm) /
                                                          std::max(stiffness_multiplier, NUMERIC_EPSILON);
                    const double mass = std::max(0.0, solid_volume_m3) * setup.material.density_kg_m3 *
                                        std::min(1.0, effective_solid_fraction(density) + 0.025 * (wall_count - 2));
                    const double normalized_mass = mass / std::max(solid_volume_m3 * setup.material.density_kg_m3, NUMERIC_EPSILON);
                    const double print_time = factors.time * (0.20 / layer_height) * (0.7 + density) * (1.0 + 0.08 * (wall_count - 2));
                    const bool feasible = predicted + 1e-9 >= required_safety_factor(setup) &&
                        (setup.criteria.maximum_displacement_mm <= 0.0 ||
                         predicted_displacement <= setup.criteria.maximum_displacement_mm + 1e-9);
                    const double score = setup.criteria.mass_weight * normalized_mass +
                                         setup.criteria.print_time_weight * print_time -
                                         setup.criteria.stiffness_weight * std::min(stiffness_multiplier, 10.0) * 0.1;
                    output.push_back({wall_count, layer_height, density, pattern, predicted, predicted_displacement,
                                      mass, score, feasible});
                }
            }
        }
    }
    std::stable_sort(output.begin(), output.end(), [](const auto &lhs, const auto &rhs) {
        if (lhs.feasible != rhs.feasible) return lhs.feasible > rhs.feasible;
        return lhs.score < rhs.score;
    });
    if (output.size() > 24) output.resize(24);
    return output;
}

Result analyze(const indexed_triangle_set &mesh, const Setup &setup, const CancelPredicate &cancel)
{
    Result result;
    const std::vector<std::string> errors = validate(mesh, setup);
    if (!errors.empty()) {
        result.status = AnalysisStatus::InvalidInput;
        std::ostringstream message;
        for (size_t i = 0; i < errors.size(); ++i) {
            if (i) message << ' ';
            message << errors[i];
        }
        result.message = message.str();
        return result;
    }
    auto cancelled = [&cancel] { return cancel && cancel(); };
    if (cancelled()) {
        result.status = AnalysisStatus::Cancelled;
        result.message = "Analysis cancelled.";
        return result;
    }

    const Material material = setup.material.calibrated();
    const double solid_volume_m3 = std::abs(its_volume(mesh)) * MM3_TO_M3;
    const double solid_fraction = effective_solid_fraction(setup.infill.background_density);
    result.effective_volume_m3 = solid_volume_m3 * solid_fraction;
    result.estimated_mass_kg = result.effective_volume_m3 * material.density_kg_m3;
    if (!(solid_volume_m3 > NUMERIC_EPSILON)) {
        result.status = AnalysisStatus::InvalidInput;
        result.message = "Mesh must enclose a positive volume for mass and gravity analysis.";
        return result;
    }

    struct Edge { size_t a; size_t b; double length_m; double tributary_area_m2; };
    std::map<std::pair<int, int>, double> edge_face_area;
    for (size_t face_index = 0; face_index < mesh.indices.size(); ++face_index) {
        if ((face_index & 1023u) == 0u && cancelled()) {
            result.status = AnalysisStatus::Cancelled; result.message = "Analysis cancelled."; return result;
        }
        const Vec3i32 &face = mesh.indices[face_index];
        if (face.minCoeff() < 0 || face.maxCoeff() >= int(mesh.vertices.size())) {
            result.status = AnalysisStatus::InvalidInput;
            result.message = "Mesh contains an out-of-range triangle index.";
            return result;
        }
        const Vec3d a = mesh.vertices[face.x()].cast<double>();
        const Vec3d b = mesh.vertices[face.y()].cast<double>();
        const Vec3d c = mesh.vertices[face.z()].cast<double>();
        const double area_m2 = 0.5 * (b - a).cross(c - a).norm() * 1e-6;
        const std::array<int, 3> ids{face.x(), face.y(), face.z()};
        for (int e = 0; e < 3; ++e) {
            int i = ids[e], j = ids[(e + 1) % 3];
            if (i > j) std::swap(i, j);
            edge_face_area[{i, j}] += area_m2 / 3.0;
        }
    }

    std::vector<Edge> edges;
    edges.reserve(edge_face_area.size());
    double total_edge_length_m = 0.0;
    for (const auto &[indices, face_area] : edge_face_area) {
        const Vec3d delta_m = (mesh.vertices[indices.second] - mesh.vertices[indices.first]).cast<double>() * MM_TO_M;
        const double length_m = delta_m.norm();
        if (length_m <= NUMERIC_EPSILON) continue;
        total_edge_length_m += length_m;
        edges.push_back({size_t(indices.first), size_t(indices.second), length_m, face_area});
    }
    if (edges.empty() || total_edge_length_m <= NUMERIC_EPSILON) {
        result.status = AnalysisStatus::InvalidInput;
        result.message = "Mesh does not contain usable edges.";
        return result;
    }
    const double common_area_m2 = result.effective_volume_m3 / total_edge_length_m;
    for (Edge &edge : edges)
        edge.tributary_area_m2 = std::max(common_area_m2, edge.tributary_area_m2 * solid_fraction * 0.05);

    const size_t vertex_count = mesh.vertices.size();
    const size_t dof_count = vertex_count * 3;
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(edges.size() * 36 + dof_count);
    double maximum_edge_stiffness = 0.0;
    const PatternFactors infill_factor = pattern_factors(setup.infill.background_pattern);
    for (size_t edge_index = 0; edge_index < edges.size(); ++edge_index) {
        if ((edge_index & 2047u) == 0u && cancelled()) {
            result.status = AnalysisStatus::Cancelled; result.message = "Analysis cancelled."; return result;
        }
        const Edge &edge = edges[edge_index];
        const Vec3d delta = (mesh.vertices[edge.b] - mesh.vertices[edge.a]).cast<double>() * MM_TO_M;
        const Vec3d direction = delta / edge.length_m;
        const double elastic_modulus = anisotropic_mix(material.elastic_modulus_xy_pa, material.elastic_modulus_z_pa,
                                                        direction, setup.print_layer_axis) * infill_factor.stiffness;
        const double shear_modulus = anisotropic_mix(material.shear_modulus_xy_pa, material.shear_modulus_xz_pa,
                                                      direction, setup.print_layer_axis);
        const double axial = elastic_modulus * edge.tributary_area_m2 / edge.length_m;
        const double transverse = shear_modulus * edge.tributary_area_m2 / edge.length_m * setup.solver.transverse_stiffness_ratio;
        maximum_edge_stiffness = std::max(maximum_edge_stiffness, axial);
        const Eigen::Matrix3d nn = direction * direction.transpose();
        const Eigen::Matrix3d block = axial * nn + transverse * (Eigen::Matrix3d::Identity() - nn);
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                const double value = block(row, col);
                triplets.emplace_back(3 * edge.a + row, 3 * edge.a + col, value);
                triplets.emplace_back(3 * edge.b + row, 3 * edge.b + col, value);
                triplets.emplace_back(3 * edge.a + row, 3 * edge.b + col, -value);
                triplets.emplace_back(3 * edge.b + row, 3 * edge.a + col, -value);
            }
        }
    }

    std::set<size_t> fixed_vertices;
    for (const Load &load : setup.loads) {
        if (!load.active || load.type != LoadType::Fixed) continue;
        const std::vector<size_t> selected = region_vertices(mesh, load.region);
        fixed_vertices.insert(selected.begin(), selected.end());
    }
    const double reference_stiffness = std::max(maximum_edge_stiffness, 1.0);
    const double regularization = reference_stiffness * setup.solver.regularization_ratio;
    const double fixed_penalty = reference_stiffness * 1e10;
    for (size_t dof = 0; dof < dof_count; ++dof)
        triplets.emplace_back(dof, dof, regularization);
    for (size_t vertex : fixed_vertices)
        for (int axis = 0; axis < 3; ++axis)
            triplets.emplace_back(3 * vertex + axis, 3 * vertex + axis, fixed_penalty);

    Eigen::SparseMatrix<double> stiffness{Eigen::Index(dof_count), Eigen::Index(dof_count)};
    stiffness.setFromTriplets(triplets.begin(), triplets.end());
    Eigen::VectorXd force = Eigen::VectorXd::Zero(Eigen::Index(dof_count));
    for (const Load &load : setup.loads) {
        if (!load.active || load.type == LoadType::Fixed) continue;
        const Vec3d direction = normalized_or_zero(load.direction);
        const double scale = load.type == LoadType::ImpactForce ? load.impact_factor : 1.0;
        const Vec3d resultant = direction * load.magnitude_n * scale;
        result.requested_resultant_force_n += resultant;
        SphericalRegion region = load.region;
        if (load.type == LoadType::GlobalForce) region.whole_model = true;
        std::vector<size_t> selected = region_vertices(mesh, region);
        selected.erase(std::remove_if(selected.begin(), selected.end(), [&fixed_vertices](size_t index) {
            return fixed_vertices.count(index) != 0;
        }), selected.end());
        if (selected.empty()) selected = region_vertices(mesh, region);
        const Vec3d per_vertex = resultant / double(selected.size());
        for (size_t vertex : selected)
            for (int axis = 0; axis < 3; ++axis)
                force[Eigen::Index(3 * vertex + axis)] += per_vertex[axis];
    }
    if (setup.gravity.enabled) {
        const Vec3d resultant = result.estimated_mass_kg * setup.gravity.acceleration_m_s2;
        result.requested_resultant_force_n += resultant;
        std::vector<size_t> selected;
        for (size_t vertex = 0; vertex < vertex_count; ++vertex)
            if (fixed_vertices.count(vertex) == 0) selected.push_back(vertex);
        if (selected.empty()) selected.resize(vertex_count), std::iota(selected.begin(), selected.end(), size_t(0));
        const Vec3d per_vertex = resultant / double(selected.size());
        for (size_t vertex : selected)
            for (int axis = 0; axis < 3; ++axis)
                force[Eigen::Index(3 * vertex + axis)] += per_vertex[axis];
    }
    if (cancelled()) {
        result.status = AnalysisStatus::Cancelled; result.message = "Analysis cancelled."; return result;
    }

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(stiffness);
    if (solver.info() != Eigen::Success) {
        result.status = AnalysisStatus::Singular;
        result.message = "The stiffness system is singular. Enlarge or add fixed regions.";
        return result;
    }
    const Eigen::VectorXd displacement = solver.solve(force);
    if (solver.info() != Eigen::Success || !displacement.allFinite()) {
        result.status = AnalysisStatus::NumericalFailure;
        result.message = "The structural estimate did not converge to finite displacements.";
        return result;
    }

    result.vertices.resize(vertex_count);
    std::vector<std::array<double, 6>> stress_sum(vertex_count, {0, 0, 0, 0, 0, 0});
    std::vector<double> stress_weight(vertex_count, 0.0);
    for (size_t vertex = 0; vertex < vertex_count; ++vertex) {
        VertexResult &value = result.vertices[vertex];
        value.position_mm = mesh.vertices[vertex].cast<double>();
        value.displacement_m = Vec3d(displacement[Eigen::Index(3 * vertex)], displacement[Eigen::Index(3 * vertex + 1)],
                                     displacement[Eigen::Index(3 * vertex + 2)]);
    }
    for (const Edge &edge : edges) {
        const Vec3d delta = (mesh.vertices[edge.b] - mesh.vertices[edge.a]).cast<double>() * MM_TO_M;
        const Vec3d direction = delta / edge.length_m;
        const Vec3d relative = result.vertices[edge.b].displacement_m - result.vertices[edge.a].displacement_m;
        const double strain = relative.dot(direction) / edge.length_m;
        const double modulus = anisotropic_mix(material.elastic_modulus_xy_pa, material.elastic_modulus_z_pa,
                                               direction, setup.print_layer_axis) * infill_factor.stiffness;
        const double axial_stress = modulus * strain;
        const Eigen::Matrix3d tensor = axial_stress * (direction * direction.transpose());
        const std::array<double, 6> components{tensor(0, 0), tensor(1, 1), tensor(2, 2), tensor(0, 1), tensor(0, 2), tensor(1, 2)};
        for (size_t vertex : {edge.a, edge.b}) {
            for (size_t component = 0; component < components.size(); ++component)
                stress_sum[vertex][component] += components[component] * edge.tributary_area_m2;
            stress_weight[vertex] += edge.tributary_area_m2;
        }
    }

    const Vec3d layer_axis = normalized_or_zero(setup.print_layer_axis);
    for (size_t vertex = 0; vertex < vertex_count; ++vertex) {
        VertexResult &value = result.vertices[vertex];
        const double weight = std::max(stress_weight[vertex], NUMERIC_EPSILON);
        for (double &component : stress_sum[vertex]) component /= weight;
        value.normal_stress_pa = Vec3d(stress_sum[vertex][0], stress_sum[vertex][1], stress_sum[vertex][2]);
        value.shear_stress_pa = Vec3d(stress_sum[vertex][3], stress_sum[vertex][4], stress_sum[vertex][5]);
        const double sx = value.normal_stress_pa.x(), sy = value.normal_stress_pa.y(), sz = value.normal_stress_pa.z();
        const double txy = value.shear_stress_pa.x(), txz = value.shear_stress_pa.y(), tyz = value.shear_stress_pa.z();
        value.von_mises_pa = std::sqrt(std::max(0.0, 0.5 * ((sx - sy) * (sx - sy) + (sy - sz) * (sy - sz) +
                                      (sz - sx) * (sz - sx)) + 3.0 * (txy * txy + txz * txz + tyz * tyz)));
        Eigen::Matrix3d tensor;
        tensor << sx, txy, txz, txy, sy, tyz, txz, tyz, sz;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(tensor, Eigen::EigenvaluesOnly);
        if (eigen_solver.info() == Eigen::Success)
            value.maximum_shear_pa = 0.5 * (eigen_solver.eigenvalues().maxCoeff() - eigen_solver.eigenvalues().minCoeff());
        const Vec3d traction = tensor * layer_axis;
        const double layer_fraction = value.von_mises_pa > NUMERIC_EPSILON ?
            std::clamp(traction.norm() / value.von_mises_pa, 0.0, 1.0) : 0.0;
        const auto [normal_allowable, shear_allowable] = governing_strength(setup, layer_fraction);
        const double normal_factor = value.von_mises_pa > NUMERIC_EPSILON ? normal_allowable / value.von_mises_pa : std::numeric_limits<double>::infinity();
        const double shear_factor = value.maximum_shear_pa > NUMERIC_EPSILON ? shear_allowable / value.maximum_shear_pa : std::numeric_limits<double>::infinity();
        value.safety_factor = std::min(normal_factor, shear_factor) * infill_factor.strength;

        const double displacement_norm = value.displacement_m.norm();
        if (displacement_norm > result.maximum_displacement_m) {
            result.maximum_displacement_m = displacement_norm;
            result.maximum_displacement_vertex = vertex;
        }
        if (value.von_mises_pa > result.maximum_von_mises_pa) {
            result.maximum_von_mises_pa = value.von_mises_pa;
            result.maximum_stress_vertex = vertex;
        }
        if (value.safety_factor < result.minimum_safety_factor) {
            result.minimum_safety_factor = value.safety_factor;
            result.minimum_safety_factor_vertex = vertex;
        }
    }

    result.status = AnalysisStatus::Success;
    result.message = "Linear-static printed-part engineering estimate completed.";
    result.governing_target_safety_factor = required_safety_factor(setup);
    result.safety_factor_target_met = result.minimum_safety_factor + 1e-9 >= result.governing_target_safety_factor;
    result.displacement_target_met = setup.criteria.maximum_displacement_mm <= 0.0 ||
        result.maximum_displacement_m * 1000.0 <= setup.criteria.maximum_displacement_mm + 1e-9;
    result.warnings.emplace_back("Engineering estimate only: the surface-lattice model is not certification-grade finite-element analysis.");
    result.warnings.emplace_back("The model excludes contact, plasticity, creep, fatigue, buckling, residual/thermal stress, and transient impact response.");
    result.warnings.emplace_back("Validate generic material values with the specific filament datasheet and printed test coupons.");
    if (setup.gravity.enabled)
        result.warnings.emplace_back("Gravity mass uses estimated shell-plus-infill volume until sliced extrusion volume is available.");
    if (!result.safety_factor_target_met)
        result.warnings.emplace_back("The governing safety-factor target is not met by the current estimate.");
    if (!result.displacement_target_met)
        result.warnings.emplace_back("The maximum-displacement criterion is not met by the current estimate.");

    if (!result.vertices.empty() && result.maximum_von_mises_pa > NUMERIC_EPSILON) {
        const BoundingBoxf3 bounds = bounding_box(mesh);
        const double threshold_stress = setup.infill.dense_stress_threshold * result.maximum_von_mises_pa;
        std::vector<size_t> high_stress_vertices;
        for (size_t vertex = 0; vertex < result.vertices.size(); ++vertex) {
            if (result.vertices[vertex].von_mises_pa + NUMERIC_EPSILON >= threshold_stress)
                high_stress_vertices.push_back(vertex);
        }
        if (high_stress_vertices.empty())
            high_stress_vertices.push_back(result.maximum_stress_vertex);

        result.dense_region.available = true;
        Vec3d center = Vec3d::Zero();
        double total_weight = 0.0;
        for (size_t vertex : high_stress_vertices) {
            const double weight = std::max(result.vertices[vertex].von_mises_pa, NUMERIC_EPSILON);
            center += result.vertices[vertex].position_mm * weight;
            total_weight += weight;
        }
        center /= total_weight;
        const double padding = std::max(1.0, bounds.size().cast<double>().norm() * 0.03);
        double radius = padding;
        for (size_t vertex : high_stress_vertices)
            radius = std::max(radius, (result.vertices[vertex].position_mm - center).norm() + padding);
        result.dense_region.region.center_mm = center;
        result.dense_region.region.radius_mm = radius;
        result.dense_region.stress_fraction = setup.infill.dense_stress_threshold;
        result.dense_region.recommended_density = setup.infill.dense_density;
        result.dense_region.recommended_pattern = setup.infill.dense_pattern;
        for (const SphericalRegion &preserve : setup.preserve_regions) {
            const bool overlap = preserve.whole_model ||
                (result.dense_region.region.center_mm - preserve.center_mm).norm() <=
                    result.dense_region.region.radius_mm + preserve.radius_mm;
            if (overlap) {
                result.dense_region.available = false;
                result.warnings.emplace_back("The threshold-based dense-region candidate overlaps a preserve region and was not recommended.");
                break;
            }
        }
    }
    const double finite_reference_sf = std::isfinite(result.minimum_safety_factor) ? result.minimum_safety_factor : 100.0;
    result.infill_comparisons = compare_infill_patterns(solid_volume_m3, setup, finite_reference_sf);
    result.mass_strength_curve = estimate_mass_strength_curve(solid_volume_m3, setup, finite_reference_sf);
    result.orientation_recommendations = recommend_orientations(mesh, setup, finite_reference_sf);
    result.print_settings_candidates = recommend_print_settings(solid_volume_m3, setup, finite_reference_sf,
                                                                 result.maximum_displacement_m * 1000.0);
    return result;
}

std::string serialize_setup(const Setup &setup)
{
    nlohmann::json j;
    j["schema_version"] = setup.schema_version;
    const Material &m = setup.material;
    j["material"] = {
        {"key", m.key}, {"name", m.name}, {"provenance", m.provenance}, {"density_kg_m3", m.density_kg_m3},
        {"elastic_modulus_xy_pa", m.elastic_modulus_xy_pa}, {"elastic_modulus_z_pa", m.elastic_modulus_z_pa},
        {"poisson_xy", m.poisson_xy}, {"shear_modulus_xy_pa", m.shear_modulus_xy_pa}, {"shear_modulus_xz_pa", m.shear_modulus_xz_pa},
        {"yield_strength_xy_pa", m.yield_strength_xy_pa}, {"yield_strength_z_pa", m.yield_strength_z_pa},
        {"ultimate_strength_xy_pa", m.ultimate_strength_xy_pa}, {"ultimate_strength_z_pa", m.ultimate_strength_z_pa},
        {"shear_strength_xy_pa", m.shear_strength_xy_pa}, {"shear_strength_xz_pa", m.shear_strength_xz_pa},
        {"calibration", {{"modulus_xy_scale", m.calibration.modulus_xy_scale}, {"modulus_z_scale", m.calibration.modulus_z_scale},
                         {"strength_xy_scale", m.calibration.strength_xy_scale}, {"strength_z_scale", m.calibration.strength_z_scale},
                         {"shear_scale", m.calibration.shear_scale}, {"source", m.calibration.source}}}
    };
    j["print_layer_axis"] = vec_to_array(setup.print_layer_axis);
    j["loads"] = nlohmann::json::array();
    for (const Load &load : setup.loads) {
        j["loads"].push_back({{"name", load.name}, {"type", to_string(load.type)}, {"active", load.active},
                              {"region", region_json(load.region)}, {"direction", vec_to_array(load.direction)},
                              {"magnitude_n", load.magnitude_n}, {"impact_factor", load.impact_factor},
                              {"target_safety_factor", load.target_safety_factor}, {"strength_basis", to_string(load.strength_basis)}});
    }
    j["gravity"] = {{"enabled", setup.gravity.enabled}, {"acceleration_m_s2", vec_to_array(setup.gravity.acceleration_m_s2)}};
    j["preserve_regions"] = nlohmann::json::array();
    for (const SphericalRegion &region : setup.preserve_regions) j["preserve_regions"].push_back(region_json(region));
    j["infill"] = {{"background_pattern", to_string(setup.infill.background_pattern)},
                   {"background_density", setup.infill.background_density}, {"dense_pattern", to_string(setup.infill.dense_pattern)},
                   {"dense_density", setup.infill.dense_density}, {"dense_stress_threshold", setup.infill.dense_stress_threshold}};
    nlohmann::json patterns = nlohmann::json::array();
    for (InfillPattern pattern : setup.criteria.candidate_patterns) patterns.push_back(to_string(pattern));
    j["criteria"] = {{"minimum_safety_factor", setup.criteria.minimum_safety_factor},
                     {"maximum_displacement_mm", setup.criteria.maximum_displacement_mm}, {"mass_weight", setup.criteria.mass_weight},
                     {"stiffness_weight", setup.criteria.stiffness_weight}, {"support_weight", setup.criteria.support_weight},
                     {"print_time_weight", setup.criteria.print_time_weight}, {"candidate_densities", setup.criteria.candidate_densities},
                     {"candidate_patterns", patterns}};
    j["solver"] = {{"maximum_vertices", setup.solver.maximum_vertices},
                   {"transverse_stiffness_ratio", setup.solver.transverse_stiffness_ratio},
                   {"regularization_ratio", setup.solver.regularization_ratio}};
    return j.dump();
}

bool deserialize_setup(const std::string &json_text, Setup &setup, std::string *error)
{
    try {
        const nlohmann::json j = nlohmann::json::parse(json_text);
        if (!j.is_object()) throw std::runtime_error("Root must be a JSON object.");
        const int version = j.value("schema_version", 0);
        if (version != 1) throw std::runtime_error("Unsupported Strength Analysis setup schema version.");
        Setup parsed;
        parsed.schema_version = version;
        if (j.contains("material")) {
            const auto &m = j["material"];
            parsed.material.key = m.value("key", parsed.material.key); parsed.material.name = m.value("name", parsed.material.name);
            parsed.material.provenance = m.value("provenance", parsed.material.provenance);
            parsed.material.density_kg_m3 = m.value("density_kg_m3", parsed.material.density_kg_m3);
            parsed.material.elastic_modulus_xy_pa = m.value("elastic_modulus_xy_pa", parsed.material.elastic_modulus_xy_pa);
            parsed.material.elastic_modulus_z_pa = m.value("elastic_modulus_z_pa", parsed.material.elastic_modulus_z_pa);
            parsed.material.poisson_xy = m.value("poisson_xy", parsed.material.poisson_xy);
            parsed.material.shear_modulus_xy_pa = m.value("shear_modulus_xy_pa", parsed.material.shear_modulus_xy_pa);
            parsed.material.shear_modulus_xz_pa = m.value("shear_modulus_xz_pa", parsed.material.shear_modulus_xz_pa);
            parsed.material.yield_strength_xy_pa = m.value("yield_strength_xy_pa", parsed.material.yield_strength_xy_pa);
            parsed.material.yield_strength_z_pa = m.value("yield_strength_z_pa", parsed.material.yield_strength_z_pa);
            parsed.material.ultimate_strength_xy_pa = m.value("ultimate_strength_xy_pa", parsed.material.ultimate_strength_xy_pa);
            parsed.material.ultimate_strength_z_pa = m.value("ultimate_strength_z_pa", parsed.material.ultimate_strength_z_pa);
            parsed.material.shear_strength_xy_pa = m.value("shear_strength_xy_pa", parsed.material.shear_strength_xy_pa);
            parsed.material.shear_strength_xz_pa = m.value("shear_strength_xz_pa", parsed.material.shear_strength_xz_pa);
            if (m.contains("calibration")) {
                const auto &c = m["calibration"];
                parsed.material.calibration.modulus_xy_scale = c.value("modulus_xy_scale", 1.0);
                parsed.material.calibration.modulus_z_scale = c.value("modulus_z_scale", 1.0);
                parsed.material.calibration.strength_xy_scale = c.value("strength_xy_scale", 1.0);
                parsed.material.calibration.strength_z_scale = c.value("strength_z_scale", 1.0);
                parsed.material.calibration.shear_scale = c.value("shear_scale", 1.0);
                parsed.material.calibration.source = c.value("source", parsed.material.calibration.source);
            }
        }
        if (j.contains("print_layer_axis")) parsed.print_layer_axis = array_to_vec(j["print_layer_axis"], parsed.print_layer_axis);
        static const std::map<std::string, LoadType> load_types{{"fixed", LoadType::Fixed}, {"local_force", LoadType::LocalForce},
            {"directional_force", LoadType::DirectionalForce}, {"bearing_force", LoadType::BearingForce},
            {"impact_force", LoadType::ImpactForce}, {"global_force", LoadType::GlobalForce}};
        static const std::map<std::string, StrengthBasis> bases{{"yield", StrengthBasis::Yield}, {"ultimate", StrengthBasis::Ultimate},
            {"calibrated_yield", StrengthBasis::CalibratedYield}};
        static const std::map<std::string, InfillPattern> patterns{{"rectilinear", InfillPattern::Rectilinear}, {"grid", InfillPattern::Grid},
            {"triangles", InfillPattern::Triangles}, {"honeycomb", InfillPattern::Honeycomb}, {"cubic", InfillPattern::Cubic},
            {"gyroid", InfillPattern::Gyroid}};
        parsed.loads.clear();
        if (j.contains("loads")) for (const auto &item : j["loads"]) {
            Load load;
            load.name = item.value("name", load.name); load.type = enum_from_string(item.value("type", "local_force"), load_types, load.type);
            load.active = item.value("active", load.active); if (item.contains("region")) load.region = region_from_json(item["region"]);
            if (item.contains("direction")) load.direction = array_to_vec(item["direction"], load.direction);
            load.magnitude_n = item.value("magnitude_n", load.magnitude_n); load.impact_factor = item.value("impact_factor", load.impact_factor);
            load.target_safety_factor = item.value("target_safety_factor", load.target_safety_factor);
            load.strength_basis = enum_from_string(item.value("strength_basis", "yield"), bases, load.strength_basis);
            parsed.loads.push_back(load);
        }
        if (j.contains("gravity")) {
            parsed.gravity.enabled = j["gravity"].value("enabled", parsed.gravity.enabled);
            if (j["gravity"].contains("acceleration_m_s2"))
                parsed.gravity.acceleration_m_s2 = array_to_vec(j["gravity"]["acceleration_m_s2"], parsed.gravity.acceleration_m_s2);
        }
        parsed.preserve_regions.clear();
        if (j.contains("preserve_regions")) for (const auto &item : j["preserve_regions"])
            parsed.preserve_regions.push_back(region_from_json(item));
        if (j.contains("infill")) {
            const auto &i = j["infill"];
            parsed.infill.background_pattern = enum_from_string(i.value("background_pattern", "gyroid"), patterns, parsed.infill.background_pattern);
            parsed.infill.background_density = i.value("background_density", parsed.infill.background_density);
            parsed.infill.dense_pattern = enum_from_string(i.value("dense_pattern", "gyroid"), patterns, parsed.infill.dense_pattern);
            parsed.infill.dense_density = i.value("dense_density", parsed.infill.dense_density);
            parsed.infill.dense_stress_threshold = i.value("dense_stress_threshold", parsed.infill.dense_stress_threshold);
        }
        if (j.contains("criteria")) {
            const auto &c = j["criteria"];
            parsed.criteria.minimum_safety_factor = c.value("minimum_safety_factor", parsed.criteria.minimum_safety_factor);
            parsed.criteria.maximum_displacement_mm = c.value("maximum_displacement_mm", parsed.criteria.maximum_displacement_mm);
            parsed.criteria.mass_weight = c.value("mass_weight", parsed.criteria.mass_weight);
            parsed.criteria.stiffness_weight = c.value("stiffness_weight", parsed.criteria.stiffness_weight);
            parsed.criteria.support_weight = c.value("support_weight", parsed.criteria.support_weight);
            parsed.criteria.print_time_weight = c.value("print_time_weight", parsed.criteria.print_time_weight);
            if (c.contains("candidate_densities")) parsed.criteria.candidate_densities = c["candidate_densities"].get<std::vector<double>>();
            if (c.contains("candidate_patterns")) {
                parsed.criteria.candidate_patterns.clear();
                for (const auto &value : c["candidate_patterns"])
                    parsed.criteria.candidate_patterns.push_back(enum_from_string(value.get<std::string>(), patterns, InfillPattern::Gyroid));
            }
        }
        if (j.contains("solver")) {
            const auto &s = j["solver"];
            parsed.solver.maximum_vertices = s.value("maximum_vertices", parsed.solver.maximum_vertices);
            parsed.solver.transverse_stiffness_ratio = s.value("transverse_stiffness_ratio", parsed.solver.transverse_stiffness_ratio);
            parsed.solver.regularization_ratio = s.value("regularization_ratio", parsed.solver.regularization_ratio);
        }
        setup = std::move(parsed);
        if (error) error->clear();
        return true;
    } catch (const std::exception &exception) {
        if (error) *error = exception.what();
        return false;
    }
}

std::string serialize_setup_for_config(const Setup &setup)
{
    static constexpr char HEX[] = "0123456789abcdef";
    const std::string json = serialize_setup(setup);
    std::string encoded;
    encoded.reserve(4 + json.size() * 2);
    encoded = "sa1:";
    for (const unsigned char byte : json) {
        encoded.push_back(HEX[byte >> 4]);
        encoded.push_back(HEX[byte & 0x0f]);
    }
    return encoded;
}

bool deserialize_setup_from_config(const std::string &config_text, Setup &setup, std::string *error)
{
    if (config_text.rfind("sa1:", 0) != 0)
        return deserialize_setup(config_text, setup, error);

    const std::string_view encoded(config_text.data() + 4, config_text.size() - 4);
    if ((encoded.size() & 1u) != 0u) {
        if (error) *error = "Malformed Strength Analysis config encoding.";
        return false;
    }
    auto nibble = [](char value) -> int {
        if (value >= '0' && value <= '9') return value - '0';
        if (value >= 'a' && value <= 'f') return value - 'a' + 10;
        if (value >= 'A' && value <= 'F') return value - 'A' + 10;
        return -1;
    };
    std::string json;
    json.reserve(encoded.size() / 2);
    for (size_t index = 0; index < encoded.size(); index += 2) {
        const int high = nibble(encoded[index]);
        const int low = nibble(encoded[index + 1]);
        if (high < 0 || low < 0) {
            if (error) *error = "Malformed Strength Analysis config encoding.";
            return false;
        }
        json.push_back(char((high << 4) | low));
    }
    return deserialize_setup(json, setup, error);
}

} // namespace Slic3r::StrengthAnalysis
