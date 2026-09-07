#include "StrengthAnalysis.hpp"

#include "AABBTreeIndirect.hpp"
#include "KDTreeIndirect.hpp"
#include "nlohmann/json.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <exception>
#include <iterator>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <string_view>

namespace Slic3r::StrengthAnalysis {

namespace {

constexpr double MM_TO_M = 1e-3;
constexpr double MM3_TO_M3 = 1e-9;
constexpr double NUMERIC_EPSILON = 1e-12;

std::uint64_t dense_profile_fingerprint(const indexed_triangle_set &mesh, const Result &result)
{
    std::uint64_t fingerprint = 14695981039346656037ULL;
    const auto mix = [&fingerprint](auto value) {
        static_assert(sizeof(value) <= sizeof(std::uint64_t));
        std::uint64_t bits = 0;
        std::memcpy(&bits, &value, sizeof(value));
        fingerprint = (fingerprint ^ bits) * 1099511628211ULL;
    };
    mix(mesh.vertices.size());
    mix(mesh.indices.size());
    for (const Vec3f &vertex : mesh.vertices)
        for (int axis = 0; axis < 3; ++axis)
            mix(vertex[axis]);
    for (const Vec3i32 &triangle : mesh.indices)
        for (int corner = 0; corner < 3; ++corner)
            mix(triangle[corner]);
    mix(int(result.status));
    mix(result.vertices.size());
    mix(result.maximum_stress_vertex);
    mix(result.maximum_von_mises_pa);
    // Hash every candidate, since a different vertex can become the hotspot while the old
    // hotspot's position and stress remain unchanged. Do not quantize away small mesh edits.
    for (const VertexResult &vertex : result.vertices) {
        for (int axis = 0; axis < 3; ++axis)
            mix(vertex.position_mm[axis]);
        mix(vertex.von_mises_pa);
    }
    return fingerprint;
}

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

bool valid_region_extent(const SphericalRegion &region)
{
    if (region.whole_model)
        return true;
    switch (region.shape) {
    case RegionShape::Sphere:
        return finite_positive(region.radius_mm);
    case RegionShape::Box:
        return region.size_mm.allFinite() && (region.size_mm.array() > 0.0).all();
    case RegionShape::Cylinder:
        return finite_positive(region.radius_mm) && std::isfinite(region.size_mm.z()) && region.size_mm.z() > 0.0 &&
            region.axis.allFinite() && region.axis.squaredNorm() > NUMERIC_EPSILON;
    case RegionShape::Surface:
        return !region.surface_triangles.empty();
    }
    return false;
}

std::vector<size_t> region_vertices_impl(const indexed_triangle_set &mesh, const SphericalRegion &region,
                                        bool select_nearest);
Vec3d normalized_or_zero(const Vec3d &v);

double squared_distance_to_triangle(const Vec3d &point, const Vec3d &a, const Vec3d &b, const Vec3d &c)
{
    const Vec3d ab = b - a;
    const Vec3d ac = c - a;
    const Vec3d ap = point - a;
    const double d1 = ab.dot(ap);
    const double d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0)
        return ap.squaredNorm();

    const Vec3d bp = point - b;
    const double d3 = ab.dot(bp);
    const double d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3)
        return bp.squaredNorm();

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double v = d1 / (d1 - d3);
        return (point - (a + v * ab)).squaredNorm();
    }

    const Vec3d cp = point - c;
    const double d5 = ab.dot(cp);
    const double d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6)
        return cp.squaredNorm();

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        const double w = d2 / (d2 - d6);
        return (point - (a + w * ac)).squaredNorm();
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0) {
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (point - (b + w * (c - b))).squaredNorm();
    }

    const double denominator = va + vb + vc;
    if (std::abs(denominator) <= NUMERIC_EPSILON)
        return std::min({ap.squaredNorm(), bp.squaredNorm(), cp.squaredNorm()});
    const double v = vb / denominator;
    const double w = vc / denominator;
    return (point - (a + ab * v + ac * w)).squaredNorm();
}

double distance_to_region(const indexed_triangle_set &mesh, const SphericalRegion &region, const Vec3d &point)
{
    if (region.whole_model)
        return 0.0;
    const Vec3d delta = point - region.center_mm;
    switch (region.shape) {
    case RegionShape::Sphere:
        return std::max(0.0, delta.norm() - std::max(0.0, region.radius_mm));
    case RegionShape::Box: {
        const Vec3d outside = (delta.cwiseAbs() - 0.5 * region.size_mm.cwiseAbs()).cwiseMax(0.0);
        return outside.norm();
    }
    case RegionShape::Cylinder: {
        const Vec3d axis = normalized_or_zero(region.axis);
        if (axis.isZero(NUMERIC_EPSILON))
            return std::numeric_limits<double>::infinity();
        const double axial = std::abs(delta.dot(axis));
        const double radial = (delta - delta.dot(axis) * axis).norm();
        return std::hypot(std::max(0.0, radial - std::max(0.0, region.radius_mm)),
                          std::max(0.0, axial - 0.5 * std::abs(region.size_mm.z())));
    }
    case RegionShape::Surface: {
        double squared_distance = std::numeric_limits<double>::infinity();
        for (size_t triangle_index : region.surface_triangles) {
            if (triangle_index >= mesh.indices.size())
                continue;
            const Vec3i32 &triangle = mesh.indices[triangle_index];
            if (triangle.minCoeff() < 0 || triangle.maxCoeff() >= int(mesh.vertices.size()))
                continue;
            squared_distance = std::min(squared_distance, squared_distance_to_triangle(
                point, mesh.vertices[size_t(triangle[0])].cast<double>(),
                mesh.vertices[size_t(triangle[1])].cast<double>(),
                mesh.vertices[size_t(triangle[2])].cast<double>()));
        }
        return std::sqrt(squared_distance);
    }
    }
    return std::numeric_limits<double>::infinity();
}

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

std::vector<size_t> region_vertices_impl(const indexed_triangle_set &mesh, const SphericalRegion &region, bool select_nearest)
{
    std::vector<size_t> selected;
    if (region.whole_model) {
        selected.resize(mesh.vertices.size());
        std::iota(selected.begin(), selected.end(), size_t(0));
        return selected;
    }

    if (region.shape == RegionShape::Surface && !region.surface_triangles.empty()) {
        std::set<size_t> vertices;
        for (size_t face : region.surface_triangles) {
            if (face >= mesh.indices.size())
                continue;
            const Vec3i32 &triangle = mesh.indices[face];
            for (int corner = 0; corner < 3; ++corner)
                if (triangle[corner] >= 0 && size_t(triangle[corner]) < mesh.vertices.size())
                    vertices.insert(size_t(triangle[corner]));
        }
        selected.assign(vertices.begin(), vertices.end());
        // Surface regions are an exact topological selection. Falling back to a nearby vertex
        // would silently move a load after a mesh topology change.
        return selected;
    }

    double nearest_distance = std::numeric_limits<double>::infinity();
    size_t nearest = 0;
    for (size_t index = 0; index < mesh.vertices.size(); ++index) {
        const Vec3d position = mesh.vertices[index].cast<double>();
        const double distance = (position - region.center_mm).norm();
        if (region.contains(position))
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
    return {{"center_mm", vec_to_array(region.center_mm)}, {"radius_mm", region.radius_mm},
            {"whole_model", region.whole_model}, {"shape", to_string(region.shape)},
            {"size_mm", vec_to_array(region.size_mm)}, {"axis", vec_to_array(region.axis)},
            {"surface_triangles", region.surface_triangles}};
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
    static const std::map<std::string, RegionShape> shapes{{"sphere", RegionShape::Sphere}, {"box", RegionShape::Box},
        {"cylinder", RegionShape::Cylinder}, {"surface", RegionShape::Surface}};
    region.shape = enum_from_string(j.value("shape", "sphere"), shapes, region.shape);
    if (j.contains("size_mm"))
        region.size_mm = array_to_vec(j["size_mm"], region.size_mm);
    if (j.contains("axis"))
        region.axis = array_to_vec(j["axis"], region.axis);
    if (j.contains("surface_triangles"))
        region.surface_triangles = j["surface_triangles"].get<std::vector<size_t>>();
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

std::vector<size_t> vertices_in_region(const indexed_triangle_set &mesh, const SphericalRegion &region, bool select_nearest)
{
    return region_vertices_impl(mesh, region, select_nearest);
}

bool SphericalRegion::contains(const Vec3d &position_mm) const
{
    if (whole_model)
        return true;
    const Vec3d delta = position_mm - center_mm;
    switch (shape) {
    case RegionShape::Sphere:
        return finite_positive(radius_mm) && delta.squaredNorm() <= radius_mm * radius_mm;
    case RegionShape::Box:
        return size_mm.allFinite() && (size_mm.array() > 0.0).all() &&
            (delta.array().abs() <= 0.5 * size_mm.array()).all();
    case RegionShape::Cylinder: {
        const Vec3d direction = normalized_or_zero(axis);
        if (direction.isZero(NUMERIC_EPSILON) || !finite_positive(radius_mm) ||
            !std::isfinite(size_mm.z()) || size_mm.z() <= 0.0)
            return false;
        const double axial = delta.dot(direction);
        const Vec3d radial = delta - axial * direction;
        return std::abs(axial) <= 0.5 * size_mm.z() && radial.squaredNorm() <= radius_mm * radius_mm;
    }
    case RegionShape::Surface:
        return false;
    }
    return false;
}

namespace {

using MeshEdge = std::pair<int32_t, int32_t>;

MeshEdge normalized_edge(int32_t first, int32_t second)
{
    return first < second ? MeshEdge{first, second} : MeshEdge{second, first};
}

bool valid_mesh_triangle(const indexed_triangle_set &mesh, const Vec3i32 &triangle)
{
    return triangle[0] >= 0 && triangle[1] >= 0 && triangle[2] >= 0 &&
        size_t(triangle[0]) < mesh.vertices.size() && size_t(triangle[1]) < mesh.vertices.size() &&
        size_t(triangle[2]) < mesh.vertices.size();
}

std::vector<std::vector<size_t>> face_adjacency(const indexed_triangle_set &mesh)
{
    std::map<MeshEdge, std::vector<size_t>> edge_faces;
    for (size_t face = 0; face < mesh.indices.size(); ++face) {
        const Vec3i32 &triangle = mesh.indices[face];
        if (!valid_mesh_triangle(mesh, triangle))
            continue;
        for (int edge = 0; edge < 3; ++edge)
            edge_faces[normalized_edge(triangle[edge], triangle[(edge + 1) % 3])].push_back(face);
    }
    std::vector<std::vector<size_t>> adjacency(mesh.indices.size());
    for (const auto &[edge, faces] : edge_faces) {
        (void) edge;
        for (size_t first = 0; first < faces.size(); ++first)
            for (size_t second = first + 1; second < faces.size(); ++second) {
                adjacency[faces[first]].push_back(faces[second]);
                adjacency[faces[second]].push_back(faces[first]);
            }
    }
    return adjacency;
}

} // namespace

std::vector<std::vector<size_t>> mesh_connected_components(const indexed_triangle_set &mesh)
{
    const auto adjacency = face_adjacency(mesh);
    std::vector<unsigned char> visited(mesh.indices.size(), 0);
    std::vector<std::vector<size_t>> components;
    for (size_t seed = 0; seed < mesh.indices.size(); ++seed) {
        if (visited[seed] || !valid_mesh_triangle(mesh, mesh.indices[seed]))
            continue;
        components.emplace_back();
        std::queue<size_t> pending;
        pending.push(seed);
        visited[seed] = 1;
        while (!pending.empty()) {
            const size_t face = pending.front();
            pending.pop();
            components.back().push_back(face);
            for (size_t neighbor : adjacency[face])
                if (!visited[neighbor]) {
                    visited[neighbor] = 1;
                    pending.push(neighbor);
                }
        }
    }
    return components;
}

std::vector<SurfacePatch> group_coplanar_surfaces(const indexed_triangle_set &mesh,
                                                  const SurfaceGroupingSettings &settings)
{
    if (!std::isfinite(settings.coplanar_angle_degrees) || settings.coplanar_angle_degrees < 0.0 ||
        settings.coplanar_angle_degrees >= 90.0 || !std::isfinite(settings.plane_tolerance_mm) ||
        settings.plane_tolerance_mm < 0.0)
        return {};

    const auto adjacency = face_adjacency(mesh);
    std::vector<Vec3d> normals(mesh.indices.size(), Vec3d::Zero());
    std::vector<double> areas(mesh.indices.size(), 0.0);
    std::vector<Vec3d> centroids(mesh.indices.size(), Vec3d::Zero());
    for (size_t face = 0; face < mesh.indices.size(); ++face) {
        const Vec3i32 &triangle = mesh.indices[face];
        if (!valid_mesh_triangle(mesh, triangle))
            continue;
        const Vec3d a = mesh.vertices[size_t(triangle[0])].cast<double>();
        const Vec3d b = mesh.vertices[size_t(triangle[1])].cast<double>();
        const Vec3d c = mesh.vertices[size_t(triangle[2])].cast<double>();
        const Vec3d cross = (b - a).cross(c - a);
        areas[face] = 0.5 * cross.norm();
        if (areas[face] > NUMERIC_EPSILON)
            normals[face] = cross.normalized();
        centroids[face] = (a + b + c) / 3.0;
    }

    std::vector<size_t> component_for_face(mesh.indices.size(), 0);
    const auto components = mesh_connected_components(mesh);
    for (size_t component = 0; component < components.size(); ++component)
        for (size_t face : components[component])
            component_for_face[face] = component;

    constexpr double pi = 3.14159265358979323846;
    const double cosine_limit = std::cos(settings.coplanar_angle_degrees * pi / 180.0);
    std::vector<unsigned char> visited(mesh.indices.size(), 0);
    std::vector<SurfacePatch> patches;
    for (size_t seed = 0; seed < mesh.indices.size(); ++seed) {
        if (visited[seed] || areas[seed] <= NUMERIC_EPSILON)
            continue;
        SurfacePatch patch;
        patch.component_index = component_for_face[seed];
        const Vec3d seed_normal = normals[seed];
        const double seed_plane = seed_normal.dot(centroids[seed]);
        std::queue<size_t> pending;
        pending.push(seed);
        visited[seed] = 1;
        Vec3d weighted_normal = Vec3d::Zero();
        Vec3d weighted_centroid = Vec3d::Zero();
        while (!pending.empty()) {
            const size_t face = pending.front();
            pending.pop();
            patch.triangles.push_back(face);
            patch.area_mm2 += areas[face];
            Vec3d aligned_normal = normals[face];
            if (aligned_normal.dot(seed_normal) < 0.0)
                aligned_normal = -aligned_normal;
            weighted_normal += aligned_normal * areas[face];
            weighted_centroid += centroids[face] * areas[face];
            for (size_t neighbor : adjacency[face]) {
                if (visited[neighbor] || areas[neighbor] <= NUMERIC_EPSILON ||
                    std::abs(normals[neighbor].dot(seed_normal)) < cosine_limit)
                    continue;
                const Vec3i32 &candidate = mesh.indices[neighbor];
                bool on_plane = true;
                for (int corner = 0; corner < 3; ++corner) {
                    const Vec3d point = mesh.vertices[size_t(candidate[corner])].cast<double>();
                    if (std::abs(seed_normal.dot(point) - seed_plane) > settings.plane_tolerance_mm) {
                        on_plane = false;
                        break;
                    }
                }
                if (on_plane) {
                    visited[neighbor] = 1;
                    pending.push(neighbor);
                }
            }
        }
        if (patch.area_mm2 > NUMERIC_EPSILON) {
            patch.centroid_mm = weighted_centroid / patch.area_mm2;
            patch.normal = normalized_or_zero(weighted_normal);
        }
        patches.push_back(std::move(patch));
    }
    return patches;
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

std::string to_string(RegionShape shape)
{
    switch (shape) {
    case RegionShape::Sphere: return "sphere";
    case RegionShape::Box: return "box";
    case RegionShape::Cylinder: return "cylinder";
    case RegionShape::Surface: return "surface";
    }
    return "sphere";
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
    if (setup.infill.background_density > 0.0 && setup.infill.background_density <= 1.0 &&
        setup.infill.dense_density > 0.0 && setup.infill.dense_density <= 1.0) {
        const double density_ratio = effective_solid_fraction(setup.infill.dense_density) /
            effective_solid_fraction(setup.infill.background_density);
        const PatternFactors background = pattern_factors(setup.infill.background_pattern);
        const PatternFactors dense = pattern_factors(setup.infill.dense_pattern);
        if (density_ratio * dense.strength + NUMERIC_EPSILON < background.strength ||
            density_ratio * dense.stiffness + NUMERIC_EPSILON < background.stiffness)
            errors.emplace_back("Dense infill density and pattern must not be weaker or less stiff than the background infill.");
    }
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
        if (!valid_region_extent(region))
            errors.emplace_back("Preserve-region dimensions or selected surface are invalid.");
        else if (region.shape == RegionShape::Surface && vertices_in_region(mesh, region, false).empty())
            errors.emplace_back("A preserve-region selected surface no longer exists in this mesh.");
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
            } else if (!valid_region_extent(load.region)) {
                errors.emplace_back("Fixed-region dimensions or selected surface are invalid.");
            } else {
                const std::vector<size_t> selected = vertices_in_region(mesh, load.region, false);
                if (load.region.shape == RegionShape::Surface && selected.empty())
                    errors.emplace_back("A fixed selected surface no longer exists in this mesh.");
                constrained_vertices.insert(selected.begin(), selected.end());
            }
        } else {
            has_force = true;
            if (!valid_region_extent(load.region))
                errors.emplace_back("Load-region dimensions or selected surface are invalid.");
            else if (load.region.shape == RegionShape::Surface && vertices_in_region(mesh, load.region, false).empty())
                errors.emplace_back("A load selected surface no longer exists in this mesh.");
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
        const std::vector<size_t> selected = vertices_in_region(mesh, load.region);
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
        std::vector<size_t> selected = vertices_in_region(mesh, region);
        selected.erase(std::remove_if(selected.begin(), selected.end(), [&fixed_vertices](size_t index) {
            return fixed_vertices.count(index) != 0;
        }), selected.end());
        if (selected.empty()) selected = vertices_in_region(mesh, region);
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
            double preserve_extent = preserve.radius_mm;
            if (preserve.shape == RegionShape::Box)
                preserve_extent = 0.5 * preserve.size_mm.norm();
            else if (preserve.shape == RegionShape::Cylinder)
                preserve_extent = std::hypot(preserve.radius_mm, 0.5 * preserve.size_mm.z());
            else if (preserve.shape == RegionShape::Surface) {
                preserve_extent = 0.0;
                for (size_t vertex : vertices_in_region(mesh, preserve, false))
                    preserve_extent = std::max(preserve_extent,
                        (mesh.vertices[vertex].cast<double>() - preserve.center_mm).norm());
            }
            const bool overlap = preserve.whole_model || preserve.contains(result.dense_region.region.center_mm) ||
                (result.dense_region.region.center_mm - preserve.center_mm).norm() <=
                    result.dense_region.region.radius_mm + preserve_extent;
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

namespace {

// Convex polygon clipping in doubles, with no spatial samples. The signed projected integral
// sum_faces integral_XY clamp(z_face - z_lower, 0, dz) dA is exactly the solid volume in a
// rectangular cell by the divergence theorem. Inward cavity faces subtract automatically.
// Splitting each face at cell Z planes accounts for every vertex/edge/grid crossing event.
using CellPolygon = std::vector<Vec3d>;

CellPolygon clip_cell_polygon(const CellPolygon &polygon, int axis, double plane, bool keep_above)
{
    CellPolygon clipped;
    if (polygon.empty())
        return clipped;
    clipped.reserve(polygon.size() + 1);
    Vec3d previous = polygon.back();
    double previous_distance = (previous[axis] - plane) * (keep_above ? 1.0 : -1.0);
    for (const Vec3d &point : polygon) {
        const double distance = (point[axis] - plane) * (keep_above ? 1.0 : -1.0);
        if ((distance >= 0.0) != (previous_distance >= 0.0)) {
            Vec3d crossing = previous + (point - previous) * (previous_distance / (previous_distance - distance));
            crossing[axis] = plane;
            clipped.push_back(crossing);
        }
        if (distance >= 0.0)
            clipped.push_back(point);
        previous = point;
        previous_distance = distance;
    }
    return clipped;
}

// Volume and first moments of the signed column above cut. Coordinates are relative to the
// model origin to limit cancellation when a part has a large object-coordinate translation.
Eigen::Vector4d column_integrals(const CellPolygon &polygon, double cut)
{
    const CellPolygon above = clip_cell_polygon(polygon, 2, cut, true);
    Eigen::Vector4d integral = Eigen::Vector4d::Zero();
    for (size_t i = 1; i + 1 < above.size(); ++i) {
        const std::array<Vec3d, 3> p{above[0], above[i], above[i + 1]};
        const double area = 0.5 * ((p[1].x() - p[0].x()) * (p[2].y() - p[0].y()) -
                                   (p[1].y() - p[0].y()) * (p[2].x() - p[0].x()));
        const Vec3d h(p[0].z() - cut, p[1].z() - cut, p[2].z() - cut);
        integral[0] += area * h.sum() / 3.0;
        for (int axis = 0; axis < 2; ++axis) {
            const Vec3d coordinate(p[0][axis], p[1][axis], p[2][axis]);
            integral[axis + 1] += area * (coordinate.sum() * h.sum() + coordinate.dot(h)) / 12.0;
        }
        integral[3] += area * (cut * h.sum() / 3.0 +
            (h.squaredNorm() + h[0] * h[1] + h[0] * h[2] + h[1] * h[2]) / 12.0);
    }
    return integral;
}

std::array<size_t, 3> dense_grid_size(const DenseRegionPreviewProfile &profile)
{
    return {profile.grid_planes_mm[0].size() - 1, profile.grid_planes_mm[1].size() - 1,
            profile.grid_planes_mm[2].size() - 1};
}

std::array<size_t, 3> dense_cell_coordinates(size_t id, const std::array<size_t, 3> &size)
{
    return {id % size[0], (id / size[0]) % size[1], id / (size[0] * size[1])};
}

void dense_cell_bounds(const DenseRegionPreviewProfile &profile, size_t id, Vec3d &lower, Vec3d &upper)
{
    const auto coordinate = dense_cell_coordinates(id, dense_grid_size(profile));
    for (int axis = 0; axis < 3; ++axis) {
        lower[axis] = profile.grid_planes_mm[axis][coordinate[axis]];
        upper[axis] = profile.grid_planes_mm[axis][coordinate[axis] + 1];
    }
}

bool cell_intersects_preserve(const indexed_triangle_set &mesh, const SphericalRegion &region,
                             const Vec3d &lower, const Vec3d &upper, double epsilon)
{
    if (region.whole_model)
        return true;
    const Vec3d center = 0.5 * (lower + upper);
    const Vec3d half = 0.5 * (upper - lower);
    if (region.shape == RegionShape::Sphere)
        return (region.center_mm - center).cwiseAbs().cwiseMax(half).operator-(half).norm() <= region.radius_mm + epsilon;
    if (region.shape == RegionShape::Box)
        return ((region.center_mm - center).cwiseAbs().array() <=
                (half + 0.5 * region.size_mm.cwiseAbs()).array() + epsilon).all();
    if (region.shape == RegionShape::Cylinder) {
        // A conservative distance bound also supports arbitrarily tilted cylinders. It can
        // exclude a few extra boundary cells, but can never reinforce a preserved feature.
        return distance_to_region(mesh, region, center) <= half.norm() + epsilon;
    }
    for (size_t face : region.surface_triangles) {
        const Vec3i32 &triangle = mesh.indices[face];
        CellPolygon polygon;
        for (int corner = 0; corner < 3; ++corner)
            polygon.push_back(mesh.vertices[size_t(triangle[corner])].cast<double>());
        for (int axis = 0; axis < 3 && !polygon.empty(); ++axis) {
            polygon = clip_cell_polygon(polygon, axis, lower[axis] - epsilon, true);
            polygon = clip_cell_polygon(polygon, axis, upper[axis] + epsilon, false);
        }
        if (!polygon.empty())
            return true;
    }
    return false;
}

indexed_triangle_set dense_boundary_mesh(const DenseRegionPreviewProfile &profile, const std::vector<unsigned char> &selected)
{
    indexed_triangle_set mesh;
    const auto size = dense_grid_size(profile);
    const std::array<size_t, 3> stride{1, size[0], size[0] * size[1]};
    const size_t vx = size[0] + 1, vy = size[1] + 1;
    std::vector<int32_t> vertices(vx * vy * (size[2] + 1), -1);
    const auto vertex_index = [&](const std::array<size_t, 3> &point) {
        int32_t &index = vertices[point[0] + vx * (point[1] + vy * point[2])];
        if (index < 0) {
            index = int32_t(mesh.vertices.size());
            mesh.vertices.emplace_back(float(profile.grid_planes_mm[0][point[0]]),
                                       float(profile.grid_planes_mm[1][point[1]]),
                                       float(profile.grid_planes_mm[2][point[2]]));
        }
        return index;
    };
    // Emit each exposed face once, with outward winding; face-adjacent cells share vertices.
    for (size_t id = 0; id < selected.size(); ++id) {
        if (!selected[id])
            continue;
        const auto coordinate = dense_cell_coordinates(id, size);
        for (int axis = 0; axis < 3; ++axis) {
            const int u = (axis + 1) % 3, v = (axis + 2) % 3;
            for (int side = 0; side < 2; ++side) {
                if ((side == 0 && coordinate[axis] > 0 && selected[id - stride[axis]]) ||
                    (side == 1 && coordinate[axis] + 1 < size[axis] && selected[id + stride[axis]]))
                    continue;
                auto point = coordinate;
                point[axis] += size_t(side);
                const int32_t a = vertex_index(point);
                ++point[u];
                const int32_t b = vertex_index(point);
                ++point[v];
                const int32_t c = vertex_index(point);
                --point[u];
                const int32_t d = vertex_index(point);
                if (side == 1) {
                    mesh.indices.emplace_back(a, b, c);
                    mesh.indices.emplace_back(a, c, d);
                } else {
                    mesh.indices.emplace_back(a, c, b);
                    mesh.indices.emplace_back(a, d, c);
                }
            }
        }
    }
    return mesh;
}

} // namespace

DenseRegionPreviewProfile build_dense_region_preview_profile(const indexed_triangle_set &mesh, const Result &result,
                                                              const CancelPredicate &cancel)
{
    DenseRegionPreviewProfile profile;
    const auto fail = [&](const std::string &warning) {
        DenseRegionPreviewProfile unavailable;
        unavailable.warning = warning;
        return unavailable;
    };
    const auto cancelled = [&]() { return cancel && cancel(); };
    if (cancelled())
        return fail("Dense-region profile preparation was cancelled.");
    if (!result.succeeded())
        return fail("Run a successful strength analysis before building a dense-region profile.");
    if (mesh.vertices.empty() || mesh.indices.empty() || result.vertices.size() != mesh.vertices.size())
        return fail("The solved result does not match the preview mesh.");
    constexpr size_t maximum_cells = 262144;
    constexpr size_t maximum_work = 8 * 1024 * 1024;
    if (mesh.vertices.size() > 200000 || mesh.indices.size() > 400000)
        return fail("Dense-region profile exceeds the mesh work limit (200000 vertices / 400000 faces); simplify the analysis mesh.");
    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        if (!mesh.vertices[i].allFinite() || !result.vertices[i].position_mm.allFinite() ||
            (mesh.vertices[i].cast<double>() - result.vertices[i].position_mm).squaredNorm() > 1e-12)
            return fail("The solved vertex positions do not match the preview mesh.");
        if (!std::isfinite(result.vertices[i].von_mises_pa) || result.vertices[i].von_mises_pa < 0.0)
            return fail("The solved stress field contains a non-finite or negative value.");
        if (result.vertices[i].von_mises_pa > profile.hotspot_stress_pa) {
            profile.hotspot_vertex = i;
            profile.hotspot_stress_pa = result.vertices[i].von_mises_pa;
        }
        if ((i & 1023) == 0 && cancelled())
            return fail("Dense-region profile preparation was cancelled.");
    }
    for (const Vec3i32 &triangle : mesh.indices)
        if (triangle.minCoeff() < 0 || triangle.maxCoeff() >= int(mesh.vertices.size()))
            return fail("The preview mesh contains an out-of-range triangle index.");
    if (profile.hotspot_stress_pa <= NUMERIC_EPSILON)
        return fail("The solved result has no finite non-zero stress hotspot to strengthen.");

    const double signed_volume = its_volume(mesh);
    if (!std::isfinite(signed_volume) || std::abs(signed_volume) <= NUMERIC_EPSILON)
        return fail("The preview mesh must enclose a positive finite volume.");
    const double orientation = signed_volume < 0.0 ? -1.0 : 1.0;
    const Vec3d origin = bounding_box(mesh).min.cast<double>();
    const auto components = mesh_connected_components(mesh);
    if (cancelled())
        return fail("Dense-region profile preparation was cancelled.");
    if (components.empty() || components.size() > 256)
        return fail("Dense-region profile exceeds the connected-geometry work limit (256 shells); repair or simplify the analysis mesh.");

    std::vector<size_t> face_component(mesh.indices.size(), 0);
    std::vector<std::vector<size_t>> component_vertices(components.size());
    size_t resolution = 40;
    for (;;) {
        for (auto &planes : profile.grid_planes_mm)
            planes.clear();
        // Each body supplies its own planes, so remote gaps cannot coarsen small or thin bodies.
        // Round planes to the exact float coordinates that the native modifier will store.
        for (size_t component = 0; component < components.size(); ++component) {
            Vec3d lower = Vec3d::Constant(std::numeric_limits<double>::infinity());
            Vec3d upper = -lower;
            auto &vertices = component_vertices[component];
            vertices.clear();
            for (size_t face : components[component]) {
                face_component[face] = component;
                for (int corner = 0; corner < 3; ++corner) {
                    const size_t vertex = size_t(mesh.indices[face][corner]);
                    vertices.push_back(vertex);
                    lower = lower.cwiseMin(mesh.vertices[vertex].cast<double>());
                    upper = upper.cwiseMax(mesh.vertices[vertex].cast<double>());
                }
            }
            std::sort(vertices.begin(), vertices.end());
            vertices.erase(std::unique(vertices.begin(), vertices.end()), vertices.end());
            const double longest = (upper - lower).maxCoeff();
            if (!(longest > 0.0))
                return fail("The preview mesh contains degenerate geometry.");
            for (int axis = 0; axis < 3; ++axis) {
                if (!(upper[axis] > lower[axis]))
                    return fail("The preview mesh contains an open or zero-thickness shell.");
                const size_t divisions = std::max(size_t(4), size_t(std::ceil(double(resolution) * (upper[axis] - lower[axis]) / longest)));
                for (size_t i = 0; i <= divisions; ++i)
                    profile.grid_planes_mm[axis].push_back(double(float(lower[axis] +
                        (upper[axis] - lower[axis]) * double(i) / double(divisions))));
            }
        }
        for (auto &planes : profile.grid_planes_mm) {
            std::sort(planes.begin(), planes.end());
            planes.erase(std::unique(planes.begin(), planes.end()), planes.end());
            if (planes.size() < 2)
                return fail("The preview grid has no finite extent.");
        }
        const auto candidate_size = dense_grid_size(profile);
        if (candidate_size[0] * candidate_size[1] * candidate_size[2] <= maximum_cells)
            break;
        if (resolution == 4)
            return fail("Dense-region profile exceeds the interior-cell memory limit (262144 cells); simplify the analysis mesh.");
        resolution = std::max(size_t(4), resolution * 3 / 4);
    }
    const auto size = dense_grid_size(profile);
    const size_t cell_count = size[0] * size[1] * size[2];
    if (cell_count > maximum_cells)
        return fail("Dense-region profile exceeds the interior-cell memory limit (262144 cells); simplify the analysis mesh.");
    const auto &xs = profile.grid_planes_mm[0];
    const auto &ys = profile.grid_planes_mm[1];
    const auto &zs = profile.grid_planes_mm[2];
    struct Integral {
        Eigen::Vector4d value{Eigen::Vector4d::Zero()};
        Eigen::Vector4d correction{Eigen::Vector4d::Zero()};
        void add(const Eigen::Vector4d &amount) {
            const Eigen::Vector4d corrected = amount - correction;
            const Eigen::Vector4d next = value + corrected;
            correction = (next - value) - corrected;
            value = next;
        }
    };
    std::vector<Integral> integrals(cell_count);
    size_t work = 0;
    for (size_t face = 0; face < mesh.indices.size(); ++face) {
        if (cancelled())
            return fail("Dense-region profile preparation was cancelled.");
        CellPolygon triangle;
        Vec3d lower = Vec3d::Constant(std::numeric_limits<double>::infinity());
        Vec3d upper = -lower;
        for (int corner = 0; corner < 3; ++corner) {
            const Vec3d point = mesh.vertices[size_t(mesh.indices[face][corner])].cast<double>();
            triangle.push_back(point - origin);
            lower = lower.cwiseMin(point);
            upper = upper.cwiseMax(point);
        }
        if ((triangle[1] - triangle[0]).cross(triangle[2] - triangle[0]).z() == 0.0)
            continue;
        const auto first_interval = [](const std::vector<double> &planes, double value) {
            return size_t(std::max(std::ptrdiff_t(0), std::upper_bound(planes.begin(), planes.end(), value) - planes.begin() - 1));
        };
        for (size_t y = first_interval(ys, lower.y()); y < size[1] && ys[y] < upper.y(); ++y) {
            CellPolygon row = clip_cell_polygon(triangle, 1, ys[y] - origin.y(), true);
            row = clip_cell_polygon(row, 1, ys[y + 1] - origin.y(), false);
            for (size_t x = first_interval(xs, lower.x()); x < size[0] && xs[x] < upper.x(); ++x) {
                if (++work > maximum_work)
                    return fail("Dense-region integration exceeds the face/cell work limit (8388608 operations); simplify the analysis mesh.");
                CellPolygon column = clip_cell_polygon(row, 0, xs[x] - origin.x(), true);
                column = clip_cell_polygon(column, 0, xs[x + 1] - origin.x(), false);
                if (column.size() < 3)
                    continue;
                Eigen::Vector4d below = column_integrals(column, zs.front() - origin.z());
                for (size_t z = 0; z < size[2] && zs[z] < upper.z(); ++z) {
                    if (++work > maximum_work)
                        return fail("Dense-region integration exceeds the face/cell work limit (8388608 operations); simplify the analysis mesh.");
                    if ((work & 1023) == 0 && cancelled())
                        return fail("Dense-region profile preparation was cancelled.");
                    const Eigen::Vector4d above = column_integrals(column, zs[z + 1] - origin.z());
                    integrals[x + size[0] * (y + size[1] * z)].add(orientation * (below - above));
                    below = above;
                }
            }
        }
    }

    const auto coordinate = [&](size_t vertex, size_t axis) { return double(mesh.vertices[vertex][axis]); };
    using StressTree = KDTreeIndirect<3, double, decltype(coordinate)>;
    std::vector<StressTree> stress_trees;
    stress_trees.reserve(components.size());
    for (auto &vertices : component_vertices)
        stress_trees.emplace_back(coordinate, std::move(vertices));
    const auto surface_tree = AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(mesh.vertices, mesh.indices);
    for (size_t id = 0; id < cell_count; ++id) {
        if ((id & 255) == 0 && cancelled())
            return fail("Dense-region profile preparation was cancelled.");
        const double volume = integrals[id].value[0];
        Vec3d lower, upper;
        dense_cell_bounds(profile, id, lower, upper);
        const double box_volume = (upper - lower).prod();
        const double tolerance = std::max(box_volume * 1e-9, std::abs(signed_volume) * 1e-13);
        if (!std::isfinite(volume) || volume < -tolerance || volume > box_volume + tolerance)
            return fail("Integrated cells contain negative or overlapping solid volume; repair the analysis mesh.");
        if (volume <= tolerance)
            continue;
        const Vec3d centroid = (origin + integrals[id].value.tail<3>() / volume).cwiseMax(lower).cwiseMin(upper);
        size_t closest_face = 0;
        Vec3d surface_point;
        AABBTreeIndirect::squared_distance_to_indexed_triangle_set(
            mesh.vertices, mesh.indices, surface_tree, centroid, closest_face, surface_point);
        // Restrict interpolation to the nearest boundary's connected shell. A close but
        // disconnected body must never contribute its (possibly very different) solved stress.
        const StressTree &tree = stress_trees[face_component[closest_face]];
        const auto nearby = find_closest_points<8>(tree, centroid);
        double weighted_stress = 0.0, weights = 0.0;
        for (size_t vertex : nearby) {
            if (vertex == StressTree::npos)
                continue;
            const double distance_squared = (mesh.vertices[vertex].cast<double>() - centroid).squaredNorm();
            const double weight = 1.0 / std::max(distance_squared, 1e-12);
            weighted_stress += weight * result.vertices[vertex].von_mises_pa;
            weights += weight;
        }
        if (!(weights > 0.0))
            return fail("Could not interpolate the solved stress inside the model.");
        profile.cells.push_back({id, std::min(volume, box_volume), weighted_stress / weights});
    }
    if (cancelled())
        return fail("Dense-region profile preparation was cancelled.");
    std::sort(profile.cells.begin(), profile.cells.end(), [](const DenseRegionCell &lhs, const DenseRegionCell &rhs) {
        return lhs.stress_pa != rhs.stress_pa ? lhs.stress_pa > rhs.stress_pa : lhs.grid_index < rhs.grid_index;
    });
    double cumulative_volume = 0.0, correction = 0.0;
    for (const DenseRegionCell &cell : profile.cells) {
        const double corrected = cell.volume_mm3 - correction;
        const double next = cumulative_volume + corrected;
        correction = (next - cumulative_volume) - corrected;
        cumulative_volume = next;
        profile.cumulative_volume_mm3.push_back(cumulative_volume);
    }
    if (!(cumulative_volume > 0.0) || std::abs(cumulative_volume - std::abs(signed_volume)) > 0.001 * std::abs(signed_volume))
        return fail("Integrated cells disagree with the enclosed mesh volume by more than 0.1%; repair the analysis mesh.");
    profile.vertex_count = mesh.vertices.size();
    profile.triangle_count = mesh.indices.size();
    profile.hotspot_center_mm = mesh.vertices[profile.hotspot_vertex].cast<double>();
    profile.sampled_volume_mm3 = cumulative_volume;
    profile.solid_volume_m3 = cumulative_volume * MM3_TO_M3;
    profile.mesh_result_fingerprint = dense_profile_fingerprint(mesh, result);
    if (cancelled())
        return fail("Dense-region profile preparation was cancelled.");
    profile.available = true;
    return profile;
}

DenseRegionPreview preview_dense_region(const indexed_triangle_set &mesh, const Setup &setup,
                                        const Result &result, double target_volume_fraction,
                                        const DenseRegionPreviewProfile *prepared_profile, bool generate_modifier_mesh,
                                        double minimum_stress_fraction)
{
    DenseRegionPreview preview;
    if (!std::isfinite(target_volume_fraction)) {
        preview.warning = "The strengthened-volume target must be finite.";
        return preview;
    }
    if (!std::isfinite(minimum_stress_fraction) || minimum_stress_fraction < 0.0 || minimum_stress_fraction > 1.0) {
        preview.warning = "The relative stress cutoff must be within [0, 1].";
        return preview;
    }
    preview.target_volume_fraction = std::clamp(target_volume_fraction, 0.0, 1.0);
    preview.estimated_total_mass_kg = std::max(0.0, result.estimated_mass_kg);
    preview.predicted_minimum_safety_factor = result.minimum_safety_factor;
    preview.predicted_maximum_displacement_mm = std::max(0.0, result.maximum_displacement_m * 1000.0);

    DenseRegionPreviewProfile local_profile;
    if (prepared_profile == nullptr) {
        local_profile = build_dense_region_preview_profile(mesh, result);
        prepared_profile = &local_profile;
    }
    const DenseRegionPreviewProfile &profile = *prepared_profile;
    if (!profile.available) {
        preview.warning = profile.warning;
        return preview;
    }
    if (!result.succeeded() || profile.vertex_count != mesh.vertices.size() ||
        profile.triangle_count != mesh.indices.size() || result.vertices.size() != mesh.vertices.size() ||
        profile.cells.empty() || profile.cells.size() != profile.cumulative_volume_mm3.size() ||
        profile.mesh_result_fingerprint != dense_profile_fingerprint(mesh, result)) {
        preview.warning = "The dense-region profile is stale; rebuild it for the current mesh and solved stress field.";
        return preview;
    }
    size_t preserve_work = 0;
    for (const SphericalRegion &preserve : setup.preserve_regions) {
        if (!valid_region_extent(preserve) || !preserve.center_mm.allFinite() ||
            (preserve.shape == RegionShape::Surface && std::any_of(preserve.surface_triangles.begin(),
                preserve.surface_triangles.end(), [&](size_t face) { return face >= mesh.indices.size(); }))) {
            preview.warning = "A preserve region is invalid; correct it before creating reinforcement.";
            return preview;
        }
        const size_t cost = preserve.shape == RegionShape::Surface ? preserve.surface_triangles.size() : 1;
        if (cost > (8 * 1024 * 1024 - preserve_work) / profile.cells.size()) {
            preview.warning = "Dense-region preserve exclusion exceeds the cell/region work limit (8388608 checks); simplify the preserved selection.";
            return preview;
        }
        preserve_work += cost * profile.cells.size();
    }
    const auto size = dense_grid_size(profile);
    std::vector<unsigned char> selected(size[0] * size[1] * size[2], 0);
    double included_volume_mm3 = 0.0, included_demand = 0.0, total_demand = 0.0;
    const double target_volume = preview.target_volume_fraction * profile.sampled_volume_mm3;
    const double epsilon = std::max(1e-7, bounding_box(mesh).size().cast<double>().norm() * 1e-9);
    Vec3d summary_min = Vec3d::Constant(std::numeric_limits<double>::infinity()), summary_max = -summary_min;
    bool skipped_preserve = false, skipped_threshold = false;
    for (const DenseRegionCell &cell : profile.cells) {
        total_demand += cell.volume_mm3 * cell.stress_pa;
        if (preview.target_volume_fraction <= 0.0 ||
            (preview.target_volume_fraction < 1.0 && included_volume_mm3 >= target_volume))
            continue;
        if (cell.stress_pa < minimum_stress_fraction * profile.hotspot_stress_pa) {
            skipped_threshold = true;
            continue;
        }
        Vec3d lower, upper;
        dense_cell_bounds(profile, cell.grid_index, lower, upper);
        if (std::any_of(setup.preserve_regions.begin(), setup.preserve_regions.end(), [&](const SphericalRegion &preserve) {
                return cell_intersects_preserve(mesh, preserve, lower, upper, epsilon);
            })) {
            skipped_preserve = true;
            continue;
        }
        selected[cell.grid_index] = 1;
        ++preview.selected_cell_count;
        included_volume_mm3 += cell.volume_mm3;
        included_demand += cell.volume_mm3 * cell.stress_pa;
        preview.equivalent_stress_threshold = std::clamp(cell.stress_pa / profile.hotspot_stress_pa, 0.0, 1.0);
        summary_min = summary_min.cwiseMin(lower);
        summary_max = summary_max.cwiseMax(upper);
    }
    preview.available = true;
    preview.region.shape = RegionShape::Box;
    preview.region.radius_mm = 0.0;
    preview.region.size_mm.setZero();
    if (preview.selected_cell_count > 0) {
        preview.region.center_mm = 0.5 * (summary_min + summary_max);
        preview.region.size_mm = summary_max - summary_min;
        preview.region.radius_mm = 0.5 * preview.region.size_mm.norm();
        if (generate_modifier_mesh)
            preview.modifier_mesh = dense_boundary_mesh(profile, selected);
    }
    // A selected boundary vertex can belong to either side of a grid plane. Check all adjacent
    // cells, never assign response benefit merely because it lies inside the diagnostic bounds.
    for (size_t vertex = 0; vertex < mesh.vertices.size(); ++vertex) {
        std::array<std::vector<size_t>, 3> adjacent;
        for (int axis = 0; axis < 3; ++axis) {
            const auto &planes = profile.grid_planes_mm[axis];
            const double value = double(mesh.vertices[vertex][axis]);
            const auto position = std::lower_bound(planes.begin(), planes.end(), value);
            const size_t index = size_t(position - planes.begin());
            if (index > 0)
                adjacent[axis].push_back(index - 1);
            if (position != planes.end() && std::abs(*position - value) <= epsilon && index < size[axis])
                adjacent[axis].push_back(index);
        }
        bool affected = false;
        for (size_t z : adjacent[2])
            for (size_t y : adjacent[1])
                for (size_t x : adjacent[0])
                    affected = affected || selected[x + size[0] * (y + size[1] * z)];
        if (affected)
            preview.affected_vertices.push_back(vertex);
    }
    preview.estimated_volume_fraction = std::clamp(included_volume_mm3 / profile.sampled_volume_mm3, 0.0, 1.0);
    preview.estimated_volume_m3 = included_volume_mm3 * MM3_TO_M3;
    preview.stress_coverage = total_demand > 0.0 ? std::clamp(included_demand / total_demand, 0.0, 1.0) : 0.0;
    if (skipped_preserve)
        preview.warning = included_volume_mm3 + 1e-9 < target_volume ?
            "Preserved features and their intersecting grid cells limit the attainable reinforced volume." :
            "Cells intersecting preserved features were excluded from reinforcement.";
    if (skipped_threshold) {
        if (!preview.warning.empty())
            preview.warning += " ";
        preview.warning += "Cells below the requested stress threshold were excluded from reinforcement.";
    }

    const double background_fraction = effective_solid_fraction(setup.infill.background_density);
    const double dense_fraction = effective_solid_fraction(setup.infill.dense_density);
    const PatternFactors background_pattern = pattern_factors(setup.infill.background_pattern);
    const PatternFactors dense_pattern = pattern_factors(setup.infill.dense_pattern);
    const double density_ratio = dense_fraction / std::max(background_fraction, NUMERIC_EPSILON);
    preview.local_strength_multiplier = density_ratio * dense_pattern.strength /
        std::max(background_pattern.strength, NUMERIC_EPSILON);
    preview.local_stiffness_multiplier = density_ratio * dense_pattern.stiffness /
        std::max(background_pattern.stiffness, NUMERIC_EPSILON);
    const Material material = setup.material.calibrated();
    preview.estimated_added_mass_kg = std::max(0.0, preview.estimated_volume_m3 * material.density_kg_m3 *
                                                       (dense_fraction - background_fraction));
    preview.estimated_total_mass_kg = std::max(0.0, result.estimated_mass_kg) + preview.estimated_added_mass_kg;

    std::vector<unsigned char> strengthened(mesh.vertices.size(), 0);
    for (size_t vertex : preview.affected_vertices)
        strengthened[vertex] = 1;
    double predicted_safety_factor = std::numeric_limits<double>::infinity();
    double predicted_displacement_mm = 0.0;
    for (size_t vertex = 0; vertex < result.vertices.size(); ++vertex) {
        const VertexResult &value = result.vertices[vertex];
        const double strength_multiplier = strengthened[vertex] ? preview.local_strength_multiplier : 1.0;
        const double stiffness_multiplier = strengthened[vertex] ? preview.local_stiffness_multiplier : 1.0;
        if (std::isfinite(value.safety_factor))
            predicted_safety_factor = std::min(predicted_safety_factor,
                                               value.safety_factor * strength_multiplier);
        if (value.displacement_m.allFinite())
            predicted_displacement_mm = std::max(predicted_displacement_mm,
                value.displacement_m.norm() * 1000.0 / stiffness_multiplier);
    }
    preview.predicted_minimum_safety_factor = predicted_safety_factor;
    preview.predicted_maximum_displacement_mm = predicted_displacement_mm;
    if (setup.gravity.enabled && preview.estimated_added_mass_kg > NUMERIC_EPSILON) {
        preview.response_estimate_available = false;
        preview.predicted_minimum_safety_factor = std::numeric_limits<double>::quiet_NaN();
        preview.predicted_maximum_displacement_mm = std::numeric_limits<double>::quiet_NaN();
        if (!preview.warning.empty())
            preview.warning += " ";
        preview.warning += "Safety factor and deformation require re-solving with the changed self-weight; only region sizing and mass are estimated.";
    }
    return preview;
}

Vec3d print_layer_axis_for_transform(const Transform3d &transform)
{
    const Matrix3d linear = transform.linear();
    if (!transform.matrix().allFinite() || !std::isfinite(linear.determinant()) || linear.determinant() == 0.0)
        return Vec3d::Zero();
    return normalized_or_zero(linear.transpose() * Vec3d::UnitZ());
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
    j["follow_prepare_orientation"] = setup.follow_prepare_orientation;
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
        parsed.follow_prepare_orientation = j.value("follow_prepare_orientation", true);
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
