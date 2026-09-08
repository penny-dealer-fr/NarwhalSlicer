#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <nlohmann/json.hpp>

#include "libslic3r/StrengthAnalysis.hpp"
#include "libslic3r/Format/3mf.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"
#include "libslic3r/ClipperUtils.hpp"

#include "test_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

using namespace Slic3r;
using namespace Slic3r::StrengthAnalysis;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

Setup cube_setup(const indexed_triangle_set &mesh)
{
    Setup setup;
    const BoundingBoxf3 bounds = bounding_box(mesh);
    const Vec3d center = bounds.center().cast<double>();
    const Vec3d size = bounds.size().cast<double>();

    Load fixed;
    fixed.name = "Fixed left face";
    fixed.type = LoadType::Fixed;
    fixed.region.center_mm = Vec3d(bounds.min.x(), center.y(), center.z());
    fixed.region.radius_mm = 0.51 * std::hypot(size.y(), size.z());
    setup.loads.push_back(fixed);

    Load force;
    force.name = "Right-face force";
    force.type = LoadType::LocalForce;
    force.region.center_mm = Vec3d(bounds.max.x(), center.y(), center.z());
    force.region.radius_mm = 0.51 * std::max(size.y(), size.z());
    force.direction = Vec3d::UnitX();
    force.magnitude_n = 100.0;
    setup.loads.push_back(force);
    return setup;
}

Result synthetic_result(const indexed_triangle_set &mesh, size_t hotspot = 0)
{
    Result result;
    result.status = AnalysisStatus::Success;
    result.message = "Synthetic solved field";
    result.vertices.resize(mesh.vertices.size());
    hotspot = std::min(hotspot, mesh.vertices.empty() ? size_t(0) : mesh.vertices.size() - 1);
    for (size_t vertex = 0; vertex < mesh.vertices.size(); ++vertex) {
        VertexResult &value = result.vertices[vertex];
        value.position_mm = mesh.vertices[vertex].cast<double>();
        value.von_mises_pa = vertex == hotspot ? 2.0e6 : 1.0e6;
        value.maximum_shear_pa = 0.5 * value.von_mises_pa;
        value.safety_factor = vertex == hotspot ? 2.0 : 4.0;
        value.displacement_m = Vec3d::Constant(vertex == hotspot ? 2.0e-4 : 1.0e-4);
    }
    result.maximum_stress_vertex = hotspot;
    result.maximum_von_mises_pa = 2.0e6;
    result.minimum_safety_factor_vertex = hotspot;
    result.minimum_safety_factor = 2.0;
    result.maximum_displacement_vertex = hotspot;
    result.maximum_displacement_m = result.vertices.empty() ? 0.0 : result.vertices[hotspot].displacement_m.norm();
    result.estimated_mass_kg = std::abs(its_volume(mesh)) * 1e-9 * 1240.0 * (0.18 + 0.82 * 0.20);
    return result;
}

double sliced_dense_volume(const indexed_triangle_set &model, const DenseRegionPreview &preview,
                           const DenseRegionPreviewProfile &profile)
{
    std::vector<float> zs;
    std::vector<double> weights;
    const auto &planes = profile.grid_planes_mm[2];
    for (size_t i = 1; i < planes.size(); ++i) {
        const double midpoint = 0.5 * (planes[i - 1] + planes[i]);
        const double half_height = 0.5 * (planes[i] - planes[i - 1]);
        for (double direction : {-1.0, 1.0}) {
            zs.push_back(float(midpoint + direction * half_height / std::sqrt(3.0)));
            weights.push_back(half_height);
        }
    }
    const auto model_slices = slice_mesh_ex(model, zs);
    const auto modifier_slices = slice_mesh_ex(preview.modifier_mesh, zs);
    double volume = 0.0;
    for (size_t i = 0; i < zs.size(); ++i)
        for (const ExPolygon &polygon : intersection_ex(model_slices[i], modifier_slices[i]))
            volume += polygon.area() * SCALING_FACTOR * SCALING_FACTOR * weights[i];
    return volume;
}

} // namespace

TEST_CASE("Spherical region selection exposes the exact solver vertex set", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 20.0, 20.0);
    REQUIRE_FALSE(mesh.vertices.empty());

    SphericalRegion local;
    local.center_mm = mesh.vertices.front().cast<double>();
    local.radius_mm = 0.01;
    const std::vector<size_t> local_vertices = vertices_in_region(mesh, local, false);
    REQUIRE_FALSE(local_vertices.empty());
    for (size_t vertex : local_vertices)
        CHECK((mesh.vertices[vertex].cast<double>() - local.center_mm).norm() <= local.radius_mm);

    local.center_mm = Vec3d(1000.0, 1000.0, 1000.0);
    CHECK(vertices_in_region(mesh, local, false).empty());
    CHECK(vertices_in_region(mesh, local, true).size() == 1);

    local.whole_model = true;
    CHECK(vertices_in_region(mesh, local).size() == mesh.vertices.size());
}

TEST_CASE("Region shapes select the same exact vertices used by the solver", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 20.0, 20.0);
    const BoundingBoxf3 bounds = bounding_box(mesh);
    const Vec3d center = bounds.center().cast<double>();

    SphericalRegion box;
    box.shape = RegionShape::Box;
    box.center_mm = Vec3d(bounds.min.x(), center.y(), center.z());
    box.size_mm = Vec3d(0.1, 21.0, 21.0);
    const auto box_vertices = vertices_in_region(mesh, box, false);
    REQUIRE(box_vertices.size() == 4);
    for (size_t vertex : box_vertices)
        CHECK_THAT(double(mesh.vertices[vertex].x()), WithinAbs(bounds.min.x(), 1e-6));

    SphericalRegion cylinder;
    cylinder.shape = RegionShape::Cylinder;
    cylinder.center_mm = Vec3d(bounds.min.x(), center.y(), center.z());
    cylinder.axis = Vec3d::UnitX();
    cylinder.radius_mm = 15.0;
    cylinder.size_mm.z() = 0.1;
    CHECK(vertices_in_region(mesh, cylinder, false).size() == 4);

    SphericalRegion surface;
    surface.shape = RegionShape::Surface;
    surface.surface_triangles = {0};
    const auto surface_vertices = vertices_in_region(mesh, surface, false);
    REQUIRE(surface_vertices.size() == 3);
    for (int corner = 0; corner < 3; ++corner)
        CHECK(std::find(surface_vertices.begin(), surface_vertices.end(), size_t(mesh.indices[0][corner])) != surface_vertices.end());

    surface.surface_triangles = {mesh.indices.size() + 10};
    CHECK(vertices_in_region(mesh, surface, false).empty());
    CHECK(vertices_in_region(mesh, surface, true).empty());

    Setup stale_surface = cube_setup(mesh);
    stale_surface.loads.back().region = surface;
    const auto errors = validate(mesh, stale_surface);
    CHECK(std::any_of(errors.begin(), errors.end(), [](const std::string &error) {
        return error.find("selected surface no longer exists") != std::string::npos;
    }));
}

TEST_CASE("Coplanar grouping creates solid-like cube faces without changing the mesh", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 20.0, 20.0);
    const auto original_vertices = mesh.vertices;
    const auto original_indices = mesh.indices;
    const auto patches = group_coplanar_surfaces(mesh);
    REQUIRE(patches.size() == 6);
    for (const SurfacePatch &patch : patches) {
        CHECK(patch.triangles.size() == 2);
        CHECK_THAT(patch.area_mm2, WithinRel(400.0, 1e-6));
        CHECK_THAT(patch.normal.norm(), WithinRel(1.0, 1e-10));
        CHECK(patch.component_index == 0);
    }
    const auto components = mesh_connected_components(mesh);
    REQUIRE(components.size() == 1);
    CHECK(components.front().size() == mesh.indices.size());
    REQUIRE(mesh.vertices.size() == original_vertices.size());
    REQUIRE(mesh.indices.size() == original_indices.size());
    for (size_t index = 0; index < mesh.vertices.size(); ++index)
        CHECK(mesh.vertices[index].isApprox(original_vertices[index]));
    for (size_t index = 0; index < mesh.indices.size(); ++index)
        CHECK(mesh.indices[index].isApprox(original_indices[index]));
}

TEST_CASE("Strength setup survives a versioned JSON round trip", "[StrengthAnalysis]")
{
    Setup setup;
    setup.material = *find_builtin_material("petg_generic");
    setup.material.calibration.strength_z_scale = 1.17;
    setup.material.calibration.source = "Printed Z coupon";
    setup.print_layer_axis = Vec3d::UnitY();
    setup.gravity.enabled = true;
    setup.infill.background_pattern = StrengthAnalysis::InfillPattern::Cubic;
    setup.infill.background_density = 0.37;
    setup.preserve_regions.push_back({Vec3d(1.0, 2.0, 3.0), 4.0, false});
    setup.preserve_regions.back().shape = RegionShape::Cylinder;
    setup.preserve_regions.back().size_mm = Vec3d(8.0, 8.0, 12.0);
    setup.preserve_regions.back().axis = Vec3d::UnitY();
    Load load;
    load.type = LoadType::ImpactForce;
    load.region.center_mm = Vec3d(3.0, 4.0, 5.0);
    load.direction = Vec3d(1.0, 2.0, 3.0);
    load.magnitude_n = 42.0;
    load.impact_factor = 3.0;
    load.target_safety_factor = 2.2;
    load.strength_basis = StrengthBasis::Ultimate;
    setup.loads.push_back(load);

    Setup decoded;
    std::string error;
    REQUIRE(deserialize_setup(serialize_setup(setup), decoded, &error));
    REQUIRE(error.empty());
    CHECK(decoded.material.key == "petg_generic");
    CHECK_THAT(decoded.material.calibration.strength_z_scale, WithinRel(1.17, 1e-12));
    CHECK(decoded.material.calibration.source == "Printed Z coupon");
    CHECK(decoded.print_layer_axis.isApprox(Vec3d::UnitY()));
    CHECK(decoded.gravity.enabled);
    CHECK(decoded.infill.background_pattern == StrengthAnalysis::InfillPattern::Cubic);
    CHECK_THAT(decoded.infill.background_density, WithinRel(0.37, 1e-12));
    REQUIRE(decoded.loads.size() == 1);
    CHECK(decoded.loads.front().type == LoadType::ImpactForce);
    CHECK_THAT(decoded.loads.front().magnitude_n, WithinRel(42.0, 1e-12));
    CHECK_THAT(decoded.loads.front().target_safety_factor, WithinRel(2.2, 1e-12));
    CHECK(decoded.loads.front().strength_basis == StrengthBasis::Ultimate);
    REQUIRE(decoded.preserve_regions.size() == 1);
    CHECK(decoded.preserve_regions.front().shape == RegionShape::Cylinder);
    CHECK(decoded.preserve_regions.front().size_mm.isApprox(Vec3d(8.0, 8.0, 12.0)));
    CHECK(decoded.preserve_regions.front().axis.isApprox(Vec3d::UnitY()));
    CHECK(decoded.preserve_regions.front().center_mm.isApprox(Vec3d(1.0, 2.0, 3.0)));
}

TEST_CASE("Strength setup follows Prepare orientation by default and preserves the choice in JSON", "[StrengthAnalysis]")
{
    Setup setup;
    REQUIRE(setup.follow_prepare_orientation);
    const bool follow_prepare_orientation = GENERATE(true, false);
    CAPTURE(follow_prepare_orientation);
    setup.follow_prepare_orientation = follow_prepare_orientation;

    const std::string encoded = serialize_setup(setup);
    const auto json = nlohmann::json::parse(encoded);
    REQUIRE(json.contains("follow_prepare_orientation"));
    REQUIRE(json.at("follow_prepare_orientation").is_boolean());
    CHECK(json.at("follow_prepare_orientation").get<bool>() == follow_prepare_orientation);

    Setup decoded;
    decoded.follow_prepare_orientation = !follow_prepare_orientation;
    std::string error;
    REQUIRE(deserialize_setup(encoded, decoded, &error));
    CHECK(error.empty());
    CHECK(decoded.follow_prepare_orientation == follow_prepare_orientation);
}

TEST_CASE("Older strength setup JSON follows Prepare orientation when the choice is missing", "[StrengthAnalysis]")
{
    Setup setup;
    setup.follow_prepare_orientation = false;
    setup.print_layer_axis = Vec3d::UnitY();
    auto json = nlohmann::json::parse(serialize_setup(setup));
    REQUIRE(json.erase("follow_prepare_orientation") == 1);

    Setup decoded;
    decoded.follow_prepare_orientation = false;
    std::string error;
    REQUIRE(deserialize_setup(json.dump(), decoded, &error));
    CHECK(error.empty());
    CHECK(decoded.follow_prepare_orientation);
    CHECK_THAT((decoded.print_layer_axis - Vec3d::UnitY()).norm(), WithinAbs(0.0, 1e-12));
}

TEST_CASE("Print layer axes recover the world layer normal after rotation scaling and mirroring", "[StrengthAnalysis]")
{
    const double mirror = GENERATE(1.0, -1.0);
    CAPTURE(mirror);
    Transform3d transform = Transform3d::Identity();
    transform.rotate(Eigen::AngleAxisd(0.71, Vec3d(1.0, 2.0, 3.0).normalized()));
    transform.scale(Vec3d(2.0 * mirror, 0.5, 3.0));

    const Vec3d axis = print_layer_axis_for_transform(transform);
    REQUIRE(axis.allFinite());
    REQUIRE_THAT(axis.norm(), WithinAbs(1.0, 1e-12));
    // A plane normal maps to world coordinates by the inverse transpose,
    // including under nonuniform scaling and a change of handedness.
    const Vec3d world_normal = (transform.linear().inverse().transpose() * axis).normalized();
    CHECK_THAT((world_normal - Vec3d::UnitZ()).norm(), WithinAbs(0.0, 1e-12));
}

TEST_CASE("Translation does not change the print layer axis", "[StrengthAnalysis]")
{
    Transform3d transform = Transform3d::Identity();
    CHECK_THAT((print_layer_axis_for_transform(transform) - Vec3d::UnitZ()).norm(), WithinAbs(0.0, 1e-12));
    transform.rotate(Eigen::AngleAxisd(0.71, Vec3d(1.0, 2.0, 3.0).normalized()));
    transform.scale(Vec3d(-2.0, 0.5, 3.0));
    const Vec3d untranslated_axis = print_layer_axis_for_transform(transform);
    REQUIRE_THAT(untranslated_axis.norm(), WithinAbs(1.0, 1e-12));

    transform.translation() = Vec3d(123.0, -456.0, 789.0);
    CHECK_THAT((print_layer_axis_for_transform(transform) - untranslated_axis).norm(), WithinAbs(0.0, 1e-12));
}

TEST_CASE("Study coordinate mapping keeps points normals and physical vectors in their proper frames", "[StrengthAnalysis]")
{
    const double mirror = GENERATE(1.0, -1.0);
    Transform3d transform = Transform3d::Identity();
    transform.rotate(Eigen::AngleAxisd(0.7, Vec3d(1.0, 2.0, 3.0).normalized()));
    transform.scale(Vec3d(2.0 * mirror, 0.5, 3.0));
    transform.translation() = Vec3d(500.0, -250.0, 40.0);
    const StudyCoordinateFrame frame(transform);
    REQUIRE(frame.valid);
    const Vec3d point(4.0, -2.0, 7.0);
    CHECK_THAT((frame.scene_to_model * (frame.model_to_scene * point) - point).norm(), WithinAbs(0.0, 1e-12));
    const Vec3d displacement(0.001, -0.002, 0.003);
    const Vec3d scene_vector = frame.physical_vector_to_scene(displacement);
    CHECK_THAT(scene_vector.norm(), WithinRel(displacement.norm(), 1e-12));
    CHECK_THAT((frame.scene_vector_to_physical(scene_vector) - displacement).norm(), WithinAbs(0.0, 1e-12));
    CHECK_THAT((frame.physical_vector_to_model(displacement).cwiseProduct(frame.scale) - displacement).norm(),
               WithinAbs(0.0, 1e-12));
    const Vec3d tangent(1.0, 2.0, 0.0);
    const Vec3d normal(-2.0, 1.0, 0.0);
    CHECK_THAT(frame.normal_to_scene(normal).dot(frame.model_to_scene * tangent), WithinAbs(0.0, 1e-12));
    const Vec3d screen_drag(2.0, -3.0, 0.0);
    const Vec3d raw_drag = frame.scene_to_model * screen_drag;
    CHECK_THAT((frame.model_to_scene * (point + raw_drag) - frame.model_to_scene * point - screen_drag).norm(),
               WithinAbs(0.0, 1e-12));
}

TEST_CASE("Invalid study coordinate mappings remain finite and explicitly invalid", "[StrengthAnalysis]")
{
    Transform3d transform = Transform3d::Identity();
    transform.linear()(1, 1) = GENERATE(0.0, std::numeric_limits<double>::infinity());
    const StudyCoordinateFrame frame(transform);
    CHECK_FALSE(frame.valid);
    CHECK(frame.model_to_scene.allFinite());
    CHECK(frame.scene_to_model.allFinite());
    CHECK_THAT((frame.physical_vector_to_model(Vec3d::Ones()) - Vec3d::Ones()).norm(), WithinAbs(0.0, 0.0));
}

TEST_CASE("Prepare downward gravity preserves world direction and magnitude on transformed instances", "[StrengthAnalysis]")
{
    Transform3d transform = Transform3d::Identity();
    transform.rotate(Eigen::AngleAxisd(1.1, Vec3d(2.0, -1.0, 3.0).normalized()));
    const double mirror = GENERATE(1.0, -1.0);
    transform.scale(Vec3d(2.0 * mirror, 0.5, 3.0));
    const StudyCoordinateFrame frame(transform);
    REQUIRE(frame.valid);
    const Vec3d down(0.0, 0.0, -9.80665);
    const Vec3d object_gravity = frame.scene_vector_to_physical(down);
    CHECK_THAT(object_gravity.norm(), WithinRel(down.norm(), 1e-12));
    CHECK_THAT((frame.physical_vector_to_scene(object_gravity) - down).norm(), WithinAbs(0.0, 1e-12));
    // Raw inverse-transform components would incorrectly scale the acceleration.
    CHECK((object_gravity - frame.scene_to_model * down).norm() > 1.0);
}

TEST_CASE("Singular transforms have no valid print layer axis", "[StrengthAnalysis]")
{
    const int collapsed_axis = GENERATE(0, 1, 2, 3);
    CAPTURE(collapsed_axis);
    Transform3d transform = Transform3d::Identity();
    if (collapsed_axis == 3)
        transform.linear().setZero();
    else
        transform.linear()(collapsed_axis, collapsed_axis) = 0.0;

    // Collapsing X or Y still leaves a nonzero transpose-times-Z vector,
    // but the singular transform cannot define a valid plane normal.
    const Vec3d axis = print_layer_axis_for_transform(transform);
    REQUIRE(axis.allFinite());
    CHECK_THAT(axis.norm(), WithinAbs(0.0, 0.0));
}

TEST_CASE("Nonfinite transforms have no valid print layer axis", "[StrengthAnalysis]")
{
    const double invalid = GENERATE(std::numeric_limits<double>::quiet_NaN(),
                                   std::numeric_limits<double>::infinity(),
                                   -std::numeric_limits<double>::infinity());
    const int row = GENERATE(0, 1, 2, 3);
    const int column = GENERATE(0, 1, 2, 3);
    CAPTURE(invalid, row, column);
    Transform3d transform = Transform3d::Identity();
    transform.matrix()(row, column) = invalid;

    const Vec3d axis = print_layer_axis_for_transform(transform);
    REQUIRE(axis.allFinite());
    CHECK_THAT(axis.norm(), WithinAbs(0.0, 0.0));
}

TEST_CASE("Strength setup travels with its model object through 3MF", "[StrengthAnalysis][3mf]")
{
    Setup setup;
    setup.geometry_scale = Vec3d(2.0, 0.5, 1.5);
    setup.follow_prepare_orientation = GENERATE(true, false);
    setup.material = *find_builtin_material("abs_generic");
    setup.gravity.enabled = true;
    Load fixed;
    fixed.type = LoadType::Fixed;
    setup.loads.push_back(fixed);

    Model source;
    ModelObject *source_object = source.add_object();
    source_object->add_volume(TriangleMesh(its_make_cube(12.0, 8.0, 4.0)));
    source_object->add_instance();
    source_object->instances.front()->set_scaling_factor(setup.geometry_scale);
    const std::string encoded = serialize_setup_for_config(setup);
    REQUIRE(encoded.rfind("sa1:", 0) == 0);
    source_object->config.set_key_value("strength_analysis_setup", new ConfigOptionString(encoded));
    ModelVolume *modifier = source_object->add_volume(
        TriangleMesh(its_make_sphere(2.0, PI / 12.0)), ModelVolumeType::PARAMETER_MODIFIER);
    modifier->config.set_key_value("strength_analysis_modifier", new ConfigOptionBool(true));
    modifier->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(65.0));
    modifier->config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<Slic3r::InfillPattern>(ipGyroid));

    ScopedTemporaryFile file(".3mf");
    REQUIRE(store_3mf(file.string().c_str(), &source, nullptr, false));

    Model restored;
    DynamicPrintConfig config;
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    REQUIRE(load_3mf(file.string().c_str(), config, substitutions, &restored, false));
    REQUIRE(restored.objects.size() == 1);
    const auto *option = restored.objects.front()->config.get().option<ConfigOptionString>("strength_analysis_setup");
    REQUIRE(option != nullptr);
    CHECK(option->value == encoded);

    Setup decoded;
    REQUIRE(deserialize_setup_from_config(option->value, decoded));
    CHECK(decoded.material.key == "abs_generic");
    CHECK(decoded.follow_prepare_orientation == setup.follow_prepare_orientation);
    CHECK_THAT((decoded.geometry_scale - setup.geometry_scale).norm(), WithinAbs(0.0, 1e-12));
    REQUIRE(restored.objects.front()->instances.size() == 1);
    CHECK_THAT((restored.objects.front()->instances.front()->get_scaling_factor() - setup.geometry_scale).norm(),
               WithinAbs(0.0, 1e-6));
    CHECK(decoded.gravity.enabled);
    REQUIRE(decoded.loads.size() == 1);
    CHECK(decoded.loads.front().type == LoadType::Fixed);
    const auto restored_modifier = std::find_if(restored.objects.front()->volumes.begin(), restored.objects.front()->volumes.end(),
        [](const ModelVolume *volume) {
            const auto *marker = dynamic_cast<const ConfigOptionBool *>(
                volume->config.option("strength_analysis_modifier"));
            return marker != nullptr && marker->value;
        });
    REQUIRE(restored_modifier != restored.objects.front()->volumes.end());
    REQUIRE((*restored_modifier)->is_modifier());
    const auto *density = (*restored_modifier)->config.get().option<ConfigOptionPercent>("sparse_infill_density");
    const auto *pattern = (*restored_modifier)->config.get().option<ConfigOptionEnum<Slic3r::InfillPattern>>("sparse_infill_pattern");
    REQUIRE(density != nullptr);
    REQUIRE(pattern != nullptr);
    CHECK_THAT(density->value, WithinAbs(65.0, 1e-8));
    CHECK(pattern->value == ipGyroid);
}

TEST_CASE("Strength setup survives the Orca project 3MF path", "[StrengthAnalysis][3mf]")
{
    Setup setup;
    setup.material = *find_builtin_material("pc_generic");
    setup.criteria.minimum_safety_factor = 2.25;
    const std::string encoded = serialize_setup_for_config(setup);

    Model source;
    ModelObject *source_object = source.add_object();
    source_object->add_volume(TriangleMesh(its_make_cube(12.0, 8.0, 4.0)));
    source_object->add_instance();
    source_object->config.set_key_value("strength_analysis_setup", new ConfigOptionString(encoded));
    const auto source_mesh = source_object->raw_mesh().its;
    const auto source_result = synthetic_result(source_mesh);
    const auto preview = preview_dense_region(source_mesh, setup, source_result, 0.25);
    REQUIRE(preview.applicable());
    auto *modifier = source_object->add_volume(TriangleMesh(preview.modifier_mesh), ModelVolumeType::PARAMETER_MODIFIER, false);
    modifier->config.set_key_value("strength_analysis_modifier", new ConfigOptionBool(true));
    modifier->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(65.0));
    modifier->config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<Slic3r::InfillPattern>(ipGyroid));
    ScopedTemporaryDir backup_dir("orca_strength");
    source.set_backup_path(backup_dir.string());

    DynamicPrintConfig source_config = DynamicPrintConfig::full_print_config();
    auto *plate = new PlateData();
    plate->plate_index = 0;
    ScopedTemporaryFile file(".3mf");
    StoreParams params;
    params.path = file.string();
    params.model = &source;
    params.config = &source_config;
    params.plate_data_list.push_back(plate);
    params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
    REQUIRE(store_bbs_3mf(params));

    Model restored;
    ScopedTemporaryDir restored_backup_dir("orca_strength_restore");
    restored.set_backup_path(restored_backup_dir.string());
    DynamicPrintConfig restored_config;
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    PlateDataPtrs restored_plates;
    std::vector<Preset *> restored_presets;
    bool is_bambu = false;
    bool is_orca = false;
    Semver file_version;
    const bool loaded = load_bbs_3mf(file.string().c_str(), &restored_config, &substitutions, &restored,
        &restored_plates, &restored_presets, &is_bambu, &is_orca, &file_version, nullptr,
        LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
    REQUIRE(loaded);
    REQUIRE(restored.objects.size() == 1);
    const auto *option = restored.objects.front()->config.get().option<ConfigOptionString>("strength_analysis_setup");
    REQUIRE(option != nullptr);
    CHECK(option->value == encoded);
    Setup decoded;
    REQUIRE(deserialize_setup_from_config(option->value, decoded));
    CHECK(decoded.material.key == "pc_generic");
    CHECK_THAT(decoded.criteria.minimum_safety_factor, WithinRel(2.25, 1e-12));
    const auto restored_modifier = std::find_if(restored.objects.front()->volumes.begin(), restored.objects.front()->volumes.end(),
        [](const ModelVolume *volume) { return volume->is_modifier(); });
    REQUIRE(restored_modifier != restored.objects.front()->volumes.end());
    const auto &modifier_config = (*restored_modifier)->config.get();
    const auto *marker = modifier_config.option<ConfigOptionBool>("strength_analysis_modifier");
    const auto *density = modifier_config.option<ConfigOptionPercent>("sparse_infill_density");
    const auto *pattern = modifier_config.option<ConfigOptionEnum<Slic3r::InfillPattern>>("sparse_infill_pattern");
    REQUIRE(marker != nullptr);
    CHECK(marker->value);
    REQUIRE(density != nullptr);
    REQUIRE(pattern != nullptr);
    CHECK_THAT(density->value, WithinAbs(65.0, 1e-8));
    CHECK(pattern->value == ipGyroid);
    CHECK_THAT(std::abs(its_volume((*restored_modifier)->mesh().its)),
               WithinRel(double(std::abs(its_volume(preview.modifier_mesh))), 1e-6));

    release_PlateData_list(restored_plates);
    delete plate;
}

TEST_CASE("Material calibration scales directional properties without mutating defaults", "[StrengthAnalysis]")
{
    const Material baseline = *find_builtin_material("pla_generic");
    Material measured = baseline;
    measured.calibration.modulus_xy_scale = 1.25;
    measured.calibration.strength_z_scale = 2.0;
    measured.calibration.shear_scale = 0.8;
    const Material calibrated = measured.calibrated();

    CHECK_THAT(calibrated.elastic_modulus_xy_pa, WithinRel(baseline.elastic_modulus_xy_pa * 1.25, 1e-12));
    CHECK_THAT(calibrated.yield_strength_z_pa, WithinRel(baseline.yield_strength_z_pa * 2.0, 1e-12));
    CHECK_THAT(calibrated.shear_modulus_xy_pa, WithinRel(baseline.shear_modulus_xy_pa * 0.8, 1e-12));
    CHECK_THAT(calibrated.shear_strength_xy_pa, WithinRel(baseline.shear_strength_xy_pa * 0.8, 1e-12));
    CHECK_THAT(baseline.yield_strength_z_pa, WithinRel(25.0e6, 1e-12));
    CHECK(baseline.validate().empty());
}

TEST_CASE("Loads retain their requested resultant and gravity scales with density", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 20.0, 20.0);
    Setup setup = cube_setup(mesh);
    setup.gravity.enabled = true;
    const Result light = analyze(mesh, setup);
    REQUIRE(light.succeeded());
    CHECK_THAT(light.requested_resultant_force_n.x(), WithinRel(100.0, 1e-10));
    CHECK(light.requested_resultant_force_n.z() < 0.0);

    setup.material.density_kg_m3 *= 2.0;
    const Result heavy = analyze(mesh, setup);
    REQUIRE(heavy.succeeded());
    CHECK_THAT(heavy.estimated_mass_kg, WithinRel(light.estimated_mass_kg * 2.0, 1e-10));
    CHECK_THAT(heavy.requested_resultant_force_n.z(), WithinRel(light.requested_resultant_force_n.z() * 2.0, 1e-10));
}

TEST_CASE("Uniform geometry scaling preserves raw supports and follows fixed-force similarity", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 20.0, 20.0);
    Setup setup = cube_setup(mesh);
    setup.gravity.enabled = false;
    setup.geometry_scale = Vec3d::Ones();
    const auto supports = vertices_in_region(mesh, setup.loads.front().region, false);
    REQUIRE(supports.size() == 4);
    const Result baseline = analyze(mesh, setup);
    INFO(baseline.message);
    REQUIRE(baseline.succeeded());
    REQUIRE(baseline.maximum_von_mises_pa > 0.0);
    REQUIRE(baseline.maximum_displacement_m > 0.0);

    setup.geometry_scale = Vec3d::Constant(2.0);
    const Result scaled = analyze(mesh, setup);
    INFO(scaled.message);
    REQUIRE(scaled.succeeded());
    CHECK(vertices_in_region(mesh, setup.loads.front().region, false) == supports);
    CHECK_THAT((baseline.geometry_scale - Vec3d::Ones()).norm(), WithinAbs(0.0, 1e-12));
    CHECK_THAT((scaled.geometry_scale - setup.geometry_scale).norm(), WithinAbs(0.0, 1e-12));
    CHECK_THAT(scaled.effective_volume_m3, WithinRel(baseline.effective_volume_m3 * 8.0, 1e-10));
    CHECK_THAT(scaled.estimated_mass_kg, WithinRel(baseline.estimated_mass_kg * 8.0, 1e-10));
    CHECK_THAT((scaled.requested_resultant_force_n - baseline.requested_resultant_force_n).norm(), WithinAbs(0.0, 1e-10));
    CHECK_THAT(scaled.maximum_von_mises_pa, WithinRel(baseline.maximum_von_mises_pa / 4.0, 1e-6));
    CHECK_THAT(scaled.maximum_displacement_m, WithinRel(baseline.maximum_displacement_m / 2.0, 1e-6));
    REQUIRE(scaled.vertices.size() == mesh.vertices.size());
    REQUIRE(baseline.vertices.size() == mesh.vertices.size());
    for (size_t vertex = 0; vertex < mesh.vertices.size(); ++vertex) {
        CAPTURE(vertex);
        CHECK_THAT((scaled.vertices[vertex].position_mm - mesh.vertices[vertex].cast<double>()).norm(), WithinAbs(0.0, 1e-12));
        CHECK_THAT((scaled.vertices[vertex].displacement_m - baseline.vertices[vertex].displacement_m / 2.0).norm(),
                   WithinAbs(0.0, baseline.maximum_displacement_m * 1e-6));
    }
    // Fixed supports use a finite 1e10 stiffness penalty, not exact DOF elimination.
    // Check negligible relative motion, in addition to the per-vertex similarity above.
    for (size_t vertex : supports)
        CHECK(scaled.vertices[vertex].displacement_m.norm() < scaled.maximum_displacement_m * 1e-7);
}

TEST_CASE("Nonuniform geometry scaling matches physical mesh coordinates and transformed layer normals", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 12.0, 8.0);
    Setup setup = cube_setup(mesh);
    setup.geometry_scale = Vec3d(2.0, 0.5, 3.0);
    setup.print_layer_axis = Vec3d(1.0, 2.0, 3.0).normalized();
    setup.gravity.enabled = GENERATE(false, true);
    setup.gravity.acceleration_m_s2 = Vec3d(2.0, -3.0, -9.0);
    setup.loads.front().region.shape = RegionShape::Box;
    setup.loads.front().region.size_mm = Vec3d(0.1, 13.0, 9.0);
    setup.loads.back().direction = Vec3d(2.0, -1.0, 3.0);
    setup.loads.back().region.shape = RegionShape::Surface;
    for (size_t triangle = 0; triangle < mesh.indices.size(); ++triangle) {
        bool right_face = true;
        for (int corner = 0; corner < 3; ++corner)
            right_face = right_face && std::abs(mesh.vertices[mesh.indices[triangle][corner]].x() - 20.0f) < 1e-6f;
        if (right_face)
            setup.loads.back().region.surface_triangles.push_back(triangle);
    }
    REQUIRE(setup.loads.back().region.surface_triangles.size() == 2);

    indexed_triangle_set physical_mesh = mesh;
    for (Vec3f &vertex : physical_mesh.vertices)
        vertex = vertex.cast<double>().cwiseProduct(setup.geometry_scale).cast<float>().eval();
    Setup physical_setup = setup;
    physical_setup.geometry_scale = Vec3d::Ones();
    physical_setup.print_layer_axis = setup.print_layer_axis.cwiseQuotient(setup.geometry_scale).normalized();
    for (Load &load : physical_setup.loads) {
        load.region.center_mm = load.region.center_mm.cwiseProduct(setup.geometry_scale).eval();
        load.region.size_mm = load.region.size_mm.cwiseProduct(setup.geometry_scale).eval();
    }
    for (size_t load = 0; load < setup.loads.size(); ++load) {
        const auto selected = vertices_in_region(mesh, setup.loads[load].region, false);
        REQUIRE(selected.size() == 4);
        CHECK(selected == vertices_in_region(physical_mesh, physical_setup.loads[load].region, false));
    }

    const Result scaled = analyze(mesh, setup);
    const Result physical = analyze(physical_mesh, physical_setup);
    INFO(scaled.message);
    INFO(physical.message);
    REQUIRE(scaled.succeeded());
    REQUIRE(physical.succeeded());
    REQUIRE(physical.maximum_displacement_m > 0.0);
    REQUIRE(physical.maximum_von_mises_pa > 0.0);
    CHECK_THAT((scaled.geometry_scale - setup.geometry_scale).norm(), WithinAbs(0.0, 1e-12));
    CHECK_THAT(scaled.effective_volume_m3, WithinRel(physical.effective_volume_m3, 1e-10));
    CHECK_THAT(scaled.estimated_mass_kg, WithinRel(physical.estimated_mass_kg, 1e-10));
    Vec3d expected_force = setup.loads.back().direction.normalized() * setup.loads.back().magnitude_n;
    if (setup.gravity.enabled)
        expected_force += scaled.estimated_mass_kg * setup.gravity.acceleration_m_s2;
    CHECK_THAT((scaled.requested_resultant_force_n - expected_force).norm(), WithinAbs(0.0, 1e-10));
    CHECK_THAT((scaled.requested_resultant_force_n - physical.requested_resultant_force_n).norm(), WithinAbs(0.0, 1e-10));
    CHECK_THAT(scaled.maximum_displacement_m, WithinRel(physical.maximum_displacement_m, 1e-6));
    CHECK_THAT(scaled.maximum_von_mises_pa, WithinRel(physical.maximum_von_mises_pa, 1e-6));
    CHECK_THAT(scaled.minimum_safety_factor, WithinRel(physical.minimum_safety_factor, 1e-6));
    REQUIRE(scaled.vertices.size() == mesh.vertices.size());
    REQUIRE(physical.vertices.size() == mesh.vertices.size());
    for (size_t vertex = 0; vertex < mesh.vertices.size(); ++vertex) {
        CAPTURE(vertex);
        const VertexResult &actual = scaled.vertices[vertex];
        const VertexResult &expected = physical.vertices[vertex];
        CHECK_THAT((actual.position_mm - mesh.vertices[vertex].cast<double>()).norm(), WithinAbs(0.0, 1e-12));
        CHECK_THAT((expected.position_mm - physical_mesh.vertices[vertex].cast<double>()).norm(), WithinAbs(0.0, 1e-12));
        // Displacements already use physical meters; do not multiply or divide them by geometry_scale.
        CHECK_THAT((actual.displacement_m - expected.displacement_m).norm(),
                   WithinAbs(0.0, physical.maximum_displacement_m * 1e-6));
        CHECK_THAT((actual.normal_stress_pa - expected.normal_stress_pa).norm(),
                   WithinAbs(0.0, physical.maximum_von_mises_pa * 1e-6));
        CHECK_THAT((actual.shear_stress_pa - expected.shear_stress_pa).norm(),
                   WithinAbs(0.0, physical.maximum_von_mises_pa * 1e-6));
    }
}

TEST_CASE("Uniform geometry scaling multiplies gravity mass and force by the volume factor", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 20.0, 20.0);
    Setup setup = cube_setup(mesh);
    setup.loads.back().active = false;
    setup.gravity.enabled = true;
    setup.gravity.acceleration_m_s2 = Vec3d(1.0, -2.0, -9.0);
    setup.geometry_scale = Vec3d::Ones();
    const Result baseline = analyze(mesh, setup);
    REQUIRE(baseline.succeeded());
    REQUIRE(baseline.estimated_mass_kg > 0.0);
    setup.geometry_scale = Vec3d::Constant(2.0);
    const Result scaled = analyze(mesh, setup);
    REQUIRE(scaled.succeeded());
    CHECK_THAT(scaled.estimated_mass_kg, WithinRel(baseline.estimated_mass_kg * 8.0, 1e-10));
    for (int axis = 0; axis < 3; ++axis) {
        CAPTURE(axis);
        CHECK_THAT(scaled.requested_resultant_force_n[axis], WithinRel(baseline.requested_resultant_force_n[axis] * 8.0, 1e-10));
        CHECK_THAT(scaled.requested_resultant_force_n[axis],
                   WithinRel(scaled.estimated_mass_kg * setup.gravity.acceleration_m_s2[axis], 1e-10));
    }
}

TEST_CASE("Geometry scale rejects zero negative and nonfinite components", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 20.0, 20.0);
    Setup setup = cube_setup(mesh);
    REQUIRE(validate(mesh, setup).empty());
    const int axis = GENERATE(0, 1, 2);
    const double invalid_scale = GENERATE(0.0, -1.0, std::numeric_limits<double>::infinity(),
                                          -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN());
    CAPTURE(axis, invalid_scale);
    setup.geometry_scale[axis] = invalid_scale;
    CHECK_FALSE(validate(mesh, setup).empty());
    const Result result = analyze(mesh, setup);
    CHECK(result.status == AnalysisStatus::InvalidInput);
    CHECK_FALSE(result.message.empty());
}

TEST_CASE("Geometry scale survives JSON and defaults to ones for older setups", "[StrengthAnalysis]")
{
    Setup setup;
    CHECK_THAT((setup.geometry_scale - Vec3d::Ones()).norm(), WithinAbs(0.0, 1e-12));
    CHECK_THAT((Result{}.geometry_scale - Vec3d::Ones()).norm(), WithinAbs(0.0, 1e-12));
    setup.geometry_scale = Vec3d(2.0, 0.5, 3.0);
    auto json = nlohmann::json::parse(serialize_setup(setup));
    REQUIRE(json.contains("geometry_scale"));
    REQUIRE(json.at("geometry_scale").is_array());
    REQUIRE(json.at("geometry_scale").size() == 3);
    for (int axis = 0; axis < 3; ++axis)
        CHECK_THAT(json.at("geometry_scale").at(axis).get<double>(), WithinRel(setup.geometry_scale[axis], 1e-12));
    Setup decoded;
    REQUIRE(deserialize_setup(json.dump(), decoded));
    CHECK_THAT((decoded.geometry_scale - setup.geometry_scale).norm(), WithinAbs(0.0, 1e-12));
    REQUIRE(deserialize_setup_from_config(serialize_setup_for_config(setup), decoded));
    CHECK_THAT((decoded.geometry_scale - setup.geometry_scale).norm(), WithinAbs(0.0, 1e-12));

    json.erase("geometry_scale");
    // Decoding legacy data must reset a reused destination's previous nonunit scale.
    REQUIRE(deserialize_setup(json.dump(), decoded));
    CHECK_THAT((decoded.geometry_scale - Vec3d::Ones()).norm(), WithinAbs(0.0, 1e-12));
}

TEST_CASE("Dense-region preview scales physical volume and mass while retaining raw cells", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 20.0, 20.0);
    Setup setup;
    setup.infill.background_density = 0.20;
    setup.infill.dense_density = 0.65;
    const Result baseline = synthetic_result(mesh);
    Result scaled = baseline;
    scaled.geometry_scale = Vec3d::Constant(2.0);
    scaled.estimated_mass_kg *= 8.0;
    // A subsequent setup change must not replace the scale recorded by the solved result.
    setup.geometry_scale = Vec3d::Constant(3.0);
    const auto baseline_profile = build_dense_region_preview_profile(mesh, baseline);
    const auto scaled_profile = build_dense_region_preview_profile(mesh, scaled);
    REQUIRE(baseline_profile.available);
    REQUIRE(scaled_profile.available);
    REQUIRE(scaled_profile.cells.size() == baseline_profile.cells.size());
    for (size_t cell = 0; cell < baseline_profile.cells.size(); ++cell) {
        CHECK(scaled_profile.cells[cell].grid_index == baseline_profile.cells[cell].grid_index);
        CHECK_THAT(scaled_profile.cells[cell].volume_mm3, WithinRel(baseline_profile.cells[cell].volume_mm3, 1e-12));
    }
    const auto original = preview_dense_region(mesh, setup, baseline, 0.25, &baseline_profile);
    const auto enlarged = preview_dense_region(mesh, setup, scaled, 0.25, &scaled_profile);
    REQUIRE(original.applicable());
    REQUIRE(enlarged.applicable());
    REQUIRE(original.estimated_volume_m3 > 0.0);
    REQUIRE(original.estimated_added_mass_kg > 0.0);
    CHECK_THAT(original.estimated_volume_m3,
               WithinRel(original.estimated_volume_fraction * std::abs(its_volume(mesh)) * 1e-9, 1e-6));
    CHECK_THAT(enlarged.estimated_volume_m3, WithinRel(original.estimated_volume_m3 * 8.0, 1e-10));
    CHECK_THAT(enlarged.estimated_added_mass_kg, WithinRel(original.estimated_added_mass_kg * 8.0, 1e-10));
    CHECK_THAT(enlarged.estimated_total_mass_kg, WithinRel(original.estimated_total_mass_kg * 8.0, 1e-10));
    CHECK_THAT(enlarged.estimated_volume_fraction, WithinRel(original.estimated_volume_fraction, 1e-12));
    CHECK(enlarged.selected_cell_count == original.selected_cell_count);
    CHECK(enlarged.affected_vertices == original.affected_vertices);
    CHECK_THAT((enlarged.region.center_mm - original.region.center_mm).norm(), WithinAbs(0.0, 1e-12));
    CHECK_THAT((enlarged.region.size_mm - original.region.size_mm).norm(), WithinAbs(0.0, 1e-12));
    REQUIRE(enlarged.modifier_mesh.vertices.size() == original.modifier_mesh.vertices.size());
    REQUIRE(enlarged.modifier_mesh.indices.size() == original.modifier_mesh.indices.size());
    for (size_t vertex = 0; vertex < original.modifier_mesh.vertices.size(); ++vertex)
        CHECK_THAT(double((enlarged.modifier_mesh.vertices[vertex] - original.modifier_mesh.vertices[vertex]).norm()),
                   WithinAbs(0.0, 1e-12));
    for (size_t triangle = 0; triangle < original.modifier_mesh.indices.size(); ++triangle)
        CHECK((enlarged.modifier_mesh.indices[triangle] - original.modifier_mesh.indices[triangle]).squaredNorm() == 0);
}

TEST_CASE("Dense stress interpolation measures distance in physical scaled coordinates", "[StrengthAnalysis]")
{
    const auto mesh = its_make_cube(20.0, 20.0, 20.0);
    Result result = synthetic_result(mesh);
    result.geometry_scale = Vec3d(3.0, 0.5, 2.0);
    const auto profile = build_dense_region_preview_profile(mesh, result);
    REQUIRE(profile.available);
    REQUIRE(mesh.vertices.size() == 8);
    REQUIRE(profile.cells.size() > 2);
    const size_t nx = profile.grid_planes_mm[0].size() - 1;
    const size_t ny = profile.grid_planes_mm[1].size() - 1;
    bool distinguishes_unscaled_distance = false;
    for (size_t rank : {size_t(0), profile.cells.size() / 2, profile.cells.size() - 1}) {
        const auto &cell = profile.cells[rank];
        const std::array<size_t, 3> cell_index{cell.grid_index % nx, (cell.grid_index / nx) % ny, cell.grid_index / (nx * ny)};
        Vec3d center;
        for (int axis = 0; axis < 3; ++axis)
            center[axis] = 0.5 * (profile.grid_planes_mm[axis][cell_index[axis]] +
                                  profile.grid_planes_mm[axis][cell_index[axis] + 1]);
        double total_weight = 0.0, weighted_stress = 0.0;
        double unscaled_weights = 0.0, unscaled_stress = 0.0;
        for (size_t vertex = 0; vertex < mesh.vertices.size(); ++vertex) {
            const double distance_squared = (mesh.vertices[vertex].cast<double>() - center)
                .cwiseProduct(result.geometry_scale).squaredNorm();
            const double weight = 1.0 / std::max(distance_squared, 1e-12);
            total_weight += weight;
            weighted_stress += weight * result.vertices[vertex].von_mises_pa;
            const double raw_weight = 1.0 / std::max((mesh.vertices[vertex].cast<double>() - center).squaredNorm(), 1e-12);
            unscaled_weights += raw_weight;
            unscaled_stress += raw_weight * result.vertices[vertex].von_mises_pa;
        }
        // The profile uses integrated cell centroids. Compare the analytic cube midpoint
        // to one part per million and require a signal larger than that tolerance.
        CHECK_THAT(cell.stress_pa, WithinRel(weighted_stress / total_weight, 1e-6));
        distinguishes_unscaled_distance = distinguishes_unscaled_distance ||
            std::abs(unscaled_stress / unscaled_weights - weighted_stress / total_weight) > cell.stress_pa * 1e-4;
    }
    CHECK(distinguishes_unscaled_distance);
}

TEST_CASE("Every supported force type produces its intended equivalent-static resultant", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 20.0, 20.0);
    const std::array<LoadType, 5> force_types{LoadType::LocalForce, LoadType::DirectionalForce,
        LoadType::BearingForce, LoadType::ImpactForce, LoadType::GlobalForce};
    for (LoadType type : force_types) {
        Setup setup = cube_setup(mesh);
        setup.loads.back().type = type;
        setup.loads.back().impact_factor = 2.5;
        const Result result = analyze(mesh, setup);
        REQUIRE(result.succeeded());
        const double expected = type == LoadType::ImpactForce ? 250.0 : 100.0;
        CHECK_THAT(result.requested_resultant_force_n.x(), WithinRel(expected, 1e-10));
        CHECK_THAT(result.requested_resultant_force_n.y(), WithinAbs(0.0, 1e-12));
        CHECK_THAT(result.requested_resultant_force_n.z(), WithinAbs(0.0, 1e-12));
    }
}

TEST_CASE("A constrained cube produces finite coupled strength results", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 20.0, 20.0);
    const Result result = analyze(mesh, cube_setup(mesh));

    REQUIRE(result.succeeded());
    REQUIRE(result.vertices.size() == mesh.vertices.size());
    CHECK(result.maximum_displacement_m > 0.0);
    CHECK(std::isfinite(result.maximum_displacement_m));
    CHECK(result.maximum_von_mises_pa > 0.0);
    CHECK(std::isfinite(result.maximum_von_mises_pa));
    CHECK(result.minimum_safety_factor > 0.0);
    CHECK(std::isfinite(result.minimum_safety_factor));
    CHECK_THAT(result.governing_target_safety_factor, WithinRel(1.5, 1e-12));
    CHECK(result.displacement_target_met);
    CHECK(result.dense_region.available);
    CHECK_FALSE(result.infill_comparisons.empty());
    CHECK_FALSE(result.mass_strength_curve.empty());
    CHECK_FALSE(result.orientation_recommendations.empty());
    CHECK_FALSE(result.print_settings_candidates.empty());
    for (const VertexResult &vertex : result.vertices) {
        CHECK(vertex.displacement_m.allFinite());
        CHECK(vertex.normal_stress_pa.allFinite());
        CHECK(vertex.shear_stress_pa.allFinite());
        CHECK(std::isfinite(vertex.von_mises_pa));
        CHECK(std::isfinite(vertex.maximum_shear_pa));
    }
}

TEST_CASE("Print direction changes anisotropic response", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 20.0, 20.0);
    Setup xy = cube_setup(mesh);
    xy.print_layer_axis = Vec3d::UnitZ();
    const Result strong = analyze(mesh, xy);
    REQUIRE(strong.succeeded());

    Setup z = xy;
    z.print_layer_axis = Vec3d::UnitX();
    const Result weak = analyze(mesh, z);
    REQUIRE(weak.succeeded());
    CHECK(weak.maximum_displacement_m > strong.maximum_displacement_m);
}

TEST_CASE("Safety-factor basis and calibration govern allowable strength", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 20.0, 20.0);
    Setup setup = cube_setup(mesh);
    setup.loads.back().strength_basis = StrengthBasis::Yield;
    const Result yield = analyze(mesh, setup);
    REQUIRE(yield.succeeded());

    setup.loads.back().strength_basis = StrengthBasis::Ultimate;
    const Result ultimate = analyze(mesh, setup);
    REQUIRE(ultimate.succeeded());
    CHECK(ultimate.minimum_safety_factor > yield.minimum_safety_factor);

    setup.loads.back().strength_basis = StrengthBasis::CalibratedYield;
    setup.material.calibration.strength_xy_scale = 0.5;
    setup.material.calibration.strength_z_scale = 0.5;
    const Result calibrated = analyze(mesh, setup);
    REQUIRE(calibrated.succeeded());
    CHECK_THAT(calibrated.minimum_safety_factor, WithinRel(yield.minimum_safety_factor * 0.5, 1e-8));
}

TEST_CASE("Per-load targets and displacement limits constrain recommendations", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 20.0, 20.0);
    Setup setup = cube_setup(mesh);
    const Result baseline = analyze(mesh, setup);
    REQUIRE(baseline.succeeded());

    setup.criteria.minimum_safety_factor = 1.1;
    setup.loads.back().target_safety_factor = baseline.minimum_safety_factor * 1.1;
    const Result targeted = analyze(mesh, setup);
    REQUIRE(targeted.succeeded());
    CHECK_THAT(targeted.governing_target_safety_factor, WithinRel(setup.loads.back().target_safety_factor, 1e-10));
    CHECK_FALSE(targeted.safety_factor_target_met);

    Setup optimization;
    optimization.criteria.minimum_safety_factor = 2.0;
    optimization.criteria.maximum_displacement_mm = 0.5;
    const auto candidates = recommend_print_settings(1e-5, optimization, 5.0, 1.0);
    REQUIRE_FALSE(candidates.empty());
    REQUIRE(std::any_of(candidates.begin(), candidates.end(), [](const PrintSettingsCandidate &candidate) {
        return candidate.feasible;
    }));
    for (const PrintSettingsCandidate &candidate : candidates) {
        if (!candidate.feasible)
            continue;
        CHECK(candidate.predicted_safety_factor + 1e-9 >= optimization.criteria.minimum_safety_factor);
        CHECK(candidate.predicted_displacement_mm <= optimization.criteria.maximum_displacement_mm + 1e-9);
    }
}

TEST_CASE("Dense recommendations respect full preserve-region overlap", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 20.0, 20.0);
    Setup setup = cube_setup(mesh);
    const Result initial = analyze(mesh, setup);
    REQUIRE(initial.succeeded());
    REQUIRE(initial.dense_region.available);

    SphericalRegion preserve;
    preserve.radius_mm = 1.0;
    preserve.center_mm = initial.dense_region.region.center_mm +
        Vec3d(initial.dense_region.region.radius_mm + 0.5, 0.0, 0.0);
    REQUIRE_FALSE(preserve.contains(initial.dense_region.region.center_mm));
    setup.preserve_regions.push_back(preserve);
    const Result preserved = analyze(mesh, setup);
    REQUIRE(preserved.succeeded());
    CHECK_FALSE(preserved.dense_region.available);
}

TEST_CASE("Dense-region preview maps volume endpoints to baseline and fully dense estimates", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 20.0, 20.0);
    const Setup setup = cube_setup(mesh);
    const Result result = analyze(mesh, setup);
    REQUIRE(result.succeeded());
    const DenseRegionPreviewProfile profile = build_dense_region_preview_profile(mesh, result);
    REQUIRE(profile.available);

    const DenseRegionPreview zero = preview_dense_region(mesh, setup, result, 0.0, &profile);
    REQUIRE(zero.available);
    CHECK_FALSE(zero.applicable());
    CHECK(zero.affected_vertices.empty());
    CHECK_THAT(zero.estimated_volume_fraction, WithinAbs(0.0, 1e-12));
    CHECK_THAT(zero.estimated_total_mass_kg, WithinRel(result.estimated_mass_kg, 1e-12));
    CHECK_THAT(zero.predicted_minimum_safety_factor, WithinRel(result.minimum_safety_factor, 1e-12));

    const DenseRegionPreview full = preview_dense_region(mesh, setup, result, 1.0, &profile);
    REQUIRE(full.applicable());
    CHECK(full.affected_vertices.size() == mesh.vertices.size());
    CHECK_THAT(full.estimated_volume_fraction, WithinRel(1.0, 1e-12));
    CHECK(full.estimated_total_mass_kg >= result.estimated_mass_kg);
    CHECK(full.predicted_minimum_safety_factor >= result.minimum_safety_factor);
    CHECK(full.predicted_maximum_displacement_mm <= result.maximum_displacement_m * 1000.0 + 1e-12);

    const DenseRegionPreview below = preview_dense_region(mesh, setup, result, -5.0, &profile);
    const DenseRegionPreview above = preview_dense_region(mesh, setup, result, 5.0, &profile);
    CHECK_THAT(below.target_volume_fraction, WithinAbs(0.0, 1e-12));
    CHECK_THAT(above.target_volume_fraction, WithinRel(1.0, 1e-12));
    CHECK_THAT(above.estimated_total_mass_kg, WithinRel(full.estimated_total_mass_kg, 1e-12));
}

TEST_CASE("Dense-region preview grows monotonically from the solved stress hotspot", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 20.0, 20.0);
    const Setup setup = cube_setup(mesh);
    const Result result = analyze(mesh, setup);
    REQUIRE(result.succeeded());
    const DenseRegionPreviewProfile profile = build_dense_region_preview_profile(mesh, result);
    REQUIRE(profile.available);

    double previous_radius = 0.0;
    double previous_volume = 0.0;
    double previous_mass = result.estimated_mass_kg;
    double previous_safety_factor = result.minimum_safety_factor;
    for (double target : {0.01, 0.10, 0.25, 0.50, 0.75, 1.0}) {
        const DenseRegionPreview first = preview_dense_region(mesh, setup, result, target, &profile);
        const DenseRegionPreview second = preview_dense_region(mesh, setup, result, target, &profile);
        REQUIRE(first.available);
        CHECK(first.region.radius_mm + 1e-12 >= previous_radius);
        CHECK(first.estimated_volume_fraction + 1e-12 >= previous_volume);
        CHECK(first.estimated_total_mass_kg + 1e-12 >= previous_mass);
        CHECK(first.predicted_minimum_safety_factor + 1e-12 >= previous_safety_factor);
        CHECK_THAT(first.region.radius_mm, WithinAbs(second.region.radius_mm, 1e-12));
        CHECK_THAT(first.estimated_volume_fraction, WithinAbs(second.estimated_volume_fraction, 1e-12));
        CHECK(first.affected_vertices == second.affected_vertices);
        CHECK(std::binary_search(first.affected_vertices.begin(), first.affected_vertices.end(), result.maximum_stress_vertex));
        previous_radius = first.region.radius_mm;
        previous_volume = first.estimated_volume_fraction;
        previous_mass = first.estimated_total_mass_kg;
        previous_safety_factor = first.predicted_minimum_safety_factor;
    }
}

TEST_CASE("Dense-region profile integrates disconnected solids without filling the gap", "[StrengthAnalysis]")
{
    indexed_triangle_set mesh = its_make_cube(10.0, 10.0, 10.0);
    indexed_triangle_set second = its_make_cube(10.0, 10.0, 10.0);
    its_transform(second, identity3f().translate(Vec3f(40.0f, 0.0f, 0.0f)));
    its_merge(mesh, second);
    const Result result = synthetic_result(mesh, 0);

    const DenseRegionPreviewProfile profile = build_dense_region_preview_profile(mesh, result);
    REQUIRE(profile.available);
    CHECK_THAT(profile.solid_volume_m3, WithinRel(2.0e-6, 1e-6));
    CHECK_THAT(profile.sampled_volume_mm3, WithinRel(2000.0, 1e-6));

    const Setup setup;
    const DenseRegionPreview half = preview_dense_region(mesh, setup, result, 0.5, &profile);
    REQUIRE(half.available);
    CHECK_THAT(half.estimated_volume_fraction, WithinAbs(0.5, 0.02));
    CHECK(half.region.radius_mm < 30.0);
}

TEST_CASE("Dense-region preview withholds response predictions when self-weight changes", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 20.0, 20.0);
    Setup setup = cube_setup(mesh);
    const Result result = analyze(mesh, setup);
    REQUIRE(result.succeeded());
    const DenseRegionPreviewProfile profile = build_dense_region_preview_profile(mesh, result);
    REQUIRE(profile.available);

    const DenseRegionPreview without_gravity = preview_dense_region(mesh, setup, result, 0.5, &profile);
    REQUIRE(without_gravity.applicable());
    REQUIRE(without_gravity.estimated_added_mass_kg > 0.0);
    Setup with_gravity = setup;
    with_gravity.gravity.enabled = true;
    const DenseRegionPreview gravity = preview_dense_region(mesh, with_gravity, result, 0.5, &profile);
    REQUIRE(gravity.applicable());
    CHECK(gravity.estimated_total_mass_kg > result.estimated_mass_kg);
    CHECK_FALSE(gravity.response_estimate_available);
    CHECK(std::isnan(gravity.predicted_minimum_safety_factor));
    CHECK(std::isnan(gravity.predicted_maximum_displacement_mm));
    const auto baseline_preview = preview_dense_region(mesh, with_gravity, result, 0.0, &profile);
    CHECK(baseline_preview.response_estimate_available);
    CHECK(gravity.warning.find("self-weight") != std::string::npos);
}

TEST_CASE("Dense-region preview fails closed and never crosses a preserve region", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 20.0, 20.0);
    Setup setup = cube_setup(mesh);
    const Result baseline = analyze(mesh, setup);
    REQUIRE(baseline.succeeded());
    const DenseRegionPreviewProfile profile = build_dense_region_preview_profile(mesh, baseline);
    REQUIRE(profile.available);

    const Vec3d hotspot = baseline.vertices[baseline.maximum_stress_vertex].position_mm;
    SphericalRegion nearby_preserve;
    nearby_preserve.center_mm = bounding_box(mesh).center().cast<double>();
    nearby_preserve.radius_mm = 5.0;
    Setup limited_setup = setup;
    limited_setup.preserve_regions.push_back(nearby_preserve);
    const DenseRegionPreview limited = preview_dense_region(mesh, limited_setup, baseline, 1.0, &profile);
    REQUIRE(limited.available);
    CHECK_FALSE(limited.overlaps_preserve);
    for (const Vec3f &vertex : limited.modifier_mesh.vertices)
        CHECK_FALSE(nearby_preserve.contains(vertex.cast<double>()));
    CHECK(limited.estimated_volume_fraction < 1.0);
    CHECK_FALSE(limited.warning.empty());

    SphericalRegion preserve;
    preserve.center_mm = hotspot;
    preserve.radius_mm = 2.0;
    preserve.whole_model = true;
    setup.preserve_regions.push_back(preserve);
    const DenseRegionPreview blocked = preview_dense_region(mesh, setup, baseline, 0.5, &profile);
    REQUIRE(blocked.available);
    CHECK_FALSE(blocked.applicable());
    CHECK(blocked.affected_vertices.empty());
    CHECK_FALSE(blocked.warning.empty());

    Result mismatched = baseline;
    mismatched.vertices.pop_back();
    const DenseRegionPreview invalid = preview_dense_region(mesh, setup, mismatched, 0.5, &profile);
    CHECK_FALSE(invalid.available);
    CHECK_FALSE(invalid.warning.empty());

    const DenseRegionPreview non_finite = preview_dense_region(
        mesh, setup, baseline, std::numeric_limits<double>::quiet_NaN(), &profile);
    CHECK_FALSE(non_finite.available);
    CHECK_FALSE(non_finite.warning.empty());
}

TEST_CASE("Dense-region mesh excludes box cylinder and selected-face preserve regions", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 20.0, 20.0);
    const Setup base_setup = cube_setup(mesh);
    const Result result = analyze(mesh, base_setup);
    REQUIRE(result.succeeded());
    const DenseRegionPreviewProfile profile = build_dense_region_preview_profile(mesh, result);
    REQUIRE(profile.available);
    const Vec3d hotspot = profile.hotspot_center_mm;

    SphericalRegion box;
    box.shape = RegionShape::Box;
    box.center_mm = bounding_box(mesh).center().cast<double>();
    box.size_mm = Vec3d(10.0, 2.0, 2.0);
    Setup box_setup = base_setup;
    box_setup.preserve_regions.push_back(box);
    const DenseRegionPreview box_limited = preview_dense_region(mesh, box_setup, result, 1.0, &profile);
    REQUIRE(box_limited.available);
    CHECK(box_limited.estimated_volume_fraction < 1.0);
    for (const Vec3f &vertex : box_limited.modifier_mesh.vertices)
        CHECK_FALSE(box.contains(vertex.cast<double>()));

    SphericalRegion cylinder;
    cylinder.shape = RegionShape::Cylinder;
    cylinder.center_mm = bounding_box(mesh).center().cast<double>();
    cylinder.axis = Vec3d(1.0, 1.0, 1.0).normalized();
    cylinder.radius_mm = 5.0;
    cylinder.size_mm.z() = 4.0;
    Setup cylinder_setup = base_setup;
    cylinder_setup.preserve_regions.push_back(cylinder);
    const DenseRegionPreview cylinder_limited = preview_dense_region(mesh, cylinder_setup, result, 1.0, &profile);
    REQUIRE(cylinder_limited.available);
    CHECK(cylinder_limited.estimated_volume_fraction < 1.0);
    for (const Vec3f &vertex : cylinder_limited.modifier_mesh.vertices)
        CHECK_FALSE(cylinder.contains(vertex.cast<double>()));

    const auto patches = group_coplanar_surfaces(mesh);
    const auto opposite = std::max_element(patches.begin(), patches.end(), [&hotspot](const SurfacePatch &lhs,
                                                                                     const SurfacePatch &rhs) {
        return (lhs.centroid_mm - hotspot).norm() < (rhs.centroid_mm - hotspot).norm();
    });
    REQUIRE(opposite != patches.end());
    SphericalRegion surface;
    surface.shape = RegionShape::Surface;
    surface.surface_triangles = opposite->triangles;
    Setup surface_setup = base_setup;
    surface_setup.preserve_regions.push_back(surface);
    const DenseRegionPreview surface_limited = preview_dense_region(mesh, surface_setup, result, 1.0, &profile);
    REQUIRE(surface_limited.available);
    CHECK(surface_limited.estimated_volume_fraction < 1.0);
    for (const Vec3f &vertex : surface_limited.modifier_mesh.vertices)
        CHECK(std::abs((vertex.cast<double>() - opposite->centroid_mm).dot(opposite->normal)) > 1e-5);
}

TEST_CASE("Reinforcement selects separate high-stress bodies before a low-stress body", "[StrengthAnalysis]")
{
    indexed_triangle_set mesh;
    for (float x : {0.0f, 15.0f, 30.0f}) {
        auto body = its_make_cube(5.0, 5.0, 5.0);
        its_transform(body, identity3f().translate(Vec3f(x, 0.0f, 0.0f)));
        its_merge(mesh, body);
    }
    Result result = synthetic_result(mesh);
    for (auto &vertex : result.vertices)
        vertex.von_mises_pa = vertex.position_mm.x() < 10.0 ? 100e6 : vertex.position_mm.x() > 25.0 ? 80e6 : 1e6;
    const auto profile = build_dense_region_preview_profile(mesh, result);
    INFO(profile.warning);
    REQUIRE(profile.available);
    const auto preview = preview_dense_region(mesh, Setup{}, result, 0.5, &profile);
    REQUIRE(preview.applicable());
    REQUIRE_FALSE(preview.modifier_mesh.empty());
    bool left = false, right = false;
    for (const Vec3f &vertex : preview.modifier_mesh.vertices) {
        left = left || vertex.x() <= 5.0f;
        right = right || vertex.x() >= 30.0f;
        const bool in_low_stress_body = vertex.x() > 10.0f && vertex.x() < 25.0f;
        CHECK_FALSE(in_low_stress_body);
    }
    CHECK(left);
    CHECK(right);
    CHECK(preview.equivalent_stress_threshold > 0.79);
    CHECK_THAT(preview.estimated_volume_fraction, WithinAbs(0.5, 0.002));
    CHECK_THAT(sliced_dense_volume(mesh, preview, profile), WithinRel(preview.estimated_volume_m3 * 1e9, 1e-5));
}

TEST_CASE("Stress threshold maps to eligible model volume after preserving features", "[StrengthAnalysis]")
{
    indexed_triangle_set mesh;
    for (float x : {0.0f, 15.0f, 30.0f}) {
        auto body = its_make_cube(5.0, 5.0, 5.0);
        its_transform(body, identity3f().translate(Vec3f(x, 0.0f, 0.0f)));
        its_merge(mesh, body);
    }
    Result result = synthetic_result(mesh);
    for (auto &vertex : result.vertices)
        vertex.von_mises_pa = vertex.position_mm.x() < 10.0 ? 100e6 : vertex.position_mm.x() > 25.0 ? 80e6 : 1e6;
    const auto profile = build_dense_region_preview_profile(mesh, result);
    REQUIRE(profile.available);
    Setup setup;
    const auto threshold = preview_dense_region(mesh, setup, result, 1.0, &profile, true, 0.7);
    REQUIRE(threshold.applicable());
    CHECK_THAT(threshold.estimated_volume_fraction, WithinAbs(2.0 / 3.0, 1e-6));
    CHECK(threshold.equivalent_stress_threshold >= 0.7);
    CHECK_THAT(sliced_dense_volume(mesh, threshold, profile), WithinRel(threshold.estimated_volume_m3 * 1e9, 1e-5));

    SphericalRegion preserve;
    preserve.shape = RegionShape::Box;
    preserve.center_mm = Vec3d(2.5, 2.5, 2.5);
    preserve.size_mm = Vec3d::Constant(6.0);
    setup.preserve_regions.push_back(preserve);
    const auto limited = preview_dense_region(mesh, setup, result, 1.0, &profile, false, 0.7);
    REQUIRE(limited.applicable());
    CHECK(limited.modifier_mesh.empty());
    CHECK_THAT(limited.estimated_volume_fraction, WithinAbs(1.0 / 3.0, 1e-6));
    const auto sized = preview_dense_region(mesh, setup, result, limited.estimated_volume_fraction, &profile);
    REQUIRE(sized.applicable());
    CHECK_THAT(sized.estimated_volume_fraction, WithinAbs(limited.estimated_volume_fraction, 0.001));
    CHECK_FALSE(preview_dense_region(mesh, setup, result, 1.0, &profile, false, -0.1).available);
    CHECK_FALSE(preview_dense_region(mesh, setup, result, 1.0, &profile, false, 1.1).available);
    CHECK_FALSE(preview_dense_region(mesh, setup, result, 1.0, &profile, false,
                                    std::numeric_limits<double>::quiet_NaN()).available);
}

TEST_CASE("Interior-cell volumes retain sloped walls cavities and thin separated solids", "[StrengthAnalysis]")
{
    indexed_triangle_set mesh;
    double expected = 0.0;
    SECTION("Sloped triangular prism") {
        mesh.vertices = {{0, 0, 0}, {100, 0, 0}, {0, 100, 0}, {0, 0, 1}, {100, 0, 1}, {0, 100, 1}};
        mesh.indices = {{0, 2, 1}, {3, 4, 5}, {0, 1, 4}, {0, 4, 3}, {1, 2, 5}, {1, 5, 4}, {2, 0, 3}, {2, 3, 5}};
        expected = 5000.0;
    }
    SECTION("Tetrahedron sloping through three grid axes") {
        mesh.vertices = {{0, 0, 0}, {20, 0, 0}, {0, 20, 0}, {0, 0, 20}};
        mesh.indices = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
        expected = 8000.0 / 6.0;
    }
    SECTION("Hollow cube with an off-grid cavity") {
        mesh = its_make_cube(20.0, 20.0, 20.0);
        auto cavity = its_make_cube(10.0, 10.0, 10.0);
        its_transform(cavity, identity3f().translate(Vec3f(3.7f, 4.3f, 5.1f)));
        for (Vec3i32 &triangle : cavity.indices)
            std::swap(triangle[0], triangle[1]);
        its_merge(mesh, cavity);
        expected = 7000.0;
    }
    SECTION("Thin plate below a distant solid") {
        mesh = its_make_cube(10.0, 10.0, 0.2);
        auto upper = its_make_cube(10.0, 10.0, 1.0);
        its_transform(upper, identity3f().translate(Vec3f(0.0f, 0.0f, 99.0f)));
        its_merge(mesh, upper);
        expected = 120.0;
    }
    const auto result = synthetic_result(mesh);
    const auto profile = build_dense_region_preview_profile(mesh, result);
    INFO(profile.warning);
    REQUIRE(profile.available);
    CHECK_THAT(profile.sampled_volume_mm3, WithinRel(expected, 1e-6));
    for (double fraction : {0.01, 0.15, 0.5, 1.0}) {
        const auto preview = preview_dense_region(mesh, Setup{}, result, fraction, &profile);
        REQUIRE(preview.applicable());
        CHECK_THAT(preview.estimated_volume_fraction, WithinAbs(fraction, 0.02));
        CHECK_THAT(sliced_dense_volume(mesh, preview, profile), WithinRel(preview.estimated_volume_m3 * 1e9, 1e-5));
    }
}

TEST_CASE("Dense-region profiles reject changed geometry and stress and support cancellation", "[StrengthAnalysis]")
{
    auto mesh = its_make_cube(20.0, 20.0, 20.0);
    auto result = synthetic_result(mesh);
    const auto profile = build_dense_region_preview_profile(mesh, result);
    REQUIRE(profile.available);
    auto changed = result;
    changed.vertices.back().von_mises_pa = 10e6;
    CHECK_FALSE(preview_dense_region(mesh, Setup{}, changed, 0.5, &profile).available);
    mesh.vertices.back().x() += 1.0f;
    result.vertices.back().position_mm = mesh.vertices.back().cast<double>();
    CHECK_FALSE(preview_dense_region(mesh, Setup{}, result, 0.5, &profile).available);
    const auto cancelled = build_dense_region_preview_profile(mesh, result, [] { return true; });
    CHECK_FALSE(cancelled.available);
    CHECK(cancelled.warning.find("cancelled") != std::string::npos);
    size_t checks = 0;
    const auto interrupted = build_dense_region_preview_profile(mesh, result, [&] { return ++checks > 20; });
    CHECK_FALSE(interrupted.available);
    CHECK(interrupted.warning.find("cancelled") != std::string::npos);
}

TEST_CASE("Strength orientation alignment respects existing instance transforms", "[StrengthAnalysis]")
{
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(TriangleMesh(its_make_cube(10.0, 10.0, 10.0)));
    const Vec3d layer_axis = Vec3d(1.0, 2.0, 3.0).normalized();
    for (bool mirrored : {false, true}) {
        INFO("Mirrored: " << mirrored);
        ModelInstance *instance = object->add_instance();
        instance->set_rotation(Vec3d(0.7, 0.4, 1.2));
        instance->set_scaling_factor(Vec3d(1.5, 0.8, 2.0));
        instance->set_mirror(Vec3d(mirrored ? -1.0 : 1.0, 1.0, 1.0));
        instance->set_offset(Vec3d(42.0, 37.0, 20.0));
        const Vec3d scaling = instance->get_scaling_factor();
        const Vec3d offset = instance->get_offset();
        // Use the same plane-normal alignment and native rotate operation as the GUI action.
        for (int application = 0; application < 2; ++application) {
            const Vec3d current_axis = instance->get_matrix().linear().inverse().transpose() * layer_axis;
            Vec3d rotation_axis;
            double angle = 0.0;
            Matrix3d rotation;
            Geometry::rotation_from_two_vectors(current_axis, Vec3d::UnitZ(), rotation_axis, angle, &rotation);
            instance->rotate(rotation);
            const Vec3d aligned = (instance->get_matrix().linear().inverse().transpose() * layer_axis).normalized();
            CHECK_THAT((aligned - Vec3d::UnitZ()).norm(), WithinAbs(0.0, 1e-8));
            CHECK_THAT((instance->get_scaling_factor() - scaling).norm(), WithinAbs(0.0, 1e-8));
            CHECK_THAT((instance->get_offset() - offset).norm(), WithinAbs(0.0, 1e-8));
            CHECK(instance->is_left_handed() == mirrored);
        }
    }
}

TEST_CASE("Dense-region settings cannot weaken or soften the background", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(20.0, 20.0, 20.0);
    Setup weaker = cube_setup(mesh);
    weaker.infill.background_pattern = StrengthAnalysis::InfillPattern::Gyroid;
    weaker.infill.background_density = 0.65;
    weaker.infill.dense_pattern = StrengthAnalysis::InfillPattern::Rectilinear;
    weaker.infill.dense_density = 0.65;
    const std::vector<std::string> errors = validate(mesh, weaker);
    CHECK(std::any_of(errors.begin(), errors.end(), [](const std::string &error) {
        return error.find("must not be weaker or less stiff") != std::string::npos;
    }));
    CHECK(analyze(mesh, weaker).status == AnalysisStatus::InvalidInput);

    Setup stronger = cube_setup(mesh);
    stronger.infill.background_pattern = StrengthAnalysis::InfillPattern::Rectilinear;
    stronger.infill.background_density = 0.65;
    stronger.infill.dense_pattern = StrengthAnalysis::InfillPattern::Gyroid;
    stronger.infill.dense_density = 0.65;
    const std::vector<std::string> stronger_errors = validate(mesh, stronger);
    CHECK(std::none_of(stronger_errors.begin(), stronger_errors.end(), [](const std::string &error) {
        return error.find("must not be weaker or less stiff") != std::string::npos;
    }));
}

TEST_CASE("Invalid and cancelled analyses fail explicitly", "[StrengthAnalysis]")
{
    const indexed_triangle_set mesh = its_make_cube(10.0, 10.0, 10.0);
    Setup invalid;
    const Result invalid_result = analyze(mesh, invalid);
    CHECK(invalid_result.status == AnalysisStatus::InvalidInput);
    CHECK_FALSE(invalid_result.message.empty());

    Setup invalid_threshold = cube_setup(mesh);
    invalid_threshold.infill.dense_stress_threshold = 0.0;
    CHECK(analyze(mesh, invalid_threshold).status == AnalysisStatus::InvalidInput);

    Setup invalid_constraint = cube_setup(mesh);
    invalid_constraint.loads.front().region.whole_model = true;
    CHECK(analyze(mesh, invalid_constraint).status == AnalysisStatus::InvalidInput);

    const Result cancelled = analyze(mesh, cube_setup(mesh), [] { return true; });
    CHECK(cancelled.status == AnalysisStatus::Cancelled);
}

TEST_CASE("Optimization comparisons are deterministic and mass targets are monotone", "[StrengthAnalysis]")
{
    Setup setup;
    const auto first = compare_infill_patterns(1e-5, setup, 1.4);
    const auto second = compare_infill_patterns(1e-5, setup, 1.4);
    REQUIRE(first.size() == 6);
    REQUIRE(first.size() == second.size());
    for (size_t index = 0; index < first.size(); ++index) {
        CHECK(first[index].pattern == second[index].pattern);
        CHECK_THAT(first[index].estimated_safety_factor, WithinAbs(second[index].estimated_safety_factor, 1e-12));
    }

    const auto curve = estimate_mass_strength_curve(1e-5, setup, 1.4);
    REQUIRE(curve.size() > 2);
    for (size_t index = 1; index < curve.size(); ++index) {
        CHECK(curve[index].target_safety_factor >= curve[index - 1].target_safety_factor);
        CHECK(curve[index].estimated_mass_kg >= curve[index - 1].estimated_mass_kg);
    }
}

TEST_CASE("Load ramp agrees with a proportional solve including gravity and impact", "[StrengthAnalysis][LoadRamp]")
{
    const auto mesh = its_make_cube(20.0, 20.0, 20.0);
    Setup setup = cube_setup(mesh);
    setup.loads.back().type = LoadType::ImpactForce;
    setup.loads.back().impact_factor = 2.5;
    setup.gravity.enabled = true;
    const Result endpoint = analyze(mesh, setup);
    REQUIRE(endpoint.succeeded());
    const auto ramp = make_load_ramp(mesh, setup, endpoint, 8.0);
    REQUIRE(ramp.probes.size() == 1);
    CHECK_THAT(ramp.probes.front().force_n, WithinRel(250.0, 1e-10));
    CHECK_THAT(ramp.duration_s, WithinAbs(8.0, 1e-10));
    const Result zero = load_ramp_frame(endpoint, 0.0);
    CHECK_THAT(zero.requested_resultant_force_n.norm(), WithinAbs(0.0, 1e-12));
    CHECK_THAT(zero.maximum_displacement_m, WithinAbs(0.0, 1e-12));
    CHECK(std::isinf(zero.minimum_safety_factor));
    setup.loads.back().magnitude_n *= 0.4;
    setup.gravity.acceleration_m_s2 *= 0.4;
    const Result solved = analyze(mesh, setup);
    const Result frame = load_ramp_frame(endpoint, 0.4);
    REQUIRE(solved.succeeded());
    CHECK_THAT(frame.maximum_displacement_m, WithinRel(solved.maximum_displacement_m, 1e-8));
    CHECK_THAT(frame.maximum_von_mises_pa, WithinRel(solved.maximum_von_mises_pa, 1e-8));
    for (size_t i = 0; i < frame.vertices.size(); ++i) {
        CHECK_THAT((frame.vertices[i].displacement_m - solved.vertices[i].displacement_m).norm(), WithinAbs(0.0, 1e-10));
        CHECK_THAT((frame.vertices[i].shear_stress_pa - solved.vertices[i].shear_stress_pa).norm(), WithinAbs(0.0, 1e-5));
    }
    CHECK_THAT(load_ramp_frame(endpoint, 1.0).maximum_displacement_m, WithinRel(endpoint.maximum_displacement_m, 1e-12));
}

TEST_CASE("Ramp elastic limit is independent of ultimate strength and target safety factor", "[StrengthAnalysis][LoadRamp]")
{
    const auto mesh = its_make_cube(20.0, 20.0, 20.0);
    Setup setup = cube_setup(mesh);
    const Result result = analyze(mesh, setup);
    REQUIRE(result.succeeded());
    const auto original = make_load_ramp(mesh, setup, result, 3.0);
    REQUIRE(std::isfinite(original.elastic_limit_fraction));
    setup.loads.back().strength_basis = StrengthBasis::Ultimate;
    setup.loads.back().target_safety_factor = 100.0;
    setup.material.ultimate_strength_xy_pa *= 10.0;
    setup.material.ultimate_strength_z_pa *= 10.0;
    CHECK_THAT(make_load_ramp(mesh, setup, result, 3.0).elastic_limit_fraction,
               WithinRel(original.elastic_limit_fraction, 1e-12));
    setup.material.yield_strength_xy_pa *= 0.5;
    setup.material.yield_strength_z_pa *= 0.5;
    setup.material.shear_strength_xy_pa *= 0.5;
    setup.material.shear_strength_xz_pa *= 0.5;
    CHECK_THAT(make_load_ramp(mesh, setup, result, 3.0).elastic_limit_fraction,
               WithinRel(original.elastic_limit_fraction * 0.5, 1e-12));
    CHECK_THAT(make_load_ramp(mesh, setup, result, -1.0).duration_s, WithinAbs(0.0, 1e-12));
    CHECK_THAT(make_load_ramp(mesh, setup, result, std::numeric_limits<double>::quiet_NaN()).duration_s, WithinAbs(0.0, 1e-12));
}

#include "libslic3r/StrengthTransient.hpp"

namespace {
Setup transient_cube_setup(double force_n)
{
    Setup setup;
    setup.infill.background_density = 1.0;
    setup.infill.dense_density = 1.0;
    Load support;
    support.type = LoadType::Fixed;
    support.region.shape = RegionShape::Box;
    support.region.center_mm = Vec3d(0.5,2,2);
    support.region.size_mm = Vec3d(1.01,5,5);
    setup.loads.push_back(support);
    Load force;
    force.direction = Vec3d::UnitX();
    force.magnitude_n = force_n;
    force.region.shape = RegionShape::Box;
    force.region.center_mm = Vec3d(3.5,2,2);
    force.region.size_mm = Vec3d(1.01,5,5);
    setup.loads.push_back(force);
    return setup;
}
TransientSettings small_transient_settings()
{
    TransientSettings settings;
    settings.layer_height_mm = 1.0;
    settings.first_layer_height_mm = 1.0;
    settings.cell_width_mm = 1.0;
    settings.increments = 20;
    settings.wall_loops = 0;
    return settings;
}
}

TEST_CASE("Layer-resolved elastic loading springs back after unloading", "[StrengthAnalysis][Transient]")
{
    const auto mesh = its_make_cube(4,4,4);
    auto settings = small_transient_settings();
    settings.plasticity = false;
    settings.fracture = false;
    const auto result = analyze_transient(mesh, transient_cube_setup(10), settings);
    INFO(result.message);
    REQUIRE(result.succeeded());
    REQUIRE(result.frames.size() == settings.increments + 1);
    CHECK(result.layer_count == 4);
    CHECK_THAT(result.frames.front().probe_displacements_mm[0], WithinAbs(0,1e-12));
    CHECK(result.frames[10].probe_displacements_mm[0] > 0);
    CHECK_THAT(result.frames.back().probe_displacements_mm[0], WithinAbs(0,1e-8));
    CHECK_THAT(result.frames[5].probe_displacements_mm[0], WithinRel(result.frames[15].probe_displacements_mm[0],1e-7));
    CHECK_THAT(result.frames.back().applied_forces_n[0], WithinAbs(0,1e-12));
    const Result display = transient_display_frame(result,10);
    CHECK(display.vertices.size() == result.display_mesh.vertices.size());
    CHECK(display.maximum_displacement_m > 0);
}

TEST_CASE("Plastic loading leaves permanent deformation after force removal", "[StrengthAnalysis][Transient]")
{
    auto settings = small_transient_settings();
    settings.fracture = false;
    const auto result = analyze_transient(its_make_cube(4,4,4), transient_cube_setup(1000), settings);
    INFO(result.message);
    REQUIRE(result.succeeded());
    CHECK(std::isfinite(result.first_yield_time_s));
    CHECK(result.frames.back().probe_displacements_mm[0] > 0.001);
    CHECK(result.frames.back().probe_displacements_mm[0] < result.frames[10].probe_displacements_mm[0]);
    CHECK_THAT(result.frames.back().applied_forces_n[0], WithinAbs(0,1e-12));
    CHECK(result.frames.back().maximum_plastic_strain > 0);
}

TEST_CASE("Failed load paths never reconnect or regain force during unloading", "[StrengthAnalysis][Transient]")
{
    auto settings = small_transient_settings();
    settings.failure_plastic_strain = 0.002;
    const auto result = analyze_transient(its_make_cube(4,4,4), transient_cube_setup(3000), settings);
    INFO(result.message);
    REQUIRE(result.succeeded());
    CHECK(std::isfinite(result.first_failure_time_s));
    CHECK(result.frames.back().detached_cells > 0);
    size_t previous_failed = 0, previous_detached = 0;
    bool released = false;
    for (const auto &frame : result.frames) {
        CHECK(frame.failed_bonds >= previous_failed);
        CHECK(frame.detached_cells >= previous_detached);
        previous_failed = frame.failed_bonds;
        previous_detached = frame.detached_cells;
        if (frame.detached_cells && frame.applied_forces_n[0] < 1e-10) released = true;
        if (released) CHECK_THAT(frame.applied_forces_n[0], WithinAbs(0,1e-10));
    }
    CHECK(released);
}

TEST_CASE("Layer geometry and thermal bonding respond to print inputs", "[StrengthAnalysis][Transient]")
{
    auto settings = small_transient_settings();
    settings.thermal_bonding = true;
    settings.bonding_scale = 0.7;
    settings.increments = 4;
    const auto mesh = its_make_cube(4,4,4);
    const auto setup = transient_cube_setup(10);
    const auto baseline = analyze_transient(mesh, setup, settings);
    REQUIRE(baseline.succeeded());
    settings.first_layer_height_mm = 0.5;
    settings.layer_height_mm = 0.5;
    settings.nozzle_temperature_c = 170;
    const auto changed = analyze_transient(mesh, setup, settings);
    INFO(changed.message);
    REQUIRE(changed.succeeded());
    CHECK(changed.layer_count == baseline.layer_count * 2);
    CHECK(changed.layer_bond_factors.back() < baseline.layer_bond_factors.back());
    CHECK(changed.layer_interface_temperature_c.back() < baseline.layer_interface_temperature_c.back());
    settings.maximum_cells = 8;
    CHECK(analyze_transient(mesh, setup, settings).status == AnalysisStatus::InvalidInput);
    CHECK(analyze_transient(mesh, setup, small_transient_settings(), [] { return true; }).status == AnalysisStatus::Cancelled);
}
