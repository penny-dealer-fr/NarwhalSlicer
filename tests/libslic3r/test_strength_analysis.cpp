#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

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

TEST_CASE("Strength setup travels with its model object through 3MF", "[StrengthAnalysis][3mf]")
{
    Setup setup;
    setup.material = *find_builtin_material("abs_generic");
    setup.gravity.enabled = true;
    Load fixed;
    fixed.type = LoadType::Fixed;
    setup.loads.push_back(fixed);

    Model source;
    ModelObject *source_object = source.add_object();
    source_object->add_volume(TriangleMesh(its_make_cube(12.0, 8.0, 4.0)));
    source_object->add_instance();
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
