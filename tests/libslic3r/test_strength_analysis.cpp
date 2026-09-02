#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/StrengthAnalysis.hpp"
#include "libslic3r/Format/3mf.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Model.hpp"

#include "test_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>

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

} // namespace

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
