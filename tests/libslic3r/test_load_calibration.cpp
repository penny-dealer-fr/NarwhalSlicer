#include <catch2/catch_all.hpp>
#include "libslic3r/LoadCalibration.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/Print.hpp"
#include "test_utils.hpp"
#include <limits>

using namespace Slic3r;
namespace LC = Slic3r::LoadCalibration;
using Catch::Matchers::WithinAbs;

TEST_CASE("Load sweeps generate stable identities for both orientations", "[LoadCalibration]")
{
    const auto values = LC::range(0.1, 0.3, 0.1);
    REQUIRE(values.size() == 3);
    const auto samples = LC::make_samples(values, 3);
    REQUIRE(samples.size() == 18);
    REQUIRE(samples[0].id == "H001-XY");
    REQUIRE(samples[3].id == "H004-Z");
    REQUIRE(samples[6].value == values[1]);
    REQUIRE(LC::range(3, 1, -1).size() == 3);
    REQUIRE_THROWS(LC::range(1, 3, 0));
    REQUIRE_THROWS(LC::range(1, 3, -1));
    REQUIRE_THROWS(LC::range(0, 100, 1));
    REQUIRE_THROWS(LC::range(0, std::numeric_limits<double>::infinity(), 1));
    REQUIRE_THROWS(LC::make_samples({"1", "1"}, 2));
    REQUIRE_THROWS(LC::make_samples(values, 0));
}
TEST_CASE("Surviving hooks remain censored and untested hooks do not affect strength", "[LoadCalibration]")
{
    auto samples       = LC::make_samples({"30"}, 4);
    samples[0].outcome = LC::Outcome::Failed;
    samples[0].load_n  = 100;
    samples[0].mass_g  = 10;
    samples[1].outcome = LC::Outcome::Failed;
    samples[1].load_n  = 200;
    samples[1].mass_g  = 20;
    samples[2].outcome = LC::Outcome::Survived;
    samples[2].load_n  = 500;
    auto stats         = LC::summarize(samples, "30", "XY");
    REQUIRE(stats.failed == 2);
    REQUIRE(stats.survived == 1);
    REQUIRE(stats.untested == 1);
    REQUIRE_THAT(stats.mean_n, WithinAbs(150, 1e-9));
    REQUIRE_THAT(stats.minimum_n, WithinAbs(100, 1e-9));
    REQUIRE_THAT(stats.sd_n, WithinAbs(std::sqrt(5000), 1e-9));
    REQUIRE_THAT(*stats.mean_n_per_g, WithinAbs(10, 1e-9));
    REQUIRE(LC::summarize(samples, "30", "Z").failed == 0);
    samples[2].load_n.reset();
    REQUIRE_THROWS(LC::summarize(samples, "30", "XY"));
}
TEST_CASE("Hook results round trip with missing loads and notes", "[LoadCalibration]")
{
    auto samples       = LC::make_samples({"gyroid"}, 2);
    samples[0].outcome = LC::Outcome::Survived;
    samples[0].load_n  = 123.45;
    samples[0].mass_g  = 4.2;
    samples[1].notes   = "Failed print; not a tensile failure";
    const auto loaded  = LC::deserialize_samples(LC::serialize_samples(samples));
    REQUIRE(loaded.size() == samples.size());
    REQUIRE(loaded[0].outcome == LC::Outcome::Survived);
    REQUIRE_THAT(*loaded[0].load_n, WithinAbs(123.45, 1e-9));
    REQUIRE_FALSE(loaded[1].load_n);
    REQUIRE(loaded[1].notes == samples[1].notes);
    samples[0].mass_g = -1;
    REQUIRE_THROWS(LC::serialize_samples(samples));
}
TEST_CASE("Calibration changes tensile limits without inventing elastic or shear measurements", "[LoadCalibration]")
{
    auto samples = LC::make_samples({"30"}, 2);
    for (size_t i = 0; i < samples.size(); ++i) {
        samples[i].outcome = LC::Outcome::Failed;
        samples[i].load_n  = 100 + 10 * i;
    }
    const auto base   = StrengthAnalysis::builtin_materials().front();
    const auto result = LC::calibrate(base, "PLA Blue", samples, "30", 0.2, 0.1);
    REQUIRE(result.name == "PLA Blue -- CALIBRATED");
    REQUIRE_THAT(result.ultimate_strength_xy_pa, WithinAbs(20e6, 1));
    REQUIRE_THAT(result.ultimate_strength_z_pa, WithinAbs(12e6, 1));
    REQUIRE_THAT(result.elastic_modulus_xy_pa, WithinAbs(base.elastic_modulus_xy_pa, 1));
    REQUIRE_THAT(result.shear_strength_xz_pa, WithinAbs(base.shear_strength_xz_pa, 1));
    REQUIRE(result.validate().empty());
    REQUIRE_THROWS(LC::calibrate(base, "PLA", samples, "30", 0, 0.1));
    samples[0].outcome = LC::Outcome::Survived;
    REQUIRE_THROWS(LC::calibrate(base, "PLA", samples, "30", 0.2, 0.1));
}
TEST_CASE("Load sweeps apply real slicer options and preserve the baseline", "[LoadCalibration]")
{
    auto config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("wall_loops", "2");
    auto walls = LC::setting_config(config, "wall_loops", "4");
    REQUIRE(walls.opt_int("wall_loops") == 4);
    REQUIRE(config.opt_int("wall_loops") == 2);
    REQUIRE_THROWS(LC::setting_config(config, "wall_loops", "2.5"));
    REQUIRE_THROWS(LC::setting_config(config, "wall_loops", "-1"));
    REQUIRE_THROWS(LC::setting_config(config, "sparse_infill_density", "101"));
    REQUIRE_THROWS(LC::setting_config(config, "sparse_infill_pattern", "invalid"));
    auto temperature = LC::setting_config(config, "nozzle_temperature", "220");
    REQUIRE(temperature.option<ConfigOptionInts>("nozzle_temperature")->values.front() == 220);
    REQUIRE(temperature.option<ConfigOptionInts>("nozzle_temperature_initial_layer")->values.front() == 220);
    auto bed = LC::setting_config(config, "bed_temperature", "65");
    REQUIRE(bed.option<ConfigOptionInts>("textured_plate_temp")->values.front() == 65);
    REQUIRE(bed.option<ConfigOptionInts>("textured_cool_plate_temp_initial_layer")->values.front() == 65);
}
TEST_CASE("CNC hook projects preserve orientation identity and settings through 3MF", "[LoadCalibration]")
{
    const auto stl = (boost::filesystem::path(TEST_DATA_DIR).parent_path().parent_path() / "resources/handy_models/CNC_Testhook.stl")
                         .string();
    auto config = LC::setting_config(DynamicPrintConfig::full_print_config(), "wall_loops", "4");
    config.set_deserialize_strict("printable_area", "0x0,200x0,200x200,0x200");
    config.set_deserialize_strict("printable_height", "200");
    const auto samples = LC::make_samples({"4"}, 1);
    auto flat          = LC::hook_model(stl, samples[0], config);
    auto upright       = LC::hook_model(stl, samples[1], config);
    REQUIRE(flat.objects.front()->instances.size() == 1);
    REQUIRE_THAT(flat.objects.front()->bounding_box_exact().size().z(), WithinAbs(9, 0.01));
    REQUIRE_THAT(upright.objects.front()->bounding_box_exact().size().z(), WithinAbs(70, 0.01));
    REQUIRE_THAT(flat.objects.front()->bounding_box_exact().min.z(), WithinAbs(0, 0.001));
    REQUIRE_THAT(upright.objects.front()->bounding_box_exact().min.z(), WithinAbs(0, 0.001));
    ScopedTemporaryFile file(".3mf");
    REQUIRE_NOTHROW(LC::store_project(file.string(), upright, config));
    ScopedSlic3rTemporaryDir temporary;
    Model loaded;
    DynamicPrintConfig restored;
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Enable);
    PlateDataPtrs plates;
    std::vector<Preset*> presets;
    bool bbl = false, orca = false;
    Semver version;
    REQUIRE(load_bbs_3mf(file.string().c_str(), &restored, &substitutions, &loaded, &plates, &presets, &bbl, &orca, &version, nullptr,
                         LoadStrategy::LoadModel | LoadStrategy::LoadConfig));
    release_PlateData_list(plates);
    for (auto* preset : presets)
        delete preset;
    REQUIRE(loaded.objects.size() == 1);
    REQUIRE(loaded.objects.front()->name == upright.objects.front()->name);
    REQUIRE(restored.has("wall_loops"));
    REQUIRE(restored.opt_int("wall_loops") == 4);
    REQUIRE_THAT(loaded.objects.front()->bounding_box_exact().size().z(), WithinAbs(70, 0.01));
    REQUIRE_THAT(loaded.objects.front()->bounding_box_exact().min.z(), WithinAbs(0, 0.001));
    config.set_deserialize_strict("printable_height", "20");
    REQUIRE_THROWS(LC::hook_model(stl, samples[1], config));
    config.set_deserialize_strict("printable_area", "0x0,20x0,20x20,0x20");
    REQUIRE_THROWS(LC::hook_model(stl, samples[0], config));
}
TEST_CASE("Selected hooks share plates without overlap and remain on the bed after reload", "[LoadCalibration]")
{
    const auto stl = (boost::filesystem::path(TEST_DATA_DIR).parent_path().parent_path() / "resources/handy_models/CNC_Testhook.stl")
                         .string();
    auto config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("printable_area", "0x0,160x0,160x160,0x160");
    config.set_deserialize_strict("printable_height", "200");
    config.set_deserialize_strict("bed_exclude_area", "");
    const auto samples = LC::make_samples({"2", "4"}, 3);
    auto project       = LC::arrange_hooks(stl, samples, config, "wall_loops");
    REQUIRE(project.model.objects.size() == samples.size());
    REQUIRE(project.plates.size() > 1);
    REQUIRE(project.plates.size() < samples.size());
    for (const auto& plate : project.plates) {
        for (size_t i = 0; i < plate.objects.size(); ++i) {
            const auto box = project.model.objects[plate.objects[i]]->bounding_box_exact();
            REQUIRE_THAT(box.min.z(), WithinAbs(0, .001));
            REQUIRE(box.min.x() >= plate.origin.x());
            REQUIRE(box.max.x() <= plate.origin.x() + 160);
            REQUIRE(box.min.y() >= plate.origin.y());
            REQUIRE(box.max.y() <= plate.origin.y() + 160);
            for (size_t j = i + 1; j < plate.objects.size(); ++j) {
                const auto other  = project.model.objects[plate.objects[j]]->bounding_box_exact();
                const bool spaced = box.max.x() + 23 <= other.min.x() || other.max.x() + 23 <= box.min.x() ||
                                    box.max.y() + 23 <= other.min.y() || other.max.y() + 23 <= box.min.y();
                REQUIRE(spaced);
            }
        }
    }
    ScopedTemporaryFile file(".3mf");
    LC::store_project(file.string(), project);
    ScopedSlic3rTemporaryDir temporary;
    Model loaded;
    DynamicPrintConfig restored;
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Enable);
    PlateDataPtrs plates;
    std::vector<Preset*> presets;
    bool bbl = false, orca = false;
    Semver version;
    REQUIRE(load_bbs_3mf(file.string().c_str(), &restored, &substitutions, &loaded, &plates, &presets, &bbl, &orca, &version, nullptr,
                         LoadStrategy::LoadModel | LoadStrategy::LoadConfig));
    REQUIRE(plates.size() == project.plates.size());
    REQUIRE(loaded.objects.size() == samples.size());
    for (size_t i = 0; i < samples.size(); ++i) {
        const auto box      = loaded.objects[i]->bounding_box_exact();
        const auto original = project.model.objects[i]->bounding_box_exact();
        REQUIRE_THAT((box.min - original.min).norm(), WithinAbs(0, .001));
        REQUIRE_THAT(box.size().z(), WithinAbs(samples[i].orientation == "XY" ? 9 : 70, .01));
        REQUIRE(loaded.objects[i]->config.opt_int("wall_loops") == std::stoi(samples[i].value));
    }
    release_PlateData_list(plates);
    for (auto* preset : presets)
        delete preset;
    auto isolated = LC::arrange_hooks(stl, LC::make_samples({"5", "10"}, 1), config, "slow_down_layer_time");
    REQUIRE(isolated.plates.size() == 4);
}
TEST_CASE("Pattern and density studies preserve every combination through serialization and arrangement", "[LoadCalibration]")
{
    auto samples = LC::make_samples({"gyroid", "grid"}, 2, "sparse_infill_density", {"20", "30", "50"});
    REQUIRE(samples.size() == 24);
    auto restored = LC::deserialize_samples(LC::serialize_samples(samples));
    REQUIRE(restored[4].related.at("sparse_infill_density") == "30");
    REQUIRE(LC::sample_label(restored[0]) != LC::sample_label(restored[4]));
    REQUIRE(LC::summarize(restored, LC::sample_label(restored[0]), "XY").untested == 2);
    auto config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("printable_area", "0x0,200x0,200x200,0x200");
    config.set_deserialize_strict("printable_height", "200");
    config.set_deserialize_strict("bed_exclude_area", "");
    const auto stl = (boost::filesystem::path(TEST_DATA_DIR).parent_path().parent_path() / "resources/handy_models/CNC_Testhook.stl")
                         .string();
    const auto project = LC::arrange_hooks(stl, restored, config, "sparse_infill_pattern");
    REQUIRE(project.model.objects.size() == 24);
    for (size_t i = 0; i < restored.size(); ++i) {
        const auto& c = project.model.objects[i]->config;
        REQUIRE(c.opt_serialize("sparse_infill_pattern") == restored[i].value);
        REQUIRE_THAT(c.opt_float("sparse_infill_density"), WithinAbs(std::stod(restored[i].related.at("sparse_infill_density")), 1e-6));
    }
    ScopedTemporaryFile file(".3mf");
    auto saved = LC::arrange_hooks(stl, restored, config, "sparse_infill_pattern");
    LC::store_project(file.string(), saved);
    ScopedSlic3rTemporaryDir temporary;
    Model loaded;
    DynamicPrintConfig loaded_config;
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Enable);
    PlateDataPtrs plates;
    std::vector<Preset*> presets;
    bool bbl = false, orca = false;
    Semver version;
    REQUIRE(load_bbs_3mf(file.string().c_str(), &loaded_config, &substitutions, &loaded, &plates, &presets, &bbl, &orca, &version, nullptr,
                         LoadStrategy::LoadModel | LoadStrategy::LoadConfig));
    REQUIRE(loaded.objects.size() == restored.size());
    for (size_t i = 0; i < restored.size(); ++i) {
        REQUIRE(loaded.objects[i]->config.opt_serialize("sparse_infill_pattern") == restored[i].value);
        REQUIRE_THAT(loaded.objects[i]->config.opt_float("sparse_infill_density"),
                     WithinAbs(std::stod(restored[i].related.at("sparse_infill_density")), 1e-6));
        REQUIRE_THAT(loaded.objects[i]->bounding_box_exact().min.z(), WithinAbs(0, .001));
    }
    release_PlateData_list(plates);
    for (auto* preset : presets)
        delete preset;
    REQUIRE_THROWS(LC::make_samples({"grid", "gyroid"}, 20, "sparse_infill_density", {"20", "30", "50"}));
}
TEST_CASE("Internal flow calibration enables its gate without changing exterior flow", "[LoadCalibration]")
{
    auto config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("set_other_flow_ratios", "0");
    config.set_deserialize_strict("outer_wall_flow_ratio", "1.2");
    auto result = LC::setting_config(config, "inner_wall_flow_ratio", "1.1");
    REQUIRE(result.opt_bool("set_other_flow_ratios"));
    REQUIRE_THAT(result.opt_float("inner_wall_flow_ratio"), WithinAbs(1.1, 1e-6));
    REQUIRE_THAT(result.opt_float("outer_wall_flow_ratio"), WithinAbs(1, 1e-6));
    REQUIRE_THAT(result.opt_float("first_layer_flow_ratio"), WithinAbs(1, 1e-6));
    config.set_deserialize_strict("set_other_flow_ratios", "1");
    REQUIRE_THAT(LC::setting_config(config, "sparse_infill_flow_ratio", "1.1").opt_float("outer_wall_flow_ratio"), WithinAbs(1.2, 1e-6));
    REQUIRE(LC::setting_config(config, "alternate_extra_wall", "1").opt_bool("alternate_extra_wall"));
    REQUIRE_FALSE(LC::setting_config(config, "alternate_extra_wall", "0").opt_bool("alternate_extra_wall"));
    for (const auto* value : {"0", "1"})
        REQUIRE(LC::setting_config(config, "alternate_extra_wall", value).opt_serialize("ensure_vertical_shell_thickness") == "ensure_moderate");
    REQUIRE_THROWS(LC::setting_config(config, "alternate_extra_wall", "0.5"));
    for (const auto* key : {"inner_wall_line_width", "sparse_infill_line_width", "internal_solid_infill_line_width"})
        REQUIRE_THAT(LC::setting_config(config, key, "0.45").option<ConfigOptionFloatOrPercent>(key)->value, WithinAbs(.45, 1e-6));
}
