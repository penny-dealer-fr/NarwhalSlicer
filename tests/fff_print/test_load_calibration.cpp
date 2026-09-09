#include <catch2/catch_all.hpp>
#include "libslic3r/LoadCalibration.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "test_helpers.hpp"
#include "test_utils.hpp"
#include <fstream>
#include "nlohmann/json.hpp"

using namespace Slic3r;
TEST_CASE("CNC testhook plates slice in XY and Z with the swept settings", "[LoadCalibration]")
{
    const std::string orientation = GENERATE("XY", "Z");
    auto baseline                 = Test::multifilament_config(1, {{"printable_area", "0x0,200x0,200x200,0x200"},
                                                                   {"printable_height", "200"},
                                                                   {"layer_change_gcode", "G92 E0"},
                                                                   {"layer_height", "0.3"},
                                                                   {"wall_loops", "2"},
                                                                   {"sparse_infill_density", "15"}});
    auto config                   = LoadCalibration::setting_config(baseline, "nozzle_temperature", "215");
    const auto stl = (boost::filesystem::path(TEST_DATA_DIR).parent_path().parent_path() / "resources/handy_models/CNC_Testhook.stl")
                         .string();
    LoadCalibration::Sample sample;
    sample.id          = "H001-" + orientation;
    sample.orientation = orientation;
    sample.value       = "215";
    auto model         = LoadCalibration::hook_model(stl, sample, config);
    Print print;
    print.set_status_silent();
    print.apply(model, config);
    const auto validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    REQUIRE_NOTHROW(print.process());
    ScopedTemporaryFile gcode(".gcode");
    REQUIRE_NOTHROW(print.export_gcode(gcode.string(), nullptr));
    std::ifstream input(gcode.string());
    std::string text((std::istreambuf_iterator<char>(input)), {});
    REQUIRE(text.find("215") != std::string::npos);
    REQUIRE(text.find("; layer_height = 0.3") != std::string::npos);
    size_t extrusions = 0;
    GCodeReader reader;
    reader.parse_buffer(text, [&](GCodeReader& r, const GCodeReader::GCodeLine& line) {
        if (line.extruding(r) && line.dist_XY(r) > 0)
            ++extrusions;
    });
    REQUIRE(extrusions > 100);
}

// Temporary local diagnostic; removed after reducing the regression.
TEST_CASE("Local saved calibration study diagnostic", "[.LoadCalibrationDiagnostic]")
{
    const auto source=std::getenv("NARWHAL_STUDY_REPRO");
    REQUIRE(source!=nullptr);
    std::ifstream input(source);
    nlohmann::json study; input>>study;
    DynamicPrintConfig baseline;
    for(auto it=study["baseline"].begin();it!=study["baseline"].end();++it)
        baseline.set_deserialize_strict(it.key(),it.value().get<std::string>());
    auto config=LoadCalibration::setting_config(baseline,study["setting"],study["samples"][0]["value"]);
    auto samples=LoadCalibration::deserialize_samples(study["samples"].dump());
    auto stl=(boost::filesystem::path(TEST_DATA_DIR).parent_path().parent_path()/"resources/handy_models/CNC_Testhook.stl").string();
    auto model=LoadCalibration::hook_model(stl,samples.front(),config);
    Print print; print.is_BBL_printer()=true; print.set_status_silent(); print.apply(model,config);
    auto error=print.validate(); INFO(error.string); INFO(error.opt_key); REQUIRE(error.string.empty());
    print.process(); ScopedTemporaryFile gcode(".gcode"); REQUIRE_NOTHROW(print.export_gcode(gcode.string(),nullptr));
}
