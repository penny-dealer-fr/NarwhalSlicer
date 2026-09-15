#include <catch2/catch_all.hpp>
#include "libslic3r/MaterialExperiments.hpp"
using namespace Slic3r;
namespace ME = MaterialExperiments;
namespace LC = LoadCalibration;
using Catch::Matchers::WithinRel;
namespace {
ME::Json study()
{
    auto r       = ME::create(StrengthAnalysis::builtin_materials().front());
    auto samples = LC::make_samples({"20", "30", "50"}, 2);
    for (auto& s : samples) {
        s.outcome = LC::Outcome::Failed;
        s.load_n  = 100 + 10 * std::stod(s.value);
    }
    ME::append_study(r, "first", "sparse_infill_density", {{"sparse_infill_density", "30"}}, samples, .1, .05, "regressed");
    return r;
}
} // namespace
TEST_CASE("Experimental curves interpolate and extrapolate with bounded physical domains", "[MaterialExperiments]")
{
    auto r       = study();
    auto middle  = ME::predict(r, "ultimate_strength_xy_pa", {{"sparse_infill_density", "35"}});
    auto outside = ME::predict(r, "ultimate_strength_xy_pa", {{"sparse_infill_density", "75"}});
    REQUIRE(middle.experimental);
    REQUIRE_FALSE(middle.extrapolated);
    REQUIRE(middle.points == 6);
    REQUIRE(middle.configurations == 3);
    REQUIRE_THAT(middle.value, WithinRel(45e6, .05));
    REQUIRE(outside.extrapolated);
    REQUIRE_THAT(outside.value, WithinRel(85e6, .08));
    REQUIRE(ME::predict(r, "ultimate_strength_xy_pa", {{"sparse_infill_density", "110"}}).bounded);
    REQUIRE_FALSE(ME::predict(r, "elastic_modulus_xy_pa", {}).experimental);
}
TEST_CASE("Saving studies accumulates evidence without duplicating hooks or restoring excluded data", "[MaterialExperiments]")
{
    auto r       = study();
    auto samples = LC::make_samples({"75"}, 2);
    for (auto& s : samples) {
        s.outcome = LC::Outcome::Failed;
        s.load_n  = 850;
    }
    ME::append_study(r, "second", "sparse_infill_density", {}, samples, .1, .05, "regressed");
    REQUIRE(r["observations"].size() == 16);
    r["observations"][12]["excluded"] = true;
    ME::append_study(r, "second", "sparse_infill_density", {}, samples, .1, .05, "regressed");
    REQUIRE(r["observations"].size() == 16);
    REQUIRE(r["observations"][12]["excluded"].get<bool>());
    REQUIRE(ME::predict(r, "ultimate_strength_xy_pa", {}).points == 7);
    r["observations"][13]["outcome"] = 2;
    REQUIRE(ME::predict(r, "ultimate_strength_xy_pa", {}).points == 6);
}
TEST_CASE("Simple mode uses tested minima and categorical values never interpolate", "[MaterialExperiments]")
{
    auto r                              = study();
    r["modes"]["sparse_infill_density"] = "simple";
    auto p                              = ME::predict(r, "ultimate_strength_xy_pa", {{"sparse_infill_density", "20.0000"}});
    REQUIRE(p.experimental);
    REQUIRE_THAT(p.value, WithinRel(30e6, 1e-8));
    REQUIRE_FALSE(ME::predict(r, "ultimate_strength_xy_pa", {{"sparse_infill_density", "35"}}).experimental);
    auto categories = ME::create(StrengthAnalysis::builtin_materials().front());
    auto samples    = LC::make_samples({"gyroid", "grid"}, 2);
    for (auto& s : samples) {
        s.outcome = LC::Outcome::Failed;
        s.load_n  = 400;
    }
    ME::append_study(categories, "patterns", "sparse_infill_pattern", {}, samples, .1, .05, "regressed");
    REQUIRE(ME::predict(categories, "ultimate_strength_xy_pa", {{"sparse_infill_pattern", "grid"}}).experimental);
    REQUIRE_FALSE(ME::predict(categories, "ultimate_strength_xy_pa", {{"sparse_infill_pattern", "cubic"}}).experimental);
}
TEST_CASE("Flat observations do not manufacture a trend", "[MaterialExperiments]")
{
    auto r = study();
    for (auto& o : r["observations"])
        o["measured"] = 40e6;
    auto p = ME::predict(r, "ultimate_strength_xy_pa", {{"sparse_infill_density", "75"}});
    REQUIRE(p.experimental);
    REQUIRE_THAT(p.value, WithinRel(40e6, 1e-6));
    REQUIRE(p.model.find("Pooled mean") != std::string::npos);
}
TEST_CASE("Experimental materials persist their evidence and evaluate the simulation settings", "[MaterialExperiments]")
{
    StrengthAnalysis::Setup setup;
    setup.material                      = StrengthAnalysis::builtin_materials().front();
    const auto modulus                  = setup.material.elastic_modulus_xy_pa;
    setup.material.experimental_data    = study().dump();
    setup.material.experimental_context = {{"sparse_infill_density", "20"}};
    setup.infill.background_density     = .75;
    StrengthAnalysis::Setup restored;
    REQUIRE(StrengthAnalysis::deserialize_setup(StrengthAnalysis::serialize_setup(setup), restored));
    REQUIRE(restored.material.experimental_data == setup.material.experimental_data);
    const auto evaluated = ME::evaluate_setup(restored);
    REQUIRE_THAT(evaluated.material.ultimate_strength_xy_pa, WithinRel(85e6, .08));
    REQUIRE_THAT(evaluated.material.elastic_modulus_xy_pa, WithinRel(modulus, 1e-8));
    auto invalid = study();
    invalid["observations"].push_back(invalid["observations"][0]);
    REQUIRE_THROWS(ME::validate(invalid));
    invalid                     = study();
    invalid["modes"]["unknown"] = "regressed";
    REQUIRE_THROWS(ME::validate(invalid));
}
TEST_CASE("Recommendations replace measured density factors and retain unmeasured effects", "[MaterialExperiments]")
{
    StrengthAnalysis::Setup setup;
    setup.material                   = StrengthAnalysis::builtin_materials().front();
    setup.material.experimental_data = study().dump();
    setup.infill.background_density  = .30;
    double factor                    = ME::strength_multiplier(setup, {{"sparse_infill_density", "75"}},
                                                               {{"sparse_infill_density", 2.5}, {"wall_loops", 1.12}});
    REQUIRE_THAT(factor, WithinRel(85. / 40. * 1.12, .08));
    setup.material.experimental_data.clear();
    REQUIRE_THAT(ME::strength_multiplier(setup, {}, {{"sparse_infill_density", 2.5}}), WithinRel(2.5, 1e-9));
}
TEST_CASE("Thermal fits hold at measured limits and equivalent numeric settings remain one configuration", "[MaterialExperiments]")
{
    auto r = ME::create(StrengthAnalysis::builtin_materials().front());
    auto samples = LC::make_samples({"190", "200", "210", "220", "230"}, 2);
    for (auto& s : samples) {
        s.outcome = LC::Outcome::Failed;
        s.load_n = 500 - .2 * std::pow(std::stod(s.value) - 210, 2);
    }
    ME::append_study(r, "thermal", "nozzle_temperature", {{"nozzle_temperature", "210"}}, samples, .1, .05, "regressed");
    auto p = ME::predict(r, "ultimate_strength_xy_pa", {{"nozzle_temperature", "300"}});
    REQUIRE(p.experimental);
    REQUIRE(p.bounded);
    REQUIRE(p.extrapolated);
    REQUIRE_THAT(p.value, WithinRel(ME::predict(r, "ultimate_strength_xy_pa", {{"nozzle_temperature", "230"}}).value, 1e-8));
    auto numeric = study();
    numeric["observations"][1]["context"]["sparse_infill_density"] = "20.0000";
    REQUIRE(ME::predict(numeric, "ultimate_strength_xy_pa", {}).configurations == 3);
    numeric.erase("reference");
    REQUIRE_THROWS(ME::validate(numeric));
}
TEST_CASE("Each infill pattern learns an independent density curve", "[MaterialExperiments]")
{
    auto record  = ME::create(StrengthAnalysis::builtin_materials().front());
    auto samples = LC::make_samples({"grid", "gyroid"}, 2, "sparse_infill_density", {"20", "30", "50"});
    for (auto& s : samples) {
        s.outcome            = LC::Outcome::Failed;
        const double density = std::stod(s.related.at("sparse_infill_density"));
        s.load_n             = s.value == "grid" ? 100 + 5 * density : 50 + 15 * density;
    }
    ME::append_study(record, "factorial", "sparse_infill_pattern", {{"sparse_infill_pattern", "grid"}, {"sparse_infill_density", "30"}},
                     samples, .1, .05, "regressed");
    const auto grid = ME::predict(record, "ultimate_strength_xy_pa", {{"sparse_infill_pattern", "grid"}, {"sparse_infill_density", "35"}});
    const auto gyroid = ME::predict(record, "ultimate_strength_xy_pa",
                                    {{"sparse_infill_pattern", "gyroid"}, {"sparse_infill_density", "75"}});
    REQUIRE(grid.points == 6);
    REQUIRE(grid.configurations == 3);
    REQUIRE_THAT(grid.value, WithinRel(27.5e6, .05));
    REQUIRE_THAT(gyroid.value, WithinRel(117.5e6, .08));
    REQUIRE_FALSE(ME::predict(record, "ultimate_strength_xy_pa", {{"sparse_infill_pattern", "cubic"}}).experimental);
    ME::append_study(record, "factorial", "sparse_infill_pattern", {}, samples, .1, .05, "regressed");
    REQUIRE(record["observations"].size() == 24);
}
