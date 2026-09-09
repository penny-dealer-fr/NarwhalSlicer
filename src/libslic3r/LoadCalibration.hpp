#ifndef slic3r_LoadCalibration_hpp_
#define slic3r_LoadCalibration_hpp_

#include "PrintConfig.hpp"
#include "Model.hpp"
#include "StrengthAnalysis.hpp"
#include <optional>

namespace Slic3r { class Print; }

namespace Slic3r::LoadCalibration {
struct Setting
{
    std::string key;
};
const std::vector<Setting>& settings();
std::vector<std::string> range(double start, double end, double step);
DynamicPrintConfig setting_config(const DynamicPrintConfig& baseline, const std::string& key, const std::string& value);

enum class Outcome { NotTested, Failed, Survived };
struct Sample
{
    std::string id, value, orientation;
    Outcome outcome{Outcome::NotTested};
    std::optional<double> load_n, mass_g;
    std::string notes;
};
struct Statistics
{
    size_t failed{0}, survived{0}, untested{0};
    double mean_n{0}, sd_n{0}, minimum_n{0};
    std::optional<double> mean_n_per_g;
};
Statistics summarize(const std::vector<Sample>& samples, const std::string& value, const std::string& orientation);
void validate_sample(const Sample& sample);
std::vector<Sample> make_samples(const std::vector<std::string>& values, int repeats);
// One hook per plate deliberately isolates thermal history and prevents replicate identity ambiguity.
Model hook_model(const std::string& stl, const Sample& sample, const DynamicPrintConfig& config);
void store_project(const std::string& path, Model& model, DynamicPrintConfig& config);
void prepare_print(Print& print, const Model& model, const DynamicPrintConfig& config);
struct HookPlate {
    std::vector<size_t> objects;
    DynamicPrintConfig config;
    Vec3d origin{Vec3d::Zero()};
};
struct HookProject {
    Model model;
    DynamicPrintConfig config;
    std::vector<HookPlate> plates;
};
HookProject arrange_hooks(const std::string& stl, const std::vector<Sample>& samples,
    const DynamicPrintConfig& baseline, const std::string& setting, size_t max_plates = 36);
void store_project(const std::string& path, HookProject& project);
// Versioned, whitelisted native plate metadata; empty for ordinary plates.
std::string serialize_plate_settings(const DynamicPrintConfig& config);
DynamicPrintConfig deserialize_plate_settings(const std::string& text);
// Stress per force must come from a validated hook/fixture model, not force divided by arbitrary hook area.
StrengthAnalysis::Material calibrate(const StrengthAnalysis::Material& base,
                                     const std::string& name,
                                     const std::vector<Sample>& samples,
                                     const std::string& value,
                                     double xy_mpa_per_n,
                                     double z_mpa_per_n);
std::string serialize_samples(const std::vector<Sample>& samples);
std::vector<Sample> deserialize_samples(const std::string& text);
} // namespace Slic3r::LoadCalibration
#endif
