#ifndef slic3r_MaterialExperiments_hpp_
#define slic3r_MaterialExperiments_hpp_
#include "StrengthAnalysis.hpp"
#include "LoadCalibration.hpp"
#include "nlohmann/json.hpp"
#include <map>
#include <set>
namespace Slic3r::MaterialExperiments {
using Json    = nlohmann::json;
using Context = std::map<std::string, std::string>;
struct Property
{
    const char* key;
    const char* label;
    double StrengthAnalysis::Material::* member;
};
const std::vector<Property>& properties();
Json create(const StrengthAnalysis::Material& base);
void validate(const Json& record);
void append_study(Json& record,
                  const std::string& source,
                  const std::string& parameter,
                  const Context& context,
                  const std::vector<LoadCalibration::Sample>& samples,
                  double xy_mpa_per_n,
                  double z_mpa_per_n,
                  const std::string& mode);
struct Prediction
{
    bool experimental{false}, extrapolated{false}, bounded{false};
    size_t points{0}, configurations{0};
    double value{0}, rmse{0};
    std::string model{"Inherited"};
    std::set<std::string> parameters;
};
Prediction predict(const Json& record, const std::string& property, const Context& context);
StrengthAnalysis::Material evaluate(const StrengthAnalysis::Material& material,
                                    const Context& context,
                                    std::map<std::string, Prediction>* predictions = nullptr);
StrengthAnalysis::Setup evaluate_setup(const StrengthAnalysis::Setup& setup);
// Replace supported empirical parameter factors while retaining unmeasured estimates.
double strength_multiplier(const StrengthAnalysis::Setup& setup,
                           const Context& changes,
                           const std::map<std::string, double>& estimated_factors);
Context context_from_config(const DynamicPrintConfig& config);
} // namespace Slic3r::MaterialExperiments
#endif
