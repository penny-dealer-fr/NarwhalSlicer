#include "MaterialExperiments.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>
#include <iomanip>
#include <stdexcept>
namespace Slic3r::MaterialExperiments {
const std::vector<Property>& properties()
{
    using M = StrengthAnalysis::Material;
    static const std::vector<Property> list{{"density_kg_m3", "Density (kg/m³)", &M::density_kg_m3},
                                            {"elastic_modulus_xy_pa", "Elastic modulus XY (Pa)", &M::elastic_modulus_xy_pa},
                                            {"elastic_modulus_z_pa", "Elastic modulus Z (Pa)", &M::elastic_modulus_z_pa},
                                            {"poisson_xy", "Poisson ratio XY", &M::poisson_xy},
                                            {"shear_modulus_xy_pa", "Shear modulus XY (Pa)", &M::shear_modulus_xy_pa},
                                            {"shear_modulus_xz_pa", "Shear modulus XZ (Pa)", &M::shear_modulus_xz_pa},
                                            {"yield_strength_xy_pa", "Yield strength XY (Pa)", &M::yield_strength_xy_pa},
                                            {"yield_strength_z_pa", "Yield strength Z (Pa)", &M::yield_strength_z_pa},
                                            {"ultimate_strength_xy_pa", "Ultimate strength XY (Pa)", &M::ultimate_strength_xy_pa},
                                            {"ultimate_strength_z_pa", "Ultimate strength Z (Pa)", &M::ultimate_strength_z_pa},
                                            {"shear_strength_xy_pa", "Shear strength XY (Pa)", &M::shear_strength_xy_pa},
                                            {"shear_strength_xz_pa", "Shear strength XZ (Pa)", &M::shear_strength_xz_pa}};
    return list;
}
Json create(const StrengthAnalysis::Material& base)
{
    Json values = Json::object();
    for (const auto& p : properties())
        values[p.key] = base.*(p.member);
    return {{"version", 1},
            {"base", values},
            {"base_name", base.name},
            {"observations", Json::array()},
            {"modes", Json::object()},
            {"reference", Context{}},
            {"ranges", Json::object()}};
}
namespace {
bool numeric(const std::string& text, double& x)
{
    try {
        size_t used = 0;
        x           = std::stod(text, &used);
        return used == text.size() && std::isfinite(x);
    } catch (...) {
        return false;
    }
}
bool same_value(const std::string& a, const std::string& b)
{
    double x, y;
    return numeric(a, x) && numeric(b, y) ? std::abs(x - y) <= 1e-9 * std::max({1., std::abs(x), std::abs(y)}) : a == b;
}
bool known_property(const std::string& key)
{
    return std::any_of(properties().begin(), properties().end(), [&](const Property& p) { return key == p.key; });
}
bool known_parameter(const std::string& key)
{
    return key == "direct" || std::any_of(LoadCalibration::settings().begin(), LoadCalibration::settings().end(),
                                          [&](const LoadCalibration::Setting& s) { return key == s.key; });
}
bool monotone(const std::string& p)
{
    return p == "sparse_infill_density" || p == "wall_loops" || p == "top_shell_layers" || p == "bottom_shell_layers";
}
bool thermal(const std::string& p) { return p == "nozzle_temperature" || p == "bed_temperature" || p == "slow_down_layer_time"; }
} // namespace
void validate(const Json& r)
{
    if (r.at("version") != 1 || !r.at("observations").is_array() || r.at("observations").size() > 20000)
        throw std::invalid_argument("Unsupported or oversized experimental material record.");
    for (const auto& p : properties()) {
        const double value = r.at("base").at(p.key).get<double>();
        if (!std::isfinite(value) || (std::string(p.key) == "poisson_xy" ? (value <= -1 || value >= 0.5) : value <= 0))
            throw std::invalid_argument("Invalid base material property.");
    }
    if (!r.at("modes").is_object() || !r.at("ranges").is_object() || r.at("reference").get<Context>().size() > 100)
        throw std::invalid_argument("Invalid experimental settings.");
    std::set<std::string> ids;
    for (const auto& o : r.at("observations")) {
        const auto id = o.at("id").get<std::string>();
        if (id.empty() || !ids.insert(id).second || !known_property(o.at("property")) || !known_parameter(o.at("parameter")))
            throw std::invalid_argument("Invalid or duplicated experimental data point.");
        o.at("value").get<std::string>();
        o.value("excluded", false);
        o.value("source", std::string());
        o.value("notes", std::string());
        auto context = o.at("context").get<Context>();
        if (context.size() > 100)
            throw std::invalid_argument("Oversized experimental context.");
        if (!o.at("measured").is_null()) {
            const double y = o.at("measured").get<double>();
            if (!std::isfinite(y) || y <= 0 || (o.at("property") == "poisson_xy" && y >= 0.5))
                throw std::invalid_argument("Invalid measurement.");
        }
        const auto state = o.at("outcome").get<int>();
        if (state < 0 || state > 2)
            throw std::invalid_argument("Invalid observation outcome.");
    }
    for (auto it = r.at("modes").begin(); it != r.at("modes").end(); ++it)
        if (!known_parameter(it.key()) || (it.value() != "simple" && it.value() != "regressed"))
            throw std::invalid_argument("Unknown calculation mode.");
    for (auto it = r.at("ranges").begin(); it != r.at("ranges").end(); ++it) {
        if (!known_property(it.key()) || !it.value().is_array() || it.value().size() != 2)
            throw std::invalid_argument("Invalid expected range.");
        double lo = it.value()[0], hi = it.value()[1];
        if (!std::isfinite(lo) || !std::isfinite(hi) || lo >= hi)
            throw std::invalid_argument("Expected range requires a finite minimum below maximum.");
    }
}
Context context_from_config(const DynamicPrintConfig& config)
{
    Context result;
    for (const auto& p : LoadCalibration::settings()) {
        std::string key = p.key;
        if (key == "bed_temperature") {
            const auto* bed = config.option<ConfigOptionEnum<BedType>>("curr_bed_type");
            key             = bed ? get_bed_temp_key(bed->value) : "hot_plate_temp";
        }
        if (config.has(key)) {
            auto value = config.opt_serialize(key);
            if (auto pos = value.find(';'); pos != std::string::npos)
                value.resize(pos);
            if (!value.empty() && value.back() == '%')
                value.pop_back();
            result[p.key] = value;
        }
    }
    return result;
}
void append_study(Json& r,
                  const std::string& source,
                  const std::string& parameter,
                  const Context& context,
                  const std::vector<LoadCalibration::Sample>& samples,
                  double xy,
                  double z,
                  const std::string& mode)
{
    validate(r);
    if (!known_parameter(parameter) || !std::isfinite(xy) || !std::isfinite(z) || xy <= 0 || z <= 0)
        throw std::invalid_argument("Supply validated positive XY and Z stress-per-force factors.");
    if (mode != "regressed" && mode != "simple")
        throw std::invalid_argument("Unknown calculation mode.");
    auto updated                = r;
    updated["modes"][parameter] = mode;
    if (updated["reference"].empty())
        updated["reference"] = context;
    for (const auto& sample : samples) {
        LoadCalibration::validate_sample(sample);
        const std::string property = sample.orientation == "XY" ? "ultimate_strength_xy_pa" : "ultimate_strength_z_pa";
        auto inputs                = context;
        inputs[parameter]          = sample.value;
        Json row                   = {{"id", source + "/" + sample.id + "/" + property},
                                      {"source", source},
                                      {"hook", sample.id},
                                      {"property", property},
                                      {"parameter", parameter},
                                      {"value", sample.value},
                                      {"context", inputs},
                                      {"outcome", int(sample.outcome)},
                                      {"excluded", false},
                                      {"notes", sample.notes},
                                      {"force_n", sample.load_n ? Json(*sample.load_n) : Json(nullptr)},
                                      {"mass_g", sample.mass_g ? Json(*sample.mass_g) : Json(nullptr)},
                                      {"stress_per_force", sample.orientation == "XY" ? xy : z}};
        row["measured"]            = sample.load_n ? Json(*sample.load_n * (sample.orientation == "XY" ? xy : z) * 1e6) : Json(nullptr);
        bool found                 = false;
        for (auto& old : updated["observations"])
            if (old.at("id") == row.at("id")) {
                row["excluded"] = old.value("excluded", false);
                old             = std::move(row);
                found           = true;
                break;
            }
        if (!found)
            updated["observations"].push_back(std::move(row));
    }
    validate(updated);
    r = std::move(updated);
}
Prediction predict(const Json& r, const std::string& property, const Context& supplied)
{
    Prediction out;
    out.value     = r.at("base").at(property).get<double>();
    Context input = r.at("reference").get<Context>();
    for (const auto& [k, v] : supplied)
        input[k] = v;
    std::vector<const Json*> rows;
    std::set<std::string> parameters;
    for (const auto& o : r.at("observations")) {
        if (o.at("property") != property || o.value("excluded", false) || o.at("outcome") != int(LoadCalibration::Outcome::Failed) ||
            o.at("measured").is_null())
            continue;
        rows.push_back(&o);
        if (o.at("parameter") != "direct")
            parameters.insert(o.at("parameter").get<std::string>());
    }
    // Simple and categorical modes only pool matching values; never fabricate category interpolation.
    std::set<std::string> numeric_parameters;
    for (const auto& p : parameters) {
        bool all_numeric = true;
        std::set<double> unique;
        for (auto* o : rows) {
            const auto c = o->at("context").get<Context>();
            double x;
            if (!c.count(p) || !numeric(c.at(p), x)) {
                all_numeric = false;
                break;
            }
            unique.insert(x);
        }
        const auto* definition = print_config_def.get(p);
        if (r.at("modes").value(p, "regressed") == "regressed" && (!definition || definition->type != coEnum) && all_numeric &&
            unique.size() >= 3)
            numeric_parameters.insert(p);
        else {
            rows.erase(std::remove_if(rows.begin(), rows.end(),
                                      [&](const Json* o) {
                                          const auto c = o->at("context").get<Context>();
                                          return !input.count(p) || !c.count(p) || !same_value(c.at(p), input.at(p));
                                      }),
                       rows.end());
        }
    }
    out.points = rows.size();
    if (rows.size() < 2)
        return out;
    out.points     = rows.size();
    out.parameters = parameters;
    struct Group
    {
        Context context;
        double sum{0};
        size_t n{0};
        double y{0};
    };
    std::map<std::string, Group> groups;
    for (auto* o : rows) {
        auto c = o->at("context").get<Context>();
        Context relevant;
        for (const auto& p : parameters)
            if (c.count(p)) {
                double x;
                if (numeric(c[p], x)) {
                    std::ostringstream value;
                    value << std::setprecision(12) << x;
                    relevant[p] = value.str();
                } else relevant[p] = c[p];
            }
        auto& g   = groups[Json(relevant).dump()];
        g.context = std::move(c);
        g.sum += o->at("measured").get<double>();
        ++g.n;
    }
    out.configurations = groups.size();
    std::vector<Group> data;
    for (auto& [key, g] : groups) {
        g.y = g.sum / g.n;
        data.push_back(g);
    }
    const bool all_simple = parameters.empty() ? r.at("modes").value("direct", "regressed") == "simple" :
                                                 std::all_of(parameters.begin(), parameters.end(), [&](const std::string& p) {
                                                     return r.at("modes").value(p, "regressed") == "simple";
                                                 });
    if (numeric_parameters.empty() || data.size() < 3) {
        out.experimental = true;
        out.model        = all_simple ? "Simple minimum at tested values" : "Replicate mean at tested values";
        out.value        = all_simple ? std::numeric_limits<double>::infinity() : 0;
        for (auto* o : rows) {
            double y = o->at("measured");
            if (all_simple)
                out.value = std::min(out.value, y);
            else
                out.value += y / rows.size();
        }
        return out;
    }
    struct Feature
    {
        std::string key;
        double lo, hi;
        bool square;
    };
    std::vector<Feature> features;
    for (const auto& p : numeric_parameters) {
        double lo = std::numeric_limits<double>::infinity(), hi = -lo;
        for (const auto& g : data) {
            double x;
            numeric(g.context.at(p), x);
            lo = std::min(lo, x);
            hi = std::max(hi, x);
        }
        if (hi <= lo || features.size() + 2 >= data.size())
            continue;
        features.push_back({p, lo, hi, false});
        // Temperature can have one optimum; only allow a concave quadratic with five or more levels.
        if (thermal(p) && data.size() >= 5 && features.size() + 2 < data.size())
            features.push_back({p, lo, hi, true});
    }
    if (features.empty())
        return out;
    const size_t n = data.size(), d = features.size() + 1;
    Eigen::MatrixXd X(n, d);
    Eigen::VectorXd y(n), query(d);
    query[0]     = 1;
    double scale = 0;
    for (const auto& g : data)
        scale += g.y / n;
    for (size_t i = 0; i < n; ++i) {
        X(i, 0) = 1;
        y[i]    = data[i].y / scale;
    }
    for (size_t j = 0; j < features.size(); ++j) {
        const auto& f = features[j];
        double q;
        if (!input.count(f.key) || !numeric(input.at(f.key), q))
            return out;
        if (q < f.lo || q > f.hi)
            out.extrapolated = true;
        // Physical-domain extrapolation for density; thermal predictions hold at measured boundary.
        if (f.key == "sparse_infill_density") {
            double bounded = std::clamp(q, 0., 100.);
            out.bounded |= bounded != q;
            q = bounded;
        }
        if (thermal(f.key)) {
            double bounded = std::clamp(q, f.lo, f.hi);
            out.bounded |= bounded != q;
            q = bounded;
        }
        q            = (q - f.lo) / (f.hi - f.lo);
        query[j + 1] = f.square ? q * q : q;
        for (size_t i = 0; i < n; ++i) {
            double x;
            numeric(data[i].context.at(f.key), x);
            x           = (x - f.lo) / (f.hi - f.lo);
            X(i, j + 1) = f.square ? x * x : x;
        }
    }
    const size_t folds = std::min(size_t(8), n);
    auto fit           = [&](double lambda, int omitted) {
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(d, d);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(d);
        for (size_t i = 0; i < n; ++i)
            if (omitted < 0 || int(i % folds) != omitted) {
                A.noalias() += X.row(i).transpose() * X.row(i);
                b.noalias() += X.row(i).transpose() * y[i];
            }
        for (size_t j = 1; j < d; ++j)
            A(j, j) += lambda;
        std::vector<bool> fixed(d, false);
        Eigen::VectorXd beta = Eigen::VectorXd::Zero(d);
        for (size_t iteration = 0; iteration < d; ++iteration) {
            auto a   = A;
            auto rhs = b;
            for (size_t j = 0; j < d; ++j)
                if (fixed[j]) {
                    a.row(j).setZero();
                    a.col(j).setZero();
                    a(j, j) = 1;
                    rhs[j]  = 0;
                }
            beta         = a.completeOrthogonalDecomposition().solve(rhs);
            bool changed = false;
            for (size_t j = 1; j < d; ++j)
                if (!fixed[j] && ((monotone(features[j - 1].key) && beta[j] < 0) || (features[j - 1].square && beta[j] > 0))) {
                    fixed[j] = true;
                    changed  = true;
                }
            if (!changed)
                break;
        }
        return beta;
    };
    // Hold out complete configurations: replicates cannot leak into validation folds.
    const double lambdas[] = {0.01, 0.1, 1., 10., 100.};
    double best = std::numeric_limits<double>::infinity(), chosen = 1;
    for (double lambda : lambdas) {
        double error = 0;
        for (size_t fold = 0; fold < folds; ++fold) {
            auto beta = fit(lambda, int(fold));
            for (size_t i = fold; i < n; i += folds) {
                double e = X.row(i).dot(beta) - y[i];
                error += e * e / n;
            }
        }
        if (error < best) {
            best   = error;
            chosen = lambda;
        }
    }
    double constant_error = 0;
    for (size_t fold = 0; fold < folds; ++fold) {
        double sum   = 0;
        size_t count = 0;
        for (size_t i = 0; i < n; ++i)
            if (i % folds != fold) {
                sum += y[i];
                ++count;
            }
        for (size_t i = fold; i < n; i += folds)
            constant_error += std::pow(sum / count - y[i], 2) / n;
    }
    out.experimental = true;
    out.rmse         = std::sqrt(best) * scale;
    if (best >= constant_error * 0.95) {
        out.model = "Pooled mean (no validated trend)";
        out.rmse  = std::sqrt(constant_error) * scale;
        out.value = y.mean() * scale;
    } else {
        auto beta = fit(chosen, -1);
        out.value = query.dot(beta) * scale;
        out.model = "Constrained ridge main effects; configuration cross-validation";
    }
    // Do not return nonphysical negative properties from extrapolation.
    double bounded = std::max(out.value, scale * 0.01);
    if (property == "poisson_xy")
        bounded = std::clamp(bounded, 0.001, 0.499);
    out.bounded |= bounded != out.value;
    out.value = bounded;
    return out;
}
StrengthAnalysis::Material evaluate(const StrengthAnalysis::Material& m,
                                    const Context& context,
                                    std::map<std::string, Prediction>* predictions)
{
    if (m.experimental_data.empty())
        return m;
    const auto record = Json::parse(m.experimental_data);
    validate(record);
    auto result = m;
    for (const auto& p : properties()) {
        auto value         = predict(record, p.key, context);
        result.*(p.member) = value.value;
        if (predictions)
            (*predictions)[p.key] = value;
    }
    // A tensile limit constrains an inherited yield estimate without claiming a yield measurement.
    result.yield_strength_xy_pa = std::min(result.yield_strength_xy_pa, result.ultimate_strength_xy_pa);
    result.yield_strength_z_pa  = std::min(result.yield_strength_z_pa, result.ultimate_strength_z_pa);
    return result;
}
StrengthAnalysis::Setup evaluate_setup(const StrengthAnalysis::Setup& original)
{
    if (original.material.experimental_data.empty())
        return original;
    auto result                     = original;
    auto inputs                     = result.material.experimental_context;
    inputs["sparse_infill_density"] = std::to_string(original.infill.background_density * 100);
    inputs["sparse_infill_pattern"] = StrengthAnalysis::to_string(original.infill.background_pattern);
    result.material                 = evaluate(original.material, inputs);
    return result;
}
double strength_multiplier(const StrengthAnalysis::Setup& setup,
                           const Context& changes,
                           const std::map<std::string, double>& estimated_factors)
{
    double factor = 1;
    for (const auto& [key, value] : estimated_factors)
        factor *= value;
    if (setup.material.experimental_data.empty())
        return factor;
    const auto record               = Json::parse(setup.material.experimental_data);
    Context before                  = setup.material.experimental_context;
    before["sparse_infill_density"] = std::to_string(setup.infill.background_density * 100);
    before["sparse_infill_pattern"] = StrengthAnalysis::to_string(setup.infill.background_pattern);
    auto after                      = before;
    for (const auto& [key, value] : changes)
        after[key] = value;
    const auto xy0 = predict(record, "ultimate_strength_xy_pa", before);
    const auto z0  = predict(record, "ultimate_strength_z_pa", before);
    const auto xy1 = predict(record, "ultimate_strength_xy_pa", after);
    const auto z1  = predict(record, "ultimate_strength_z_pa", after);
    if (!xy0.experimental || !z0.experimental || !xy1.experimental || !z1.experimental)
        return factor;
    bool replaced = false;
    for (const auto& [key, value] : estimated_factors)
        if (value > 0 && xy0.parameters.count(key) && z0.parameters.count(key) && xy1.parameters.count(key) && z1.parameters.count(key)) {
            factor /= value;
            replaced = true;
        }
    // Use the weaker directional improvement; these remain estimates, not a new solve.
    return replaced ? factor * std::min(xy1.value / xy0.value, z1.value / z0.value) : factor;
}
} // namespace Slic3r::MaterialExperiments
