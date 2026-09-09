#include "LoadCalibration.hpp"
#include "ClipperUtils.hpp"
#include "Print.hpp"
#include "Format/bbs_3mf.hpp"
#include <boost/filesystem.hpp>
#include "nlohmann/json.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <set>
#include <cctype>

namespace Slic3r::LoadCalibration {
const std::vector<Setting>& settings()
{
    static const std::vector<Setting>
        list{{"sparse_infill_pattern"},  {"sparse_infill_density"}, {"wall_loops"},       {"top_surface_pattern"},
             {"bottom_surface_pattern"}, {"layer_height"},          {"line_width"},       {"slow_down_layer_time"},
             {"nozzle_temperature"},     {"bed_temperature"},       {"curr_bed_type"},    {"top_shell_layers"},
             {"bottom_shell_layers"},    {"outer_wall_speed"},      {"inner_wall_speed"}, {"sparse_infill_speed"},
             {"fan_max_speed"},          {"filament_flow_ratio"},   {"infill_direction"}};
    return list;
}
std::vector<std::string> range(double start, double end, double step)
{
    if (!std::isfinite(start) || !std::isfinite(end) || !std::isfinite(step) || step == 0 || (end - start) * step < 0)
        throw std::invalid_argument("Range requires finite values and a nonzero step toward the end.");
    const double count = (end - start) / step;
    if (!std::isfinite(count) || count > 49)
        throw std::invalid_argument("Use at most 50 setting values.");
    std::vector<std::string> values;
    for (int i = 0; i <= int(std::floor(count + 1e-9)); ++i) {
        std::ostringstream s;
        s.imbue(std::locale::classic());
        s << std::setprecision(12) << start + i * step;
        values.push_back(s.str());
    }
    return values;
}
DynamicPrintConfig setting_config(const DynamicPrintConfig& baseline, const std::string& key, const std::string& value)
{
    if (std::none_of(settings().begin(), settings().end(), [&](const Setting& s) { return s.key == key; }))
        throw std::invalid_argument("Unsupported calibration setting.");
    DynamicPrintConfig result(baseline);
    auto assign = [&](const std::string& option_key) {
        const auto* def = print_config_def.get(option_key);
        if (!def)
            throw std::invalid_argument("Unknown setting: " + option_key);
        if (def->type != coEnum) {
            size_t used    = 0;
            const double n = std::stod(value, &used);
            if (used != value.size() || !std::isfinite(n) || n < def->min || n > def->max ||
                ((def->type == coInt || def->type == coInts) && std::floor(n) != n))
                throw std::invalid_argument("Value outside the permitted range for " + option_key);
        }
        result.set_deserialize_strict(option_key, value);
    };
    if (key == "bed_temperature") {
        for (const auto& k : {"hot_plate_temp", "textured_plate_temp", "cool_plate_temp", "textured_cool_plate_temp", "eng_plate_temp",
                              "supertack_plate_temp"})
            if (print_config_def.get(k)) {
                assign(k);
                assign(std::string(k) + "_initial_layer");
            }
    } else {
        assign(key);
        if (key == "nozzle_temperature")
            assign("nozzle_temperature_initial_layer");
    }
    if (key == "layer_height" && result.opt_float("layer_height") <= 0)
        throw std::invalid_argument("Layer height must be positive.");
    // Generated studies are single-filament, single-extruder jobs using filament 1.
    result.set_deserialize_strict("print_sequence", "by layer");
    return result;
}
void validate_sample(const Sample& s)
{
    if (s.id.empty() || s.id.size() > 32 ||
        !std::all_of(s.id.begin(), s.id.end(), [](unsigned char c) { return std::isalnum(c) || c == '-'; }))
        throw std::invalid_argument("Invalid hook ID.");
    if (s.orientation != "XY" && s.orientation != "Z")
        throw std::invalid_argument("Unknown orientation.");
    if (s.outcome != Outcome::NotTested && s.outcome != Outcome::Failed && s.outcome != Outcome::Survived)
        throw std::invalid_argument("Unknown test outcome.");
    if (s.outcome != Outcome::NotTested && (!s.load_n || !std::isfinite(*s.load_n) || *s.load_n <= 0))
        throw std::invalid_argument("Failed and survived hooks require a positive measured load.");
    if (s.load_n && (!std::isfinite(*s.load_n) || *s.load_n <= 0))
        throw std::invalid_argument("Load must be positive.");
    if (s.mass_g && (!std::isfinite(*s.mass_g) || *s.mass_g <= 0))
        throw std::invalid_argument("Printed mass must be positive.");
}
Statistics summarize(const std::vector<Sample>& samples, const std::string& value, const std::string& orientation)
{
    Statistics r;
    double m2 = 0, ratio = 0;
    size_t weighed = 0;
    for (const auto& s : samples) {
        if (s.value != value || s.orientation != orientation)
            continue;
        validate_sample(s);
        if (s.outcome == Outcome::NotTested) {
            ++r.untested;
            continue;
        }
        if (s.outcome == Outcome::Survived) {
            ++r.survived;
            continue;
        }
        const double n = *s.load_n;
        ++r.failed;
        const double delta = n - r.mean_n;
        r.mean_n += delta / r.failed;
        m2 += delta * (n - r.mean_n);
        r.minimum_n = r.failed == 1 ? n : std::min(n, r.minimum_n);
        if (s.mass_g) {
            ratio += n / *s.mass_g;
            ++weighed;
        }
    }
    if (r.failed > 1)
        r.sd_n = std::sqrt(m2 / (r.failed - 1));
    if (weighed)
        r.mean_n_per_g = ratio / weighed;
    return r;
}
std::vector<Sample> make_samples(const std::vector<std::string>& values, int repeats)
{
    if (values.empty() || values.size() > 50 || repeats < 1 || repeats > 20 || values.size() * repeats * 2 > 200)
        throw std::invalid_argument("Use 1–50 unique values, 1–20 repeats, and at most 200 hooks per study.");
    if (std::set<std::string>(values.begin(), values.end()).size() != values.size())
        throw std::invalid_argument("Setting values must be unique.");
    std::vector<Sample> result;
    for (const auto& v : values)
        for (const auto& o : {"XY", "Z"})
            for (int i = 0; i < repeats; ++i) {
                std::ostringstream id;
                id << "H" << std::setw(3) << std::setfill('0') << result.size() + 1 << "-" << o;
                Sample s;
                s.id          = id.str();
                s.value       = v;
                s.orientation = o;
                result.push_back(s);
            }
    return result;
}
Model hook_model(const std::string& stl, const Sample& sample, const DynamicPrintConfig& config)
{
    Model model = Model::read_from_file(stl);
    if (model.objects.size() != 1)
        throw std::runtime_error("CNC Testhook must contain exactly one object.");
    auto* object = model.objects.front();
    object->name = sample.id + " " + sample.value;
    if (sample.orientation == "Z")
        object->rotate(0.5 * M_PI, X);
    if (object->instances.empty())
        object->add_instance();
    object->center_around_origin();
    object->ensure_on_bed();
    object->config.set_key_value("extruder", new ConfigOptionInt(1));
    const auto* area = config.option<ConfigOptionPoints>("printable_area");
    if (!area || area->values.size() < 3)
        throw std::runtime_error("Choose a printer with a printable bed.");
    Polygon bed;
    for (const auto& p : area->values)
        bed.points.emplace_back(Point::new_scale(p.x(), p.y()));
    Polygons usable{bed};
    const auto* excluded = config.option<ConfigOptionPoints>("bed_exclude_area");
    if (excluded && excluded->values.size() >= 3) {
        Polygon exclusion;
        for (const auto& p : excluded->values)
            exclusion.points.emplace_back(Point::new_scale(p.x(), p.y()));
        usable = diff(usable, Polygons{exclusion});
    }
    const auto box = object->bounding_box_exact();
    if (box.size().z() > config.opt_float("printable_height"))
        throw std::runtime_error("Hook exceeds printer height.");
    BoundingBoxf bounds(area->values);
    // Reserve 12 mm around the hook for adhesion and keep the whole rectangle inside the actual bed polygon.
    for (double y = bounds.min.y() + 12; y + box.size().y() + 12 <= bounds.max.y(); y += 5)
        for (double x = bounds.min.x() + 12; x + box.size().x() + 12 <= bounds.max.x(); x += 5) {
            Polygon footprint;
            footprint.points = {Point::new_scale(x - 12, y - 12), Point::new_scale(x + box.size().x() + 12, y - 12),
                                Point::new_scale(x + box.size().x() + 12, y + box.size().y() + 12),
                                Point::new_scale(x - 12, y + box.size().y() + 12)};
            if (diff(Polygons{footprint}, usable).empty()) {
                object->instances.front()->set_offset(Vec3d(x - box.min.x(), y - box.min.y(), 0));
                object->ensure_on_bed();
                return model;
            }
        }
    throw std::runtime_error("The CNC Testhook and its clearance do not fit this printer bed.");
}
void prepare_print(Print& print, const Model& model, const DynamicPrintConfig& config)
{
    // Match the normal background slicer and CLI: validation and G-code depend on this flag.
    const auto* printer = config.option<ConfigOptionString>("printer_model");
    print.is_BBL_printer() = printer && printer->value.compare(0, 9, "Bambu Lab") == 0;
    print.set_plate_index(0);
    print.set_plate_origin(Vec3d::Zero());
    print.apply(model, config);
}
namespace {
bool object_setting(const std::string& key)
{
    return PrintObjectConfig::defaults().has(key) || PrintRegionConfig::defaults().has(key);
}
const std::vector<std::string>& plate_keys()
{
    static const std::vector<std::string> keys = [] {
        std::vector<std::string> keys;
        for (const auto& setting : settings())
            if (setting.key != "bed_temperature" && !object_setting(setting.key)) keys.push_back(setting.key);
        keys.push_back("nozzle_temperature_initial_layer");
        for (const auto& key : {"hot_plate_temp", "textured_plate_temp", "cool_plate_temp", "textured_cool_plate_temp",
                               "eng_plate_temp", "supertack_plate_temp"}) {
            keys.emplace_back(key); keys.push_back(std::string(key)+"_initial_layer");
        }
        return keys;
    }();
    return keys;
}
}
std::string serialize_plate_settings(const DynamicPrintConfig& config)
{
    nlohmann::json options = nlohmann::json::object();
    for (const auto& key : plate_keys())
        if (config.has(key)) options[key] = config.opt_serialize(key);
    return options.empty() ? std::string() : nlohmann::json{{"version",1},{"options",options}}.dump();
}
DynamicPrintConfig deserialize_plate_settings(const std::string& text)
{
    const auto json = nlohmann::json::parse(text);
    if (json.at("version") != 1 || !json.at("options").is_object())
        throw std::invalid_argument("Unsupported load-calibration plate settings.");
    DynamicPrintConfig result;
    for (auto it = json.at("options").begin(); it != json.at("options").end(); ++it) {
        if (std::find(plate_keys().begin(),plate_keys().end(),it.key()) == plate_keys().end())
            throw std::invalid_argument("Unsupported load-calibration plate option.");
        result.set_deserialize_strict(it.key(), it.value().get<std::string>());
    }
    return result;
}
HookProject arrange_hooks(const std::string& stl, const std::vector<Sample>& samples,
    const DynamicPrintConfig& baseline, const std::string& setting, size_t max_plates)
{
    if (samples.empty() || samples.size() > 200 || max_plates == 0)
        throw std::invalid_argument("Select between 1 and 200 hooks.");
    HookProject project;
    project.config = setting_config(baseline, setting, samples.front().value);
    const auto* area = baseline.option<ConfigOptionPoints>("printable_area");
    if (!area || area->values.size()<3) throw std::invalid_argument("A printable bed is required.");
    const BoundingBoxf bounds(area->values);
    Polygon bed; for (const auto& p : area->values) bed.points.emplace_back(Point::new_scale(p.x(),p.y()));
    Polygons usable{bed};
    const auto* excluded = baseline.option<ConfigOptionPoints>("bed_exclude_area");
    if (excluded && excluded->values.size() >= 3) {
        Polygon polygon;
        for (const auto& p : excluded->values) polygon.points.emplace_back(Point::new_scale(p.x(),p.y()));
        usable = diff(usable,Polygons{polygon});
    }
    std::vector<Polygons> occupied;
    std::vector<std::string> plate_values;
    const bool per_object = object_setting(setting);
    const bool isolated = setting == "slow_down_layer_time";
    for (const auto& sample : samples) {
        validate_sample(sample);
        const auto config = setting_config(baseline,setting,sample.value);
        auto hook = hook_model(stl,sample,config);
        auto* object = project.model.add_object(*hook.objects.front());
        if (per_object) object->config.set_key_value(setting,config.option(setting)->clone());
        const auto box = object->bounding_box_exact();
        bool placed = false;
        for (size_t plate=0; !placed && plate<=project.plates.size(); ++plate) {
            if (plate == project.plates.size()) {
                if (plate >= max_plates) throw std::invalid_argument("Selected hooks require too many plates. Open a smaller selection.");
                HookPlate item;
                if (!per_object) {
                    for (const auto& key : baseline.diff(config))
                        if (key != "print_sequence") item.config.set_key_value(key,config.option(key)->clone());
                    // Also retain the first value when it equals the baseline.
                    for (const auto& key : plate_keys())
                        if (config.has(key)) item.config.set_key_value(key,config.option(key)->clone());
                }
                project.plates.push_back(std::move(item)); occupied.emplace_back(); plate_values.push_back(sample.value);
            }
            if ((!per_object && plate_values[plate] != sample.value) || (isolated && !occupied[plate].empty())) continue;
            for (double y=bounds.min.y()+12; !placed && y+box.size().y()+12<=bounds.max.y(); y+=5)
                for (double x=bounds.min.x()+12; !placed && x+box.size().x()+12<=bounds.max.x(); x+=5) {
                    Polygon footprint;
                    footprint.points={Point::new_scale(x-12,y-12),Point::new_scale(x+box.size().x()+12,y-12),
                        Point::new_scale(x+box.size().x()+12,y+box.size().y()+12),Point::new_scale(x-12,y+box.size().y()+12)};
                    if (!diff(Polygons{footprint},usable).empty() || !intersection(Polygons{footprint},occupied[plate]).empty()) continue;
                    auto* instance=object->instances.front();
                    instance->set_offset(instance->get_offset()+Vec3d(x-box.min.x(),y-box.min.y(),0));
                    object->invalidate_bounding_box();
                    occupied[plate].push_back(footprint); project.plates[plate].objects.push_back(project.model.objects.size()-1);
                    placed=true;
                }
        }
    }
    // Same logical plate grid as PartPlateList: ceil(sqrt(count)), 20% spacing.
    const int columns=int(std::ceil(std::sqrt(double(project.plates.size()))));
    for (size_t i=0;i<project.plates.size();++i) {
        auto& plate=project.plates[i];
        plate.origin=Vec3d(int(i)%columns*bounds.size().x()*1.2,-int(i)/columns*bounds.size().y()*1.2,0);
        for (size_t index:plate.objects) {
            auto* object=project.model.objects[index]; auto* instance=object->instances.front();
            instance->set_offset(instance->get_offset()+plate.origin); object->invalidate_bounding_box();
        }
    }
    return project;
}
void store_project(const std::string& path, HookProject& project)
{
    const auto backup=boost::filesystem::path(path).parent_path()/boost::filesystem::unique_path(".hook-export-%%%%-%%%%");
    boost::filesystem::create_directories(backup); project.model.set_backup_path(backup.string());
    std::vector<PlateData> plates(project.plates.size());
    StoreParams params; params.path=path; params.model=&project.model; params.config=&project.config;
    params.strategy=SaveStrategy::Zip64|SaveStrategy::Silence;
    for (size_t i=0;i<plates.size();++i) {
        auto& plate=plates[i]; plate.plate_index=int(i); plate.plate_name="CNC Testhooks "+std::to_string(i+1);
        plate.config=project.plates[i].config;
        for (size_t index:project.plates[i].objects) plate.objects_and_instances.emplace_back(int(index),0);
        params.plate_data_list.push_back(&plate);
    }
    try { if (!store_bbs_3mf(params)) throw std::runtime_error("Could not save selected hooks."); }
    catch (...) { boost::filesystem::remove_all(backup); throw; }
    boost::filesystem::remove_all(backup);
}
void store_project(const std::string& path, Model& model, DynamicPrintConfig& config)
{
    const auto backup = boost::filesystem::path(path).parent_path() / boost::filesystem::unique_path(".hook-export-%%%%-%%%%");
    boost::filesystem::create_directories(backup);
    model.set_backup_path(backup.string());
    PlateData plate;
    plate.plate_index           = 0;
    plate.plate_name            = model.objects.front()->name;
    plate.objects_and_instances = {{0, 0}};
    StoreParams params;
    params.path            = path;
    params.model           = &model;
    params.config          = &config;
    params.plate_data_list = {&plate};
    params.strategy        = SaveStrategy::Zip64 | SaveStrategy::Silence;
    try {
        if (!store_bbs_3mf(params))
            throw std::runtime_error("Could not save hook project.");
    } catch (...) {
        boost::filesystem::remove_all(backup);
        throw;
    }
    boost::filesystem::remove_all(backup);
}
StrengthAnalysis::Material calibrate(const StrengthAnalysis::Material& base,
                                     const std::string& name,
                                     const std::vector<Sample>& samples,
                                     const std::string& value,
                                     double xy,
                                     double z)
{
    if (name.empty() || !std::isfinite(xy) || !std::isfinite(z) || xy <= 0 || z <= 0)
        throw std::invalid_argument("Enter a material name and validated XY/Z peak tensile stress per newton.");
    const auto a = summarize(samples, value, "XY"), b = summarize(samples, value, "Z");
    if (a.failed < 2 || b.failed < 2 || a.survived || b.survived)
        throw std::invalid_argument(
            "Calibration needs at least two failures per orientation and no censored (survived) results in the selected group.");
    auto m        = base.calibrated();
    m.calibration = {};
    m.name        = name + " -- CALIBRATED";
    // Conservative empirical peak tensile limits. Yield, modulus and shear are not measured by this test.
    m.ultimate_strength_xy_pa = a.minimum_n * xy * 1e6;
    m.ultimate_strength_z_pa  = b.minimum_n * z * 1e6;
    m.yield_strength_xy_pa    = std::min(m.yield_strength_xy_pa, m.ultimate_strength_xy_pa);
    m.yield_strength_z_pa     = std::min(m.yield_strength_z_pa, m.ultimate_strength_z_pa);
    m.calibration.source =
        "CNC Testhook: minimum failure load × supplied fixture stress/force; yield capped at ultimate; modulus/shear unchanged.";
    m.provenance = m.calibration.source + " Setting value: " + value;
    if (!m.validate().empty())
        throw std::invalid_argument("Calculated material properties are invalid.");
    return m;
}
std::string serialize_samples(const std::vector<Sample>& samples)
{
    nlohmann::json j = nlohmann::json::array();
    for (const auto& s : samples) {
        validate_sample(s);
        nlohmann::json row = {{"id", s.id},
                              {"value", s.value},
                              {"orientation", s.orientation},
                              {"outcome", int(s.outcome)},
                              {"notes", s.notes}};
        row["load_n"]      = s.load_n ? nlohmann::json(*s.load_n) : nlohmann::json(nullptr);
        row["mass_g"]      = s.mass_g ? nlohmann::json(*s.mass_g) : nlohmann::json(nullptr);
        j.push_back(row);
    }
    return j.dump();
}
std::vector<Sample> deserialize_samples(const std::string& text)
{
    const auto j = nlohmann::json::parse(text);
    if (!j.is_array() || j.size() > 200)
        throw std::invalid_argument("Invalid sample list.");
    std::vector<Sample> result;
    std::set<std::string> ids;
    for (const auto& row : j) {
        Sample s;
        s.id          = row.at("id");
        s.value       = row.at("value");
        s.orientation = row.at("orientation");
        s.outcome     = Outcome(row.at("outcome").get<int>());
        s.notes       = row.value("notes", "");
        if (!row.at("load_n").is_null())
            s.load_n = row.at("load_n").get<double>();
        if (!row.at("mass_g").is_null())
            s.mass_g = row.at("mass_g").get<double>();
        validate_sample(s);
        if (s.id.empty() || !ids.insert(s.id).second)
            throw std::invalid_argument("Duplicate or empty hook ID.");
        result.push_back(s);
    }
    return result;
}
} // namespace Slic3r::LoadCalibration
