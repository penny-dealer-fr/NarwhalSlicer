#include "LoadCalibrationPanel.hpp"
#include "GUI_App.hpp"
#include "I18N.hpp"
#include "Plater.hpp"
#include "MainFrame.hpp"
#include "libslic3r/LoadCalibration.hpp"
#include "libslic3r/MaterialExperiments.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/Print.hpp"

#include "libslic3r/Utils.hpp"
#include "nlohmann/json.hpp"
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <wx/button.h>
#include <wx/choice.h>
#include <wx/checklst.h>
#include <wx/clrpicker.h>
#include <wx/grid.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/timer.h>
#include <wx/filename.h>
#include <wx/filedlg.h>
#include <wx/dcbuffer.h>
#include <wx/checkbox.h>
#include <wx/textdlg.h>
#include <wx/statbox.h>
#include <atomic>
#include <mutex>
#include <thread>
#include <sstream>
#include <set>
#include <cmath>
#include <algorithm>

namespace Slic3r::GUI {
namespace LC = LoadCalibration;
namespace SA = StrengthAnalysis;
namespace ME = MaterialExperiments;
using Json   = nlohmann::json;
namespace {
boost::filesystem::path root_path() { return boost::filesystem::path(data_dir()) / "load_calibration"; }
Json read_json(const boost::filesystem::path& path)
{
    boost::filesystem::ifstream file(path);
    if (!file)
        throw std::runtime_error("Cannot read " + path.string());
    Json j;
    file >> j;
    return j;
}
void write_json(const boost::filesystem::path& path, const Json& json)
{
    boost::filesystem::create_directories(path.parent_path());
    auto temp = path;
    temp += ".tmp";
    {
        boost::filesystem::ofstream out(temp);
        out << json.dump(2);
        out.flush();
        if (!out)
            throw std::runtime_error("Cannot save " + path.string());
    }
    // wxRenameFile replaces an existing destination on Windows as well as Unix.
    if (!wxRenameFile(wxString::FromUTF8(temp.string()), wxString::FromUTF8(path.string()), true))
        throw std::runtime_error("Cannot replace " + path.string());
}
std::vector<SA::Material>& catalog()
{
    static std::vector<SA::Material> result;
    return result;
}
void refresh_catalog()
{
    auto result = SA::builtin_materials();
    auto path   = root_path() / "materials.json";
    if (boost::filesystem::exists(path)) {
        const auto j = read_json(path);
        if (j.at("version") != 1)
            throw std::runtime_error("Unsupported material library version.");
        for (const auto& row : j.at("materials")) {
            SA::Setup setup;
            std::string error;
            if (!SA::deserialize_setup(row.at("setup").dump(), setup, &error))
                throw std::runtime_error(error);
            result.push_back(setup.material);
        }
    }
    catalog() = std::move(result);
}
std::string utf8(wxTextCtrl* field) { return field->GetValue().ToUTF8().data(); }
double number(wxTextCtrl* field)
{
    double value;
    if (!field->GetValue().ToDouble(&value) || !std::isfinite(value))
        throw std::invalid_argument("Enter a finite number.");
    return value;
}
} // namespace
const std::vector<SA::Material>& load_materials()
{
    if (catalog().empty()) {
        try {
            refresh_catalog();
        } catch (const std::exception& e) {
            catalog() = SA::builtin_materials();
            wxLogWarning("%s", wxString::FromUTF8(e.what()));
        }
    }
    return catalog();
}
struct LoadCalibrationPanel::Impl
{
    LoadCalibrationPanel* panel;
    wxChoice *setting{}, *study_choice{}, *unit{}, *base{}, *target{}, *group{}, *calculation{};
    wxTextCtrl *values{}, *start{}, *end{}, *step{}, *material{}, *type{}, *xy{}, *z{}, *summary{};
    wxCheckListBox* categories{};
    wxChoice* related_mode{};
    wxTextCtrl* related_values{};
    wxStaticText* related_label{};
    std::vector<wxWindow*> numeric_controls, categorical_controls, related_controls;
    std::vector<std::string> category_values;
    std::string related_key;
    wxSpinCtrl* repeats{};
    wxColourPickerCtrl* color{};
    std::string legacy_color;
    wxGrid* grid{};
    wxStaticText *status{}, *help{};
    wxButton *generate{}, *save{}, *calibrate_button{};
    wxTimer timer;
    Json study;
    std::vector<boost::filesystem::path> studies;
    std::vector<LC::Sample> samples;
    boost::filesystem::path current_path;
    std::thread worker;
    std::atomic<bool> cancel{false}, done{false};
    std::mutex mutex;
    std::shared_ptr<Print> active_print;
    std::string progress;
    bool busy{false};
    int displayed_unit{0};

    explicit Impl(LoadCalibrationPanel* p) : panel(p), timer(p) {}
    ~Impl()
    {
        stop();
        if (worker.joinable())
            worker.join();
    }
    std::string color_value() const
    {
        return legacy_color.empty() ? std::string(color->GetColour().GetAsString(wxC2S_HTML_SYNTAX).ToUTF8().data()) : legacy_color;
    }
    void set_color(const std::string& saved)
    {
        wxColour parsed(wxString::FromUTF8(saved));
        legacy_color = parsed.IsOk() ? std::string() : saved;
        color->SetColour(parsed.IsOk() ? parsed : *wxWHITE);
        color->SetToolTip(legacy_color.empty() ?
                              _L("Choose material color") :
                              wxString::Format(_L("Saved color: %s. Choose a swatch to replace it."), wxString::FromUTF8(saved)));
    }
    DynamicPrintConfig baseline_config() const
    {
        DynamicPrintConfig baseline;
        for (auto it = study.at("baseline").begin(); it != study.at("baseline").end(); ++it)
            baseline.set_deserialize_strict(it.key(), it.value().get<std::string>());
        return baseline;
    }
    void stop()
    {
        cancel = true;
        std::lock_guard<std::mutex> lock(mutex);
        if (active_print)
            active_print->cancel();
    }
    void message(const std::string& text)
    {
        std::lock_guard<std::mutex> lock(mutex);
        progress = text;
    }
    template<class F> void guarded(F f)
    {
        try {
            f();
        } catch (const std::exception& e) {
            wxMessageBox(wxString::FromUTF8(e.what()), _L("Load calibration"), wxOK | wxICON_ERROR, panel);
        }
    }
    void refresh_studies()
    {
        studies.clear();
        study_choice->Clear();
        const auto root = root_path();
        if (boost::filesystem::exists(root))
            for (const auto& entry : boost::filesystem::directory_iterator(root))
                if (boost::filesystem::is_directory(entry) && boost::filesystem::exists(entry.path() / "study.json"))
                    studies.push_back(entry.path());
        std::sort(studies.begin(), studies.end());
        for (const auto& path : studies)
            study_choice->Append(wxString::FromUTF8(path.filename().string()));
        for (size_t i = 0; i < studies.size(); ++i)
            if (studies[i] == current_path)
                study_choice->SetSelection(int(i));
    }
    void refresh_targets()
    {
        base->Clear();
        target->Clear();
        target->Append(_L("New calibrated material"));
        for (const auto& item : load_materials())
            base->Append(wxString::FromUTF8(item.name));
        for (size_t i = SA::builtin_materials().size(); i < load_materials().size(); ++i)
            target->Append(wxString::FromUTF8(load_materials()[i].name));
        base->SetSelection(0);
        target->SetSelection(0);
    }
    static std::vector<std::string> split_values(const std::string& text)
    {
        std::vector<std::string> list;
        std::istringstream input(text);
        std::string value;
        while (std::getline(input, value, ',')) {
            const auto a = value.find_first_not_of(" \t\r\n"), b = value.find_last_not_of(" \t\r\n");
            if (a != std::string::npos)
                list.push_back(value.substr(a, b - a + 1));
        }
        return list;
    }
    void related_help()
    {
        related_values->Enable(related_mode->GetSelection() != 0);
        if (related_mode->GetSelection() == 1) {
            const auto list = split_values(utf8(related_values));
            if (list.size() > 1)
                related_values->ChangeValue(wxString::FromUTF8(list.front()));
        }
    }
    void setting_help(bool restoring = false)
    {
        const std::string key = LC::settings()[setting->GetSelection()].key;
        const auto* def        = print_config_def.get(key);
        const bool categorical = def && (def->type == coEnum || def->type == coBool);
        for (auto* control : numeric_controls)
            control->Show(!categorical);
        for (auto* control : categorical_controls)
            control->Show(categorical);
        category_values.clear();
        categories->Clear();
        if (categorical) {
            category_values     = def->type == coBool ? std::vector<std::string>{"0", "1"} : def->enum_values;
            const auto selected = split_values(utf8(values));
            for (size_t i = 0; i < category_values.size(); ++i) {
                categories->Append(def->type == coBool ?
                                       (i == 0 ? _L("No") : _L("Yes")) :
                                       (i < def->enum_labels.size() ? _(def->enum_labels[i]) : wxString::FromUTF8(category_values[i])));
                categories->Check(unsigned(i),
                                  restoring ? std::find(selected.begin(), selected.end(), category_values[i]) != selected.end() : i < 2);
            }
        }
        related_key = key == "sparse_infill_pattern"            ? "sparse_infill_density" :
                      key == "alternate_extra_wall"             ? "wall_loops" :
                      key == "inner_wall_flow_ratio"            ? "inner_wall_line_width" :
                      key == "sparse_infill_flow_ratio"         ? "sparse_infill_line_width" :
                      key == "internal_solid_infill_flow_ratio" ? "internal_solid_infill_line_width" :
                                                                  "";
        for (auto* control : related_controls)
            control->Show(!related_key.empty());
        if (!related_key.empty()) {
            related_label->SetLabel(_L("Related parameter: ") + _(print_config_def.get(related_key)->label));
            related_mode->SetSelection(0);
            related_values->ChangeValue(related_key == "sparse_infill_density" ? "20,30,50" :
                                        related_key == "wall_loops"            ? "2,3,4" :
                                                                                 "0.4,0.45,0.5");
            if (restoring && !samples.empty() && samples.front().related.count(related_key)) {
                std::set<std::string> seen;
                wxString joined;
                for (const auto& sample : samples) {
                    const auto it = sample.related.find(related_key);
                    if (it != sample.related.end() && seen.insert(it->second).second) {
                        if (!joined.empty())
                            joined += ",";
                        joined += wxString::FromUTF8(it->second);
                    }
                }
                related_mode->SetSelection(seen.size() > 1 ? 2 : 1);
                related_values->ChangeValue(joined);
            }
        }
        related_help();
        help->SetLabel(categorical ? _L("Check every value to test. Related sweeps print every selected combination in both XY and Z.") :
                                     _L("Enter comma-separated values, or fill the numeric range and click Use range."));
        if (key == "alternate_extra_wall")
            help->SetLabel(help->GetLabel() + _L(" Ensure vertical shell thickness uses Moderate for both No and Yes when Prepare is set to All."));
        help->Wrap(panel->FromDIP(850));
        panel->Layout();
        panel->FitInside();
    }
    void table()
    {
        if (grid->GetNumberRows())
            grid->DeleteRows(0, grid->GetNumberRows());
        if (!samples.empty())
            grid->AppendRows(int(samples.size()));
        group->Clear();
        std::set<std::string> seen;
        for (size_t i = 0; i < samples.size(); ++i) {
            const auto& s = samples[i];
            grid->SetCellValue(int(i), 0, wxString::FromUTF8(s.id));
            grid->SetCellValue(int(i), 1, wxString::FromUTF8(LC::sample_label(s)));
            grid->SetCellValue(int(i), 2, wxString::FromUTF8(s.orientation));
            for (int c = 0; c < 3; ++c)
                grid->SetReadOnly(int(i), c);
            wxArrayString outcomes;
            outcomes.Add(_L("Not tested"));
            outcomes.Add(_L("Failed"));
            outcomes.Add(_L("Did not break"));
            grid->SetCellEditor(int(i), 3, new wxGridCellChoiceEditor(outcomes));
            grid->SetCellValue(int(i), 3, outcomes[int(s.outcome)]);
            if (s.load_n)
                grid->SetCellValue(int(i), 4, wxString::Format("%.6g", *s.load_n));
            if (s.mass_g)
                grid->SetCellValue(int(i), 5, wxString::Format("%.6g", *s.mass_g));
            grid->SetCellValue(int(i), 6, wxString::FromUTF8(s.notes));
            if (seen.insert(LC::sample_label(s)).second)
                group->Append(wxString::FromUTF8(LC::sample_label(s)));
        }
        unit->SetSelection(0);
        displayed_unit = 0;
        if (group->GetCount())
            group->SetSelection(0);
    }
    void collect()
    {
        grid->SaveEditControlValue();
        auto updated        = samples;
        const double factor = displayed_unit == 1 ? 9.80665 : displayed_unit == 2 ? 4.4482216152605 : 1.;
        for (size_t i = 0; i < updated.size(); ++i) {
            auto& s         = updated[i];
            const auto text = grid->GetCellValue(int(i), 3);
            s.outcome       = text == _L("Failed")        ? LC::Outcome::Failed :
                              text == _L("Did not break") ? LC::Outcome::Survived :
                                                            LC::Outcome::NotTested;
            auto read       = [&](int col) -> std::optional<double> {
                const auto text = grid->GetCellValue(int(i), col);
                if (text.IsEmpty())
                    return {};
                double v;
                if (!text.ToDouble(&v) || !std::isfinite(v) || v <= 0)
                    throw std::invalid_argument(s.id + ": enter a positive number or leave blank.");
                return v;
            };
            s.load_n = read(4);
            if (s.load_n)
                *s.load_n *= factor;
            s.mass_g = read(5);
            s.notes  = grid->GetCellValue(int(i), 6).ToUTF8().data();
            LC::validate_sample(s);
        }
        samples = std::move(updated);
    }
    void report()
    {
        wxString text = _L("Failure results (N): survived loads are lower bounds and are excluded from failure statistics.") + "\n";
        for (unsigned i = 0; i < group->GetCount(); ++i) {
            for (const auto& axis : {"XY", "Z"}) {
                const auto value  = group->GetString(i);
                const auto result = LC::summarize(samples, value.ToUTF8().data(), axis);
                text += wxString::Format(_L("%s / %s: %u failed, %u survived, %u untested"), value, wxString::FromUTF8(axis),
                                         unsigned(result.failed), unsigned(result.survived), unsigned(result.untested));
                if (result.failed)
                    text += wxString::Format(_L("; mean %.3f, min %.3f, sample SD %.3f"), result.mean_n, result.minimum_n, result.sd_n);
                if (result.mean_n_per_g)
                    text += wxString::Format(_L("; mean N/g %.3f"), *result.mean_n_per_g);
                text += "\n";
            }
        }
        summary->ChangeValue(text);
    }
    void persist(bool metadata = true)
    {
        if (current_path.empty())
            throw std::invalid_argument("Generate or open a study first.");
        collect();
        study["samples"] = Json::parse(LC::serialize_samples(samples));
        if (metadata) {
            study["material"]     = utf8(material);
            study["type"]         = utf8(type);
            study["color"]        = color_value();
            study["xy_mpa_per_n"] = utf8(xy);
            study["z_mpa_per_n"]  = utf8(z);
            if (base->GetSelection() >= 0)
                study["base_key"] = load_materials().at(base->GetSelection()).key;
            study["selected_value"] = group->GetStringSelection().ToUTF8().data();
        }
        write_json(current_path / "study.json", study);
        report();
    }
    void open()
    {
        if (busy)
            return;
        int index = study_choice->GetSelection();
        if (index < 0)
            return;
        if (!current_path.empty())
            persist();
        const auto path      = studies[index];
        const auto candidate = read_json(path / "study.json");
        if (candidate.at("version") != 1)
            throw std::invalid_argument("Unsupported study version.");
        auto loaded  = LC::deserialize_samples(candidate.at("samples").dump());
        study        = candidate;
        samples      = std::move(loaded);
        current_path = path;
        material->ChangeValue(wxString::FromUTF8(study.value("material", "")));
        type->ChangeValue(wxString::FromUTF8(study.value("type", "")));
        set_color(study.value("color", ""));
        xy->ChangeValue(wxString::FromUTF8(study.value("xy_mpa_per_n", "")));
        z->ChangeValue(wxString::FromUTF8(study.value("z_mpa_per_n", "")));
        table();
        for (size_t i = 0; i < LC::settings().size(); ++i)
            if (LC::settings()[i].key == study.at("setting").get<std::string>())
                setting->SetSelection(int(i));
        wxString restored_values;
        std::set<std::string> restored_seen;
        for (const auto& sample : samples) {
            if (!restored_seen.insert(sample.value).second)
                continue;
            if (!restored_values.empty())
                restored_values += ",";
            restored_values += wxString::FromUTF8(sample.value);
        }
        values->ChangeValue(restored_values);
        if (!samples.empty())
            repeats->SetValue(int(std::count_if(samples.begin(), samples.end(), [&](const LC::Sample& sample) {
                return LC::sample_label(sample) == LC::sample_label(samples.front()) && sample.orientation == "XY";
            })));
        for (size_t i = 0; i < load_materials().size(); ++i)
            if (load_materials()[i].key == study.value("base_key", ""))
                base->SetSelection(int(i));
        const auto selected = wxString::FromUTF8(study.value("selected_value", ""));
        if (group->FindString(selected) != wxNOT_FOUND)
            group->SetStringSelection(selected);
        setting_help(true);
        report();
        status->SetLabel(wxString::FromUTF8(path.string()));
    }
    void generate_study()
    {
        if (busy)
            return;
        if (!current_path.empty())
            persist(false);
        if (utf8(material).empty() || utf8(type).empty() || color_value().empty())
            throw std::invalid_argument("Enter material name, type, and color.");
        auto list = split_values(utf8(values));
        if (categories->IsShown()) {
            list.clear();
            for (unsigned i = 0; i < categories->GetCount(); ++i)
                if (categories->IsChecked(i))
                    list.push_back(category_values.at(i));
        }
        const auto related = !related_key.empty() && related_mode->GetSelection() != 0 ? split_values(utf8(related_values)) :
                                                                                         std::vector<std::string>{};
        if (!related_key.empty() && related_mode->GetSelection() == 1 && related.size() != 1)
            throw std::invalid_argument("Enter one related value for Fixed, or choose Sweep for multiple values.");
        const bool use_related = !related_key.empty() && related_mode->GetSelection() != 0;
        auto planned           = LC::make_samples(list, repeats->GetValue(), use_related ? related_key : "", related);
        const auto baseline    = wxGetApp().preset_bundle->full_config_secure();
        const std::string key = LC::settings()[setting->GetSelection()].key;
        std::vector<DynamicPrintConfig> configs;
        for (const auto& sample : planned)
            configs.push_back(LC::sample_config(baseline, key, sample));
        const auto stl = (boost::filesystem::path(resources_dir()) / "handy_models" / "CNC_Testhook.stl").string();
        // Validate geometry before creating a persistent study or starting the worker.
        LC::hook_model(stl, planned.front(), configs.front());
        LC::hook_model(stl, planned.back(), configs.back());
        auto path     = root_path() / boost::filesystem::unique_path("study-%%%%-%%%%-%%%%");
        Json snapshot = Json::object();
        for (const auto& k : baseline.keys())
            snapshot[k] = baseline.opt_serialize(k);
        study = {{"version", 1},
                 {"setting", key},
                 {"baseline", snapshot},
                 {"material", utf8(material)},
                 {"type", utf8(type)},
                 {"color", color_value()},
                 {"samples", Json::parse(LC::serialize_samples(planned))},
                 {"layout", "One hook per plate; XY flat, Z rotated 90 degrees around X; filament 1"}};
        boost::filesystem::create_directories(path);
        boost::filesystem::copy_file(stl, path / "CNC_Testhook.stl");
        write_json(path / "study.json", study);
        samples      = planned;
        current_path = path;
        table();
        report();
        refresh_studies();
        run_worker(std::move(configs), (path / "CNC_Testhook.stl").string());
    }
    void run_worker(std::vector<DynamicPrintConfig> configs, const std::string& stl)
    {
        if (worker.joinable())
            worker.join();
        cancel = false;
        done   = false;
        busy   = true;
        generate->Disable();
        study_choice->Disable();
        const auto planned = samples;
        const auto path    = current_path;
        worker             = std::thread([this, configs = std::move(configs), planned, path, stl]() mutable {
            size_t failed = 0;
            try {
                for (size_t i = 0; i < planned.size(); ++i) {
                    if (cancel)
                        throw CanceledException();
                    message("Slicing " + planned[i].id + " (" + std::to_string(i + 1) + "/" + std::to_string(planned.size()) + ")");
                    const auto stem = path / planned[i].id;
                    if (boost::filesystem::exists(stem.string() + ".complete.json") &&
                        boost::filesystem::exists(stem.string() + ".gcode") &&
                        read_json(stem.string() + ".complete.json").value("geometry_version", 0) == 2)
                        continue;
                    try {
                        auto model = LC::hook_model(stl, planned[i], configs[i]);
                        LC::store_project(stem.string() + ".3mf", model, configs[i]);
                        auto print = std::make_shared<Print>();
                        {
                            std::lock_guard<std::mutex> lock(mutex);
                            active_print = print;
                        }
                        print->set_status_silent();
                        print->auto_assign_extruders(model.objects.front());
                        LC::prepare_print(*print, model, configs[i]);
                        if (cancel) {
                            print->cancel();
                            throw CanceledException();
                        }
                        auto error = print->validate();
                        if (!error.string.empty())
                            throw std::runtime_error(error.string);
                        print->process();
                        print->export_gcode(stem.string() + ".gcode", nullptr);
                        if (cancel)
                            throw CanceledException();
                        write_json(stem.string() + ".complete.json", {{"id", planned[i].id},
                                                                      {"value", planned[i].value},
                                                                      {"orientation", planned[i].orientation},
                                                                      {"geometry_version", 2}});
                        boost::system::error_code ignored;
                        boost::filesystem::remove(stem.string() + ".error.json", ignored);
                    } catch (const CanceledException&) {
                        throw;
                    } catch (const std::exception& e) {
                        if (cancel)
                            throw CanceledException();
                        ++failed;
                        write_json(stem.string() + ".error.json", {{"id", planned[i].id}, {"error", e.what()}});
                    }
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        active_print.reset();
                    }
                }
                message(failed ?
                                        std::to_string(failed) + " hook(s) could not be sliced. See per-hook .error.json files in " + path.string() :
                                        "All hooks sliced. Projects and G-code: " + path.string());
            } catch (const std::exception& e) {
                message(std::string("Slicing stopped: ") + e.what() + ". Completed files remain in " + path.string());
            }
            {
                std::lock_guard<std::mutex> lock(mutex);
                active_print.reset();
            }
            done = true;
        });
        timer.Start(250);
    }
    void resume()
    {
        if (busy)
            return;
        persist();
        DynamicPrintConfig baseline;
        for (auto it = study.at("baseline").begin(); it != study.at("baseline").end(); ++it)
            baseline.set_deserialize_strict(it.key(), it.value().get<std::string>());
        std::vector<DynamicPrintConfig> configs;
        for (const auto& sample : samples)
            configs.push_back(LC::sample_config(baseline, study.at("setting").get<std::string>(), sample));
        run_worker(std::move(configs), (current_path / "CNC_Testhook.stl").string());
    }
    void save_material()
    {
        persist();
        if (base->GetSelection() < 0 || group->GetSelection() < 0)
            throw std::invalid_argument("Select a base material and setting value.");
        const std::string value = group->GetStringSelection().ToUTF8().data();
        const auto mode         = calculation->GetSelection() == 1 ? "simple" : "regressed";
        const auto path         = root_path() / "materials.json";
        Json library            = boost::filesystem::exists(path) ? read_json(path) : Json{{"version", 1}, {"materials", Json::array()}};
        if (library.at("version") != 1)
            throw std::runtime_error("Unsupported material library version.");
        auto calibrated    = load_materials().at(base->GetSelection());
        const int selected = target->GetSelection();
        if (selected > 0)
            calibrated = load_materials().at(SA::builtin_materials().size() + selected - 1);
        else
            calibrated.key = "calibrated_" + boost::filesystem::unique_path("%%%%-%%%%-%%%%").string();
        auto record = calibrated.experimental_data.empty() ? ME::create(load_materials().at(base->GetSelection())) :
                                                             Json::parse(calibrated.experimental_data);
        // Recover available observations from the earlier single-setting library format before appending new tests.
        if (selected > 0 && calibrated.experimental_data.empty()) {
            for (const auto& old : library["materials"]) {
                SA::Setup existing;
                if (!SA::deserialize_setup(old.at("setup").dump(), existing) || existing.material.key != calibrated.key)
                    continue;
                const auto old_source = old.value("study", "");
                const auto old_path   = root_path() / boost::filesystem::path(old_source).filename() / "study.json";
                if (!old_source.empty() && boost::filesystem::exists(old_path)) {
                    const auto previous = read_json(old_path);
                    DynamicPrintConfig config;
                    for (auto it = previous.at("baseline").begin(); it != previous.at("baseline").end(); ++it)
                        config.set_deserialize_strict(it.key(), it.value().get<std::string>());
                    ME::append_study(record, old_source, previous.at("setting"), ME::context_from_config(config),
                                     LC::deserialize_samples(previous.at("samples").dump()), old.value("xy_mpa_per_n", number(xy)),
                                     old.value("z_mpa_per_n", number(z)), "simple");
                }
            }
        }
        ME::append_study(record, current_path.filename().string(), study.at("setting"), ME::context_from_config(baseline_config()), samples,
                         number(xy), number(z), mode);
        calibrated.experimental_context                                         = ME::context_from_config(baseline_config());
        for (const auto& sample : samples)
            if (LC::sample_label(sample) == value) {
                calibrated.experimental_context[study.at("setting").get<std::string>()] = sample.value;
                for (const auto& [key, v] : sample.related)
                    calibrated.experimental_context[key] = v;
                break;
            }
        calibrated.experimental_data                                            = record.dump();
        calibrated                                                              = ME::evaluate(calibrated, calibrated.experimental_context);
        calibrated.name                                                         = utf8(material) + " -- CALIBRATED";
        calibrated.provenance         = "Accumulated experimental data; type: " + utf8(type) + "; color: " + color_value();
        calibrated.calibration        = {};
        calibrated.calibration.source = "Experimental material library: inspect Materials for measured and inherited properties.";
        SA::Setup setup;
        setup.material = calibrated;
        Json row       = {{"setup", Json::parse(SA::serialize_setup(setup))},
                          {"study", current_path.filename().string()},
                          {"type", utf8(type)},
                          {"color", color_value()},
                          {"value", value},
                          {"xy_mpa_per_n", number(xy)},
                          {"z_mpa_per_n", number(z)}};
        bool replaced  = false;
        for (auto& old : library["materials"]) {
            SA::Setup existing;
            if (SA::deserialize_setup(old.at("setup").dump(), existing) && existing.material.key == calibrated.key) {
                old      = row;
                replaced = true;
                break;
            }
        }
        if (!replaced)
            library["materials"].push_back(row);
        write_json(path, library);
        refresh_catalog();
        refresh_targets();
        status->SetLabel(_L("Calibrated material saved. Select it in Load → Study material."));
    }
};

LoadCalibrationPanel::LoadCalibrationPanel(wxWindow* parent) : wxScrolledWindow(parent, wxID_ANY), m(std::make_unique<Impl>(this))
{
    SetScrollRate(FromDIP(10), FromDIP(10));
    const int gap = FromDIP(8);
    auto* root    = new wxBoxSizer(wxVERTICAL);
    SetSizer(root);
    auto add_text = [&](const wxString& text) {
        auto* label = new wxStaticText(this, wxID_ANY, text);
        label->Wrap(FromDIP(900));
        root->Add(label, 0, wxEXPAND | wxALL, gap);
        return label;
    };
    add_text(_L("Load calibration — CNC Testhook"));
    add_text(_L("Sweep a setting and optional related values using the current printer, process, and first filament. Every repeat gets an "
                "XY hook and a Z hook, each "
                "on its own plate, so temperatures and cooling tests stay independent. Z hooks stand upright; configure support and brim "
                "in Prepare before generating. Each plate is saved as a project and automatically sliced to G-code."));
    auto* form = new wxFlexGridSizer(2, gap, gap);
    form->AddGrowableCol(1, 1);
    root->Add(form, 0, wxEXPAND | wxALL, gap);
    std::vector<wxWindow*>* control_group = nullptr;
    auto field                            = [&](const wxString& label, wxWindow* control) {
        auto* caption = new wxStaticText(this, wxID_ANY, label);
        form->Add(caption, 0, wxALIGN_CENTER_VERTICAL);
        form->Add(control, 1, wxEXPAND);
        if (control_group) {
            control_group->push_back(caption);
            control_group->push_back(control);
        }
    };
    auto text = [&](const wxString& label, const wxString& value) {
        auto* c = new wxTextCtrl(this, wxID_ANY, value);
        field(label, c);
        return c;
    };
    m->study_choice = new wxChoice(this, wxID_ANY);
    field(_L("Saved study"), m->study_choice);
    m->material = text(_L("Material name"), "");
    m->type     = text(_L("Material type (PLA, PETG, …)"), "");
    m->color    = new wxColourPickerCtrl(this, wxID_ANY, *wxWHITE);
    field(_L("Color"), m->color);
    m->color->Bind(wxEVT_COLOURPICKER_CHANGED, [this](wxColourPickerEvent&) { m->legacy_color.clear(); });
    m->setting = new wxChoice(this, wxID_ANY);
    for (const auto& s : LC::settings()) {
        const auto* d = print_config_def.get(s.key);
        m->setting->Append(d ? _(d->label) : _L("Bed temperature"));
    }
    m->setting->SetSelection(1);
    field(_L("Setting to vary"), m->setting);
    control_group = &m->categorical_controls;
    m->categories = new wxCheckListBox(this, wxID_ANY, wxDefaultPosition, FromDIP(wxSize(500, 150)));
    field(_L("Values to test"), m->categories);
    control_group      = &m->numeric_controls;
    m->values          = text(_L("Setting values (comma-separated)"), "15,30,50");
    m->start           = text(_L("Range start"), "15");
    m->end             = text(_L("Range end"), "50");
    m->step            = text(_L("Range increment"), "5");
    auto* range_button = new wxButton(this, wxID_ANY, _L("Use range"));
    field(wxEmptyString, range_button);
    control_group    = &m->related_controls;
    m->related_label = new wxStaticText(this, wxID_ANY, _L("Related parameter"));
    field(wxEmptyString, m->related_label);
    m->related_mode = new wxChoice(this, wxID_ANY);
    m->related_mode->Append(_L("Use current Prepare setting"));
    m->related_mode->Append(_L("Fixed value for all tests"));
    m->related_mode->Append(_L("Sweep every combination"));
    m->related_mode->SetSelection(0);
    field(_L("Related values"), m->related_mode);
    m->related_values = text(_L("Value(s), comma-separated"), "20,30,50");
    m->related_mode->Bind(wxEVT_CHOICE, [this](wxCommandEvent&) { m->related_help(); });
    control_group = nullptr;
    m->repeats = new wxSpinCtrl(this, wxID_ANY);
    m->repeats->SetRange(1, 20);
    m->repeats->SetValue(3);
    field(_L("Hooks per value per orientation"), m->repeats);
    m->help       = add_text("");
    auto* buttons = new wxBoxSizer(wxHORIZONTAL);
    root->Add(buttons, 0, wxEXPAND | wxALL, gap);
    m->generate = new wxButton(this, wxID_ANY, _L("Generate and slice study"));
    buttons->Add(m->generate, 0, wxRIGHT, gap);
    auto* cancel = new wxButton(this, wxID_ANY, _L("Cancel slicing"));
    buttons->Add(cancel, 0, wxRIGHT, gap);
    auto* resume = new wxButton(this, wxID_ANY, _L("Resume slicing"));
    buttons->Add(resume, 0, wxRIGHT, gap);
    auto* folder = new wxButton(this, wxID_ANY, _L("Open study folder"));
    buttons->Add(folder, 0, wxRIGHT, gap);
    m->status = add_text(_L("No study generated."));
    add_text(_L("Record each hook’s breaking load, or the maximum tested load if it did not break. Leave untested hooks as Not tested and "
                "record failed prints, lost hooks, or other reasons in Notes. Optional mass excludes supports and brim."));
    m->unit = new wxChoice(this, wxID_ANY);
    m->unit->Append("N");
    m->unit->Append("kgf");
    m->unit->Append("lbf");
    m->unit->SetSelection(0);
    root->Add(m->unit, 0, wxALL, gap);
    m->grid = new wxGrid(this, wxID_ANY, wxDefaultPosition, FromDIP(wxSize(950, 300)));
    m->grid->CreateGrid(0, 7);
    const wxString columns[] = {_L("Hook / plate"),         _L("Value"),    _L("Orientation"), _L("Outcome"),
                                _L("Load (selected unit)"), _L("Mass (g)"), _L("Notes")};
    for (int c = 0; c < 7; ++c) {
        m->grid->SetColLabelValue(c, columns[c]);
        m->grid->SetColSize(c, FromDIP(c == 6 ? 230 : c == 3 ? 150 : 120));
    }
    root->Add(m->grid, 0, wxEXPAND | wxALL, gap);
    auto* result_buttons = new wxBoxSizer(wxHORIZONTAL);
    root->Add(result_buttons, 0, wxALL, gap);
    m->save = new wxButton(this, wxID_ANY, _L("Save results and calculate"));
    result_buttons->Add(m->save, 0, wxRIGHT, gap);
    auto* open_hook = new wxButton(this, wxID_ANY, _L("Open selected hooks in Prepare"));
    result_buttons->Add(open_hook, 0, wxRIGHT, gap);
    m->summary = new wxTextCtrl(this, wxID_ANY, "", wxDefaultPosition, FromDIP(wxSize(900, 170)), wxTE_MULTILINE | wxTE_READONLY);
    root->Add(m->summary, 0, wxEXPAND | wxALL, gap);
    add_text(_L("Save observations to a new or existing material. Regressed mode combines old and new data; simple mode uses minima at "
                "tested settings. Supply validated XY/Z peak tensile stress per force (MPa/N). Survived and untested hooks remain recorded "
                "but are excluded from failure fits. See Materials for curves, modes, and inherited properties."));
    auto* calibration = new wxFlexGridSizer(2, gap, gap);
    calibration->AddGrowableCol(1, 1);
    root->Add(calibration, 0, wxEXPAND | wxALL, gap);
    form    = calibration;
    m->base = new wxChoice(this, wxID_ANY);
    field(_L("Base mechanical material"), m->base);
    m->target = new wxChoice(this, wxID_ANY);
    field(_L("Save to"), m->target);
    m->calculation = new wxChoice(this, wxID_ANY);
    m->calculation->Append(_L("Regressed (default)"));
    m->calculation->Append(_L("Simple (minimum at tested value)"));
    m->calculation->SetSelection(0);
    field(_L("Calculation"), m->calculation);
    m->group = new wxChoice(this, wxID_ANY);
    field(_L("Reference value (all study data is saved)"), m->group);
    m->xy               = text(_L("XY peak tensile stress / force (MPa/N)"), "");
    m->z                = text(_L("Z peak tensile stress / force (MPa/N)"), "");
    m->calibrate_button = new wxButton(this, wxID_ANY, _L("Save calibrated material"));
    root->Add(m->calibrate_button, 0, wxALL, gap);
    open_hook->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) {
        m->guarded([&] {
            m->persist();
            std::vector<LC::Sample> selected;
            for (int row = 0; row < m->grid->GetNumberRows(); ++row) {
                bool included = false;
                for (int column = 0; column < m->grid->GetNumberCols(); ++column)
                    included = included || m->grid->IsInSelection(row, column);
                if (included)
                    selected.push_back(m->samples.at(size_t(row)));
            }
            if (selected.empty()) {
                const int row = m->grid->GetGridCursorRow();
                if (row >= 0 && size_t(row) < m->samples.size())
                    selected.push_back(m->samples[size_t(row)]);
            }
            if (selected.empty())
                throw std::invalid_argument("Select one or more hooks in the results table.");
            auto project    = LC::arrange_hooks((m->current_path / "CNC_Testhook.stl").string(), selected, m->baseline_config(),
                                                m->study.at("setting").get<std::string>());
            const auto path = m->current_path / boost::filesystem::unique_path("selected-hooks-%%%%-%%%%.3mf");
            LC::store_project(path.string(), project);
            auto* plater = wxGetApp().plater();
            if (plater->new_project(false, false, _L("CNC Testhook")) == wxID_CANCEL)
                return;
            plater->load_files(std::vector<boost::filesystem::path>{path}, LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
            wxGetApp().mainframe->select_tab(TAB_ID_PREPARE);
        });
    });
    m->generate->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { m->guarded([&] { m->generate_study(); }); });
    resume->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { m->guarded([&] { m->resume(); }); });
    cancel->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { m->stop(); });
    folder->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) {
        if (!m->current_path.empty())
            wxLaunchDefaultApplication(wxString::FromUTF8(m->current_path.string()));
    });
    range_button->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) {
        m->guarded([&] {
            const auto list = LC::range(number(m->start), number(m->end), number(m->step));
            wxString text;
            for (const auto& v : list) {
                if (!text.empty())
                    text += ",";
                text += wxString::FromUTF8(v);
            }
            m->values->ChangeValue(text);
        });
    });
    m->setting->Bind(wxEVT_CHOICE, [this](wxCommandEvent&) { m->setting_help(); });
    m->study_choice->Bind(wxEVT_CHOICE, [this](wxCommandEvent&) { m->guarded([&] { m->open(); }); });
    m->grid->Bind(wxEVT_GRID_CELL_CHANGED, [this](wxGridEvent& event) {
        if (!m->current_path.empty()) {
            try {
                m->persist();
            } catch (const std::exception& e) {
                m->status->SetLabel(wxString::FromUTF8(e.what()));
            }
        }
        event.Skip();
    });
    m->unit->Bind(wxEVT_CHOICE, [this](wxCommandEvent&) {
        const double factors[] = {1., 9.80665, 4.4482216152605};
        const int next         = m->unit->GetSelection();
        m->grid->SaveEditControlValue();
        std::vector<wxString> converted;
        for (int row = 0; row < m->grid->GetNumberRows(); ++row) {
            auto text = m->grid->GetCellValue(row, 4);
            double value;
            if (!text.empty() && (!text.ToDouble(&value) || !std::isfinite(value))) {
                m->unit->SetSelection(m->displayed_unit);
                return;
            }
            converted.push_back(text.empty() ? wxString() : wxString::Format("%.9g", value * factors[m->displayed_unit] / factors[next]));
        }
        for (size_t i = 0; i < converted.size(); ++i)
            m->grid->SetCellValue(int(i), 4, converted[i]);
        m->displayed_unit = next;
    });
    m->save->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { m->guarded([&] { m->persist(); }); });
    m->calibrate_button->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { m->guarded([&] { m->save_material(); }); });
    Bind(wxEVT_TIMER, [this](wxTimerEvent&) {
        {
            std::lock_guard<std::mutex> lock(m->mutex);
            m->status->SetLabel(wxString::FromUTF8(m->progress));
        }
        m->status->Wrap(FromDIP(900));
        if (m->done) {
            m->timer.Stop();
            if (m->worker.joinable())
                m->worker.join();
            m->busy = false;
            m->generate->Enable();
            m->study_choice->Enable();
            FitInside();
        }
    });
    m->refresh_targets();
    m->guarded([&] { m->refresh_studies(); });
    m->setting_help();
    update_colors();
    FitInside();
}
void LoadCalibrationPanel::rescale()
{
    SetScrollRate(FromDIP(10), FromDIP(10));
    for (int c = 0; c < 7; ++c)
        m->grid->SetColSize(c, FromDIP(c == 6 ? 230 : c == 3 ? 150 : 120));
    Layout();
    FitInside();
}
void LoadCalibrationPanel::update_colors()
{
    wxGetApp().UpdateDarkUIWin(this);
    m->grid->SetDefaultCellBackgroundColour(GetBackgroundColour());
    m->grid->SetDefaultCellTextColour(GetForegroundColour());
    m->grid->SetLabelBackgroundColour(GetBackgroundColour());
    m->grid->SetLabelTextColour(GetForegroundColour());
    m->grid->ForceRefresh();
}
LoadCalibrationPanel::~LoadCalibrationPanel() = default;

namespace {
class MaterialCurve : public wxPanel
{
public:
    std::vector<wxRealPoint> line, points;
    wxString caption;
    bool show_points{true};
    explicit MaterialCurve(wxWindow* parent) : wxPanel(parent, wxID_ANY, wxDefaultPosition, parent->FromDIP(wxSize(800, 280)))
    {
        SetBackgroundStyle(wxBG_STYLE_PAINT);
        Bind(wxEVT_PAINT, [this](wxPaintEvent&) {
            wxAutoBufferedPaintDC dc(this);
            dc.SetBackground(wxBrush(GetBackgroundColour()));
            dc.Clear();
            dc.SetTextForeground(GetForegroundColour());
            dc.DrawText(caption, FromDIP(10), FromDIP(8));
            if (line.empty() && points.empty())
                return;
            double xmin = std::numeric_limits<double>::infinity(), xmax = -xmin, ymin = xmin, ymax = -xmin;
            auto include = [&](const auto& values) {
                for (const auto& p : values) {
                    xmin = std::min(xmin, p.x);
                    xmax = std::max(xmax, p.x);
                    ymin = std::min(ymin, p.y);
                    ymax = std::max(ymax, p.y);
                }
            };
            include(line);
            include(points);
            if (xmax <= xmin)
                xmax = xmin + 1;
            if (ymax <= ymin)
                ymax = ymin + 1;
            const double margin = (ymax - ymin) * .1;
            ymin                = std::max(0., ymin - margin);
            ymax += margin;
            const int left = FromDIP(100), top = FromDIP(40), width = std::max(1, GetClientSize().x - left - FromDIP(30)),
                      height = std::max(1, GetClientSize().y - top - FromDIP(40));
            auto map         = [&](const wxRealPoint& p) {
                return wxPoint(left + int((p.x - xmin) / (xmax - xmin) * width), top + height - int((p.y - ymin) / (ymax - ymin) * height));
            };
            dc.SetPen(wxPen(GetForegroundColour()));
            dc.DrawLine(left, top, left, top + height);
            dc.DrawLine(left, top + height, left + width, top + height);
            dc.DrawText(wxString::Format("%.4g", ymax), FromDIP(8), top);
            dc.DrawText(wxString::Format("%.4g", ymin), FromDIP(8), top + height - FromDIP(14));
            dc.DrawText(wxString::Format("%.4g", xmin), left, top + height + FromDIP(5));
            dc.DrawText(wxString::Format("%.4g", xmax), left + width - FromDIP(45), top + height + FromDIP(5));
            dc.SetPen(wxPen(wxSystemSettings::GetColour(wxSYS_COLOUR_HOTLIGHT), FromDIP(2)));
            for (size_t i = 1; i < line.size(); ++i)
                dc.DrawLine(map(line[i - 1]), map(line[i]));
            if (show_points) {
                dc.SetPen(wxPen(GetForegroundColour()));
                dc.SetBrush(wxBrush(GetForegroundColour()));
                for (const auto& p : points)
                    dc.DrawCircle(map(p), FromDIP(3));
            }
        });
    }
};
} // namespace
struct CalibratedMaterialsPanel::Impl
{
    CalibratedMaterialsPanel* panel;
    wxChoice *materials{}, *parameter{}, *property{}, *mode{}, *curve_category{};
    wxStaticText* curve_category_label{};
    std::string curve_category_key;
    std::vector<std::string> curve_category_values;
    wxTextCtrl *name{}, *type{}, *query{}, *low{}, *high{}, *details{};
    wxColourPickerCtrl* color{};
    wxGrid *sheet{}, *data{};
    MaterialCurve* graph{};
    Json library, record;
    std::string key;
    SA::Material material;
    ME::Context context;
    std::vector<size_t> data_indices;
    explicit Impl(CalibratedMaterialsPanel* p) : panel(p) {}
    template<class F> void guarded(F action)
    {
        try {
            action();
        } catch (const std::exception& e) {
            wxMessageBox(wxString::FromUTF8(e.what()), _L("Calibrated materials"), wxOK | wxICON_ERROR, panel);
        }
    }
    void refresh()
    {
        library = boost::filesystem::exists(root_path() / "materials.json") ? read_json(root_path() / "materials.json") :
                                                                              Json{{"version", 1}, {"materials", Json::array()}};
        if (library.at("version") != 1)
            throw std::runtime_error("Unsupported material library version.");
        materials->Clear();
        int selected = 0;
        for (size_t i = 0; i < library["materials"].size(); ++i) {
            SA::Setup setup;
            std::string error;
            if (!SA::deserialize_setup(library["materials"][i].at("setup").dump(), setup, &error))
                throw std::runtime_error(error);
            materials->Append(wxString::FromUTF8(setup.material.name));
            if (setup.material.key == key)
                selected = int(i);
        }
        if (materials->GetCount()) {
            materials->SetSelection(selected);
            select();
        } else {
            key.clear();
            sheet->ClearGrid();
            details->ChangeValue(_L("No calibrated materials yet. Save experimental data from Calibration → Load."));
            if (data->GetNumberRows())
                data->DeleteRows(0, data->GetNumberRows());
            graph->line.clear();
            graph->points.clear();
            graph->Refresh();
        }
    }
    void select()
    {
        const int index = materials->GetSelection();
        if (index < 0)
            return;
        const auto& row = library["materials"][size_t(index)];
        SA::Setup setup;
        if (!SA::deserialize_setup(row.at("setup").dump(), setup))
            throw std::runtime_error("Invalid material.");
        material = setup.material;
        key      = material.key;
        record   = material.experimental_data.empty() ? ME::create(material) : Json::parse(material.experimental_data);
        ME::validate(record);
        context = record.at("reference").get<ME::Context>();
        for (const auto& [k, v] : material.experimental_context)
            context[k] = v;
        name->ChangeValue(wxString::FromUTF8(material.name));
        type->ChangeValue(wxString::FromUTF8(row.value("type", "")));
        wxColour chosen(wxString::FromUTF8(row.value("color", "#FFFFFF")));
        color->SetColour(chosen.IsOk() ? chosen : *wxWHITE);
        const auto previous_parameter = parameter->GetStringSelection();
        parameter->Clear();
        for (auto it = record["modes"].begin(); it != record["modes"].end(); ++it)
            parameter->Append(wxString::FromUTF8(it.key()));
        if (parameter->FindString("direct") == wxNOT_FOUND)
            parameter->Append("direct");
        for (const auto& setting : LC::settings())
            if (parameter->FindString(wxString::FromUTF8(setting.key)) == wxNOT_FOUND)
                parameter->Append(wxString::FromUTF8(setting.key));
        parameter->SetSelection(0);
        if (parameter->FindString(previous_parameter) != wxNOT_FOUND)
            parameter->SetStringSelection(previous_parameter);
        if (property->GetSelection() < 0)
            property->SetSelection(0);
        update_controls();
        render();
    }
    std::string parameter_key() const { return parameter->GetStringSelection().ToUTF8().data(); }
    std::string property_key() const { return ME::properties().at(property->GetSelection()).key; }
    void update_controls()
    {
        if (key.empty())
            return;
        const auto p = parameter_key();
        mode->SetSelection(record["modes"].value(p, "regressed") == "simple" ? 1 : 0);
        query->ChangeValue(wxString::FromUTF8(context.count(p) ? context[p] : ""));
        curve_category_key.clear();
        curve_category_values.clear();
        curve_category->Clear();
        for (auto it = record["modes"].begin(); it != record["modes"].end(); ++it) {
            const auto* def = print_config_def.get(it.key());
            if (it.key() != p && def && (def->type == coEnum || def->type == coBool)) {
                curve_category_key = it.key();
                curve_category_label->SetLabel(_L("Curve for: ") + _(def->label));
                std::set<std::string> seen;
                for (const auto& o : record["observations"]) {
                    const auto c = o.at("context").get<ME::Context>();
                    if (c.count(it.key()) && seen.insert(c.at(it.key())).second) {
                        curve_category_values.push_back(c.at(it.key()));
                        curve_category->Append(def->type == coBool ? (c.at(it.key()) == "1" ? _L("Yes") : _L("No")) :
                                                                     wxString::FromUTF8(c.at(it.key())));
                    }
                }
                break;
            }
        }
        curve_category->Show(!curve_category_values.empty());
        curve_category_label->Show(!curve_category_values.empty());
        if (!curve_category_values.empty()) {
            auto selected = std::find(curve_category_values.begin(), curve_category_values.end(), context[curve_category_key]);
            size_t index  = selected == curve_category_values.end() ? 0 : size_t(selected - curve_category_values.begin());
            curve_category->SetSelection(int(index));
            context[curve_category_key] = curve_category_values[index];
        }
        const auto prop = property_key();
        low->Clear();
        high->Clear();
        if (record["ranges"].contains(prop)) {
            low->ChangeValue(wxString::Format("%.9g", record["ranges"][prop][0].get<double>()));
            high->ChangeValue(wxString::Format("%.9g", record["ranges"][prop][1].get<double>()));
        }
    }
    void render()
    {
        if (key.empty())
            return;
        std::map<std::string, ME::Prediction> predictions;
        material.experimental_data = record.dump();
        const auto evaluated       = ME::evaluate(material, context, &predictions);
        for (size_t i = 0; i < ME::properties().size(); ++i) {
            const auto& prop   = ME::properties()[i];
            const auto& fit    = predictions.at(prop.key);
            const double value = evaluated.*(prop.member), base = record["base"][prop.key];
            sheet->SetCellValue(int(i), 0, _(prop.label));
            sheet->SetCellValue(int(i), 1, wxString::Format("%.6g", value));
            sheet->SetCellValue(int(i), 2, fit.experimental ? _L("Calculated") : _L("Inherited"));
            sheet->SetCellValue(int(i), 3, wxString::Format("%u", unsigned(fit.points)));
            sheet->SetCellValue(int(i), 4, wxString::Format("%.6g", base));
            sheet->SetCellValue(int(i), 5, wxString::Format("%.1f%%", 100 * value / base));
            wxString range = _L("Not supplied");
            if (record["ranges"].contains(prop.key)) {
                double lo = record["ranges"][prop.key][0], hi = record["ranges"][prop.key][1];
                range = wxString::Format("%.4g–%.4g (%.1f%%)", lo, hi, 100 * (value - lo) / (hi - lo));
            }
            sheet->SetCellValue(int(i), 6, range);
            if (!fit.experimental && std::string(prop.key).find("yield_strength") == 0 && value < base)
                sheet->SetCellValue(int(i), 2, _L("Capped estimate"));
        }
        if (data->GetNumberRows())
            data->DeleteRows(0, data->GetNumberRows());
        data_indices.clear();
        const auto prop  = property_key();
        const auto param = parameter_key();
        for (size_t i = 0; i < record["observations"].size(); ++i) {
            const auto& o = record["observations"][i];
            if (o["property"] != prop)
                continue;
            data_indices.push_back(i);
            const int row = data->GetNumberRows();
            data->AppendRows(1);
            data->SetCellValue(row, 0, wxString::FromUTF8(o.at("id").get<std::string>()));
            data->SetCellValue(row, 1, wxString::FromUTF8(o.at("parameter").get<std::string>()));
            std::string settings = o.at("value").get<std::string>();
            if (o.contains("parameters"))
                for (const auto& key : o.at("parameters").get<std::vector<std::string>>())
                    if (key != o.at("parameter").get<std::string>() && o.at("context").contains(key))
                        settings += "; " + key + "=" + o.at("context").at(key).get<std::string>();
            data->SetCellValue(row, 2, wxString::FromUTF8(settings));
            data->SetCellValue(row, 3, o["measured"].is_null() ? wxString() : wxString::Format("%.6g", o["measured"].get<double>()));
            data->SetCellValue(row, 4,
                               o.value("excluded", false) ? _L("Excluded") :
                               o["outcome"] == 1          ? _L("Failure / measured") :
                               o["outcome"] == 2          ? _L("Survived (lower bound)") :
                                                            _L("Not tested"));
            data->SetCellValue(row, 5, wxString::FromUTF8(o.value("source", "")));
            data->SetCellValue(row, 6, wxString::FromUTF8(o.value("notes", "")));
        }
        const auto fit = predictions.at(prop);
        details->ChangeValue(
            wxString::Format(_L("%s\n%u usable measurements across %u configurations. RMSE: %.4g (property units). %s\nBase: %s. Ranges "
                                "are supplied by the user; no manufacturer range is inferred from a generic estimate.\nFits exclude "
                                "survived, untested, and excluded points. Inherited properties are not measurements."),
                             wxString::FromUTF8(fit.model), unsigned(fit.points), unsigned(fit.configurations), fit.rmse,
                             fit.extrapolated ? _L("Extrapolation outside measured settings.") :
                                                _L("Within measured settings or inherited."),
                             wxString::FromUTF8(record.value("base_name", ""))));
        graph->line.clear();
        graph->points.clear();
        double xmin = std::numeric_limits<double>::infinity(), xmax = -xmin;
        for (const auto& o : record["observations"]) {
            if (o["property"] != prop || o.value("excluded", false) || o["outcome"] != 1 || o["measured"].is_null())
                continue;
            const auto inputs = o.at("context").get<ME::Context>();
            if (!curve_category_key.empty() &&
                (!inputs.count(curve_category_key) || inputs.at(curve_category_key) != context[curve_category_key]))
                continue;
            if (!inputs.count(param))
                continue;
            double x;
            if (!wxString::FromUTF8(inputs.at(param)).ToDouble(&x))
                continue;
            graph->points.emplace_back(x, o["measured"].get<double>());
            xmin = std::min(xmin, x);
            xmax = std::max(xmax, x);
        }
        double q;
        if (query->GetValue().ToDouble(&q) && std::isfinite(q)) {
            xmin = std::min(xmin, q);
            xmax = std::max(xmax, q);
        }
        if (std::isfinite(xmin) && xmax > xmin) {
            for (int i = 0; i <= 40; ++i) {
                double x      = xmin + (xmax - xmin) * i / 40;
                auto inputs   = context;
                inputs[param] = wxString::Format("%.12g", x).ToUTF8().data();
                const auto p  = ME::predict(record, prop, inputs);
                if (p.experimental)
                    graph->line.emplace_back(x, p.value);
            }
        }
        if (record["modes"].value(param, "regressed") == "simple")
            graph->line.clear();
        graph->caption = wxString::FromUTF8(
            prop + " by " + param + (curve_category_key.empty() ? "" : " — " + curve_category_key + "=" + context[curve_category_key]));
        graph->Refresh();
        panel->FitInside();
    }
    void commit()
    {
        if (key.empty())
            throw std::invalid_argument("Select a material.");
        ME::validate(record);
        material.experimental_data    = record.dump();
        material.experimental_context = context;
        material                      = ME::evaluate(material, context);
        auto current                  = read_json(root_path() / "materials.json");
        bool found                    = false;
        for (auto& row : current["materials"]) {
            SA::Setup setup;
            if (!SA::deserialize_setup(row.at("setup").dump(), setup) || setup.material.key != key)
                continue;
            material.name = utf8(name);
            if (material.name.find(" -- CALIBRATED") == std::string::npos)
                material.name += " -- CALIBRATED";
            setup.material = material;
            row["setup"]   = Json::parse(SA::serialize_setup(setup));
            row["type"]    = utf8(type);
            row["color"]   = color->GetColour().GetAsString(wxC2S_HTML_SYNTAX).ToUTF8().data();
            found          = true;
            break;
        }
        if (!found)
            throw std::runtime_error("Material was removed. Refresh the library.");
        write_json(root_path() / "materials.previous.json", read_json(root_path() / "materials.json"));
        write_json(root_path() / "materials.json", current);
        refresh_catalog();
        refresh();
    }
    void settings()
    {
        if (key.empty())
            throw std::invalid_argument("Select a material.");
        record["modes"][parameter_key()] = mode->GetSelection() == 1 ? "simple" : "regressed";
        if (!query->GetValue().empty())
            context[parameter_key()] = utf8(query);
        const auto prop = property_key();
        if (low->GetValue().empty() && high->GetValue().empty())
            record["ranges"].erase(prop);
        else
            record["ranges"][prop] = {number(low), number(high)};
        commit();
    }
    void exclude()
    {
        if (key.empty())
            return;
        std::vector<int> selected;
        for (int i = 0; i < data->GetNumberRows(); ++i) {
            bool chosen = false;
            for (int c = 0; c < data->GetNumberCols(); ++c)
                chosen |= data->IsInSelection(i, c);
            if (chosen)
                selected.push_back(i);
        }
        if (selected.empty() && data->GetGridCursorRow() >= 0)
            selected.push_back(data->GetGridCursorRow());
        for (int row : selected) {
            auto& o       = record["observations"][data_indices.at(size_t(row))];
            o["excluded"] = !o.value("excluded", false);
        }
        commit();
    }
    void add_measurement()
    {
        if (key.empty())
            throw std::invalid_argument("Select a material first.");
        wxTextEntryDialog dialog(panel,
                                 _L("Enter the measured property value in the units shown in the data sheet. This records a direct "
                                    "measurement at the current settings."),
                                 _L("Add measurement"));
        if (dialog.ShowModal() != wxID_OK)
            return;
        double value;
        if (!dialog.GetValue().ToDouble(&value) || !std::isfinite(value) || value <= 0)
            throw std::invalid_argument("Enter a positive finite measurement.");
        auto inputs  = context;
        const auto p = parameter_key();
        if (!query->GetValue().empty())
            inputs[p] = utf8(query);
        record["observations"].push_back({{"id", "manual/" + boost::filesystem::unique_path("%%%%-%%%%-%%%%").string()},
                                          {"property", property_key()},
                                          {"parameter", p},
                                          {"value", inputs.count(p) ? inputs[p] : "direct"},
                                          {"context", inputs},
                                          {"measured", value},
                                          {"outcome", 1},
                                          {"excluded", false},
                                          {"source", "Manual measured property"},
                                          {"notes", ""}});
        record["modes"][p] = mode->GetSelection() == 1 ? "simple" : "regressed";
        commit();
    }
    void remove()
    {
        if (key.empty())
            return;
        auto current = read_json(root_path() / "materials.json");
        auto& rows   = current["materials"];
        rows.erase(std::remove_if(rows.begin(), rows.end(),
                                  [&](const Json& row) {
                                      SA::Setup setup;
                                      return SA::deserialize_setup(row.at("setup").dump(), setup) && setup.material.key == key;
                                  }),
                   rows.end());
        write_json(root_path() / "materials.previous.json", read_json(root_path() / "materials.json"));
        write_json(root_path() / "materials.json", current);
        refresh_catalog();
        key.clear();
        refresh();
    }
    void export_material()
    {
        if (key.empty())
            return;
        wxFileDialog file(panel, _L("Export calibrated material"), "", "calibrated-material.json", _L("JSON files (*.json)|*.json"),
                          wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
        if (file.ShowModal() != wxID_OK)
            return;
        write_json(file.GetPath().ToUTF8().data(),
                   Json{{"version", 1}, {"materials", Json::array({library["materials"][materials->GetSelection()]})}});
    }
    void import_material()
    {
        wxFileDialog file(panel, _L("Import calibrated materials"), "", "", _L("JSON files (*.json)|*.json"),
                          wxFD_OPEN | wxFD_FILE_MUST_EXIST);
        if (file.ShowModal() != wxID_OK)
            return;
        const boost::filesystem::path path(file.GetPath().ToUTF8().data());
        if (boost::filesystem::file_size(path) > 20 * 1024 * 1024)
            throw std::invalid_argument("Material import is limited to 20 MB.");
        auto incoming = read_json(path);
        if (incoming.at("version") != 1 || !incoming.at("materials").is_array() || incoming["materials"].size() > 100)
            throw std::invalid_argument("Invalid material library.");
        auto current = boost::filesystem::exists(root_path() / "materials.json") ? read_json(root_path() / "materials.json") :
                                                                                   Json{{"version", 1}, {"materials", Json::array()}};
        for (auto row : incoming["materials"]) {
            SA::Setup setup;
            std::string error;
            if (!SA::deserialize_setup(row.at("setup").dump(), setup, &error) || !setup.material.validate().empty())
                throw std::invalid_argument("Invalid imported material: " + error);
            setup.material.key = "calibrated_" + boost::filesystem::unique_path("%%%%-%%%%-%%%%").string();
            row["setup"]       = Json::parse(SA::serialize_setup(setup));
            current["materials"].push_back(row);
            key = setup.material.key;
        }
        if (boost::filesystem::exists(root_path() / "materials.json"))
            write_json(root_path() / "materials.previous.json", read_json(root_path() / "materials.json"));
        write_json(root_path() / "materials.json", current);
        refresh_catalog();
        refresh();
    }
};
CalibratedMaterialsPanel::CalibratedMaterialsPanel(wxWindow* parent) : wxScrolledWindow(parent, wxID_ANY), m(std::make_unique<Impl>(this))
{
    SetScrollRate(FromDIP(10), FromDIP(10));
    const int gap = FromDIP(8);
    auto* root    = new wxBoxSizer(wxVERTICAL);
    SetSizer(root);
    auto text = [&](const wxString& value) {
        auto* label = new wxStaticText(this, wxID_ANY, value);
        label->Wrap(FromDIP(1000));
        root->Add(label, 0, wxEXPAND | wxALL, gap);
    };
    text(_L("Calibrated materials — data, models, and inherited properties"));
    auto* toolbar = new wxBoxSizer(wxHORIZONTAL);
    root->Add(toolbar, 0, wxEXPAND | wxALL, gap);
    m->materials = new wxChoice(this, wxID_ANY);
    toolbar->Add(m->materials, 1, wxRIGHT, gap);
    auto button = [&](const wxString& label, auto action) {
        auto* b = new wxButton(this, wxID_ANY, label);
        toolbar->Add(b, 0, wxRIGHT, gap);
        b->Bind(wxEVT_BUTTON, [this, action](wxCommandEvent&) { m->guarded(action); });
    };
    button(_L("Refresh"), [this] { m->refresh(); });
    button(_L("Import"), [this] { m->import_material(); });
    button(_L("Export"), [this] { m->export_material(); });
    button(_L("Delete material"), [this] { m->remove(); });
    auto* form = new wxFlexGridSizer(2, gap, gap);
    form->AddGrowableCol(1, 1);
    root->Add(form, 0, wxEXPAND | wxALL, gap);
    auto field = [&](const wxString& label, wxWindow* control) {
        form->Add(new wxStaticText(this, wxID_ANY, label), 0, wxALIGN_CENTER_VERTICAL);
        form->Add(control, 1, wxEXPAND);
    };
    m->name = new wxTextCtrl(this, wxID_ANY);
    field(_L("Name"), m->name);
    m->type = new wxTextCtrl(this, wxID_ANY);
    field(_L("Material type"), m->type);
    m->color = new wxColourPickerCtrl(this, wxID_ANY, *wxWHITE);
    field(_L("Color"), m->color);
    m->sheet = new wxGrid(this, wxID_ANY, wxDefaultPosition, FromDIP(wxSize(1000, 280)));
    m->sheet->CreateGrid(int(ME::properties().size()), 7);
    m->sheet->EnableEditing(false);
    const wxString cols[] =
        {_L("Property / units"),         _L("Value"), _L("Source"), _L("Data points"), _L("Base estimate"), _L("% of base"),
         _L("Expected range / position")};
    for (int i = 0; i < 7; ++i) {
        m->sheet->SetColLabelValue(i, cols[i]);
        m->sheet->SetColSize(i, FromDIP(i == 0 || i == 6 ? 230 : 130));
    }
    root->Add(m->sheet, 0, wxEXPAND | wxALL, gap);
    form = new wxFlexGridSizer(2, gap, gap);
    form->AddGrowableCol(1, 1);
    root->Add(form, 0, wxEXPAND | wxALL, gap);
    m->parameter = new wxChoice(this, wxID_ANY);
    field(_L("Parameter"), m->parameter);
    m->curve_category_label = new wxStaticText(this, wxID_ANY, _L("Curve for"));
    m->curve_category       = new wxChoice(this, wxID_ANY);
    form->Add(m->curve_category_label, 0, wxALIGN_CENTER_VERTICAL);
    form->Add(m->curve_category, 1, wxEXPAND);
    m->curve_category->Bind(wxEVT_CHOICE, [this](wxCommandEvent&) {
        m->guarded([&] {
            const int selected = m->curve_category->GetSelection();
            if (selected >= 0)
                m->context[m->curve_category_key] = m->curve_category_values.at(size_t(selected));
            m->render();
        });
    });
    m->property = new wxChoice(this, wxID_ANY);
    for (const auto& p : ME::properties())
        m->property->Append(_(p.label));
    m->property->SetSelection(8);
    field(_L("Property"), m->property);
    m->mode = new wxChoice(this, wxID_ANY);
    m->mode->Append(_L("Regressed (default)"));
    m->mode->Append(_L("Simple"));
    m->mode->SetSelection(0);
    field(_L("Per-parameter calculation"), m->mode);
    m->query = new wxTextCtrl(this, wxID_ANY);
    field(_L("Evaluate / graph through value"), m->query);
    m->low = new wxTextCtrl(this, wxID_ANY);
    field(_L("Expected base range minimum (property units)"), m->low);
    m->high = new wxTextCtrl(this, wxID_ANY);
    field(_L("Expected base range maximum (property units)"), m->high);
    toolbar = new wxBoxSizer(wxHORIZONTAL);
    root->Add(toolbar, 0, wxALL, gap);
    button(_L("Save settings and recalculate"), [this] { m->settings(); });
    button(_L("Add measured property"), [this] { m->add_measurement(); });
    auto* show = new wxCheckBox(this, wxID_ANY, _L("Show experimental data points"));
    show->SetValue(true);
    root->Add(show, 0, wxALL, gap);
    m->graph = new MaterialCurve(this);
    root->Add(m->graph, 0, wxEXPAND | wxALL, gap);
    m->details = new wxTextCtrl(this, wxID_ANY, "", wxDefaultPosition, FromDIP(wxSize(1000, 100)), wxTE_MULTILINE | wxTE_READONLY);
    root->Add(m->details, 0, wxEXPAND | wxALL, gap);
    text(_L("Data for the selected property. Exclude faulty measurements to remove them from fits; toggle again to restore them. Library "
            "edits keep a materials.previous.json recovery copy."));
    m->data = new wxGrid(this, wxID_ANY, wxDefaultPosition, FromDIP(wxSize(1000, 260)));
    m->data->CreateGrid(0, 7);
    m->data->EnableEditing(false);
    const wxString data_cols[] = {_L("Data point ID"), _L("Parameter"),    _L("Setting"), _L("Measurement"),
                                  _L("Status"),        _L("Source study"), _L("Notes")};
    for (int i = 0; i < 7; ++i) {
        m->data->SetColLabelValue(i, data_cols[i]);
        m->data->SetColSize(i, FromDIP(i == 0 ? 280 : i == 4 ? 200 : 150));
    }
    root->Add(m->data, 0, wxEXPAND | wxALL, gap);
    toolbar = new wxBoxSizer(wxHORIZONTAL);
    root->Add(toolbar, 0, wxALL, gap);
    button(_L("Exclude / restore selected data"), [this] { m->exclude(); });
    m->materials->Bind(wxEVT_CHOICE, [this](wxCommandEvent&) { m->guarded([&] { m->select(); }); });
    for (auto* choice : {m->parameter, m->property})
        choice->Bind(wxEVT_CHOICE, [this](wxCommandEvent&) {
            m->guarded([&] {
                m->update_controls();
                m->render();
            });
        });
    show->Bind(wxEVT_CHECKBOX, [this, show](wxCommandEvent&) {
        m->graph->show_points = show->GetValue();
        m->graph->Refresh();
    });
    m->guarded([&] { m->refresh(); });
    update_colors();
    FitInside();
}
CalibratedMaterialsPanel::~CalibratedMaterialsPanel() = default;
bool CalibratedMaterialsPanel::Show(bool show)
{
    if (show)
        m->guarded([&] { m->refresh(); });
    return wxScrolledWindow::Show(show);
}
void CalibratedMaterialsPanel::update_colors()
{
    wxGetApp().UpdateDarkUIWin(this);
    for (auto* grid : {m->sheet, m->data}) {
        grid->SetDefaultCellBackgroundColour(GetBackgroundColour());
        grid->SetDefaultCellTextColour(GetForegroundColour());
        grid->SetLabelBackgroundColour(GetBackgroundColour());
        grid->SetLabelTextColour(GetForegroundColour());
        grid->ForceRefresh();
    }
}
} // namespace Slic3r::GUI
