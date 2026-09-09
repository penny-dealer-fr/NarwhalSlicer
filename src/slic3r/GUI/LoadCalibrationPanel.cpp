#include "LoadCalibrationPanel.hpp"
#include "GUI_App.hpp"
#include "I18N.hpp"
#include "Plater.hpp"
#include "MainFrame.hpp"
#include "libslic3r/LoadCalibration.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/Print.hpp"

#include "libslic3r/Utils.hpp"
#include "nlohmann/json.hpp"
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <wx/button.h>
#include <wx/choice.h>
#include <wx/clrpicker.h>
#include <wx/grid.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/timer.h>
#include <wx/filename.h>
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
    wxChoice *setting{}, *study_choice{}, *unit{}, *base{}, *target{}, *group{};
    wxTextCtrl *values{}, *start{}, *end{}, *step{}, *material{}, *type{}, *xy{}, *z{}, *summary{};
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
    std::string color_value() const {
        return legacy_color.empty() ? std::string(color->GetColour().GetAsString(wxC2S_HTML_SYNTAX).ToUTF8().data()) : legacy_color;
    }
    void set_color(const std::string& saved) {
        wxColour parsed(wxString::FromUTF8(saved));
        legacy_color = parsed.IsOk() ? std::string() : saved;
        color->SetColour(parsed.IsOk() ? parsed : *wxWHITE);
        color->SetToolTip(legacy_color.empty() ? _L("Choose material color") :
            wxString::Format(_L("Saved color: %s. Choose a swatch to replace it."),wxString::FromUTF8(saved)));
    }
    DynamicPrintConfig baseline_config() const {
        DynamicPrintConfig baseline;
        for (auto it=study.at("baseline").begin();it!=study.at("baseline").end();++it)
            baseline.set_deserialize_strict(it.key(),it.value().get<std::string>());
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
    void setting_help()
    {
        const std::string key = LC::settings()[setting->GetSelection()].key;
        const auto* def       = print_config_def.get(key);
        wxString text         = _L("Enter comma-separated values, or fill the numeric range and click Use range.");
        if (def && !def->enum_values.empty()) {
            text = _L("Available values: ");
            for (const auto& value : def->enum_values)
                text += wxString::FromUTF8(value) + "  ";
        }
        if (key == "slow_down_layer_time")
            text += _L(" Minimum layer time is a cooling target; actual layer time can be longer.");
        help->SetLabel(text);
        help->Wrap(panel->FromDIP(850));
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
            grid->SetCellValue(int(i), 1, wxString::FromUTF8(s.value));
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
            if (seen.insert(s.value).second)
                group->Append(wxString::FromUTF8(s.value));
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
        for (unsigned i = 0; i < group->GetCount(); ++i) {
            if (i)
                restored_values += ",";
            restored_values += group->GetString(i);
        }
        values->ChangeValue(restored_values);
        if (!samples.empty())
            repeats->SetValue(int(std::count_if(samples.begin(), samples.end(), [&](const LC::Sample& sample) {
                return sample.value == samples.front().value && sample.orientation == "XY";
            })));
        for (size_t i = 0; i < load_materials().size(); ++i)
            if (load_materials()[i].key == study.value("base_key", ""))
                base->SetSelection(int(i));
        const auto selected = wxString::FromUTF8(study.value("selected_value", ""));
        if (group->FindString(selected) != wxNOT_FOUND)
            group->SetStringSelection(selected);
        setting_help();
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
        std::vector<std::string> list;
        std::istringstream input(utf8(values));
        std::string v;
        while (std::getline(input, v, ',')) {
            const auto a = v.find_first_not_of(" \t\r\n"), b = v.find_last_not_of(" \t\r\n");
            if (a != std::string::npos)
                list.push_back(v.substr(a, b - a + 1));
        }
        auto planned          = LC::make_samples(list, repeats->GetValue());
        const auto baseline   = wxGetApp().preset_bundle->full_config_secure();
        const std::string key = LC::settings()[setting->GetSelection()].key;
        std::vector<DynamicPrintConfig> configs;
        for (const auto& s : planned)
            configs.push_back(LC::setting_config(baseline, key, s.value));
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
                    if (boost::filesystem::exists(stem.string() + ".complete.json") && boost::filesystem::exists(stem.string() + ".gcode"))
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
                    write_json(stem.string() + ".complete.json",
                                           {{"id", planned[i].id}, {"value", planned[i].value}, {"orientation", planned[i].orientation}});
                    boost::system::error_code ignored;
                    boost::filesystem::remove(stem.string()+".error.json",ignored);
                    } catch(const CanceledException&) { throw; }
                    catch(const std::exception& e) {
                        if(cancel) throw CanceledException();
                        ++failed;
                        write_json(stem.string()+".error.json",{{"id",planned[i].id},{"error",e.what()}});
                    }
                    { std::lock_guard<std::mutex> lock(mutex); active_print.reset(); }

                }
                message(failed ? std::to_string(failed)+" hook(s) could not be sliced. See per-hook .error.json files in "+path.string()
                               : "All hooks sliced. Projects and G-code: " + path.string());
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
            configs.push_back(LC::setting_config(baseline, study.at("setting").get<std::string>(), sample.value));
        run_worker(std::move(configs), (current_path / "CNC_Testhook.stl").string());
    }
    void save_material()
    {
        persist();
        if (base->GetSelection() < 0 || group->GetSelection() < 0)
            throw std::invalid_argument("Select a base material and setting value.");
        const std::string value = group->GetStringSelection().ToUTF8().data();
        auto calibrated = LC::calibrate(load_materials().at(base->GetSelection()), utf8(material), samples, value, number(xy), number(z));
        const auto path = root_path() / "materials.json";
        Json library    = boost::filesystem::exists(path) ? read_json(path) : Json{{"version", 1}, {"materials", Json::array()}};
        if (library.at("version") != 1)
            throw std::runtime_error("Unsupported material library version.");
        const int selected = target->GetSelection();
        if (selected > 0)
            calibrated.key = load_materials().at(SA::builtin_materials().size() + selected - 1).key;
        else
            calibrated.key = "calibrated_" + boost::filesystem::unique_path("%%%%-%%%%-%%%%").string();
        calibrated.provenance += "; type: " + utf8(type) + "; color: " + color_value() + "; study: " + current_path.filename().string() +
                                 "; parameter: " + study.at("setting").get<std::string>();
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
    add_text(_L("Sweep one setting using the current printer, process, and first filament. Every repeat gets an XY hook and a Z hook, each "
                "on its own plate, so temperatures and cooling tests stay independent. Z hooks stand upright; configure support and brim "
                "in Prepare before generating. Each plate is saved as a project and automatically sliced to G-code."));
    auto* form = new wxFlexGridSizer(2, gap, gap);
    form->AddGrowableCol(1, 1);
    root->Add(form, 0, wxEXPAND | wxALL, gap);
    auto field = [&](const wxString& label, wxWindow* control) {
        form->Add(new wxStaticText(this, wxID_ANY, label), 0, wxALIGN_CENTER_VERTICAL);
        form->Add(control, 1, wxEXPAND);
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
    m->color = new wxColourPickerCtrl(this,wxID_ANY,*wxWHITE);
    field(_L("Color"),m->color);
    m->color->Bind(wxEVT_COLOURPICKER_CHANGED,[this](wxColourPickerEvent&){m->legacy_color.clear();});
    m->setting  = new wxChoice(this, wxID_ANY);
    for (const auto& s : LC::settings()) {
        const auto* d = print_config_def.get(s.key);
        m->setting->Append(d ? _(d->label) : _L("Bed temperature"));
    }
    m->setting->SetSelection(1);
    field(_L("Setting to vary"), m->setting);
    m->values          = text(_L("Setting values (comma-separated)"), "15,30,50");
    m->start           = text(_L("Range start"), "15");
    m->end             = text(_L("Range end"), "50");
    m->step            = text(_L("Range increment"), "5");
    auto* range_button = new wxButton(this, wxID_ANY, _L("Use range"));
    field(wxEmptyString, range_button);
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
    add_text(_L("Calibrate one setting group at a time. Supply validated peak tensile stress per unit force (MPa/N) for the hook and "
                "fixture in each orientation. Failure load alone cannot identify material stress. At least two failures in XY and Z are "
                "required; survived samples cannot be treated as failures. The minimum failure estimates tensile limits; elastic and shear "
                "properties remain inherited from the base material."));
    auto* calibration = new wxFlexGridSizer(2, gap, gap);
    calibration->AddGrowableCol(1, 1);
    root->Add(calibration, 0, wxEXPAND | wxALL, gap);
    form    = calibration;
    m->base = new wxChoice(this, wxID_ANY);
    field(_L("Base mechanical material"), m->base);
    m->target = new wxChoice(this, wxID_ANY);
    field(_L("Save to"), m->target);
    m->group = new wxChoice(this, wxID_ANY);
    field(_L("Setting value to calibrate"), m->group);
    m->xy               = text(_L("XY peak tensile stress / force (MPa/N)"), "");
    m->z                = text(_L("Z peak tensile stress / force (MPa/N)"), "");
    m->calibrate_button = new wxButton(this, wxID_ANY, _L("Save calibrated material"));
    root->Add(m->calibrate_button, 0, wxALL, gap);
    open_hook->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) {
        m->guarded([&] {
            m->persist();
            std::vector<LC::Sample> selected;
            for(int row=0;row<m->grid->GetNumberRows();++row) {
                bool included=false;
                for(int column=0;column<m->grid->GetNumberCols();++column)
                    included=included || m->grid->IsInSelection(row,column);
                if(included) selected.push_back(m->samples.at(size_t(row)));
            }
            if(selected.empty()) {
                const int row=m->grid->GetGridCursorRow();
                if(row>=0 && size_t(row)<m->samples.size()) selected.push_back(m->samples[size_t(row)]);
            }
            if(selected.empty()) throw std::invalid_argument("Select one or more hooks in the results table.");
            auto project=LC::arrange_hooks((m->current_path/"CNC_Testhook.stl").string(),selected,m->baseline_config(),
                m->study.at("setting").get<std::string>());
            const auto path=m->current_path/boost::filesystem::unique_path("selected-hooks-%%%%-%%%%.3mf");
            LC::store_project(path.string(),project);
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
} // namespace Slic3r::GUI
