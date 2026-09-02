#include "StrengthAnalysisPanel.hpp"

#include "I18N.hpp"
#include "Plater.hpp"

#include "libslic3r/Geometry.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"

#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/choice.h>
#include <wx/dcbuffer.h>
#include <wx/listbox.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/statbox.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/thread.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <sstream>

namespace Slic3r::GUI {

wxDEFINE_EVENT(EVT_STRENGTH_ANALYSIS_FINISHED, wxThreadEvent);

namespace {

namespace SA = StrengthAnalysis;

wxString number(double value)
{
    return wxString::Format("%.8g", value);
}

bool read_number(wxTextCtrl *control, double &value)
{
    return control != nullptr && control->GetValue().ToDouble(&value) && std::isfinite(value);
}

wxTextCtrl *number_input(wxWindow *parent, double value, int width = 92)
{
    return new wxTextCtrl(parent, wxID_ANY, number(value), wxDefaultPosition, parent->FromDIP(wxSize(width, -1)));
}

void add_labeled(wxSizer *sizer, wxWindow *parent, const wxString &label, wxWindow *control, int proportion = 0)
{
    sizer->Add(new wxStaticText(parent, wxID_ANY, label), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, parent->FromDIP(6));
    sizer->Add(control, proportion, wxALIGN_CENTER_VERTICAL | (proportion ? wxEXPAND : 0));
}

void add_labeled(wxSizer *sizer, wxWindow *parent, const wxString &label, wxSizer *control, int proportion = 0)
{
    sizer->Add(new wxStaticText(parent, wxID_ANY, label), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, parent->FromDIP(6));
    sizer->Add(control, proportion, proportion ? wxEXPAND : 0);
}

wxBoxSizer *vector_editor(wxWindow *parent, wxTextCtrl *controls[3], const Vec3d &value, const wxString &unit = {})
{
    auto *row = new wxBoxSizer(wxHORIZONTAL);
    static const std::array<wxString, 3> axes{"X", "Y", "Z"};
    for (int axis = 0; axis < 3; ++axis) {
        row->Add(new wxStaticText(parent, wxID_ANY, axes[axis]), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, parent->FromDIP(3));
        controls[axis] = number_input(parent, value[axis], 76);
        row->Add(controls[axis], 0, wxRIGHT, parent->FromDIP(6));
    }
    if (!unit.empty())
        row->Add(new wxStaticText(parent, wxID_ANY, unit), 0, wxALIGN_CENTER_VERTICAL);
    return row;
}

Vec3d read_vector(wxTextCtrl *const controls[3], const Vec3d &fallback)
{
    Vec3d output = fallback;
    for (int axis = 0; axis < 3; ++axis) {
        double value = output[axis];
        if (read_number(controls[axis], value))
            output[axis] = value;
    }
    return output;
}

wxArrayString load_type_names()
{
    return {_L("Fixed region"), _L("Local force"), _L("Directional force"), _L("Bearing force"),
            _L("Impact force"), _L("Global force")};
}

wxArrayString infill_names()
{
    return {_L("Rectilinear"), _L("Grid"), _L("Triangles"), _L("Honeycomb"), _L("Cubic"), _L("Gyroid")};
}

Slic3r::InfillPattern print_pattern(SA::InfillPattern pattern)
{
    switch (pattern) {
    case SA::InfillPattern::Rectilinear: return ipRectilinear;
    case SA::InfillPattern::Grid: return ipGrid;
    case SA::InfillPattern::Triangles: return ipTriangles;
    case SA::InfillPattern::Honeycomb: return ipHoneycomb;
    case SA::InfillPattern::Cubic: return ipCubic;
    case SA::InfillPattern::Gyroid: return ipGyroid;
    }
    return ipGyroid;
}

wxString vector_text(const Vec3d &value)
{
    return wxString::Format("(%.4g, %.4g, %.4g)", value.x(), value.y(), value.z());
}

bool meshes_equal(const indexed_triangle_set &lhs, const indexed_triangle_set &rhs)
{
    if (lhs.vertices.size() != rhs.vertices.size() || lhs.indices.size() != rhs.indices.size())
        return false;
    for (size_t index = 0; index < lhs.vertices.size(); ++index) {
        if (!lhs.vertices[index].isApprox(rhs.vertices[index]))
            return false;
    }
    for (size_t index = 0; index < lhs.indices.size(); ++index) {
        if ((lhs.indices[index].array() != rhs.indices[index].array()).any())
            return false;
    }
    return true;
}

bool nearly_equal(double lhs, double rhs)
{
    return std::abs(lhs - rhs) <= 1e-7 * std::max({1.0, std::abs(lhs), std::abs(rhs)});
}

bool material_properties_match(const SA::Material &lhs, const SA::Material &rhs)
{
    return nearly_equal(lhs.density_kg_m3, rhs.density_kg_m3) &&
        nearly_equal(lhs.elastic_modulus_xy_pa, rhs.elastic_modulus_xy_pa) &&
        nearly_equal(lhs.elastic_modulus_z_pa, rhs.elastic_modulus_z_pa) &&
        nearly_equal(lhs.poisson_xy, rhs.poisson_xy) &&
        nearly_equal(lhs.shear_modulus_xy_pa, rhs.shear_modulus_xy_pa) &&
        nearly_equal(lhs.shear_modulus_xz_pa, rhs.shear_modulus_xz_pa) &&
        nearly_equal(lhs.yield_strength_xy_pa, rhs.yield_strength_xy_pa) &&
        nearly_equal(lhs.yield_strength_z_pa, rhs.yield_strength_z_pa) &&
        nearly_equal(lhs.ultimate_strength_xy_pa, rhs.ultimate_strength_xy_pa) &&
        nearly_equal(lhs.ultimate_strength_z_pa, rhs.ultimate_strength_z_pa) &&
        nearly_equal(lhs.shear_strength_xy_pa, rhs.shear_strength_xy_pa) &&
        nearly_equal(lhs.shear_strength_xz_pa, rhs.shear_strength_xz_pa);
}

} // namespace

StrengthLoadPanel::StrengthLoadPanel(wxWindow *parent, Plater *plater, std::shared_ptr<StrengthAnalysisSession> session)
    : wxScrolledWindow(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL | wxTAB_TRAVERSAL)
    , m_plater(plater)
    , m_session(std::move(session))
{
    SetBackgroundColour(*wxWHITE);
    SetScrollRate(0, FromDIP(12));
    build_ui();
    Bind(EVT_STRENGTH_ANALYSIS_FINISHED, [this](wxThreadEvent &) { on_analysis_finished(); });
}

StrengthLoadPanel::~StrengthLoadPanel()
{
    m_cancel = true;
    if (m_worker.joinable())
        m_worker.join();
}

void StrengthLoadPanel::build_ui()
{
    const int gap = FromDIP(8);
    auto *root = new wxBoxSizer(wxVERTICAL);
    root->Add(new wxStaticText(this, wxID_ANY, _L("Strength Analysis — Load Setup")), 0, wxLEFT | wxRIGHT | wxTOP, gap);
    m_object_label = new wxStaticText(this, wxID_ANY, _L("Select one model object in Prepare."));
    root->Add(m_object_label, 0, wxEXPAND | wxALL, gap);

    auto *material = new wxStaticBoxSizer(wxVERTICAL, this, _L("Material and print direction"));
    auto *material_choice_row = new wxBoxSizer(wxHORIZONTAL);
    m_material_choice = new wxChoice(this, wxID_ANY);
    for (const SA::Material &item : SA::builtin_materials())
        m_material_choice->Append(wxString::FromUTF8(item.name));
    m_material_choice->Append(_L("Custom material"));
    add_labeled(material_choice_row, this, _L("Material"), m_material_choice, 1);
    material->Add(material_choice_row, 0, wxEXPAND | wxALL, gap);

    static const std::array<wxString, 12> material_labels{
        _L("Density (kg/m³)"), _L("Elastic modulus XY (GPa)"), _L("Elastic modulus Z (GPa)"), _L("Poisson ratio XY"),
        _L("Shear modulus XY (GPa)"), _L("Shear modulus XZ (GPa)"), _L("Yield strength XY (MPa)"),
        _L("Yield strength Z (MPa)"), _L("Ultimate strength XY (MPa)"), _L("Ultimate strength Z (MPa)"),
        _L("Shear strength XY (MPa)"), _L("Shear strength XZ (MPa)")};
    auto *properties = new wxFlexGridSizer(2, gap, gap);
    properties->AddGrowableCol(1, 1);
    for (const wxString &label : material_labels) {
        auto *field = number_input(this, 0.0, 130);
        m_material_fields.push_back(field);
        add_labeled(properties, this, label, field, 1);
    }
    material->Add(properties, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);

    auto *calibration = new wxFlexGridSizer(2, gap, gap);
    calibration->AddGrowableCol(1, 1);
    static const std::array<wxString, 5> calibration_labels{
        _L("Calibration: modulus XY scale"), _L("Calibration: modulus Z scale"), _L("Calibration: strength XY scale"),
        _L("Calibration: strength Z scale"), _L("Calibration: shear scale")};
    for (const wxString &label : calibration_labels) {
        auto *field = number_input(this, 1.0, 130);
        m_calibration_fields.push_back(field);
        add_labeled(calibration, this, label, field, 1);
    }
    m_calibration_source = new wxTextCtrl(this, wxID_ANY);
    add_labeled(calibration, this, _L("Calibration source / coupon"), m_calibration_source, 1);
    material->Add(calibration, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    auto *layer_row = new wxBoxSizer(wxHORIZONTAL);
    layer_row->Add(new wxStaticText(this, wxID_ANY, _L("Layer-normal axis in model coordinates")), 0,
                   wxALIGN_CENTER_VERTICAL | wxRIGHT, gap);
    layer_row->Add(vector_editor(this, m_layer_axis, Vec3d::UnitZ()), 0);
    material->Add(layer_row, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    root->Add(material, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);

    auto *loads = new wxStaticBoxSizer(wxVERTICAL, this, _L("Loads, constraints, and per-load safety factor"));
    m_load_list = new wxListBox(this, wxID_ANY, wxDefaultPosition, FromDIP(wxSize(-1, 110)));
    loads->Add(m_load_list, 0, wxEXPAND | wxALL, gap);
    auto *load_buttons = new wxBoxSizer(wxHORIZONTAL);
    auto *add_load = new wxButton(this, wxID_ANY, _L("Add load"));
    auto *remove_load = new wxButton(this, wxID_ANY, _L("Remove load"));
    load_buttons->Add(add_load, 0, wxRIGHT, gap);
    load_buttons->Add(remove_load, 0);
    loads->Add(load_buttons, 0, wxLEFT | wxRIGHT | wxBOTTOM, gap);

    auto *load_grid = new wxFlexGridSizer(2, gap, gap);
    load_grid->AddGrowableCol(1, 1);
    m_load_name = new wxTextCtrl(this, wxID_ANY);
    add_labeled(load_grid, this, _L("Name"), m_load_name, 1);
    m_load_type = new wxChoice(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, load_type_names());
    add_labeled(load_grid, this, _L("Type"), m_load_type, 1);
    m_load_active = new wxCheckBox(this, wxID_ANY, _L("Enabled"));
    add_labeled(load_grid, this, _L("State"), m_load_active);
    m_load_whole_model = new wxCheckBox(this, wxID_ANY, _L("Apply across the whole model"));
    add_labeled(load_grid, this, _L("Region"), m_load_whole_model);
    add_labeled(load_grid, this, _L("Region center (mm)"), vector_editor(this, m_load_center, Vec3d::Zero()), 1);
    m_load_radius = number_input(this, 5.0);
    add_labeled(load_grid, this, _L("Region radius (mm)"), m_load_radius, 1);
    add_labeled(load_grid, this, _L("Direction"), vector_editor(this, m_load_direction, Vec3d::UnitZ()), 1);
    m_load_magnitude = number_input(this, 100.0);
    add_labeled(load_grid, this, _L("Magnitude (N)"), m_load_magnitude, 1);
    m_load_impact = number_input(this, 2.0);
    add_labeled(load_grid, this, _L("Impact factor"), m_load_impact, 1);
    m_load_safety_factor = number_input(this, 1.5);
    add_labeled(load_grid, this, _L("Target safety factor"), m_load_safety_factor, 1);
    m_strength_basis = new wxChoice(this, wxID_ANY, wxDefaultPosition, wxDefaultSize,
                                    {_L("Yield"), _L("Ultimate"), _L("Calibrated yield")});
    add_labeled(load_grid, this, _L("Strength basis"), m_strength_basis, 1);
    loads->Add(load_grid, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    root->Add(loads, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);

    auto *environment = new wxStaticBoxSizer(wxVERTICAL, this, _L("Gravity, infill, and preserve regions"));
    m_gravity_enabled = new wxCheckBox(this, wxID_ANY, _L("Include gravity and estimated part mass"));
    environment->Add(m_gravity_enabled, 0, wxALL, gap);
    auto *gravity_row = new wxBoxSizer(wxHORIZONTAL);
    gravity_row->Add(new wxStaticText(this, wxID_ANY, _L("Gravity acceleration")), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, gap);
    gravity_row->Add(vector_editor(this, m_gravity, Vec3d(0.0, 0.0, -9.80665), _L("m/s²")), 0);
    environment->Add(gravity_row, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    auto *infill_grid = new wxFlexGridSizer(2, gap, gap);
    infill_grid->AddGrowableCol(1, 1);
    m_background_pattern = new wxChoice(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, infill_names());
    m_dense_pattern = new wxChoice(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, infill_names());
    m_background_density = number_input(this, 20.0);
    m_dense_density = number_input(this, 65.0);
    m_dense_threshold = number_input(this, 70.0);
    add_labeled(infill_grid, this, _L("Background infill pattern"), m_background_pattern, 1);
    add_labeled(infill_grid, this, _L("Background infill density (%)"), m_background_density, 1);
    add_labeled(infill_grid, this, _L("Dense-region pattern"), m_dense_pattern, 1);
    add_labeled(infill_grid, this, _L("Dense-region density (%)"), m_dense_density, 1);
    add_labeled(infill_grid, this, _L("Dense stress threshold (%)"), m_dense_threshold, 1);
    environment->Add(infill_grid, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);

    m_preserve_list = new wxListBox(this, wxID_ANY, wxDefaultPosition, FromDIP(wxSize(-1, 82)));
    environment->Add(m_preserve_list, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    auto *preserve_row = new wxBoxSizer(wxHORIZONTAL);
    preserve_row->Add(new wxStaticText(this, wxID_ANY, _L("Preserve center")), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, gap);
    preserve_row->Add(vector_editor(this, m_preserve_center, Vec3d::Zero(), _L("mm")), 0, wxRIGHT, gap);
    m_preserve_radius = number_input(this, 5.0, 76);
    add_labeled(preserve_row, this, _L("Radius"), m_preserve_radius);
    environment->Add(preserve_row, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    auto *preserve_buttons = new wxBoxSizer(wxHORIZONTAL);
    auto *add_preserve = new wxButton(this, wxID_ANY, _L("Add preserve region"));
    auto *remove_preserve = new wxButton(this, wxID_ANY, _L("Remove preserve region"));
    preserve_buttons->Add(add_preserve, 0, wxRIGHT, gap);
    preserve_buttons->Add(remove_preserve, 0);
    environment->Add(preserve_buttons, 0, wxLEFT | wxRIGHT | wxBOTTOM, gap);
    root->Add(environment, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);

    auto *criteria = new wxStaticBoxSizer(wxVERTICAL, this, _L("Optimization criteria"));
    auto *criteria_grid = new wxFlexGridSizer(2, gap, gap);
    criteria_grid->AddGrowableCol(1, 1);
    m_minimum_safety_factor = number_input(this, 1.5);
    m_maximum_displacement = number_input(this, 0.0);
    add_labeled(criteria_grid, this, _L("Minimum safety factor"), m_minimum_safety_factor, 1);
    add_labeled(criteria_grid, this, _L("Maximum displacement (mm; 0 disables)"), m_maximum_displacement, 1);
    static const std::array<wxString, 4> weight_labels{
        _L("Mass weight"), _L("Stiffness weight"), _L("Support weight"), _L("Print-time weight")};
    for (int index = 0; index < 4; ++index) {
        m_objective_weights[index] = number_input(this, index < 2 ? 1.0 : 0.25);
        add_labeled(criteria_grid, this, weight_labels[index], m_objective_weights[index], 1);
    }
    criteria->Add(criteria_grid, 0, wxEXPAND | wxALL, gap);
    root->Add(criteria, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);

    auto *actions = new wxBoxSizer(wxHORIZONTAL);
    auto *save_button = new wxButton(this, wxID_ANY, _L("Save setup"));
    m_run_button = new wxButton(this, wxID_ANY, _L("Run analysis"));
    m_cancel_button = new wxButton(this, wxID_ANY, _L("Cancel"));
    m_dense_button = new wxButton(this, wxID_ANY, _L("Create dense modifier"));
    m_orientation_button = new wxButton(this, wxID_ANY, _L("Apply best orientation"));
    m_settings_button = new wxButton(this, wxID_ANY, _L("Apply optimized settings"));
    for (wxButton *button : {save_button, m_run_button, m_cancel_button, m_dense_button, m_orientation_button, m_settings_button})
        actions->Add(button, 0, wxRIGHT, gap);
    root->Add(actions, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    m_status_label = new wxStaticText(this, wxID_ANY, _L("Not run. Results are engineering estimates, not certification-grade FEA."));
    root->Add(m_status_label, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    SetSizer(root);

    m_material_choice->Bind(wxEVT_CHOICE, [this](wxCommandEvent &) {
        const int index = m_material_choice->GetSelection();
        if (index >= 0 && size_t(index) < SA::builtin_materials().size()) {
            m_session->setup.material = SA::builtin_materials()[size_t(index)];
        } else {
            m_session->setup.material.key = "custom";
            m_session->setup.material.name = "Custom material";
            m_session->setup.material.provenance =
                "User-entered material properties; verify against a filament datasheet and printed coupons.";
        }
        populate_material_fields();
        mark_stale();
    });
    m_load_list->Bind(wxEVT_LISTBOX, [this](wxCommandEvent &) {
        const int selected = m_load_list->GetSelection();
        save_current_load_editor();
        refresh_load_list();
        load_current_load_editor(selected);
    });
    add_load->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
        save_current_load_editor();
        SA::Load load;
        load.name = "Load " + std::to_string(m_session->setup.loads.size() + 1);
        m_session->setup.loads.push_back(load);
        refresh_load_list();
        load_current_load_editor(int(m_session->setup.loads.size()) - 1);
        mark_stale();
    });
    remove_load->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
        const int selected = m_load_list->GetSelection();
        if (selected >= 0 && size_t(selected) < m_session->setup.loads.size()) {
            m_session->setup.loads.erase(m_session->setup.loads.begin() + selected);
            m_current_load = -1;
            refresh_load_list();
            load_current_load_editor(std::min(selected, int(m_session->setup.loads.size()) - 1));
            mark_stale();
        }
    });
    add_preserve->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
        SA::SphericalRegion region;
        region.center_mm = read_vector(m_preserve_center, Vec3d::Zero());
        read_number(m_preserve_radius, region.radius_mm);
        m_session->setup.preserve_regions.push_back(region);
        refresh_preserve_list();
        mark_stale();
    });
    remove_preserve->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
        const int selected = m_preserve_list->GetSelection();
        if (selected >= 0 && size_t(selected) < m_session->setup.preserve_regions.size()) {
            m_session->setup.preserve_regions.erase(m_session->setup.preserve_regions.begin() + selected);
            refresh_preserve_list();
            mark_stale();
        }
    });
    save_button->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
        if (collect_setup(true)) {
            persist_setup();
            m_status_label->SetLabel(_L("Strength setup saved in the selected model object's 3MF metadata."));
        }
    });
    m_run_button->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { run_analysis(); });
    m_cancel_button->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { cancel_analysis(); });
    m_dense_button->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { create_dense_modifier(); });
    m_orientation_button->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { apply_recommended_orientation(); });
    m_settings_button->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { apply_optimized_settings(); });

    std::vector<wxTextCtrl *> stale_text_fields = m_material_fields;
    stale_text_fields.insert(stale_text_fields.end(), m_calibration_fields.begin(), m_calibration_fields.end());
    stale_text_fields.push_back(m_calibration_source);
    stale_text_fields.push_back(m_load_name);
    stale_text_fields.push_back(m_load_radius);
    stale_text_fields.push_back(m_load_magnitude);
    stale_text_fields.push_back(m_load_impact);
    stale_text_fields.push_back(m_load_safety_factor);
    stale_text_fields.push_back(m_background_density);
    stale_text_fields.push_back(m_dense_density);
    stale_text_fields.push_back(m_dense_threshold);
    stale_text_fields.push_back(m_preserve_radius);
    stale_text_fields.push_back(m_minimum_safety_factor);
    stale_text_fields.push_back(m_maximum_displacement);
    for (int axis = 0; axis < 3; ++axis) {
        stale_text_fields.push_back(m_layer_axis[axis]);
        stale_text_fields.push_back(m_load_center[axis]);
        stale_text_fields.push_back(m_load_direction[axis]);
        stale_text_fields.push_back(m_gravity[axis]);
        stale_text_fields.push_back(m_preserve_center[axis]);
    }
    for (wxTextCtrl *field : m_objective_weights)
        stale_text_fields.push_back(field);
    for (wxTextCtrl *field : stale_text_fields)
        field->Bind(wxEVT_TEXT, [this](wxCommandEvent &) { mark_stale(); });
    for (wxChoice *choice : {m_load_type, m_strength_basis, m_background_pattern, m_dense_pattern})
        choice->Bind(wxEVT_CHOICE, [this](wxCommandEvent &) { mark_stale(); });
    for (wxCheckBox *checkbox : {m_load_active, m_load_whole_model, m_gravity_enabled})
        checkbox->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent &) { mark_stale(); });

    m_cancel_button->Disable();
    m_dense_button->Disable();
    m_orientation_button->Disable();
    m_settings_button->Disable();
}

void StrengthLoadPanel::activate()
{
    load_selected_object();
}

void StrengthLoadPanel::load_selected_object()
{
    int index = m_plater->get_selected_object_idx();
    if (index < 0 && m_plater->model().objects.size() == 1)
        index = 0;
    if (index < 0 || size_t(index) >= m_plater->model().objects.size()) {
        m_object_label->SetLabel(_L("Select one model object in Prepare."));
        m_run_button->Disable();
        return;
    }

    ModelObject *object = m_plater->model().objects[size_t(index)];
    const indexed_triangle_set current_mesh = object->raw_mesh().its;
    const bool changed_geometry = index == m_session->object_index && !meshes_equal(current_mesh, m_session->mesh);
    if (index != m_session->object_index) {
        m_session->object_index = index;
        m_session->setup = SA::Setup{};
        if (const auto *option = object->config.get().option<ConfigOptionString>("strength_analysis_setup")) {
            std::string error;
            if (!option->value.empty() && !SA::deserialize_setup_from_config(option->value, m_session->setup, &error))
                m_status_label->SetLabel(wxString::Format(_L("Saved strength setup could not be read: %s"), wxString::FromUTF8(error)));
        }
        m_session->result = SA::Result{};
        m_session->stale = true;
        ++m_session->revision;
        m_current_load = -1;
        populate_from_setup();
    } else if (changed_geometry) {
        m_session->stale = true;
        ++m_session->revision;
        m_dense_button->Disable();
        m_orientation_button->Disable();
        m_settings_button->Disable();
        m_status_label->SetLabel(_L("Model geometry changed; run the analysis again."));
        if (m_result_callback)
            m_result_callback();
    }
    m_session->mesh = current_mesh;
    m_object_label->SetLabel(wxString::Format(_L("Object: %s — %zu vertices, %zu triangles"),
        wxString::FromUTF8(object->name), current_mesh.vertices.size(), current_mesh.indices.size()));
    m_run_button->Enable(!current_mesh.empty());
}

void StrengthLoadPanel::populate_material_fields()
{
    const SA::Setup &setup = m_session->setup;
    int material_index = int(SA::builtin_materials().size());
    for (size_t index = 0; index < SA::builtin_materials().size(); ++index)
        if (SA::builtin_materials()[index].key == setup.material.key) material_index = int(index);
    m_material_choice->SetSelection(material_index);
    const SA::Material &m = setup.material;
    const std::array<double, 12> values{m.density_kg_m3, m.elastic_modulus_xy_pa / 1e9, m.elastic_modulus_z_pa / 1e9,
        m.poisson_xy, m.shear_modulus_xy_pa / 1e9, m.shear_modulus_xz_pa / 1e9, m.yield_strength_xy_pa / 1e6,
        m.yield_strength_z_pa / 1e6, m.ultimate_strength_xy_pa / 1e6, m.ultimate_strength_z_pa / 1e6,
        m.shear_strength_xy_pa / 1e6, m.shear_strength_xz_pa / 1e6};
    for (size_t index = 0; index < values.size(); ++index)
        m_material_fields[index]->ChangeValue(number(values[index]));
    const std::array<double, 5> scales{m.calibration.modulus_xy_scale, m.calibration.modulus_z_scale,
        m.calibration.strength_xy_scale, m.calibration.strength_z_scale, m.calibration.shear_scale};
    for (size_t index = 0; index < scales.size(); ++index)
        m_calibration_fields[index]->ChangeValue(number(scales[index]));
    m_calibration_source->ChangeValue(wxString::FromUTF8(m.calibration.source));
}

void StrengthLoadPanel::populate_from_setup()
{
    const SA::Setup &setup = m_session->setup;
    populate_material_fields();
    for (int axis = 0; axis < 3; ++axis)
        m_layer_axis[axis]->ChangeValue(number(setup.print_layer_axis[axis]));
    m_gravity_enabled->SetValue(setup.gravity.enabled);
    for (int axis = 0; axis < 3; ++axis)
        m_gravity[axis]->ChangeValue(number(setup.gravity.acceleration_m_s2[axis]));
    m_background_pattern->SetSelection(int(setup.infill.background_pattern));
    m_dense_pattern->SetSelection(int(setup.infill.dense_pattern));
    m_background_density->ChangeValue(number(setup.infill.background_density * 100.0));
    m_dense_density->ChangeValue(number(setup.infill.dense_density * 100.0));
    m_dense_threshold->ChangeValue(number(setup.infill.dense_stress_threshold * 100.0));
    m_minimum_safety_factor->ChangeValue(number(setup.criteria.minimum_safety_factor));
    m_maximum_displacement->ChangeValue(number(setup.criteria.maximum_displacement_mm));
    const std::array<double, 4> weights{setup.criteria.mass_weight, setup.criteria.stiffness_weight,
        setup.criteria.support_weight, setup.criteria.print_time_weight};
    for (int index = 0; index < 4; ++index)
        m_objective_weights[index]->ChangeValue(number(weights[index]));
    refresh_load_list();
    load_current_load_editor(setup.loads.empty() ? -1 : 0);
    refresh_preserve_list();
}

bool StrengthLoadPanel::collect_setup(bool show_errors)
{
    save_current_load_editor();
    SA::Setup &setup = m_session->setup;
    SA::Material &m = setup.material;
    const int selected_material = m_material_choice->GetSelection();
    if (selected_material >= 0 && size_t(selected_material) < SA::builtin_materials().size()) {
        const SA::MaterialCalibration calibration = m.calibration;
        m = SA::builtin_materials()[size_t(selected_material)];
        m.calibration = calibration;
    } else {
        m.key = "custom";
        m.name = "Custom material";
        m.provenance = "User-entered material properties; verify against a filament datasheet and printed coupons.";
    }
    std::array<double, 12> values{};
    bool numeric = true;
    for (size_t index = 0; index < values.size(); ++index)
        numeric = read_number(m_material_fields[index], values[index]) && numeric;
    m.density_kg_m3 = values[0];
    m.elastic_modulus_xy_pa = values[1] * 1e9;
    m.elastic_modulus_z_pa = values[2] * 1e9;
    m.poisson_xy = values[3];
    m.shear_modulus_xy_pa = values[4] * 1e9;
    m.shear_modulus_xz_pa = values[5] * 1e9;
    m.yield_strength_xy_pa = values[6] * 1e6;
    m.yield_strength_z_pa = values[7] * 1e6;
    m.ultimate_strength_xy_pa = values[8] * 1e6;
    m.ultimate_strength_z_pa = values[9] * 1e6;
    m.shear_strength_xy_pa = values[10] * 1e6;
    m.shear_strength_xz_pa = values[11] * 1e6;
    if (selected_material >= 0 && size_t(selected_material) < SA::builtin_materials().size()) {
        const SA::Material &builtin = SA::builtin_materials()[size_t(selected_material)];
        if (!material_properties_match(m, builtin)) {
            m.key = "custom";
            m.name = builtin.name + " (custom)";
            m.provenance = "User-edited values based on a bundled material estimate; verify against a filament datasheet and printed coupons.";
            m_material_choice->SetSelection(int(SA::builtin_materials().size()));
        }
    }
    std::array<double, 5> scales{};
    for (size_t index = 0; index < scales.size(); ++index)
        numeric = read_number(m_calibration_fields[index], scales[index]) && numeric;
    m.calibration.modulus_xy_scale = scales[0];
    m.calibration.modulus_z_scale = scales[1];
    m.calibration.strength_xy_scale = scales[2];
    m.calibration.strength_z_scale = scales[3];
    m.calibration.shear_scale = scales[4];
    m.calibration.source = m_calibration_source->GetValue().utf8_string();
    setup.print_layer_axis = read_vector(m_layer_axis, setup.print_layer_axis);
    setup.gravity.enabled = m_gravity_enabled->GetValue();
    setup.gravity.acceleration_m_s2 = read_vector(m_gravity, setup.gravity.acceleration_m_s2);
    setup.infill.background_pattern = SA::InfillPattern(std::max(0, m_background_pattern->GetSelection()));
    setup.infill.dense_pattern = SA::InfillPattern(std::max(0, m_dense_pattern->GetSelection()));
    double value = 0.0;
    numeric = read_number(m_background_density, value) && numeric; setup.infill.background_density = value / 100.0;
    numeric = read_number(m_dense_density, value) && numeric; setup.infill.dense_density = value / 100.0;
    numeric = read_number(m_dense_threshold, value) && numeric; setup.infill.dense_stress_threshold = value / 100.0;
    numeric = read_number(m_minimum_safety_factor, setup.criteria.minimum_safety_factor) && numeric;
    numeric = read_number(m_maximum_displacement, setup.criteria.maximum_displacement_mm) && numeric;
    numeric = read_number(m_objective_weights[0], setup.criteria.mass_weight) && numeric;
    numeric = read_number(m_objective_weights[1], setup.criteria.stiffness_weight) && numeric;
    numeric = read_number(m_objective_weights[2], setup.criteria.support_weight) && numeric;
    numeric = read_number(m_objective_weights[3], setup.criteria.print_time_weight) && numeric;

    const std::vector<std::string> errors = SA::validate(m_session->mesh, setup);
    if (!numeric || !errors.empty()) {
        if (show_errors) {
            wxString message = numeric ? wxString() : _L("One or more numeric fields are invalid.\n");
            for (const std::string &error : errors)
                message += wxString::FromUTF8(error) + "\n";
            wxMessageBox(message, _L("Strength setup is incomplete"), wxOK | wxICON_WARNING, this);
        }
        return false;
    }
    mark_stale();
    return true;
}

void StrengthLoadPanel::persist_setup(bool take_snapshot)
{
    const int index = m_session->object_index;
    if (index < 0 || size_t(index) >= m_plater->model().objects.size())
        return;
    ModelObject *object = m_plater->model().objects[size_t(index)];
    const std::string serialized = SA::serialize_setup_for_config(m_session->setup);
    if (const auto *existing = object->config.get().option<ConfigOptionString>("strength_analysis_setup")) {
        if (existing->value == serialized)
            return;
    }
    if (take_snapshot)
        m_plater->take_snapshot("Save strength analysis setup");
    object->config.set_key_value("strength_analysis_setup", new ConfigOptionString(serialized));
    m_plater->set_plater_dirty(true);
}

void StrengthLoadPanel::mark_stale()
{
    m_session->stale = true;
    ++m_session->revision;
    m_dense_button->Disable();
    m_orientation_button->Disable();
    m_settings_button->Disable();
    if (m_session->result.status != SA::AnalysisStatus::NotRun)
        m_status_label->SetLabel(_L("Inputs changed; displayed simulation results are stale until rerun."));
    if (m_result_callback)
        m_result_callback();
}

void StrengthLoadPanel::refresh_load_list()
{
    m_load_list->Clear();
    for (const SA::Load &load : m_session->setup.loads)
        m_load_list->Append(wxString::Format("%s — %s%s", wxString::FromUTF8(load.name),
            wxString::FromUTF8(SA::to_string(load.type)), load.active ? "" : " (disabled)"));
}

void StrengthLoadPanel::save_current_load_editor()
{
    if (m_current_load < 0 || size_t(m_current_load) >= m_session->setup.loads.size())
        return;
    SA::Load &load = m_session->setup.loads[size_t(m_current_load)];
    load.name = m_load_name->GetValue().utf8_string();
    load.type = SA::LoadType(std::max(0, m_load_type->GetSelection()));
    load.active = m_load_active->GetValue();
    load.region.whole_model = m_load_whole_model->GetValue() || load.type == SA::LoadType::GlobalForce;
    load.region.center_mm = read_vector(m_load_center, load.region.center_mm);
    read_number(m_load_radius, load.region.radius_mm);
    load.direction = read_vector(m_load_direction, load.direction);
    read_number(m_load_magnitude, load.magnitude_n);
    read_number(m_load_impact, load.impact_factor);
    read_number(m_load_safety_factor, load.target_safety_factor);
    load.strength_basis = SA::StrengthBasis(std::max(0, m_strength_basis->GetSelection()));
}

void StrengthLoadPanel::load_current_load_editor(int index)
{
    m_current_load = index;
    const bool enabled = index >= 0 && size_t(index) < m_session->setup.loads.size();
    const std::array<wxWindow *, 9> load_controls{m_load_name, m_load_type, m_load_active, m_load_whole_model,
        m_load_radius, m_load_magnitude, m_load_impact, m_load_safety_factor, m_strength_basis};
    for (wxWindow *control : load_controls)
        control->Enable(enabled);
    for (int axis = 0; axis < 3; ++axis) {
        m_load_center[axis]->Enable(enabled);
        m_load_direction[axis]->Enable(enabled);
    }
    if (!enabled)
        return;
    const SA::Load &load = m_session->setup.loads[size_t(index)];
    m_load_list->SetSelection(index);
    m_load_name->ChangeValue(wxString::FromUTF8(load.name));
    m_load_type->SetSelection(int(load.type));
    m_load_active->SetValue(load.active);
    m_load_whole_model->SetValue(load.region.whole_model);
    for (int axis = 0; axis < 3; ++axis) {
        m_load_center[axis]->ChangeValue(number(load.region.center_mm[axis]));
        m_load_direction[axis]->ChangeValue(number(load.direction[axis]));
    }
    m_load_radius->ChangeValue(number(load.region.radius_mm));
    m_load_magnitude->ChangeValue(number(load.magnitude_n));
    m_load_impact->ChangeValue(number(load.impact_factor));
    m_load_safety_factor->ChangeValue(number(load.target_safety_factor));
    m_strength_basis->SetSelection(int(load.strength_basis));
}

void StrengthLoadPanel::refresh_preserve_list()
{
    m_preserve_list->Clear();
    for (const SA::SphericalRegion &region : m_session->setup.preserve_regions)
        m_preserve_list->Append(wxString::Format(_L("Center %s, radius %.4g mm"), vector_text(region.center_mm), region.radius_mm));
}

void StrengthLoadPanel::run_analysis()
{
    load_selected_object();
    if (!collect_setup(true))
        return;
    persist_setup();
    if (m_worker.joinable())
        m_worker.join();
    const SA::Setup setup = m_session->setup;
    const indexed_triangle_set mesh = m_session->mesh;
    const uint64_t revision = m_session->revision;
    m_cancel = false;
    m_run_button->Disable();
    m_cancel_button->Enable();
    m_status_label->SetLabel(_L("Running offline linear-static engineering estimate…"));
    m_worker = std::thread([this, setup, mesh, revision] {
        SA::Result result = SA::analyze(mesh, setup, [this] { return m_cancel.load(); });
        {
            std::lock_guard<std::mutex> lock(m_pending_mutex);
            m_pending_result = std::move(result);
            m_pending_revision = revision;
        }
        wxQueueEvent(this, new wxThreadEvent(EVT_STRENGTH_ANALYSIS_FINISHED));
    });
}

void StrengthLoadPanel::cancel_analysis()
{
    m_cancel = true;
    m_status_label->SetLabel(_L("Cancelling analysis…"));
}

void StrengthLoadPanel::on_analysis_finished()
{
    if (m_worker.joinable())
        m_worker.join();
    SA::Result result;
    uint64_t revision = 0;
    {
        std::lock_guard<std::mutex> lock(m_pending_mutex);
        result = std::move(m_pending_result);
        revision = m_pending_revision;
    }
    m_run_button->Enable();
    m_cancel_button->Disable();
    if (revision != m_session->revision) {
        m_status_label->SetLabel(_L("Analysis finished, but inputs changed while it was running; results were discarded."));
        return;
    }
    m_session->result = std::move(result);
    m_session->stale = false;
    const SA::Result &stored = m_session->result;
    m_status_label->SetLabel(wxString::Format("%s — %s", wxString::FromUTF8(SA::to_string(stored.status)),
                                              wxString::FromUTF8(stored.message)));
    const bool ready = stored.succeeded();
    m_dense_button->Enable(ready && stored.dense_region.available);
    m_orientation_button->Enable(ready && !stored.orientation_recommendations.empty());
    m_settings_button->Enable(ready && !stored.print_settings_candidates.empty() &&
                              stored.print_settings_candidates.front().feasible);
    if (m_result_callback)
        m_result_callback();
}

void StrengthLoadPanel::create_dense_modifier()
{
    if (m_session->stale || !m_session->result.succeeded() || !m_session->result.dense_region.available)
        return;
    const int index = m_session->object_index;
    if (index < 0 || size_t(index) >= m_plater->model().objects.size())
        return;
    const SA::DenseRegionRecommendation &recommendation = m_session->result.dense_region;
    m_plater->take_snapshot("Create strength dense-region modifier");
    ModelObject *object = m_plater->model().objects[size_t(index)];
    TriangleMesh sphere(its_make_sphere(recommendation.region.radius_mm, PI / 18.0));
    ModelVolume *volume = object->add_volume(std::move(sphere), ModelVolumeType::PARAMETER_MODIFIER);
    volume->name = "Strength dense region";
    volume->set_offset(recommendation.region.center_mm);
    volume->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(recommendation.recommended_density * 100.0));
    volume->config.set_key_value("sparse_infill_pattern",
        new ConfigOptionEnum<Slic3r::InfillPattern>(print_pattern(recommendation.recommended_pattern)));
    m_plater->changed_object(index);
    m_plater->set_plater_dirty(true);
    m_dense_button->Disable();
    m_status_label->SetLabel(_L("Created a native parameter modifier for the highest-stress region."));
}

void StrengthLoadPanel::apply_recommended_orientation()
{
    if (m_session->stale || m_session->result.orientation_recommendations.empty())
        return;
    const int index = m_session->object_index;
    if (index < 0 || size_t(index) >= m_plater->model().objects.size())
        return;
    const Vec3d layer_axis = m_session->result.orientation_recommendations.front().layer_axis;
    Vec3d rotation_axis;
    double angle = 0.0;
    Matrix3d rotation;
    Geometry::rotation_from_two_vectors(layer_axis, Vec3d::UnitZ(), rotation_axis, angle, &rotation);
    m_plater->take_snapshot("Apply strength-optimized orientation");
    ModelObject *object = m_plater->model().objects[size_t(index)];
    for (ModelInstance *instance : object->instances)
        instance->rotate(rotation);
    object->invalidate_bounding_box();
    m_session->setup.print_layer_axis = layer_axis;
    populate_from_setup();
    persist_setup(false);
    m_plater->changed_object(index);
    mark_stale();
    m_status_label->SetLabel(_L("Applied the best estimated print orientation. Rerun to validate the rotated part."));
}

void StrengthLoadPanel::apply_optimized_settings()
{
    if (m_session->stale || m_session->result.print_settings_candidates.empty() ||
        !m_session->result.print_settings_candidates.front().feasible)
        return;
    const int index = m_session->object_index;
    if (index < 0 || size_t(index) >= m_plater->model().objects.size())
        return;
    const SA::PrintSettingsCandidate &candidate = m_session->result.print_settings_candidates.front();
    m_plater->take_snapshot("Apply strength-optimized print settings");
    ModelObject *object = m_plater->model().objects[size_t(index)];
    object->config.set_key_value("wall_loops", new ConfigOptionInt(candidate.wall_loops));
    object->config.set_key_value("layer_height", new ConfigOptionFloat(candidate.layer_height_mm));
    object->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(candidate.infill_density * 100.0));
    object->config.set_key_value("sparse_infill_pattern",
        new ConfigOptionEnum<Slic3r::InfillPattern>(print_pattern(candidate.pattern)));
    m_session->setup.infill.background_density = candidate.infill_density;
    m_session->setup.infill.background_pattern = candidate.pattern;
    populate_from_setup();
    persist_setup(false);
    m_plater->changed_object(index);
    mark_stale();
    m_status_label->SetLabel(wxString::Format(_L("Applied %d walls, %.3g mm layers, %.3g%% %s infill. Rerun to validate."),
        candidate.wall_loops, candidate.layer_height_mm, candidate.infill_density * 100.0,
        wxString::FromUTF8(SA::to_string(candidate.pattern))));
}

class StrengthSimulationPanel::ResultCanvas final : public wxPanel
{
public:
    ResultCanvas(wxWindow *parent, std::shared_ptr<StrengthAnalysisSession> session, std::function<void(size_t)> probe)
        : wxPanel(parent, wxID_ANY, wxDefaultPosition, parent->FromDIP(wxSize(720, 520)), wxBORDER_SIMPLE)
        , m_session(std::move(session)), m_probe(std::move(probe))
    {
        SetBackgroundStyle(wxBG_STYLE_PAINT);
        Bind(wxEVT_PAINT, [this](wxPaintEvent &) { paint(); });
        Bind(wxEVT_LEFT_DOWN, [this](wxMouseEvent &event) { click(event.GetPosition()); });
    }

    void set_mode(int mode) { m_mode = mode; Refresh(); }
    void set_projection(int projection) { m_projection = projection; Refresh(); }
    void set_deformation_scale(double scale) { m_deformation_scale = std::max(0.0, scale); Refresh(); }

private:
    std::shared_ptr<StrengthAnalysisSession> m_session;
    std::function<void(size_t)> m_probe;
    int m_mode{0};
    int m_projection{0};
    double m_deformation_scale{1.0};
    std::vector<wxPoint> m_projected;

    double scalar(const SA::VertexResult &vertex) const
    {
        switch (m_mode) {
        case 0: return std::isfinite(vertex.safety_factor) ? vertex.safety_factor : 10.0;
        case 1: return vertex.displacement_m.norm() * 1000.0;
        case 2: return vertex.von_mises_pa / 1e6;
        case 3: return vertex.maximum_shear_pa / 1e6;
        case 4: return vertex.normal_stress_pa.x() / 1e6;
        case 5: return vertex.normal_stress_pa.y() / 1e6;
        case 6: return vertex.normal_stress_pa.z() / 1e6;
        case 7: return vertex.shear_stress_pa.x() / 1e6;
        case 8: return vertex.shear_stress_pa.y() / 1e6;
        case 9: return vertex.shear_stress_pa.z() / 1e6;
        default: return 0.0;
        }
    }

    std::pair<double, double> project(const Vec3d &value) const
    {
        if (m_projection == 1) return {value.x(), value.z()};
        if (m_projection == 2) return {value.y(), value.z()};
        return {value.x(), value.y()};
    }

    static wxColour heat(double fraction)
    {
        fraction = std::clamp(fraction, 0.0, 1.0);
        const double r = std::clamp(1.5 - std::abs(4.0 * fraction - 3.0), 0.0, 1.0);
        const double g = std::clamp(1.5 - std::abs(4.0 * fraction - 2.0), 0.0, 1.0);
        const double b = std::clamp(1.5 - std::abs(4.0 * fraction - 1.0), 0.0, 1.0);
        return wxColour(int(255.0 * r), int(255.0 * g), int(255.0 * b));
    }

    void paint()
    {
        wxAutoBufferedPaintDC dc(this);
        dc.SetBackground(wxBrush(wxColour(250, 250, 250)));
        dc.Clear();
        const SA::Result &result = m_session->result;
        if (!result.succeeded() || result.vertices.empty() || m_session->mesh.indices.empty()) {
            dc.DrawText(_L("Run an analysis from the Load tab to display coupled results."), FromDIP(20), FromDIP(20));
            return;
        }

        std::vector<std::pair<double, double>> coordinates;
        coordinates.reserve(result.vertices.size());
        double min_u = std::numeric_limits<double>::infinity(), max_u = -min_u;
        double min_v = min_u, max_v = -min_u;
        double min_value = std::numeric_limits<double>::infinity(), max_value = -min_value;
        for (const SA::VertexResult &vertex : result.vertices) {
            const Vec3d deformed = vertex.position_mm + vertex.displacement_m * (1000.0 * m_deformation_scale);
            const auto projected = project(deformed);
            coordinates.push_back(projected);
            min_u = std::min(min_u, projected.first); max_u = std::max(max_u, projected.first);
            min_v = std::min(min_v, projected.second); max_v = std::max(max_v, projected.second);
            const double value = scalar(vertex);
            if (std::isfinite(value)) { min_value = std::min(min_value, value); max_value = std::max(max_value, value); }
        }
        if (m_mode == 0) max_value = std::min(max_value, 10.0);
        const wxSize size = GetClientSize();
        const int margin = FromDIP(42);
        const double width = std::max(max_u - min_u, 1e-9), height = std::max(max_v - min_v, 1e-9);
        const double scale = std::min(double(size.x - 2 * margin) / width, double(size.y - 2 * margin) / height);
        const double offset_x = 0.5 * (size.x - scale * width);
        const double offset_y = 0.5 * (size.y + scale * height);
        m_projected.resize(coordinates.size());
        for (size_t index = 0; index < coordinates.size(); ++index)
            m_projected[index] = wxPoint(int(offset_x + (coordinates[index].first - min_u) * scale),
                                         int(offset_y - (coordinates[index].second - min_v) * scale));

        std::vector<size_t> faces(m_session->mesh.indices.size());
        std::iota(faces.begin(), faces.end(), size_t(0));
        std::stable_sort(faces.begin(), faces.end(), [this, &result](size_t lhs, size_t rhs) {
            auto depth = [this, &result](size_t face) {
                const Vec3i32 &triangle = m_session->mesh.indices[face];
                double sum = 0.0;
                for (int corner = 0; corner < 3; ++corner) {
                    const Vec3d &position = result.vertices[size_t(triangle[corner])].position_mm;
                    sum += m_projection == 0 ? position.z() : (m_projection == 1 ? position.y() : position.x());
                }
                return sum;
            };
            return depth(lhs) < depth(rhs);
        });
        const double range = std::max(max_value - min_value, 1e-12);
        dc.SetPen(wxPen(wxColour(80, 80, 80), 1));
        for (size_t face : faces) {
            const Vec3i32 &triangle = m_session->mesh.indices[face];
            wxPoint points[3];
            double value = 0.0;
            bool valid = true;
            for (int corner = 0; corner < 3; ++corner) {
                const size_t vertex = size_t(triangle[corner]);
                if (vertex >= m_projected.size()) { valid = false; break; }
                points[corner] = m_projected[vertex];
                value += scalar(result.vertices[vertex]);
            }
            if (!valid) continue;
            value /= 3.0;
            double fraction = (std::clamp(value, min_value, max_value) - min_value) / range;
            if (m_mode == 0)
                fraction = 1.0 - fraction;
            dc.SetBrush(wxBrush(heat(fraction)));
            dc.DrawPolygon(3, points);
        }
        dc.SetTextForeground(*wxBLACK);
        dc.DrawText(wxString::Format(_L("Range: %.5g to %.5g"), min_value, max_value), FromDIP(8), FromDIP(8));
        if (m_session->stale) {
            dc.SetTextForeground(wxColour(180, 70, 0));
            dc.DrawText(_L("STALE — rerun after input or geometry changes"), FromDIP(8), FromDIP(27));
        }
    }

    void click(const wxPoint &point)
    {
        if (m_projected.empty()) return;
        size_t closest = 0;
        long closest_distance = std::numeric_limits<long>::max();
        for (size_t index = 0; index < m_projected.size(); ++index) {
            const long dx = point.x - m_projected[index].x;
            const long dy = point.y - m_projected[index].y;
            const long distance = dx * dx + dy * dy;
            if (distance < closest_distance) { closest_distance = distance; closest = index; }
        }
        if (closest_distance <= long(FromDIP(20) * FromDIP(20)))
            m_probe(closest);
    }
};

StrengthSimulationPanel::StrengthSimulationPanel(wxWindow *parent, std::shared_ptr<StrengthAnalysisSession> session)
    : wxScrolledWindow(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL | wxTAB_TRAVERSAL)
    , m_session(std::move(session))
{
    SetBackgroundColour(*wxWHITE);
    SetScrollRate(0, FromDIP(12));
    const int gap = FromDIP(8);
    auto *root = new wxBoxSizer(wxVERTICAL);
    root->Add(new wxStaticText(this, wxID_ANY, _L("Strength Analysis — Simulation Results")), 0, wxALL, gap);
    m_status = new wxStaticText(this, wxID_ANY, _L("No analysis has been run."));
    root->Add(m_status, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    auto *controls = new wxBoxSizer(wxHORIZONTAL);
    m_result_mode = new wxChoice(this, wxID_ANY, wxDefaultPosition, wxDefaultSize,
        {_L("Safety factor"), _L("Deformation (mm)"), _L("Von Mises stress (MPa)"), _L("Maximum shear (MPa)"),
         _L("Normal stress X (MPa)"), _L("Normal stress Y (MPa)"), _L("Normal stress Z (MPa)"),
         _L("Shear stress XY (MPa)"), _L("Shear stress XZ (MPa)"), _L("Shear stress YZ (MPa)")});
    m_result_mode->SetSelection(0);
    m_projection = new wxChoice(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, {_L("XY view"), _L("XZ view"), _L("YZ view")});
    m_projection->SetSelection(0);
    m_deformation_scale = number_input(this, 1.0, 78);
    add_labeled(controls, this, _L("Result"), m_result_mode);
    controls->AddSpacer(gap);
    add_labeled(controls, this, _L("Projection"), m_projection);
    controls->AddSpacer(gap);
    add_labeled(controls, this, _L("Deformation scale"), m_deformation_scale);
    root->Add(controls, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    m_canvas = new ResultCanvas(this, m_session, [this](size_t vertex) { update_probe(vertex); });
    root->Add(m_canvas, 1, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    m_probe = new wxStaticText(this, wxID_ANY, _L("Point probe: click near a mesh vertex."));
    root->Add(m_probe, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    m_summary = new wxTextCtrl(this, wxID_ANY, {}, wxDefaultPosition, FromDIP(wxSize(-1, 260)),
                               wxTE_MULTILINE | wxTE_READONLY | wxTE_DONTWRAP);
    root->Add(m_summary, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    SetSizer(root);

    m_result_mode->Bind(wxEVT_CHOICE, [this](wxCommandEvent &) { m_canvas->set_mode(m_result_mode->GetSelection()); });
    m_projection->Bind(wxEVT_CHOICE, [this](wxCommandEvent &) { m_canvas->set_projection(m_projection->GetSelection()); });
    m_deformation_scale->Bind(wxEVT_TEXT, [this](wxCommandEvent &) {
        double value = 1.0;
        if (read_number(m_deformation_scale, value)) m_canvas->set_deformation_scale(value);
    });
}

void StrengthSimulationPanel::activate()
{
    refresh();
}

void StrengthSimulationPanel::refresh()
{
    const SA::Result &result = m_session->result;
    if (!result.succeeded()) {
        m_status->SetLabel(result.status == SA::AnalysisStatus::NotRun ? _L("No analysis has been run.") :
            wxString::Format("%s — %s", wxString::FromUTF8(SA::to_string(result.status)), wxString::FromUTF8(result.message)));
        m_summary->ChangeValue({});
        m_canvas->Refresh();
        return;
    }
    m_status->SetLabel(m_session->stale ? _L("Results are stale because setup or geometry changed.") :
        _L("Current offline engineering estimate. Validate critical parts with physical tests."));
    wxString text;
    text += wxString::Format(_L("Estimated mass: %.6g kg\n"), result.estimated_mass_kg);
    text += wxString::Format(_L("Maximum deformation: %.6g mm\n"), result.maximum_displacement_m * 1000.0);
    text += wxString::Format(_L("Maximum Von Mises stress: %.6g MPa\n"), result.maximum_von_mises_pa / 1e6);
    text += wxString::Format(_L("Minimum safety factor: %.6g\n"), result.minimum_safety_factor);
    text += wxString::Format(_L("Governing safety-factor target: %.6g — %s\n"), result.governing_target_safety_factor,
        result.safety_factor_target_met ? _L("met") : _L("not met"));
    if (m_session->setup.criteria.maximum_displacement_mm > 0.0)
        text += wxString::Format(_L("Maximum-displacement target: %.6g mm — %s\n"),
            m_session->setup.criteria.maximum_displacement_mm, result.displacement_target_met ? _L("met") : _L("not met"));
    text += wxString::Format(_L("Requested resultant force: %s N\n\n"), vector_text(result.requested_resultant_force_n));
    if (result.dense_region.available)
        text += wxString::Format(_L("Dense-region recommendation: center %s mm, radius %.5g mm, %.4g%% %s "
                                     "(vertices at or above %.4g%% of peak stress)\n"),
            vector_text(result.dense_region.region.center_mm), result.dense_region.region.radius_mm,
            result.dense_region.recommended_density * 100.0, wxString::FromUTF8(SA::to_string(result.dense_region.recommended_pattern)),
            result.dense_region.stress_fraction * 100.0);
    if (!result.orientation_recommendations.empty()) {
        const auto &item = result.orientation_recommendations.front();
        text += wxString::Format(_L("Best estimated layer axis: %s, predicted safety factor %.5g, support score %.5g\n"),
            vector_text(item.layer_axis), item.predicted_safety_factor, item.support_score);
    }
    if (!result.print_settings_candidates.empty()) {
        const auto &item = result.print_settings_candidates.front();
        text += wxString::Format(_L("Best settings candidate: %d walls, %.4g mm layers, %.4g%% %s; predicted SF %.5g; "
                                     "predicted deformation %.5g mm; mass %.5g kg%s\n"),
            item.wall_loops, item.layer_height_mm, item.infill_density * 100.0, wxString::FromUTF8(SA::to_string(item.pattern)),
            item.predicted_safety_factor, item.predicted_displacement_mm, item.estimated_mass_kg,
            item.feasible ? "" : " (criteria not met)");
    }
    text += _L("\nInfill pattern comparison:\n");
    for (const auto &item : result.infill_comparisons)
        text += wxString::Format("  %s %.4g%% — SF %.5g, mass %.5g kg, relative stiffness %.4g, "
                                 "relative directional strength %.4g\n",
            wxString::FromUTF8(SA::to_string(item.pattern)), item.density * 100.0, item.estimated_safety_factor,
            item.estimated_mass_kg, item.relative_stiffness, item.relative_directional_strength);
    text += _L("\nEstimated mass at target safety factors:\n");
    for (const auto &item : result.mass_strength_curve)
        text += wxString::Format("  SF %.4g — %s, mass %.5g kg, %.4g%% %s\n", item.target_safety_factor,
            item.feasible ? _L("feasible") : _L("not feasible"), item.estimated_mass_kg, item.density * 100.0,
            wxString::FromUTF8(SA::to_string(item.pattern)));
    text += _L("\nModel limitations and warnings:\n");
    for (const std::string &warning : result.warnings)
        text += "  • " + wxString::FromUTF8(warning) + "\n";
    m_summary->ChangeValue(text);
    m_canvas->Refresh();
}

void StrengthSimulationPanel::update_probe(size_t vertex_index)
{
    if (vertex_index >= m_session->result.vertices.size()) return;
    const SA::VertexResult &value = m_session->result.vertices[vertex_index];
    m_probe->SetLabel(wxString::Format(
        _L("Point probe #%zu — position %s mm; displacement %s mm (|u| %.6g); normal stress %s MPa; "
           "shear XY/XZ/YZ %s MPa; Von Mises %.6g MPa; maximum shear %.6g MPa; safety factor %.6g"),
        vertex_index, vector_text(value.position_mm), vector_text(value.displacement_m * 1000.0), value.displacement_m.norm() * 1000.0,
        vector_text(value.normal_stress_pa / 1e6), vector_text(value.shear_stress_pa / 1e6), value.von_mises_pa / 1e6,
        value.maximum_shear_pa / 1e6, value.safety_factor));
    Layout();
}

} // namespace Slic3r::GUI
