#ifndef slic3r_GUI_StrengthAnalysisPanel_hpp_
#define slic3r_GUI_StrengthAnalysisPanel_hpp_

#include "libslic3r/StrengthAnalysis.hpp"

#include <wx/scrolwin.h>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

class wxButton;
class wxCheckBox;
class wxChoice;
class wxListBox;
class wxPanel;
class wxStaticText;
class wxTextCtrl;
class wxTreeCtrl;

namespace Slic3r::GUI {

class Plater;

struct StrengthAnalysisSession
{
    StrengthAnalysis::Setup setup;
    StrengthAnalysis::Result result;
    indexed_triangle_set mesh;
    indexed_triangle_set solved_mesh;
    StrengthAnalysis::Setup solved_setup;
    int object_index{-1};
    bool stale{true};
    uint64_t revision{0};
};

class StrengthLoadPanel final : public wxScrolledWindow
{
public:
    StrengthLoadPanel(wxWindow *parent, Plater *plater, std::shared_ptr<StrengthAnalysisSession> session);
    ~StrengthLoadPanel() override;

    void activate();
    void set_result_callback(std::function<void()> callback) { m_result_callback = std::move(callback); }

private:
    class SetupCanvas;

    Plater *m_plater;
    std::shared_ptr<StrengthAnalysisSession> m_session;
    std::function<void()> m_result_callback;
    std::thread m_worker;
    std::atomic_bool m_cancel{false};
    std::mutex m_pending_mutex;
    StrengthAnalysis::Result m_pending_result;
    uint64_t m_pending_revision{0};
    int m_current_load{-1};
    int m_selected_kind{0};
    int m_selected_index{-1};
    SetupCanvas *m_setup_canvas{nullptr};
    wxTreeCtrl *m_study_tree{nullptr};
    wxPanel *m_operation_panel{nullptr};
    wxStaticText *m_operation_title{nullptr};
    wxTextCtrl *m_operation_name{nullptr};
    wxChoice *m_operation_shape{nullptr};
    wxTextCtrl *m_operation_center[3]{};
    wxTextCtrl *m_operation_size[3]{};
    wxTextCtrl *m_operation_radius{nullptr};
    wxTextCtrl *m_operation_axis[3]{};
    wxTextCtrl *m_operation_direction[3]{};
    wxTextCtrl *m_operation_magnitude{nullptr};
    wxButton *m_operation_apply{nullptr};
    wxButton *m_operation_popout{nullptr};

    wxStaticText *m_object_label{nullptr};
    wxStaticText *m_status_label{nullptr};
    wxChoice *m_material_choice{nullptr};
    std::vector<wxTextCtrl *> m_material_fields;
    std::vector<wxTextCtrl *> m_calibration_fields;
    wxTextCtrl *m_calibration_source{nullptr};
    wxTextCtrl *m_layer_axis[3]{};

    wxListBox *m_load_list{nullptr};
    wxTextCtrl *m_load_name{nullptr};
    wxChoice *m_load_type{nullptr};
    wxCheckBox *m_load_active{nullptr};
    wxCheckBox *m_load_whole_model{nullptr};
    wxTextCtrl *m_load_center[3]{};
    wxTextCtrl *m_load_radius{nullptr};
    wxTextCtrl *m_load_direction[3]{};
    wxTextCtrl *m_load_magnitude{nullptr};
    wxTextCtrl *m_load_impact{nullptr};
    wxTextCtrl *m_load_safety_factor{nullptr};
    wxChoice *m_strength_basis{nullptr};

    wxCheckBox *m_gravity_enabled{nullptr};
    wxTextCtrl *m_gravity[3]{};
    wxChoice *m_background_pattern{nullptr};
    wxChoice *m_dense_pattern{nullptr};
    wxTextCtrl *m_background_density{nullptr};
    wxTextCtrl *m_dense_density{nullptr};
    wxTextCtrl *m_dense_threshold{nullptr};

    wxListBox *m_preserve_list{nullptr};
    wxTextCtrl *m_preserve_center[3]{};
    wxTextCtrl *m_preserve_radius{nullptr};

    wxTextCtrl *m_minimum_safety_factor{nullptr};
    wxTextCtrl *m_maximum_displacement{nullptr};
    wxTextCtrl *m_objective_weights[4]{};
    wxButton *m_run_button{nullptr};
    wxButton *m_precheck_button{nullptr};
    wxButton *m_cancel_button{nullptr};
    wxButton *m_dense_button{nullptr};
    wxButton *m_orientation_button{nullptr};
    wxButton *m_settings_button{nullptr};

    void build_ui();
    void load_selected_object();
    void populate_material_fields();
    void populate_from_setup();
    bool collect_setup(bool show_errors);
    void persist_setup(bool take_snapshot = true);
    void mark_stale();
    void refresh_load_list();
    void save_current_load_editor();
    void load_current_load_editor(int index);
    void refresh_preserve_list();
    void refresh_study_tree();
    void select_study_tree_item(int kind, int index);
    void begin_place_load(StrengthAnalysis::LoadType type);
    void place_load_at(StrengthAnalysis::LoadType type, const Vec3d &position_mm, const Vec3d &normal,
                       const std::vector<size_t> &surface_triangles);
    void place_preserve_at(const Vec3d &position_mm, const std::vector<size_t> &surface_triangles);
    void refresh_operation_panel();
    void apply_operation_panel();
    bool edit_load_dialog(StrengthAnalysis::Load &load, bool creating);
    bool edit_preserve_dialog(StrengthAnalysis::SphericalRegion &region, bool creating);
    void edit_gravity_dialog();
    void edit_material_dialog();
    void edit_infill_dialog();
    void edit_criteria_dialog();
    void select_canvas_item(int kind, int index, bool edit);
    bool run_precheck(bool show_success);
    void run_analysis();
    void cancel_analysis();
    void on_analysis_finished();
    void create_dense_modifier();
    void apply_recommended_orientation();
    void apply_optimized_settings();
};

class StrengthSimulationPanel final : public wxScrolledWindow
{
public:
    StrengthSimulationPanel(wxWindow *parent, std::shared_ptr<StrengthAnalysisSession> session,
                            std::function<void()> synchronize_session = {});
    void activate();

private:
    class ResultCanvas;
    std::shared_ptr<StrengthAnalysisSession> m_session;
    std::function<void()> m_synchronize_session;
    wxStaticText *m_status{nullptr};
    wxChoice *m_result_mode{nullptr};
    wxChoice *m_projection{nullptr};
    wxTextCtrl *m_deformation_scale{nullptr};
    wxCheckBox *m_show_setup{nullptr};
    wxCheckBox *m_show_wireframe{nullptr};
    wxCheckBox *m_banded_contours{nullptr};
    wxStaticText *m_probe{nullptr};
    wxTextCtrl *m_summary{nullptr};
    ResultCanvas *m_canvas{nullptr};

    void refresh();
    void update_probe(size_t vertex_index);
};

} // namespace Slic3r::GUI

#endif
