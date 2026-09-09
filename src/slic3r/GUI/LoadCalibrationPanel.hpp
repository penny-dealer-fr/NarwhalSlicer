#ifndef slic3r_GUI_LoadCalibrationPanel_hpp_
#define slic3r_GUI_LoadCalibrationPanel_hpp_
#include <wx/scrolwin.h>
#include "libslic3r/StrengthAnalysis.hpp"
namespace Slic3r::GUI {
// Includes persisted user calibrations, followed by the same custom-material sentinel in the UI.
const std::vector<StrengthAnalysis::Material>& load_materials();
class LoadCalibrationPanel : public wxScrolledWindow
{
public:
    explicit LoadCalibrationPanel(wxWindow* parent);
    ~LoadCalibrationPanel() override;
    void rescale();
    void update_colors();

private:
    struct Impl;
    std::unique_ptr<Impl> m;
};
} // namespace Slic3r::GUI
#endif
