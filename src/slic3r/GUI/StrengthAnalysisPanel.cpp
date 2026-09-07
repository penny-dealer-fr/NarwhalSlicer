#include "StrengthAnalysisPanel.hpp"

#include "I18N.hpp"
#include "Plater.hpp"
#include "Selection.hpp"
#include "Widgets/Button.hpp"

#include "libslic3r/Geometry.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"

#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/choice.h>
#include <wx/dcbuffer.h>
#include <wx/dialog.h>
#include <wx/image.h>
#include <wx/listbox.h>
#include <wx/msgdlg.h>
#include <wx/settings.h>
#include <wx/sizer.h>
#include <wx/slider.h>
#include <wx/statbox.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/thread.h>
#include <wx/treectrl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>

namespace Slic3r::GUI {

wxDEFINE_EVENT(EVT_STRENGTH_ANALYSIS_FINISHED, wxThreadEvent);

namespace {

namespace SA = StrengthAnalysis;

constexpr const char *STRENGTH_MODIFIER_NAME = "Strength dense region";

bool is_strength_dense_modifier(const ModelVolume *volume)
{
    if (volume == nullptr || !volume->is_modifier())
        return false;
    const auto *marker = dynamic_cast<const ConfigOptionBool *>(
        volume->config.option("strength_analysis_modifier"));
    return marker != nullptr && marker->value;
}

wxString number(double value)
{
    return wxString::Format("%.8g", value);
}

bool read_number(wxTextCtrl *control, double &value)
{
    if (control == nullptr)
        return false;
    // Keep exact stored values when the user has not changed their rounded display text.
    if (std::isfinite(value) && control->GetValue() == number(value))
        return true;
    double parsed = 0.0;
    if (!control->GetValue().ToDouble(&parsed) || !std::isfinite(parsed))
        return false;
    value = parsed;
    return true;
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

wxBoxSizer *vector_editor(wxWindow *parent, wxTextCtrl *controls[3], const Vec3d &value,
                          const wxString &unit = {}, int width = 76)
{
    auto *row = new wxBoxSizer(wxHORIZONTAL);
    static const std::array<wxString, 3> axes{"X", "Y", "Z"};
    for (int axis = 0; axis < 3; ++axis) {
        row->Add(new wxStaticText(parent, wxID_ANY, axes[axis]), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, parent->FromDIP(3));
        controls[axis] = number_input(parent, value[axis], width);
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

bool read_vector_strict(wxTextCtrl *const controls[3], Vec3d &output)
{
    bool valid = true;
    Vec3d parsed = output;
    for (int axis = 0; axis < 3; ++axis) {
        valid = read_number(controls[axis], parsed[axis]) && valid;
    }
    if (valid)
        output = parsed;
    return valid;
}

wxArrayString load_type_names()
{
    return {_L("Fixed region"), _L("Local force"), _L("Directional force"), _L("Bearing force"),
            _L("Impact force"), _L("Global force")};
}

wxArrayString region_shape_names()
{
    return {_L("Sphere"), _L("Box"), _L("Cylinder"), _L("Selected face")};
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

wxColour load_colour(SA::LoadType type)
{
    switch (type) {
    case SA::LoadType::Fixed: return wxColour(52, 111, 205);
    case SA::LoadType::BearingForce: return wxColour(173, 66, 184);
    case SA::LoadType::ImpactForce: return wxColour(231, 128, 35);
    case SA::LoadType::GlobalForce: return wxColour(200, 45, 94);
    default: return wxColour(214, 61, 55);
    }
}

wxColour blend_colour(const wxColour &base, const wxColour &overlay, double amount)
{
    amount = std::clamp(amount, 0.0, 1.0);
    const auto channel = [amount](unsigned char lhs, unsigned char rhs) {
        return static_cast<unsigned char>(std::lround((1.0 - amount) * lhs + amount * rhs));
    };
    return wxColour(channel(base.Red(), overlay.Red()), channel(base.Green(), overlay.Green()),
                    channel(base.Blue(), overlay.Blue()));
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

bool valid_triangle(const Vec3i32 &triangle, size_t vertex_count)
{
    return triangle[0] >= 0 && triangle[1] >= 0 && triangle[2] >= 0 &&
           size_t(triangle[0]) < vertex_count && size_t(triangle[1]) < vertex_count &&
           size_t(triangle[2]) < vertex_count;
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

struct CameraFrame
{
    Vec3d center{Vec3d::Zero()};
    Vec3d right{Vec3d::UnitX()};
    Vec3d up{Vec3d::UnitZ()};
    Vec3d forward{Vec3d::UnitY()};
    double scale{1.0};
    wxPoint2DDouble origin;
};

struct ScreenVertex
{
    wxPoint point;
    double depth{0.0};
};

// A small CPU rasterizer keeps this self-contained wxWidgets viewport depth-correct. Painter's
// sorting cannot correctly display intersecting or mutually overlapping triangles; the per-pixel
// depth buffer below can, and also permits smooth barycentric result contours without OpenGL state.
class DepthBitmap
{
public:
    explicit DepthBitmap(const wxSize &size)
        : m_width(std::max(1, size.x)), m_height(std::max(1, size.y)),
          m_depth(size_t(m_width) * size_t(m_height), -std::numeric_limits<double>::infinity()),
          m_rgb(size_t(m_width) * size_t(m_height) * 3, 0),
          m_alpha(size_t(m_width) * size_t(m_height), 0)
    {}

    template<class ColourFunction>
    void triangle(const ScreenVertex &a, const ScreenVertex &b, const ScreenVertex &c, ColourFunction colour)
    {
        const double area = edge(double(a.point.x), double(a.point.y), double(b.point.x), double(b.point.y),
                                 double(c.point.x), double(c.point.y));
        if (std::abs(area) < 1e-9)
            return;
        const int minimum_x = std::clamp(std::min({a.point.x, b.point.x, c.point.x}), 0, m_width - 1);
        const int maximum_x = std::clamp(std::max({a.point.x, b.point.x, c.point.x}), 0, m_width - 1);
        const int minimum_y = std::clamp(std::min({a.point.y, b.point.y, c.point.y}), 0, m_height - 1);
        const int maximum_y = std::clamp(std::max({a.point.y, b.point.y, c.point.y}), 0, m_height - 1);
        for (int y = minimum_y; y <= maximum_y; ++y) {
            for (int x = minimum_x; x <= maximum_x; ++x) {
                const double px = double(x) + 0.5, py = double(y) + 0.5;
                const double wa = edge(double(b.point.x), double(b.point.y), double(c.point.x), double(c.point.y), px, py) / area;
                const double wb = edge(double(c.point.x), double(c.point.y), double(a.point.x), double(a.point.y), px, py) / area;
                const double wc = 1.0 - wa - wb;
                constexpr double tolerance = -1e-8;
                if (wa < tolerance || wb < tolerance || wc < tolerance)
                    continue;
                const double depth = wa * a.depth + wb * b.depth + wc * c.depth;
                const size_t pixel = size_t(y) * size_t(m_width) + size_t(x);
                if (depth <= m_depth[pixel])
                    continue;
                m_depth[pixel] = depth;
                put(pixel, colour(wa, wb, wc));
            }
        }
    }

    void line(const ScreenVertex &a, const ScreenVertex &b, const wxColour &colour, int width = 1)
    {
        const int dx = b.point.x - a.point.x, dy = b.point.y - a.point.y;
        const int steps = std::max(std::abs(dx), std::abs(dy));
        if (steps == 0)
            return;
        const int radius = std::max(0, width / 2);
        for (int step = 0; step <= steps; ++step) {
            const double t = double(step) / double(steps);
            const int x = int(std::lround(double(a.point.x) + t * dx));
            const int y = int(std::lround(double(a.point.y) + t * dy));
            const double depth = (1.0 - t) * a.depth + t * b.depth;
            for (int oy = -radius; oy <= radius; ++oy)
                for (int ox = -radius; ox <= radius; ++ox) {
                    const int sample_x = x + ox, sample_y = y + oy;
                    if (sample_x < 0 || sample_y < 0 || sample_x >= m_width || sample_y >= m_height)
                        continue;
                    const size_t pixel = size_t(sample_y) * size_t(m_width) + size_t(sample_x);
                    const double tolerance = 1e-5 * std::max(1.0, std::abs(depth));
                    if (depth + tolerance >= m_depth[pixel])
                        put(pixel, colour);
                }
        }
    }

    void draw(wxDC &dc) const
    {
        wxImage image(m_width, m_height);
        std::copy(m_rgb.begin(), m_rgb.end(), image.GetData());
        image.InitAlpha();
        std::copy(m_alpha.begin(), m_alpha.end(), image.GetAlpha());
        dc.DrawBitmap(wxBitmap(image), 0, 0, true);
    }

private:
    int m_width;
    int m_height;
    std::vector<double> m_depth;
    std::vector<unsigned char> m_rgb;
    std::vector<unsigned char> m_alpha;

    static double edge(double ax, double ay, double bx, double by, double px, double py)
    {
        return (px - ax) * (by - ay) - (py - ay) * (bx - ax);
    }

    void put(size_t pixel, const wxColour &colour)
    {
        m_rgb[pixel * 3] = colour.Red();
        m_rgb[pixel * 3 + 1] = colour.Green();
        m_rgb[pixel * 3 + 2] = colour.Blue();
        m_alpha[pixel] = colour.Alpha();
    }
};

struct StudyTreeItemData final : public wxTreeItemData
{
    StudyTreeItemData(int item_kind, int item_index = -1) : kind(item_kind), index(item_index) {}
    int kind;
    int index;
};

class SoftwareViewport3D : public wxPanel
{
public:
    SoftwareViewport3D(wxWindow *parent, const wxSize &minimum_size)
        : wxPanel(parent, wxID_ANY, wxDefaultPosition, minimum_size, wxBORDER_SIMPLE)
    {
        SetMinSize(minimum_size);
        SetBackgroundStyle(wxBG_STYLE_PAINT);
        Bind(wxEVT_PAINT, [this](wxPaintEvent &) { paint(); });
        Bind(wxEVT_LEFT_DOWN, [this](wxMouseEvent &event) { begin_drag(event); });
        Bind(wxEVT_LEFT_UP, [this](wxMouseEvent &event) { end_drag(event); });
        Bind(wxEVT_LEFT_DCLICK, [this](wxMouseEvent &event) { clicked(event.GetPosition(), true); });
        Bind(wxEVT_MIDDLE_DOWN, [this](wxMouseEvent &event) { begin_drag(event); });
        Bind(wxEVT_MIDDLE_UP, [this](wxMouseEvent &event) { end_drag(event); });
        Bind(wxEVT_RIGHT_DOWN, [this](wxMouseEvent &event) { begin_drag(event); });
        Bind(wxEVT_RIGHT_UP, [this](wxMouseEvent &event) { end_drag(event); });
        Bind(wxEVT_MOTION, [this](wxMouseEvent &event) { drag(event); });
        Bind(wxEVT_MOUSEWHEEL, [this](wxMouseEvent &event) {
            const double steps = double(event.GetWheelRotation()) / std::max(1, event.GetWheelDelta());
            m_zoom = std::clamp(m_zoom * std::pow(1.12, steps), 0.08, 40.0);
            Refresh();
        });
        Bind(wxEVT_MOUSE_CAPTURE_LOST, [this](wxMouseCaptureLostEvent &) {
            const bool finish_interaction = m_custom_interaction;
            m_dragging = false;
            m_custom_interaction = false;
            if (finish_interaction)
                end_interaction();
        });
    }

    void fit_view()
    {
        m_zoom = 1.0;
        m_pan = wxPoint2DDouble(0.0, 0.0);
        Refresh();
    }

    void set_view(int view)
    {
        constexpr double pi = 3.14159265358979323846;
        switch (view) {
        case 1: m_yaw = -0.5 * pi; m_pitch = 0.0; break;       // Front: X/Z.
        case 2: m_yaw = -0.5 * pi; m_pitch = 0.5 * pi; break; // Top: X/Y.
        case 3: m_yaw = 0.0; m_pitch = 0.0; break;            // Right: Y/Z.
        default: m_yaw = -0.75; m_pitch = 0.48; break;
        }
        fit_view();
    }

protected:
    virtual std::vector<Vec3d> scene_points() const = 0;
    virtual void draw_scene(wxDC &dc, const CameraFrame &camera) = 0;
    virtual void clicked(const wxPoint &, bool) {}
    virtual void hovered(const wxPoint &) {}
    virtual bool begin_interaction(const wxPoint &) { return false; }
    virtual void update_interaction(const wxPoint &) {}
    virtual void end_interaction() {}

    void set_right_margin(int pixels) { m_right_margin = std::max(0, pixels); }

    CameraFrame camera() const
    {
        CameraFrame output;
        const std::vector<Vec3d> points = scene_points();
        if (!points.empty()) {
            Vec3d minimum = points.front();
            Vec3d maximum = points.front();
            for (const Vec3d &point : points) {
                minimum = minimum.cwiseMin(point);
                maximum = maximum.cwiseMax(point);
            }
            output.center = 0.5 * (minimum + maximum);
            double radius = 0.0;
            for (const Vec3d &point : points)
                radius = std::max(radius, (point - output.center).norm());
            const wxSize size = GetClientSize();
            const double available_width = std::max(80, size.x - m_right_margin);
            const double available_height = std::max(80, size.y);
            output.scale = 0.43 * std::min(available_width, available_height) / std::max(radius, 1e-6) * m_zoom;
            output.origin = wxPoint2DDouble(0.5 * available_width + m_pan.m_x, 0.5 * available_height + m_pan.m_y);
        }
        output.forward = Vec3d(std::cos(m_pitch) * std::cos(m_yaw),
                               std::cos(m_pitch) * std::sin(m_yaw), std::sin(m_pitch));
        output.right = Vec3d(-std::sin(m_yaw), std::cos(m_yaw), 0.0);
        output.up = output.forward.cross(output.right).normalized();
        return output;
    }

    ScreenVertex project(const Vec3d &point, const CameraFrame &camera) const
    {
        const Vec3d delta = point - camera.center;
        return {
            wxPoint(int(std::lround(camera.origin.m_x + delta.dot(camera.right) * camera.scale)),
                    int(std::lround(camera.origin.m_y - delta.dot(camera.up) * camera.scale))),
            delta.dot(camera.forward)
        };
    }

    static bool barycentric(const wxPoint &point, const wxPoint triangle[3], double weights[3])
    {
        const double denominator = double(triangle[1].y - triangle[2].y) * (triangle[0].x - triangle[2].x) +
            double(triangle[2].x - triangle[1].x) * (triangle[0].y - triangle[2].y);
        if (std::abs(denominator) < 1e-9)
            return false;
        weights[0] = (double(triangle[1].y - triangle[2].y) * (point.x - triangle[2].x) +
                      double(triangle[2].x - triangle[1].x) * (point.y - triangle[2].y)) / denominator;
        weights[1] = (double(triangle[2].y - triangle[0].y) * (point.x - triangle[2].x) +
                      double(triangle[0].x - triangle[2].x) * (point.y - triangle[2].y)) / denominator;
        weights[2] = 1.0 - weights[0] - weights[1];
        constexpr double epsilon = -1e-6;
        return weights[0] >= epsilon && weights[1] >= epsilon && weights[2] >= epsilon;
    }

    static void draw_arrow(wxDC &dc, const wxPoint &start, const wxPoint &end, const wxColour &colour, int width = 3)
    {
        dc.SetPen(wxPen(colour, width));
        const double dx = double(end.x - start.x), dy = double(end.y - start.y);
        const double length = std::hypot(dx, dy);
        if (length < 2.0) {
            // A vector parallel to the view direction has no projected shaft. Keep its direction
            // visibly selectable with the conventional target/dot symbol instead of hiding it.
            dc.SetBrush(*wxTRANSPARENT_BRUSH);
            dc.DrawCircle(start, 7 + width);
            dc.SetBrush(wxBrush(colour));
            dc.DrawCircle(start, 2 + width / 2);
            return;
        }
        dc.DrawLine(start, end);
        const double ux = dx / length, uy = dy / length;
        const double head = 11.0 + width;
        const double wing = 5.0 + width;
        wxPoint points[3]{
            end,
            wxPoint(int(std::lround(end.x - head * ux + wing * uy)), int(std::lround(end.y - head * uy - wing * ux))),
            wxPoint(int(std::lround(end.x - head * ux - wing * uy)), int(std::lround(end.y - head * uy + wing * ux)))
        };
        dc.SetBrush(wxBrush(colour));
        dc.DrawPolygon(3, points);
    }

    void draw_region_box(wxDC &dc, const CameraFrame &camera, const Vec3d &center, const Vec3d &half_extent,
                         const wxColour &colour, bool selected) const
    {
        const Vec3d extent = half_extent.cwiseMax(Vec3d::Constant(1e-6));
        std::array<ScreenVertex, 8> corners;
        for (int corner = 0; corner < 8; ++corner) {
            const Vec3d offset((corner & 1) ? extent.x() : -extent.x(),
                               (corner & 2) ? extent.y() : -extent.y(),
                               (corner & 4) ? extent.z() : -extent.z());
            corners[size_t(corner)] = project(center + offset, camera);
        }
        static constexpr int face_indices[6][4]{
            {0, 2, 6, 4}, {1, 5, 7, 3}, {0, 4, 5, 1},
            {2, 3, 7, 6}, {0, 1, 3, 2}, {4, 6, 7, 5}};
        std::array<int, 6> order{0, 1, 2, 3, 4, 5};
        std::stable_sort(order.begin(), order.end(), [&corners](int lhs, int rhs) {
            double left = 0.0, right = 0.0;
            for (int corner = 0; corner < 4; ++corner) {
                left += corners[size_t(face_indices[lhs][corner])].depth;
                right += corners[size_t(face_indices[rhs][corner])].depth;
            }
            return left < right;
        });
        dc.SetPen(*wxTRANSPARENT_PEN);
        dc.SetBrush(wxBrush(wxColour(colour.Red(), colour.Green(), colour.Blue(), selected ? 64 : 34)));
        for (int face : order) {
            wxPoint polygon[4];
            for (int corner = 0; corner < 4; ++corner)
                polygon[corner] = corners[size_t(face_indices[face][corner])].point;
            dc.DrawPolygon(4, polygon);
        }
        static constexpr int edges[12][2]{
            {0, 1}, {0, 2}, {0, 4}, {1, 3}, {1, 5}, {2, 3},
            {2, 6}, {3, 7}, {4, 5}, {4, 6}, {5, 7}, {6, 7}};
        dc.SetPen(wxPen(colour, selected ? 3 : 2, selected ? wxPENSTYLE_SOLID : wxPENSTYLE_SHORT_DASH));
        for (const auto &edge : edges)
            dc.DrawLine(corners[size_t(edge[0])].point, corners[size_t(edge[1])].point);
    }

    void draw_region_box(wxDC &dc, const CameraFrame &camera, const Vec3d &center, double half_extent,
                         const wxColour &colour, bool selected) const
    {
        draw_region_box(dc, camera, center, Vec3d::Constant(half_extent), colour, selected);
    }

    void draw_region_sphere(wxDC &dc, const CameraFrame &camera, const Vec3d &center, double radius,
                            const wxColour &colour, bool selected) const
    {
        const ScreenVertex projected_center = project(center, camera);
        const int screen_radius = std::clamp(int(std::lround(std::max(radius, 1e-6) * camera.scale)),
                                             FromDIP(4), FromDIP(240));
        // The circle is the exact orthographic silhouette of the spherical solver region. The
        // surrounding box is retained as a familiar Fusion-style editable extent/manipulator.
        dc.SetPen(wxPen(colour, selected ? 3 : 2, selected ? wxPENSTYLE_SOLID : wxPENSTYLE_SHORT_DASH));
        dc.SetBrush(wxBrush(blend_colour(wxColour(236, 241, 247), colour, selected ? 0.20 : 0.11),
                            wxBRUSHSTYLE_BDIAGONAL_HATCH));
        dc.DrawCircle(projected_center.point, screen_radius);
        dc.DrawLine(projected_center.point + wxPoint(-screen_radius, 0),
                    projected_center.point + wxPoint(screen_radius, 0));
        dc.DrawLine(projected_center.point + wxPoint(0, -screen_radius),
                    projected_center.point + wxPoint(0, screen_radius));
    }

    void draw_region_shape(wxDC &dc, const CameraFrame &camera, const SA::SphericalRegion &region,
                           const wxColour &colour, bool selected) const
    {
        switch (region.shape) {
        case SA::RegionShape::Sphere:
            draw_region_sphere(dc, camera, region.center_mm, region.radius_mm, colour, selected);
            draw_region_box(dc, camera, region.center_mm, region.radius_mm, colour, selected);
            break;
        case SA::RegionShape::Box:
            draw_region_box(dc, camera, region.center_mm, 0.5 * region.size_mm.cwiseAbs(), colour, selected);
            break;
        case SA::RegionShape::Cylinder: {
            const Vec3d axis = region.axis.squaredNorm() > 1e-12 ? region.axis.normalized() : Vec3d::UnitZ();
            const Vec3d half_axis = axis * (0.5 * std::max(region.size_mm.z(), 1e-6));
            const ScreenVertex first = project(region.center_mm - half_axis, camera);
            const ScreenVertex second = project(region.center_mm + half_axis, camera);
            const int radius = std::clamp(int(std::lround(std::max(region.radius_mm, 1e-6) * camera.scale)),
                                          FromDIP(4), FromDIP(240));
            const double dx = double(second.point.x - first.point.x), dy = double(second.point.y - first.point.y);
            const double length = std::max(1.0, std::hypot(dx, dy));
            const wxPoint side(int(std::lround(-dy / length * radius)), int(std::lround(dx / length * radius)));
            dc.SetPen(wxPen(colour, selected ? 3 : 2, selected ? wxPENSTYLE_SOLID : wxPENSTYLE_SHORT_DASH));
            dc.SetBrush(wxBrush(blend_colour(wxColour(236, 241, 247), colour, selected ? 0.20 : 0.11),
                                wxBRUSHSTYLE_BDIAGONAL_HATCH));
            dc.DrawCircle(first.point, radius);
            dc.DrawCircle(second.point, radius);
            dc.DrawLine(first.point + side, second.point + side);
            dc.DrawLine(first.point - side, second.point - side);
            break;
        }
        case SA::RegionShape::Surface: {
            const ScreenVertex center = project(region.center_mm, camera);
            dc.SetPen(wxPen(colour, selected ? 4 : 2));
            dc.DrawLine(center.point + wxPoint(-FromDIP(7), 0), center.point + wxPoint(FromDIP(7), 0));
            dc.DrawLine(center.point + wxPoint(0, -FromDIP(7)), center.point + wxPoint(0, FromDIP(7)));
            break;
        }
        }
    }

    void draw_orientation_gizmo(wxDC &dc, const CameraFrame &camera) const
    {
        const wxSize size = GetClientSize();
        const wxPoint origin(FromDIP(45), size.y - FromDIP(42));
        const std::array<Vec3d, 3> axes{Vec3d::UnitX(), Vec3d::UnitY(), Vec3d::UnitZ()};
        const std::array<wxColour, 3> colours{wxColour(220, 55, 55), wxColour(55, 165, 80), wxColour(55, 105, 220)};
        const std::array<wxString, 3> labels{"X", "Y", "Z"};
        for (size_t axis = 0; axis < axes.size(); ++axis) {
            const wxPoint end(origin.x + int(std::lround(axes[axis].dot(camera.right) * FromDIP(28))),
                              origin.y - int(std::lround(axes[axis].dot(camera.up) * FromDIP(28))));
            draw_arrow(dc, origin, end, colours[axis], 2);
            dc.SetTextForeground(colours[axis]);
            dc.DrawText(labels[axis], end + wxPoint(2, -8));
        }
    }

private:
    double m_yaw{-0.75};
    double m_pitch{0.48};
    double m_zoom{1.0};
    wxPoint2DDouble m_pan{0.0, 0.0};
    wxPoint m_drag_start;
    wxPoint m_last_mouse;
    bool m_dragging{false};
    bool m_drag_moved{false};
    bool m_custom_interaction{false};
    int m_drag_button{0};
    int m_right_margin{0};

    void paint()
    {
        wxAutoBufferedPaintDC dc(this);
        const wxSize size = GetClientSize();
        dc.GradientFillLinear(wxRect(wxPoint(0, 0), size), wxColour(247, 249, 252), wxColour(218, 225, 234), wxSOUTH);
        const CameraFrame frame = camera();
        draw_scene(dc, frame);
        draw_orientation_gizmo(dc, frame);
    }

    void begin_drag(wxMouseEvent &event)
    {
        m_dragging = true;
        m_drag_moved = false;
        m_drag_start = m_last_mouse = event.GetPosition();
        m_drag_button = event.LeftIsDown() ? 1 : (event.MiddleIsDown() ? 2 : 3);
        m_custom_interaction = m_drag_button == 1 && begin_interaction(event.GetPosition());
        if (!HasCapture())
            CaptureMouse();
    }

    void drag(wxMouseEvent &event)
    {
        if (!m_dragging) {
            hovered(event.GetPosition());
            return;
        }
        const wxPoint point = event.GetPosition();
        const wxPoint delta = point - m_last_mouse;
        if (std::abs(point.x - m_drag_start.x) + std::abs(point.y - m_drag_start.y) > FromDIP(4))
            m_drag_moved = true;
        if (m_custom_interaction) {
            update_interaction(point);
        } else if (m_drag_button == 1) {
            m_yaw += 0.010 * delta.x;
            m_pitch = std::clamp(m_pitch - 0.010 * delta.y, -1.50, 1.50);
        } else {
            m_pan.m_x += delta.x;
            m_pan.m_y += delta.y;
        }
        m_last_mouse = point;
        Refresh();
    }

    void end_drag(wxMouseEvent &event)
    {
        if (!m_dragging)
            return;
        const bool was_click = m_drag_button == 1 && !m_drag_moved;
        const bool was_custom = m_custom_interaction;
        m_dragging = false;
        m_custom_interaction = false;
        if (HasCapture())
            ReleaseMouse();
        if (was_custom)
            end_interaction();
        else if (was_click)
            clicked(event.GetPosition(), false);
    }
};

} // namespace

class StrengthLoadPanel::SetupCanvas final : public SoftwareViewport3D
{
public:
    enum class Placement { None = -1, Preserve = 100 };

    SetupCanvas(wxWindow *parent, std::shared_ptr<StrengthAnalysisSession> session,
                std::function<void(SA::LoadType, const Vec3d &, const Vec3d &, const std::vector<size_t> &)> load_placed,
                std::function<void(const Vec3d &, const std::vector<size_t> &)> preserve_placed,
                std::function<void(int, int, bool)> item_selected,
                std::function<void(bool)> item_changed)
        : SoftwareViewport3D(parent, parent->FromDIP(wxSize(560, 500)))
        , m_session(std::move(session))
        , m_load_placed(std::move(load_placed))
        , m_preserve_placed(std::move(preserve_placed))
        , m_item_selected(std::move(item_selected))
        , m_item_changed(std::move(item_changed))
    {
        Bind(wxEVT_CHAR_HOOK, [this](wxKeyEvent &event) {
            if (event.GetKeyCode() == WXK_ESCAPE && m_placement != int(Placement::None))
                cancel_placement();
            else
                event.Skip();
        });
    }

    void rebuild_surface_groups()
    {
        m_surface_patches = SA::group_coplanar_surfaces(m_session->mesh);
        m_components = SA::mesh_connected_components(m_session->mesh);
        Refresh();
    }

    void set_selection_mode(int mode)
    {
        m_selection_mode = std::clamp(mode, 0, 2);
        Refresh();
    }

    void begin_load(SA::LoadType type)
    {
        m_placement = int(type);
        m_hover_surface = false;
        SetCursor(wxCursor(wxCURSOR_CROSS));
        SetFocus();
        Refresh();
    }

    void begin_preserve()
    {
        m_placement = int(Placement::Preserve);
        m_hover_surface = false;
        SetCursor(wxCursor(wxCURSOR_CROSS));
        SetFocus();
        Refresh();
    }

    void cancel_placement()
    {
        m_placement = int(Placement::None);
        m_hover_surface = false;
        SetCursor(wxNullCursor);
        Refresh();
    }

    void select_item(int kind, int index)
    {
        m_selected_kind = kind;
        m_selected_index = index;
        Refresh();
    }

    void preview_load(const SA::Load &load)
    {
        if (!m_preview_region_active || !m_preview_region.center_mm.isApprox(load.region.center_mm))
            m_preview_face = std::numeric_limits<size_t>::max();
        m_preview_region_active = true;
        m_preview_kind = 1;
        m_preview_load_type = load.type;
        m_preview_region = load.region;
        m_preview_direction = load.direction.squaredNorm() > 1e-12 ? load.direction.normalized() : Vec3d::UnitZ();
        Refresh();
        Update();
    }

    void preview_preserve(const SA::SphericalRegion &region)
    {
        if (!m_preview_region_active || !m_preview_region.center_mm.isApprox(region.center_mm))
            m_preview_face = std::numeric_limits<size_t>::max();
        m_preview_region_active = true;
        m_preview_kind = 2;
        m_preview_region = region;
        Refresh();
        Update();
    }

    void clear_preview()
    {
        m_preview_region_active = false;
        m_preview_face = std::numeric_limits<size_t>::max();
        Refresh();
    }

protected:
    std::vector<Vec3d> scene_points() const override
    {
        std::vector<Vec3d> points;
        points.reserve(m_session->mesh.vertices.size());
        for (const Vec3f &point : m_session->mesh.vertices)
            points.emplace_back(point.cast<double>());
        return points;
    }

    void draw_scene(wxDC &dc, const CameraFrame &camera) override
    {
        const indexed_triangle_set &mesh = m_session->mesh;
        m_glyphs.clear();
        m_center_handle = m_direction_handle = m_size_handle = m_height_handle = wxPoint(-10000, -10000);
        if (mesh.vertices.empty() || mesh.indices.empty()) {
            dc.SetTextForeground(wxColour(65, 72, 82));
            dc.DrawText(_L("Select a model object in Prepare to begin a simulation study."), FromDIP(22), FromDIP(22));
            return;
        }

        std::vector<ScreenVertex> projected;
        projected.reserve(mesh.vertices.size());
        for (const Vec3f &point : mesh.vertices)
            projected.push_back(project(point.cast<double>(), camera));

        std::vector<size_t> faces;
        faces.reserve(mesh.indices.size());
        for (size_t face = 0; face < mesh.indices.size(); ++face)
            if (valid_triangle(mesh.indices[face], projected.size()))
                faces.push_back(face);
        const Vec3d light = Vec3d(-0.3, -0.5, 0.82).normalized();
        const SA::SphericalRegion *highlight_region = nullptr;
        bool highlight_whole_model = false;
        wxColour highlight_colour(63, 145, 224);
        if (m_preview_region_active) {
            highlight_region = &m_preview_region;
            highlight_whole_model = m_preview_region.whole_model;
            highlight_colour = m_preview_kind == 2 ? wxColour(43, 153, 91) : load_colour(m_preview_load_type);
        } else if (m_hover_surface) {
            highlight_region = &m_hover_region;
            highlight_whole_model = false;
            highlight_colour = m_placement == int(Placement::Preserve) ? wxColour(43, 153, 91) :
                load_colour(SA::LoadType(m_placement));
        } else if (m_selected_kind == 1 && m_selected_index >= 0 &&
                   size_t(m_selected_index) < m_session->setup.loads.size()) {
            const SA::Load &selected = m_session->setup.loads[size_t(m_selected_index)];
            highlight_region = &selected.region;
            highlight_whole_model = selected.region.whole_model;
            highlight_colour = load_colour(selected.type);
        } else if (m_selected_kind == 2 && m_selected_index >= 0 &&
                   size_t(m_selected_index) < m_session->setup.preserve_regions.size()) {
            highlight_region = &m_session->setup.preserve_regions[size_t(m_selected_index)];
            highlight_colour = wxColour(43, 153, 91);
        }
        std::vector<unsigned char> highlighted_vertices(projected.size(), 0);
        if (highlight_region != nullptr) {
            for (size_t vertex : SA::vertices_in_region(mesh, *highlight_region))
                if (vertex < highlighted_vertices.size())
                    highlighted_vertices[vertex] = 1;
        }
        DepthBitmap surface_bitmap(GetClientSize());
        for (size_t face : faces) {
            const Vec3i32 &triangle = mesh.indices[face];
            const Vec3d a = mesh.vertices[size_t(triangle[0])].cast<double>();
            const Vec3d b = mesh.vertices[size_t(triangle[1])].cast<double>();
            const Vec3d c = mesh.vertices[size_t(triangle[2])].cast<double>();
            const Vec3d cross = (b - a).cross(c - a);
            const double shade = cross.squaredNorm() > 1e-12 ? 0.55 + 0.35 * std::abs(cross.normalized().dot(light)) : 0.65;
            const int base = int(std::lround(205.0 * shade));
            wxColour surface(std::clamp(base + 18, 0, 255), std::clamp(base + 25, 0, 255),
                             std::clamp(base + 32, 0, 255));
            if (highlight_region != nullptr) {
                const int selected_vertices = int(highlighted_vertices[size_t(triangle[0])]) +
                    int(highlighted_vertices[size_t(triangle[1])]) + int(highlighted_vertices[size_t(triangle[2])]);
                const bool exact_surface = highlight_region->shape == SA::RegionShape::Surface;
                const bool selected_surface = exact_surface && std::find(highlight_region->surface_triangles.begin(),
                    highlight_region->surface_triangles.end(), face) != highlight_region->surface_triangles.end();
                const bool in_region = highlight_whole_model || selected_surface ||
                    (!exact_surface && selected_vertices > 0) ||
                    (m_preview_region_active && m_preview_face == face) ||
                    (m_hover_surface && m_hover_face == face);
                if (in_region)
                    surface = blend_colour(surface, highlight_colour,
                                           (m_preview_face == face || m_hover_face == face) ? 0.82 :
                                               std::min(0.72, 0.34 + 0.12 * selected_vertices));
            }
            surface_bitmap.triangle(projected[size_t(triangle[0])], projected[size_t(triangle[1])],
                                    projected[size_t(triangle[2])],
                                    [surface](double, double, double) { return surface; });
        }
        for (size_t face : faces) {
            const Vec3i32 &triangle = mesh.indices[face];
            for (int edge = 0; edge < 3; ++edge)
                surface_bitmap.line(projected[size_t(triangle[edge])], projected[size_t(triangle[(edge + 1) % 3])],
                                    wxColour(94, 108, 124));
        }
        surface_bitmap.draw(dc);
        if (highlight_region != nullptr && !highlight_whole_model) {
            dc.SetPen(wxPen(highlight_colour, 1));
            dc.SetBrush(wxBrush(highlight_colour));
            for (size_t vertex = 0; vertex < highlighted_vertices.size(); ++vertex)
                if (highlighted_vertices[vertex])
                    dc.DrawCircle(projected[vertex].point, FromDIP(2));
        }

        double radius = 1.0;
        for (const Vec3f &point : mesh.vertices)
            radius = std::max(radius, (point.cast<double>() - camera.center).norm());
        const double glyph_length = 0.30 * radius;
        double maximum_load = 0.0;
        for (const SA::Load &load : m_session->setup.loads)
            if (load.active && load.type != SA::LoadType::Fixed)
                maximum_load = std::max(maximum_load, load.magnitude_n *
                    (load.type == SA::LoadType::ImpactForce ? load.impact_factor : 1.0));

        for (size_t index = 0; index < m_session->setup.loads.size(); ++index) {
            const SA::Load &load = m_session->setup.loads[index];
            if (!load.active)
                continue;
            const Vec3d center = load.region.whole_model ? camera.center : load.region.center_mm;
            const ScreenVertex anchor = project(center, camera);
            const bool selected = m_selected_kind == 1 && m_selected_index == int(index);
            const int width = selected ? 5 : 3;
            const wxColour colour = load_colour(load.type);
            wxPoint glyph_end = anchor.point;
            if (!load.region.whole_model)
                draw_region_shape(dc, camera, load.region, colour, selected);
            if (load.type == SA::LoadType::Fixed) {
                const int half = FromDIP(selected ? 10 : 8);
                dc.SetPen(wxPen(colour, width));
                dc.SetBrush(wxBrush(wxColour(213, 228, 252)));
                wxPoint support[3]{anchor.point + wxPoint(0, -half), anchor.point + wxPoint(-half, half),
                                   anchor.point + wxPoint(half, half)};
                dc.DrawPolygon(3, support);
                dc.DrawLine(anchor.point + wxPoint(-half - 4, half + 3), anchor.point + wxPoint(half + 4, half + 3));
            } else {
                const Vec3d direction = load.direction.squaredNorm() > 1e-12 ? load.direction.normalized() : Vec3d::UnitZ();
                const double effective_magnitude = load.magnitude_n *
                    (load.type == SA::LoadType::ImpactForce ? load.impact_factor : 1.0);
                const double length_scale = maximum_load > 0.0 ?
                    std::clamp(std::sqrt(std::max(0.0, effective_magnitude) / maximum_load), 0.55, 1.35) : 1.0;
                const ScreenVertex tip = project(center + direction * glyph_length * length_scale, camera);
                draw_arrow(dc, anchor.point, tip.point, colour, width);
                glyph_end = tip.point;
                dc.SetPen(wxPen(colour, width));
                dc.SetBrush(wxBrush(wxColour(255, 255, 255)));
                dc.DrawCircle(anchor.point, FromDIP(selected ? 6 : 4));
            }
            dc.SetTextForeground(wxColour(43, 49, 58));
            dc.DrawText(wxString::FromUTF8(load.name), anchor.point + wxPoint(10, -18));
            m_glyphs.push_back({1, int(index), anchor.point, glyph_end, FromDIP(load.type == SA::LoadType::Fixed ? 14 : 9)});
            if (selected) {
                draw_handle(dc, anchor.point, wxColour(35, 129, 211), false);
                m_center_handle = anchor.point;
                if (load.type != SA::LoadType::Fixed) {
                    draw_handle(dc, glyph_end, colour, true);
                    m_direction_handle = glyph_end;
                }
                if (!load.region.whole_model && load.region.shape != SA::RegionShape::Surface) {
                    const double extent = load.region.shape == SA::RegionShape::Box ?
                        0.5 * load.region.size_mm.norm() : load.region.radius_mm;
                    m_size_handle = project(center + camera.right * extent, camera).point;
                    draw_handle(dc, m_size_handle, colour, false);
                    if (load.region.shape == SA::RegionShape::Cylinder) {
                        const Vec3d axis = load.region.axis.squaredNorm() > 1e-12 ?
                            load.region.axis.normalized() : Vec3d::UnitZ();
                        Vec3d screen_axis(axis.dot(camera.right), -axis.dot(camera.up), 0.0);
                        const double projected = screen_axis.head<2>().norm();
                        if (projected < 0.1)
                            screen_axis = Vec3d(0.0, -1.0, 0.0);
                        else
                            screen_axis /= projected;
                        m_height_handle = anchor.point + wxPoint(
                            int(std::lround(screen_axis.x() * 0.5 * load.region.size_mm.z() * camera.scale)),
                            int(std::lround(screen_axis.y() * 0.5 * load.region.size_mm.z() * camera.scale)));
                        draw_handle(dc, m_height_handle, colour, true);
                    }
                }
            }
        }

        if (m_session->setup.gravity.enabled) {
            const Vec3d direction = m_session->setup.gravity.acceleration_m_s2.squaredNorm() > 1e-12 ?
                m_session->setup.gravity.acceleration_m_s2.normalized() : -Vec3d::UnitZ();
            const Vec3d start = camera.center - direction * 0.55 * radius;
            const ScreenVertex p0 = project(start, camera);
            const ScreenVertex p1 = project(start + direction * glyph_length, camera);
            draw_arrow(dc, p0.point, p1.point, wxColour(38, 133, 140), m_selected_kind == 3 ? 5 : 3);
            dc.SetTextForeground(wxColour(28, 95, 102));
            dc.DrawText(_L("Gravity"), p0.point + wxPoint(8, -18));
            m_glyphs.push_back({3, 0, p0.point, p1.point, FromDIP(9)});
        }

        for (size_t index = 0; index < m_session->setup.preserve_regions.size(); ++index) {
            const SA::SphericalRegion &region = m_session->setup.preserve_regions[index];
            const ScreenVertex center = project(region.center_mm, camera);
            const double world_extent = region.shape == SA::RegionShape::Box ? 0.5 * region.size_mm.norm() : region.radius_mm;
            const int screen_extent = std::clamp(int(std::lround(world_extent * camera.scale)), FromDIP(6), FromDIP(180));
            const bool selected = m_selected_kind == 2 && m_selected_index == int(index);
            draw_region_shape(dc, camera, region, wxColour(43, 153, 91), selected);
            dc.SetTextForeground(wxColour(31, 113, 66));
            dc.DrawText(wxString::Format(_L("Preserve %zu"), index + 1), center.point + wxPoint(10, 8));
            m_glyphs.push_back({2, int(index), center.point, center.point,
                                std::clamp(screen_extent, FromDIP(12), FromDIP(36))});
            if (selected) {
                m_center_handle = center.point;
                draw_handle(dc, m_center_handle, wxColour(35, 129, 211), false);
                if (region.shape != SA::RegionShape::Surface) {
                    m_size_handle = project(region.center_mm + camera.right * world_extent, camera).point;
                    draw_handle(dc, m_size_handle, wxColour(43, 153, 91), false);
                    if (region.shape == SA::RegionShape::Cylinder) {
                        const Vec3d axis = region.axis.squaredNorm() > 1e-12 ? region.axis.normalized() : Vec3d::UnitZ();
                        Vec3d screen_axis(axis.dot(camera.right), -axis.dot(camera.up), 0.0);
                        const double projected = screen_axis.head<2>().norm();
                        if (projected < 0.1)
                            screen_axis = Vec3d(0.0, -1.0, 0.0);
                        else
                            screen_axis /= projected;
                        m_height_handle = center.point + wxPoint(
                            int(std::lround(screen_axis.x() * 0.5 * region.size_mm.z() * camera.scale)),
                            int(std::lround(screen_axis.y() * 0.5 * region.size_mm.z() * camera.scale)));
                        draw_handle(dc, m_height_handle, wxColour(43, 153, 91), true);
                    }
                }
            }
        }

        if (m_preview_region_active) {
            const wxColour colour = m_preview_kind == 2 ? wxColour(43, 153, 91) : load_colour(m_preview_load_type);
            if (!m_preview_region.whole_model)
                draw_region_shape(dc, camera, m_preview_region, colour, true);
            const ScreenVertex center = project(m_preview_region.center_mm, camera);
            dc.SetPen(wxPen(colour, 4));
            dc.SetBrush(wxBrush(wxColour(255, 255, 255)));
            dc.DrawCircle(center.point, FromDIP(6));
            if (m_preview_kind == 1 && m_preview_load_type == SA::LoadType::Fixed) {
                const int half = FromDIP(10);
                dc.SetBrush(wxBrush(wxColour(213, 228, 252)));
                wxPoint support[3]{center.point + wxPoint(0, -half), center.point + wxPoint(-half, half),
                                   center.point + wxPoint(half, half)};
                dc.DrawPolygon(3, support);
            } else if (m_preview_kind == 1) {
                const ScreenVertex tip = project(m_preview_region.center_mm + m_preview_direction * glyph_length, camera);
                draw_arrow(dc, center.point, tip.point, colour, 4);
            }
        } else if (m_hover_surface) {
            const bool preserve = m_placement == int(Placement::Preserve);
            const SA::LoadType type = preserve ? SA::LoadType::Fixed : SA::LoadType(m_placement);
            const wxColour colour = preserve ? wxColour(43, 153, 91) : load_colour(type);
            draw_region_shape(dc, camera, m_hover_region, colour, true);
            const ScreenVertex center = project(m_hover_region.center_mm, camera);
            dc.SetPen(wxPen(colour, 3));
            dc.SetBrush(wxBrush(wxColour(255, 255, 255)));
            dc.DrawCircle(center.point, FromDIP(5));
            if (!preserve && type == SA::LoadType::Fixed) {
                const int half = FromDIP(9);
                dc.SetBrush(wxBrush(wxColour(213, 228, 252)));
                wxPoint support[3]{center.point + wxPoint(0, -half), center.point + wxPoint(-half, half),
                                   center.point + wxPoint(half, half)};
                dc.DrawPolygon(3, support);
            } else if (!preserve) {
                draw_arrow(dc, center.point,
                           project(m_hover_region.center_mm + m_hover_direction * glyph_length, camera).point,
                           colour, 3);
            }
        }

        dc.SetTextForeground(wxColour(48, 57, 68));
        dc.DrawText(_L("Drag to orbit • middle/right drag to pan • wheel to zoom • double-click a glyph to edit"),
                    FromDIP(12), GetClientSize().y - FromDIP(24));
        if (m_placement != int(Placement::None)) {
            dc.SetBrush(wxBrush(wxColour(255, 247, 210)));
            dc.SetPen(wxPen(wxColour(210, 169, 55), 1));
            const wxString instruction = m_placement == int(Placement::Preserve) ?
                _L("Placement active: click a model face to center the preserve region. Esc or Cancel Placement to stop.") :
                _L("Placement active: click a model face to place the load or constraint and open its properties.");
            const wxSize extent = dc.GetTextExtent(instruction);
            dc.DrawRoundedRectangle(FromDIP(10), FromDIP(10), extent.x + FromDIP(20), extent.y + FromDIP(12), FromDIP(5));
            dc.SetTextForeground(wxColour(90, 65, 10));
            dc.DrawText(instruction, FromDIP(20), FromDIP(16));
        }
    }

    void hovered(const wxPoint &point) override
    {
        if (m_placement == int(Placement::None)) {
            if (m_hover_surface) {
                m_hover_surface = false;
                Refresh();
            }
            return;
        }
        if (std::abs(point.x - m_last_hover.x) + std::abs(point.y - m_last_hover.y) < FromDIP(3))
            return;
        m_last_hover = point;
        Vec3d position, normal;
        size_t face = 0;
        const bool found = pick_surface(point, position, normal, face);
        if (found) {
            m_hover_region.center_mm = position;
            m_hover_region.radius_mm = 5.0;
            m_hover_region.shape = SA::RegionShape::Surface;
            m_hover_region.surface_triangles = selection_faces(face);
            m_hover_direction = normal.squaredNorm() > 1e-12 ? Vec3d(-normal.normalized()) : Vec3d(0.0, 0.0, -1.0);
            m_hover_face = face;
        }
        if (m_hover_surface != found || found) {
            m_hover_surface = found;
            Refresh();
        }
    }

    void clicked(const wxPoint &point, bool double_click) override
    {
        if (m_placement != int(Placement::None)) {
            Vec3d position, normal;
            size_t face = 0;
            if (!pick_surface(point, position, normal, face))
                return;
            const int placement = m_placement;
            cancel_placement();
            m_preview_region_active = true;
            m_preview_kind = placement == int(Placement::Preserve) ? 2 : 1;
            m_preview_region.center_mm = position;
            m_preview_region.radius_mm = 5.0;
            m_preview_region.shape = SA::RegionShape::Surface;
            m_preview_region.surface_triangles = selection_faces(face);
            m_preview_face = face;
            m_preview_load_type = placement == int(Placement::Preserve) ? SA::LoadType::Fixed : SA::LoadType(placement);
            m_preview_direction = normal.squaredNorm() > 1e-12 ? Vec3d(-normal.normalized()) : Vec3d(0.0, 0.0, -1.0);
            Refresh();
            Update();
            if (placement == int(Placement::Preserve))
                m_preserve_placed(position, m_preview_region.surface_triangles);
            else
                m_load_placed(SA::LoadType(placement), position, normal, m_preview_region.surface_triangles);
            clear_preview();
            return;
        }

        double closest_distance = std::numeric_limits<double>::infinity();
        const Glyph *closest = nullptr;
        for (const Glyph &glyph : m_glyphs) {
            const double distance = distance_to_segment_squared(point, glyph.point, glyph.end);
            if (distance <= double(glyph.hit_radius * glyph.hit_radius) && distance < closest_distance) {
                closest_distance = distance;
                closest = &glyph;
            }
        }
        if (closest != nullptr) {
            select_item(closest->kind, closest->index);
            m_item_selected(closest->kind, closest->index, double_click);
        }
    }

    bool begin_interaction(const wxPoint &point) override
    {
        if (m_placement != int(Placement::None))
            return false;
        const double hit = double(FromDIP(11) * FromDIP(11));
        auto distance = [&point](const wxPoint &handle) {
            const double dx = double(point.x - handle.x), dy = double(point.y - handle.y);
            return dx * dx + dy * dy;
        };
        if (distance(m_direction_handle) <= hit)
            m_active_handle = Handle::Direction;
        else if (distance(m_height_handle) <= hit)
            m_active_handle = Handle::Height;
        else if (distance(m_size_handle) <= hit)
            m_active_handle = Handle::Size;
        else if (distance(m_center_handle) <= hit)
            m_active_handle = Handle::Center;
        else
            return false;

        m_interaction_start = point;
        if (m_selected_kind == 1 && m_selected_index >= 0 &&
            size_t(m_selected_index) < m_session->setup.loads.size()) {
            const SA::Load &load = m_session->setup.loads[size_t(m_selected_index)];
            m_original_region = load.region;
            m_original_direction = load.direction;
        } else if (m_selected_kind == 2 && m_selected_index >= 0 &&
                   size_t(m_selected_index) < m_session->setup.preserve_regions.size()) {
            m_original_region = m_session->setup.preserve_regions[size_t(m_selected_index)];
        } else {
            m_active_handle = Handle::None;
            return false;
        }
        SetCursor(wxCursor(wxCURSOR_HAND));
        return true;
    }

    void update_interaction(const wxPoint &point) override
    {
        SA::SphericalRegion *region = nullptr;
        SA::Load *load = nullptr;
        if (m_selected_kind == 1 && m_selected_index >= 0 &&
            size_t(m_selected_index) < m_session->setup.loads.size()) {
            load = &m_session->setup.loads[size_t(m_selected_index)];
            region = &load->region;
        } else if (m_selected_kind == 2 && m_selected_index >= 0 &&
                   size_t(m_selected_index) < m_session->setup.preserve_regions.size()) {
            region = &m_session->setup.preserve_regions[size_t(m_selected_index)];
        }
        if (region == nullptr)
            return;
        const CameraFrame frame = camera();
        const double dx = double(point.x - m_interaction_start.x);
        const double dy = double(point.y - m_interaction_start.y);
        if (m_active_handle == Handle::Center) {
            Vec3d position, normal;
            size_t face = 0;
            if (pick_surface(point, position, normal, face)) {
                region->center_mm = position;
                if (region->shape == SA::RegionShape::Surface)
                    region->surface_triangles = selection_faces(face);
            } else if (region->shape != SA::RegionShape::Surface) {
                region->center_mm = m_original_region.center_mm + (frame.right * dx - frame.up * dy) / frame.scale;
            }
        } else if (m_active_handle == Handle::Direction && load != nullptr) {
            const ScreenVertex center = project(region->center_mm, frame);
            const double screen_x = double(point.x - center.point.x);
            const double screen_y = double(point.y - center.point.y);
            const double projected_length = std::max(1.0, std::hypot(screen_x, screen_y));
            const Vec3d original_direction = m_original_direction.squaredNorm() > 1e-12 ?
                m_original_direction.normalized() : Vec3d::UnitZ();
            const Vec3d candidate = frame.right * screen_x - frame.up * screen_y +
                frame.forward * (original_direction.dot(frame.forward) * projected_length);
            if (candidate.squaredNorm() > 1e-12)
                load->direction = candidate.normalized();
        } else if (m_active_handle == Handle::Size) {
            const ScreenVertex center = project(region->center_mm, frame);
            const double extent = std::max(0.1, std::hypot(double(point.x - center.point.x),
                                                          double(point.y - center.point.y)) / frame.scale);
            if (region->shape == SA::RegionShape::Box) {
                const double original_extent = std::max(1e-9, 0.5 * m_original_region.size_mm.norm());
                region->size_mm = m_original_region.size_mm * (extent / original_extent);
            } else {
                region->radius_mm = extent;
                if (region->shape == SA::RegionShape::Cylinder) {
                    region->size_mm.x() = 2.0 * extent;
                    region->size_mm.y() = 2.0 * extent;
                }
            }
        } else if (m_active_handle == Handle::Height && region->shape == SA::RegionShape::Cylinder) {
            const Vec3d axis = region->axis.squaredNorm() > 1e-12 ? region->axis.normalized() : Vec3d::UnitZ();
            Vec2d screen_axis(axis.dot(frame.right), -axis.dot(frame.up));
            double projected = screen_axis.norm();
            if (projected < 0.1) {
                screen_axis = Vec2d(0.0, -1.0);
                projected = 1.0;
            } else {
                screen_axis /= projected;
            }
            const ScreenVertex center = project(region->center_mm, frame);
            const Vec2d cursor(double(point.x - center.point.x), double(point.y - center.point.y));
            const double half_height = std::abs(cursor.dot(screen_axis)) / frame.scale;
            region->size_mm.z() = std::max(0.2, 2.0 * half_height);
        }
        m_item_changed(false);
        Refresh();
    }

    void end_interaction() override
    {
        if (m_active_handle == Handle::None)
            return;
        m_active_handle = Handle::None;
        SetCursor(wxNullCursor);
        m_item_changed(true);
    }

private:
    struct Glyph { int kind; int index; wxPoint point; wxPoint end; int hit_radius; };
    enum class Handle { None, Center, Direction, Size, Height };
    std::shared_ptr<StrengthAnalysisSession> m_session;
    std::function<void(SA::LoadType, const Vec3d &, const Vec3d &, const std::vector<size_t> &)> m_load_placed;
    std::function<void(const Vec3d &, const std::vector<size_t> &)> m_preserve_placed;
    std::function<void(int, int, bool)> m_item_selected;
    std::function<void(bool)> m_item_changed;
    std::vector<Glyph> m_glyphs;
    std::vector<SA::SurfacePatch> m_surface_patches;
    std::vector<std::vector<size_t>> m_components;
    int m_selection_mode{1};
    int m_placement{int(Placement::None)};
    int m_selected_kind{0};
    int m_selected_index{-1};
    bool m_preview_region_active{false};
    int m_preview_kind{0};
    SA::LoadType m_preview_load_type{SA::LoadType::LocalForce};
    SA::SphericalRegion m_preview_region;
    Vec3d m_preview_direction{Vec3d::UnitZ()};
    size_t m_preview_face{std::numeric_limits<size_t>::max()};
    bool m_hover_surface{false};
    SA::SphericalRegion m_hover_region;
    Vec3d m_hover_direction{Vec3d::UnitZ()};
    size_t m_hover_face{std::numeric_limits<size_t>::max()};
    wxPoint m_last_hover{-10000, -10000};
    wxPoint m_center_handle{-10000, -10000};
    wxPoint m_direction_handle{-10000, -10000};
    wxPoint m_size_handle{-10000, -10000};
    wxPoint m_height_handle{-10000, -10000};
    wxPoint m_interaction_start;
    Handle m_active_handle{Handle::None};
    SA::SphericalRegion m_original_region;
    Vec3d m_original_direction{Vec3d::UnitZ()};

    static void draw_handle(wxDC &dc, const wxPoint &point, const wxColour &colour, bool round)
    {
        dc.SetPen(wxPen(colour, 2));
        dc.SetBrush(wxBrush(*wxWHITE));
        if (round)
            dc.DrawCircle(point, 6);
        else
            dc.DrawRectangle(point.x - 5, point.y - 5, 10, 10);
    }

    std::vector<size_t> selection_faces(size_t face) const
    {
        if (m_selection_mode == 1)
            for (const SA::SurfacePatch &patch : m_surface_patches)
                if (std::find(patch.triangles.begin(), patch.triangles.end(), face) != patch.triangles.end())
                    return patch.triangles;
        if (m_selection_mode == 2)
            for (const auto &component : m_components)
                if (std::find(component.begin(), component.end(), face) != component.end())
                    return component;
        return {face};
    }

    static double distance_to_segment_squared(const wxPoint &point, const wxPoint &start, const wxPoint &end)
    {
        const double dx = double(end.x - start.x), dy = double(end.y - start.y);
        const double length_squared = dx * dx + dy * dy;
        const double t = length_squared > 1e-9 ? std::clamp(
            (double(point.x - start.x) * dx + double(point.y - start.y) * dy) / length_squared, 0.0, 1.0) : 0.0;
        const double x = double(start.x) + t * dx;
        const double y = double(start.y) + t * dy;
        const double ex = double(point.x) - x, ey = double(point.y) - y;
        return ex * ex + ey * ey;
    }

    bool pick_surface(const wxPoint &point, Vec3d &position, Vec3d &normal, size_t &face_index) const
    {
        const indexed_triangle_set &mesh = m_session->mesh;
        if (mesh.vertices.empty())
            return false;
        const CameraFrame frame = camera();
        std::vector<ScreenVertex> projected;
        projected.reserve(mesh.vertices.size());
        for (const Vec3f &vertex : mesh.vertices)
            projected.push_back(project(vertex.cast<double>(), frame));
        bool found = false;
        double best_depth = -std::numeric_limits<double>::infinity();
        for (size_t face = 0; face < mesh.indices.size(); ++face) {
            const Vec3i32 &triangle = mesh.indices[face];
            const size_t i0 = size_t(triangle[0]), i1 = size_t(triangle[1]), i2 = size_t(triangle[2]);
            if (i0 >= projected.size() || i1 >= projected.size() || i2 >= projected.size())
                continue;
            const wxPoint screen[3]{projected[i0].point, projected[i1].point, projected[i2].point};
            double weights[3];
            if (!barycentric(point, screen, weights))
                continue;
            const double depth = weights[0] * projected[i0].depth + weights[1] * projected[i1].depth +
                                 weights[2] * projected[i2].depth;
            if (depth <= best_depth)
                continue;
            const Vec3d a = mesh.vertices[i0].cast<double>();
            const Vec3d b = mesh.vertices[i1].cast<double>();
            const Vec3d c = mesh.vertices[i2].cast<double>();
            const Vec3d cross = (b - a).cross(c - a);
            if (cross.squaredNorm() < 1e-12)
                continue;
            best_depth = depth;
            position = weights[0] * a + weights[1] * b + weights[2] * c;
            normal = cross.normalized();
            if (normal.dot(frame.forward) < 0.0)
                normal = -normal;
            face_index = face;
            found = true;
        }
        return found;
    }
};

StrengthLoadPanel::StrengthLoadPanel(wxWindow *parent, Plater *plater, std::shared_ptr<StrengthAnalysisSession> session)
    : wxScrolledWindow(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL | wxHSCROLL | wxTAB_TRAVERSAL)
    , m_plater(plater)
    , m_session(std::move(session))
{
    SetBackgroundColour(*wxWHITE);
    SetScrollRate(FromDIP(12), FromDIP(12));
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
    m_status_label = new wxStaticText(this, wxID_ANY,
        _L("Pre-check pending. Results are offline engineering estimates, not certification-grade FEA."));
    root->Add(m_status_label, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);

    auto *workspace = new wxStaticBoxSizer(wxVERTICAL, this, _L("Simulation study workspace"));
    auto *placement_tools = new wxBoxSizer(wxHORIZONTAL);
    auto *place_fixed = new wxButton(this, wxID_ANY, _L("Fixed support"));
    auto *place_force = new wxButton(this, wxID_ANY, _L("Force"));
    auto *place_bearing = new wxButton(this, wxID_ANY, _L("Bearing"));
    auto *place_impact = new wxButton(this, wxID_ANY, _L("Impact"));
    auto *place_global = new wxButton(this, wxID_ANY, _L("Global load"));
    auto *place_preserve = new wxButton(this, wxID_ANY, _L("Preserve region"));
    auto *gravity_dialog = new wxButton(this, wxID_ANY, _L("Gravity"));
    auto *generate_faces = new wxButton(this, wxID_ANY, _L("Generate solid faces"));
    auto *selection_mode = new wxChoice(this, wxID_ANY, wxDefaultPosition, wxDefaultSize,
                                        {_L("Triangle"), _L("Solid face"), _L("Component")});
    selection_mode->SetSelection(1);
    auto *cancel_placement = new wxButton(this, wxID_ANY, _L("Cancel placement"));
    for (wxButton *button : {place_fixed, place_force, place_bearing, place_impact, place_global,
                             place_preserve, gravity_dialog, generate_faces, cancel_placement})
        placement_tools->Add(button, 0, wxRIGHT, gap);
    placement_tools->Add(new wxStaticText(this, wxID_ANY, _L("Selection")), 0,
                         wxALIGN_CENTER_VERTICAL | wxRIGHT, FromDIP(4));
    placement_tools->Add(selection_mode, 0, wxRIGHT, gap);
    workspace->Add(placement_tools, 0, wxEXPAND | wxALL, gap);

    auto *study_tools = new wxBoxSizer(wxHORIZONTAL);
    auto *material_dialog = new wxButton(this, wxID_ANY, _L("Study material"));
    auto *infill_dialog = new wxButton(this, wxID_ANY, _L("Print structure"));
    auto *criteria_dialog = new wxButton(this, wxID_ANY, _L("Optimization objectives"));
    auto *fit_view = new wxButton(this, wxID_ANY, _L("Fit"));
    m_setup_undo = new wxButton(this, wxID_ANY, _L("Undo setup"));
    m_setup_redo = new wxButton(this, wxID_ANY, _L("Redo setup"));
    auto *view = new wxChoice(this, wxID_ANY, wxDefaultPosition, wxDefaultSize,
                              {_L("Isometric"), _L("Front"), _L("Top"), _L("Right")});
    view->SetSelection(0);
    m_precheck_button = new Button(this, _L("Pre-check"));
    m_precheck_button->SetStyle(ButtonStyle::Regular, ButtonType::Compact);
    m_precheck_button->SetPaddingSize(FromDIP(wxSize(12, 5)));
    set_precheck_state(0);
    m_run_button = new wxButton(this, wxID_ANY, _L("Solve"));
    m_cancel_button = new wxButton(this, wxID_ANY, _L("Cancel solve"));
    for (wxButton *button : {material_dialog, infill_dialog, criteria_dialog, fit_view})
        study_tools->Add(button, 0, wxRIGHT, gap);
    study_tools->Add(new wxStaticText(this, wxID_ANY, _L("View")), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, FromDIP(4));
    study_tools->Add(view, 0, wxRIGHT, gap);
    study_tools->Add(m_setup_undo, 0, wxRIGHT, gap);
    study_tools->Add(m_setup_redo, 0, wxRIGHT, gap);
    study_tools->AddStretchSpacer();
    study_tools->Add(m_precheck_button, 0, wxRIGHT, gap);
    study_tools->Add(m_run_button, 0, wxRIGHT, gap);
    study_tools->Add(m_cancel_button, 0);
    workspace->Add(study_tools, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);

    m_setup_canvas = new SetupCanvas(this, m_session,
        [this](SA::LoadType type, const Vec3d &position, const Vec3d &normal, const std::vector<size_t> &faces) {
            place_load_at(type, position, normal, faces);
        },
        [this](const Vec3d &position, const std::vector<size_t> &faces) { place_preserve_at(position, faces); },
        [this](int kind, int index, bool edit) { select_canvas_item(kind, index, edit); },
        [this](bool commit) {
            refresh_operation_panel();
            if (commit) {
                // Dragging changes the model directly. Synchronize the secondary editor before
                // a later pre-check can accidentally write its old coordinates back to the load.
                load_current_load_editor(m_current_load);
                persist_setup();
                mark_stale();
                refresh_load_list();
                refresh_preserve_list();
                refresh_study_tree();
            }
        });
    auto *workspace_body = new wxBoxSizer(wxHORIZONTAL);
    m_study_tree = new wxTreeCtrl(this, wxID_ANY, wxDefaultPosition, FromDIP(wxSize(245, 500)),
                                  wxTR_HIDE_ROOT | wxTR_HAS_BUTTONS | wxTR_SINGLE | wxBORDER_SIMPLE);
    workspace_body->Add(m_study_tree, 0, wxEXPAND | wxRIGHT, gap);
    workspace_body->Add(m_setup_canvas, 1, wxEXPAND);

    m_operation_panel = new wxPanel(this, wxID_ANY, wxDefaultPosition, FromDIP(wxSize(330, 500)), wxBORDER_SIMPLE);
    m_operation_panel->SetMinSize(FromDIP(wxSize(330, 400)));
    auto *operation = new wxBoxSizer(wxVERTICAL);
    m_operation_title = new wxStaticText(m_operation_panel, wxID_ANY, _L("Operation details"));
    operation->Add(m_operation_title, 0, wxEXPAND | wxALL, gap);
    auto *operation_help = new wxStaticText(m_operation_panel, wxID_ANY,
        _L("Select a load, support, or preserve region. Drag its white handles in the model or edit exact values here."));
    operation_help->Wrap(FromDIP(300));
    operation->Add(operation_help, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    auto *operation_grid = new wxFlexGridSizer(2, gap, gap);
    operation_grid->AddGrowableCol(1, 1);
    m_operation_name = new wxTextCtrl(m_operation_panel, wxID_ANY);
    add_labeled(operation_grid, m_operation_panel, _L("Name"), m_operation_name, 1);
    m_operation_shape = new wxChoice(m_operation_panel, wxID_ANY, wxDefaultPosition, wxDefaultSize, region_shape_names());
    add_labeled(operation_grid, m_operation_panel, _L("Region shape"), m_operation_shape, 1);
    add_labeled(operation_grid, m_operation_panel, _L("Position (mm)"),
                vector_editor(m_operation_panel, m_operation_center, Vec3d::Zero(), {}, 54), 1);
    add_labeled(operation_grid, m_operation_panel, _L("Box size XYZ / cylinder height Z (mm)"),
                vector_editor(m_operation_panel, m_operation_size, Vec3d::Constant(10.0), {}, 54), 1);
    m_operation_radius = number_input(m_operation_panel, 5.0, 100);
    add_labeled(operation_grid, m_operation_panel, _L("Radius (mm)"), m_operation_radius, 1);
    add_labeled(operation_grid, m_operation_panel, _L("Region axis"),
                vector_editor(m_operation_panel, m_operation_axis, Vec3d::UnitZ(), {}, 54), 1);
    add_labeled(operation_grid, m_operation_panel, _L("Force direction"),
                vector_editor(m_operation_panel, m_operation_direction, Vec3d::UnitZ(), {}, 54), 1);
    m_operation_magnitude = number_input(m_operation_panel, 100.0, 100);
    add_labeled(operation_grid, m_operation_panel, _L("Force (N)"), m_operation_magnitude, 1);
    operation->Add(operation_grid, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    auto *operation_buttons = new wxBoxSizer(wxHORIZONTAL);
    m_operation_apply = new wxButton(m_operation_panel, wxID_ANY, _L("Apply"));
    m_operation_popout = new wxButton(m_operation_panel, wxID_ANY, _L("Pop out…"));
    m_operation_delete = new wxButton(m_operation_panel, wxID_ANY, _L("Delete"));
    operation_buttons->Add(m_operation_apply, 0, wxRIGHT, gap);
    operation_buttons->Add(m_operation_popout, 0, wxRIGHT, gap);
    operation_buttons->Add(m_operation_delete, 0);
    operation->Add(operation_buttons, 0, wxLEFT | wxRIGHT | wxBOTTOM, gap);
    m_operation_panel->SetSizer(operation);
    workspace_body->Add(m_operation_panel, 0, wxEXPAND | wxLEFT, gap);
    workspace->Add(workspace_body, 1, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    root->Add(workspace, 1, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);

    place_fixed->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { begin_place_load(SA::LoadType::Fixed); });
    place_force->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { begin_place_load(SA::LoadType::DirectionalForce); });
    place_bearing->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { begin_place_load(SA::LoadType::BearingForce); });
    place_impact->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { begin_place_load(SA::LoadType::ImpactForce); });
    place_global->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { begin_place_load(SA::LoadType::GlobalForce); });
    place_preserve->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { m_setup_canvas->begin_preserve(); });
    gravity_dialog->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { edit_gravity_dialog(); });
    generate_faces->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
        m_setup_canvas->rebuild_surface_groups();
        const auto patches = SA::group_coplanar_surfaces(m_session->mesh);
        m_status_label->SetLabel(wxString::Format(
            _L("Generated %zu solid-like planar faces. The source triangle mesh and shape were not changed."), patches.size()));
    });
    selection_mode->Bind(wxEVT_CHOICE, [this, selection_mode](wxCommandEvent &) {
        m_setup_canvas->set_selection_mode(selection_mode->GetSelection());
    });
    cancel_placement->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { m_setup_canvas->cancel_placement(); });
    material_dialog->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { edit_material_dialog(); });
    infill_dialog->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { edit_infill_dialog(); });
    criteria_dialog->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { edit_criteria_dialog(); });
    m_operation_apply->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { apply_operation_panel(); });
    m_operation_popout->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
        select_canvas_item(m_selected_kind, m_selected_index, true);
    });
    m_operation_delete->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { delete_selected_operation(); });
    m_setup_undo->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
        if (m_setup_history_index > 0)
            restore_setup_history(m_setup_history_index - 1);
    });
    m_setup_redo->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
        if (m_setup_history_index + 1 < m_setup_history.size())
            restore_setup_history(m_setup_history_index + 1);
    });
    fit_view->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { m_setup_canvas->fit_view(); });
    view->Bind(wxEVT_CHOICE, [this, view](wxCommandEvent &) { m_setup_canvas->set_view(view->GetSelection()); });
    m_study_tree->Bind(wxEVT_TREE_SEL_CHANGED, [this](wxTreeEvent &event) {
        if (m_refreshing_tree)
            return;
        auto *data = event.GetItem().IsOk() ?
            dynamic_cast<StudyTreeItemData *>(m_study_tree->GetItemData(event.GetItem())) : nullptr;
        if (data != nullptr && data->kind >= 1 && data->kind <= 3)
            select_canvas_item(data->kind, data->index, false);
        else {
            m_selected_kind = 0;
            m_selected_index = -1;
            if (m_setup_canvas != nullptr)
                m_setup_canvas->select_item(0, -1);
            refresh_operation_panel();
        }
    });
    m_study_tree->Bind(wxEVT_TREE_ITEM_ACTIVATED, [this](wxTreeEvent &event) {
        auto *data = dynamic_cast<StudyTreeItemData *>(m_study_tree->GetItemData(event.GetItem()));
        if (data == nullptr)
            return;
        if (data->kind >= 1 && data->kind <= 3)
            select_canvas_item(data->kind, data->index, true);
        else if (data->kind == 4)
            edit_material_dialog();
        else if (data->kind == 5)
            edit_infill_dialog();
        else if (data->kind == 6)
            edit_criteria_dialog();
    });
    m_study_tree->Bind(wxEVT_KEY_DOWN, [this](wxKeyEvent &event) {
        if (event.GetKeyCode() != WXK_DELETE && event.GetKeyCode() != WXK_BACK) {
            event.Skip();
            return;
        }
        const wxTreeItemId selected = m_study_tree->GetSelection();
        auto *data = selected.IsOk() ? dynamic_cast<StudyTreeItemData *>(m_study_tree->GetItemData(selected)) : nullptr;
        if (data == nullptr || data->kind < 1 || data->kind > 3) {
            event.Skip();
            return;
        }
        select_canvas_item(data->kind, data->index, false);
        delete_selected_operation();
    });

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
    m_follow_prepare_orientation = new wxCheckBox(this, wxID_ANY, _L("Follow selected instance orientation in Prepare"));
    m_follow_prepare_orientation->SetValue(true);
    material->Add(m_follow_prepare_orientation, 0, wxLEFT | wxRIGHT | wxBOTTOM, gap);
    m_follow_prepare_orientation->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent &) {
        m_session->setup.follow_prepare_orientation = m_follow_prepare_orientation->GetValue();
        if (m_session->setup.follow_prepare_orientation)
            m_session->setup.print_layer_axis = SA::print_layer_axis_for_transform(m_session->instance_transform);
        for (int axis = 0; axis < 3; ++axis) {
            m_layer_axis[axis]->ChangeValue(number(m_session->setup.print_layer_axis[axis]));
            m_layer_axis[axis]->Enable(!m_session->setup.follow_prepare_orientation);
        }
        mark_stale();
    });
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
    m_dense_button = new wxButton(this, wxID_ANY, _L("Preview dense region"));
    m_remove_dense_button = new wxButton(this, wxID_ANY, _L("Remove dense modifier"));
    m_orientation_button = new wxButton(this, wxID_ANY, _L("Apply best orientation"));
    m_settings_button = new wxButton(this, wxID_ANY, _L("Apply optimized settings"));
    for (wxButton *button : {save_button, m_dense_button, m_remove_dense_button, m_orientation_button, m_settings_button})
        actions->Add(button, 0, wxRIGHT, gap);
    root->Add(actions, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
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
        if (!save_current_load_editor()) {
            m_load_list->SetSelection(m_current_load);
            m_status_label->SetLabel(_L("Correct the invalid numeric load fields before selecting another operation."));
            return;
        }
        refresh_load_list();
        select_canvas_item(1, selected, false);
    });
    m_load_list->Bind(wxEVT_LISTBOX_DCLICK, [this](wxCommandEvent &) {
        const int selected = m_load_list->GetSelection();
        select_canvas_item(1, selected, true);
    });
    add_load->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
        if (!collect_setup(true, false))
            return;
        SA::Load load;
        load.name = "Load " + std::to_string(m_session->setup.loads.size() + 1);
        m_session->setup.loads.push_back(load);
        populate_from_setup();
        select_canvas_item(1, int(m_session->setup.loads.size()) - 1, false);
        persist_setup();
        mark_stale();
    });
    remove_load->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
        const int selected = m_load_list->GetSelection();
        if (selected >= 0 && size_t(selected) < m_session->setup.loads.size()) {
            m_session->setup.loads.erase(m_session->setup.loads.begin() + selected);
            m_current_load = -1;
            refresh_load_list();
            const int next = std::min(selected, int(m_session->setup.loads.size()) - 1);
            load_current_load_editor(next);
            m_selected_kind = next >= 0 ? 1 : 0;
            m_selected_index = next;
            m_setup_canvas->select_item(m_selected_kind, m_selected_index);
            refresh_study_tree();
            refresh_operation_panel();
            persist_setup();
            mark_stale();
        }
    });
    add_preserve->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
        SA::SphericalRegion region;
        if (!read_vector_strict(m_preserve_center, region.center_mm) ||
            !read_number(m_preserve_radius, region.radius_mm) || region.radius_mm <= 0.0) {
            wxMessageBox(_L("Enter a finite preserve center and a radius greater than zero."),
                         _L("Invalid preserve region"), wxOK | wxICON_WARNING, this);
            return;
        }
        region.shape = SA::RegionShape::Sphere;
        m_session->setup.preserve_regions.push_back(region);
        populate_from_setup();
        select_canvas_item(2, int(m_session->setup.preserve_regions.size()) - 1, false);
        persist_setup();
        mark_stale();
    });
    m_preserve_list->Bind(wxEVT_LISTBOX, [this](wxCommandEvent &) {
        select_canvas_item(2, m_preserve_list->GetSelection(), false);
    });
    m_preserve_list->Bind(wxEVT_LISTBOX_DCLICK, [this](wxCommandEvent &) {
        const int selected = m_preserve_list->GetSelection();
        select_canvas_item(2, selected, true);
    });
    remove_preserve->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
        const int selected = m_preserve_list->GetSelection();
        if (selected >= 0 && size_t(selected) < m_session->setup.preserve_regions.size()) {
            m_session->setup.preserve_regions.erase(m_session->setup.preserve_regions.begin() + selected);
            const int next = std::min(selected, int(m_session->setup.preserve_regions.size()) - 1);
            m_selected_kind = next >= 0 ? 2 : 0;
            m_selected_index = next;
            m_setup_canvas->select_item(m_selected_kind, m_selected_index);
            refresh_preserve_list();
            refresh_study_tree();
            refresh_operation_panel();
            persist_setup();
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
    m_precheck_button->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { run_precheck(true); });
    m_cancel_button->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { cancel_analysis(); });
    m_dense_button->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { preview_dense_region(); });
    m_remove_dense_button->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { remove_dense_modifier(); });
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
    const auto edited_setup = [this](wxCommandEvent &) {
        const uint64_t revision = m_session->revision;
        collect_setup(false, false);
        if (revision == m_session->revision)
            mark_stale();
    };
    for (wxTextCtrl *field : stale_text_fields)
        field->Bind(wxEVT_TEXT, edited_setup);
    for (wxChoice *choice : {m_load_type, m_strength_basis, m_background_pattern, m_dense_pattern})
        choice->Bind(wxEVT_CHOICE, edited_setup);
    for (wxCheckBox *checkbox : {m_load_active, m_load_whole_model})
        checkbox->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent &) {
            collect_setup(false, false);
            persist_setup();
            mark_stale();
        });
    m_gravity_enabled->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent &) {
        m_session->setup.gravity.enabled = m_gravity_enabled->GetValue();
        persist_setup();
        mark_stale();
        refresh_operation_panel();
    });

    m_cancel_button->Disable();
    m_dense_button->Disable();
    m_orientation_button->Disable();
    m_settings_button->Disable();
    refresh_study_tree();
    refresh_operation_panel();
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
    int instance_index = m_plater->get_selection().get_instance_idx();
    if (m_plater->get_selection().get_object_idx() != index)
        instance_index = -1;
    if (index >= 0 && size_t(index) < m_plater->model().objects.size()) {
        const auto &instances = m_plater->model().objects[size_t(index)]->instances;
        if (instances.size() == 1)
            instance_index = 0;
        if (instance_index < 0 || size_t(instance_index) >= instances.size())
            index = -1;
    }
    if (index < 0 || size_t(index) >= m_plater->model().objects.size()) {
        m_object_label->SetLabel(_L("Select one model instance in Prepare."));
        m_run_button->Disable();
        m_precheck_button->Disable();
        m_remove_dense_button->Disable();
        m_dense_button->Disable();
        m_orientation_button->Disable();
        m_settings_button->Disable();
        set_precheck_state(-1);
        if (m_session->object_index != -1) {
            m_session->object_index = -1;
            m_session->object_id = ObjectID();
            m_session->instance_id = ObjectID();
            m_session->instance_index = -1;
            m_session->mesh = {};
            m_session->result = {};
            m_session->solved_mesh = {};
            m_session->solved_setup = {};
            m_session->dense_profile.reset();
            m_session->persisted_setup.clear();
            m_session->stale = true;
            ++m_session->revision;
        }
        m_setup_history.clear();
        m_setup_history_index = 0;
        m_selected_kind = 0;
        m_selected_index = -1;
        m_current_load = -1;
        m_setup_canvas->select_item(0, -1);
        refresh_study_tree();
        refresh_operation_panel();
        update_setup_history_buttons();
        return;
    }

    ModelObject *object = m_plater->model().objects[size_t(index)];
    const bool new_object = object->id() != m_session->object_id;
    const ModelInstance *instance = object->instances[size_t(instance_index)];
    const Transform3d instance_transform = instance->get_matrix();
    const bool changed_instance = new_object || instance->id() != m_session->instance_id ||
        !instance_transform.matrix().isApprox(m_session->instance_transform.matrix(), 1e-12);
    m_session->instance_id = instance->id();
    m_session->instance_index = instance_index;
    m_session->instance_transform = instance_transform;
    const indexed_triangle_set current_mesh = object->raw_mesh().its;
    const bool changed_geometry = !new_object && !meshes_equal(current_mesh, m_session->mesh);
    m_session->object_index = index;
    m_session->mesh = current_mesh;
    SA::Setup stored_setup;
    bool stored_setup_readable = true;
    if (const auto *option = object->config.get().option<ConfigOptionString>("strength_analysis_setup");
        option != nullptr && !option->value.empty()) {
        std::string error;
        stored_setup_readable = SA::deserialize_setup_from_config(option->value, stored_setup, &error);
        if (!stored_setup_readable)
            m_status_label->SetLabel(wxString::Format(_L("Saved strength setup could not be read: %s"),
                                                      wxString::FromUTF8(error)));
    }
    const std::string serialized_stored_setup = SA::serialize_setup(stored_setup);
    const bool changed_persisted_setup = !new_object && stored_setup_readable &&
        serialized_stored_setup != m_session->persisted_setup;
    if (new_object) {
        set_precheck_state(0);
        m_session->object_id = object->id();
        m_session->setup = stored_setup_readable ? stored_setup : SA::Setup{};
        m_session->persisted_setup = stored_setup_readable ? serialized_stored_setup : std::string();
        m_session->result = SA::Result{};
        m_session->solved_mesh = {};
        m_session->solved_setup = SA::Setup{};
        m_session->dense_profile.reset();
        m_session->stale = true;
        ++m_session->revision;
        m_current_load = -1;
        m_selected_kind = 0;
        m_selected_index = -1;
        m_setup_canvas->select_item(0, -1);
        populate_from_setup();
        m_setup_history.assign(1, m_session->setup);
        m_setup_history_index = 0;
        update_setup_history_buttons();
    } else if (changed_persisted_setup) {
        set_precheck_state(0);
        m_session->setup = stored_setup;
        m_session->persisted_setup = serialized_stored_setup;
        m_session->stale = true;
        ++m_session->revision;
        m_current_load = -1;
        m_selected_kind = 0;
        m_selected_index = -1;
        m_setup_canvas->select_item(0, -1);
        populate_from_setup();
        m_setup_history.assign(1, m_session->setup);
        m_setup_history_index = 0;
        update_setup_history_buttons();
        m_status_label->SetLabel(_L("Strength setup changed through the main Undo/Redo history; results require a new solve."));
    }
    if (changed_instance || new_object || changed_persisted_setup) {
        if (m_session->setup.follow_prepare_orientation) {
            m_session->setup.print_layer_axis = SA::print_layer_axis_for_transform(instance_transform);
            for (int axis = 0; axis < 3; ++axis)
                m_layer_axis[axis]->ChangeValue(number(m_session->setup.print_layer_axis[axis]));
            if (stored_setup_readable)
                persist_setup(false);
        }
        m_session->stale = true;
        ++m_session->revision;
    }
    if (changed_geometry) {
        set_precheck_state(0);
        m_session->stale = true;
        ++m_session->revision;
        m_dense_button->Disable();
        m_orientation_button->Disable();
        m_settings_button->Disable();
        m_status_label->SetLabel(_L("Model geometry changed; run the analysis again."));
    }
    if (m_session->stale) {
        m_dense_button->Disable();
        m_orientation_button->Disable();
        m_settings_button->Disable();
    }
    m_object_label->SetLabel(wxString::Format(_L("Object: %s — %zu vertices, %zu triangles"),
        wxString::FromUTF8(object->name), current_mesh.vertices.size(), current_mesh.indices.size()));
    m_run_button->Enable(!current_mesh.empty() && !m_analysis_running);
    m_precheck_button->Enable(!current_mesh.empty() && !m_analysis_running);
    m_remove_dense_button->Enable(std::any_of(object->volumes.begin(), object->volumes.end(), is_strength_dense_modifier));
    if (m_setup_canvas != nullptr) {
        if (new_object || changed_geometry)
            m_setup_canvas->rebuild_surface_groups();
        if (new_object)
            m_setup_canvas->fit_view();
        else
            m_setup_canvas->Refresh();
    }
    refresh_study_tree();
    if (new_object || changed_persisted_setup || changed_geometry)
        refresh_operation_panel();
    refresh_precheck_state();
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
    m_numeric_inputs_valid = true;
    if (m_session->setup.follow_prepare_orientation)
        m_session->setup.print_layer_axis = SA::print_layer_axis_for_transform(m_session->instance_transform);
    const SA::Setup &setup = m_session->setup;
    populate_material_fields();
    m_follow_prepare_orientation->SetValue(setup.follow_prepare_orientation);
    for (int axis = 0; axis < 3; ++axis) {
        m_layer_axis[axis]->ChangeValue(number(setup.print_layer_axis[axis]));
        m_layer_axis[axis]->Enable(!setup.follow_prepare_orientation);
    }
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
    load_current_load_editor(setup.loads.empty() ? -1 :
        (m_selected_kind == 1 && m_selected_index >= 0 && size_t(m_selected_index) < setup.loads.size() ? m_selected_index : 0));
    refresh_preserve_list();
    refresh_study_tree();
    if ((m_selected_kind == 1 && (m_selected_index < 0 || size_t(m_selected_index) >= setup.loads.size())) ||
        (m_selected_kind == 2 && (m_selected_index < 0 || size_t(m_selected_index) >= setup.preserve_regions.size()))) {
        m_selected_kind = 0;
        m_selected_index = -1;
    }
    refresh_operation_panel();
}

bool StrengthLoadPanel::collect_setup(bool show_errors, bool validate_setup)
{
    const SA::Setup previous = m_session->setup;
    const std::string previous_setup = SA::serialize_setup(m_session->setup);
    bool numeric = save_current_load_editor();
    SA::Setup &setup = m_session->setup;
    SA::Material &m = setup.material;
    const int selected_material = m_material_choice->GetSelection();
    if (selected_material >= 0 && size_t(selected_material) < SA::builtin_materials().size()) {
        const SA::MaterialCalibration calibration = m.calibration;
        m = SA::builtin_materials()[size_t(selected_material)];
        m.calibration = calibration;
    }
    const std::array<double *, 12> properties{&m.density_kg_m3, &m.elastic_modulus_xy_pa, &m.elastic_modulus_z_pa,
        &m.poisson_xy, &m.shear_modulus_xy_pa, &m.shear_modulus_xz_pa, &m.yield_strength_xy_pa, &m.yield_strength_z_pa,
        &m.ultimate_strength_xy_pa, &m.ultimate_strength_z_pa, &m.shear_strength_xy_pa, &m.shear_strength_xz_pa};
    const std::array<double, 12> units{1.0, 1e9, 1e9, 1.0, 1e9, 1e9, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6};
    for (size_t index = 0; index < properties.size(); ++index) {
        // A pre-check must not round a measured value merely because its display has fewer
        // significant digits. Preserve untouched properties and imported provenance exactly.
        if (m_material_fields[index]->GetValue() == number(*properties[index] / units[index]))
            continue;
        double value = 0.0;
        if (read_number(m_material_fields[index], value))
            *properties[index] = value * units[index];
        else
            numeric = false;
    }
    if (selected_material >= 0 && size_t(selected_material) < SA::builtin_materials().size()) {
        const SA::Material &builtin = SA::builtin_materials()[size_t(selected_material)];
        if (!material_properties_match(m, builtin)) {
            m.key = "custom";
            m.name = builtin.name + " (custom)";
            m.provenance = "User-edited values based on a bundled material estimate; verify against a filament datasheet and printed coupons.";
            m_material_choice->SetSelection(int(SA::builtin_materials().size()));
        }
    }
    std::array<double, 5> scales{m.calibration.modulus_xy_scale, m.calibration.modulus_z_scale,
        m.calibration.strength_xy_scale, m.calibration.strength_z_scale, m.calibration.shear_scale};
    for (size_t index = 0; index < scales.size(); ++index)
        numeric = read_number(m_calibration_fields[index], scales[index]) && numeric;
    m.calibration.modulus_xy_scale = scales[0];
    m.calibration.modulus_z_scale = scales[1];
    m.calibration.strength_xy_scale = scales[2];
    m.calibration.strength_z_scale = scales[3];
    m.calibration.shear_scale = scales[4];
    m.calibration.source = m_calibration_source->GetValue().utf8_string();
    if (setup.follow_prepare_orientation)
        setup.print_layer_axis = SA::print_layer_axis_for_transform(m_session->instance_transform);
    else
        numeric = read_vector_strict(m_layer_axis, setup.print_layer_axis) && numeric;
    setup.gravity.enabled = m_gravity_enabled->GetValue();
    numeric = read_vector_strict(m_gravity, setup.gravity.acceleration_m_s2) && numeric;
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

    m_numeric_inputs_valid = numeric;
    const std::vector<std::string> errors = validate_setup ? SA::validate(m_session->mesh, setup) : std::vector<std::string>();
    if (!numeric || !errors.empty()) {
        m_session->setup = previous;
        set_precheck_state(-1);
        if (show_errors) {
            wxString message = numeric ? wxString() : _L("One or more numeric fields are invalid.\n");
            for (const std::string &error : errors)
                message += wxString::FromUTF8(error) + "\n";
            wxMessageBox(message, _L("Strength setup is incomplete"), wxOK | wxICON_WARNING, this);
        }
        return false;
    }
    if (SA::serialize_setup(setup) != previous_setup)
        mark_stale();
    else
        refresh_precheck_state();
    return true;
}

void StrengthLoadPanel::persist_setup(bool take_snapshot)
{
    const int index = m_session->object_index;
    if (index < 0 || size_t(index) >= m_plater->model().objects.size())
        return;
    ModelObject *object = m_plater->model().objects[size_t(index)];
    if (object->id() != m_session->object_id)
        return;
    const std::string serialized = SA::serialize_setup_for_config(m_session->setup);
    if (const auto *existing = object->config.get().option<ConfigOptionString>("strength_analysis_setup")) {
        if (existing->value == serialized) {
            m_session->persisted_setup = SA::serialize_setup(m_session->setup);
            return;
        }
    }
    if (take_snapshot)
        m_plater->take_snapshot("Save strength analysis setup");
    object->config.set_key_value("strength_analysis_setup", new ConfigOptionString(serialized));
    m_session->persisted_setup = SA::serialize_setup(m_session->setup);
    m_plater->set_plater_dirty(true);
}

void StrengthLoadPanel::mark_stale()
{
    record_setup_history();
    // Keep accepted study edits in the project, including dialog edits made before the first
    // solve. Setup Undo/Redo owns the fine-grained history; avoid a main snapshot per keystroke.
    persist_setup(false);
    m_session->stale = true;
    ++m_session->revision;
    m_dense_button->Disable();
    m_orientation_button->Disable();
    m_settings_button->Disable();
    if (m_session->result.status != SA::AnalysisStatus::NotRun)
        m_status_label->SetLabel(_L("Inputs changed; displayed simulation results are stale until rerun."));
    if (m_result_callback)
        m_result_callback();
    if (m_setup_canvas != nullptr)
        m_setup_canvas->Refresh();
    refresh_study_tree();
    refresh_precheck_state();
}

void StrengthLoadPanel::record_setup_history()
{
    if (m_restoring_history)
        return;
    if (m_setup_history.empty()) {
        m_setup_history.push_back(m_session->setup);
        m_setup_history_index = 0;
        update_setup_history_buttons();
        return;
    }
    if (SA::serialize_setup(m_setup_history[m_setup_history_index]) == SA::serialize_setup(m_session->setup)) {
        update_setup_history_buttons();
        return;
    }
    if (m_setup_history_index + 1 < m_setup_history.size())
        m_setup_history.erase(m_setup_history.begin() + std::ptrdiff_t(m_setup_history_index + 1), m_setup_history.end());
    m_setup_history.push_back(m_session->setup);
    if (m_setup_history.size() > 100)
        m_setup_history.erase(m_setup_history.begin());
    m_setup_history_index = m_setup_history.size() - 1;
    update_setup_history_buttons();
}

void StrengthLoadPanel::restore_setup_history(size_t index)
{
    const uint64_t revision = m_session->revision;
    load_selected_object();
    if (revision != m_session->revision)
        return;
    if (index >= m_setup_history.size() || index == m_setup_history_index)
        return;
    m_restoring_history = true;
    m_setup_history_index = index;
    m_session->setup = m_setup_history[index];
    m_selected_kind = 0;
    m_selected_index = -1;
    m_current_load = -1;
    m_setup_canvas->select_item(0, -1);
    populate_from_setup();
    persist_setup(false);
    mark_stale();
    m_restoring_history = false;
    update_setup_history_buttons();
    m_status_label->SetLabel(_L("Restored the selected setup history state. Solve again to refresh results."));
}

void StrengthLoadPanel::update_setup_history_buttons()
{
    if (m_setup_undo != nullptr)
        m_setup_undo->Enable(!m_setup_history.empty() && m_setup_history_index > 0);
    if (m_setup_redo != nullptr)
        m_setup_redo->Enable(!m_setup_history.empty() && m_setup_history_index + 1 < m_setup_history.size());
}

void StrengthLoadPanel::delete_selected_operation()
{
    const uint64_t revision = m_session->revision;
    load_selected_object();
    if (revision != m_session->revision)
        return;
    // Keep pending valid edits in history before deleting a different operation.
    if (!collect_setup(true, false))
        return;
    bool changed = false;
    if (m_selected_kind == 1 && m_selected_index >= 0 &&
        size_t(m_selected_index) < m_session->setup.loads.size()) {
        m_session->setup.loads.erase(m_session->setup.loads.begin() + m_selected_index);
        m_selected_index = std::min(m_selected_index, int(m_session->setup.loads.size()) - 1);
        changed = true;
    } else if (m_selected_kind == 2 && m_selected_index >= 0 &&
               size_t(m_selected_index) < m_session->setup.preserve_regions.size()) {
        m_session->setup.preserve_regions.erase(m_session->setup.preserve_regions.begin() + m_selected_index);
        m_selected_index = std::min(m_selected_index, int(m_session->setup.preserve_regions.size()) - 1);
        changed = true;
    } else if (m_selected_kind == 3 && m_session->setup.gravity.enabled) {
        m_session->setup.gravity.enabled = false;
        changed = true;
    }
    if (!changed)
        return;
    if (m_selected_index < 0)
        m_selected_kind = 0;
    m_current_load = -1;
    populate_from_setup();
    if (m_setup_canvas != nullptr)
        m_setup_canvas->select_item(m_selected_kind, m_selected_index);
    persist_setup();
    mark_stale();
    m_status_label->SetLabel(_L("Deleted the selected setup operation. Use Undo setup to restore it."));
}

void StrengthLoadPanel::refresh_load_list()
{
    m_load_list->Clear();
    for (const SA::Load &load : m_session->setup.loads)
        m_load_list->Append(wxString::Format("%s — %s%s", wxString::FromUTF8(load.name),
            wxString::FromUTF8(SA::to_string(load.type)), load.active ? "" : " (disabled)"));
}

bool StrengthLoadPanel::save_current_load_editor()
{
    if (m_current_load < 0 || size_t(m_current_load) >= m_session->setup.loads.size())
        return true;
    SA::Load load = m_session->setup.loads[size_t(m_current_load)];
    load.name = m_load_name->GetValue().utf8_string();
    load.type = SA::LoadType(std::max(0, m_load_type->GetSelection()));
    load.active = m_load_active->GetValue();
    load.region.whole_model = m_load_whole_model->GetValue() || load.type == SA::LoadType::GlobalForce;
    bool numeric = read_vector_strict(m_load_center, load.region.center_mm);
    numeric = read_number(m_load_radius, load.region.radius_mm) && numeric;
    numeric = read_vector_strict(m_load_direction, load.direction) && numeric;
    numeric = read_number(m_load_magnitude, load.magnitude_n) && numeric;
    numeric = read_number(m_load_impact, load.impact_factor) && numeric;
    numeric = read_number(m_load_safety_factor, load.target_safety_factor) && numeric;
    load.strength_basis = SA::StrengthBasis(std::max(0, m_strength_basis->GetSelection()));
    if (numeric)
        m_session->setup.loads[size_t(m_current_load)] = std::move(load);
    return numeric;
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
        m_preserve_list->Append(wxString::Format(_L("%s — center %s"),
            wxString::FromUTF8(SA::to_string(region.shape)), vector_text(region.center_mm)));
}

void StrengthLoadPanel::refresh_study_tree()
{
    if (m_study_tree == nullptr)
        return;
    m_refreshing_tree = true;
    m_study_tree->Freeze();
    m_study_tree->DeleteAllItems();
    const wxTreeItemId root = m_study_tree->AddRoot("study-root");
    const wxTreeItemId study = m_study_tree->AppendItem(root, _L("Study 1 — Static strength"));
    m_study_tree->AppendItem(study,
        wxString::Format(_L("Material — %s"), wxString::FromUTF8(m_session->setup.material.name)),
        -1, -1, new StudyTreeItemData(4));

    const wxTreeItemId load_case = m_study_tree->AppendItem(study, _L("Load case 1"));
    if (m_session->setup.loads.empty()) {
        m_study_tree->AppendItem(load_case, _L("[needs input] Add a support and a load"));
    } else {
        for (size_t index = 0; index < m_session->setup.loads.size(); ++index) {
            const SA::Load &load = m_session->setup.loads[index];
            const wxString state = load.active ? _L("[configured]") : _L("[suppressed]");
            m_study_tree->AppendItem(load_case,
                wxString::Format("%s %s — %s", state, wxString::FromUTF8(load.name),
                                 wxString::FromUTF8(SA::to_string(load.type))),
                -1, -1, new StudyTreeItemData(1, int(index)));
        }
    }

    const wxTreeItemId preserves = m_study_tree->AppendItem(study, _L("Preserve regions"));
    if (m_session->setup.preserve_regions.empty()) {
        m_study_tree->AppendItem(preserves, _L("None"));
    } else {
        for (size_t index = 0; index < m_session->setup.preserve_regions.size(); ++index)
            m_study_tree->AppendItem(preserves, wxString::Format(_L("Preserve %zu"), index + 1),
                                     -1, -1, new StudyTreeItemData(2, int(index)));
    }

    m_study_tree->AppendItem(study,
        m_session->setup.gravity.enabled ? _L("Gravity — enabled") : _L("Gravity — suppressed"),
        -1, -1, new StudyTreeItemData(3, 0));
    m_study_tree->AppendItem(study,
        wxString::Format(_L("Print structure — %.4g%% %s"), m_session->setup.infill.background_density * 100.0,
                         wxString::FromUTF8(SA::to_string(m_session->setup.infill.background_pattern))),
        -1, -1, new StudyTreeItemData(5));
    m_study_tree->AppendItem(study,
        wxString::Format(_L("Objectives — safety factor %.4g"), m_session->setup.criteria.minimum_safety_factor),
        -1, -1, new StudyTreeItemData(6));
    m_study_tree->AppendItem(study,
        m_session->mesh.empty() ? _L("Mesh — select a model") :
            wxString::Format(_L("Mesh — %zu triangles"), m_session->mesh.indices.size()));
    const std::vector<std::string> precheck_errors = SA::validate(m_session->mesh, m_session->setup);
    const size_t precheck_issue_count = precheck_errors.size() + (m_numeric_inputs_valid ? 0 : 1);
    m_study_tree->AppendItem(study, precheck_issue_count == 0 ? _L("Pre-check — READY") :
        wxString::Format(_L("Pre-check — %zu issue(s)"), precheck_issue_count));
    const wxString result_state = m_session->result.status == SA::AnalysisStatus::NotRun ? _L("not solved") :
        wxString::FromUTF8(SA::to_string(m_session->result.status));
    m_study_tree->AppendItem(study,
        wxString::Format(_L("Results — %s%s"), result_state, m_session->stale ? _L(" (stale)") : wxString()));
    m_study_tree->ExpandAll();
    if (m_selected_kind >= 1 && m_selected_kind <= 3)
        select_study_tree_item(m_selected_kind, m_selected_index);
    m_study_tree->Thaw();
    m_refreshing_tree = false;
}

void StrengthLoadPanel::select_study_tree_item(int kind, int index)
{
    if (m_study_tree == nullptr)
        return;
    std::function<bool(const wxTreeItemId &)> visit = [&](const wxTreeItemId &item) {
        if (!item.IsOk())
            return false;
        if (auto *data = dynamic_cast<StudyTreeItemData *>(m_study_tree->GetItemData(item));
            data != nullptr && data->kind == kind && data->index == index) {
            if (m_study_tree->GetSelection() != item) {
                const bool refreshing = m_refreshing_tree;
                m_refreshing_tree = true;
                m_study_tree->SelectItem(item);
                m_refreshing_tree = refreshing;
            }
            m_study_tree->EnsureVisible(item);
            return true;
        }
        wxTreeItemIdValue cookie;
        for (wxTreeItemId child = m_study_tree->GetFirstChild(item, cookie); child.IsOk();
             child = m_study_tree->GetNextChild(item, cookie))
            if (visit(child))
                return true;
        return false;
    };
    visit(m_study_tree->GetRootItem());
}

void StrengthLoadPanel::begin_place_load(SA::LoadType type)
{
    load_selected_object();
    if (m_session->mesh.empty())
        return;
    if (type != SA::LoadType::GlobalForce) {
        m_setup_canvas->begin_load(type);
        m_status_label->SetLabel(_L("Placement active: click a model face. Dragging still orbits the view."));
        return;
    }

    SA::Load load;
    load.name = "Global load " + std::to_string(m_session->setup.loads.size() + 1);
    load.type = SA::LoadType::GlobalForce;
    load.region.whole_model = true;
    if (!m_session->mesh.vertices.empty()) {
        Vec3d minimum = m_session->mesh.vertices.front().cast<double>();
        Vec3d maximum = minimum;
        for (const Vec3f &vertex : m_session->mesh.vertices) {
            minimum = minimum.cwiseMin(vertex.cast<double>());
            maximum = maximum.cwiseMax(vertex.cast<double>());
        }
        load.region.center_mm = 0.5 * (minimum + maximum);
    }
    m_session->setup.loads.push_back(load);
    populate_from_setup();
    const int index = int(m_session->setup.loads.size()) - 1;
    select_canvas_item(1, index, false);
    persist_setup();
    mark_stale();
}

void StrengthLoadPanel::place_load_at(SA::LoadType type, const Vec3d &position_mm, const Vec3d &normal,
                                      const std::vector<size_t> &surface_triangles)
{
    SA::Load load;
    load.type = type;
    load.region.center_mm = position_mm;
    load.region.shape = SA::RegionShape::Surface;
    load.region.surface_triangles = surface_triangles;
    load.direction = normal.squaredNorm() > 1e-12 ? Vec3d(-normal.normalized()) : Vec3d(0.0, 0.0, -1.0);
    const wxString type_name = wxString::FromUTF8(SA::to_string(type));
    load.name = wxString::Format("%s %zu", type_name, m_session->setup.loads.size() + 1).utf8_string();
    if (type == SA::LoadType::Fixed) {
        load.name = "Fixed support " + std::to_string(m_session->setup.loads.size() + 1);
        load.magnitude_n = 0.0;
    }
    m_session->setup.loads.push_back(load);
    populate_from_setup();
    const int index = int(m_session->setup.loads.size()) - 1;
    select_canvas_item(1, index, false);
    persist_setup();
    mark_stale();
}

void StrengthLoadPanel::refresh_operation_panel()
{
    if (m_operation_panel == nullptr)
        return;
    SA::SphericalRegion *region = nullptr;
    SA::Load *load = nullptr;
    if (m_selected_kind == 1 && m_selected_index >= 0 &&
        size_t(m_selected_index) < m_session->setup.loads.size()) {
        load = &m_session->setup.loads[size_t(m_selected_index)];
        region = &load->region;
        m_operation_title->SetLabel(wxString::Format(_L("Load / constraint %d"), m_selected_index + 1));
        m_operation_name->ChangeValue(wxString::FromUTF8(load->name));
    } else if (m_selected_kind == 2 && m_selected_index >= 0 &&
               size_t(m_selected_index) < m_session->setup.preserve_regions.size()) {
        region = &m_session->setup.preserve_regions[size_t(m_selected_index)];
        m_operation_title->SetLabel(wxString::Format(_L("Preserve region %d"), m_selected_index + 1));
        m_operation_name->ChangeValue(wxString::Format(_L("Preserve %d"), m_selected_index + 1));
    } else if (m_selected_kind == 3) {
        m_operation_title->SetLabel(_L("Gravity load"));
    } else {
        m_operation_title->SetLabel(_L("Operation details — nothing selected"));
    }

    const bool enabled = region != nullptr;
    const std::array<wxWindow *, 7> controls{m_operation_name, m_operation_shape, m_operation_radius,
        m_operation_magnitude, m_operation_apply, m_operation_popout, m_operation_panel};
    for (wxWindow *control : controls)
        if (control != m_operation_panel)
            control->Enable(enabled);
    m_operation_delete->Enable(enabled || (m_selected_kind == 3 && m_session->setup.gravity.enabled));
    m_operation_popout->Enable(enabled || m_selected_kind == 3);
    for (int axis = 0; axis < 3; ++axis) {
        m_operation_center[axis]->Enable(enabled);
        m_operation_size[axis]->Enable(enabled);
        m_operation_axis[axis]->Enable(enabled);
        m_operation_direction[axis]->Enable(load != nullptr && load->type != SA::LoadType::Fixed);
    }
    m_operation_name->Enable(load != nullptr);
    m_operation_magnitude->Enable(load != nullptr && load->type != SA::LoadType::Fixed);
    if (!enabled)
        return;

    m_operation_shape->SetSelection(int(region->shape));
    for (int axis = 0; axis < 3; ++axis) {
        m_operation_center[axis]->ChangeValue(number(region->center_mm[axis]));
        m_operation_size[axis]->ChangeValue(number(region->size_mm[axis]));
        m_operation_axis[axis]->ChangeValue(number(region->axis[axis]));
        m_operation_direction[axis]->ChangeValue(number(load != nullptr ? load->direction[axis] : 0.0));
    }
    m_operation_radius->ChangeValue(number(region->radius_mm));
    m_operation_magnitude->ChangeValue(number(load != nullptr ? load->magnitude_n : 0.0));
}

void StrengthLoadPanel::apply_operation_panel()
{
    SA::SphericalRegion *region = nullptr;
    SA::Load *load = nullptr;
    if (m_selected_kind == 1 && m_selected_index >= 0 &&
        size_t(m_selected_index) < m_session->setup.loads.size()) {
        load = &m_session->setup.loads[size_t(m_selected_index)];
        region = &load->region;
    } else if (m_selected_kind == 2 && m_selected_index >= 0 &&
               size_t(m_selected_index) < m_session->setup.preserve_regions.size()) {
        region = &m_session->setup.preserve_regions[size_t(m_selected_index)];
    }
    if (region == nullptr)
        return;

    SA::SphericalRegion candidate = *region;
    bool valid = read_vector_strict(m_operation_center, candidate.center_mm);
    valid = read_vector_strict(m_operation_size, candidate.size_mm) && valid;
    valid = read_vector_strict(m_operation_axis, candidate.axis) && valid;
    valid = read_number(m_operation_radius, candidate.radius_mm) && valid;
    candidate.shape = SA::RegionShape(std::clamp(m_operation_shape->GetSelection(), 0, 3));
    Vec3d direction = load != nullptr ? load->direction : Vec3d::UnitZ();
    double magnitude = load != nullptr ? load->magnitude_n : 0.0;
    if (load != nullptr && load->type != SA::LoadType::Fixed) {
        valid = read_vector_strict(m_operation_direction, direction) && valid;
        valid = read_number(m_operation_magnitude, magnitude) && valid;
    }
    wxString error;
    if (!valid)
        error += _L("All numeric values must be finite.\n");
    if (candidate.shape == SA::RegionShape::Sphere && candidate.radius_mm <= 0.0)
        error += _L("Sphere radius must be greater than zero.\n");
    if (candidate.shape == SA::RegionShape::Box && (candidate.size_mm.array() <= 0.0).any())
        error += _L("Every box size must be greater than zero.\n");
    if (candidate.shape == SA::RegionShape::Cylinder &&
        (candidate.radius_mm <= 0.0 || candidate.size_mm.z() <= 0.0 || candidate.axis.squaredNorm() <= 1e-12))
        error += _L("Cylinder radius, height, and axis must be valid.\n");
    if (candidate.shape == SA::RegionShape::Surface && candidate.surface_triangles.empty())
        error += _L("Pick a model face before using Selected face.\n");
    if (load != nullptr && load->type != SA::LoadType::Fixed && direction.squaredNorm() <= 1e-12)
        error += _L("Force direction must be non-zero.\n");
    if (load != nullptr && load->type != SA::LoadType::Fixed && magnitude <= 0.0)
        error += _L("Force magnitude must be greater than zero.\n");
    if (!error.empty()) {
        wxMessageBox(error, _L("Operation details are incomplete"), wxOK | wxICON_WARNING, this);
        return;
    }

    *region = std::move(candidate);
    if (load != nullptr) {
        load->name = m_operation_name->GetValue().utf8_string();
        if (load->type != SA::LoadType::Fixed) {
            load->direction = direction.normalized();
            load->magnitude_n = magnitude;
        }
    }
    if (load != nullptr)
        load_current_load_editor(m_selected_index);
    persist_setup();
    mark_stale();
    refresh_load_list();
    refresh_preserve_list();
    refresh_study_tree();
    refresh_operation_panel();
    m_setup_canvas->Refresh();
}

bool StrengthLoadPanel::edit_load_dialog(SA::Load &load, bool creating)
{
    wxDialog dialog(this, wxID_ANY, creating ? _L("Create structural load or constraint") : _L("Edit structural load or constraint"),
                    wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER);
    const int gap = FromDIP(8);
    auto *root = new wxBoxSizer(wxVERTICAL);
    root->Add(new wxStaticText(&dialog, wxID_ANY,
        _L("Select the load type and define its target region. Direction vectors use model coordinates.")),
        0, wxEXPAND | wxALL, gap);
    auto *grid = new wxFlexGridSizer(2, gap, gap);
    grid->AddGrowableCol(1, 1);
    auto *name = new wxTextCtrl(&dialog, wxID_ANY, wxString::FromUTF8(load.name));
    auto *type = new wxChoice(&dialog, wxID_ANY, wxDefaultPosition, wxDefaultSize, load_type_names());
    type->SetSelection(int(load.type));
    auto *active = new wxCheckBox(&dialog, wxID_ANY, _L("Enabled in this study"));
    active->SetValue(load.active);
    auto *whole_model = new wxCheckBox(&dialog, wxID_ANY, _L("Apply to the whole model"));
    whole_model->SetValue(load.region.whole_model || load.type == SA::LoadType::GlobalForce);
    wxTextCtrl *center[3]{};
    wxTextCtrl *size[3]{};
    wxTextCtrl *axis[3]{};
    wxTextCtrl *direction[3]{};
    auto *shape = new wxChoice(&dialog, wxID_ANY, wxDefaultPosition, wxDefaultSize, region_shape_names());
    shape->SetSelection(int(load.region.shape));
    auto *direction_mode = new wxChoice(&dialog, wxID_ANY, wxDefaultPosition, wxDefaultSize,
        {_L("Picked surface normal / current vector"), _L("+X axis"), _L("-X axis"),
         _L("+Y axis"), _L("-Y axis"), _L("+Z axis"), _L("-Z axis")});
    direction_mode->SetSelection(0);
    auto *flip_direction = new wxButton(&dialog, wxID_ANY, _L("Flip direction"));
    auto *radius = number_input(&dialog, load.region.radius_mm);
    auto *magnitude = number_input(&dialog, load.magnitude_n);
    auto *impact = number_input(&dialog, load.impact_factor);
    auto *target = number_input(&dialog, load.target_safety_factor);
    auto *basis = new wxChoice(&dialog, wxID_ANY, wxDefaultPosition, wxDefaultSize,
                               {_L("Yield"), _L("Ultimate"), _L("Calibrated yield")});
    basis->SetSelection(int(load.strength_basis));
    add_labeled(grid, &dialog, _L("Name"), name, 1);
    add_labeled(grid, &dialog, _L("Type"), type, 1);
    add_labeled(grid, &dialog, _L("State"), active);
    add_labeled(grid, &dialog, _L("Target"), whole_model);
    add_labeled(grid, &dialog, _L("Region shape"), shape, 1);
    add_labeled(grid, &dialog, _L("Region center (mm)"), vector_editor(&dialog, center, load.region.center_mm), 1);
    add_labeled(grid, &dialog, _L("Box size XYZ / cylinder height in Z (mm)"),
                vector_editor(&dialog, size, load.region.size_mm), 1);
    add_labeled(grid, &dialog, _L("Region radius (mm)"), radius, 1);
    add_labeled(grid, &dialog, _L("Cylinder axis"), vector_editor(&dialog, axis, load.region.axis), 1);
    add_labeled(grid, &dialog, _L("Direction preset"), direction_mode, 1);
    auto *direction_row = vector_editor(&dialog, direction, load.direction);
    direction_row->Add(flip_direction, 0, wxLEFT, gap);
    add_labeled(grid, &dialog, _L("Direction / force vector"), direction_row, 1);
    add_labeled(grid, &dialog, _L("Magnitude (N)"), magnitude, 1);
    add_labeled(grid, &dialog, _L("Impact multiplier"), impact, 1);
    add_labeled(grid, &dialog, _L("Required safety factor"), target, 1);
    add_labeled(grid, &dialog, _L("Strength basis"), basis, 1);
    root->Add(grid, 1, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    root->Add(new wxStaticText(&dialog, wxID_ANY,
        _L("The arrow in the 3D view shows the applied vector. Double-click it later to reopen this dialog.")),
        0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    root->Add(dialog.CreateStdDialogButtonSizer(wxOK | wxCANCEL), 0, wxEXPAND | wxALL, gap);
    dialog.SetSizerAndFit(root);
    dialog.SetMinSize(FromDIP(wxSize(620, 480)));
    dialog.CentreOnParent();

    const auto set_direction = [direction](const Vec3d &value) {
        for (int axis = 0; axis < 3; ++axis)
            direction[axis]->ChangeValue(number(value[axis]));
    };
    direction_mode->Bind(wxEVT_CHOICE, [direction_mode, set_direction](wxCommandEvent &) {
        static const std::array<Vec3d, 6> presets{
            Vec3d::UnitX(), -Vec3d::UnitX(), Vec3d::UnitY(), -Vec3d::UnitY(), Vec3d::UnitZ(), -Vec3d::UnitZ()};
        const int selected = direction_mode->GetSelection();
        if (selected > 0 && size_t(selected - 1) < presets.size())
            set_direction(presets[size_t(selected - 1)]);
    });
    flip_direction->Bind(wxEVT_BUTTON, [direction, set_direction](wxCommandEvent &) {
        Vec3d value = Vec3d::Zero();
        if (read_vector_strict(direction, value))
            set_direction(-value);
    });
    const auto refresh_visual = [this, &load, type, whole_model, shape, center, size, radius, axis, direction] {
        if (m_setup_canvas == nullptr)
            return;
        SA::Load preview = load;
        preview.type = SA::LoadType(std::max(0, type->GetSelection()));
        preview.region.whole_model = whole_model->GetValue() || preview.type == SA::LoadType::GlobalForce;
        preview.region.shape = SA::RegionShape(std::clamp(shape->GetSelection(), 0, 3));
        preview.region.center_mm = read_vector(center, preview.region.center_mm);
        preview.region.size_mm = read_vector(size, preview.region.size_mm);
        read_number(radius, preview.region.radius_mm);
        preview.region.axis = read_vector(axis, preview.region.axis);
        preview.direction = read_vector(direction, preview.direction);
        m_setup_canvas->preview_load(preview);
    };
    type->Bind(wxEVT_CHOICE, [refresh_visual](wxCommandEvent &) { refresh_visual(); });
    shape->Bind(wxEVT_CHOICE, [refresh_visual](wxCommandEvent &) { refresh_visual(); });
    whole_model->Bind(wxEVT_CHECKBOX, [refresh_visual](wxCommandEvent &) { refresh_visual(); });
    radius->Bind(wxEVT_TEXT, [refresh_visual](wxCommandEvent &) { refresh_visual(); });
    for (int coordinate = 0; coordinate < 3; ++coordinate) {
        center[coordinate]->Bind(wxEVT_TEXT, [refresh_visual](wxCommandEvent &) { refresh_visual(); });
        size[coordinate]->Bind(wxEVT_TEXT, [refresh_visual](wxCommandEvent &) { refresh_visual(); });
        direction[coordinate]->Bind(wxEVT_TEXT, [refresh_visual](wxCommandEvent &) { refresh_visual(); });
    }
    for (wxTextCtrl *field : axis)
        field->Bind(wxEVT_TEXT, [refresh_visual](wxCommandEvent &) { refresh_visual(); });
    refresh_visual();

    while (dialog.ShowModal() == wxID_OK) {
        SA::Load candidate = load;
        candidate.name = name->GetValue().utf8_string();
        candidate.type = SA::LoadType(std::max(0, type->GetSelection()));
        candidate.active = active->GetValue();
        candidate.region.whole_model = whole_model->GetValue() || candidate.type == SA::LoadType::GlobalForce;
        candidate.region.shape = SA::RegionShape(std::clamp(shape->GetSelection(), 0, 3));
        bool numeric = true;
        numeric = read_vector_strict(center, candidate.region.center_mm) && numeric;
        numeric = read_vector_strict(size, candidate.region.size_mm) && numeric;
        numeric = read_number(radius, candidate.region.radius_mm) && numeric;
        numeric = read_vector_strict(axis, candidate.region.axis) && numeric;
        numeric = read_vector_strict(direction, candidate.direction) && numeric;
        numeric = read_number(magnitude, candidate.magnitude_n) && numeric;
        numeric = read_number(impact, candidate.impact_factor) && numeric;
        numeric = read_number(target, candidate.target_safety_factor) && numeric;
        candidate.strength_basis = SA::StrengthBasis(std::max(0, basis->GetSelection()));

        wxString error;
        if (!numeric) error += _L("All numeric fields must contain finite numbers.\n");
        if (candidate.name.empty()) error += _L("Enter a name for this study item.\n");
        if (!candidate.region.whole_model && candidate.region.shape == SA::RegionShape::Sphere && candidate.region.radius_mm <= 0.0)
            error += _L("Sphere radius must be greater than zero.\n");
        if (!candidate.region.whole_model && candidate.region.shape == SA::RegionShape::Box &&
            (candidate.region.size_mm.array() <= 0.0).any())
            error += _L("Every box size must be greater than zero.\n");
        if (!candidate.region.whole_model && candidate.region.shape == SA::RegionShape::Cylinder &&
            (candidate.region.radius_mm <= 0.0 || candidate.region.size_mm.z() <= 0.0 ||
             candidate.region.axis.squaredNorm() <= 1e-12))
            error += _L("Cylinder radius, height, and axis must be valid.\n");
        if (!candidate.region.whole_model && candidate.region.shape == SA::RegionShape::Surface &&
            candidate.region.surface_triangles.empty())
            error += _L("A selected-face region needs a face picked in the viewport.\n");
        if (candidate.type != SA::LoadType::Fixed && candidate.direction.squaredNorm() <= 1e-12)
            error += _L("Load direction must be non-zero.\n");
        if (candidate.type != SA::LoadType::Fixed && candidate.magnitude_n <= 0.0)
            error += _L("Load magnitude must be greater than zero.\n");
        if (candidate.type == SA::LoadType::ImpactForce && candidate.impact_factor < 1.0)
            error += _L("Impact multiplier must be at least 1.\n");
        if (candidate.target_safety_factor <= 0.0)
            error += _L("Required safety factor must be greater than zero.\n");
        if (!error.empty()) {
            wxMessageBox(error, _L("Load properties are incomplete"), wxOK | wxICON_WARNING, &dialog);
            continue;
        }
        if (candidate.type != SA::LoadType::Fixed)
            candidate.direction.normalize();
        load = std::move(candidate);
        if (m_setup_canvas != nullptr)
            m_setup_canvas->clear_preview();
        return true;
    }
    if (m_setup_canvas != nullptr)
        m_setup_canvas->clear_preview();
    return false;
}

void StrengthLoadPanel::place_preserve_at(const Vec3d &position_mm, const std::vector<size_t> &surface_triangles)
{
    SA::SphericalRegion region;
    region.center_mm = position_mm;
    region.shape = SA::RegionShape::Box;
    region.surface_triangles = surface_triangles;
    m_session->setup.preserve_regions.push_back(region);
    populate_from_setup();
    const int index = int(m_session->setup.preserve_regions.size()) - 1;
    select_canvas_item(2, index, false);
    persist_setup();
    mark_stale();
}

bool StrengthLoadPanel::edit_preserve_dialog(SA::SphericalRegion &region, bool creating)
{
    wxDialog dialog(this, wxID_ANY, creating ? _L("Create preserve region") : _L("Edit preserve region"),
                    wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER);
    const int gap = FromDIP(8);
    auto *root = new wxBoxSizer(wxVERTICAL);
    root->Add(new wxStaticText(&dialog, wxID_ANY,
        _L("Preserve regions protect load interfaces, mounting features, and other geometry from dense-region replacement.")),
        0, wxEXPAND | wxALL, gap);
    auto *grid = new wxFlexGridSizer(2, gap, gap);
    grid->AddGrowableCol(1, 1);
    wxTextCtrl *center[3]{};
    wxTextCtrl *size[3]{};
    wxTextCtrl *axis[3]{};
    auto *shape = new wxChoice(&dialog, wxID_ANY, wxDefaultPosition, wxDefaultSize, region_shape_names());
    shape->SetSelection(int(region.shape));
    auto *radius = number_input(&dialog, region.radius_mm);
    add_labeled(grid, &dialog, _L("Shape"), shape, 1);
    add_labeled(grid, &dialog, _L("Center (mm)"), vector_editor(&dialog, center, region.center_mm), 1);
    add_labeled(grid, &dialog, _L("Box size XYZ / cylinder height in Z (mm)"),
                vector_editor(&dialog, size, region.size_mm), 1);
    add_labeled(grid, &dialog, _L("Radius (mm)"), radius, 1);
    add_labeled(grid, &dialog, _L("Cylinder axis"), vector_editor(&dialog, axis, region.axis), 1);
    root->Add(grid, 1, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    root->Add(dialog.CreateStdDialogButtonSizer(wxOK | wxCANCEL), 0, wxEXPAND | wxALL, gap);
    dialog.SetSizerAndFit(root);
    dialog.SetMinSize(FromDIP(wxSize(560, 230)));
    dialog.CentreOnParent();
    const auto refresh_visual = [this, &region, shape, center, size, radius, axis] {
        if (m_setup_canvas == nullptr)
            return;
        SA::SphericalRegion preview = region;
        preview.shape = SA::RegionShape(std::clamp(shape->GetSelection(), 0, 3));
        preview.center_mm = read_vector(center, preview.center_mm);
        preview.size_mm = read_vector(size, preview.size_mm);
        read_number(radius, preview.radius_mm);
        preview.axis = read_vector(axis, preview.axis);
        preview.whole_model = false;
        m_setup_canvas->preview_preserve(preview);
    };
    radius->Bind(wxEVT_TEXT, [refresh_visual](wxCommandEvent &) { refresh_visual(); });
    shape->Bind(wxEVT_CHOICE, [refresh_visual](wxCommandEvent &) { refresh_visual(); });
    for (wxTextCtrl *field : center)
        field->Bind(wxEVT_TEXT, [refresh_visual](wxCommandEvent &) { refresh_visual(); });
    for (wxTextCtrl *field : size)
        field->Bind(wxEVT_TEXT, [refresh_visual](wxCommandEvent &) { refresh_visual(); });
    for (wxTextCtrl *field : axis)
        field->Bind(wxEVT_TEXT, [refresh_visual](wxCommandEvent &) { refresh_visual(); });
    refresh_visual();
    while (dialog.ShowModal() == wxID_OK) {
        SA::SphericalRegion candidate = region;
        candidate.shape = SA::RegionShape(std::clamp(shape->GetSelection(), 0, 3));
        const bool numeric = read_vector_strict(center, candidate.center_mm) &&
            read_vector_strict(size, candidate.size_mm) && read_number(radius, candidate.radius_mm) &&
            read_vector_strict(axis, candidate.axis);
        const bool shape_valid =
            (candidate.shape == SA::RegionShape::Sphere && candidate.radius_mm > 0.0) ||
            (candidate.shape == SA::RegionShape::Box && (candidate.size_mm.array() > 0.0).all()) ||
            (candidate.shape == SA::RegionShape::Cylinder && candidate.radius_mm > 0.0 &&
             candidate.size_mm.z() > 0.0 && candidate.axis.squaredNorm() > 1e-12) ||
            (candidate.shape == SA::RegionShape::Surface && !candidate.surface_triangles.empty());
        if (!numeric || !shape_valid) {
            wxMessageBox(_L("Enter finite dimensions for the selected shape. Face regions require a face picked in the viewport."), _L("Invalid preserve region"),
                         wxOK | wxICON_WARNING, &dialog);
            continue;
        }
        candidate.whole_model = false;
        region = candidate;
        if (m_setup_canvas != nullptr)
            m_setup_canvas->clear_preview();
        return true;
    }
    if (m_setup_canvas != nullptr)
        m_setup_canvas->clear_preview();
    return false;
}

void StrengthLoadPanel::edit_gravity_dialog()
{
    wxDialog dialog(this, wxID_ANY, _L("Gravity load"), wxDefaultPosition, wxDefaultSize,
                    wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER);
    const int gap = FromDIP(8);
    auto *root = new wxBoxSizer(wxVERTICAL);
    auto *enabled = new wxCheckBox(&dialog, wxID_ANY, _L("Include self-weight from the estimated printed mass"));
    enabled->SetValue(m_session->setup.gravity.enabled);
    root->Add(enabled, 0, wxEXPAND | wxALL, gap);
    auto *grid = new wxFlexGridSizer(2, gap, gap);
    grid->AddGrowableCol(1, 1);
    wxTextCtrl *acceleration[3]{};
    add_labeled(grid, &dialog, _L("Acceleration (m/s²)"),
                vector_editor(&dialog, acceleration, m_session->setup.gravity.acceleration_m_s2), 1);
    root->Add(grid, 1, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    root->Add(new wxStaticText(&dialog, wxID_ANY,
        _L("Earth gravity is normally (0, 0, -9.80665). The 3D arrow previews the selected direction.")),
        0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    root->Add(dialog.CreateStdDialogButtonSizer(wxOK | wxCANCEL), 0, wxEXPAND | wxALL, gap);
    dialog.SetSizerAndFit(root);
    dialog.SetMinSize(FromDIP(wxSize(570, 250)));
    dialog.CentreOnParent();
    while (dialog.ShowModal() == wxID_OK) {
        Vec3d value = m_session->setup.gravity.acceleration_m_s2;
        const bool numeric = read_vector_strict(acceleration, value);
        if (!numeric || (enabled->GetValue() && value.squaredNorm() <= 1e-12)) {
            wxMessageBox(_L("Acceleration must contain finite numbers, and enabled gravity requires a non-zero vector."), _L("Invalid gravity load"),
                         wxOK | wxICON_WARNING, &dialog);
            continue;
        }
        m_session->setup.gravity.enabled = enabled->GetValue();
        m_session->setup.gravity.acceleration_m_s2 = value;
        populate_from_setup();
        m_setup_canvas->select_item(3, 0);
        mark_stale();
        return;
    }
}

void StrengthLoadPanel::edit_material_dialog()
{
    wxDialog dialog(this, wxID_ANY, _L("Study material and print direction"), wxDefaultPosition, wxDefaultSize,
                    wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER);
    const int gap = FromDIP(8);
    auto *root = new wxBoxSizer(wxVERTICAL);
    auto *content = new wxScrolledWindow(&dialog, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL | wxTAB_TRAVERSAL);
    content->SetScrollRate(0, FromDIP(12));
    auto *content_sizer = new wxBoxSizer(wxVERTICAL);
    content_sizer->Add(new wxStaticText(content, wxID_ANY,
        _L("Assign anisotropic printed-material properties. Validate critical values with supplier data and printed coupons.")),
        0, wxEXPAND | wxALL, gap);

    auto *choice_row = new wxBoxSizer(wxHORIZONTAL);
    auto *material_choice = new wxChoice(content, wxID_ANY);
    for (const SA::Material &material : SA::builtin_materials())
        material_choice->Append(wxString::FromUTF8(material.name));
    material_choice->Append(_L("Custom material"));
    int selected = int(SA::builtin_materials().size());
    for (size_t index = 0; index < SA::builtin_materials().size(); ++index)
        if (SA::builtin_materials()[index].key == m_session->setup.material.key) selected = int(index);
    material_choice->SetSelection(selected);
    add_labeled(choice_row, content, _L("Material"), material_choice, 1);
    content_sizer->Add(choice_row, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);

    static const std::array<wxString, 12> labels{
        _L("Density (kg/m³)"), _L("Elastic modulus XY (GPa)"), _L("Elastic modulus Z (GPa)"), _L("Poisson ratio XY"),
        _L("Shear modulus XY (GPa)"), _L("Shear modulus XZ (GPa)"), _L("Yield strength XY (MPa)"),
        _L("Yield strength Z (MPa)"), _L("Ultimate strength XY (MPa)"), _L("Ultimate strength Z (MPa)"),
        _L("Shear strength XY (MPa)"), _L("Shear strength XZ (MPa)")};
    auto *properties = new wxFlexGridSizer(2, gap, gap);
    properties->AddGrowableCol(1, 1);
    std::array<wxTextCtrl *, 12> fields{};
    const SA::Material &current = m_session->setup.material;
    const std::array<double, 12> initial{current.density_kg_m3, current.elastic_modulus_xy_pa / 1e9,
        current.elastic_modulus_z_pa / 1e9, current.poisson_xy, current.shear_modulus_xy_pa / 1e9,
        current.shear_modulus_xz_pa / 1e9, current.yield_strength_xy_pa / 1e6, current.yield_strength_z_pa / 1e6,
        current.ultimate_strength_xy_pa / 1e6, current.ultimate_strength_z_pa / 1e6,
        current.shear_strength_xy_pa / 1e6, current.shear_strength_xz_pa / 1e6};
    for (size_t index = 0; index < fields.size(); ++index) {
        fields[index] = number_input(content, initial[index], 140);
        add_labeled(properties, content, labels[index], fields[index], 1);
    }
    content_sizer->Add(properties, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);

    static const std::array<wxString, 5> calibration_labels{
        _L("Modulus XY calibration"), _L("Modulus Z calibration"), _L("Strength XY calibration"),
        _L("Strength Z calibration"), _L("Shear calibration")};
    auto *calibration_grid = new wxFlexGridSizer(2, gap, gap);
    calibration_grid->AddGrowableCol(1, 1);
    std::array<wxTextCtrl *, 5> calibration{};
    const std::array<double, 5> calibration_values{current.calibration.modulus_xy_scale,
        current.calibration.modulus_z_scale, current.calibration.strength_xy_scale,
        current.calibration.strength_z_scale, current.calibration.shear_scale};
    for (size_t index = 0; index < calibration.size(); ++index) {
        calibration[index] = number_input(content, calibration_values[index], 140);
        add_labeled(calibration_grid, content, calibration_labels[index], calibration[index], 1);
    }
    auto *source = new wxTextCtrl(content, wxID_ANY, wxString::FromUTF8(current.calibration.source));
    add_labeled(calibration_grid, content, _L("Calibration source / coupon"), source, 1);
    wxTextCtrl *layer_axis[3]{};
    add_labeled(calibration_grid, content, _L("Layer-normal axis"),
                vector_editor(content, layer_axis, m_session->setup.print_layer_axis), 1);
    for (wxTextCtrl *field : layer_axis)
        field->Enable(!m_session->setup.follow_prepare_orientation);
    content_sizer->Add(calibration_grid, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    content->SetSizer(content_sizer);
    root->Add(content, 1, wxEXPAND);
    root->Add(dialog.CreateStdDialogButtonSizer(wxOK | wxCANCEL), 0, wxEXPAND | wxALL, gap);
    dialog.SetSizer(root);
    dialog.SetSize(FromDIP(wxSize(690, 760)));
    dialog.CentreOnParent();

    material_choice->Bind(wxEVT_CHOICE, [material_choice, &fields](wxCommandEvent &) {
        const int index = material_choice->GetSelection();
        if (index < 0 || size_t(index) >= SA::builtin_materials().size())
            return;
        const SA::Material &material = SA::builtin_materials()[size_t(index)];
        const std::array<double, 12> values{material.density_kg_m3, material.elastic_modulus_xy_pa / 1e9,
            material.elastic_modulus_z_pa / 1e9, material.poisson_xy, material.shear_modulus_xy_pa / 1e9,
            material.shear_modulus_xz_pa / 1e9, material.yield_strength_xy_pa / 1e6, material.yield_strength_z_pa / 1e6,
            material.ultimate_strength_xy_pa / 1e6, material.ultimate_strength_z_pa / 1e6,
            material.shear_strength_xy_pa / 1e6, material.shear_strength_xz_pa / 1e6};
        for (size_t field = 0; field < fields.size(); ++field)
            fields[field]->ChangeValue(number(values[field]));
    });

    while (dialog.ShowModal() == wxID_OK) {
        const int material_index = material_choice->GetSelection();
        SA::Material candidate = material_index >= 0 && size_t(material_index) < SA::builtin_materials().size() ?
            SA::builtin_materials()[size_t(material_index)] : m_session->setup.material;
        std::array<double, 12> values{};
        bool numeric = true;
        for (size_t index = 0; index < fields.size(); ++index)
            numeric = read_number(fields[index], values[index]) && numeric;
        candidate.density_kg_m3 = values[0];
        candidate.elastic_modulus_xy_pa = values[1] * 1e9;
        candidate.elastic_modulus_z_pa = values[2] * 1e9;
        candidate.poisson_xy = values[3];
        candidate.shear_modulus_xy_pa = values[4] * 1e9;
        candidate.shear_modulus_xz_pa = values[5] * 1e9;
        candidate.yield_strength_xy_pa = values[6] * 1e6;
        candidate.yield_strength_z_pa = values[7] * 1e6;
        candidate.ultimate_strength_xy_pa = values[8] * 1e6;
        candidate.ultimate_strength_z_pa = values[9] * 1e6;
        candidate.shear_strength_xy_pa = values[10] * 1e6;
        candidate.shear_strength_xz_pa = values[11] * 1e6;
        std::array<double, 5> scales{};
        for (size_t index = 0; index < calibration.size(); ++index)
            numeric = read_number(calibration[index], scales[index]) && numeric;
        candidate.calibration.modulus_xy_scale = scales[0];
        candidate.calibration.modulus_z_scale = scales[1];
        candidate.calibration.strength_xy_scale = scales[2];
        candidate.calibration.strength_z_scale = scales[3];
        candidate.calibration.shear_scale = scales[4];
        candidate.calibration.source = source->GetValue().utf8_string();
        Vec3d axis = m_session->setup.print_layer_axis;
        numeric = read_vector_strict(layer_axis, axis) && numeric;
        if (material_index < 0 || size_t(material_index) >= SA::builtin_materials().size()) {
            candidate.key = "custom";
            candidate.name = "Custom material";
            candidate.provenance = "User-entered material properties; verify against a filament datasheet and printed coupons.";
        } else if (!material_properties_match(candidate, SA::builtin_materials()[size_t(material_index)])) {
            candidate.key = "custom";
            candidate.name = SA::builtin_materials()[size_t(material_index)].name + " (custom)";
            candidate.provenance = "User-edited bundled estimate; verify against a filament datasheet and printed coupons.";
        }
        wxString error;
        if (!numeric) error += _L("All property and calibration fields must be finite numbers.\n");
        for (const std::string &item : candidate.validate())
            error += wxString::FromUTF8(item) + "\n";
        if (axis.squaredNorm() <= 1e-12)
            error += _L("Layer-normal axis must be non-zero.\n");
        if (!error.empty()) {
            wxMessageBox(error, _L("Material properties are incomplete"), wxOK | wxICON_WARNING, &dialog);
            continue;
        }
        m_session->setup.material = std::move(candidate);
        m_session->setup.print_layer_axis = axis.normalized();
        populate_from_setup();
        mark_stale();
        return;
    }
}

void StrengthLoadPanel::edit_infill_dialog()
{
    wxDialog dialog(this, wxID_ANY, _L("Print structure and dense-region strategy"), wxDefaultPosition, wxDefaultSize,
                    wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER);
    const int gap = FromDIP(8);
    auto *root = new wxBoxSizer(wxVERTICAL);
    root->Add(new wxStaticText(&dialog, wxID_ANY,
        _L("Define the baseline print structure. In Simulation, use this stress threshold to size the stress-directed region, then refine its volume.")),
        0, wxEXPAND | wxALL, gap);
    auto *grid = new wxFlexGridSizer(2, gap, gap);
    grid->AddGrowableCol(1, 1);
    auto *background_pattern = new wxChoice(&dialog, wxID_ANY, wxDefaultPosition, wxDefaultSize, infill_names());
    auto *dense_pattern = new wxChoice(&dialog, wxID_ANY, wxDefaultPosition, wxDefaultSize, infill_names());
    background_pattern->SetSelection(int(m_session->setup.infill.background_pattern));
    dense_pattern->SetSelection(int(m_session->setup.infill.dense_pattern));
    auto *background_density = number_input(&dialog, m_session->setup.infill.background_density * 100.0);
    auto *dense_density = number_input(&dialog, m_session->setup.infill.dense_density * 100.0);
    auto *threshold = number_input(&dialog, m_session->setup.infill.dense_stress_threshold * 100.0);
    add_labeled(grid, &dialog, _L("Background pattern"), background_pattern, 1);
    add_labeled(grid, &dialog, _L("Background density (%)"), background_density, 1);
    add_labeled(grid, &dialog, _L("Dense-region pattern"), dense_pattern, 1);
    add_labeled(grid, &dialog, _L("Dense-region density (%)"), dense_density, 1);
    add_labeled(grid, &dialog, _L("Stress threshold (% of peak)"), threshold, 1);
    root->Add(grid, 1, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    root->Add(dialog.CreateStdDialogButtonSizer(wxOK | wxCANCEL), 0, wxEXPAND | wxALL, gap);
    dialog.SetSizerAndFit(root);
    dialog.SetMinSize(FromDIP(wxSize(590, 350)));
    dialog.CentreOnParent();
    while (dialog.ShowModal() == wxID_OK) {
        double background = 0.0, dense = 0.0, stress = 0.0;
        const bool numeric = read_number(background_density, background) && read_number(dense_density, dense) &&
                             read_number(threshold, stress);
        if (!numeric || background <= 0.0 || background > 100.0 || dense <= 0.0 || dense > 100.0 ||
            stress <= 0.0 || stress > 100.0 || dense < background) {
            wxMessageBox(_L("Densities and threshold must be within (0, 100], and dense density must not be below the background."),
                         _L("Invalid print structure"), wxOK | wxICON_WARNING, &dialog);
            continue;
        }
        SA::Setup candidate = m_session->setup;
        candidate.infill.background_pattern = SA::InfillPattern(std::max(0, background_pattern->GetSelection()));
        candidate.infill.dense_pattern = SA::InfillPattern(std::max(0, dense_pattern->GetSelection()));
        candidate.infill.background_density = background / 100.0;
        candidate.infill.dense_density = dense / 100.0;
        candidate.infill.dense_stress_threshold = stress / 100.0;
        const std::vector<std::string> validation = SA::validate(m_session->mesh, candidate);
        const auto weaker = std::find_if(validation.begin(), validation.end(), [](const std::string &message) {
            return message.find("must not be weaker or less stiff") != std::string::npos;
        });
        if (weaker != validation.end()) {
            wxMessageBox(wxString::FromUTF8(*weaker), _L("Invalid print structure"),
                         wxOK | wxICON_WARNING, &dialog);
            continue;
        }
        m_session->setup.infill = candidate.infill;
        populate_from_setup();
        mark_stale();
        return;
    }
}

void StrengthLoadPanel::edit_criteria_dialog()
{
    wxDialog dialog(this, wxID_ANY, _L("Optimization objectives"), wxDefaultPosition, wxDefaultSize,
                    wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER);
    const int gap = FromDIP(8);
    auto *root = new wxBoxSizer(wxVERTICAL);
    root->Add(new wxStaticText(&dialog, wxID_ANY,
        _L("Set feasibility limits first, then tune the relative weights used to rank printable candidates.")),
        0, wxEXPAND | wxALL, gap);
    auto *grid = new wxFlexGridSizer(2, gap, gap);
    grid->AddGrowableCol(1, 1);
    auto *minimum_sf = number_input(&dialog, m_session->setup.criteria.minimum_safety_factor);
    auto *maximum_displacement = number_input(&dialog, m_session->setup.criteria.maximum_displacement_mm);
    std::array<wxTextCtrl *, 4> weights{
        number_input(&dialog, m_session->setup.criteria.mass_weight),
        number_input(&dialog, m_session->setup.criteria.stiffness_weight),
        number_input(&dialog, m_session->setup.criteria.support_weight),
        number_input(&dialog, m_session->setup.criteria.print_time_weight)};
    add_labeled(grid, &dialog, _L("Minimum safety factor"), minimum_sf, 1);
    add_labeled(grid, &dialog, _L("Maximum displacement (mm; 0 disables)"), maximum_displacement, 1);
    static const std::array<wxString, 4> labels{
        _L("Mass weight"), _L("Stiffness weight"), _L("Support weight"), _L("Print-time weight")};
    for (size_t index = 0; index < weights.size(); ++index)
        add_labeled(grid, &dialog, labels[index], weights[index], 1);
    root->Add(grid, 1, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    root->Add(dialog.CreateStdDialogButtonSizer(wxOK | wxCANCEL), 0, wxEXPAND | wxALL, gap);
    dialog.SetSizerAndFit(root);
    dialog.SetMinSize(FromDIP(wxSize(610, 390)));
    dialog.CentreOnParent();
    while (dialog.ShowModal() == wxID_OK) {
        double sf = 0.0, displacement = 0.0;
        std::array<double, 4> values{};
        bool numeric = read_number(minimum_sf, sf) && read_number(maximum_displacement, displacement);
        for (size_t index = 0; index < weights.size(); ++index)
            numeric = read_number(weights[index], values[index]) && numeric;
        if (!numeric || sf <= 0.0 || displacement < 0.0 ||
            std::any_of(values.begin(), values.end(), [](double value) { return value < 0.0; }) ||
            std::accumulate(values.begin(), values.end(), 0.0) <= 0.0) {
            wxMessageBox(_L("Safety factor must be positive, displacement and weights cannot be negative, and at least one weight must be positive."),
                         _L("Invalid optimization objectives"), wxOK | wxICON_WARNING, &dialog);
            continue;
        }
        m_session->setup.criteria.minimum_safety_factor = sf;
        m_session->setup.criteria.maximum_displacement_mm = displacement;
        m_session->setup.criteria.mass_weight = values[0];
        m_session->setup.criteria.stiffness_weight = values[1];
        m_session->setup.criteria.support_weight = values[2];
        m_session->setup.criteria.print_time_weight = values[3];
        populate_from_setup();
        mark_stale();
        return;
    }
}

void StrengthLoadPanel::select_canvas_item(int kind, int index, bool edit)
{
    if (!save_current_load_editor()) {
        m_status_label->SetLabel(_L("Correct the invalid numeric load fields before selecting another operation."));
        return;
    }
    m_selected_kind = kind;
    m_selected_index = index;
    select_study_tree_item(kind, index);
    if (m_setup_canvas != nullptr && kind >= 1 && kind <= 3)
        m_setup_canvas->select_item(kind, index);
    if (kind == 1 && index >= 0 && size_t(index) < m_session->setup.loads.size()) {
        save_current_load_editor();
        load_current_load_editor(index);
        if (edit) {
            SA::Load candidate = m_session->setup.loads[size_t(index)];
            if (edit_load_dialog(candidate, false)) {
                m_session->setup.loads[size_t(index)] = candidate;
                populate_from_setup();
                load_current_load_editor(index);
                m_setup_canvas->select_item(1, index);
                mark_stale();
            }
        }
    } else if (kind == 2 && index >= 0 && size_t(index) < m_session->setup.preserve_regions.size()) {
        m_preserve_list->SetSelection(index);
        if (edit) {
            SA::SphericalRegion candidate = m_session->setup.preserve_regions[size_t(index)];
            if (edit_preserve_dialog(candidate, false)) {
                m_session->setup.preserve_regions[size_t(index)] = candidate;
                populate_from_setup();
                m_preserve_list->SetSelection(index);
                m_setup_canvas->select_item(2, index);
                mark_stale();
            }
        }
    } else if (kind == 3 && edit) {
        edit_gravity_dialog();
    }
    refresh_operation_panel();
}

bool StrengthLoadPanel::run_precheck(bool show_success)
{
    load_selected_object();
    if (m_session->mesh.empty()) {
        set_precheck_state(-1);
        return false;
    }
    if (!collect_setup(true)) {
        set_precheck_state(-1);
        m_status_label->SetLabel(_L("Pre-check found setup issues. Correct the highlighted study inputs before solving."));
        refresh_study_tree();
        return false;
    }
    const std::vector<std::string> errors = SA::validate(m_session->mesh, m_session->setup);
    if (!errors.empty()) {
        set_precheck_state(-1);
        m_status_label->SetLabel(wxString::Format(_L("Pre-check found %zu issue(s)."), errors.size()));
        refresh_study_tree();
        return false;
    }
    set_precheck_state(1);
    m_status_label->SetLabel(_L("Pre-check READY — material, mesh, supports, loads, and objectives are valid."));
    refresh_study_tree();
    if (show_success)
        wxMessageBox(_L("Pre-check is ready. The study has a valid mesh, material, support, and applied load."),
                     _L("Strength study pre-check"), wxOK | wxICON_INFORMATION, this);
    return true;
}

void StrengthLoadPanel::refresh_precheck_state()
{
    // Use the same validation as Solve, without a dialog or any mutation of the study.
    // A partially typed number must not leave the last accepted setup looking ready.
    set_precheck_state(m_numeric_inputs_valid && !m_session->mesh.empty() &&
        SA::validate(m_session->mesh, m_session->setup).empty() ? 1 : -1);
}

void StrengthLoadPanel::set_precheck_state(int state)
{
    if (m_precheck_button == nullptr)
        return;
    m_precheck_button->SetStyle(ButtonStyle::Regular, ButtonType::Compact);
    if (state > 0) {
        m_precheck_button->SetLabel(_L("Pre-check: READY"));
        m_precheck_button->SetBackgroundColor(StateColor(
            std::pair<wxColour, int>(wxColour(238, 238, 238), StateColor::Disabled),
            std::pair<wxColour, int>(wxColour(224, 244, 229), StateColor::Normal)));
        m_precheck_button->SetTextColor(StateColor(
            std::pair<wxColour, int>(wxColour(144, 144, 144), StateColor::Disabled),
            std::pair<wxColour, int>(wxColour(33, 104, 55), StateColor::Normal)));
        m_precheck_button->SetBorderColor(StateColor(wxColour(33, 104, 55)));
        m_precheck_button->SetToolTip(_L("Pre-check passed for the current setup."));
    } else if (state < 0) {
        m_precheck_button->SetLabel(_L("Pre-check: issues"));
        m_precheck_button->SetBackgroundColor(StateColor(
            std::pair<wxColour, int>(wxColour(238, 238, 238), StateColor::Disabled),
            std::pair<wxColour, int>(wxColour(255, 236, 207), StateColor::Normal)));
        m_precheck_button->SetTextColor(StateColor(
            std::pair<wxColour, int>(wxColour(144, 144, 144), StateColor::Disabled),
            std::pair<wxColour, int>(wxColour(139, 75, 15), StateColor::Normal)));
        m_precheck_button->SetBorderColor(StateColor(wxColour(139, 75, 15)));
        m_precheck_button->SetToolTip(_L("Pre-check found one or more setup issues."));
    } else {
        m_precheck_button->SetLabel(_L("Pre-check"));
        m_precheck_button->SetToolTip(_L("Validate the mesh, material, constraints, loads, and objectives."));
    }
    m_precheck_button->Refresh();
    Layout();
}

void StrengthLoadPanel::run_analysis()
{
    if (m_analysis_running)
        return;
    load_selected_object();
    if (!run_precheck(false))
        return;
    persist_setup();
    if (m_worker.joinable())
        m_worker.join();
    const SA::Setup setup = m_session->setup;
    const indexed_triangle_set mesh = m_session->mesh;
    const uint64_t revision = m_session->revision;
    m_cancel = false;
    m_analysis_running = true;
    m_run_button->Disable();
    m_precheck_button->Disable();
    m_cancel_button->Enable();
    m_status_label->SetLabel(_L("Running offline linear-static engineering estimate…"));
    m_worker = std::thread([this, setup, mesh, revision] {
        SA::Result result = SA::analyze(mesh, setup, [this] { return m_cancel.load(); });
        std::shared_ptr<const SA::DenseRegionPreviewProfile> profile;
        if (result.succeeded()) {
            profile = std::make_shared<SA::DenseRegionPreviewProfile>(
                SA::build_dense_region_preview_profile(mesh, result, [this] { return m_cancel.load(); }));
            if (m_cancel.load()) {
                result.status = SA::AnalysisStatus::Cancelled;
                result.message = "Strength preview preparation was cancelled.";
                profile.reset();
            }
        }
        {
            std::lock_guard<std::mutex> lock(m_pending_mutex);
            m_pending_result = std::move(result);
            m_pending_profile = std::move(profile);
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
    m_analysis_running = false;
    SA::Result result;
    std::shared_ptr<const SA::DenseRegionPreviewProfile> profile;
    uint64_t revision = 0;
    {
        std::lock_guard<std::mutex> lock(m_pending_mutex);
        result = std::move(m_pending_result);
        profile = std::move(m_pending_profile);
        revision = m_pending_revision;
    }
    m_run_button->Enable();
    m_precheck_button->Enable();
    m_cancel_button->Disable();
    load_selected_object();
    if (revision != m_session->revision) {
        m_status_label->SetLabel(_L("Analysis finished, but inputs changed while it was running; results were discarded."));
        if (m_result_callback)
            m_result_callback();
        return;
    }
    m_session->result = std::move(result);
    m_session->dense_profile = std::move(profile);
    ++m_session->solved_revision;
    m_session->stale = false;
    const SA::Result &stored = m_session->result;
    if (stored.succeeded()) {
        m_session->solved_mesh = m_session->mesh;
        m_session->solved_setup = m_session->setup;
    } else {
        m_session->solved_mesh = {};
        m_session->solved_setup = SA::Setup{};
    }
    m_status_label->SetLabel(wxString::Format("%s — %s", wxString::FromUTF8(SA::to_string(stored.status)),
                                              wxString::FromUTF8(stored.message)));
    const bool ready = stored.succeeded();
    m_dense_button->Enable(ready);
    m_orientation_button->Enable(ready && !stored.orientation_recommendations.empty());
    m_settings_button->Enable(ready && !stored.print_settings_candidates.empty() &&
                              stored.print_settings_candidates.front().feasible);
    refresh_study_tree();
    if (m_result_callback)
        m_result_callback();
}

void StrengthLoadPanel::preview_dense_region()
{
    load_selected_object();
    if (!m_session->stale && m_session->result.succeeded() && m_preview_callback)
        m_preview_callback();
}

bool StrengthLoadPanel::create_dense_modifier_from_preview(const SA::DenseRegionPreview &preview)
{
    const ObjectID object_id = m_session->object_id;
    const uint64_t revision = m_session->revision;
    load_selected_object();
    if (object_id != m_session->object_id || revision != m_session->revision ||
        m_session->stale || !m_session->result.succeeded() || !preview.applicable() ||
        !meshes_equal(m_session->mesh, m_session->solved_mesh) || preview.modifier_mesh.empty())
        return false;
    for (const Vec3f &vertex : preview.modifier_mesh.vertices)
        if (!vertex.allFinite())
            return false;
    for (const Vec3i32 &triangle : preview.modifier_mesh.indices)
        if (!valid_triangle(triangle, preview.modifier_mesh.vertices.size()))
            return false;
    const int index = m_session->object_index;
    if (index < 0 || size_t(index) >= m_plater->model().objects.size())
        return false;
    const SA::InfillSettings &infill = m_session->solved_setup.infill;
    m_plater->take_snapshot("Apply previewed strength dense-region modifier");
    ModelObject *object = m_plater->model().objects[size_t(index)];
    ModelVolume *volume = object->add_volume(TriangleMesh(preview.modifier_mesh), ModelVolumeType::PARAMETER_MODIFIER, false);
    volume->name = STRENGTH_MODIFIER_NAME;
    // The preview mesh uses the same object coordinates as ModelObject::raw_mesh().
    // Keep them intact; auto-centering would require an additional offset transform.
    volume->set_transformation(Geometry::Transformation());
    volume->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(infill.dense_density * 100.0));
    volume->config.set_key_value("sparse_infill_pattern",
        new ConfigOptionEnum<Slic3r::InfillPattern>(print_pattern(infill.dense_pattern)));
    volume->config.set_key_value("strength_analysis_modifier", new ConfigOptionBool(true));
    object->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(infill.background_density * 100.0));
    object->config.set_key_value("sparse_infill_pattern",
        new ConfigOptionEnum<Slic3r::InfillPattern>(print_pattern(infill.background_pattern)));
    // Add the replacement first: deleting down to one volume would collapse the part's
    // transform into its instances and change the object coordinates used by this preview.
    for (size_t volume_index = object->volumes.size(); volume_index-- > 0;) {
        const ModelVolume *candidate = object->volumes[volume_index];
        if (candidate != volume && is_strength_dense_modifier(candidate))
            object->delete_volume(volume_index);
    }
    m_plater->changed_object(index);
    m_plater->set_plater_dirty(true);
    m_remove_dense_button->Enable();
    m_status_label->SetLabel(wxString::Format(
        _L("Applied a native dense-infill modifier at approximately %.3g%% of model volume: %.3g%% %s inside, %.3g%% %s background. "
           "Other user modifiers can override these settings. Slice and validate the printed design separately."),
        preview.estimated_volume_fraction * 100.0, infill.dense_density * 100.0,
        wxString::FromUTF8(SA::to_string(infill.dense_pattern)), infill.background_density * 100.0,
        wxString::FromUTF8(SA::to_string(infill.background_pattern))));
    return true;
}

void StrengthLoadPanel::remove_dense_modifier()
{
    const ObjectID object_id = m_session->object_id;
    load_selected_object();
    if (object_id != m_session->object_id)
        return;
    const int index = m_session->object_index;
    if (index < 0 || size_t(index) >= m_plater->model().objects.size())
        return;
    ModelObject *object = m_plater->model().objects[size_t(index)];
    std::vector<size_t> modifier_indices;
    for (size_t volume = 0; volume < object->volumes.size(); ++volume) {
        const ModelVolume *candidate = object->volumes[volume];
        if (is_strength_dense_modifier(candidate))
            modifier_indices.push_back(volume);
    }
    if (modifier_indices.empty())
        return;
    m_plater->take_snapshot("Remove strength dense-region modifier");
    std::vector<std::pair<ModelVolume *, Geometry::Transformation>> remaining_transforms;
    for (ModelVolume *volume : object->volumes)
        if (!is_strength_dense_modifier(volume))
            remaining_transforms.emplace_back(volume, volume->get_transformation());
    std::vector<Geometry::Transformation> instance_transforms;
    for (const ModelInstance *instance : object->instances)
        instance_transforms.push_back(instance->get_transformation());
    for (auto volume = modifier_indices.rbegin(); volume != modifier_indices.rend(); ++volume)
        object->delete_volume(*volume);
    // Removing down to a single part normalizes its transform into the instances. Restore the
    // equivalent original frame so the saved loads and constraints do not jump on removal.
    for (const auto &remaining : remaining_transforms)
        remaining.first->set_transformation(remaining.second);
    for (size_t instance = 0; instance < object->instances.size(); ++instance)
        object->instances[instance]->set_transformation(instance_transforms[instance]);
    object->invalidate_bounding_box();
    m_plater->changed_object(index);
    m_plater->set_plater_dirty(true);
    load_selected_object();
    mark_stale();
    m_remove_dense_button->Disable();
    m_status_label->SetLabel(_L("Removed the managed dense-infill modifier; loads and constraints keep their placement. "
                               "Use the main Undo command to restore the modifier."));
}

void StrengthLoadPanel::apply_recommended_orientation()
{
    const ObjectID object_id = m_session->object_id;
    const uint64_t revision = m_session->revision;
    load_selected_object();
    if (object_id != m_session->object_id || revision != m_session->revision ||
        m_session->stale || m_session->result.orientation_recommendations.empty())
        return;
    const int index = m_session->object_index;
    if (index < 0 || size_t(index) >= m_plater->model().objects.size())
        return;
    const Vec3d layer_axis = m_session->result.orientation_recommendations.front().layer_axis;
    m_plater->take_snapshot("Apply strength-optimized orientation");
    ModelObject *object = m_plater->model().objects[size_t(index)];
    if (m_session->instance_index >= 0 && size_t(m_session->instance_index) < object->instances.size()) {
        ModelInstance *instance = object->instances[size_t(m_session->instance_index)];
        // A layer axis is a plane normal. Transform it with the inverse transpose before
        // alignment, including any existing instance rotation, nonuniform scale, or mirror.
        const Vec3d world_layer_axis = instance->get_matrix().linear().inverse().transpose() * layer_axis;
        Vec3d rotation_axis;
        double angle = 0.0;
        Matrix3d rotation;
        Geometry::rotation_from_two_vectors(world_layer_axis, Vec3d::UnitZ(), rotation_axis, angle, &rotation);
        instance->rotate(rotation);
        m_session->instance_transform = instance->get_matrix();
    }
    object->invalidate_bounding_box();
    m_session->setup.print_layer_axis = layer_axis;
    m_session->setup.follow_prepare_orientation = true;
    populate_from_setup();
    persist_setup(false);
    m_plater->changed_object(index);
    load_selected_object();
    mark_stale();
    m_status_label->SetLabel(_L("Applied the best estimated print orientation. Rerun to validate the rotated part."));
}

void StrengthLoadPanel::apply_optimized_settings()
{
    const ObjectID object_id = m_session->object_id;
    const uint64_t revision = m_session->revision;
    load_selected_object();
    if (object_id != m_session->object_id || revision != m_session->revision ||
        m_session->stale || m_session->result.print_settings_candidates.empty() ||
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

class StrengthSimulationPanel::ResultCanvas final : public SoftwareViewport3D
{
public:
    ResultCanvas(wxWindow *parent, std::shared_ptr<StrengthAnalysisSession> session, std::function<void(size_t)> probe)
        : SoftwareViewport3D(parent, parent->FromDIP(wxSize(560, 560)))
        , m_session(std::move(session)), m_probe(std::move(probe))
    {
        set_right_margin(parent->FromDIP(112));
    }

    void set_mode(int mode) { m_mode = mode; Refresh(); }
    void set_projection(int projection) { set_view(projection); }
    void set_deformation_scale(double scale) { m_deformation_scale = std::clamp(scale, 0.0, 10000.0); Refresh(); }
    void set_show_setup(bool value) { m_show_setup = value; Refresh(); }
    void set_show_wireframe(bool value) { m_show_wireframe = value; Refresh(); }
    void set_banded(bool value) { m_banded = value; Refresh(); }
    void set_dense_preview(const SA::DenseRegionPreview &preview, bool visible)
    {
        m_dense_preview = preview;
        m_show_dense_preview = visible;
        Refresh();
    }

protected:
    std::vector<Vec3d> scene_points() const override
    {
        std::vector<Vec3d> points;
        const indexed_triangle_set &mesh = display_mesh();
        if (m_session->result.succeeded() && m_session->result.vertices.size() == mesh.vertices.size()) {
            points.reserve(m_session->result.vertices.size());
            for (size_t index = 0; index < m_session->result.vertices.size(); ++index) {
                const SA::VertexResult &vertex = m_session->result.vertices[index];
                const Vec3d offset = vertex.displacement_m *
                    (1000.0 * m_deformation_scale * preview_load_multiplier() / preview_stiffness(index));
                points.push_back(vertex.position_mm + (offset.allFinite() ? offset : Vec3d::Zero()));
            }
        } else {
            points.reserve(mesh.vertices.size());
            for (const Vec3f &vertex : mesh.vertices)
                points.push_back(vertex.cast<double>());
        }
        return points;
    }

    void draw_scene(wxDC &dc, const CameraFrame &camera) override
    {
        const SA::Result &result = m_session->result;
        const indexed_triangle_set &mesh = display_mesh();
        m_projected.clear();
        if (mesh.vertices.empty() || mesh.indices.empty()) {
            dc.SetTextForeground(wxColour(65, 72, 82));
            dc.DrawText(_L("Select a model and solve its study from the Load tab."), FromDIP(22), FromDIP(22));
            return;
        }
        if (!result.succeeded() || result.vertices.size() != mesh.vertices.size()) {
            draw_reference_mesh(dc, camera, false);
            if (m_show_setup)
                draw_setup_glyphs(dc, camera);
            dc.SetTextForeground(wxColour(65, 72, 82));
            dc.DrawText(_L("Solve the study from the Load tab to display contour results."), FromDIP(22), FromDIP(22));
            return;
        }

        std::vector<Vec3d> deformed;
        deformed.reserve(result.vertices.size());
        m_projected.reserve(result.vertices.size());
        double minimum = std::numeric_limits<double>::infinity();
        double maximum = -minimum;
        size_t minimum_vertex = 0, maximum_vertex = 0;
        for (size_t index = 0; index < result.vertices.size(); ++index) {
            const SA::VertexResult &vertex = result.vertices[index];
            const Vec3d offset = vertex.displacement_m *
                (1000.0 * m_deformation_scale * preview_load_multiplier() / preview_stiffness(index));
            deformed.push_back(vertex.position_mm + (offset.allFinite() ? offset : Vec3d::Zero()));
            m_projected.push_back(project(deformed.back(), camera));
            const double value = scalar(index);
            if (std::isfinite(value)) {
                if (value < minimum) { minimum = value; minimum_vertex = index; }
                if (value > maximum) { maximum = value; maximum_vertex = index; }
            }
        }
        if (!std::isfinite(minimum) || !std::isfinite(maximum)) {
            minimum = maximum = 0.0;
        }
        if (m_mode == 0) {
            const double useful_ceiling = std::max(2.0, result.governing_target_safety_factor * 2.0);
            maximum = std::max(minimum, std::min(maximum, useful_ceiling));
        } else if (m_mode >= 4) {
            const double magnitude = std::max(std::abs(minimum), std::abs(maximum));
            minimum = -magnitude;
            maximum = magnitude;
        }
        const double range = std::max(maximum - minimum, 1e-12);

        std::vector<size_t> faces;
        faces.reserve(mesh.indices.size());
        for (size_t face = 0; face < mesh.indices.size(); ++face)
            if (valid_triangle(mesh.indices[face], m_projected.size()))
                faces.push_back(face);
        DepthBitmap result_bitmap(GetClientSize());
        for (size_t face : faces) {
            const Vec3i32 &triangle = mesh.indices[face];
            std::array<double, 3> values{};
            bool finite = true;
            for (int corner = 0; corner < 3; ++corner) {
                const size_t vertex = size_t(triangle[corner]);
                values[corner] = scalar(vertex);
                finite = std::isfinite(values[corner]) && finite;
            }
            result_bitmap.triangle(m_projected[size_t(triangle[0])], m_projected[size_t(triangle[1])],
                                   m_projected[size_t(triangle[2])],
                [this, finite, values, minimum, maximum, range](double a, double b, double c) {
                    return finite ? contour_colour(a * values[0] + b * values[1] + c * values[2],
                                                   minimum, maximum, range) : wxColour(150, 150, 150);
                });
        }
        if (m_show_wireframe)
            for (size_t face : faces) {
                const Vec3i32 &triangle = mesh.indices[face];
                for (int edge = 0; edge < 3; ++edge)
                    result_bitmap.line(m_projected[size_t(triangle[edge])],
                                       m_projected[size_t(triangle[(edge + 1) % 3])], wxColour(62, 67, 73));
            }
        result_bitmap.draw(dc);

        if (m_show_wireframe)
            draw_reference_mesh(dc, camera, true);
        if (m_show_setup)
            draw_setup_glyphs(dc, camera);
        if (m_show_dense_preview && m_dense_preview.available && !m_dense_preview.modifier_mesh.empty()) {
            const wxColour colour = m_dense_preview.overlaps_preserve ? wxColour(207, 67, 67) : wxColour(118, 49, 190);
            // X-ray the actual undeformed modifier mesh so interior reinforcement remains
            // visible through the contour surface. Depth-test the mask against itself.
            DepthBitmap mask(GetClientSize());
            std::vector<ScreenVertex> projected_mask;
            projected_mask.reserve(m_dense_preview.modifier_mesh.vertices.size());
            for (const Vec3f &vertex : m_dense_preview.modifier_mesh.vertices)
                projected_mask.push_back(project(vertex.cast<double>(), camera));
            for (const Vec3i32 &triangle : m_dense_preview.modifier_mesh.indices) {
                if (!valid_triangle(triangle, projected_mask.size()))
                    continue;
                const auto &vertices = m_dense_preview.modifier_mesh.vertices;
                const Vec3d normal = (vertices[triangle[1]] - vertices[triangle[0]]).cast<double>().cross(
                    (vertices[triangle[2]] - vertices[triangle[0]]).cast<double>());
                const double shade = normal.squaredNorm() > 1e-18 ?
                    0.6 + 0.4 * std::abs(normal.normalized().dot(camera.forward)) : 1.0;
                const wxColour shaded(int(colour.Red() * shade), int(colour.Green() * shade), int(colour.Blue() * shade), 135);
                mask.triangle(projected_mask[triangle[0]], projected_mask[triangle[1]], projected_mask[triangle[2]],
                    [shaded](double, double, double) { return shaded; });
            }
            mask.draw(dc);
            const ScreenVertex center = project(m_dense_preview.region.center_mm, camera);
            dc.SetTextForeground(colour);
            dc.DrawText(wxString::Format(_L("Stress-directed reinforcement %.1f%% (undeformed)"),
                m_dense_preview.estimated_volume_fraction * 100.0), center.point + wxPoint(10, 8));
        }

        draw_extreme_marker(dc, minimum_vertex, _L("MIN"),
                            m_mode == 0 ? wxColour(215, 52, 48) : wxColour(42, 92, 210));
        draw_extreme_marker(dc, maximum_vertex, _L("MAX"),
                            m_mode == 0 ? wxColour(42, 92, 210) : wxColour(215, 52, 48));
        draw_legend(dc, minimum, maximum);

        dc.SetTextForeground(wxColour(48, 57, 68));
        dc.DrawText(_L("Drag to orbit • middle/right drag to pan • wheel to zoom • click the model to probe"),
                    FromDIP(12), GetClientSize().y - FromDIP(24));
        if (m_session->stale) {
            dc.SetBrush(wxBrush(wxColour(255, 234, 203)));
            dc.SetPen(wxPen(wxColour(202, 119, 30), 1));
            dc.DrawRoundedRectangle(FromDIP(10), FromDIP(10), FromDIP(310), FromDIP(32), FromDIP(5));
            dc.SetTextForeground(wxColour(145, 72, 8));
            dc.DrawText(_L("STALE RESULT — rerun after setup or geometry changes"), FromDIP(20), FromDIP(18));
        }
    }

    void clicked(const wxPoint &point, bool) override
    {
        if (m_projected.empty())
            return;
        const indexed_triangle_set &mesh = display_mesh();
        bool found = false;
        double nearest_depth = -std::numeric_limits<double>::infinity();
        size_t picked_vertex = 0;
        for (const Vec3i32 &triangle : mesh.indices) {
            if (!valid_triangle(triangle, m_projected.size()))
                continue;
            const size_t indices[3]{size_t(triangle[0]), size_t(triangle[1]), size_t(triangle[2])};
            const wxPoint screen[3]{m_projected[indices[0]].point, m_projected[indices[1]].point,
                                    m_projected[indices[2]].point};
            double weights[3];
            if (!barycentric(point, screen, weights))
                continue;
            const double depth = weights[0] * m_projected[indices[0]].depth +
                weights[1] * m_projected[indices[1]].depth + weights[2] * m_projected[indices[2]].depth;
            if (depth <= nearest_depth)
                continue;
            nearest_depth = depth;
            picked_vertex = indices[std::distance(weights, std::max_element(weights, weights + 3))];
            found = true;
        }
        if (found)
            m_probe(picked_vertex);
    }

private:
    std::shared_ptr<StrengthAnalysisSession> m_session;
    std::function<void(size_t)> m_probe;
    int m_mode{0};
    double m_deformation_scale{1.0};
    bool m_show_setup{true};
    bool m_show_wireframe{true};
    bool m_banded{false};
    bool m_show_dense_preview{true};
    SA::DenseRegionPreview m_dense_preview;
    std::vector<ScreenVertex> m_projected;

    const indexed_triangle_set &display_mesh() const
    {
        return m_session->result.succeeded() && !m_session->solved_mesh.empty() ?
            m_session->solved_mesh : m_session->mesh;
    }

    const SA::Setup &display_setup() const
    {
        return m_session->result.succeeded() && !m_session->solved_mesh.empty() ?
            m_session->solved_setup : m_session->setup;
    }

    bool previewed(size_t vertex) const
    {
        return m_show_dense_preview && m_dense_preview.available && m_dense_preview.response_estimate_available &&
            !m_dense_preview.overlaps_preserve &&
            std::binary_search(m_dense_preview.affected_vertices.begin(), m_dense_preview.affected_vertices.end(), vertex);
    }

    double preview_strength(size_t vertex) const
    {
        return previewed(vertex) ? std::max(1e-12, m_dense_preview.local_strength_multiplier) : 1.0;
    }

    double preview_stiffness(size_t vertex) const
    {
        return previewed(vertex) ? std::max(1e-12, m_dense_preview.local_stiffness_multiplier) : 1.0;
    }

    double preview_load_multiplier() const
    {
        return m_show_dense_preview && m_dense_preview.available && m_dense_preview.response_estimate_available &&
            m_session->solved_setup.gravity.enabled &&
            m_session->result.estimated_mass_kg > 1e-12 ?
            std::max(1.0, m_dense_preview.estimated_total_mass_kg / m_session->result.estimated_mass_kg) : 1.0;
    }

    double scalar(size_t index) const
    {
        const SA::VertexResult &vertex = m_session->result.vertices[index];
        const double strength = preview_strength(index);
        const double stiffness = preview_stiffness(index);
        const double load = preview_load_multiplier();
        switch (m_mode) {
        case 0: return std::isfinite(vertex.safety_factor) ? vertex.safety_factor * strength / load : 10.0;
        case 1: return vertex.displacement_m.norm() * 1000.0 * load / stiffness;
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

    static wxColour rainbow(double fraction)
    {
        fraction = std::clamp(fraction, 0.0, 1.0);
        const double r = std::clamp(1.5 - std::abs(4.0 * fraction - 3.0), 0.0, 1.0);
        const double g = std::clamp(1.5 - std::abs(4.0 * fraction - 2.0), 0.0, 1.0);
        const double b = std::clamp(1.5 - std::abs(4.0 * fraction - 1.0), 0.0, 1.0);
        return wxColour(int(255.0 * r), int(255.0 * g), int(255.0 * b));
    }

    wxColour contour_colour(double value, double minimum, double maximum, double range) const
    {
        if (!std::isfinite(value))
            return wxColour(150, 150, 150);
        double fraction = (std::clamp(value, minimum, maximum) - minimum) / range;
        if (m_mode == 0)
            fraction = 1.0 - fraction;
        if (m_banded)
            fraction = std::round(fraction * 10.0) / 10.0;
        return rainbow(fraction);
    }

    void draw_reference_mesh(wxDC &dc, const CameraFrame &camera, bool undeformed_overlay)
    {
        const indexed_triangle_set &mesh = display_mesh();
        std::vector<ScreenVertex> original;
        original.reserve(mesh.vertices.size());
        for (const Vec3f &vertex : mesh.vertices)
            original.push_back(project(vertex.cast<double>(), camera));
        if (undeformed_overlay) {
            dc.SetPen(wxPen(wxColour(80, 88, 98), 1, wxPENSTYLE_SHORT_DASH));
            dc.SetBrush(*wxTRANSPARENT_BRUSH);
            for (const Vec3i32 &triangle : mesh.indices) {
                if (!valid_triangle(triangle, original.size()))
                    continue;
                wxPoint polygon[4]{original[size_t(triangle[0])].point, original[size_t(triangle[1])].point,
                                   original[size_t(triangle[2])].point, original[size_t(triangle[0])].point};
                dc.DrawLines(4, polygon);
            }
        } else {
            DepthBitmap reference(GetClientSize());
            for (const Vec3i32 &triangle : mesh.indices) {
                if (!valid_triangle(triangle, original.size()))
                    continue;
                reference.triangle(original[size_t(triangle[0])], original[size_t(triangle[1])],
                                   original[size_t(triangle[2])],
                                   [](double, double, double) { return wxColour(188, 199, 212); });
            }
            for (const Vec3i32 &triangle : mesh.indices) {
                if (!valid_triangle(triangle, original.size()))
                    continue;
                for (int edge = 0; edge < 3; ++edge)
                    reference.line(original[size_t(triangle[edge])], original[size_t(triangle[(edge + 1) % 3])],
                                   wxColour(95, 106, 119));
            }
            reference.draw(dc);
        }
    }

    void draw_setup_glyphs(wxDC &dc, const CameraFrame &camera)
    {
        const indexed_triangle_set &mesh = display_mesh();
        const SA::Setup &setup = display_setup();
        double radius = 1.0;
        for (const Vec3f &point : mesh.vertices)
            radius = std::max(radius, (point.cast<double>() - camera.center).norm());
        const double length = 0.24 * radius;
        for (const SA::Load &load : setup.loads) {
            if (!load.active)
                continue;
            const Vec3d center = load.region.whole_model ? camera.center : load.region.center_mm;
            const ScreenVertex anchor = project(center, camera);
            const wxColour colour = load_colour(load.type);
            if (!load.region.whole_model) {
                draw_region_shape(dc, camera, load.region, colour, false);
            }
            if (load.type == SA::LoadType::Fixed) {
                dc.SetPen(wxPen(colour, 3));
                dc.SetBrush(wxBrush(wxColour(214, 228, 252)));
                const int half = FromDIP(7);
                wxPoint support[3]{anchor.point + wxPoint(0, -half), anchor.point + wxPoint(-half, half),
                                   anchor.point + wxPoint(half, half)};
                dc.DrawPolygon(3, support);
                dc.SetTextForeground(colour);
            } else {
                const Vec3d direction = load.direction.squaredNorm() > 1e-12 ? load.direction.normalized() : Vec3d::UnitZ();
                const ScreenVertex tip = project(center + direction * length, camera);
                draw_arrow(dc, anchor.point, tip.point, colour, 3);
                dc.SetPen(wxPen(colour, 3));
                dc.SetBrush(wxBrush(wxColour(255, 255, 255)));
                dc.DrawCircle(anchor.point, FromDIP(4));
                dc.SetTextForeground(colour);
            }
            dc.DrawText(wxString::FromUTF8(load.name), anchor.point + wxPoint(9, -17));
        }
        if (setup.gravity.enabled) {
            const Vec3d direction = setup.gravity.acceleration_m_s2.normalized();
            const Vec3d start = camera.center - direction * 0.5 * radius;
            const wxPoint anchor = project(start, camera).point;
            draw_arrow(dc, anchor, project(start + direction * length, camera).point, wxColour(38, 133, 140), 3);
            dc.SetTextForeground(wxColour(28, 95, 102));
            dc.DrawText(_L("Gravity"), anchor + wxPoint(9, -17));
        }
        for (size_t index = 0; index < setup.preserve_regions.size(); ++index) {
            const SA::SphericalRegion &region = setup.preserve_regions[index];
            const ScreenVertex center = project(region.center_mm, camera);
            draw_region_shape(dc, camera, region, wxColour(43, 153, 91), false);
            dc.SetTextForeground(wxColour(31, 113, 66));
            dc.DrawText(wxString::Format(_L("Preserve %zu"), index + 1), center.point + wxPoint(9, 7));
        }
    }

    void draw_extreme_marker(wxDC &dc, size_t vertex, const wxString &label, const wxColour &colour)
    {
        if (vertex >= m_projected.size())
            return;
        const wxPoint point = m_projected[vertex].point;
        dc.SetPen(wxPen(colour, 3));
        dc.SetBrush(wxBrush(wxColour(255, 255, 255)));
        dc.DrawCircle(point, FromDIP(5));
        dc.SetTextForeground(colour);
        dc.DrawText(label, point + wxPoint(7, -14));
    }

    void draw_legend(wxDC &dc, double minimum, double maximum)
    {
        const wxSize size = GetClientSize();
        int label_width = 0;
        for (int tick = 0; tick <= 5; ++tick) {
            const double fraction = double(tick) / 5.0;
            label_width = std::max(label_width,
                dc.GetTextExtent(wxString::Format("%.4g", maximum - fraction * (maximum - minimum))).x);
        }
        const int width = FromDIP(24);
        const int x = std::max(FromDIP(20), size.x - width - label_width - FromDIP(24));
        const int top = FromDIP(70);
        const int height = std::max(FromDIP(180), size.y - FromDIP(150));
        constexpr int bands = 32;
        dc.SetPen(*wxTRANSPARENT_PEN);
        for (int band = 0; band < bands; ++band) {
            const double value_fraction = 1.0 - double(band) / double(bands - 1);
            double fraction = m_mode == 0 ? 1.0 - value_fraction : value_fraction;
            if (m_banded)
                fraction = std::round(fraction * 10.0) / 10.0;
            const int y0 = top + band * height / bands;
            const int y1 = top + (band + 1) * height / bands;
            dc.SetBrush(wxBrush(rainbow(fraction)));
            dc.DrawRectangle(x, y0, width, std::max(1, y1 - y0));
        }
        dc.SetPen(wxPen(wxColour(50, 55, 62), 1));
        dc.SetBrush(*wxTRANSPARENT_BRUSH);
        dc.DrawRectangle(x, top, width, height);
        dc.SetTextForeground(wxColour(35, 40, 48));
        dc.DrawText(result_title(), x - FromDIP(24), FromDIP(26));
        for (int tick = 0; tick <= 5; ++tick) {
            const double fraction = double(tick) / 5.0;
            const double value = maximum - fraction * (maximum - minimum);
            const int y = top + int(std::lround(fraction * height));
            dc.DrawLine(x + width, y, x + width + FromDIP(4), y);
            dc.DrawText(wxString::Format("%.4g", value), x + width + FromDIP(7), y - FromDIP(7));
        }
    }

    wxString result_title() const
    {
        static const std::array<wxString, 10> titles{
            _L("Safety factor"), _L("Displacement (mm)"), _L("Von Mises (MPa)"), _L("Max shear (MPa)"),
            _L("Normal X (MPa)"), _L("Normal Y (MPa)"), _L("Normal Z (MPa)"),
            _L("Shear XY (MPa)"), _L("Shear XZ (MPa)"), _L("Shear YZ (MPa)")};
        return titles[size_t(std::clamp(m_mode, 0, int(titles.size()) - 1))];
    }
};

StrengthSimulationPanel::StrengthSimulationPanel(wxWindow *parent, std::shared_ptr<StrengthAnalysisSession> session,
                                                 std::function<void()> synchronize_session,
                                                 std::function<bool(const SA::DenseRegionPreview &)> apply_dense_preview)
    : wxScrolledWindow(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL | wxHSCROLL | wxTAB_TRAVERSAL)
    , m_session(std::move(session))
    , m_synchronize_session(std::move(synchronize_session))
    , m_apply_dense_preview(std::move(apply_dense_preview))
{
    SetBackgroundColour(*wxWHITE);
    SetScrollRate(FromDIP(12), FromDIP(12));
    const int gap = FromDIP(8);
    auto *root = new wxBoxSizer(wxVERTICAL);
    root->Add(new wxStaticText(this, wxID_ANY, _L("Strength Analysis — 3D Results Workspace")), 0, wxALL, gap);
    m_status = new wxStaticText(this, wxID_ANY, _L("No analysis has been run."));
    root->Add(m_status, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    auto *controls = new wxBoxSizer(wxHORIZONTAL);
    m_result_mode = new wxChoice(this, wxID_ANY, wxDefaultPosition, wxDefaultSize,
        {_L("Safety factor"), _L("Deformation (mm)"), _L("Von Mises stress (MPa)"), _L("Maximum shear (MPa)"),
         _L("Normal stress X (MPa)"), _L("Normal stress Y (MPa)"), _L("Normal stress Z (MPa)"),
         _L("Shear stress XY (MPa)"), _L("Shear stress XZ (MPa)"), _L("Shear stress YZ (MPa)")});
    m_result_mode->SetSelection(0);
    m_projection = new wxChoice(this, wxID_ANY, wxDefaultPosition, wxDefaultSize,
                                {_L("Isometric"), _L("Front"), _L("Top"), _L("Right")});
    m_projection->SetSelection(0);
    m_deformation_scale = number_input(this, 1.0, 78);
    add_labeled(controls, this, _L("Result"), m_result_mode);
    controls->AddSpacer(gap);
    add_labeled(controls, this, _L("View"), m_projection);
    controls->AddSpacer(gap);
    add_labeled(controls, this, _L("Deformation scale"), m_deformation_scale);
    auto *fit_view = new wxButton(this, wxID_ANY, _L("Fit"));
    controls->AddSpacer(gap);
    controls->Add(fit_view, 0);
    root->Add(controls, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    auto *display = new wxBoxSizer(wxHORIZONTAL);
    m_show_setup = new wxCheckBox(this, wxID_ANY, _L("Show loads and constraints"));
    m_show_wireframe = new wxCheckBox(this, wxID_ANY, _L("Undeformed wireframe"));
    m_banded_contours = new wxCheckBox(this, wxID_ANY, _L("Banded contours"));
    m_show_setup->SetValue(true);
    m_show_wireframe->SetValue(true);
    m_banded_contours->SetValue(false);
    for (wxCheckBox *checkbox : {m_show_setup, m_show_wireframe, m_banded_contours})
        display->Add(checkbox, 0, wxRIGHT, FromDIP(16));
    display->AddStretchSpacer();
    display->Add(new wxStaticText(this, wxID_ANY,
        _L("Rainbow contours map the legend range; safety factor is reversed and signed stresses are centered on zero.")),
        0, wxALIGN_CENTER_VERTICAL);
    root->Add(display, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);

    auto *dense_preview = new wxStaticBoxSizer(wxVERTICAL, this, _L("Strengthened-region preview"));
    auto *dense_row = new wxBoxSizer(wxHORIZONTAL);
    m_show_dense_preview = new wxCheckBox(this, wxID_ANY, _L("Show predicted dense region"));
    m_show_dense_preview->SetValue(true);
    m_dense_volume_slider = new wxSlider(this, wxID_ANY, 15, 0, 100, wxDefaultPosition,
                                         FromDIP(wxSize(300, -1)), wxSL_HORIZONTAL);
    m_dense_volume_value = new wxStaticText(this, wxID_ANY, _L("15% of model volume"));
    m_apply_dense_button = new wxButton(this, wxID_ANY, _L("Create slicer modifier"));
    dense_row->Add(m_show_dense_preview, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, gap);
    dense_row->Add(m_dense_volume_slider, 1, wxALIGN_CENTER_VERTICAL | wxRIGHT, gap);
    dense_row->Add(m_dense_volume_value, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, gap);
    dense_row->Add(m_apply_dense_button, 0, wxALIGN_CENTER_VERTICAL);
    dense_preview->Add(dense_row, 0, wxEXPAND | wxALL, gap);
    auto *target_row = new wxBoxSizer(wxHORIZONTAL);
    m_target_safety_factor = number_input(this, 2.0, 90);
    m_size_to_safety_factor = new wxButton(this, wxID_ANY, _L("Size to target SF"));
    m_use_stress_threshold = new wxButton(this, wxID_ANY, _L("Use setup stress threshold"));
    target_row->Add(new wxStaticText(this, wxID_ANY, _L("Target estimated safety factor")),
                    0, wxALIGN_CENTER_VERTICAL | wxRIGHT, gap);
    target_row->Add(m_target_safety_factor, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, gap);
    target_row->Add(m_size_to_safety_factor, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, gap);
    target_row->Add(m_use_stress_threshold, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, gap);
    target_row->Add(new wxStaticText(this, wxID_ANY, _L("Estimate fitting in 1% volume steps; validate the design separately.")),
                    0, wxALIGN_CENTER_VERTICAL);
    dense_preview->Add(target_row, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    m_dense_preview_metrics = new wxStaticText(this, wxID_ANY,
        _L("Solve a study to preview strengthened volume, mass, deformation, and safety factor."));
    m_dense_preview_metrics->Wrap(FromDIP(920));
    dense_preview->Add(m_dense_preview_metrics, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    auto *preview_notice = new wxStaticText(this, wxID_ANY,
        _L("Predicted from the solved field for interactive sizing; local load paths are not re-solved. "
           "The created parameter modifier changes the actual sliced infill and should be validated."));
    preview_notice->Wrap(FromDIP(920));
    dense_preview->Add(preview_notice, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    root->Add(dense_preview, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);

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
    fit_view->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { m_canvas->fit_view(); });
    m_show_setup->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent &) { m_canvas->set_show_setup(m_show_setup->GetValue()); });
    m_show_wireframe->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent &) { m_canvas->set_show_wireframe(m_show_wireframe->GetValue()); });
    m_banded_contours->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent &) { m_canvas->set_banded(m_banded_contours->GetValue()); });
    m_deformation_scale->Bind(wxEVT_TEXT, [this](wxCommandEvent &) {
        double value = 1.0;
        if (read_number(m_deformation_scale, value)) m_canvas->set_deformation_scale(value);
    });
    m_dense_volume_slider->Bind(wxEVT_SLIDER, [this](wxCommandEvent &) { update_dense_preview(); });
    m_size_to_safety_factor->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { size_dense_preview_to_target(); });
    m_use_stress_threshold->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { size_dense_preview_to_threshold(); });
    m_show_dense_preview->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent &) { update_dense_preview(); });
    m_apply_dense_button->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
        if (m_apply_dense_preview && m_dense_preview.applicable() && !m_session->stale) {
            const SA::DenseRegionPreview preview = m_dense_preview;
            const bool applied = m_dense_profile && m_dense_profile == m_session->dense_profile && m_apply_dense_preview(preview);
            if (applied) {
                m_apply_dense_button->Disable();
                m_dense_preview_metrics->SetLabel(m_dense_preview_metrics->GetLabel() +
                    _L("  Slicer modifier created or updated; Slice now uses its dense infill settings."));
            } else {
                refresh();
                m_dense_preview_metrics->SetLabel(m_dense_preview_metrics->GetLabel() +
                    _L("  Modifier was not applied. Check the selected object and solve again."));
            }
            m_dense_preview_metrics->Wrap(FromDIP(920));
            Layout();
        }
    });
}

void StrengthSimulationPanel::activate()
{
    if (m_synchronize_session)
        m_synchronize_session();
    refresh();
}

void StrengthSimulationPanel::refresh()
{
    const SA::Result &result = m_session->result;
    if (!result.succeeded()) {
        m_status->SetLabel(result.status == SA::AnalysisStatus::NotRun ? _L("No analysis has been run.") :
            wxString::Format("%s — %s", wxString::FromUTF8(SA::to_string(result.status)), wxString::FromUTF8(result.message)));
        m_summary->ChangeValue({});
        update_dense_preview();
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
    const SA::Setup &display_setup = !m_session->solved_mesh.empty() ? m_session->solved_setup : m_session->setup;
    if (display_setup.criteria.maximum_displacement_mm > 0.0)
        text += wxString::Format(_L("Maximum-displacement target: %.6g mm — %s\n"),
            display_setup.criteria.maximum_displacement_mm, result.displacement_target_met ? _L("met") : _L("not met"));
    text += wxString::Format(_L("Requested resultant force: %s N\n\n"), vector_text(result.requested_resultant_force_n));
    text += wxString::Format(_L("Reinforcement setup: %.4g%% %s; seed stress threshold %.4g%% of peak. "
                                 "Use the strengthened-region controls above to preview and apply the actual shape.\n"),
        display_setup.infill.dense_density * 100.0, wxString::FromUTF8(SA::to_string(display_setup.infill.dense_pattern)),
        display_setup.infill.dense_stress_threshold * 100.0);
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
    update_dense_preview();
}

void StrengthSimulationPanel::update_dense_preview()
{
    const bool solved = m_session->result.succeeded() && !m_session->solved_mesh.empty() &&
        m_session->result.vertices.size() == m_session->solved_mesh.vertices.size();
    if (!solved || m_probe_revision != m_session->solved_revision) {
        m_probe_vertex = size_t(-1);
        m_probe->SetLabel(_L("Point probe: click near a mesh vertex."));
    }
    m_dense_volume_slider->Enable(solved && !m_session->stale);
    m_target_safety_factor->Enable(solved && !m_session->stale);
    m_size_to_safety_factor->Disable();
    m_use_stress_threshold->Disable();
    m_show_dense_preview->Enable(solved);
    m_dense_volume_value->SetLabel(wxString::Format(_L("%d%% of model volume"), m_dense_volume_slider->GetValue()));
    if (!solved) {
        m_dense_profile = {};
        m_dense_preview = {};
        m_dense_preview_metrics->SetLabel(
            _L("Solve a study to preview strengthened volume, mass, deformation, and safety factor."));
        m_apply_dense_button->Disable();
        m_canvas->set_dense_preview(m_dense_preview, false);
        Layout();
        return;
    }

    const SA::Result &result = m_session->result;
    m_dense_profile = m_session->dense_profile;
    if (!m_dense_profile || !m_dense_profile->available) {
        m_dense_preview = {};
        m_dense_volume_slider->Disable();
        m_target_safety_factor->Disable();
        m_show_dense_preview->Disable();
        m_dense_preview_metrics->SetLabel(m_dense_profile ? wxString::FromUTF8(m_dense_profile->warning) :
            _L("Solve the study to prepare its stress-directed reinforcement preview."));
        m_apply_dense_button->Disable();
        m_canvas->set_dense_preview(m_dense_preview, false);
        Layout();
        return;
    }
    m_dense_preview = SA::preview_dense_region(m_session->solved_mesh, m_session->solved_setup,
                                                result, m_dense_volume_slider->GetValue() / 100.0,
                                                m_dense_profile.get());
    m_size_to_safety_factor->Enable(!m_session->stale && m_dense_preview.available &&
        m_dense_preview.response_estimate_available && !m_session->solved_setup.gravity.enabled);
    m_use_stress_threshold->Enable(!m_session->stale && m_dense_preview.available);
    m_size_to_safety_factor->SetToolTip(m_session->solved_setup.gravity.enabled ?
        _L("Sizing to an estimated safety factor is unavailable when gravity is enabled.") :
        _L("Choose the smallest estimated region meeting the target, in 1% model-volume steps."));
    wxString metrics;
    if (!m_dense_preview.available) {
        metrics = m_dense_preview.warning.empty() ? _L("A dense-region preview is not available for this result.") :
            wxString::FromUTF8(m_dense_preview.warning);
    } else {
        metrics = wxString::Format(
            _L("Estimated strengthened volume %.2f%% (%.4g cm³) • stress threshold %.1f%% of peak • "
               "sampled stress coverage %.1f%% • estimated total mass %.4g kg (%+.4g kg)"),
            m_dense_preview.estimated_volume_fraction * 100.0, m_dense_preview.estimated_volume_m3 * 1e6,
            m_dense_preview.equivalent_stress_threshold * 100.0,
            m_dense_preview.stress_coverage * 100.0,
            m_dense_preview.estimated_total_mass_kg, m_dense_preview.estimated_added_mass_kg);
        if (m_dense_preview.response_estimate_available) {
            metrics += wxString::Format(_L(" • estimated minimum safety factor %.4g • maximum deformation %.4g mm"),
                m_dense_preview.predicted_minimum_safety_factor, m_dense_preview.predicted_maximum_displacement_mm);
        } else {
            metrics += _L(" • Safety-factor and deformation estimates unavailable; contours and probes retain the baseline response.");
        }
        if (!m_dense_preview.warning.empty())
            metrics += "  " + wxString::FromUTF8(m_dense_preview.warning);
    }
    if (m_session->stale)
        metrics += _L("  Results are stale; solve again before creating a modifier.");
    m_dense_preview_metrics->SetLabel(metrics);
    m_dense_preview_metrics->Wrap(FromDIP(920));
    m_apply_dense_button->Enable(!m_session->stale && m_dense_preview.applicable() && bool(m_apply_dense_preview));
    m_canvas->set_dense_preview(m_dense_preview, m_show_dense_preview->GetValue());
    if (m_probe_vertex < result.vertices.size())
        update_probe(m_probe_vertex);
    Layout();
}

void StrengthSimulationPanel::size_dense_preview_to_target()
{
    if (m_synchronize_session)
        m_synchronize_session();
    update_dense_preview();
    if (!m_size_to_safety_factor->IsEnabled())
        return;
    double target = 0.0;
    if (!read_number(m_target_safety_factor, target) || target <= 0.0) {
        wxMessageBox(_L("Enter a finite safety-factor target greater than zero."),
                     _L("Invalid safety-factor target"), wxOK | wxICON_WARNING, this);
        return;
    }

    const auto meets_target = [&](int percent) {
        const SA::DenseRegionPreview candidate = SA::preview_dense_region(
            m_session->solved_mesh, m_session->solved_setup, m_session->result, percent / 100.0, m_dense_profile.get(), false);
        return candidate.available && candidate.response_estimate_available && candidate.predicted_minimum_safety_factor >= target;
    };
    int selected_percent = 100;
    const bool reached = meets_target(100);
    if (reached) {
        // Validated dense settings cannot weaken the background, and the selected cell sets are
        // nested. The estimated SF is monotone, so binary search avoids 101 expensive previews.
        int lower = 0, upper = 100;
        while (lower < upper) {
            const int middle = lower + (upper - lower) / 2;
            if (meets_target(middle))
                upper = middle;
            else
                lower = middle + 1;
        }
        selected_percent = lower;
    }
    m_dense_volume_slider->SetValue(selected_percent);
    update_dense_preview();
    m_dense_preview_metrics->SetLabel(m_dense_preview_metrics->GetLabel() + (reached ?
        wxString::Format(_L("  Target estimated SF %.4g met at the smallest tested volume setting, %d%%. This is an estimate fit."),
                         target, selected_percent) :
        wxString::Format(_L("  Target estimated SF %.4g is unattainable in the tested 0–100%% range. "
                            "Showing the 100%% candidate, subject to preserve-region limits."), target)));
    m_dense_preview_metrics->Wrap(FromDIP(920));
    Layout();
}

void StrengthSimulationPanel::size_dense_preview_to_threshold()
{
    if (m_synchronize_session)
        m_synchronize_session();
    update_dense_preview();
    if (!m_use_stress_threshold->IsEnabled())
        return;
    const double threshold = m_session->solved_setup.infill.dense_stress_threshold;
    const SA::DenseRegionPreview candidate = SA::preview_dense_region(
        m_session->solved_mesh, m_session->solved_setup, m_session->result, 1.0, m_dense_profile.get(), false, threshold);
    if (!candidate.available)
        return;
    const int percent = int(std::lround(candidate.estimated_volume_fraction * 100.0));
    m_dense_volume_slider->SetValue(percent);
    update_dense_preview();
    m_dense_preview_metrics->SetLabel(m_dense_preview_metrics->GetLabel() + wxString::Format(
        _L("  Setup stress threshold %.4g%% maps to %.2f%% eligible model volume, rounded to the nearest 1%% slider step."),
        threshold * 100.0, candidate.estimated_volume_fraction * 100.0));
    m_dense_preview_metrics->Wrap(FromDIP(920));
    Layout();
}

void StrengthSimulationPanel::update_probe(size_t vertex_index)
{
    if (vertex_index >= m_session->result.vertices.size()) return;
    m_probe_vertex = vertex_index;
    m_probe_revision = m_session->solved_revision;
    const SA::VertexResult &value = m_session->result.vertices[vertex_index];
    wxString label = wxString::Format(
        _L("Point probe #%zu — position %s mm; displacement %s mm (|u| %.6g); normal stress %s MPa; "
           "shear XY/XZ/YZ %s MPa; Von Mises %.6g MPa; maximum shear %.6g MPa; safety factor %.6g"),
        vertex_index, vector_text(value.position_mm), vector_text(value.displacement_m * 1000.0), value.displacement_m.norm() * 1000.0,
        vector_text(value.normal_stress_pa / 1e6), vector_text(value.shear_stress_pa / 1e6), value.von_mises_pa / 1e6,
        value.maximum_shear_pa / 1e6, value.safety_factor);
    if (m_show_dense_preview->GetValue() && m_dense_preview.applicable() && m_dense_preview.response_estimate_available) {
        const bool strengthened = std::binary_search(m_dense_preview.affected_vertices.begin(),
                                                     m_dense_preview.affected_vertices.end(), vertex_index);
        const double strength = strengthened ? std::max(1e-12, m_dense_preview.local_strength_multiplier) : 1.0;
        const double stiffness = strengthened ? std::max(1e-12, m_dense_preview.local_stiffness_multiplier) : 1.0;
        const double load = m_session->solved_setup.gravity.enabled && m_session->result.estimated_mass_kg > 1e-12 ?
            std::max(1.0, m_dense_preview.estimated_total_mass_kg / m_session->result.estimated_mass_kg) : 1.0;
        label += wxString::Format(_L(" — dense preview: safety factor %.6g, displacement %.6g mm"),
            value.safety_factor * strength / load,
            value.displacement_m.norm() * 1000.0 * load / stiffness);
    } else if (m_show_dense_preview->GetValue() && m_dense_preview.available && !m_dense_preview.response_estimate_available) {
        label += _L(" — Dense-preview response unavailable; baseline response shown.");
    }
    m_probe->SetLabel(label);
    Layout();
}

} // namespace Slic3r::GUI
