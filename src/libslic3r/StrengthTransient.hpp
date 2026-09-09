#ifndef slic3r_StrengthTransient_hpp_
#define slic3r_StrengthTransient_hpp_
#include "StrengthAnalysis.hpp"

namespace Slic3r::StrengthAnalysis {
// Layer-resolved, small-strain lattice estimate. Not a toolpath-resolved thermal FEA.
struct TransientSettings {
    double duration_s{5.0};
    bool unload{true};
    bool layers{true};
    bool plasticity{true};
    bool fracture{true};
    bool thermal_bonding{false};
    double layer_height_mm{0.2};
    double first_layer_height_mm{0.2};
    double cell_width_mm{3.0};
    int wall_loops{2};
    double line_width_mm{0.4};
    double nozzle_temperature_c{210.0};
    double bed_temperature_c{60.0};
    double chamber_temperature_c{25.0};
    double reference_nozzle_temperature_c{210.0};
    double cooling_time_s{10.0};
    double layer_time_s{10.0};
    double bonding_scale{1.0};
    double hardening_ratio{0.05};
    double failure_plastic_strain{0.15};
    double frames_per_second{30.0}; // GUI derives increments; headless callers may choose increments directly.
    double background_density{0.15};
    InfillPattern background_pattern{InfillPattern::Gyroid};
    std::string unsupported_print_pattern;
    indexed_triangle_set dense_region_mesh; // Immutable raw-object-space preview captured at run creation.
    double dense_volume_fraction{0.0};
    size_t increments{40};
    size_t maximum_cells{20000};
    size_t maximum_history_mb{512};
};
struct TransientFrame {
    double time_s{0.0};
    double load_fraction{0.0};
    std::vector<VertexResult> cells;
    std::vector<double> applied_forces_n;
    std::vector<double> probe_displacements_mm;
    std::vector<double> probe_von_mises_pa;
    std::vector<double> probe_maximum_shear_pa;
    size_t failed_bonds{0};
    size_t detached_cells{0};
    double maximum_plastic_strain{0.0};
};
struct TransientResult {
    AnalysisStatus status{AnalysisStatus::NotRun};
    std::string message;
    TransientSettings settings;
    indexed_triangle_set display_mesh;
    Vec3d geometry_scale{Vec3d::Ones()};
    std::vector<size_t> display_cells;
    std::vector<LoadRampProbe> probes;
    std::vector<TransientFrame> frames;
    std::vector<double> layer_bond_factors;
    std::vector<double> layer_interface_temperature_c;
    size_t layer_count{0};
    size_t dense_cell_count{0};
    double first_yield_time_s{std::numeric_limits<double>::infinity()};
    double first_failure_time_s{std::numeric_limits<double>::infinity()};
    bool succeeded() const { return status == AnalysisStatus::Success; }
};
TransientResult analyze_transient(const indexed_triangle_set &mesh, const Setup &setup, const TransientSettings &settings,
                                  const CancelPredicate &cancel = {}, const std::function<void(int)> &progress = {});
// Deterministic sampled history: scrubbing never mutates the plastic/damage state.
Result transient_display_frame(const TransientResult &history, size_t frame_index);
}
#endif
