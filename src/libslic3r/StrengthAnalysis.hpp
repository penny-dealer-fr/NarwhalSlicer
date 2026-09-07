#ifndef slic3r_StrengthAnalysis_hpp_
#define slic3r_StrengthAnalysis_hpp_

#include "Point.hpp"
#include "TriangleMesh.hpp"

#include <cstdint>
#include <array>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace Slic3r::StrengthAnalysis {

// Strength Analysis is an offline engineering-estimate model for printed parts. It intentionally
// models small, linear-elastic deformation only; callers must not present its output as a certified
// finite-element result.

enum class LoadType {
    Fixed,
    LocalForce,
    DirectionalForce,
    BearingForce,
    ImpactForce,
    GlobalForce
};

enum class StrengthBasis { Yield, Ultimate, CalibratedYield };

enum class InfillPattern {
    Rectilinear,
    Grid,
    Triangles,
    Honeycomb,
    Cubic,
    Gyroid
};

enum class AnalysisStatus { NotRun, Success, InvalidInput, Singular, Cancelled, NumericalFailure };

enum class RegionShape { Sphere, Box, Cylinder, Surface };

struct SphericalRegion {
    Vec3d center_mm{Vec3d::Zero()};
    double radius_mm{5.0};
    bool whole_model{false};
    RegionShape shape{RegionShape::Sphere};
    // Full dimensions for box regions. Cylinders use Z as height and radius_mm radially.
    Vec3d size_mm{Vec3d::Constant(10.0)};
    Vec3d axis{Vec3d::UnitZ()};
    // Triangle indices for a non-destructive, solid-like surface selection.
    std::vector<size_t> surface_triangles;

    bool contains(const Vec3d &position_mm) const;
};

struct SurfacePatch {
    std::vector<size_t> triangles;
    Vec3d normal{Vec3d::Zero()};
    Vec3d centroid_mm{Vec3d::Zero()};
    double area_mm2{0.0};
    size_t component_index{0};
};

struct SurfaceGroupingSettings {
    double coplanar_angle_degrees{2.0};
    double plane_tolerance_mm{0.02};
};

struct MaterialCalibration {
    double modulus_xy_scale{1.0};
    double modulus_z_scale{1.0};
    double strength_xy_scale{1.0};
    double strength_z_scale{1.0};
    double shear_scale{1.0};
    std::string source{"Uncalibrated"};
};

struct Material {
    std::string key{"pla_generic"};
    std::string name{"Generic PLA"};
    std::string provenance;
    double density_kg_m3{1240.0};
    double elastic_modulus_xy_pa{3.2e9};
    double elastic_modulus_z_pa{1.8e9};
    double poisson_xy{0.35};
    double shear_modulus_xy_pa{1.18e9};
    double shear_modulus_xz_pa{0.65e9};
    double yield_strength_xy_pa{50.0e6};
    double yield_strength_z_pa{25.0e6};
    double ultimate_strength_xy_pa{60.0e6};
    double ultimate_strength_z_pa{32.0e6};
    double shear_strength_xy_pa{32.0e6};
    double shear_strength_xz_pa{18.0e6};
    MaterialCalibration calibration;

    std::vector<std::string> validate() const;
    Material calibrated() const;
};

const std::vector<Material> &builtin_materials();
const Material *find_builtin_material(const std::string &key);

struct Load {
    std::string name{"Load"};
    LoadType type{LoadType::LocalForce};
    bool active{true};
    SphericalRegion region;
    Vec3d direction{Vec3d::UnitZ()};
    double magnitude_n{100.0};
    double impact_factor{2.0};
    double target_safety_factor{1.5};
    StrengthBasis strength_basis{StrengthBasis::Yield};
};

struct GravityLoad {
    bool enabled{false};
    Vec3d acceleration_m_s2{0.0, 0.0, -9.80665};
};

struct InfillSettings {
    InfillPattern background_pattern{InfillPattern::Gyroid};
    double background_density{0.20};
    InfillPattern dense_pattern{InfillPattern::Gyroid};
    double dense_density{0.65};
    double dense_stress_threshold{0.70};
};

struct OptimizationCriteria {
    double minimum_safety_factor{1.5};
    double maximum_displacement_mm{0.0}; // Zero disables the constraint.
    double mass_weight{1.0};
    double stiffness_weight{1.0};
    double support_weight{0.25};
    double print_time_weight{0.25};
    std::vector<double> candidate_densities{0.15, 0.25, 0.40, 0.60, 0.80};
    std::vector<InfillPattern> candidate_patterns{
        InfillPattern::Rectilinear, InfillPattern::Grid, InfillPattern::Triangles,
        InfillPattern::Honeycomb, InfillPattern::Cubic, InfillPattern::Gyroid};
};

struct SolverSettings {
    size_t maximum_vertices{50000};
    double transverse_stiffness_ratio{0.04};
    double regularization_ratio{1e-9};
};

struct Setup {
    int schema_version{1};
    Material material;
    Vec3d print_layer_axis{Vec3d::UnitZ()};
    bool follow_prepare_orientation{true};
    std::vector<Load> loads;
    GravityLoad gravity;
    std::vector<SphericalRegion> preserve_regions;
    InfillSettings infill;
    OptimizationCriteria criteria;
    SolverSettings solver;
};

struct VertexResult {
    Vec3d position_mm{Vec3d::Zero()};
    Vec3d displacement_m{Vec3d::Zero()};
    // Cauchy-stress engineering estimate in Pa: xx, yy, zz, xy, xz, yz.
    Vec3d normal_stress_pa{Vec3d::Zero()};
    Vec3d shear_stress_pa{Vec3d::Zero()};
    double von_mises_pa{0.0};
    double maximum_shear_pa{0.0};
    double safety_factor{std::numeric_limits<double>::infinity()};
};

struct DenseRegionRecommendation {
    bool available{false};
    SphericalRegion region;
    double stress_fraction{0.0};
    double recommended_density{0.0};
    InfillPattern recommended_pattern{InfillPattern::Gyroid};
};

struct DenseRegionCell {
    size_t grid_index{0};
    double volume_mm3{0.0}; // Actual model intersection, not the enclosing voxel volume.
    double stress_pa{0.0};
};

// Prepared off the UI thread. Cells are ranked by interpolated solved stress (descending), then
// grid index for deterministic ties. Exact polyhedral integration retains cavities and thin solids.
// Grid planes and the emitted mesh use object coordinates; the slicer clips voxels to the model.
struct DenseRegionPreviewProfile {
    bool available{false};
    size_t vertex_count{0};
    size_t triangle_count{0};
    // Bitwise fingerprint of all mesh coordinates/indices and the solved positions/stress field.
    // Displacements and safety factors are read afresh by preview_dense_region().
    std::uint64_t mesh_result_fingerprint{0};
    size_t hotspot_vertex{0};
    Vec3d hotspot_center_mm{Vec3d::Zero()};
    double hotspot_stress_pa{0.0};
    double solid_volume_m3{0.0};
    double sampled_volume_mm3{0.0};
    std::array<std::vector<double>, 3> grid_planes_mm;
    std::vector<DenseRegionCell> cells;
    std::vector<double> cumulative_volume_mm3;
    std::string warning;
};

// Fast, deterministic what-if estimate for resizing the recommended dense infill modifier. The
// preview deliberately does not claim to be a re-solved stress study: it applies the local
// density/pattern response to the solved vertex field so the UI can update while a slider moves.
struct DenseRegionPreview {
    bool available{false};
    bool overlaps_preserve{false};
    // Sizing remains applicable when false, but both response predictions are NaN: changed
    // self-weight requires a new solve. Callers should display the baseline response instead.
    bool response_estimate_available{true};
    // Authoritative native PARAMETER_MODIFIER geometry in OBJECT COORDINATES. Empty when
    // generate_modifier_mesh=false; region is only a diagnostic enclosing box.
    indexed_triangle_set modifier_mesh;
    SphericalRegion region;
    size_t selected_cell_count{0};
    double target_volume_fraction{0.0};
    double estimated_volume_fraction{0.0};
    double estimated_volume_m3{0.0};
    double stress_coverage{0.0};
    double equivalent_stress_threshold{0.0};
    double local_strength_multiplier{1.0};
    double local_stiffness_multiplier{1.0};
    double predicted_minimum_safety_factor{std::numeric_limits<double>::infinity()};
    double predicted_maximum_displacement_mm{0.0};
    double estimated_added_mass_kg{0.0};
    double estimated_total_mass_kg{0.0};
    std::vector<size_t> affected_vertices;
    std::string warning;

    bool applicable() const
    {
        return available && !overlaps_preserve && target_volume_fraction > 0.0 &&
            estimated_volume_fraction > 0.0 && selected_cell_count > 0;
    }
};

struct InfillComparison {
    InfillPattern pattern{InfillPattern::Gyroid};
    double density{0.0};
    double relative_stiffness{0.0};
    double relative_directional_strength{0.0};
    double estimated_mass_kg{0.0};
    double estimated_safety_factor{0.0};
    std::string provenance;
};

struct MassStrengthPoint {
    double target_safety_factor{0.0};
    bool feasible{false};
    double estimated_mass_kg{0.0};
    double density{0.0};
    InfillPattern pattern{InfillPattern::Gyroid};
    double predicted_safety_factor{0.0};
};

struct OrientationRecommendation {
    Vec3d layer_axis{Vec3d::UnitZ()};
    double predicted_safety_factor{0.0};
    double support_score{0.0};
    double combined_score{0.0};
};

struct PrintSettingsCandidate {
    int wall_loops{2};
    double layer_height_mm{0.20};
    double infill_density{0.20};
    InfillPattern pattern{InfillPattern::Gyroid};
    double predicted_safety_factor{0.0};
    double predicted_displacement_mm{0.0};
    double estimated_mass_kg{0.0};
    double score{0.0};
    bool feasible{false};
};

struct Result {
    AnalysisStatus status{AnalysisStatus::NotRun};
    std::string message;
    std::vector<std::string> warnings;
    std::vector<VertexResult> vertices;
    Vec3d requested_resultant_force_n{Vec3d::Zero()};
    double effective_volume_m3{0.0};
    double estimated_mass_kg{0.0};
    double maximum_displacement_m{0.0};
    size_t maximum_displacement_vertex{0};
    double maximum_von_mises_pa{0.0};
    size_t maximum_stress_vertex{0};
    double minimum_safety_factor{std::numeric_limits<double>::infinity()};
    size_t minimum_safety_factor_vertex{0};
    double governing_target_safety_factor{0.0};
    bool safety_factor_target_met{false};
    bool displacement_target_met{false};
    DenseRegionRecommendation dense_region;
    std::vector<InfillComparison> infill_comparisons;
    std::vector<MassStrengthPoint> mass_strength_curve;
    std::vector<OrientationRecommendation> orientation_recommendations;
    std::vector<PrintSettingsCandidate> print_settings_candidates;

    bool succeeded() const { return status == AnalysisStatus::Success; }
};

using CancelPredicate = std::function<bool()>;

std::vector<std::string> validate(const indexed_triangle_set &mesh, const Setup &setup);
// Returns the exact vertex set used by validation and the solver for a target region. Geometric
// regions may select the nearest vertex when requested; selected-surface regions never fall back.
std::vector<size_t> vertices_in_region(const indexed_triangle_set &mesh, const SphericalRegion &region,
                                       bool select_nearest = true);
// Groups edge-connected, coplanar triangles into selectable solid-like faces. The source mesh is
// never modified, so grouping cannot alter the printable or analyzed shape.
std::vector<SurfacePatch> group_coplanar_surfaces(const indexed_triangle_set &mesh,
                                                  const SurfaceGroupingSettings &settings = {});
std::vector<std::vector<size_t>> mesh_connected_components(const indexed_triangle_set &mesh);
Result analyze(const indexed_triangle_set &mesh, const Setup &setup, const CancelPredicate &cancel = {});

// Builds a reusable cancellable stress-directed interior-cell profile. Work-limit or cancellation
// failures are unavailable with an explicit warning, never a silently truncated profile.
DenseRegionPreviewProfile build_dense_region_preview_profile(const indexed_triangle_set &mesh,
                                                              const Result &result, const CancelPredicate &cancel = {});
// Selects the volume-weighted ranked prefix, skipping every cell intersecting a preserve region.
// The target is a share of total model volume; attainable share may be smaller due to preserves.
// Metrics-only callers may suppress mesh generation. A stale supplied cache fails explicitly so
// callers can rebuild off the UI thread. No profile supplied builds synchronously for headless use.
// An optional relative stress cutoff limits the eligible cells, after preserve exclusion.
// Response metrics are engineering what-if estimates, not a re-solved or full FEA validation.
DenseRegionPreview preview_dense_region(const indexed_triangle_set &mesh, const Setup &setup,
                                        const Result &result, double target_volume_fraction,
                                        const DenseRegionPreviewProfile *profile = nullptr,
                                        bool generate_modifier_mesh = true, double minimum_stress_fraction = 0.0);

std::vector<InfillComparison> compare_infill_patterns(double solid_volume_m3, const Setup &setup, double reference_safety_factor);
std::vector<MassStrengthPoint> estimate_mass_strength_curve(double solid_volume_m3, const Setup &setup, double reference_safety_factor);
std::vector<OrientationRecommendation> recommend_orientations(const indexed_triangle_set &mesh, const Setup &setup,
                                                               double reference_safety_factor);
std::vector<PrintSettingsCandidate> recommend_print_settings(double solid_volume_m3, const Setup &setup,
                                                              double reference_safety_factor,
                                                              double reference_displacement_mm = 0.0);

std::string serialize_setup(const Setup &setup);
// Pull the build-plate plane normal back into raw-object coordinates. Invalid or singular
// instance transforms return zero, which ordinary setup validation rejects.
Vec3d print_layer_axis_for_transform(const Transform3d &transform);
bool deserialize_setup(const std::string &json_text, Setup &setup, std::string *error = nullptr);
// Object config values are written into a quoted 3MF XML attribute by existing exporters. This
// XML-safe wrapper avoids raw JSON punctuation while retaining raw-JSON backward compatibility.
std::string serialize_setup_for_config(const Setup &setup);
bool deserialize_setup_from_config(const std::string &config_text, Setup &setup, std::string *error = nullptr);

std::string to_string(LoadType type);
std::string to_string(StrengthBasis basis);
std::string to_string(InfillPattern pattern);
std::string to_string(AnalysisStatus status);
std::string to_string(RegionShape shape);

} // namespace Slic3r::StrengthAnalysis

#endif
