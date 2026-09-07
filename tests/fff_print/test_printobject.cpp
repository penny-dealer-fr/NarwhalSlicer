#include <catch2/catch_all.hpp>

#include "libslic3r/libslic3r.h"
#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/StrengthAnalysis.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"

#include "test_helpers.hpp"

#include <iterator>
#include <set>

using namespace Slic3r;
using namespace Slic3r::Test;

TEST_CASE("Resizing and removing strength modifiers updates generated infill", "[PrintObject][StrengthAnalysis]")
{
    namespace SA = Slic3r::StrengthAnalysis;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", 0.3}, {"initial_layer_print_height", 0.3}, {"nozzle_diameter", 0.4},
        {"sparse_infill_density", 20}, {"sparse_infill_pattern", "rectilinear"},
        {"wall_loops", 1}, {"top_shell_layers", 0}, {"bottom_shell_layers", 0},
        {"top_shell_thickness", 0}, {"bottom_shell_thickness", 0}
    });
    Print print;
    Model model;
    init_print({cube(20)}, print, model, config);
    ModelObject &object = *model.objects.front();
    const int scale_case = GENERATE(0, 1, 2);
    const Vec3d geometry_scale = scale_case == 0 ? Vec3d::Ones() :
        (scale_case == 1 ? Vec3d(2.0, 2.0, 2.0) : Vec3d(2.0, 0.5, 1.5));
    object.instances.front()->set_scaling_factor(geometry_scale);
    object.invalidate_bounding_box();
    object.ensure_on_bed();
    CAPTURE(scale_case, geometry_scale.x(), geometry_scale.y(), geometry_scale.z());
    const double physical_volume_mm3 = 8000.0 * geometry_scale.prod();
    const indexed_triangle_set mesh = object.raw_mesh().its;
    SA::Setup setup;
    setup.geometry_scale = geometry_scale;
    // Once a response has been solved, either orientation mode uses the same native
    // reinforcement/slicing path; changing the mode must not suppress its modifier.
    setup.follow_prepare_orientation = GENERATE(true, false);
    setup.infill.background_pattern = SA::InfillPattern::Rectilinear;
    setup.infill.background_density = 0.2;
    setup.infill.dense_pattern = SA::InfillPattern::Gyroid;
    setup.infill.dense_density = 0.65;
    SA::Result result;
    result.geometry_scale = geometry_scale;
    result.status = SA::AnalysisStatus::Success;
    result.vertices.resize(mesh.vertices.size());
    for (size_t i = 0; i < result.vertices.size(); ++i) {
        result.vertices[i].position_mm = mesh.vertices[i].cast<double>();
        result.vertices[i].von_mises_pa = i == 0 ? 2e6 : 1e6;
        result.vertices[i].safety_factor = i == 0 ? 1.0 : 2.0;
    }
    const SA::DenseRegionPreviewProfile profile = SA::build_dense_region_preview_profile(mesh, result);
    REQUIRE(profile.available);

    struct SlicedInfill { double volume_mm3{0.0}; double dense_volume_mm3{0.0}; };
    const auto slice_infill = [&]() {
        print.apply(model, config);
        print.process();
        const ModelObject &sliced_object = *model.objects.front();
        if (scale_case == 2 && sliced_object.volumes.size() == 2) {
            MeshSlicingParamsEx parameters;
            parameters.trafo = sliced_object.instances.front()->get_matrix();
            std::vector<float> planes;
            for (const Layer *layer : print.objects().front()->layers())
                planes.push_back(float(layer->slice_z));
            const auto mask_slices = slice_mesh_ex(sliced_object.volumes.back()->mesh().its, planes, parameters);
            double mask_volume = 0.0;
            for (size_t layer = 0; layer < mask_slices.size(); ++layer)
                for (const auto &polygon : mask_slices[layer])
                    mask_volume += polygon.area() * SCALING_FACTOR * SCALING_FACTOR * print.objects().front()->layers()[layer]->height;
            CHECK_THAT(mask_volume,
                       Catch::Matchers::WithinAbs(std::abs(its_volume(sliced_object.volumes.back()->mesh().its)) *
                           geometry_scale.prod(), physical_volume_mm3 * 0.02));
        }
        SlicedInfill sliced;
        for (const Layer *layer : print.objects().front()->layers()) {
            for (const LayerRegion *region : layer->regions()) {
                sliced.volume_mm3 += region->fills.total_volume();
                if (region->region().config().sparse_infill_density.value > 20.0) {
                    CHECK_THAT(region->region().config().sparse_infill_density.value,
                               Catch::Matchers::WithinAbs(65.0, 1e-8));
                    CHECK(region->region().config().sparse_infill_pattern.value == ipGyroid);
                    for (const Surface &surface : region->slices.surfaces)
                        sliced.dense_volume_mm3 += surface.expolygon.area() * SCALING_FACTOR * SCALING_FACTOR * layer->height;
                }
            }
        }
        return sliced;
    };
    const auto apply_preview = [&](double fraction) {
        const auto preview = SA::preview_dense_region(mesh, setup, result, fraction, &profile, true, 0.0);
        REQUIRE(preview.applicable());
        CHECK_THAT(preview.estimated_volume_m3 * 1e9 / physical_volume_mm3,
                   Catch::Matchers::WithinAbs(preview.estimated_volume_fraction, 1e-8));
        // Use the same native parameter volume, coordinates, and keys as the GUI apply action.
        REQUIRE_FALSE(preview.modifier_mesh.empty());
        std::map<std::pair<int, int>, size_t> edge_counts;
        double signed_volume = 0.0;
        for (const auto &face : preview.modifier_mesh.indices) {
            const Vec3d a = preview.modifier_mesh.vertices[face[0]].cast<double>();
            const Vec3d b = preview.modifier_mesh.vertices[face[1]].cast<double>();
            const Vec3d c = preview.modifier_mesh.vertices[face[2]].cast<double>();
            signed_volume += a.dot(b.cross(c)) / 6.0;
            for (int edge = 0; edge < 3; ++edge) {
                int first = face[edge], second = face[(edge + 1) % 3];
                if (first > second) std::swap(first, second);
                ++edge_counts[{first, second}];
            }
        }
        CHECK_THAT(std::abs(signed_volume) * geometry_scale.prod() / physical_volume_mm3,
                   Catch::Matchers::WithinAbs(preview.estimated_volume_fraction, 1e-6));
        const size_t nonmanifold_edges = std::count_if(edge_counts.begin(), edge_counts.end(),
            [](const auto &edge) { return edge.second != 2; });
        INFO("Mask fraction " << fraction);
        CHECK(nonmanifold_edges == 0);
        auto *modifier = object.add_volume(TriangleMesh(preview.modifier_mesh), ModelVolumeType::PARAMETER_MODIFIER, false);
        modifier->set_transformation(Geometry::Transformation());
        modifier->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(65.0));
        modifier->config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipGyroid));
        modifier->config.set_key_value("strength_analysis_modifier", new ConfigOptionBool(true));
        object.config.set_key_value("sparse_infill_density", new ConfigOptionPercent(20.0));
        object.config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipRectilinear));
        // Keep two volumes alive during replacement: deleting first collapses the remaining
        // part transform into its instances and changes the preview's object coordinate frame.
        if (object.volumes.size() > 2)
            object.delete_volume(1);
        REQUIRE(object.volumes.size() == 2);
        CHECK(object.raw_mesh().its.vertices == mesh.vertices);
    };

    const SlicedInfill baseline = slice_infill();
    REQUIRE(baseline.volume_mm3 > 0.0);
    CHECK_THAT(baseline.dense_volume_mm3, Catch::Matchers::WithinAbs(0.0, 1e-8));
    apply_preview(0.15);
    const SlicedInfill small = slice_infill();
    CHECK_THAT(small.dense_volume_mm3 / physical_volume_mm3, Catch::Matchers::WithinAbs(0.15, 0.02));
    CHECK(small.volume_mm3 > baseline.volume_mm3);
    apply_preview(0.50);
    const SlicedInfill large = slice_infill();
    CHECK_THAT(large.dense_volume_mm3 / physical_volume_mm3, Catch::Matchers::WithinAbs(0.50, 0.02));
    CHECK(large.dense_volume_mm3 > small.dense_volume_mm3);
    CHECK(large.volume_mm3 > small.volume_mm3);
    apply_preview(1.0);
    const SlicedInfill full = slice_infill();
    CHECK_THAT(full.dense_volume_mm3 / physical_volume_mm3, Catch::Matchers::WithinAbs(1.0, 0.02));
    CHECK(full.volume_mm3 > large.volume_mm3);
    const Model reinforced_snapshot = model;
    const auto part_transform = object.volumes.front()->get_transformation();
    const auto instance_transform = object.instances.front()->get_transformation();
    object.delete_volume(1);
    object.volumes.front()->set_transformation(part_transform);
    object.instances.front()->set_transformation(instance_transform);
    object.invalidate_bounding_box();
    CHECK(object.raw_mesh().its.vertices == mesh.vertices);
    const SlicedInfill removed = slice_infill();
    CHECK_THAT(removed.dense_volume_mm3, Catch::Matchers::WithinAbs(0.0, 1e-8));
    CHECK_THAT(removed.volume_mm3, Catch::Matchers::WithinRel(baseline.volume_mm3, 1e-6));
    const Model removed_snapshot = model;
    // Model restoration (as used by main Undo/Redo) must invalidate the same sliced regions.
    model = reinforced_snapshot;
    const SlicedInfill restored = slice_infill();
    CHECK_THAT(restored.dense_volume_mm3, Catch::Matchers::WithinRel(full.dense_volume_mm3, 1e-6));
    CHECK_THAT(restored.volume_mm3, Catch::Matchers::WithinRel(full.volume_mm3, 1e-6));
    model = removed_snapshot;
    const SlicedInfill redone = slice_infill();
    CHECK_THAT(redone.volume_mm3, Catch::Matchers::WithinRel(baseline.volume_mm3, 1e-6));
}

SCENARIO("Object layer heights", "[PrintObject]") {
    GIVEN("A 20mm cube") {
        WHEN("sliced with a 2mm layer height and a 3mm nozzle") {
            Slic3r::Print print;
            Slic3r::Test::init_and_process_print({cube(20)}, print, {
                { "initial_layer_print_height", 2 },
                { "layer_height",               2 },
                { "nozzle_diameter",            3 }
	        });
            ConstLayerPtrsAdaptor layers = print.objects().front()->layers();
            THEN("The output vector has 10 entries") {
                REQUIRE(layers.size() == 10);
            }
            AND_THEN("Each layer is approximately 2mm above the previous Z") {
                coordf_t last = 0.0;
                for (size_t i = 0; i < layers.size(); ++ i) {
                    REQUIRE_THAT(layers[i]->print_z - last, Catch::Matchers::WithinAbs(2.0, 1e-4));
                    last = layers[i]->print_z;
                }
            }
        }
        WHEN("sliced with a 10mm layer height and an 11mm nozzle") {
            Slic3r::Print print;
            Slic3r::Test::init_and_process_print({cube(20)}, print, {
                { "initial_layer_print_height", 2 },
                { "layer_height",               10 },
                { "nozzle_diameter",            11 }
	        });
            ConstLayerPtrsAdaptor layers = print.objects().front()->layers();
			THEN("The output vector has 3 entries") {
                REQUIRE(layers.size() == 3);
            }
            AND_THEN("Layer 0 is at 2mm") {
                REQUIRE_THAT(layers.front()->print_z, Catch::Matchers::WithinAbs(2.0, 1e-4));
            }
            AND_THEN("Layer 1 is at 12mm") {
                REQUIRE_THAT(layers[1]->print_z, Catch::Matchers::WithinAbs(12.0, 1e-4));
            }
        }
        WHEN("sliced with a 15mm layer height and a 16mm nozzle") {
            Slic3r::Print print;
            Slic3r::Test::init_and_process_print({cube(20)}, print, {
                { "initial_layer_print_height", 2 },
                { "layer_height",               15 },
                { "nozzle_diameter",            16 }
	        });
            ConstLayerPtrsAdaptor layers = print.objects().front()->layers();
			THEN("The output vector has 2 entries") {
                REQUIRE(layers.size() == 2);
            }
            AND_THEN("Layer 0 is at 2mm") {
                REQUIRE_THAT(layers[0]->print_z, Catch::Matchers::WithinAbs(2.0, 1e-4));
            }
            AND_THEN("Layer 1 is at 17mm") {
                REQUIRE_THAT(layers[1]->print_z, Catch::Matchers::WithinAbs(17.0, 1e-4));
            }
        }
        WHEN("layer height exceeds the nozzle diameter") {
            // Orca does not clamp an over-large layer height to the nozzle; it
            // rejects the slice during flow computation. Pin that behavior.
            THEN("Slicing is rejected") {
                Slic3r::Print print;
                REQUIRE_THROWS(Slic3r::Test::init_and_process_print({cube(20)}, print, {
                    { "initial_layer_print_height", 0.3 },
                    { "layer_height",               0.5 },
                    { "nozzle_diameter",            0.4 }
                }));
            }
        }
    }
}

SCENARIO("Perimeter generation", "[PrintObject]") {
    GIVEN("20mm cube and default config") {
        WHEN("make_perimeters() is called")  {
            Slic3r::Print print;
            Slic3r::Test::init_and_process_print({cube(20)}, print, { { "sparse_infill_density", 0 } });
			const PrintObject &object = *print.objects().front();
            THEN("Every layer in region 0 has 1 island of perimeters") {
                for (const Layer *layer : object.layers())
                    REQUIRE(layer->regions().front()->perimeters.entities.size() == 1);
            }
        }
        WHEN("wall_loops is set to 3")  {
            Slic3r::Print print;
            Slic3r::Test::init_and_process_print({cube(20)}, print, {
                { "sparse_infill_density", 0 },
                { "wall_loops",            3 }
            });
            const PrintObject &object = *print.objects().front();
            THEN("Every layer in region 0 has 3 perimeter loops") {
                for (const Layer *layer : object.layers())
                    REQUIRE(layer->regions().front()->perimeters.items_count() == 3);
            }
        }
    }
}

TEST_CASE("Initial layer height is honored", "[PrintObject]")
{
    const std::string gcode = Slic3r::Test::slice({cube(20)}, {
        { "initial_layer_print_height", 0.3 },
        { "layer_height",               0.2 },
        { "z_hop",                      0 } // keep recorded Z equal to the printed layer height
    });

    std::set<double> layer_zs;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&layer_zs] (GCodeReader& self, const GCodeReader::GCodeLine& line) {
        if (line.extruding(self) && line.dist_XY(self) > 0)
            layer_zs.insert(self.z());
    });

    REQUIRE(layer_zs.size() > 1);
    REQUIRE_THAT(*layer_zs.begin(),            Catch::Matchers::WithinAbs(0.3, 1e-4));
    REQUIRE_THAT(*std::next(layer_zs.begin()), Catch::Matchers::WithinAbs(0.5, 1e-4));
}
