<div align="center">
<img alt="NarwhalSlicer logo" src="resources/images/NarwhalSlicer.png" width="160" height="160">

# NarwhalSlicer

**An OrcaSlicer fork built for strength calibration, optimization, simulation, and testing.**

[Source](https://github.com/penny-dealer-fr/NarwhalSlicer/tree/Strength-Optimization) · [Releases](https://github.com/penny-dealer-fr/NarwhalSlicer/releases) · [Strength workflow](docs/strength-analysis-workflow.md) · [License](LICENSE.txt)

</div>

NarwhalSlicer brings strength-focused workflows into the OrcaSlicer desktop slicing environment. Set up loads and constraints, calibrate material behavior against physical tests, inspect simulated response, and apply localized reinforcement through native slicer modifiers.

This is an independently maintained fork of [OrcaSlicer](https://github.com/OrcaSlicer/OrcaSlicer). OrcaSlicer’s website, releases, sponsors, and community accounts belong to the upstream project. OrcaCloud and printer-vendor services retain their own names and providers.

Development currently lives on [`Strength-Optimization`](https://github.com/penny-dealer-fr/NarwhalSlicer/tree/Strength-Optimization). The `main` branch retains the upstream baseline.

## Project focus

- **Strength calibration and testing:** use measured material and load-test data to inform analysis inputs. See [load calibration](docs/load-calibration.md) and [verification](docs/strength-analysis-verification.md).
- **Simulation:** define forces, fixed constraints, and gravity; inspect stress, displacement, and safety-factor estimates in the Load and Simulation workspaces.
- **Optimization:** compare orientation and print-setting choices, size dense regions, and create native parameter modifiers for localized reinforcement.
- **Slicing:** retain OrcaSlicer’s printer profiles, calibration tools, slicing controls, G-code preview, and supported printer integrations.

Strength analysis is experimental. Its results are engineering estimates, not certified finite-element analysis or guarantees of printed-part strength. Validate material assumptions and predictions with physical testing. The [workflow guide](docs/strength-analysis-workflow.md) explains the solver’s interpretation limits.

## Get started

1. Build this fork from source, or use a NarwhalSlicer binary if one is available on this repository’s [Releases page](https://github.com/penny-dealer-fr/NarwhalSlicer/releases). No published NarwhalSlicer release was available at the housekeeping review on September 14, 2026.
2. Open a model in **Prepare**, select the instance to study, and open **Load**.
3. Configure the material, constraints, and loads, then run **Pre-check** and **Solve**.
4. Inspect **Simulation**, apply a slice modifier if needed, then slice and inspect **Preview** before printing and testing.

Upstream OrcaSlicer downloads and package-manager entries install OrcaSlicer; they do not include this fork’s strength tools. Build targets, executable filenames, configuration directories, and some package identifiers retain upstream names for compatibility. Use **Help → Show Configuration Folder** to locate this build’s data.

## Documentation and development

- [Strength-analysis workflow](docs/strength-analysis-workflow.md)
- [Load and material calibration](docs/load-calibration.md)
- [Simulation animation](docs/strength-animation.md)
- [Verification and physical-test guidance](docs/strength-analysis-verification.md)
- [Narwhal artwork](docs/narwhal-branding.md)
- [Licensing and release notes for maintainers](docs/licensing-and-branding-review.md)
- [Upstream OrcaSlicer wiki](https://www.orcaslicer.com/wiki), for inherited slicer settings

Build from the repository root with the checked-in platform scripts:

| Platform | Initial build | App-only rebuild |
| --- | --- | --- |
| macOS (Apple Silicon) | `./build_release_macos.sh -a arm64` | `./build_release_macos.sh -s -a arm64` |
| Linux | `./build_linux.sh -u`, then `./build_linux.sh -dsi` | `./build_linux.sh -s` |
| Windows | `build_release_vs.bat` | `build_release_vs.bat slicer` |

See [repository guidelines](AGENTS.md), [test instructions](tests/README.md), and the [upstream build prerequisites](https://www.orcaslicer.com/wiki/how_to_build). For reports, include the NarwhalSlicer commit, operating system, reproduction steps, and a non-sensitive sample project. Distinguish a fork-specific issue from one reproduced in upstream OrcaSlicer.

## Attribution

NarwhalSlicer builds directly on **OrcaSlicer** and the work of its contributors. Its slicing lineage includes **Bambu Studio**, **PrusaSlicer** by Prusa Research, and **Slic3r** by Alessandro Ranellucci and the RepRap community, with contributions and ideas from **SuperSlicer** and **Cura**. Existing copyright and third-party license notices remain in the source and in **Help → About → License Info**.

The original OrcaSlicer logo was designed by [Justin Levine](https://github.com/jal-co). This fork uses the separate Narwhal artwork documented in [the branding guide](docs/narwhal-branding.md). Manufacturer and service logos identify their respective products; they do not imply endorsement of NarwhalSlicer.

## License

NarwhalSlicer is distributed under the **GNU Affero General Public License, version 3 (AGPLv3)**; see [LICENSE.txt](LICENSE.txt). Third-party components retain their respective licenses and notices. The software is provided without warranty, as described in the license.

The inherited pressure-advance calibration pattern includes work adapted from Andrew Ellis’ GPLv3 generator, itself adapted from Sineos’ GPLv3 generator for Marlin. The optional Bambu networking plugin uses non-free Bambu Lab libraries with separate terms; the project’s AGPL license does not relicense those libraries.

Modified distributions must retain required notices, identify changes, and provide Corresponding Source as required by the AGPL. Modified versions that support remote network interaction must offer that source to those users under section 13. Ordinary private use does not by itself require publishing every program you use alongside the slicer. See the [license text](LICENSE.txt) and [maintainer review](docs/licensing-and-branding-review.md).
