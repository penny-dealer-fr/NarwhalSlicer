# NarwhalSlicer licensing and branding review

Reviewed September 14, 2026. This is a scoped documentation and display-text review, not a complete dependency or distribution audit.

## Fork notice

NarwhalSlicer is a modified version of OrcaSlicer. Fork changes include strength calibration, load setup, optimization, simulation, physical-test workflows, and Narwhal branding. This housekeeping pass updates project descriptions and display text. The Git history records individual changes and dates. Existing upstream copyright notices and the complete AGPLv3 text in `LICENSE.txt` are retained.

## Distribution obligations

Under [AGPLv3](https://www.gnu.org/licenses/agpl-3.0.en.html), preserve notices, identify modifications and dates, license covered modified work appropriately, and provide Corresponding Source with distributed binaries using a permitted method. For downloadable releases, provide the matching source and necessary build/install scripts alongside the binaries; upstream source alone does not cover fork changes. Preserve required interactive legal notices and warranty disclaimers. Section 13 requires a source offer to remote users of a modified program that supports network interaction. Simply running the desktop client privately does not require publishing unrelated software.

## Items requiring maintainer attention

- **Release source:** publish the exact source revision for each distributed binary, including fork changes and required build inputs. The strength implementation and assets are published on `Strength-Optimization`; the default `main` branch still contains upstream code. Link recipients to the matching fork revision, and publish remaining local changes before distributing a binary built from them.
- **Optional proprietary plugins:** the Bambu networking plugin loads separately licensed non-free libraries. Optional loading is not by itself proof of AGPL compatibility or redistribution permission. Verify the applicable library terms and any permission covering the combined distribution before shipping those binaries. If permission is insufficient, changing packaging or integration may be necessary; this pass does not alter either.
- **Artwork:** Narwhal assets derive from the user-supplied master documented in `narwhal-branding.md`. This review does not establish authorship or redistribution rights for that supplied artwork. Record its creator, license, and any required attribution before distributing it. Preserve third-party font and artwork terms. Manufacturer/service logos remain associated with their own products.
- **Hosted deployments:** if a modified NarwhalSlicer is exposed to remote users, verify the section 13 source offer in that deployed interface. No server deployment was audited here.

No slicing, solver, or printer behavior change is established as necessary by this review. The plugin and deployment questions above require verification of the actual distribution and terms.

## Names intentionally retained

`SLIC3R_APP_NAME` and `SLIC3R_APP_KEY`, executable and bundle filenames, bundle IDs, installer identities, configuration directories, URL schemes, G-code producer markers, 3MF metadata, translation catalog filenames, and updater endpoints retain upstream values. Some are parsed or used for storage and compatibility, so changing them would go beyond a display-text pass. The display-only full app and G-code viewer names are NarwhalSlicer; the existing system-info payload identifier is preserved. Visible launcher, dialog, and package descriptions use NarwhalSlicer where they do not control those mechanisms.

OrcaCloud, Bambu Lab, printer models, upstream bug references, historical version descriptions, and their service URLs retain their actual names. The inherited update checker still targets OrcaSlicer releases; its update headings identify them as upstream. Separating update feeds, install locations, and application identities is a future functional/packaging task, not a licensing requirement established here.

The inherited customer-experience/telemetry consent pages still contain claims about upstream machine learning and link to a provider privacy policy. Rewriting those assurances requires verifying collection, recipients, and provider terms; the naming pass does not invent a Narwhal privacy policy or change consent behavior.

## GitHub presentation

The repository About description should read: “NarwhalSlicer is an OrcaSlicer fork built for strength calibration, optimization, simulation, and testing of 3D-printed parts.” Its website field points to this repository. The README uses the existing Narwhal logo and removes upstream download, sponsor, and social-account claims.

At review time, the public default branch still showed upstream source and README content, no Narwhal releases were published, Issues was disabled, and GitHub Pages had no configured source (disabled). The updated issue templates are ready if the maintainer enables Issues. Local documentation and display-name changes take effect on GitHub after publication and in the application after rebuilding.
