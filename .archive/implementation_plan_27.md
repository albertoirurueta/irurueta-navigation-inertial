# Implementation Plan

## Task summary

Source: GitHub issue #27

The Bibliography section of the Antora reference page
([docs/modules/ROOT/pages/reference.adoc](docs/modules/ROOT/pages/reference.adoc)) should link every listed book
and paper to its official publisher/editorial site (for books) or the official site where the paper can be
downloaded. Auditing the current bibliography shows most entries already have such links (`book-groves`,
`matlab-companion`, `wmm-site`, `igrf-site`, `imu-tk`); three paper entries are missing one:

- `[suh2010]` — Y. S. Suh, "Orientation estimation using a quaternion-based indirect Kalman filter with adaptive
  estimation of external acceleration," IEEE Transactions on Instrumentation and Measurement, 2010.
- `[trawny2005]` — N. Trawny, S. I. Roumeliotis, "Indirect Kalman Filter for 3D Attitude Estimation," University
  of Minnesota, Dept. of Computer Science & Engineering, Technical Report 2005-002, 2005.
- `[yuan2015]` — S. Yuan, "Quaternion-based Unscented Kalman Filter for Real-time Attitude Estimation," 2015.

Research performed during planning found confident official sources for the first two:
- `suh2010` matches IEEE Xplore document `10.1109/TIM.2010.2047157`
  (https://ieeexplore.ieee.org/document/5462839/) exactly by title, author, journal, and year — this is the
  paper's official publisher page.
- `trawny2005` is hosted at the University of Minnesota Multiple Autonomous Robotic Systems (MARS) Laboratory's
  own technical report archive: https://mars.cs.umn.edu/tr/reports/Trawny05b.pdf — matching the report's own
  institution and number exactly.

No confident match was found for `yuan2015`: no paper with this exact title, author, and year could be located
(the closest candidate, a 2015 Yuan paper on indoor heading estimation in *Sensors*, has a materially different
title/scope and would misattribute the citation). Per the user's decision, this entry is left unlinked, with a
short editorial note added instead of a link so a future contributor knows this was investigated and not simply
overlooked.

This work is documentation-only (AsciiDoc content), so no task below carries a language/framework tag.

Per the issue, changes must be made directly on `release_1.10.1` (satisfied — branch `feature/27` was created
from `release_1.10.1`) so they appear in the Antora docs for the most recent version; the resulting PR merges
back into `release_1.10.1`, and from there into `master` and then `develop` as a separate, later step outside
this plan's scope.

## Current code state

- The full bibliography lives in one place: the `[#bibliography]` section of
  [docs/modules/ROOT/pages/reference.adoc](docs/modules/ROOT/pages/reference.adoc) (lines 35-96), organized into
  `=== Books`, `=== Companion software`, `=== Earth magnetic field (WMM)`, and `=== IMU calibration` subsections.
- Existing entries follow a consistent AsciiDoc pattern: an anchor + bold short-form citation key, the full
  citation, then one or more `https://...[Label]` links, e.g.:
  ```
  [[imu-tk]] *[imu-tk]* D. Tedaldi, A. Pretto, E. Menegatti, "A Robust and Easy to Implement Method for IMU
  Calibration without External Equipments," _IEEE International Conference on Robotics and Automation
  (ICRA)_, 2014. https://albertopretto.altervista.org/papers/tpm_icra2014.pdf[PDF]. Reference
  implementation: https://github.com/Kyle-ak/imu_tk[github.com/Kyle-ak/imu_tk]. ...
  ```
- The three target entries (`suh2010`, `trawny2005`, `yuan2015`) sit consecutively under `=== IMU calibration`
  (lines 86-95) and currently end their citation sentence with only `Used by xref:...[...]` — no link.
- These citations back the `SuhQuaternionStepIntegrator`, `TrawnyQuaternionStepIntegrator`, and
  `YuanQuaternionStepIntegrator` classes under
  `src/main/java/com/irurueta/navigation/inertial/calibration/gyroscope/`, referenced from
  `docs/modules/ROOT/pages/calibration/gyroscope.adoc`. No source code changes are needed for this ticket.

## Implementation steps

### Group 1 — Update the bibliography (Parallelizable: yes — single file, sequential edits within one task)

- [x] Task 1. Add official links to the `suh2010` and `trawny2005` bibliography entries, and an explanatory note
      to `yuan2015`, in
      [docs/modules/ROOT/pages/reference.adoc](docs/modules/ROOT/pages/reference.adoc) — no tests apply
      (documentation-only AsciiDoc content); see Task 1.4 for the validity check.
  - [x] Task 1.1. Update the `[[suh2010]]` entry (currently line 86-88) to append a link to the IEEE Xplore page
        right after the citation, matching the existing link style, e.g.:
        ```
        [[suh2010]] *[suh2010]* Y. S. Suh, "Orientation estimation using a quaternion-based indirect Kalman
        filter with adaptive estimation of external acceleration," _IEEE Transactions on Instrumentation and
        Measurement_, 2010. https://ieeexplore.ieee.org/document/5462839/[IEEE Xplore]. Used by
        xref:calibration/gyroscope.adoc[`SuhQuaternionStepIntegrator`].
        ```
  - [x] Task 1.2. Update the `[[trawny2005]]` entry (currently line 90-92) to append a link to the official
        University of Minnesota MARS Lab technical report PDF, e.g.:
        ```
        [[trawny2005]] *[trawny2005]* N. Trawny, S. I. Roumeliotis, "Indirect Kalman Filter for 3D Attitude
        Estimation," University of Minnesota, Dept. of Computer Science & Engineering, Technical Report
        2005-002, 2005. https://mars.cs.umn.edu/tr/reports/Trawny05b.pdf[PDF]. Used by
        xref:calibration/gyroscope.adoc[`TrawnyQuaternionStepIntegrator`].
        ```
  - [x] Task 1.3. Update the `[[yuan2015]]` entry (currently line 94-95) to add a short parenthetical note
        instead of a link, since no confidently-matching official source was found, e.g.:
        ```
        [[yuan2015]] *[yuan2015]* S. Yuan, "Quaternion-based Unscented Kalman Filter for Real-time Attitude
        Estimation," 2015. (No official source could be located for this citation.) Used by
        xref:calibration/gyroscope.adoc[`YuanQuaternionStepIntegrator`].
        ```
  - [x] Task 1.4. Re-read the edited section to confirm AsciiDoc syntax stays valid (anchors, bold markers, and
        `xref:` links unchanged aside from the added links/note) and that line wrapping stays consistent with
        the surrounding file style (existing lines wrap at roughly 100 characters). — Confirmed: anchors,
        bold citation-key markers, and `xref:` links are unchanged; the three entries wrap at the same
        width as neighboring entries (e.g. `imu-tk`).

- [x] Task 2. Build the Antora documentation site to verify the edited page renders correctly — delegated to
      `iru-gate-runner`, which ran `iru-build-docs`. Build succeeded with no errors referencing
      `docs/modules/ROOT/pages/reference.adoc` (or any other page); no fix was needed. Built site:
      `docs/build/site/index.html`.
  - Delegate this to a sub-agent so build output doesn't consume the main context window:
    ```
    Agent({
      description: "Build Antora docs to verify reference.adoc changes",
      subagent_type: "iru-gate-runner",
      prompt: "Invoke Skill({skill: \"iru-build-docs\"}) to build this repository's Antora documentation site
        end to end. Report back only: whether the build succeeded, and if not, the specific error(s) referencing
        docs/modules/ROOT/pages/reference.adoc."
    })
    ```
  - If the build fails on the edited section (e.g. broken anchor, unescaped character), fix
    [docs/modules/ROOT/pages/reference.adoc](docs/modules/ROOT/pages/reference.adoc) and re-run the build.
