# Implementation Plan

## Task summary

Source: GitHub issue #38
Base branch: develop

Issue #38 ("Improve documentation", no labels, classified as a feature) asks to:

1. Add more detailed equations and diagrams to the Antora documentation (`docs/`), using P. D. Groves, *Principles of
   GNSS, Inertial, and Multisensor Integrated Navigation Systems* (2nd ed.) as the source (a local PDF is available
   to the author; the equations are also written in the code comments, which are the primary source to check
   against).
2. Improve the calibration documentation so that it does not only list which classes perform calibration but also
   explains the algorithms: the linear systems of equations and the non-linear algorithms used to solve them,
   referencing D. Tedaldi, A. Pretto, E. Menegatti, "A Robust and Easy to Implement Method for IMU Calibration
   without External Equipments", ICRA 2014 (and the `imu_tk` reference implementation) and the equations in the
   code comments.

Approach chosen (no user decision was needed):

- Documentation-only change: only `.adoc` files, `nav.adoc` and (if needed) SVG images under `docs/` are touched. No
  Java code changes.
- Equations use the existing `stem:[]` / `[stem]` MathJax setup; diagrams use Mermaid blocks (validated by the
  existing `docs/scripts/mermaid/validate-mermaid.mjs`) or SVGs under `docs/modules/ROOT/images/` following the
  existing `calibration/sensor-error-model.svg` convention.
- Every equation is cross-checked against the Javadoc/code comments of the class that implements it, so the docs
  describe what the code really does (including the fixes from issue #36). Book content is restated in own words
  and cited by chapter/equation number; book text is never copied.
- Pages keep their existing conventions: the AI-assistance NOTE, `:description:`/`:keywords:` header, `== Class
  Reference` and `== Where to go next` sections.
- Shared calibration theory (generic least squares, non-linear solvers, robust estimators) goes into one new page
  that the per-sensor pages cross-reference, avoiding duplication.

Open clarifications: none

## Acceptance criteria coverage

| # | Criterion | Covered by tasks |
|---|---|---|
| 1 | "Include more detailed equations and diagrams (available in P. D. Groves, Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems ...)" | Task 1, Task 2, Task 3, Task 4 |
| 2 | "improve documentation about calibration, by not only indicating which classes are used for calibration, but also explaining the algorithms an equations (or linear systems of equations and non-linear algorithms used to solve calibration)" | Task 5, Task 6, Task 7, Task 8, Task 9, Task 10 |
| 3 | "You can also find many of those equations in code comments as well as in reference: D. Tedaldi, A. Pretto, E. Menegatti ... (ICRA), 2014" | Task 5, Task 8, Task 11 |
| 4 | derived: every new equation matches the implementing code/Javadoc | Task 1 to Task 10 (each sub-task verifies), Task 12 |
| 5 | derived: the docs site builds and all Mermaid diagrams validate | Task 12 |
| 6 | derived: new/changed pages are reachable from navigation and the bibliography cites the new references | Task 5.3, Task 11 |

## Current code state

- Docs live in `docs/` (Antora component `irurueta-navigation-inertial`, `docs/antora.yml`, playbook
  `docs/antora-playbook.yml` with `@djencks/asciidoctor-mathjax` and `@sntke/antora-mermaid-extension`), 22 pages
  under `docs/modules/ROOT/pages/`, navigation in `docs/modules/ROOT/nav.adoc`, images in
  `docs/modules/ROOT/images/`. Build tooling in `docs/package.json` (`validate:mermaid`, `test:nav`).
- Equation coverage today (`stem:` occurrences): estimators pages 4-16 each; `navigators/navigation-equations.adoc`
  16; `wmm/spherical-harmonic-model.adoc` 22; `calibration/*` 2-19 each (mostly the error model);
  `navigators/kalman-filtered-estimators.adoc` 0 (only a Mermaid flow) and `wmm/index.adoc` 0.
- Calibration pages are organized by class family (tables of families, usage examples, "Linear vs. non-linear
  solvers" and "The robust-estimator wrapper" paragraphs in `calibration/accelerometer.adoc`, "Easy" calibrator in
  `calibration/gyroscope.adoc`) but do not derive the linear systems (design matrix, normal equations), the
  non-linear (Levenberg-Marquardt style) formulation, the gravity-norm cost function, or the Tedaldi static-interval
  method. Implementing code: packages `calibration/accelerometer`, `calibration/gyroscope`,
  `calibration/magnetometer`, `calibration/intervals`, `calibration/generators`, `calibration/noise`,
  `calibration/bias` under `src/main/java/com/irurueta/navigation/inertial/`.
- Navigation/estimation code: `navigators/` (`ECIInertialNavigator`, `ECEFInertialNavigator`,
  `NEDInertialNavigator`), `estimators/`, `INS*KalmanFilteredEstimator` in the root package, `wmm/`.
- `docs/modules/ROOT/pages/reference.adoc` holds the bibliography (anchor `book-groves`, `imu-tk`; see archived plan
  `.archive/implementation_plan_27.md` for how entries are linked).
- Docs-only work has no language key: all tasks are untagged. Quality gates for Java are not affected; the
  verification is the docs build (`iru-build-docs`) and `npm run validate:mermaid` in `docs/`.

## Implementation steps

### Group 1 (Parallelizable: yes — each task edits different page files)

Groves-based navigation theory: more equations and diagrams.

- [x] Task 1. Expand the estimators pages in `docs/modules/ROOT/pages/estimators/`
  - [x] Task 1.1. `gravity.adoc`: add the derivation chain used by `NEDGravityEstimator` (WGS84 gravity model,
    surface gravity vs. latitude, height correction, centrifugal term, gravity vs. specific force) with a diagram
    of the gravity/gravitation/centrifugal vector decomposition; verify against the Javadoc of `estimators/*Gravity*`.
  - [x] Task 1.2. `kinematics.adoc`: add the ECI/ECEF/NED body-kinematics equations (specific force and angular rate
    from frame differences, including the Coriolis and Earth-rate terms and the corrected ½·Ω_ie·C·τ term of
    eq. 5.85 and the start-of-interval gravity fixed in issue #36) plus a data-flow diagram.
  - [x] Task 1.3. `position-velocity.adoc`: add the curvature radii (meridian/transverse), position and velocity
    rates in NED, and the ECEF/NED conversions with a short coordinate-frames diagram.
  - [x] Task 1.4. `attitude.adoc`: add the attitude representations (Euler, DCM, quaternion), Rodrigues and
    quaternion kinematics, and a frame-rotation diagram; check consistency with `navigation-equations.adoc`.
  - Note (Task 1): touched `estimators/{gravity,kinematics,position-velocity,attitude}.adoc` and new
    `images/estimators/gravity-decomposition.svg`; `npm run validate:mermaid` passes.
- [x] Task 2. Expand `navigators/navigation-equations.adoc` and `navigators/index.adoc`: full attitude, velocity and
  position update equations per frame (ECI, ECEF, NED) in the order the navigators evaluate them, the
  Groves-chapter/equation references, and a Mermaid flow of one integration step; verify against
  `navigators/*InertialNavigator` comments.
  Done: edited `navigators/navigation-equations.adoc` and `navigators/index.adoc`; mermaid validated. Flagged: NED average-matrix correction has no tau factor in code.
- [x] Task 3 (done; touched only `docs/modules/ROOT/pages/navigators/kalman-filtered-estimators.adoc`; `validate:mermaid` passes). Expand `navigators/kalman-filtered-estimators.adoc` (currently no equations): error-state definition
  (attitude, velocity, position, accelerometer and gyroscope bias, 15 states), system matrix Φ and process noise Q,
  measurement models for loosely coupled (position/velocity innovation) and tightly coupled
  (pseudo-range and pseudo-range-rate innovation with clock offset/drift), Kalman gain, state and covariance
  update, and closed-loop correction; add a block diagram. Verify against `INSLooselyCoupledKalmanFilteredEstimator`,
  `INSGNSSLooselyCoupledKalmanFilteredEstimator`, `INSGNSSTightlyCoupledKalmanFilteredEstimator` and the Kalman
  filter helper classes they use.
- [x] Task 4. Expand `wmm/index.adoc`, `wmm/spherical-harmonic-model.adoc`, `wmm/earth-magnetic-flux-density.adoc`:
  add the pipeline overview equations for `index.adoc` (geodetic to geocentric spherical coordinates, field
  components X/Y/Z, declination/inclination/horizontal intensity, secular variation over time) and a
  coordinate-conversion diagram; keep existing harmonic-expansion content consistent.
  Done: touched wmm/index.adoc, wmm/spherical-harmonic-model.adoc, wmm/earth-magnetic-flux-density.adoc; mermaid validated.

### Group 2 (Parallelizable: no — Tasks 6 to 10 cross-reference the shared theory page Task 5 creates)

Shared calibration theory.

- [x] Task 5. Create `docs/modules/ROOT/pages/calibration/algorithms.adoc` ("Calibration algorithms")
  - [x] Task 5.1. Linear model: rewrite the error model as a linear-in-parameters system (stacking the nine entries
    of M and the three of b into a parameter vector, one 3-row block per measurement), the design matrix, the
    normal equations and the least-squares solution, minimum measurement counts (matching the tables in the
    accelerometer/gyroscope/magnetometer pages), conditioning and the "common z-axis" reduction (6 vs 9 unknowns);
    verify against the `*LinearLeastSquares*Calibrator` Javadoc.
  - [x] Task 5.2. Non-linear (gravity-norm / field-norm) formulation: cost function on the residual of the
    corrected-measurement norm vs. the known norm, Jacobian, Levenberg-Marquardt iteration and initial guess
    (obtained from the linear/approximate solution), convergence behavior; verify against
    `*NonLinearLeastSquares*Calibrator` and the `KnownGravityNorm`/`KnownPosition` classes.
  - [x] Task 5.3. Robust estimation: RANSAC, LMedS, MSAC, PROSAC, PROMedS (sample selection, residual/threshold,
    quality scores) with a Mermaid flowchart; add the page to `docs/modules/ROOT/nav.adoc` under Calibration (after
    `calibration/index.adoc`) and link it from `calibration/index.adoc` ("Map of the package" and "Where to go
    next").
  - [x] Task 5.4. Tedaldi et al. (ICRA 2014) method: static-interval detection with the threshold on the
    acceleration variance, per-interval mean as the input measurement, the multi-position accelerometer cost
    (unit gravity norm), the gyroscope cost via integrated rotation between static intervals vs. the
    accelerometer-derived gravity directions, and how `TriadStaticIntervalDetector`,
    `MeasurementsGenerator` and the "Easy" calibrators implement it, with a pipeline diagram. Fetch the paper
    (https://albertopretto.altervista.org/papers/tpm_icra2014.pdf) to confirm notation; do not copy its text.
  Done: created calibration/algorithms.adoc; nav.adoc and calibration/index.adoc (Map of the package, Where to go
  next) updated; equations cross-checked against the calibrator/detector/generator code; paper notation confirmed;
  validate:mermaid (23 diagrams) and test:nav pass. Docs only, no Java gates.

### Group 3 (Parallelizable: yes — each task edits a different calibration page)

Per-sensor and supporting calibration pages, built on Task 5.

- [x] Task 6. `calibration/accelerometer.adoc`: replace the paragraph-level "Linear vs. non-linear solvers" and
  "robust-estimator wrapper" sections with the concrete system of equations per family (`KnownFrame`: f_true from
  the frame; `KnownGravityNorm`: norm constraint; `KnownPosition`: gravity from `NEDGravityEstimator`), including
  the unknown counts and a worked small example of the linear system layout; link to `calibration/algorithms.adoc`.
- [x] Task 7. `calibration/gyroscope.adoc`: add the equations for `KnownFrame`, `Turntable` (known rotation rate
  and Earth rate, bias elimination) and `Easy` calibrators, including the g-dependent matrix estimation,
  the quaternion integration equations of `EulerQuaternionStepIntegrator`, `MidPointQuaternionStepIntegrator`
  and the other integrators, with integration-error comparison and a diagram.
- [x] Task 8. `calibration/magnetometer.adoc`: add hard-iron/soft-iron equations, the ellipsoid-fit view of the
  `KnownHardIronMagneticFluxDensityNorm` calibrators (norm constraint, linear and non-linear variants), and the
  WMM-position families (reference field from `wmm`), with a diagram contrasting the measured ellipsoid and the
  corrected sphere.
- [x] Task 9. `calibration/intervals-and-generators.adoc`: add the static-interval detector statistics (window,
  variance/threshold test, initial static period used to estimate the noise baseline), the measurement-generation
  rules and the time-interval estimation equations, with a state-machine diagram; align with Task 5.4.
- [x] Task 10. `calibration/noise-and-bias.adoc`, `calibration/fixers.adoc`, `calibration/data-model.adoc` and
  `calibration/index.adoc`: add the PSD/root-PSD definitions with the time-averaging relation, the approximate bias
  estimators, the inverse-model equations used by the fixers (including how M and G are inverted) and update the
  index so each family points to its algorithm section.

### Group 4 (Parallelizable: no — depends on the pages above being final)

Bibliography, release notes and verification.

- [x] Task 11. Update `docs/modules/ROOT/pages/reference.adoc`: add the Tedaldi et al. ICRA 2014 entry (linked to the
  official paper PDF), keep the `imu-tk` entry, and ensure every chapter/equation citation used in the new text
  resolves to an entry (follow the linking convention from `.archive/implementation_plan_27.md`). Add a short line
  to `docs/modules/ROOT/pages/whats-new.adoc` describing the documentation improvements.
  Done: Tedaldi entry (imu-tk anchor) already present and annotated; all cited anchors resolve; whats-new 'Unreleased' line added; algorithms.adoc static-threshold wording fixed.
- [x] Task 12. Verification
  - [x] Task 12.1. Run `npm run validate:mermaid` and `npm run test:nav` in `docs/` and fix any invalid diagram.
  - [x] Task 12.2. Build the site with the `iru-build-docs` skill (delegate to `iru-isolated-skill-executor`/
    `iru-gate-runner` so build output stays out of the main context); check that no xref, anchor or MathJax
    expression produces warnings, and skim the built `calibration/algorithms.html`.
  - [x] Task 12.3. Run `/iru-update-docs` is not required (this change is the docs); instead re-read the diff once
    to confirm no book text was copied verbatim and every equation was cross-checked against the code comments.
