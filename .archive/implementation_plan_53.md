# Implementation Plan

## Task summary

Source: GitHub issue #53
Base branch: develop

Issue #53 ("Resolve remaining Checkstyle findings in src/main (4,370 issues)", label `enhancement`, classified as
feature) asks for a decision per rule on the four Checkstyle rules still reported under `src/main/java` after #47,
each either fixed or configured in `checkstyle.xml` with a documented rationale, with `mvn test` staying green.

Measured on `develop` with `mvn -q clean compile checkstyle:checkstyle` (JDK 21 required; the default JDK 26 breaks
the compile with "invalid target release: 21"): MagicNumber 2,763; HiddenField 1,509; ParameterNumber 84;
ThrowsCount 14 (total 4,370, matching the issue).

Exploration findings that shape the plan:

- The HiddenField relaxations suggested in the issue (`ignoreConstructorParameter`, `ignoreSetter`) are **already**
  configured by #47. The remaining findings are mostly setters whose name does not match the parameter (e.g.
  `setComputeAndKeepInliersEnabled(boolean computeAndKeepInliers)`), which `ignoreSetter` does not recognise, plus
  other methods (~1,359 of them in `calibration`, 112 files).
- MagicNumber findings are almost all integer literals 4..20 (matrix/array indices and sizes), with a handful of
  decimal literals (`1.9`, `48.0`, `-2.0`, `6.0`, `24.0`, `4.0`, `365.0`, `366.0`); ~2,675 are in `calibration`.
- ParameterNumber findings: 71 have 16 parameters, the maximum is 28. ThrowsCount: all 14 have 3 throws.

Decisions taken (confirmed with the user):

- **MagicNumber**: relax the configuration (add the integer literals 4..20 to `ignoreNumbers`, documented as
  matrix/array dimensions and indices of a numerical library) and fix the remaining decimal literals with named
  constants where they carry meaning.
- **HiddenField**: rename parameters (and locals) so they no longer shadow fields. Only parameter/local names
  change; no public API member is renamed.
- **ParameterNumber / ThrowsCount**: raise the limits (`ParameterNumber` max 28, `ThrowsCount` max 3) with a
  documented rationale. No API change.

Open clarifications: none

## Acceptance criteria coverage

| # | Criterion | Covered by tasks |
|---|---|---|
| 1 | "Each of the four rules is either fixed or configured in `checkstyle.xml` with a documented rationale." | Task 1, Task 2, Task 3, Task 4, Task 5 |
| 2 | "The full `mvn test` suite stays green." | Task 6 |
| 3 | derived: `mvn -q clean compile checkstyle:checkstyle` reports 0 findings for the four rules | Task 6 |

## Current code state

- `checkstyle.xml` (repo root): `HiddenField` with `ignoreConstructorParameter`, `ignoreSetter`,
  `setterCanReturnItsClass`; `MagicNumber` with `ignoreNumbers` `-1, 0, 1, 2, 3, 0.5`, `ignoreHashCodeMethod`,
  `ignoreFieldDeclaration`, `ignoreAnnotation`; `ParameterNumber` `max=15`, `ignoreOverriddenMethods`;
  `ThrowsCount` `max=2`. Each relaxed rule has a rationale comment.
- `pom.xml`: `maven-checkstyle-plugin` 3.6.0 in `pluginManagement` and `<reporting>` using `checkstyle.xml`.
- Findings by area: `src/main/java/com/irurueta/navigation/inertial/calibration/**` (HiddenField 1,359; MagicNumber
  2,675; ParameterNumber 39), `navigators` (ParameterNumber 40), plus `INSLooselyCoupledKalmanState`,
  `INSTightlyCoupledKalmanState`, `INSTightlyCoupledKalmanConfig`, `INSTightlyCoupledKalmanInitializerConfig`,
  `INSLooselyCoupledKalmanConfig`, `INSTightlyCoupledKalmanEpochEstimator` (46 MagicNumber),
  `INSLooselyCoupledKalmanEpochEstimator`, the Kalman initializers and `*KalmanFilteredEstimator` classes
  (ThrowsCount).
- Worst HiddenField files: `EasyGyroscopeCalibrator` (91), `AngularRateFixer` (59),
  `KnownBiasEasyGyroscopeCalibrator` (56), `TurntableGyroscopeCalibrator` (54).
- Project type: java-library, Maven single module, JDK 21, JUnit tests under `src/test/java`. Antora docs in `docs/`.
- Run Maven with `JAVA_HOME` pointing at a JDK 21.
- Precedent: `.archive/implementation_plan_47.md`.

## Implementation steps

### Group 1: Checkstyle configuration (Parallelizable: no — Tasks 1 and 2 edit the same file, `checkstyle.xml`)

- [x] Task 1. Relax `MagicNumber` in `checkstyle.xml` _(java)_
  - [x] Task 1.1. Extend `ignoreNumbers` to `-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
    20, 0.5` and update the rationale comment: integer literals up to 20 are matrix/array dimensions and indices in
    a numerical library.
  - [x] Task 1.2. Re-run `mvn -q clean compile checkstyle:checkstyle` and record the remaining MagicNumber count and
    - Result: MagicNumber 15 remaining, all decimals: 1.9 (x8), -2.0 (x2), 48.0 (x2), 24.0, 365.0, 366.0.
    the decimal literals still reported.
- [x] Task 2. Raise `ParameterNumber` and `ThrowsCount` limits in `checkstyle.xml` _(java)_
  - [x] Task 2.1. `ParameterNumber` `max=28` (keep `ignoreOverriddenMethods`), comment: public calibrator/estimator
    constructors take many measurement and configuration arguments; maximum observed is 28.
  - [x] Task 2.2. `ThrowsCount` `max=3`, comment: estimators declare `LockedException`, `NotReadyException` and
    domain exceptions as checked exceptions of a public API that cannot be changed.
  - [x] Task 2.3. Re-run Checkstyle and confirm ParameterNumber and ThrowsCount report 0.
    - Result: ParameterNumber 0, ThrowsCount 0; HiddenField 1,509 remain.

### Group 2: HiddenField renames (Parallelizable: yes — the tasks edit disjoint packages/files; renaming parameters touches no test)

- [x] Task 3. Fix HiddenField in `calibration` _(java)_
  - [x] Task 3.1. Rename every parameter or local that shadows a field in
    `src/main/java/com/irurueta/navigation/inertial/calibration/**` (e.g. `computeAndKeepInliers` →
    `computeAndKeepInliersParam`-style names consistent with the file's existing conventions, updating the
    Javadoc `@param` tags to match). Public method and class names are unchanged.
  - [x] Task 3.2. Compile and re-run Checkstyle; confirm HiddenField is 0 in this package.
- [x] Task 4. Fix HiddenField outside `calibration` _(java)_
  - [x] Task 4.1. Same renaming for the remaining ~150 findings (`INSLooselyCoupledKalmanState`,
    `INSTightlyCoupledKalmanState`, the `*KalmanConfig`/`*InitializerConfig` classes and others), with Javadoc
    `@param` tags updated.
  - [x] Task 4.2. Compile and re-run Checkstyle; confirm HiddenField is 0 outside `calibration`.

<!-- Group 2 result: 1,509 HiddenField fixed (4,960 identifier edits, 112 files); params renamed to <name>Param, locals to <name>Local; 248 resulting LineLength overflows rewrapped; Checkstyle now only MagicNumber 15 (Group 3). Targeted tests run, see report. -->

### Group 3: Residual MagicNumber (Parallelizable: yes — a single task)

- [x] Task 5. Fix residual MagicNumber findings with named constants _(java)_
  - [x] Task 5.1. For the decimal literals still reported after Task 1 (`1.9`, `48.0`, `-2.0`, `6.0`, `24.0`, `4.0`,
    `365.0`, `366.0`), extract `private static final` constants with descriptive names in the owning class, or
    add the value to `ignoreNumbers` when it is a plain dimension, documenting the choice.
    Done: private static final constants added in 9 classes; MagicNumber now 0, total Checkstyle findings 0.
  - [x] Task 5.2. Run the affected unit tests via the `iru-gate-runner` agent with `iru-java-test` to confirm behaviour
    is unchanged.

### Group 4: Verification (Parallelizable: yes — a single task)

- [x] Task 6. Final verification _(java)_
  - [x] Task 6.1. `mvn -q clean compile checkstyle:checkstyle` reports 0 findings for the four rules, and no other
    rule regresses.
  - [x] Task 6.2. Run the full suite via the `iru-gate-runner` agent with `iru-java-test` and confirm it is green;
    also run `iru-java-code-quality` to confirm PMD and SpotBugs do not regress.
  - [x] Task 6.3. Add an entry to `CHANGELOG.md` under Unreleased summarising the Checkstyle policy and fixes.
