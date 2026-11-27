# Implementation Plan

## Task summary

Source: GitHub issue #46
Base branch: develop

Issue #46 ("Fix flaky magnetometer calibrator tests (KnownPositionAndInstant noisy tests, RANSAC norm common-axis
test)", label `bug`, classified as hotfix) reports three tests that failed in CI run 45711218476 and passed on the
same code the next day:

- `KnownPositionAndInstantMagnetometerCalibratorTest.testCalibrateForGeneralCaseWithLargeNumberOfMeasurementsAndNoise`
- `KnownPositionAndInstantMagnetometerCalibratorTest.testCalibrateForCommonAxisCaseWithLargeNumberOfMeasurementsAndNoise`
- `RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibratorTest.testCalibrateCommonAxisNoNoiseInlier`

Only test code changes; production calibrators are not at fault.

Decisions taken on the user's behalf, backed by measurements on a scratch copy of the repository (100 iterations
per test, `break` removed so every iteration is evaluated):

- **Noise level**: lower `MAGNETOMETER_NOISE_STD` in `KnownPositionAndInstantMagnetometerCalibratorTest` from
  `200e-9` to `2e-9`, the value every sibling `Known*` magnetometer test already uses. Measured effect: iterations
  passing the current tolerances went from 2–3 / 100 to **100 / 100** for both tests.
  Hard-iron error: median ≈ 3–5e-9 T, max ≈ 1.9e-8 T. Rejected alternative: widening the attitude range to the full
  sphere. That changes the shared `generateBodyC` for all tests in the class, and isn't needed once the noise is
  consistent with the siblings.
- **Meaningful tolerances**: the current hard-iron tolerance (`LARGE_ABSOLUTE_ERROR = 5e-5`) is wider than the whole
  hard-iron range (`±1e-5`), and the soft-iron tolerance (`5 × 1e-2`) is ~5 orders of magnitude above the soft-iron
  range (`±1e-6`), so both checks are vacuous. For the two noisy tests only:
  - Hard-iron tolerance: `1e-7` T, about 5× the max error measured.
  - Soft iron drawn from a realistic, test-local range `±0.05`.
  - Soft-iron tolerance: `5e-3`, about 7× the max error measured (7.3e-4) and 10× tighter than the range.
  Both are new test-local constants, so the other ~80 tests in the class keep their current inputs.
- **Robust retry loops**: every `calibrator.calibrate()` inside a `for (t < TIMES)` retry loop of the robust
  magnetometer norm / position-and-instant tests is wrapped in `try { … } catch (final CalibrationException e)
  { continue; }`. This matches the existing `testCalibrateGeneralNoNoiseInlier` pattern, so one dataset without a
  RANSAC/LMedS/MSAC/PROSAC/PROMedS consensus moves on instead of erroring the test. 54 unguarded call sites were
  found in 20 files (listed per task below).
- **Seeding**: not done. The issue lists it as optional, and the retry loops become reliable without it.

- **Robust general inlier-noise tests** (decided by the user after the first `/iru-code` run stopped at Task 6.2):
  `LMedSRobustKnownPositionAndInstantMagnetometerCalibratorTest#testCalibrateGeneralWithInlierNoise` turned out to
  be flaky too, and wasn't covered by Groups 1–2. It gets the same realistic soft-iron range and measured
  tolerances as Tasks 1 and 5, and its robust siblings are measured and fixed the same way (Task 7).
  After the second run showed 0% convergence at 200 nT noise, the user decided to also lower the noise
  class-wide to 2e-9 in the 20 robust norm and position-and-instant test classes, scaling the outlier factor to
  keep the outlier noise unchanged (Task 7.1).

Open clarifications: none

## Acceptance criteria coverage

| # | Criterion | Covered by tasks |
|---|---|---|
| 1 | "The three failing tests above pass consistently" (e.g. 50 consecutive local runs with 0 failures) | Task 1, Task 2, Task 6 |
| 2 | "The tolerances in the fixed tests are tighter than the range of the randomly generated ground-truth values, so the assertions check accuracy rather than only catching divergence" | Task 1 |
| 3 | "Sibling magnetometer tests with the same patterns are reviewed and fixed where needed" | Task 3, Task 4, Task 5, Task 7 |
| 4 | "Only test code changes. No production code changes unless the investigation shows a real calibrator bug" | Task 1, Task 2, Task 3, Task 4, Task 5 (all confined to `src/test`) |
| 5 | "The full `mvn test` suite stays green on CI" | Task 6, Task 7 |

## Current code state

- Build: Maven, Java 21, single module, JUnit 5 (`pom.xml`). Project type `java-library`.
- Test package: `src/test/java/com/irurueta/navigation/inertial/calibration/magnetometer/` (abbreviated `…/magnetometer/`
  below).
- `KnownPositionAndInstantMagnetometerCalibratorTest.java`
  - Constants (lines 56–82):
    - `MIN/MAX_HARD_IRON = ±1e-5`, `MIN/MAX_SOFT_IRON = ±1e-6`, `MIN/MAX_ANGLE_DEGREES = ±45`
    - `LARGE_MEASUREMENT_NUMBER = 1000`, `MAGNETOMETER_NOISE_STD = 200e-9`
    - `ABSOLUTE_ERROR = 1e-9`, `LARGE_ABSOLUTE_ERROR = 5e-5`, `VERY_LARGE_ABSOLUTE_ERROR = 1e-2`, `TIMES = 100`
  - `testCalibrateForGeneralCaseWithLargeNumberOfMeasurementsAndNoise` (~line 7110) and
    `testCalibrateForCommonAxisCaseWithLargeNumberOfMeasurementsAndNoise` (~line 7253) loop `TIMES` iterations. Each
    builds a random position/timestamp, 1000 noisy measurements via `generateMeasures(...)`, and a
    `KnownPositionAndInstantMagnetometerCalibrator` initialised with the true `hardIron`/`mm`, then calls
    `calibrate()` and skips the iteration if `hardIron`/`mm` aren't within `LARGE_ABSOLUTE_ERROR` /
    `5.0 * VERY_LARGE_ABSOLUTE_ERROR`. The final assertion `assertTrue(numValid > 0)` is what fails.
  - Helpers `generateSoftIronGeneral()` / `generateSoftIronCommonAxis()` (~lines 7584–7606) use the class-wide
    soft-iron range.
- `RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibratorTest.java`
  - `testCalibrateCommonAxisNoNoiseInlier` (~line 8461) calls `calibrator.calibrate()` unguarded at line 8511.
  - When `RANSACRobustEstimator.estimate` throws `RobustEstimatorException` (wrapped in `CalibrationException` by
    `RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.calibrate`, line 1088), the whole test errors.
  - Reference pattern: `testCalibrateGeneralNoNoiseInlier` (~lines 8418–8422).
- Sibling `Known*` tests (`KnownHardIronPositionAndInstantMagnetometerCalibratorTest`,
  `KnownMagneticFluxDensityNormMagnetometerCalibratorTest`, `KnownHardIronMagneticFluxDensityNormMagnetometerCalibratorTest`)
  already use `MAGNETOMETER_NOISE_STD = 2e-9`. The first two share the vacuous `LARGE_ABSOLUTE_ERROR = 5e-5` hard-iron
  tolerance; all three share `VERY_LARGE_ABSOLUTE_ERROR = 1e-2`.
- Production code (`BaseMagneticFluxDensityNormMagnetometerCalibrator`, Levenberg-Marquardt with a numerical
  gradient) is unchanged by this plan.

## Implementation steps

### Group 1 — Test fixes (Parallelizable: yes, every task edits a disjoint set of test files)

- [x] Task 1. Make the noisy large-measurement tests of `KnownPositionAndInstantMagnetometerCalibratorTest` deterministic and meaningful _(java)_ — implemented in `KnownPositionAndInstantMagnetometerCalibratorTest.java`, constants/overloads added, both noisy tests guarded and tightened; group validated: `*Known*MagnetometerCalibratorTest` (40 classes) and full `mvn clean test` (14025 tests) green; coverage n/a (test-only change, no `src/main` class touched); no new Checkstyle/PMD/SpotBugs issues (analyzers do not cover test sources).
  - [x] Task 1.1. In `…/magnetometer/KnownPositionAndInstantMagnetometerCalibratorTest.java`, change
        `MAGNETOMETER_NOISE_STD` from `200e-9` to `2e-9` (aligns with all sibling tests).
  - [x] Task 1.2. Add test-local constants next to the existing ones:
        ```java
        private static final double MIN_NOISY_SOFT_IRON = -0.05;
        private static final double MAX_NOISY_SOFT_IRON = 0.05;
        private static final double NOISY_HARD_IRON_ABSOLUTE_ERROR = 1e-7;
        private static final double NOISY_SOFT_IRON_ABSOLUTE_ERROR = 5e-3;
        ```
  - [x] Task 1.3. Add overloads `generateSoftIronGeneral(final double minValue, final double maxValue)` and
        `generateSoftIronCommonAxis(final double minValue, final double maxValue)`. Have the existing no-arg
        versions delegate to them with `MIN_SOFT_IRON`/`MAX_SOFT_IRON`, so other tests are unaffected.
  - [x] Task 1.4. In both `testCalibrateFor{General,CommonAxis}CaseWithLargeNumberOfMeasurementsAndNoise`:
        - Generate `mm` with the noisy range.
        - Replace `LARGE_ABSOLUTE_ERROR` with `NOISY_HARD_IRON_ABSOLUTE_ERROR` and `5.0 * VERY_LARGE_ABSOLUTE_ERROR`
          with `NOISY_SOFT_IRON_ABSOLUTE_ERROR`, in both the `continue` checks and the `assertTrue` checks.
        - Wrap `calibrator.calibrate()` in `try { … } catch (final CalibrationException e) { continue; }`, and drop
          `CalibrationException` from the method's `throws` clause if it becomes unused.
  - [x] Task 1.5. Keep the rest of each test (`assertEstimatedResult`, covariance and chi-square assertions) as is.
        Run both tests through the `iru-gate-runner` agent (`Skill iru-java-test`, selector
        `KnownPositionAndInstantMagnetometerCalibratorTest#testCalibrateFor*WithLargeNumberOfMeasurementsAndNoise`).
        If the covariance/chi-square assertions fail with the new inputs, re-measure before loosening anything.
- [x] Task 2. Guard the robust retry loops in `RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibratorTest` _(java)_ — implemented (test-only change in RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibratorTest.java, 3 call sites guarded); group validated: `*Known*MagnetometerCalibratorTest` (40 classes) and full `mvn clean test` (14025 tests) green; coverage n/a (test-only change, no `src/main` class touched); no new Checkstyle/PMD/SpotBugs issues (analyzers do not cover test sources).
  - [x] Task 2.1. In `…/magnetometer/RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibratorTest.java`,
        wrap the unguarded `calibrator.calibrate()` in `try/catch (CalibrationException) { continue; }`, mirroring
        `testCalibrateGeneralNoNoiseInlier`:
        - `testCalibrateCommonAxisNoNoiseInlier`, line 8511: the failing test.
        - `testCalibrateCommonAxisWithInlierNoise`, line 8699.
        - `testCalibrateGeneralNoRefinement`, line 8791.
  - [x] Task 2.2. Remove `CalibrationException` from those methods' `throws` clauses where it becomes unused. Run the
        class through `iru-gate-runner` (`iru-java-test`, selector `RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibratorTest`).
- [x] Task 3. Guard the robust retry loops in the other `*RobustKnown*MagneticFluxDensityNorm*` tests _(java)_ — 24 call sites guarded in the nine listed `…MagneticFluxDensityNormMagnetometerCalibratorTest.java` files. Group validation also exposed a pre-existing flaky `assertTrue(getEstimatedQ() > 0.0)` in `RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorTest#testCalibrateGeneralNoRefinement` (measured: ~10% of valid iterations have chi-square 100-500 with 1 dof, so P rounds to 1.0 and Q = 0.0 exactly); relaxed to `>= 0.0` with a comment. License header added to `LMedSRobustKnownPositionAndInstantMagnetometerCalibratorTest.java` (Task 4 file, was missing). group validated: `*Known*MagnetometerCalibratorTest` (40 classes) and full `mvn clean test` (14025 tests) green; coverage n/a (test-only change, no `src/main` class touched); no new Checkstyle/PMD/SpotBugs issues (analyzers do not cover test sources).
  - [x] Task 3.1. Apply the same `try/catch (CalibrationException) { continue; }` pattern at these unguarded call
        sites (line numbers as of `develop` @ `8f6dbd4`; re-locate by method name if they shift). In each
        `MagneticFluxDensityNormMagnetometerCalibratorTest.java` file:

        | Prefix | Call sites |
        |---|---|
        | `LMedSRobustKnownHardIron…` | `testCalibrateCommonAxisNoNoiseInlier` 6759, `testCalibrateCommonAxisWithInlierNoise` 6936, `testCalibrateGeneralNoRefinement` 7023 |
        | `LMedSRobustKnown…` | `testCalibrateCommonAxisNoNoiseInlier` 8302, `testCalibrateCommonAxisWithInlierNoise` 8490, `testCalibrateGeneralNoRefinement` 8582 |
        | `MSACRobustKnownHardIron…` | `testCalibrateCommonAxisNoNoiseInlier` 6760, `testCalibrateCommonAxisWithInlierNoise` 6937, `testCalibrateGeneralNoRefinement` 7024 |
        | `MSACRobustKnown…` | `testCalibrateCommonAxisNoNoiseInlier` 8300, `testCalibrateCommonAxisWithInlierNoise` 8488, `testCalibrateGeneralNoRefinement` 8580 |
        | `PROMedSRobustKnownHardIron…` | `testCalibrateCommonAxisNoNoiseInlier` 12405, `testCalibrateCommonAxisWithInlierNoise` 12596 |
        | `PROMedSRobustKnown…` | `testCalibrateCommonAxisNoNoiseInlier` 15398, `testCalibrateCommonAxisWithInlierNoise` 15598, `testCalibrateGeneralNoRefinement` 15696 |
        | `PROSACRobustKnownHardIron…` | `testCalibrateCommonAxisNoNoiseInlier` 12920, `testCalibrateCommonAxisWithInlierNoise` 13111, `testCalibrateGeneralNoRefinement` 13205 |
        | `PROSACRobustKnown…` | `testCalibrateCommonAxisWithInlierNoise` 15973, `testCalibrateGeneralNoRefinement` 16071 |
        | `RANSACRobustKnownHardIron…` | `testCalibrateCommonAxisWithInlierNoise` 7156, `testCalibrateGeneralNoRefinement` 7243 |
  - [x] Task 3.2. For each guarded test, check that the loop body after `calibrate()` only uses `continue` or
        `break` (no state that must be reset), and that the final `assertTrue(numValid > 0)` is kept. Remove
        `CalibrationException` from `throws` clauses where it becomes unused.
  - [x] Task 3.3. Run these nine classes through `iru-gate-runner` (`iru-java-test`, selector
        `*RobustKnown*MagneticFluxDensityNormMagnetometerCalibratorTest`). — targeted run: 9 classes, 0 failures/errors.
- [x] Task 4. Guard the robust retry loops in the `*RobustKnown*PositionAndInstant*` tests _(java)_ — 29 call sites guarded in the ten `*RobustKnown*PositionAndInstantMagnetometerCalibratorTest` files; missing license header added to `LMedSRobustKnownPositionAndInstantMagnetometerCalibratorTest.java`; group validated: `*Known*MagnetometerCalibratorTest` (40 classes) and full `mvn clean test` (14025 tests) green; coverage n/a (test-only change, no `src/main` class touched); no new Checkstyle/PMD/SpotBugs issues (analyzers do not cover test sources).
  - [x] Task 4.1. Apply the same pattern at these unguarded call sites. In each
        `PositionAndInstantMagnetometerCalibratorTest.java` file:

        | Prefix | Call sites |
        |---|---|
        | `LMedSRobustKnownHardIron…` | `testCalibrateGeneralNoNoiseInlier` 6660, `testCalibrateCommonAxisNoNoiseInlier` 6743, `testCalibrateCommonAxisWithInlierNoise` 6913, `testCalibrateGeneralNoRefinement` 6997 |
        | `LMedSRobustKnown…` | `testCalibrateGeneralNoNoiseInlier` 7916, `testCalibrateCommonAxisNoNoiseInlier` 8000, `testCalibrateCommonAxisWithInlierNoise` 8174, `testCalibrateGeneralNoRefinement` 8259 |
        | `MSACRobustKnownHardIron…` | `testCalibrateGeneralNoNoiseInlier` 6659, `testCalibrateCommonAxisNoNoiseInlier` 6733, `testCalibrateCommonAxisWithInlierNoise` 6904 |
        | `MSACRobustKnown…` | `testCalibrateGeneralNoNoiseInlier` 8075, `testCalibrateCommonAxisNoNoiseInlier` 8161, `testCalibrateCommonAxisWithInlierNoise` 8340 |
        | `PROMedSRobustKnownHardIron…` | `testCalibrateGeneralNoNoiseInlier` 12247, `testCalibrateCommonAxisNoNoiseInlier` 12334, `testCalibrateCommonAxisWithInlierNoise` 12517, `testCalibrateGeneralNoRefinement` 12607 |
        | `PROMedSRobustKnown…` | `testCalibrateGeneralNoNoiseInlier` 15065, `testCalibrateCommonAxisNoNoiseInlier` 15159, `testCalibrateGeneralNoRefinement` 15452 |
        | `PROSACRobustKnownHardIron…` | `testCalibrateGeneralNoNoiseInlier` 12694, `testCalibrateCommonAxisNoNoiseInlier` 12786, `testCalibrateCommonAxisWithInlierNoise` 12970 |
        | `PROSACRobustKnown…` | `testCalibrateCommonAxisNoNoiseInlier` 15518, `testCalibrateGeneralNoRefinement` 15811 |
        | `RANSACRobustKnownHardIron…` | `testCalibrateGeneralNoNoiseInlier` 6899, `testCalibrateGeneralNoRefinement` 7242 |
        | `RANSACRobustKnown…` | `testCalibrateCommonAxisWithInlierNoise` 8607 |
  - [x] Task 4.2. Same checks as Task 3.2. Run these ten classes through `iru-gate-runner` (`iru-java-test`,
        selector `*RobustKnown*PositionAndInstantMagnetometerCalibratorTest`).
- [x] Task 5. Audit the sibling `Known*` noisy tests for vacuous tolerances _(java)_ — all six noisy tests had vacuous tolerances; fixed in `KnownHardIronPositionAndInstantMagnetometerCalibratorTest`, `KnownMagneticFluxDensityNormMagnetometerCalibratorTest`, `KnownHardIronMagneticFluxDensityNormMagnetometerCalibratorTest` (soft-iron range ±0.05, tolerances 2e-4 / 3e-3 soft iron and 1e-7 T hard iron, ~5-8x measured max error); group validated: `*Known*MagnetometerCalibratorTest` (40 classes) and full `mvn clean test` (14025 tests) green; coverage n/a (test-only change, no `src/main` class touched); no new Checkstyle/PMD/SpotBugs issues (analyzers do not cover test sources).
  - [x] Task 5.1. In `KnownHardIronPositionAndInstantMagnetometerCalibratorTest`,
        `KnownMagneticFluxDensityNormMagnetometerCalibratorTest` and
        `KnownHardIronMagneticFluxDensityNormMagnetometerCalibratorTest`, locate every
        `*WithLargeNumberOfMeasurementsAndNoise` (or similarly noisy) test, and record the tolerance each one uses
        against the generated value ranges.
  - [x] Task 5.2. Where a hard-iron tolerance is ≥ the hard-iron range (e.g. `LARGE_ABSOLUTE_ERROR = 5e-5` vs `±1e-5`),
        or a soft-iron tolerance is ≥ the soft-iron range:
        - Measure the actual estimation error in a temporary local run outside the repository (as done for Task 1).
        - Apply the same treatment as Task 1: test-local realistic soft-iron range and test-local tolerances at
          about 5–10× the max measured error.
        - Leave a test unchanged if measurements show its tolerance is already meaningful.
  - [x] Task 5.3. Run the three classes through `iru-gate-runner` (`iru-java-test`). — targeted run: 82 + 81 + 78 tests, 0 failures.

### Group 2 — Robust inlier-noise tests (Parallelizable: yes, single task)

Added after the first `/iru-code` run stopped at Task 6.2. It is numbered Task 7 so the existing task numbers stay
stable, and it must land before Group 3 re-runs Task 6.2.

Revised after the second run stopped on Task 7.1's divergence check. With a realistic soft-iron range, the
LMedS/PROMedS non-`HardIron` inlier-noise tests still converged in 0% of attempts. In a scratch copy with
`MAGNETOMETER_NOISE_STD = 2e-9` and `OUTLIER_ERROR_FACTOR = 100000.0`, LMedS PositionAndInstant General converged
91% (median errors 6.4e-8 T and 1.8e-3) and PROMedS Norm General converged 92%. Decision from the user: lower the
test noise class-wide to 2e-9 first (matching the sibling `Known*` tests and Task 1), then apply the realistic
soft-iron range and measured tolerances. Measurements from the second run are in the scratchpad `stats_*.txt`
files.

- [x] Task 7. Make the robust inlier-noise tests meaningful and reliable _(java)_ — implemented in the 20 robust
  `*RobustKnown*PositionAndInstant*` / `*RobustKnown*MagneticFluxDensityNorm*` test classes; group validation passed: license headers compliant (24/24), full `mvn clean test` green (14025 tests, 0 failures), no production code changed (coverage of src/main unaffected), no new Checkstyle/PMD/SpotBugs issues (test sources not analyzed).
  - [x] Task 7.1. Lower the noise class-wide in the 20 robust `*RobustKnown*PositionAndInstant*` and
        `*RobustKnown*MagneticFluxDensityNorm*` test classes (RANSAC, MSAC, PROSAC, PROMedS and LMedS, each with and
        without `HardIron`, under `…/magnetometer/`):
        - In every one of these classes that declares `MAGNETOMETER_NOISE_STD = 200e-9`, change it to `2e-9`.
        - In the same classes, multiply `OUTLIER_ERROR_FACTOR` by 100, so the outlier noise
          (`OUTLIER_ERROR_FACTOR * MAGNETOMETER_NOISE_STD`) and the outlier/inlier separation stay unchanged.
          For example, a class with `OUTLIER_ERROR_FACTOR = 1000.0` becomes `100000.0`.
        - Adjust any robust `THRESHOLD` (or other noise-scaled constant) in those classes only if the measurements
          show it no longer separates inliers from outliers. Record each one changed.
        - Leave the `*RobustKnown*Frame*` / `*HardIronAndFrame*` classes unchanged: they are out of scope for #46 and
          passed in every run.
        - Then run every class changed here in full through `iru-gate-runner` (`iru-java-test`), because these are
          class-wide constants. A test that newly fails must be measured and fixed within this task, not loosened
          blindly.
        — 13 classes changed to `MAGNETOMETER_NOISE_STD = 2e-9` with `OUTLIER_ERROR_FACTOR` 1000.0 → 100000.0 or
          700.0 → 70000.0; the other 7 were already at 2e-9. `THRESHOLD` kept at 1e-9 everywhere (inliers converge in
          86–100% of attempts). All 20 classes pass in full (2144 tests, 0 failures).
  - [x] Task 7.2. Give each `testCalibrateGeneralWithInlierNoise` and `testCalibrateCommonAxisWithInlierNoise` in
        those 20 classes (40 tests) a realistic soft-iron range and measured tolerances:
        - Add test-local constants `MIN_NOISY_SOFT_IRON = -0.05`, `MAX_NOISY_SOFT_IRON = 0.05`, and
          `generateSoftIronGeneral(min, max)` / `generateSoftIronCommonAxis(min, max)` overloads. The no-arg versions
          delegate to the new overloads, exactly as in Task 1.3. Generate `mm` with the noisy range in these tests.
        - Measure the hard- and soft-iron estimation error over at least 200 attempts per test, in a temporary local
          copy outside the repository. Set test-local tolerances to about 5–10× the error of converged attempts
          (90th percentile or max, whichever is more robust). Both tolerances must be tighter than the generated
          ranges: `±1e-5` T hard iron and `±0.05` soft iron.
          Tolerances may be shared per class (General vs. CommonAxis) when the measurements allow it.
        - Wrap `calibrate()` in `try/catch (CalibrationException) { continue; }` where not already done.
        - Stop and report only if a test still converges in fewer than 50% of its attempts at 2e-9 noise. That
          would point to a real calibrator convergence problem, which is out of scope here.
        — Measured 210–600 attempts per test (soft-iron range ±0.05): every test converges in ≥86% of attempts.
          Tolerances: hard iron 1e-6 (LMedS/PROMedS) or 1e-7 (MSAC/PROSAC/RANSAC); soft iron 2e-2 (LMedS/PROMedS,
          and General for MSAC/PROSAC/RANSAC), 2e-3 (CommonAxis for MSAC/PROSAC/RANSAC, and all HardIron
          LMedS/PROMedS), 5e-4 (HardIron MSAC/PROSAC/RANSAC). Unused `VERY_LARGE_ABSOLUTE_ERROR` (and, in the
          non-HardIron Norm classes, `LARGE_ABSOLUTE_ERROR`) removed.
  - [x] Task 7.3. Through `iru-gate-runner` (`iru-java-test`):
        - Run each class changed in Tasks 7.1–7.2 in full.
        - Run each of the 40 inlier-noise tests 30 times in a row, using a comma-separated `-Dtest` selector, and
          report the failure count. It must be 0.
        — 20 classes in full: 2144 tests, 0 failures. 40 inlier-noise tests × 30 runs (comma selector): 0 failures.

### Group 3 — Flakiness and full-suite validation (Parallelizable: yes, single task)

- [x] Task 6. Verify the fixes hold under repetition and the full suite stays green _(java)_ — verification only (no src/main classes touched, so Javadoc/coverage/quality gates N/A); package and full suite green after Task 7.
  - [x] Task 6.1. Through `iru-gate-runner`, run the three originally failing tests 50 times in a row locally, e.g.
        a loop of
        `mvn -q test -Dtest='KnownPositionAndInstantMagnetometerCalibratorTest#testCalibrateFor*WithLargeNumberOfMeasurementsAndNoise,RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibratorTest#testCalibrateCommonAxisNoNoiseInlier'`,
        reporting the failure count (must be 0).
        — 50/50 runs passed, 0 failures (3 tests per run). Note: the original `+` separator only ran 2 tests (Surefire
        treats it as another method of the first class); a comma is required.
  - [x] Task 6.2. Run the whole `com.irurueta.navigation.inertial.calibration.magnetometer` test package once through
        `iru-gate-runner` (`iru-java-test`), then the full `mvn test` suite, and confirm 0 failures and 0 errors.
        Re-run after Task 7 lands.
        — After Task 7: package run 45 classes, 4102 tests, 0 failures, 0 errors, 0 skipped. Full `mvn clean test`:
        14025 tests, 0 failures, 0 errors, 0 skipped, BUILD SUCCESS (8:23 min).
  - [x] Task 6.3. Confirm `git diff --stat develop` lists only files under `src/test/`.
        — Re-confirmed after Task 7: 25 files changed vs `develop` (1662 insertions, 450 deletions) = 24 test files under
          `…/magnetometer/` plus `.secrets.baseline` (`generated_at` timestamp only); `src/main` unchanged.

## Final verification outcome

- Full local verification green (full `mvn test`: 14025 tests, 0 failures); quality delta 0 vs baseline (125 / 456
  pre-existing issues, none introduced); security gate never fired.
- Pre-existing code-quality issues remaining: user chose "File issues". Filed on GitHub as #47 (Checkstyle),
  #48 (PMD) and #49 (SpotBugs).
