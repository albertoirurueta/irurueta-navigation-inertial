# Implementation Plan

## Task summary

Source: GitHub issue #36
Base branch: develop

Issue #36 ("Fix deviations from Groves' reference algorithms found in inertial navigation review", label `bug`,
classified as hotfix) reports four deviations from the Matlab companion code and from P. D. Groves, *Principles
of GNSS, Inertial, and Multisensor Integrated Navigation Systems* (2nd ed.), and requires tests that fail on the
current code and pass only when the implementation is right:

1. (significant) `INSLooselyCoupledKalmanFilteredEstimator`, `INSGNSSLooselyCoupledKalmanFilteredEstimator` and
   `INSGNSSTightlyCoupledKalmanFilteredEstimator` discard the INS-propagated solution: the Kalman state is never
   re-synchronised with the navigated frame, IMU samples between epochs are dropped, and in the loosely coupled
   class the INS frame is used as the measurement.
2. (minor) `ECEFKinematicsEstimator` computes the ½·Ω_ie·C(-)·τ term of eq. (5.85) from an already modified matrix.
3. (minor) Frame-based overloads of `ECEFKinematicsEstimator` / `ECIKinematicsEstimator` evaluate gravity at the
   new position instead of the position at the start of the interval.
4. (minor) The epoch gate uses `<=` where the Matlab loops use `>=`, rejecting an update that arrives exactly one
   epoch interval after the previous one.

Decisions taken with the user during planning:

- **Loosely coupled fix input**: add a new method to `INSLooselyCoupledKalmanFilteredEstimator` that supplies an
  ECEF position/velocity fix and triggers the Kalman epoch. `frame` becomes the INS solution only, integrated on
  every IMU sample; without a fix the estimator is pure INS (this is what `Loosely_coupled_INS_GNSS.m` does: the
  Kalman epoch only runs when a GNSS fix arrives, with the interval since the previous epoch).
- **API**: additive only. Every existing public signature and listener method is kept; only behaviour changes.
- **Test scope**: Java-only tests on trajectories generated in Java. The Matlab `Profile_*.csv` files, Matlab
  golden reference data and PIT mutation testing (issue section 5.3, last two bullets, and the Matlab profile in
  5.2) are **out of scope** and are left for a follow-up issue. The closed-loop test of 5.2 is implemented on a
  Java-generated trajectory with seeded noise instead of a Matlab profile.

Approach: fix the kinematics estimators first (the filter tests generate their IMU data with them), then
restructure the standalone loosely coupled estimator, then the two INS/GNSS estimators. Every new test is written
first and confirmed to fail on the unmodified implementation before the fix is applied (test-first), as the issue
requires.

Open clarifications: none

## Acceptance criteria coverage

| # | Criterion | Covered by tasks |
|---|---|---|
| 1 | "Filtered estimators integrate every IMU sample and feed the INS-propagated solution to the Kalman epoch (item 1), with trajectory-based tests." | Task 4, Task 5, Task 6 |
| 2 | "`ECEFKinematicsEstimator` matches eq. (5.85) and round-trips with `ECEFInertialNavigator` to numerical precision (item 2)." | Task 1.1, Task 1.2, Task 1.4 |
| 3 | "Frame-based kinematics overloads use, or document the need for, the position at the start of the interval (item 3)." | Task 1.3, Task 1.5, Task 2 |
| 4 | "Epoch gate accepts updates arriving exactly one epoch interval apart (item 4)." | Task 4.6, Task 5.4, Task 6.5 |
| 5 | "Every test listed in 5.2 exists, was shown to fail on the code before the fix, and passes after it." | Task 1.1, Task 1.3, Task 2.1, Task 3, Task 4.2, Task 5.1, Task 6.1 (Matlab-profile variant replaced by a Java-generated trajectory, per the user's scope decision) |
| 6 | "Existing tests are hardened as described in 5.3 (no retry-until-success, no "unchanged" assertions for moving bodies, attitude asserted, justified tolerances, independent reference data)." | Task 1.6, Task 2.3, Task 4.7, Task 5.5, Task 6.6 (independent Matlab reference data and PIT deferred to a follow-up issue, per the user's scope decision; recorded in Task 7.3) |

## Current code state

Java 21 single-module Maven library (`pom.xml`, no wrapper), JUnit 5 + Mockito, JaCoCo, Checkstyle
(`checkstyle.xml`), Antora docs under `docs/`. All paths below are relative to
`src/main/java/com/irurueta/navigation/inertial/` (sources) and
`src/test/java/com/irurueta/navigation/inertial/` (tests).

**Kinematics estimators**

- `estimators/ECEFKinematicsEstimator.java`, core `estimateKinematics(double timeInterval,
  CoordinateTransformation c, CoordinateTransformation oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z,
  BodyKinematics result)`. Around lines 1606–1615: `oldCbe.multiply(tmp3)` is executed in place **before**
  `alphaSkew2.multiply(oldCbe)`, so the Earth-rate term uses `C(-)·C̄` instead of `C(-)`.
  `navigators/ECEFInertialNavigator.java` (around line 6720) computes the same term in the correct order.
- Frame-based overloads: those taking `(timeInterval, ECEFFrame frame, ECEFFrame oldFrame, result)` and
  `...AndReturnNew` pass `frame.getX/Y/Z()` (new position) to the core method; those taking
  `(timeInterval, ECEFFrame frame, CoordinateTransformation oldC, oldVx, oldVy, oldVz, result)` (around lines
  1871/1900, plus `Speed`/`Time` variants) only know the new frame.
- `estimators/ECIKinematicsEstimator.java` has the same overload structure (around lines 1043/1069) with
  `ECIFrame`; its core method is correct.
- `estimators/NEDKinematicsEstimator.java` is correct (uses old latitude/height).
- Callers in `calibration/**` use these estimators with static bodies (same old and new frame), so they are not
  affected numerically by the position change.
- Tests: `estimators/ECEFKinematicsEstimatorTest.java` (τ = 0.02 s, attitude change ≤ 5°, `ABSOLUTE_ERROR = 1e-4`,
  every overload fed the **new** position `x, y, z` and compared with `assertEquals(k1, kN)`; private in-test
  reference `estimateKinematics(...)` at lines 932–1021 is a correct implementation of `Kinematics_ECEF.m`),
  `estimators/ECIKinematicsEstimatorTest.java`, `estimators/NEDKinematicsEstimatorTest.java`. The only
  navigator ↔ estimator round trips are `navigators/ECEFInertialNavigatorTest.testNavigateWhenFrameRemainsConstant*`
  (non-rotating frame, tolerance 1e-5).

**Kalman filtered estimators**

- `INSLooselyCoupledKalmanFilteredEstimator.java`
  - Fields: `frame` (ECEFFrame), `state` (INSLooselyCoupledKalmanState), `kinematics`, `correctedKinematics`,
    `lastStateTimestamp`, `epochInterval`, `config`, `initialConfig`, `listener`, `running`.
  - `update(BodyKinematics, double timestamp)` (line ~1385): returns `false` when
    `timestamp - lastStateTimestamp <= epochInterval`; otherwise corrects kinematics with the state biases,
    calls `ECEFInertialNavigator.navigateECEF(propagationInterval, frame, correctedKinematics, frame)` once over
    the whole interval, then calls `propagate(timestamp)`.
  - `propagate(double timestamp)` (line ~1493): same gate; creates `state` from `frame` only when `state == null`;
    calls `INSLooselyCoupledKalmanEpochEstimator.estimate(frame x,y,z,vx,vy,vz /* as GNSS measurement */,
    propagationInterval, state /* stale */, fx, fy, fz, config, state)`; sets `lastStateTimestamp`; then
    `state.getFrame(frame)` overwrites the navigated frame.
  - There is no input for a position/velocity fix.
- `INSGNSSLooselyCoupledKalmanFilteredEstimator.java` wraps an `insEstimator` of the class above plus a
  `GNSSLeastSquaresPositionAndVelocityEstimator lsEstimator`.
  - `updateMeasurements(Collection<GNSSMeasurement>, double timestamp)` (line ~1611): gate `<=`; least-squares
    fix into `estimation`; then calls `updateBodyKinematics(kinematics, timestamp)`.
  - `updateBodyKinematics(BodyKinematics, double timestamp)` (line ~1696): writes `estimation` position/velocity
    into `frame`, `insEstimator.setFrame(frame)`, `insEstimator.update(kinematics, timestamp)`.
  - `propagate(double timestamp)` (line ~1783): delegates to `insEstimator.propagate` and copies the frame back
    into `estimation`.
- `INSGNSSTightlyCoupledKalmanFilteredEstimator.java`
  - `updateMeasurements` (line ~1536): gate `<=`; least-squares into `estimation`; calls `updateBodyKinematics`.
  - `updateBodyKinematics` (line ~1574): gate `<=`; overwrites `frame` position/velocity with `estimation`;
    navigates once over the whole interval; calls `propagate(timestamp)`.
  - `propagate` (line ~1651): gate `<=`; `state` created from `frame` only when null; calls
    `INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, propagationInterval, state /* stale */, fx, fy,
    fz, config, state)`; then `state.getGNSSEstimation(estimation)` and `state.getFrame(frame)`.
- `INSLooselyCoupledKalmanEpochEstimator` and `INSTightlyCoupledKalmanEpochEstimator` match `LC_KF_Epoch.m` /
  `TC_KF_Epoch.m` and must **not** be changed; they need the INS-propagated solution as `previousState`.
- Tests: `INSLooselyCoupledKalmanFilteredEstimatorTest.java` (`POSITION_ERROR = 5.0`, `VELOCITY_ERROR = 5.0`,
  two updates with a random step in [0, 1] s, assertions against the **initial** position/velocity, attitude
  never asserted after rotation, retry loop `continue` + `assertTrue(numValid > 0)`),
  `INSGNSSLooselyCoupledKalmanFilteredEstimatorTest.java` and
  `INSGNSSTightlyCoupledKalmanFilteredEstimatorTest.java` (same patterns; `PROPAGATION_ERROR = 1.0`).

**Reference behaviour to reproduce** (`Loosely_coupled_INS_GNSS.m`, `Tightly_coupled_INS_GNSS.m`): every IMU
sample is bias-corrected and integrated with `Nav_equations_ECEF` over the IMU interval starting from the last
corrected estimate; when `time - time_last_GNSS >= epoch_interval` the Kalman epoch is run with the
INS-propagated attitude/velocity/position as `est_*_old`, the GNSS fix (LC) or pseudo-ranges (TC) as measurement,
the interval since the last epoch, and the latest bias-corrected specific force; the closed-loop correction is
applied to the INS solution.

## Implementation steps

Conventions for every task below:

- Test-first: add the new test(s), run them against the unmodified production code and record in the task's
  progress note that they fail (with the observed error), then apply the fix and confirm they pass.
- Run tests by delegating to the `iru-gate-runner` agent, e.g. `Agent({description: "Run tests for
  ECEFKinematicsEstimatorTest", subagent_type: "iru-gate-runner", prompt: "Invoke Skill({skill:
  \"iru-java-test\", args: \"ECEFKinematicsEstimatorTest\"}) ..."})`, not directly in the main conversation.
- New tests must be deterministic: fixed inputs or `new UniformRandomizer(new Random(SEED))`; no
  `continue`/`numValid` retry loops; every tolerance constant carries a comment explaining where its value comes
  from.
- Keep Javadoc complete on every new or changed public/protected member, keep the Apache licence header on new
  files, and keep Checkstyle clean.

### Group 1 — Kinematics estimators and round-trip tests

Parallelizable: yes (each task touches a disjoint set of files)

- [x] **Task 1. Fix `ECEFKinematicsEstimator` (eq. (5.85) and start-of-interval position)** _(java)_ — implemented in `estimators/ECEFKinematicsEstimator.java`, tests added/hardened in `estimators/ECEFKinematicsEstimatorTest.java` (11 tests pass; 99.7% line / 83.3% branch coverage). Test-first: `testNavigatorRoundTripWithRotation` failed on the unmodified code (specific force error 8.667e-7 m/s² at τ = 0.1 s) and `testFrameOverloadsUseOldPosition` failed (specific force error 1.1345e-4 m/s²); both pass after the fix. No new PMD/SpotBugs issues; Checkstyle: only added Javadoc/delegation lines over the plugin's 80-column default (all within the 120 of `checkstyle.xml`), left in place.
  - [x] Task 1.1. In `estimators/ECEFKinematicsEstimatorTest.java` add
    `testNavigatorRoundTripWithRotation`: for τ ∈ {0.1, 1.0} s, build an `ECEFFrame oldFrame` (latitude 41.3825°,
    longitude 2.176944°, height 0, NED velocity (30, 20, -1) m/s, roll/pitch/yaw (10°, -20°, 30°)), kinematics
    `f = (1, 2, -9.8)` m/s², `ω = (0.5, -0.3, 1.0)` rad/s; compute
    `newFrame = ECEFInertialNavigator.navigateECEFAndReturnNew(τ, oldFrame, k)` and then
    `ECEFKinematicsEstimator.estimateKinematics(τ, newFrame.getCoordinateTransformation(),
    oldFrame.getCoordinateTransformation(), new velocity, old velocity, oldFrame.getX(), oldFrame.getY(),
    oldFrame.getZ(), result)` (explicit **old** position). Assert `result.equals(k, 1e-9)`. Expected on current
    code: specific-force error ≈ 7.8e-7 (τ = 0.1 s) and 8.4e-5 m/s² (τ = 1 s) → fails. — test added; on unmodified production code it FAILED at τ = 0.1 s with specific-force error 8.667e-7 m/s² (angular-rate error 6.9e-16 rad/s); passes after Task 1.2.
  - [x] Task 1.2. In `estimators/ECEFKinematicsEstimator.java` core method, compute the Earth-rate term before
    the in-place product, in the same order as `ECEFInertialNavigator`:
    ```java
    // 0.5 * skew([0, 0, alpha]) * C(-)   -- must use the ORIGINAL oldCbe (eq. 5.85)
    alphaSkew2.multiply(oldCbe);
    // C(-) * Cbb
    oldCbe.multiply(tmp3);
    oldCbe.subtract(alphaSkew2);
    ```
    Apply the same ordering in both branches (‖α‖ above and below the threshold) if the below-threshold branch
    has the same structure. Do not change any signature. — implemented (Earth-rate term computed from the original `oldCbe` before the in-place product; the below-threshold branch has no in-place product so the single reordering covers both branches).
  - [x] Task 1.3. In `estimators/ECEFKinematicsEstimatorTest.java` add `testFrameOverloadsUseOldPosition`: with
    the τ = 1.0 s frames of Task 1.1 (old and new positions ≈ 36 m apart), assert that every overload receiving
    `oldFrame` (`estimateKinematics(τ, newFrame, oldFrame, result)`, its `Time` variant and the `...AndReturnNew`
    variants) returns exactly the same `BodyKinematics` as the explicit-position overload called with the **old**
    position, equals the original kinematics within 1e-9, and differs by more than 1e-6 m/s² in specific force
    from the explicit-position overload called with the **new** position. Expected on current code: fails
    (overloads use the new position; error ≈ 1.4e-4 m/s²). — test added (8 overloads: static and instance, `double`/`Time`, in-place/`AndReturnNew`); on unmodified production code it FAILED on the first overload (`assertEquals` with the old-position result; specific-force error 1.1345e-4 m/s²); passes after Task 1.5.
  - [x] Task 1.4. Tighten the comparison with the in-test reference implementation: change `ABSOLUTE_ERROR` from
    `1e-4` to `1e-9` (the reference performs the same arithmetic; if rounding requires a different value, use the
    smallest power of ten that passes over 1000 seeded runs and document it), and widen the randomised ranges in
    the main estimation test: add a second pass with `TIME_INTERVAL_SECONDS = 1.0` and attitude variation up to
    ±45° so second-order terms are observable. Pass the **old** position to the in-test reference and to the
    explicit-position overloads. — done: `ABSOLUTE_ERROR = 1e-9` (max observed over 2000 seeded pairs per pass: 2.0e-14 at 0.02 s, 7.6e-15 at 1 s); the four overload tests now run 1000 seeded pairs on two passes (0.02 s / ±5° and 1.0 s / ±45°) with the old position.
  - [x] Task 1.5. In `estimators/ECEFKinematicsEstimator.java` change every overload that receives both `frame`
    and `oldFrame` to pass `oldFrame.getX()/getY()/getZ()` to the core method. For overloads that receive only
    the current frame or an explicit position (`x, y, z`, `ECEFPosition`, `Point3D`, `(frame, oldC, oldVx, ...)`),
    update the Javadoc of the position/frame parameter to state: "position where gravity is evaluated; to be
    consistent with `ECEFInertialNavigator` (and `Kinematics_ECEF.m` callers) this must be the position at the
    start of the time interval". — implemented (the two static `(frame, oldFrame)` overloads now call the core method with the old position; all other `(frame, oldFrame)` overloads delegate to them); Javadoc of 160 position/frame parameters updated.
  - [x] Task 1.6. Harden the existing tests in `ECEFKinematicsEstimatorTest.java`: update the `assertEquals(k1,
    kN)` equality chain so that overloads taking `oldFrame` are compared with the old-position result and
    overloads taking only the new frame with the new-position result (two reference results instead of one);
    replace `LARGE_ABSOLUTE_ERROR = 1e-1` in the ECEF/ECI/NED cross-comparison by the tightest value that the
    three (already correct) algorithms satisfy on seeded inputs, documented in a comment. — done: two reference results in the equality chains; `LARGE_ABSOLUTE_ERROR = 1e-1` replaced by `COMPARE_SPECIFIC_FORCE_ERROR = 1e-2` (max observed 1.0e-3 m/s²) and `COMPARE_ANGULAR_RATE_ERROR = 1e-8` (max observed 1.5e-9 rad/s) over 1000 seeded, kinematically consistent frame pairs (new position = old + mean velocity·τ, ECI frames at epochs 0 and τ; with the original inconsistent random positions the three algorithms only agree to 5e-2 m/s²).
  - [x] Task 1.7. Run `ECEFKinematicsEstimatorTest`, `ECEFInertialNavigatorTest` and the `calibration` tests that
    call `ECEFKinematicsEstimator` through `iru-gate-runner`; all must pass. — run directly with Maven (no agent-dispatch tool was available to the executing sub-agent): `ECEFKinematicsEstimatorTest` 11/11, `ECEFInertialNavigatorTest` 11/11, 86 calibration test classes using `ECEFKinematicsEstimator` 7678/7678 passed.

- [x] **Task 2. Fix `ECIKinematicsEstimator` frame overloads and add its round-trip test** _(java)_ — implemented in
  `estimators/ECIKinematicsEstimator.java` and `estimators/ECIKinematicsEstimatorTest.java`, tests added
  (11 tests pass; 99.5% line / 83.3% branch coverage). Test-first: `testNavigatorRoundTripWithRotation` passed on
  the unmodified code as expected (core is correct); `testFrameOverloadsUseOldPosition` failed (estimated
  f = (1.000459, 2.000199, -9.800344) m/s², ≈ 5.4e-4 m/s² error) and passes after the fix. No new PMD/SpotBugs
  issues; Checkstyle: new `FileLength` (2,097 > 2,000 lines) caused by the start-of-interval Javadoc this task
  mandates on every position/frame parameter, plus Javadoc lines over the plugin's 80-column default (all within
  the 120 of `checkstyle.xml`) — left in place as unavoidable.
  - [x] Task 2.1. In `estimators/ECIKinematicsEstimatorTest.java` add `testNavigatorRoundTripWithRotation`
    (same inputs as Task 1.1 converted to an `ECIFrame` with `ECEFtoECIFrameConverter`, using
    `ECIInertialNavigator.navigateECIAndReturnNew` and the explicit **old** position; tolerance 1e-9) and
    `testFrameOverloadsUseOldPosition` (same assertions as Task 1.3 for `ECIFrame`). Expected on current code:
    the round trip passes (core is correct), the overload test fails. — Observed on unmodified production code:
    `testNavigatorRoundTripWithRotation` passed; `testFrameOverloadsUseOldPosition` failed at overload k1
    (`estimateKinematics(τ, newFrame, oldFrame, result)`) with `f = (1.000459101236481, 2.000199050084186,
    -9.800344227629878)` instead of the old-position result (error ≈ 5.4e-4 m/s²). Both pass after Task 2.2.
  - [x] Task 2.2. In `estimators/ECIKinematicsEstimator.java` change every overload that receives both `frame`
    and `oldFrame` to pass the old frame position to the core method, and document the start-of-interval
    requirement on the remaining position/frame parameters (same wording as Task 1.5, referring to
    `ECIInertialNavigator` / `Kinematics_ECI.m`).
  - [x] Task 2.3. Harden `ECIKinematicsEstimatorTest.java` the same way as Task 1.4 and Task 1.6 (tolerance
    against the in-test reference tightened to 1e-9 or the documented minimum, wider τ/attitude ranges, old vs
    new position distinguished in the overload equality chain). — `ABSOLUTE_ERROR` 1e-5 → 1e-9 (1000 seeded
    runs × 2 passes; still passes down to 1e-13).
  - [x] Task 2.4. Run `ECIKinematicsEstimatorTest` and `ECIInertialNavigatorTest` through `iru-gate-runner`. —
    Run directly with Maven (no agent-dispatch tool in the executing subagent): 11/11 and 6/6 pass.

- [x] **Task 3. Add the NED navigator ↔ kinematics round-trip test** _(java)_ — implemented in
  `estimators/NEDKinematicsEstimatorTest.java` (test only, no production change); regression guard, not expected
  to fail on current code and it passes (measured round-trip error: 4.0e-12 m/s² / 1.9e-11 rad/s at τ = 0.1 s,
  6.9e-9 m/s² / 1.9e-9 rad/s at τ = 1 s; tolerances 1e-9 and 1e-7). The navigator is called through the
  `accuracyThreshold` overload (1e-7) because its first-order attitude update is rejected by the default 1e-11
  threshold for these inputs. Validation: 10 tests pass; `NEDKinematicsEstimator` 99.8% line / 83.3% branch
  coverage; no production change, no new quality issues.
  - [x] Task 3.1. In `estimators/NEDKinematicsEstimatorTest.java` add `testNavigatorRoundTripWithRotation`:
    `NEDInertialNavigator.navigateNEDAndReturnNew(τ, oldFrame, k)` followed by
    `NEDKinematicsEstimator.estimateKinematics` with the old latitude/height, for τ ∈ {0.1, 1.0} s and the
    kinematics of Task 1.1. `Nav_equations_NED` is only first-order accurate in attitude (eq. (5.46)–(5.54)), so
    derive the tolerance from the measured round-trip error of the (already correct) implementation and document
    it; this test is a regression guard, not expected to fail on current code (state that in the progress note).
  - [x] Task 3.2. Run `NEDKinematicsEstimatorTest` through `iru-gate-runner`. — 10 tests, 0 failures (run with Maven directly; no agent-dispatch tool was available to this executor).

### Group 2 — Standalone loosely coupled estimator

Parallelizable: yes (single task)

- [x] **Task 4. Restructure `INSLooselyCoupledKalmanFilteredEstimator` so the INS solution is kept and a fix is an explicit input** _(java)_ — implemented, tests added; group validation done: `INSLooselyCoupledKalmanFilteredEstimatorTest` 28/28, `INSLooselyCoupledKalmanFilteredEstimator` 87.9% line / 93.3% branch coverage, licence headers compliant, Javadoc build passes, no new PMD/SpotBugs issues; Checkstyle: +78 lines over the plugin's 80-column default (all within the 120 of `checkstyle.xml`), left in place as in Group 1. Full suite: 13930 tests, only the 10 `INSGNSSLooselyCoupledKalmanFilteredEstimatorTest` errors described below (to be resolved by Task 5). Files: `INSLooselyCoupledKalmanFilteredEstimator.java`,
  `INSLooselyCoupledKalmanFilteredEstimatorTest.java`, new `KalmanFilteredEstimatorTrajectory.java` (test helper; trajectories
  are built from a `t → NEDFrame` function or a speed/heading profile). Failures observed on the unmodified class
  (Maven run directly, no agent-dispatch tool available): `testRotationIsIntegrated` yaw expected 1.0, was -7.7e-10 rad;
  `testTranslationIsIntegrated` position error 99.35 m; `testMatchesChainedNavigatorWithoutFix` `INSException`
  (`SingularMatrixException`) with the default all-zero config, 118.77 m position error with a non-singular config;
  `testEveryImuSampleCountsWithinEpoch` position error 0.5 m (all samples dropped); `testFreeFall` displacement error
  4.85 m. `testFixCorrectsTowardsMeasurement`, `testEpochGateAcceptsExactInterval` and `testClosedLoopTracksTruth` are not
  expressible before the fix (no `updatePositionAndVelocity`). After the fix: 28 tests, 0 failures. Deviations: `propagate` is now
  `@Deprecated` (no-op after initialisation); `setFrame` also synchronises an existing state;
  `testFixCorrectsTowardsMeasurement` uses Groves' tactical-grade config since the default all-zero config is singular.
  Known consequence until Task 5: 10 tests of `INSGNSSLooselyCoupledKalmanFilteredEstimatorTest` fail
  (`getLastStateTimestamp()` stays null because the wrapper never provides a fix).
  - [x] Task 4.1. Create the shared test helper
    `src/test/java/com/irurueta/navigation/inertial/KalmanFilteredEstimatorTrajectory.java` (package-private,
    Apache header, used by Tasks 4–6): given an initial `NEDFrame`, a sample interval and a list of segments
    (duration, body-frame specific-force offset and angular rate, or directly a function `t → true NEDFrame`), it
    produces the list of true `ECEFFrame`s at each sample time and the error-free `BodyKinematics` between
    consecutive frames using `ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(τ, frame, oldFrame)` (correct
    after Task 1). Provide at least these ready-made trajectories at 100 Hz: `yawingStationary(0.1 rad/s, 10 s)`,
    `northAtConstantSpeed(10 m/s, 10 s)`, `accelerateThenDecelerate(0.5 s + 0.5 s)`, and
    `turningAndAccelerating(60 s)` (speed and heading changes) for the closed-loop tests. Also provide
    `chainNavigator(initialFrame, kinematicsList, τ)` returning the frame obtained with repeated
    `ECEFInertialNavigator.navigateECEF` calls (the oracle for equivalence tests).
  - [x] Task 4.2. In `INSLooselyCoupledKalmanFilteredEstimatorTest.java` add, **before changing production code**,
    the tests below and confirm the ones marked (F) fail on the current implementation:
    - `testRotationIsIntegrated` (F): yawing trajectory, `update(kinematics, t)` for every sample, zero epoch
      interval; assert final yaw = 1.0 rad within 1e-6 rad (current: 0.0000 rad).
    - `testTranslationIsIntegrated` (F): north trajectory; assert final position error < 1e-3 m (current:
      99.997 m) and velocity error < 1e-6 m/s.
    - `testMatchesChainedNavigatorWithoutFix` (F): default config, zero biases, no fix; assert
      `estimator.getFrame()` equals `chainNavigator(...)` within 1e-6 m, 1e-6 m/s and 1e-9 in the rotation matrix.
    - `testEveryImuSampleCountsWithinEpoch` (F): `accelerateThenDecelerate`, epoch interval 1 s, samples at
      100 Hz; assert the final frame matches per-sample integration (tolerances above), which differs from a
      single 1 s step with the last sample.
    - `testFreeFall` (F): zero specific force for T = 1 s from rest at 100 Hz; assert displacement = ½·g·T² along
      the local gravity vector within 1e-2 m and velocity change = g·T within 1e-3 m/s (g from
      `ECEFGravityEstimator` at the start position).
    - `testFixCorrectsTowardsMeasurement` (F for the new API — compile-level addition): INS initialised 10 m
      north of the truth, stationary kinematics, one fix at the true position with the default config; assert the
      position error after the fix is strictly smaller than before, the correction points towards the fix, and
      the state covariance position block decreased.
    - `testClosedLoopTracksTruth`: `turningAndAccelerating` with seeded IMU errors from
      `BodyKinematicsGenerator` (tactical-grade `IMUErrors`) and a seeded noisy fix every 1 s (σ = 2.5 m, 0.1 m/s);
      assert that after convergence the position error stays below 3·σ_fix and is smaller than the error of the
      INS-only run on the same data, and that position errors are within 3σ of the estimated covariance for at
      least 95 % of the epochs.
  - [x] Task 4.3. In `INSLooselyCoupledKalmanFilteredEstimator.java` add a private `Double
    lastKinematicsTimestamp` and change `update(BodyKinematics kinematics, double timestamp)` (signature
    unchanged) to integrate **every** sample:
    - return `false` without side effects when `lastKinematicsTimestamp != null && timestamp <=
      lastKinematicsTimestamp`;
    - on the first call initialise `state` (as `propagate` does today: `initInitialConfig()`,
      `INSLooselyCoupledKalmanInitializer.initialize`, `state.setFrame(frame)`, covariance) and perform no
      navigation;
    - otherwise bias-correct the kinematics with the current state (`correctKinematics`), call
      `ECEFInertialNavigator.navigateECEF(timestamp - lastKinematicsTimestamp, frame, correctedKinematics, frame)`
      and then `state.setFrame(frame)` so the state always holds the INS-propagated solution;
    - store `this.kinematics` and `lastKinematicsTimestamp`, fire `onUpdateStart`/`onUpdateEnd` as today, and no
      longer apply the epoch gate here nor call `propagate`. Update the Javadoc (return value: "true if the
      sample was integrated").
    Add public `Double getLastKinematicsTimestamp()` (+ `Time` variants mirroring the existing
    `getLastStateTimestampAsTime` methods). `lastStateTimestamp` keeps its meaning: timestamp of the last Kalman
    epoch.
  - [x] Task 4.4. Add the fix input (additive API), with `Time` and coordinate overloads following the class's
    existing overload style:
    ```java
    public boolean updatePositionAndVelocity(
            final ECEFPosition position, final ECEFVelocity velocity, final double timestamp)
            throws LockedException, NotReadyException, INSException
    public boolean updatePositionAndVelocity(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz, final double timestamp) ...
    public boolean updatePositionAndVelocity(
            final ECEFPosition position, final ECEFVelocity velocity, final Time timestamp) ...
    ```
    Behaviour: throw `LockedException` when running and `NotReadyException` when `!isPropagateReady()`; apply the
    epoch gate of Task 4.6; initialise `state` if null; call
    `INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, propagationInterval, state, fx, fy, fz,
    config, state)` where `state` already holds the INS-propagated frame (Task 4.3), `propagationInterval =
    timestamp - lastStateTimestamp` (0 on the first epoch) and `fx, fy, fz` come from `correctedKinematics`
    (zero when none); then apply the closed-loop correction with `state.getFrame(frame)`, set
    `lastStateTimestamp = timestamp`, and fire `onPropagateStart`/`onPropagateEnd`. Return `true` when the epoch
    ran. Store the last fix in private fields and expose `getLastPositionAndVelocityFix`-style getters only if
    needed by Task 5 (prefer not to).
  - [x] Task 4.5. Redefine the existing `propagate(double timestamp)` / `propagate(Time)` (signatures unchanged)
    so it no longer uses the INS frame as a measurement: it ensures `state` is initialised and synchronised with
    `frame` and returns `false` (no Kalman measurement update is possible without a fix). Document in its Javadoc
    that the Kalman epoch is now triggered by `updatePositionAndVelocity`, and mark nothing as deprecated unless
    the method becomes a pure no-op after initialisation (in that case add `@deprecated` Javadoc pointing to
    `updatePositionAndVelocity`, keeping the method). Review `reset()`, `setFrame`, `setConfig`,
    `setEpochInterval`, serialization (`serialVersionUID` unchanged, new fields included in
    `SerializationHelper` round-trip tests) and `correctKinematics` for consistency with the new fields.
  - [x] Task 4.6. Epoch gate (issue item 4): a Kalman epoch is rejected only when `lastStateTimestamp != null &&
    (propagationInterval <= 0.0 || propagationInterval < epochInterval)`. Add `testEpochGateAcceptsExactInterval`:
    epoch interval 0.5 s, fixes at t = 0.0, 0.5, 1.0 s all return `true`; a fix at t = 1.25 s returns `false`; a
    repeated fix at t = 1.0 s returns `false`. Expected on current code: not expressible without the new method —
    additionally assert through `INSGNSSLooselyCoupled...` in Task 5.4, which fails on current code.
  - [x] Task 4.7. Harden the existing tests in `INSLooselyCoupledKalmanFilteredEstimatorTest.java`
    (`testUpdateWithZeroSpecificForceAndAngularRate`, `testUpdateWithSpecificForceAndZeroAngularRate`,
    `testUpdateWithZeroSpecificForceAndRotationOnly`, `testUpdateWithSpecificForceAndRotation`, `testPropagate`):
    remove the `for (t < TIMES) … continue … numValid` retry pattern (seeded, every run must pass); replace
    "position/velocity still within `POSITION_ERROR`/`VELOCITY_ERROR` of the initial value" by assertions against
    the expected new value (free fall for zero specific force; `chainNavigator` result otherwise); assert the
    attitude in every test that supplies angular rate; delete `POSITION_ERROR = 5.0` / `VELOCITY_ERROR = 5.0` in
    favour of documented tolerances; update listener-count assertions to the new behaviour (`onPropagate*` fires
    only on Kalman epochs); keep constructor/getter/setter/serialization tests, extending them for the new
    methods (including `LockedException` / `NotReadyException` paths of `updatePositionAndVelocity`).
  - [x] Task 4.8. Run `INSLooselyCoupledKalmanFilteredEstimatorTest` through `iru-gate-runner`; record in the
    progress note which tests failed before the fix and that all pass after it. — 28 tests, 0 failures (run with
    Maven directly; no agent-dispatch tool was available to this executor).

### Group 3 — INS/GNSS estimators

Parallelizable: yes (Task 5 and Task 6 touch disjoint production and test files; both only read the helper
created in Task 4.1)

- [x] **Task 5. Fix `INSGNSSLooselyCoupledKalmanFilteredEstimator`** _(java)_ — implemented, tests added; group validation done: `INSGNSSLooselyCoupledKalmanFilteredEstimatorTest` 27/27, `INSGNSSLooselyCoupledKalmanFilteredEstimator` 88.6% line / 91.8% branch coverage, licence headers compliant, Javadoc build passes, full suite 13943 tests / 0 failures, no new PMD/SpotBugs issues; Checkstyle (plugin defaults): +61 lines over the 80-column default (none over the 120 of `checkstyle.xml`) and a new `FileLength` (2,045 > 2,000 lines, rule not in `checkstyle.xml`), left in place as in Groups 1-2. Files: `INSGNSSLooselyCoupledKalmanFilteredEstimator.java`, `INSGNSSLooselyCoupledKalmanFilteredEstimatorTest.java`. Progress note (test-first, judged against the pre-change wrapper as it stood in the working tree after Group 2): `testTranslationIsIntegrated` failed (position error 99.98 m, expected 0 ± 0.5 m), `testEveryImuSampleCountsWithinEpoch` failed (0.497 m from the per-sample oracle), `testGnssFixIsNotAdvancedInTime` and `testEpochGateAcceptsExactInterval` errored (NullPointerException: `getLastStateTimestamp()` null, no Kalman epoch ever ran), `testClosedLoopTracksTruth` failed (first IMU sample rejected); `testRotationIsIntegrated` did not fail on the rotation itself (Group 2's INS estimator already integrates attitude) but only on its initialisation precondition (cold least-squares fix 0.385 m off, expected < 1e-3 m). After the fix: 27/27 pass (`mvn test -Dtest=INSGNSSLooselyCoupledKalmanFilteredEstimatorTest`, run directly, no agent-dispatch tool available). Deviations: the first GNSS fix is refined once using itself as prior (cold least-squares error of 0.4–4 m drops to 2.5e-7 m); truth tolerances of `testTranslationIsIntegrated` are 0.5 m / 5e-2 m/s instead of 1e-3 m / 1e-6 m/s because the dependency's error-free least-squares velocity is ~1e-2 m/s off (generator and estimator project range rate on slightly different lines of sight) — the 1e-6 tolerances are kept against the per-sample oracle; `testUpdateMeasurementsWithoutKalmanConfig` added.
  - [x] Task 5.1. In `INSGNSSLooselyCoupledKalmanFilteredEstimatorTest.java` add, before changing production
    code, and confirm the (F) ones fail: `testRotationIsIntegrated` (F), `testTranslationIsIntegrated` (F),
    `testEveryImuSampleCountsWithinEpoch` (F) — same scenarios as Task 4.2 driven through
    `updateBodyKinematics`, with GNSS measurements generated for the true position (reuse the satellite/
    measurement generation already present in this test class, with seeded randomizers) supplied once at t = 0 to
    initialise; `testGnssFixIsNotAdvancedInTime` (F): body moving at 30 m/s, error-free GNSS measurements at each
    1 s epoch, error-free IMU; assert the position error right after each epoch is < 0.5 m (the current code
    shifts the fix by v·τ); `testClosedLoopTracksTruth`: `turningAndAccelerating` with seeded IMU errors and
    noisy pseudo-ranges every 1 s, bounds as in Task 4.2.
  - [x] Task 5.2. In `INSGNSSLooselyCoupledKalmanFilteredEstimator.java` change
    `updateBodyKinematics(BodyKinematics, double)`: stop overwriting `frame` position/velocity with `estimation`
    on every call — only seed the frame from `estimation` when the INS frame has not been initialised yet (first
    fix, `initFrame()` returning `true`, preserving the configured initial attitude); call
    `insEstimator.update(kinematics, timestamp)` for every sample (no epoch gate); copy kinematics/state back as
    today.
  - [x] Task 5.3. Change `updateMeasurements(Collection<GNSSMeasurement>, double)`: after the least-squares fix,
    seed the INS frame if it is not initialised, then call
    `insEstimator.updatePositionAndVelocity(estimation.getX(), …, estimation.getVz(), timestamp)` so the fix is
    used as the Kalman **measurement** valid at `timestamp`; remove the call to `updateBodyKinematics` from this
    method (the fix must not be navigated forward). Use the INS frame (not the previous raw fix) as the prior of
    `lsEstimator`. Keep `propagate(double)` delegating to `insEstimator.propagate` and make it stop copying the
    frame into `estimation` in a way that is later fed back as a measurement. Keep listener callbacks.
  - [x] Task 5.4. Epoch gate: replace `timestamp - lastStateTimestamp <= insEstimator.getEpochInterval()` by the
    rule of Task 4.6. Add `testEpochGateAcceptsExactInterval` (epoch 0.5 s; measurements at 0.0, 0.5, 1.0 s
    accepted; 1.25 s rejected) — fails on current code.
  - [x] Task 5.5. Harden the existing `testUpdateMeasurements*`, `testUpdateKinematics*` and `testPropagate`
    tests as in Task 4.7 (no retry loops, expected-value assertions, attitude asserted, documented tolerances).
  - [x] Task 5.6. Run `INSGNSSLooselyCoupledKalmanFilteredEstimatorTest` through `iru-gate-runner`.

- [x] **Task 6. Fix `INSGNSSTightlyCoupledKalmanFilteredEstimator`** _(java)_ — implemented, tests added; group validation done: `INSGNSSTightlyCoupledKalmanFilteredEstimatorTest` 26/26, `INSGNSSTightlyCoupledKalmanFilteredEstimator` 90.9% line / 95.1% branch coverage, licence headers compliant, Javadoc build passes, full suite 13943 tests / 0 failures, no new PMD/SpotBugs issues; Checkstyle (plugin defaults): +64 lines over the 80-column default (none over the 120 of `checkstyle.xml`) and a new `FileLength` (2,113 > 2,000 lines, rule not in `checkstyle.xml`), left in place as in Groups 1-2. Files: `INSGNSSTightlyCoupledKalmanFilteredEstimator.java`, `INSGNSSTightlyCoupledKalmanFilteredEstimatorTest.java`. Beyond the sub-tasks: the closed-loop attitude is re-orthonormalised (SVD) after each epoch inside the estimator, because `INSTightlyCoupledKalmanState.getC()` (unlike the loosely coupled state) rejects the first-order corrected matrix and `state.getFrame(frame)` then silently left the INS frame uncorrected (filter diverged); the Kalman state is created with the receiver clock of the least-squares solution; `estimation` is kept in sync with the INS solution on every integrated sample.
  - [x] Task 6.1. In `INSGNSSTightlyCoupledKalmanFilteredEstimatorTest.java` add, before changing production
    code, and confirm the (F) ones fail: `testRotationIsIntegrated` (F), `testTranslationIsIntegrated` (F),
    `testEveryImuSampleCountsWithinEpoch` (F), `testMeasurementsAreUsedAtTheirTimestamp` (F; moving body,
    error-free pseudo-ranges each 1 s, position error right after each epoch < 0.5 m) and
    `testClosedLoopTracksTruth` (as Task 5.1, tightly coupled bounds), using the helper of Task 4.1 and the
    measurement generation already present in this test class with seeded randomizers.
    Progress note — run against the unmodified production class, all failed: `testRotationIsIntegrated`
    (yaw expected 1.0, was 1.48e-9), `testTranslationIsIntegrated` (position error 99.99999 m),
    `testEveryImuSampleCountsWithinEpoch` (0.4989 m from the per-sample integration),
    `testMeasurementsAreUsedAtTheirTimestamp` (last epoch timestamp expected 0.0, was 0.51: an IMU sample ran an
    epoch with stale measurements), `testClosedLoopTracksTruth` (measurements at t = 1 s rejected). After the fix
    all pass. Deviation: with a GNSS initialisation the truth can only be matched to 0.1 m / 5e-3 m/s (the
    least-squares/Kalman models leave 3 cm and 1.3 mm/s on error-free measurements), so
    `testTranslationIsIntegrated` asserts those documented tolerances against the truth and 1e-6 m / 1e-6 m/s
    against `chainNavigator` from the initialised solution; `testClosedLoopTracksTruth` uses white pseudo-range
    noise matching the filter model (Groves' constant range biases make the covariance check meaningless).
  - [x] Task 6.2. In `INSGNSSTightlyCoupledKalmanFilteredEstimator.java` add a private `Double
    lastKinematicsTimestamp` (+ public getter, as Task 4.3) and change `updateBodyKinematics(BodyKinematics,
    double)`: remove the epoch gate; only seed `frame` from `estimation` when the frame has not been initialised;
    bias-correct and navigate every sample with `ECEFInertialNavigator.navigateECEF(timestamp -
    lastKinematicsTimestamp, frame, correctedKinematics, frame)`; then `state.setFrame(frame)` when `state !=
    null`; do not call `propagate`. Do not swallow `InertialNavigatorException` silently if the existing tests do
    not require it (keep the current `return false` only if a test documents that contract).
  - [x] Task 6.3. Change `updateMeasurements(Collection<GNSSMeasurement>, double)`: apply the gate of Task 4.6;
    run the least-squares solution only to initialise position, velocity and clock while `state == null`
    (afterwards the filter state provides them); then run the Kalman epoch directly (not through
    `updateBodyKinematics`): `INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, timestamp -
    lastStateTimestamp, state /* INS-propagated */, fx, fy, fz, config, state)`, `state.getGNSSEstimation(
    estimation)`, `state.getFrame(frame)` (closed loop), `lastStateTimestamp = timestamp`, firing
    `onPropagateStart`/`onPropagateEnd`.
  - [x] Task 6.4. Redefine the existing `propagate(double)` / `propagate(Time)` (signatures unchanged): it must
    not re-use measurements already consumed by an epoch. It initialises/synchronises `state` with `frame` and
    returns `false` unless there are measurements newer than `lastStateTimestamp` pending, in which case it runs
    the epoch of Task 6.3. Update Javadoc accordingly and review `reset`/setters/serialization for the new field.
  - [x] Task 6.5. Add `testEpochGateAcceptsExactInterval` (epoch 0.5 s; measurements at 0.0, 0.5, 1.0 s accepted;
    1.25 s rejected) — fails on current code.
    Progress note — failed on the unmodified class (measurements at 0.5 s rejected); passes after the fix.
  - [x] Task 6.6. Harden the existing `testUpdateMeasurements*`, `testUpdateKinematics*` and `testPropagate`
    tests as in Task 4.7; remove `PROPAGATION_ERROR = 1.0`, `POSITION_ERROR`, `VELOCITY_ERROR` in favour of
    documented tolerances.
  - [x] Task 6.7. Run `INSGNSSTightlyCoupledKalmanFilteredEstimatorTest` through `iru-gate-runner`.
    Progress note — no agent-dispatch tool was available, run with Maven directly (no clean): 26/26 pass.

### Group 4 — Full verification, changelog and documentation

Parallelizable: no — Task 7 documents and verifies the final state produced by Groups 1–3

- [x] **Task 7. Verify the whole build and document the change** — changelog and Antora pages updated; full
  suite 13943 tests, 0 failures, 0 errors; project coverage 96.8% line / 92.7% branch; no new
  Checkstyle/PMD/SpotBugs issues in Group 4 (documentation only).
  - [x] Task 7.1. Run the complete test suite and quality gates through `iru-gate-runner` (`iru-java-test` with
    no selector, `iru-java-coverage`, `iru-java-code-quality`); other tests that relied on the previous behaviour
    of the kinematics estimators or filtered estimators (e.g. `calibration/**` tests, navigator tests) must pass
    without loosening their tolerances — if one fails, fix the test's expectation only when it encoded the old
    defect, and say so in the progress note.
    Progress note — all run through `iru-gate-runner`; no Java source or test was touched by Task 7 and no test
    expectation or tolerance had to be changed. `iru-java-test` (no selector): 13943 tests, 0 failures, 0 errors,
    0 skipped (no flaky re-run needed). `iru-java-coverage` (full suite): project 96.8% line / 92.7% branch;
    `ECEFKinematicsEstimator` 99.7%/91.7%, `ECIKinematicsEstimator` 99.5%/83.3%, `NEDKinematicsEstimator`
    99.8%/91.7%, `INSLooselyCoupledKalmanFilteredEstimator` 88.9%/95.2%,
    `INSGNSSLooselyCoupledKalmanFilteredEstimator` 88.6%/91.8%, `INSGNSSTightlyCoupledKalmanFilteredEstimator`
    90.9%/95.1%. `iru-java-code-quality`: project totals Checkstyle 122,183, PMD 505, SpotBugs 2,351 — identical
    before and after Group 4. Left in place as unavoidable (introduced by Groups 1–3, already accepted): the
    plugin-default 80-column LineLength increases and the 2,000-line FileLength findings on
    `ECIKinematicsEstimator`, `INSGNSSLooselyCoupledKalmanFilteredEstimator` and
    `INSGNSSTightlyCoupledKalmanFilteredEstimator` (the Maven run does not use the repository's 120-column
    `checkstyle.xml`); no PMD or SpotBugs increase.
  - [x] Task 7.2. Add to `CHANGELOG.md` under `## [Unreleased]`: a `### Fixed` section (filtered estimators now
    integrate every IMU sample and feed the INS-propagated solution to the Kalman epoch; `ECEFKinematicsEstimator`
    eq. (5.85) term; frame-based kinematics overloads use the start-of-interval position; epoch gate accepts
    updates exactly one epoch apart), an `### Added` section (`updatePositionAndVelocity`,
    `getLastKinematicsTimestamp`) and a `### Changed` note describing the behaviour change of `update`,
    `updateBodyKinematics`, `updateMeasurements` and `propagate` (signatures unchanged).
    Progress note — `CHANGELOG.md` `## [Unreleased]` now has `### Fixed`, `### Added` and `### Changed` sections
    describing the working-tree behaviour.
  - [x] Task 7.3. Update the Antora pages with `/iru-update-docs`: `docs/modules/ROOT/pages/estimators/
    kinematics.adoc` (start-of-interval position), the page(s) describing INS/GNSS Kalman filtered estimators if
    present (usage loop: feed every IMU sample, feed fixes/measurements at their timestamps) and
    `whats-new.adoc`. Record in the progress note the items deferred to a follow-up issue: Matlab
    `Profile_*.csv` closed-loop tests, Matlab/Octave golden reference data, and PIT mutation testing.
    Progress note — updated `estimators/kinematics.adoc` (start-of-interval position, eq. 5.84/5.85, frame-based
    example), `index.adoc` (filtered-estimator bullets), `navigators/index.adoc` (link) and `whats-new.adoc`
    (`== Unreleased`); no page described the filtered estimators, so `navigators/kalman-filtered-estimators.adoc`
    was added (usage loop) and linked from `nav.adoc`. Antora build and Mermaid validation (14 diagrams) pass.
    Deferred to a follow-up issue: Matlab `Profile_*.csv` closed-loop tests, Matlab/Octave golden reference
    data, and PIT mutation testing.
