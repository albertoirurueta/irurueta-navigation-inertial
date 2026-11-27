# Implementation Plan

## Task summary

Source: GitHub issue #49
Base branch: develop

Issue #49 ("Resolve SpotBugs findings in src/main (2,351 issues)", label `enhancement`, classified as feature) asks to
resolve the `mvn spotbugs:spotbugs` findings under `src/main/java` while keeping `mvn test` green.

Baseline re-measured on `develop` @ `79c1cbc1` (2026-11-27, JDK 21) — identical to the issue: `CT_CONSTRUCTOR_THROW`
1,815, `EI_EXPOSE_REP2` 302, `EI_EXPOSE_REP` 210, `MC_OVERRIDABLE_METHOD_CALL_IN_CLONE` 23, `DM_DEFAULT_ENCODING` 1
(total 2,351).

Decisions taken (confirmed with the user):

- **`CT_CONSTRUCTOR_THROW`**: covered by a committed, documented SpotBugs exclusion filter. Validating through setters
  in constructors is a deliberate pattern in this library, many affected classes are abstract/generic bases that can't
  be `final`, and making leaf classes `final` would break external subclassing.
- **`EI_EXPOSE_REP` / `EI_EXPOSE_REP2`**: covered by the same filter. The numerical `Matrix`/array/collection APIs
  share mutable state by design to avoid allocations; defensive copies would change API semantics and cost.
- **`MC_OVERRIDABLE_METHOD_CALL_IN_CLONE`**: fixed in code (not filtered) without changing the public API — `clone()`
  stops calling the public, overridable `copyTo(...)` and uses a `private` helper instead. Classes and `copyTo`
  are **not** made `final` (that would break subclasses such as `StandardDeviationFrameBodyKinematics`).
- **`DM_DEFAULT_ENCODING`**: fixed in code — `WMMLoader` reads with `StandardCharsets.UTF_8` (a superset of the
  ASCII WMM coefficient files, so parsing is unchanged).
- The filter excludes **only** the three policy bug patterns; any other SpotBugs pattern keeps being reported.

Open clarifications: none

Final quality gate (iru-code Step 7): Checkstyle 4370 (unchanged from baseline), PMD 0, SpotBugs 0 (from 2351). The user chose to file the remaining pre-existing Checkstyle findings as a follow-up: GitHub issue #53.

## Acceptance criteria coverage

| # | Criterion | Covered by tasks |
|---|---|---|
| 1 | "`WMMLoader` uses an explicit charset, with a test." | Task 2 |
| 2 | "No `MC_OVERRIDABLE_METHOD_CALL_IN_CLONE` finding remains." | Task 3, Task 4 |
| 3 | "`CT_CONSTRUCTOR_THROW` and `EI_EXPOSE_REP*` are either fixed or covered by a committed, documented SpotBugs exclusion filter." | Task 1, Task 4 |
| 4 | "The full `mvn test` suite stays green." | Task 4 |

## Current code state

- **Build**: single-module Maven library, Java 21, no wrapper. Maven must run with
  `JAVA_HOME=$(/usr/libexec/java_home -v 21)` (the default JDK fails with "invalid target release: 21").
- **SpotBugs wiring** in `pom.xml`: `com.github.spotbugs:spotbugs-maven-plugin` 4.9.6.0 is declared twice with no
  configuration — under `<build><plugins>` (~line 225, used by CLI `mvn spotbugs:spotbugs`) and under
  `<reporting><plugins>` (~line 381, used by `mvn site`). No filter file exists. Precedent: #47 wired
  `checkstyle.xml` and #48 wired `pmd-ruleset.xml` the same way (root-level config file referenced from both blocks,
  with an explanatory XML comment).
- **`DM_DEFAULT_ENCODING`**: `src/main/java/com/irurueta/navigation/inertial/wmm/WMMLoader.java:129` —
  `load(InputStream)` does `new InputStreamReader(stream)` (platform charset) and feeds a `StreamTokenizer`.
  `load(File)` / `loadFromResource(...)`-style entry points delegate to it. Test: `src/test/java/.../wmm/WMMLoaderTest.java`.
- **`MC_OVERRIDABLE_METHOD_CALL_IN_CLONE`** (23 classes). Every one has the same shape:
  ```java
  @Override
  protected Object clone() throws CloneNotSupportedException {
      final var result = (X) super.clone();
      copyTo(result);          // public, overridable -> flagged
      return result;
  }
  ```
  `copyTo(X output)` is public and typically delegates to `output.copyFrom(this)`.
  - Package `com.irurueta.navigation.inertial`: `BodyKinematics`, `BodyKinematicsAndMagneticFluxDensity`,
    `BodyMagneticFluxDensity`, `INSLooselyCoupledKalmanConfig`, `INSLooselyCoupledKalmanInitializerConfig`,
    `INSLooselyCoupledKalmanState`, `INSTightlyCoupledKalmanConfig`, `INSTightlyCoupledKalmanInitializerConfig`,
    `INSTightlyCoupledKalmanState`, `NEDGravity`, `RadiiOfCurvature`.
  - Package `com.irurueta.navigation.inertial.calibration`: `BodyKinematicsSequence`, `FrameBodyKinematics`,
    `FrameBodyMagneticFluxDensity`, `IMUErrors`, `StandardDeviationBodyKinematics`,
    `StandardDeviationBodyMagneticFluxDensity`, `StandardDeviationFrameBodyKinematics` (extends
    `FrameBodyKinematics`), `StandardDeviationFrameBodyMagneticFluxDensity` (extends `FrameBodyMagneticFluxDensity`),
    `StandardDeviationTimedBodyKinematics` (extends `TimedBodyKinematics`), `TimedBodyKinematics`,
    `TimedBodyKinematicsAndMagneticFluxDensity` (extends `inertial.BodyKinematicsAndMagneticFluxDensity`).
  - Package `com.irurueta.navigation.inertial.wmm`: `NEDMagneticFluxDensity`.
  - Subclass `copyTo` overloads call `super.copyTo(output)` and then copy their own fields
    (e.g. `StandardDeviationFrameBodyKinematics.copyTo`, line ~1389).
  - **Latent aliasing**: after `super.clone()` (a shallow copy), mutable fields of `result` are the *same* objects as
    in `this`; `copyFrom` then copies a `Matrix`/triad/frame "into itself" when the target is non-null (e.g.
    `INSLooselyCoupledKalmanState.copyFrom`, line ~1871: `bodyToEcefCoordinateTransformationMatrix.copyFrom(...)`).
    So clones of classes with mutable reference fields can currently share those objects with the original.
    The fix below must give clones their own copies.
  - Existing `testClone()` tests exist per class (e.g. `INSLooselyCoupledKalmanStateTest.testClone`, line ~2088).
- **`CT_CONSTRUCTOR_THROW`**: constructors of calibrators/estimators call validating setters (accelerometer 628,
  magnetometer 496, gyroscope 448, calibration 97, inertial 93, bias 51, wmm 2).
- **`EI_EXPOSE_REP*`**: by package — magnetometer 175, gyroscope 153, accelerometer 112, calibration 42,
  inertial 25, noise 3, bias 2.
- **Docs**: Antora at `docs/` — `docs/modules/ROOT/pages/reference.adoc:26` lists SpotBugs among the site reports.
  `CHANGELOG.md` at the root (Keep a Changelog).

## Implementation steps

### Group 1 — Parallelizable: yes

The three tasks touch disjoint files (`pom.xml` + new filter; `WMMLoader` + its test; the 23 clone classes + their
tests).

- [x] Task 1. Add and wire a documented SpotBugs exclusion filter _(java)_ — `spotbugs-exclude.xml` added and wired in both `pom.xml` SpotBugs declarations; build config only (no tests). SpotBugs 2351 → 0 after the group (CT_CONSTRUCTOR_THROW/EI_EXPOSE_REP/EI_EXPOSE_REP2 filtered, subpackages included); no new Checkstyle/PMD issues; full suite green (14035 tests).
  - [x] Task 1.1. Create `spotbugs-exclude.xml` at the repository root (license header style as `pmd-ruleset.xml`),
    with a top-level comment stating the policy and one `<Match>` per pattern, each with its own rationale comment,
    scoped to the library's packages:
    ```xml
    <FindBugsFilter xmlns="https://github.com/spotbugs/filter/3.0.0" ...>
      <!-- Constructors validate arguments through setters by design (see issue #49). -->
      <Match>
        <Package name="~com\.irurueta\.navigation\.inertial(\..*)?"/>
        <Bug pattern="CT_CONSTRUCTOR_THROW"/>
      </Match>
      <!-- Numerical APIs share Matrix/array/collection instances on purpose to avoid copies. -->
      <Match>
        <Package name="~com\.irurueta\.navigation\.inertial(\..*)?"/>
        <Bug pattern="EI_EXPOSE_REP,EI_EXPOSE_REP2"/>
      </Match>
    </FindBugsFilter>
    ```
    Do **not** exclude `MC_OVERRIDABLE_METHOD_CALL_IN_CLONE` or `DM_DEFAULT_ENCODING`.
  - [x] Task 1.2. In `pom.xml`, add `<configuration><excludeFilterFile>spotbugs-exclude.xml</excludeFilterFile></configuration>`
    to **both** `spotbugs-maven-plugin` declarations (`<build>` and `<reporting>`), with an XML comment mirroring the
    Checkstyle one (e.g. "makes CLI runs of spotbugs:spotbugs and the site report use spotbugs-exclude.xml").
  - [x] Task 1.3. Verify: `mvn -q clean compile spotbugs:spotbugs` and parse `target/spotbugsXml.xml` — no
    `CT_CONSTRUCTOR_THROW`/`EI_EXPOSE_REP`/`EI_EXPOSE_REP2` remain, while the `MC_*`/`DM_*` findings are still
    reported until Tasks 2–3 land.

- [x] Task 2. Make `WMMLoader` read with an explicit charset _(java)_ — `WMMLoader.java` (UTF-8 reader), test `load_whenStreamOfModelFileBytes_returnsModelEqualToFileModel` in `WMMLoaderTest` (100% line / 91.7% branch coverage); DM_DEFAULT_ENCODING fixed, no new Checkstyle/PMD/SpotBugs issues.
  - [x] Task 2.1. In `src/main/java/com/irurueta/navigation/inertial/wmm/WMMLoader.java`, change
    `new InputStreamReader(stream)` to `new InputStreamReader(stream, StandardCharsets.UTF_8)` and import
    `java.nio.charset.StandardCharsets`. Mention the charset in the `load(InputStream)` Javadoc
    ("The stream is decoded as UTF-8 regardless of the platform default charset.").
  - [x] Task 2.2. In `src/test/java/com/irurueta/navigation/inertial/wmm/WMMLoaderTest.java`, add a test that loads
    the bundled WMM model while the decoding cannot depend on the platform default — e.g. load the same model
    through `load(InputStream)` from an in-memory `ByteArrayInputStream` of the model file bytes and assert the
    resulting `WorldMagneticModel` (epoch and a few coefficients) equals the one from `load(File)`. Keep it to this
    deterministic equality check; don't try to change the JVM default charset in the test.
  - [x] Task 2.3. Run `WMMLoaderTest` via the `iru-gate-runner` agent
    (`Skill({skill: "iru-java-test", args: "WMMLoaderTest"})`).

- [x] Task 3. Stop `clone()` from calling the overridable `copyTo(...)` in the 23 flagged classes _(java)_ — 23 classes in inertial, inertial.calibration, inertial.wmm (private `copyFieldsTo` in the 10 classes with mutable reference fields; plain `super.clone()` in the 13 primitive-only classes incl. the 4 subclasses); `testClone()` extended in all 23 `*Test` classes (578 targeted tests pass; 94.0–99.6% line coverage per class); MC_OVERRIDABLE_METHOD_CALL_IN_CLONE 23 → 0, no new Checkstyle/PMD/SpotBugs issues.
  - Pattern for a **base class** `X` (no API change; public `copyTo`/`copyFrom` keep their behaviour):
    ```java
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final var result = (X) super.clone();
        copyFieldsTo(result);   // private, so not overridable
        return result;
    }

    // Copies X's own state into output, allocating new instances for every
    // mutable reference field so the clone never shares them with this instance.
    private void copyFieldsTo(final X output) { ... }
    ```
    - In `clone()`, before copying, null-out/re-allocate the mutable reference fields of `result` (which still alias
      `this` after the shallow `super.clone()`), so the copy produces independent `Matrix`/triad/frame/list objects.
      Reuse existing static helpers / copy constructors where available (e.g. `new Matrix(m)`, `triad.clone()`-style
      or `new X(other)` constructors).
    - Leave `copyTo`/`copyFrom` as they are unless the private helper can be shared without changing their
      behaviour.
  - Pattern for a **subclass** `S extends X`: `super.clone()` already deep-copies `X`'s fields via `X`'s private
    helper, so `S.clone()` only copies `S`'s own fields through its own private helper (e.g.
    `StandardDeviationFrameBodyKinematics` copies `specificForceStandardDeviation` and
    `angularRateStandardDeviation`) — never calling `copyTo`.
  - [x] Task 3.1. Apply the pattern to the `com.irurueta.navigation.inertial` classes: `BodyKinematics`,
    `BodyKinematicsAndMagneticFluxDensity`, `BodyMagneticFluxDensity`, `NEDGravity`, `RadiiOfCurvature`.
  - [x] Task 3.2. Apply it to the Kalman classes: `INSLooselyCoupledKalmanConfig`,
    `INSLooselyCoupledKalmanInitializerConfig`, `INSLooselyCoupledKalmanState`, `INSTightlyCoupledKalmanConfig`,
    `INSTightlyCoupledKalmanInitializerConfig`, `INSTightlyCoupledKalmanState` (state classes hold `Matrix` fields —
    ensure the clone gets new `Matrix` instances).
  - [x] Task 3.3. Apply it to the `calibration` base classes: `FrameBodyKinematics`, `FrameBodyMagneticFluxDensity`,
    `TimedBodyKinematics`, `StandardDeviationBodyKinematics`, `StandardDeviationBodyMagneticFluxDensity`,
    `IMUErrors`, `BodyKinematicsSequence` (generic list of `T extends TimedBodyKinematics` — the clone gets a new
    list whose items are cloned/copied, matching what `copyTo` produces today).
  - [x] Task 3.4. Apply the subclass pattern to `StandardDeviationFrameBodyKinematics`,
    `StandardDeviationFrameBodyMagneticFluxDensity`, `StandardDeviationTimedBodyKinematics`,
    `TimedBodyKinematicsAndMagneticFluxDensity` (after 3.1/3.3 within this task's own sequence, since they rely on
    their base classes' new `clone()`).
  - [x] Task 3.5. Apply it to `wmm.NEDMagneticFluxDensity`.
  - [x] Task 3.6. Extend each class's existing `testClone()` test (same test files, e.g.
    `INSLooselyCoupledKalmanStateTest`, `StandardDeviationFrameBodyKinematicsTest`) to assert, besides equality,
    that the clone is **independent**: for every mutable reference field, `assertNotSame(original.getX(),
    clone.getX())`, and mutating the clone's field (e.g. `clone.getCovariance().setElementAt(0, 0, 42.0)`) leaves
    the original unchanged. For subclasses, also assert the clone's runtime class is the subclass and the
    subclass-specific fields are copied.
  - [x] Task 3.7. Run the touched test classes via the `iru-gate-runner` agent
    (`Skill({skill: "iru-java-test", args: "<comma-separated touched test classes>"})`).

### Group 2 — Parallelizable: yes

Depends on Group 1 having landed (verifies the combined result).

- [x] Task 4. Verify the SpotBugs result, the full suite and update the docs _(java)_ — SpotBugs 0 BugInstance, docs and changelog updated (`reference.adoc`, `CHANGELOG.md`); full suite 14035 tests green; touched classes 94.0–100% line coverage; SpotBugs 0, PMD 0, Checkstyle 4370 (unchanged baseline) — no new issues.
  - [x] Task 4.1. Run `mvn -q clean compile spotbugs:spotbugs` (JDK 21) and parse `target/spotbugsXml.xml`:
    **zero** `BugInstance` entries expected. If any other pattern appears, fix it in code rather than widening the
    filter (or stop and report if it isn't trivially fixable).
  - [x] Task 4.2. Run the full `mvn test` suite via the `iru-gate-runner` agent
    (`Skill({skill: "iru-java-test"})`) — must be green; check coverage of the touched classes stays ≥ 80%
    (`iru-java-coverage`) and Checkstyle/PMD report no new findings (`iru-java-code-quality`) versus the baseline. — 14035 tests, 0 failures; all 24 touched classes ≥ 94% line coverage; Checkstyle/PMD/SpotBugs counts match baseline.
  - [x] Task 4.3. Docs: in `docs/modules/ROOT/pages/reference.adoc`, note next to the SpotBugs report that
    `spotbugs-exclude.xml` documents the excluded patterns (`CT_CONSTRUCTOR_THROW`, `EI_EXPOSE_REP`,
    `EI_EXPOSE_REP2`) and why; add an `Unreleased` entry in `CHANGELOG.md` (Fixed: `WMMLoader` platform-charset
    dependency; clones of state/config classes no longer share mutable objects with the original. Changed: SpotBugs
    exclusion filter added). Run `/iru-update-docs` if the pipeline invokes it.
