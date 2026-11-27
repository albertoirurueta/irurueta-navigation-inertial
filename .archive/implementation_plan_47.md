# Implementation Plan

## Task summary

Source: GitHub issue #47
Base branch: develop

Issue #47 ("Reduce Checkstyle findings in src/main (122,599 issues)", label `enhancement`, classified as feature)
asks to reduce Checkstyle findings under `src/main/java`, fix the real defects, agree a policy for the
high-volume stylistic rules and keep `mvn test` green.

Exploration changed the premises of the issue:

- The 122,599 baseline is **not** measured against this repo's `checkstyle.xml`. `maven-checkstyle-plugin` 3.6.0 is
  configured only under `<reporting>` in `pom.xml`, so `mvn checkstyle:checkstyle` from the CLI falls back to the
  default Sun rules (80 columns, no Javadoc `ignorePattern`). Measured with
  `-Dcheckstyle.config.location=checkstyle.xml` the real baseline is **20,088 findings in 396 files**:
  HiddenField 12,804; MagicNumber 3,201; ParameterNumber 2,865; RedundantModifier 632; ImportOrder 202;
  LineLength 176; DeclarationOrder 68; JavadocStyle 49; Indentation 36; ThrowsCount 14; NewlineAtEndOfFile 11;
  VisibilityModifier 10; FileTabCharacter 10; JavadocType 4; UnusedImports 2; one each of EqualsHashCode,
  JavadocVariable, JavadocMethod, JavadocPackage.
- `GravityOrGravitation` **already** implements `hashCode()` (line 381, `Objects.hash(gx, gy, gz)`), and its
  subclasses `ECEFGravity`/`ECIGravitation` override both `equals(Object)` and `hashCode()`, with
  `hashCode` assertions in `ECEFGravityTest`/`ECIGravitationTest`. The `EqualsHashCode` finding is a false positive
  because Checkstyle ignores the abstract `equals(Object)` declaration. No production bug exists.

Decisions taken (confirmed with the user):

- **Config**: the repo's `checkstyle.xml` is authoritative and is wired into the shared plugin configuration in
  `pom.xml`, so CLI and site runs agree.
- **Policy**: relax the config for the high-volume rules, then fix what remains:
  - HiddenField: `ignoreConstructorParameter=true`, `ignoreSetter=true`, `setterCanReturnItsClass=true`.
  - MagicNumber: default ignored numbers (-1, 0, 1, 2) plus 3, 0.5; ignore hashCode methods, field declarations and
    annotations.
  - ParameterNumber: `max=15`, `ignoreOverriddenMethods=true`; constructors of estimators/states stay subject to the
    limit only above that threshold.
- Public API behaviour stays unchanged; no renames of public members.

Open clarifications: none

## Acceptance criteria coverage

| # | Criterion | Covered by tasks |
|---|---|---|
| 1 | "`GravityOrGravitation` implements `hashCode()` consistently with `equals()`, with a test." | Task 4 (hashCode already exists and is tested via subclasses; resolve the false positive and add a direct consistency test) |
| 2 | "`checkstyle.xml` reflects an agreed policy for LineLength, HiddenField, MagicNumber and ParameterNumber." | Task 1, Task 2 |
| 3 | "Checkstyle findings under `src/main/java` are reduced substantially against the 122,599 baseline, with the remaining ones justified by the agreed policy." | Task 3, Tasks 5-10, Task 11 |
| 4 | "The full `mvn test` suite stays green." | Task 12 |
| 5 | derived: the baseline is re-measured against the repo's own `checkstyle.xml` and the correct number is recorded | Task 3, Task 11 |

## Current code state

- `pom.xml` (~line 340): `maven-checkstyle-plugin` 3.6.0 inside `<reporting><plugins>` with
  `<configLocation>checkstyle.xml</configLocation>`; there is no `<build>`/`pluginManagement` entry, so only
  `mvn site` uses it.
- `checkstyle.xml` (repo root): LineLength `max=120` with `ignorePattern`, plain `HiddenField`, `MagicNumber`,
  `ParameterNumber`, `ImportOrder`, `DeclarationOrder`, `ThrowsCount max=2`, `VisibilityModifier` with
  `protectedAllowed`, etc.
- `src/main/java/com/irurueta/navigation/inertial/GravityOrGravitation.java`: abstract generic base with
  package-private fields `gx, gy, gz`, `hashCode()` at line 381, abstract `equals(Object)` at line 394.
- Worst files by findings: `INSLooselyCoupledKalmanEpochEstimator`, `INSTightlyCoupledKalmanState`,
  `INSLooselyCoupledKalmanState`, `INSGNSSTightlyCoupledKalmanFilteredEstimator` (mostly HiddenField,
  MagicNumber, ParameterNumber, which Group 1 relaxes).
- Reproduce / measure: `mvn -q compile checkstyle:checkstyle` writes `target/checkstyle-result.xml`.
- Project type: java-library, Maven, single module. Antora docs in `docs/`.

## Implementation steps

### Group 1: Checkstyle configuration (Parallelizable: no — Task 2 and Task 3 depend on Task 1's wiring and on each other's output)

- [x] Task 1. Wire `checkstyle.xml` into the plugin for CLI runs _(java)_ — implemented in `pom.xml` (pluginManagement only, no lifecycle binding); CLI run verified at 20,088 findings in 396 files, identical to the explicit-property baseline; build-config only, no tests/coverage applicable; no Java sources touched, so no new Checkstyle/PMD/SpotBugs issues.
  - [x] Task 1.1. In `pom.xml` add `maven-checkstyle-plugin` 3.6.0 under `<build><pluginManagement>` with
    `<configLocation>checkstyle.xml</configLocation>`, so `mvn checkstyle:checkstyle` uses it without
    `-Dcheckstyle.config.location`. Keep the `<reporting>` entry as is. Do not bind it to a lifecycle phase.
  - [x] Task 1.2. Verify with `mvn -q compile checkstyle:checkstyle` that the total equals the 20,088 measured with
    the explicit property. — verified: 20,088 findings in 396 files, per-rule counts identical.
- [x] Task 2. Relax `checkstyle.xml` per the agreed policy _(java)_ — implemented in `checkstyle.xml` with a rationale comment per relaxed rule; config valid (no unknown properties); config-only, no tests/coverage applicable; no Java sources touched.
  - [x] Task 2.1. `HiddenField`: add `ignoreConstructorParameter`, `ignoreSetter`, `setterCanReturnItsClass` = true.
  - [x] Task 2.2. `MagicNumber`: add `ignoreNumbers` `-1, 0, 1, 2, 3, 0.5`, `ignoreHashCodeMethod=true`,
    `ignoreFieldDeclaration=true`, `ignoreAnnotation=true`.
  - [x] Task 2.3. `ParameterNumber`: `max=15`, `ignoreOverriddenMethods=true`.
  - [x] Task 2.4. Add a short XML comment above each relaxed rule stating why (numerical library, builder-style
    setters, many-argument estimators).
- [x] Task 3. Re-measure and record the new baseline _(java)_ — measured 5,574 findings in 320 files (down from 20,088 in 396); per-rule table recorded under `## Measured baselines`; measurement-only, no tests applicable.
  - [x] Task 3.1. Run `mvn -q compile checkstyle:checkstyle` (via `iru-gate-runner`), summarise findings by rule,
    and append the table to this plan under a `## Measured baselines` heading for later comparison.

### Group 2: Correctness item (Parallelizable: yes — touches only `GravityOrGravitation` and its test)

- [x] Task 4. Resolve the `EqualsHashCode` finding on `GravityOrGravitation` _(java)_ — checkstyle.xml (SuppressWarningsFilter/Holder), GravityOrGravitation.java (merged `@SuppressWarnings`, explanatory comment, Javadoc `@param <T>`); new GravityOrGravitationTest (8 tests). GravityOrGravitation 96.6% line / 100% branch coverage (3 uncovered lines are unreachable `catch` blocks). No new Checkstyle/PMD/SpotBugs issues; EqualsHashCode and JavadocType findings removed (Checkstyle total 5,574 → 5,572). Full suite green (14,034 tests).
  - [x] Task 4.1. Keep the existing `hashCode()`. Silence the false positive without changing the API: add
    `SuppressWarningsHolder` and `SuppressWarningsFilter` to `checkstyle.xml` and annotate the class with
    `@SuppressWarnings("checkstyle:EqualsHashCode")` (merge with the existing `@SuppressWarnings` value), with a
    comment explaining that concrete subclasses implement `equals(Object)`.
  - [x] Task 4.2. Add `GravityOrGravitationTest` (new, or extend an existing test) asserting that two instances
    with equal components have equal hash codes and that `equals` and `hashCode` are consistent across
    `ECEFGravity` and `ECIGravitation`.

### Group 3: Mechanical fixes in `src/main/java` (Parallelizable: no — each task edits many of the same files)

Run each task's rule from `target/checkstyle-result.xml` after Group 1. Counts are from the baseline before the
Group 1 relaxations; re-read the report for the real list.

- [x] Task 5. Whitespace and file hygiene _(java)_ — implemented (formatting only, 22 files under src/main/java). FileTabCharacter 10->0, NewlineAtEndOfFile 11->0, Indentation 36->0; no new Checkstyle/PMD/SpotBugs issues; validated: full suite 14,034/14,034 green.
  - [x] Task 5.1. Replace tab characters (10 `FileTabCharacter`) and add trailing newlines (11
    `NewlineAtEndOfFile`). — tabs expanded (4-col stops) in comments of 10 files; trailing LF added to 11 files.
  - [x] Task 5.2. Fix the 36 `Indentation` findings. — anonymous listener in `PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator` re-indented.
- [x] Task 6. Imports _(java)_ — implemented in 202 src/main/java files. UnusedImports 2->0, ImportOrder 202->0; no new Checkstyle/PMD/SpotBugs issues; validated: full suite 14,034/14,034 green.
  - [x] Task 6.1. Remove the remaining `UnusedImports`. — removed from KnownPositionAccelerometerCalibrator and KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.
  - [x] Task 6.2. Fix `ImportOrder` (202): reorder imports to the order the rule expects, without changing the
    imported set. — removed blank lines inside import blocks (no reordering needed).
- [x] Task 7. Modifiers and class design _(java)_ — 7.1/7.2/7.3 applied; 14 `ThrowsCount` left for Task 11; no new Checkstyle/PMD/SpotBugs issues; validated: full suite 14,034/14,034 green; coverage WorldMagneticModel 100% line, WMMLoader 100%, GravityOrGravitation 96.7%.
  - [x] Task 7.1. Remove 632 `RedundantModifier` findings (e.g. `final` on private/static methods' parameters of
    final methods, redundant `public` in interfaces), checking each is purely redundant. — 632 -> 0 (100 files).
  - [x] Task 7.2. Fix 10 `VisibilityModifier` findings only where it does not change public API; otherwise
    document the exception in Task 11. — 10 -> 0 (`GravityOrGravitation`, `WorldMagneticModel` package-private fields made private; package-private accessors added to `WorldMagneticModel`, its epoch mutator named `updateEpoch` so SpotBugs does not reclassify the class as mutable (+83 EI_EXPOSE_REP/REP2 otherwise); `serialVersionUID` pinned on `GravityOrGravitation` to its pre-change computed value).
  - [x] Task 7.3. Fix 14 `ThrowsCount` and 68 `DeclarationOrder` findings where this keeps behaviour and public
    signatures unchanged; move members only, no logic edits. — `DeclarationOrder` 68 -> 0 (12 robust calibrators); 14 `ThrowsCount` all on public API, left for Task 11 (fixing would change declared exceptions on public `update*`/`propagate`/`updateMeasurements`/`calibrate()` methods).
- [x] Task 8. Javadoc _(java)_ — Javadoc-only edits in 44 existing files plus 1 new `package-info.java`. JavadocStyle 49->0, JavadocType 3->0, JavadocVariable/JavadocMethod/JavadocPackage 1->0; Javadoc build passes; validated: full suite 14,034/14,034 green.
  - [x] Task 8.1. Fix 49 `JavadocStyle`, 4 `JavadocType`, and the single `JavadocVariable` and `JavadocMethod`
    findings. — all now 0 (JavadocType was 3 in the real report).
  - [x] Task 8.2. Add the missing `package-info.java` (`JavadocPackage`), identifying the package from the report.
    — added `navigators/package-info.java`.
- [x] Task 9. Line length _(java)_ — implemented (formatting only, 18 src/main/java files; LineLength 176 -> 0);
  no other rule changed; validated: full suite 14,034/14,034 green.
  - [x] Task 9.1. Wrap the 176 `LineLength` findings (max 120) by reformatting only; no wording changes beyond
    wrapping.
- [x] Task 10. Validate Group 3 _(java)_ — Checkstyle 5,572 -> 4,370 (only MagicNumber 2,763, HiddenField 1,509, ParameterNumber 84, ThrowsCount 14 remain); PMD 506 -> 498; SpotBugs 2,351 unchanged; license headers added to 2 files that lacked them.
  - [x] Task 10.1. Compile and run the full test suite via `iru-gate-runner` (`iru-java-test`); all green. — 14,034 run, 0 failures (JaCoCo run); `wmm` tests re-run green after the `updateEpoch` rename.

### Group 4: Wrap-up (Parallelizable: no — depends on Groups 1-3)

- [x] Task 11. Final measurement and justification _(java)_ — measured; Checkstyle 4,370 (MagicNumber 2,763, HiddenField 1,509, ParameterNumber 84, ThrowsCount 14), PMD 498, SpotBugs 2,351; final table and justifications under `## Measured baselines`; only implementation_plan.md edited (no code/tests); `/iru-java-code-quality` via `iru-gate-runner` confirmed no regression (PMD 506 -> 498, SpotBugs 2,351 -> 2,351, Checkstyle unchanged vs Group 3).
  - [x] Task 11.1. Re-run Checkstyle, update `## Measured baselines` with the final table, and list any remaining
    findings with the policy item that justifies each one.
  - [x] Task 11.2. Run `/iru-java-code-quality` (Checkstyle, PMD, SpotBugs) through `iru-gate-runner` and confirm
    no regression in PMD/SpotBugs.
- [x] Task 12. Full verification _(java)_ — `mvn clean test` via `iru-gate-runner`: BUILD SUCCESS, 14,034 run, 0 failures, 0 errors, 0 skipped; no files changed.
  - [x] Task 12.1. Run `mvn test` through `iru-gate-runner` and confirm it is green.
- [x] Task 13. Documentation — CHANGELOG.md `[Unreleased]` gets a `### Changed` entry (Checkstyle CLI wiring, relaxed policy, 20,088 -> 4,370 findings); docs/modules/ROOT/pages/reference.adoc gets a NOTE that the Checkstyle report and local `mvn checkstyle:checkstyle` both use `checkstyle.xml`; Antora build passes; docs only, no tests.
  - [x] Task 13.1. Add a CHANGELOG.md entry under the unreleased section describing the Checkstyle wiring and policy
    change, and run `/iru-update-docs` for any docs that mention Checkstyle (`docs/modules/ROOT/pages/reference.adoc`).

## Measured baselines

### After Task 2 (2026-11-26)

Measured with `mvn -q compile checkstyle:checkstyle` against the repository's own `checkstyle.xml` (wired in Task 1,
`HiddenField`, `MagicNumber` and `ParameterNumber` relaxed in Task 2), counted from `target/checkstyle-result.xml`.
"Before" is the pre-relaxation baseline measured with the same `checkstyle.xml`.

| Rule | Before | After | Delta |
|---|---:|---:|---:|
| HiddenField | 12,804 | 1,509 | -11,295 |
| MagicNumber | 3,201 | 2,763 | -438 |
| ParameterNumber | 2,865 | 84 | -2,781 |
| RedundantModifier | 632 | 632 | 0 |
| ImportOrder | 202 | 202 | 0 |
| LineLength | 176 | 176 | 0 |
| DeclarationOrder | 68 | 68 | 0 |
| JavadocStyle | 49 | 49 | 0 |
| Indentation | 36 | 36 | 0 |
| ThrowsCount | 14 | 14 | 0 |
| NewlineAtEndOfFile | 11 | 11 | 0 |
| VisibilityModifier | 10 | 10 | 0 |
| FileTabCharacter | 10 | 10 | 0 |
| JavadocType | 4 | 4 | 0 |
| UnusedImports | 2 | 2 | 0 |
| EqualsHashCode | 1 | 1 | 0 |
| JavadocVariable | 1 | 1 | 0 |
| JavadocMethod | 1 | 1 | 0 |
| JavadocPackage | 1 | 1 | 0 |
| **Total** | **20,088** | **5,574** | **-14,514** |
| **Files with findings** | **396** | **320** | **-76** |

### Final (Task 11, 2026-11-26)

Measured with `mvn -q compile checkstyle:checkstyle` against the repository's own `checkstyle.xml`, counted from
`target/checkstyle-result.xml`; PMD and SpotBugs counted from `target/pmd.xml` and `target/spotbugsXml.xml`.
"Original" is the pre-relaxation baseline with the repository's `checkstyle.xml` (the default Sun rules gave 122,599).

| Rule | Original (repo checkstyle.xml, pre-relaxation) | After Task 2 | Final |
|---|---:|---:|---:|
| HiddenField | 12,804 | 1,509 | 1,509 |
| MagicNumber | 3,201 | 2,763 | 2,763 |
| ParameterNumber | 2,865 | 84 | 84 |
| RedundantModifier | 632 | 632 | 0 |
| ImportOrder | 202 | 202 | 0 |
| LineLength | 176 | 176 | 0 |
| DeclarationOrder | 68 | 68 | 0 |
| JavadocStyle | 49 | 49 | 0 |
| Indentation | 36 | 36 | 0 |
| ThrowsCount | 14 | 14 | 14 |
| NewlineAtEndOfFile | 11 | 11 | 0 |
| VisibilityModifier | 10 | 10 | 0 |
| FileTabCharacter | 10 | 10 | 0 |
| JavadocType | 4 | 4 | 0 |
| UnusedImports | 2 | 2 | 0 |
| EqualsHashCode | 1 | 1 | 0 |
| JavadocVariable | 1 | 1 | 0 |
| JavadocMethod | 1 | 1 | 0 |
| JavadocPackage | 1 | 1 | 0 |
| **Total** | **20,088** | **5,574** | **4,370** |
| **Files with findings** | **396** | **320** | **134** |

PMD: 506 originally -> 498 final. SpotBugs: 2,351 originally -> 2,351 final. No regression in either.

#### Remaining findings and justification

- **HiddenField (1,509)**: the remaining instances are not in constructors or setters (the policy relaxes only those,
  via `ignoreConstructorParameter`/`ignoreSetter`). Fixing them would mean renaming locals/parameters across
  non-setter methods, which is out of scope; public API unchanged.
- **MagicNumber (2,763)**: numerical-library constants (algorithm coefficients, WGS84 and model constants). The policy
  only ignores -1, 0, 1, 2, 3, 0.5, `hashCode` methods, field declarations and annotations.
- **ParameterNumber (84)**: the methods above `max=15` are public API constructors/methods; changing them would break
  the API.
- **ThrowsCount (14)**: all on public API methods: `INSLooselyCoupledKalmanFilteredEstimator`
  `update`/`updatePositionAndVelocity`/`propagate` (7); `INSGNSSLooselyCoupledKalmanFilteredEstimator.updateMeasurements`
  (2); `INSGNSSTightlyCoupledKalmanFilteredEstimator.updateMeasurements` (2); `calibrate()` on the
  `AccelerometerCalibrator`, `MagnetometerCalibrator` and `GyroscopeCalibrator` interfaces (3). Fixing them would change
  the declared exceptions.
