# Implementation Plan

## Task summary

Source: GitHub issue #48
Base branch: develop

Issue #48 ("Resolve PMD findings in src/main (483 empty catch blocks, 506 total)", label `enhancement`, classified as
feature) asks to resolve the `mvn pmd:pmd` findings under `src/main/java`: review every empty `catch`, fix the small
rules, and keep `mvn test` green.

Exploration changed the premises of the issue (measured 2026-11-26 on `develop` with JDK 21):

- The current baseline is **498** findings, not 506: `UnnecessaryModifier` (6) and `UnnecessaryImport` (2) no longer
  occur (fixed by #47). Remaining: `EmptyCatchBlock` 483, `UnnecessaryFullyQualifiedName` 7, `UselessParentheses` 6,
  `UselessOverridingMethod` 2.
- All 483 empty catches are **already** named `ignore` and already carry a `// never happens` comment. PMD still flags
  them because the default `EmptyCatchBlock` has `allowCommentedBlocks=false` and
  `allowExceptionNameRegex=^(ignored|expected)$`. By type: `LockedException` 412, `WrongSizeException` 41,
  `AlgebraException` 15, `InvalidSourceAndDestinationFrameTypeException` 15.
- `maven-pmd-plugin` 3.27.0 is configured only under `<reporting>` in `pom.xml` with no ruleset, so CLI runs use PMD's
  default rules.
- Building requires JDK 21: run Maven with `JAVA_HOME=$(/usr/libexec/java_home -v 21)` (the default JDK 26 fails with
  "invalid target release: 21").

Decisions taken (confirmed with the user):

- **Empty catches**: configure a PMD ruleset wired into `pom.xml` (as #47 did for Checkstyle) with
  `allowCommentedBlocks=true` and an exception-name regex that also accepts `ignore`, instead of renaming 483 variables.
- Each catch is still **reviewed**: any one that can hide a real failure (especially in the Kalman filtered estimators)
  is changed to handle or propagate the exception, with a test.
- Public API behaviour stays unchanged unless a reviewed catch proves to hide a real failure.

Open clarifications: none

## Acceptance criteria coverage

| # | Criterion | Covered by tasks |
|---|---|---|
| 1 | "No `EmptyCatchBlock` finding remains without a justification comment or an exception name PMD accepts." | Task 1, Task 3 |
| 2 | "Any catch block that could hide a real failure (especially in the Kalman filtered estimators) handles or propagates the exception, with a test." | Task 3 |
| 3 | "The other 23 PMD findings are fixed." (now 15 remaining: 7 + 6 + 2) | Task 2 |
| 4 | "The full `mvn test` suite stays green." | Task 4 |
| 5 | derived: `mvn compile pmd:pmd` reports 0 violations | Task 1, Task 2, Task 3, Task 4 |

## Current code state

- Build: Maven single module (no wrapper), Java 21, `maven-pmd-plugin` 3.27.0 in `<reporting>` of `pom.xml:~376`, no
  ruleset file. `checkstyle.xml` at the repo root is the precedent for a wired config.
- Empty catches (483) are spread over `calibration/**` (~348), `INSGNSSLooselyCoupledKalmanFilteredEstimator` (37),
  `INSGNSSTightlyCoupledKalmanFilteredEstimator` (34), `INSLooselyCoupledKalmanFilteredEstimator` (34),
  `navigators/**` (15), `estimators/**` (5) and others. Typical shape:
  ```java
  } catch (final LockedException ignore) {
      // never happens
  }
  ```
  e.g. `INSGNSSLooselyCoupledKalmanFilteredEstimator(INSLooselyCoupledKalmanConfig)` calls `insEstimator.setConfig(config)`
  on a freshly built, unlocked object. Non-empty catches in those estimators (`GNSSException`, `INSException`,
  `AlgebraException`, `InertialNavigatorException`) already handle errors.
- Small findings:
  - `UselessOverridingMethod`: `ECEFGravity.java:78`, `ECIGravitation.java:78` (`hashCode()` that only calls
    `super.hashCode()`; `equals` in the same classes is a real override and stays).
  - `UnnecessaryFullyQualifiedName`: `INSLooselyCoupledKalmanEpochEstimator.java` 682, 697, 763, 769;
    `INSTightlyCoupledKalmanEpochEstimator.java:161`; `INSTightlyCoupledKalmanState.java:1509`;
    `navigators/ECIInertialNavigator.java:6405`.
  - `UselessParentheses`: `estimators/NEDGravityEstimator.java:174`, `estimators/NEDVelocityEstimator.java:1190`,
    `wmm/WMMEarthMagneticFluxDensityEstimator.java` 2587, 2673, 2676, `wmm/WMMLoader.java:185`.
- Precedent: `.archive/implementation_plan_47.md` (Checkstyle cleanup: wire tool config into `pom.xml`, relax
  high-volume rules, fix the rest).

## Implementation steps

### Group 1 — Ruleset, small rules and catch review (Parallelizable: yes — disjoint files)

- [x] Task 1. Wire a PMD ruleset into the build _(java)_
  - [x] Task 1.1. Create `pmd-ruleset.xml` at the repository root: reference the PMD `category/java/errorprone.xml`
    `EmptyCatchBlock` rule with `allowCommentedBlocks=true` and
    `allowExceptionNameRegex=^(ignore|ignored|expected)$`; keep the other default rules in use by the reporting run
    (e.g. reference `category/java/bestpractices.xml`, `codestyle.xml` rules that produced the other findings) so the
    current rule set does not shrink.
  - [x] Task 1.2. In `pom.xml` configure `maven-pmd-plugin` (shared plugin configuration, so `mvn pmd:pmd` from the CLI
    and the site report agree) with `<rulesets><ruleset>pmd-ruleset.xml</ruleset></rulesets>` and
    `<printFailingErrors>`/`<linkXRef>` as already used by the project; do not bind `pmd:check` to the lifecycle.
  - [x] Task 1.3. Run `mvn compile pmd:pmd` (JDK 21) and confirm the 483 `EmptyCatchBlock` findings drop to 0 except
    those reviewed in Task 3.
- [x] Task 2. Fix the 15 small findings _(java)_
  - [x] Task 2.1. Remove the redundant `hashCode()` overrides in `ECEFGravity.java` and `ECIGravitation.java` (keep the
    `equals` overrides; existing `ECEFGravityTest`/`ECIGravitationTest` `hashCode` assertions must still pass).
  - [x] Task 2.2. Replace the fully qualified names with imports (or simple names where already imported) in
    `INSLooselyCoupledKalmanEpochEstimator.java` (682, 697, 763, 769), `INSTightlyCoupledKalmanEpochEstimator.java:161`,
    `INSTightlyCoupledKalmanState.java:1509` and `navigators/ECIInertialNavigator.java:6405`; keep import order per
    `checkstyle.xml`.
  - [x] Task 2.3. Drop the useless parentheses in `NEDGravityEstimator.java:174`, `NEDVelocityEstimator.java:1190`,
    `WMMEarthMagneticFluxDensityEstimator.java` (2587, 2673, 2676) and `WMMLoader.java:185` without changing the
    evaluated expression.
- [x] Task 3. Review the empty catch blocks and fix those that can hide a failure _(java)_
  - [x] Task 3.1. Audit all `LockedException`, `WrongSizeException`, `AlgebraException` and
    `InvalidSourceAndDestinationFrameTypeException` empty catches, starting with the three Kalman filtered estimators
    (`INSGNSSLooselyCoupledKalmanFilteredEstimator`, `INSGNSSTightlyCoupledKalmanFilteredEstimator`,
    `INSLooselyCoupledKalmanFilteredEstimator`), then `navigators/**` and `estimators/**`, then `calibration/**`. A catch
    is "never happens" only if the guarded call cannot fail at that point (constructors on unlocked fresh objects,
    fixed-size matrix construction with constant sizes, frame types fixed by construction). Record the result in the
    task note (count reviewed, count changed).
    - Result: 483 empty catches reviewed (431 in constructors on fresh unlocked objects, 52 with constant-size matrices, validated arguments or frame types fixed by construction), 0 changed; all already carry a "never happens"/"no action needed" comment.
  - [x] Task 3.2. For every catch that can fail at runtime (e.g. a `LockedException` from a public setter or `update`
    path invoked while the estimator is running/locked), handle or propagate: rethrow the checked exception where the
    signature already allows it, or wrap in the type used by neighbouring non-empty catches
    (`INSException`/`IllegalStateException`), and add a unit test in the matching `*Test` class that exercises the
    failure path.
  - [x] Task 3.3. Where a "never happens" comment is too terse to justify the swallow, extend it with the reason
    (e.g. `// matrix size is constant, never happens`) in the touched files only; do not bulk-edit the rest.

  - Notes: pmd-ruleset.xml extends the plugin default ruleset (EmptyCatchBlock reconfigured); pom.xml pluginManagement + reporting. `mvn compile pmd:pmd` now reports 0 violations. 2.1: removing hashCode() made PMD raise OverrideBothEqualsAndHashcode, so hashCode() was kept with @SuppressWarnings("PMD.UselessOverridingMethod") and an explanatory Javadoc note. 3.2/3.3: nothing to change (no touched file contains a swallowed catch). Scoped tests: 160 run, 0 failures. Full suite deferred to Task 4.

### Group 2 — Validation (Parallelizable: yes — single task)

- [x] Task 4. Validate _(java)_
  - [x] Task 4.1. Run `mvn compile pmd:pmd` under JDK 21 and verify `target/pmd.xml` has 0 violations (or only
    findings explicitly justified in the plan notes).
  - [x] Task 4.2. Delegate the full `mvn test` run to the `iru-gate-runner` agent (JDK 21) and confirm it stays green;
    run `/iru-java-code-quality` against the touched files to check Checkstyle did not regress.
  - [x] Task 4.3. Run `/iru-check-license` on any new file (`pmd-ruleset.xml` is exempt if headers only apply to Java
    sources) and `/iru-update-docs` only if a docs page mentions static analysis or the PMD report.
