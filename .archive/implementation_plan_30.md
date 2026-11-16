# Implementation Plan

## Task summary

Source: GitHub issue #30
Base branch: develop

Update the `com.irurueta` dependencies declared in `pom.xml` to their latest patch versions, as requested by the
issue (title "Update dependencies"). The issue is classified as a feature (no labels, maintenance work).

Approach chosen: edit only the `<version>` elements in `pom.xml`; no API changes are expected because every bump
is a patch release. Verification is by compiling and running the full test suite plus the project's quality gates.

Open clarifications: none

## Acceptance criteria coverage

| # | Criterion | Covered by tasks |
|---|---|---|
| 1 | derived: `irurueta-numerical` and `irurueta-geometry` are at 1.6.1 in `pom.xml` | Task 1 |
| 2 | derived: `irurueta-units` and `irurueta-algebra` are at 1.4.1 in `pom.xml` | Task 1 |
| 3 | derived: `irurueta-navigation` is at 1.8.2 in `pom.xml` | Task 1 |
| 4 | derived: the project compiles and all existing tests pass with the new versions | Task 2 |

## Current code state

- Build: Maven single module (`pom.xml`, no wrapper), Java library `irurueta-navigation-inertial` version
  `1.11.0-SNAPSHOT`.
- Dependencies in `pom.xml` (lines ~60-89), all `compile` scope:
  - `irurueta-numerical` 1.6.0 → 1.6.1
  - `irurueta-geometry` 1.6.0 → 1.6.1
  - `irurueta-units` 1.4.0 → 1.4.1
  - `irurueta-algebra` 1.4.0 → 1.4.1
  - `irurueta-navigation` 1.8.1 → 1.8.2
- Docs (`docs/`, Antora) and `README.md` do not mention these versions; no docs change needed.
- Tests live under `src/test/java`.

## Implementation steps

### Group 1 — Bump dependencies and verify (Parallelizable: yes — single task group, tasks are sequential)

- [x] Task 1. Update dependency versions in `pom.xml` _(java)_ — pom.xml only; no tests added.
  - [x] Task 1.1. Set `irurueta-numerical` and `irurueta-geometry` `<version>` to `1.6.1`
  - [x] Task 1.2. Set `irurueta-units` and `irurueta-algebra` `<version>` to `1.4.1`
  - [x] Task 1.3. Set `irurueta-navigation` `<version>` to `1.8.2`
- [x] Task 2. Verify the build _(java)_
  - [x] Task 2.1. Compile and run the full test suite via the `iru-gate-runner` agent (`iru-java-test`); fix any
    failure caused by the new versions
  - [x] Task 2.2. Run `iru-java-coverage` and `iru-java-code-quality` via `iru-gate-runner` to confirm no regression — 13912 tests, only intermittent pre-existing flaky robust-calibrator tests; coverage 96.78% line, 92.60% branch; Checkstyle/SpotBugs unchanged; PMD 505 identical on pre-bump pom (baseline artifact).
