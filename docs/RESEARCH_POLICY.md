# Research-before-build Policy

Status: required for new subsystems and major features

## Before implementation

The issue or design note must contain a bounded prior-art search before code is
written. Search official GitHub organizations, paper indexes, project pages,
vendor documentation, and the relevant ROS 2 or simulator ecosystem.

Record at least:

1. The capability and measurable acceptance test.
2. Candidate projects and papers, including exact repository links.
3. Latest reviewed revision or release date.
4. License and redistribution constraints.
5. Supported robots, input formats, runtime, and simulator assumptions.
6. Maintenance signals: recent commits, releases, issues, and tests.
7. The decision: adopt, adapt, contribute upstream, reference only, or build.

Search popularity is not sufficient evidence. Prefer official repositories,
papers, standards, and vendor documentation over summaries or copied examples.

## Adoption gate

Before adding a dependency:

- create the smallest adapter spike;
- run the same public fixture through the candidate and OHMC contracts;
- compare residuals, constraints, determinism, runtime, and failure reporting;
- pin an immutable revision and preserve license/provenance evidence;
- isolate upstream types at an adapter boundary;
- document an offline/cache path and upgrade procedure.

When upstream behavior is missing, open an issue or propose a generally useful
change upstream before maintaining a fork. Local patches must stay small and
carry a link to the upstream discussion.

## Review cadence

Review [`UPSTREAM_ADOPTION.md`](UPSTREAM_ADOPTION.md) before every milestone and
at least once per quarter. A review may retire internal code after parity is
proven. It must never silently replace a pinned dependency or weaken evidence,
licensing, or physical-robot safety gates.

## Evidence required in a pull request

A major feature pull request must link its prior-art record and state:

- why the selected upstream was adopted or rejected;
- which code remains OHMC-specific;
- how license and provenance are preserved;
- which parity/regression tests prevent accidental divergence.
