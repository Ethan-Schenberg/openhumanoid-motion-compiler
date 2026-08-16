# Contributing to OHMC

OHMC welcomes reproducible bug reports, documentation fixes, robot profiles, motion fixtures, compiler passes, validation metrics, and vendor-adapter improvements.

## Before contributing

- Do not submit credentials, private network details, field logs, personal recordings, or restricted vendor assets.
- Confirm that every added model, motion, dataset, SDK fragment, and generated artifact can legally be redistributed.
- Keep physical-robot execution out of ordinary tests and examples.
- Open an issue before proposing a breaking Motion IR schema change.

## Development setup

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[dev]'
.venv/bin/python -m pytest
.venv/bin/ohmc validate-ir examples/minimal_motion.json
```

## Pull requests

A pull request should explain:

- The problem and intended behavior.
- The affected Motion IR, robot profile, compiler pass, or vendor integration.
- Tests and fixtures used for validation.
- Licensing and provenance for new third-party material.
- Whether the change affects simulation, recorded interfaces, or physical hardware.

New behavior requires tests. Safety or compatibility warnings must not be weakened simply to make an example pass.

## Developer Certificate of Origin

By contributing, you certify that you have the right to submit the work under the project's Apache-2.0 license. Add a `Signed-off-by` line to commits when requested during review.

