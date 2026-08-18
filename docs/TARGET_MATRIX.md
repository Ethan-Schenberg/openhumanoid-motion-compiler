# One-command target matrix

`ohmc simulate INPUT --target all ...` runs every registered target compatible
with the input benchmark contract. It writes one atomic matrix directory rather
than stopping at the first failure.

The source hierarchy selects the benchmark family deterministically:

- a source containing all 16 `ohmc.full_body_landmarks/v0.1` names selects the
  `full_body_landmarks_v1` multi-limb targets;
- other sources select the `simple_motion_v1` smoke targets.

Each target executes in its own subdirectory. A successful row records the
relative bundle path and its `manifest.json` SHA-256. A failed row records the
exact error while allowing other targets to continue. The top-level result is
`fail` if any selected target failed, but the matrix and every successful child
bundle are preserved for diagnosis.

The matrix schema and semantic validator enforce unique target names, matching
pass/fail counts, consistent overall status, source provenance, and
`hardware_commands_sent: false`. The matrix never retries a failed target and
does not enable hardware transport.
