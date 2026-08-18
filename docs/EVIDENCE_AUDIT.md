# Evidence integrity audit

`ohmc verify-evidence DIRECTORY` independently verifies an OHMC simulation
bundle or target matrix without loading a robot model, running MuJoCo, or
enabling hardware transport.

For a bundle it checks:

- manifest JSON shape against `ohmc.simulation_bundle/v0.1`;
- every artifact path is relative and remains inside the bundle;
- every declared artifact exists;
- every artifact SHA-256 matches the manifest.

For a matrix it additionally checks target uniqueness, pass/fail counts,
overall status, each successful child manifest hash, and the complete contents
of every successful child bundle. Recorded execution failures are not integrity
failures: a matrix may faithfully report that a vendor dependency was absent.

The deterministic `ohmc.evidence_audit/v0.1` report contains no timestamp. It
records the root manifest hash, checked bundle/artifact counts, exact issues,
and `hardware_commands_sent: false`.

```bash
ohmc verify-evidence build/full-body-matrix \
  --report build/full-body-matrix-audit.json
```

A mismatch returns a non-zero exit code without modifying the evidence.
