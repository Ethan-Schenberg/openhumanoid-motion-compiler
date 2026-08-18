# Full-body landmark coverage

`ohmc.full_body_landmarks/v0.1` defines 16 source landmarks: pelvis (`Hips`),
spine, chest, head, bilateral shoulders, elbows, wrists, hips, knees, and
ankles. The names are a compiler contract, not an assertion that every input
format or robot exposes identical anatomy.

`ohmc landmark-report` measures two independent dimensions:

- source coverage: which required landmarks exist in canonical motion;
- task coverage: which required landmarks are consumed by an IK task map.

The report includes exact present/missing lists, counts, ratios, input content
hash, and `hardware_commands_sent: false`. A partial source or map produces a
preserved `warning` report. `--require-full-source` and `--require-full-tasks`
turn the selected gap into a non-zero command result without discarding the
report.

The original `examples/full_body_motion.bvh` fixture covers 16/16 source
landmarks. Current vendor multi-limb maps cover 9/16 task landmarks; this is why
the project does not yet claim whole-body IK.
