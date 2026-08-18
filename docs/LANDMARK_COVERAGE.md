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
landmarks. The current vendor multi-limb maps also consume all 16 landmarks as
explicit position tasks. Targets declaring `full_body_landmarks_v1` now fail
if either source or task coverage is incomplete.

Complete landmark-name coverage is not the same as complete whole-body IK.
The Unitree G1 29DoF model has no articulated head, so its head landmark uses a
documented torso-frame position proxy. The current contract also lacks frame
orientation, contact, balance, and collision tasks. Manifests therefore expose
`full_body_landmark_position_tasks: true` while continuing to set
`constrained_whole_body_ik: false`.
