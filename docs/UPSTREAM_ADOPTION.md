# Upstream Adoption Map

Status: active engineering decision record

Last reviewed: 2026-08-18

OHMC is not intended to become another general-purpose IK library, physics
simulator, reinforcement-learning framework, or ROS 2 control stack. Its useful
boundary is a reproducible motion compiler and evidence harness across vendor
models: licensed input, explicit semantics, pinned dependencies, target-specific
adapters, quality gates, and independently verifiable artifacts.

## What already exists

| Upstream | What it already does well | License observed | OHMC decision |
|---|---|---|---|
| [Mink](https://github.com/kevinzakka/mink) | MuJoCo differential IK, task objectives, limits, collision avoidance, closed chains | Apache-2.0 | Preferred future IK backend. Keep OHMC DLS only as a small deterministic oracle until parity tests pass. |
| [GMR](https://github.com/YanjieZe/GMR) | General humanoid retargeting for G1 and other robots from BVH, SMPL/SMPL-X, FBX and live sources; built on Mink | MIT | Build an input/output adapter and benchmark it. Contribute an X2 configuration upstream when it is generally useful. |
| [ProtoMotions](https://github.com/NVlabs/ProtoMotions) | Large-scale humanoid motion tracking and imitation-learning pipelines across simulator backends | Apache-2.0 | Export versioned OHMC Motion IR and provenance to it; do not recreate its training stack. |
| [HumanoidVerse](https://github.com/LeCAR-Lab/HumanoidVerse) | Multi-simulator humanoid learning and deployment workflows | MIT | Treat as an optional downstream training backend behind an adapter. |
| [ASAP](https://github.com/LeCAR-Lab/ASAP) | Sim-to-real physics alignment for agile humanoid motion | MIT | Reuse its experimental method and interfaces where applicable; do not claim equivalent sim-to-real validation. |
| [SPIDER](https://github.com/facebookresearch/spider) | Physics-informed retargeting layered on Mink | CC BY-NC 4.0 | Paper/reference benchmark only in the default Apache-2.0 distribution unless license compatibility is resolved. |
| [H2O / OmniH2O](https://github.com/LeCAR-Lab/human2humanoid) | Learned whole-body humanoid teleoperation | CC BY-NC 4.0 | Research reference only; no source dependency in the default distribution. |
| [GVHMR](https://github.com/zju3dv/GVHMR) | World-grounded human motion recovery from video | Research/non-profit terms | Optional external frontend evaluation only; video pose recovery stays out of OHMC core. |
| [ros2_control](https://github.com/ros-controls/ros2_control) | Standard ROS 2 hardware/control framework | Apache-2.0 | Any future live adapter must integrate through standard ROS 2 control boundaries instead of inventing a transport stack. |
| [AgiBot X2 URDF](https://github.com/AgibotTech/agibot_x2_urdf) | Official X2 v1.3/v1.4 robot descriptions | MulanPSL-2.0 | Adopted as a pinned Git dependency at `77f43eb...`; the former copied ZIP workflow is retired for the model. |

Licenses in this table are a project-maintainer screening record, not legal
advice. Every revision update must re-check the upstream license and notices.

## Build, adopt, or contribute upstream

OHMC should build code only when at least one of these is true:

- The behavior is part of OHMC's versioned compiler or evidence contract.
- No maintained permissively licensed upstream satisfies the acceptance tests.
- A compatibility layer is needed to join otherwise incompatible upstreams.
- A small reference implementation materially improves deterministic auditing.

If an upstream almost fits, the default order is: adapter spike, parity test,
upstream issue or pull request, then the smallest local compatibility layer. A
large local fork is the last resort.

## Project-specific value

The durable OHMC work is therefore:

- a solver-neutral Motion IR and retargeting problem/evidence contract;
- identical provenance, coverage, safety, and regression gates for G1 and X2;
- pinned official vendor models and explicit redistribution decisions;
- one-command offline target matrices with independently verifiable manifests;
- adapters to mature retargeting, learning, simulation, and ROS 2 ecosystems.

The internal DLS solver must not grow into a competing general IK framework.
New task types should first target a Mink backend, while DLS remains a compact
fixture and regression oracle.

## Research reading queue

1. [Retargeting Matters: General Motion Retargeting for Humanoid Motion Tracking](https://arxiv.org/abs/2510.02252) — establish GMR parity metrics and failure classes.
2. [ASAP: Aligning Simulation and Real-World Physics for Learning Agile Humanoid Whole-Body Skills](https://arxiv.org/abs/2502.01143) — define what evidence would be required before any sim-to-real claim.
3. [H2O](https://human2humanoid.com/) and [OmniH2O](https://openreview.net/pdf?id=oL1WEZQal8) — study real-time whole-body control interfaces without importing non-commercial code.
4. [ProtoMotions](https://github.com/NVlabs/ProtoMotions) — define the first stable Motion IR export boundary for downstream policy training.

Every reading item must end in a short decision record: adopt, adapt, contribute
upstream, reference only, or reject, with the evidence and license recorded.
