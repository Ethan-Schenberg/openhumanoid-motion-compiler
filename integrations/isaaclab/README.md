# OHMC X2 Isaac Lab extension

This extension registers `OHMC-X2-RGBD-Rough-v0` and its one-environment play task. It is deliberately separate from the lightweight OHMC web/process environment because the pinned Isaac Lab beta uses Python 3.12 and Isaac Sim 6.0.1.

## Locked upstream

The machine-readable lock is [`integration-lock.yaml`](integration-lock.yaml):

- Isaac Lab `v3.0.0-beta2.patch1`, commit `ffff603eafc6b74264a5261cc0183d6a65390d78`.
- RSL-RL `5.0.1`.
- Isaac Sim container base `nvcr.io/nvidia/isaac-sim:6.0.1`.
- Official X2 URDF v1.3.0 with SHA-256 `1163b3c7…f7f11`.

The container tag is not treated as an immutable digest. A production target must capture the pulled image digest; the lock intentionally remains `requires_target_validation` until that is done.

## Native WSL2 setup

Follow the official Isaac Lab installation prerequisites, then:

```bash
git clone https://github.com/isaac-sim/IsaacLab.git /workspace/IsaacLab
git -C /workspace/IsaacLab checkout ffff603eafc6b74264a5261cc0183d6a65390d78

export OHMC_ISAACLAB_ROOT=/workspace/IsaacLab
export OHMC_ISAACLAB_PYTHON=/workspace/IsaacLab/_isaac_sim/python.sh

"$OHMC_ISAACLAB_PYTHON" -m pip install -e /workspace/ohmc/integrations/isaaclab
"$OHMC_ISAACLAB_PYTHON" -c 'import ohmc_x2; from importlib.metadata import version; print(version("rsl-rl-lib"))'
```

Use the complete official model checkout because the URDF references sibling `meshes/` files:

```bash
export OHMC_X2_URDF=/workspace/ohmc/.ohmc-cache/sources/agibot_x2/urdf/X2_URDF-v1.3.0/x2_ultra.urdf
```

Run the root environment doctor before training:

```bash
cd /workspace/ohmc
.venv/bin/ohmc doctor
```

## Container route

Build Isaac Lab from the locked upstream checkout using its own `docker/Dockerfile.base` and `docker/docker-compose.yaml`. The expected hashes of those two files are in `integration-lock.yaml`; verify them before building. Install this extension into the resulting image or mounted workspace with the Isaac Python command above.

The container needs the NVIDIA runtime, persistent mounts for the OHMC run directory and X2 model directory, and no robot network access. Do not bake vendor credentials or a hardware transport into the image.

## Outputs and recovery

`python -m ohmc_x2.train --recipe … --run-dir …` runs the ordered curriculum and emits:

- `curriculum-progress.json` after each completed phase;
- `checkpoint.pt`;
- `policy.onnx`;
- `preview.mp4`;
- `backend-result.json` with `simulation_only_unevaluated` authority.

If the host stops, rerunning the wrapper reads `curriculum-progress.json`, locates the same run's newest checkpoint, and continues at the first incomplete stage. It never searches for or downloads a published pretrained checkpoint.

## Validation boundary

The configuration is source-compatible with the pinned upstream APIs and is covered by OHMC contract tests. It has not yet been instantiated on the target WSL2 RTX 3070 host. In particular, camera frame orientation, initial standing height, provisional simulation PD gains, collision performance, memory use and learning quality require target validation before results can pass E3 Sim2Sim.
