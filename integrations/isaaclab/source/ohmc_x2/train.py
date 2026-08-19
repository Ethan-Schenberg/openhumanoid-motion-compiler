"""Run the pinned Isaac Lab curriculum and export visible policy artifacts."""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

import yaml

ISAAC_LAB_REVISION = "ffff603eafc6b74264a5261cc0183d6a65390d78"
RSL_RL_VERSION = "5.0.1"
X2_URDF_SHA256 = "1163b3c76b31c4ea0afd284b67b28948003ce46f7ea4b2d9826f1309e9af7f11"
CURRICULUM = (
    ("00_stand", 0.04),
    ("01_flat", 0.16),
    ("02_slope", 0.16),
    ("03_uneven", 0.20),
    ("04_low_obstacle", 0.20),
    ("05_stairs", 0.24),
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_json(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}-", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(document, stream, indent=2, sort_keys=True)
            stream.write("\n")
        Path(temporary_name).replace(path)
    except Exception:
        Path(temporary_name).unlink(missing_ok=True)
        raise


def _load_recipe(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        recipe = yaml.safe_load(stream)
    if not isinstance(recipe, dict):
        raise TypeError("TrainingRecipe must be an object")
    if recipe.get("initialization") != "random":
        raise RuntimeError("production training must start from random initialization")
    versions = recipe.get("backend", {}).get("versions", {})
    expected = {
        "isaac_lab": "v3.0.0-beta2.patch1",
        "isaac_lab_revision": ISAAC_LAB_REVISION,
        "rsl_rl": RSL_RL_VERSION,
    }
    if versions != expected:
        raise RuntimeError(
            f"recipe backend lock differs from integration: {versions!r}"
        )
    return recipe


def _preflight() -> tuple[Path, Path]:
    root_raw = os.environ.get("OHMC_ISAACLAB_ROOT")
    urdf_raw = os.environ.get("OHMC_X2_URDF")
    if not root_raw or not urdf_raw:
        raise RuntimeError("OHMC_ISAACLAB_ROOT and OHMC_X2_URDF are required")
    root = Path(root_raw).expanduser().resolve()
    urdf = Path(urdf_raw).expanduser().resolve()
    revision = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=root,
        text=True,
        capture_output=True,
        check=True,
    ).stdout.strip()
    if revision != ISAAC_LAB_REVISION:
        raise RuntimeError(
            f"Isaac Lab revision {revision} does not match {ISAAC_LAB_REVISION}"
        )
    installed_rsl = importlib.metadata.version("rsl-rl-lib")
    if installed_rsl != RSL_RL_VERSION:
        raise RuntimeError(
            f"rsl-rl-lib {installed_rsl} does not match {RSL_RL_VERSION}"
        )
    if _sha256(urdf) != X2_URDF_SHA256:
        raise RuntimeError("X2 URDF hash does not match the locked official model")
    return root, urdf


def _latest_checkpoint(run_dir: Path) -> Path | None:
    checkpoints = list(
        (run_dir / "logs" / "rsl_rl" / "ohmc_x2_rgbd_rough").glob("**/model_*.pt")
    )
    return (
        max(checkpoints, key=lambda path: path.stat().st_mtime) if checkpoints else None
    )


def _stage_iterations(total: int) -> list[int]:
    if total < len(CURRICULUM):
        raise ValueError(
            "max_iterations must allocate at least one iteration per stage"
        )
    values = [max(1, int(total * fraction)) for _, fraction in CURRICULUM]
    values[-1] += total - sum(values)
    return values


def _run(command: list[str], env: dict[str, str]) -> None:
    print(f"[OHMC] {' '.join(command)}", flush=True)
    subprocess.run(command, env=env, check=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--recipe", type=Path, required=True)
    parser.add_argument("--run-dir", type=Path, required=True)
    arguments = parser.parse_args(argv)

    recipe_path = arguments.recipe.expanduser().resolve()
    run_dir = arguments.run_dir.expanduser().resolve()
    recipe = _load_recipe(recipe_path)
    isaac_root, _ = _preflight()
    train_script = (
        isaac_root / "scripts" / "reinforcement_learning" / "rsl_rl" / "train.py"
    )
    play_script = (
        isaac_root / "scripts" / "reinforcement_learning" / "rsl_rl" / "play.py"
    )
    if not train_script.is_file() or not play_script.is_file():
        raise RuntimeError("pinned Isaac Lab RSL-RL scripts are missing")

    num_envs = int(
        os.environ.get("OHMC_ENV_COUNT", recipe["compute"]["default_env_count"])
    )
    seed = int(recipe["algorithm"]["seed"])
    total_iterations = int(recipe["algorithm"]["max_iterations"])
    progress_path = run_dir / "curriculum-progress.json"
    completed: list[str] = []
    if progress_path.is_file():
        with progress_path.open(encoding="utf-8") as stream:
            progress = json.load(stream)
        completed = list(progress.get("completed_stages", []))

    checkpoint = _latest_checkpoint(run_dir)
    for (stage, _), iterations in zip(CURRICULUM, _stage_iterations(total_iterations)):
        if stage in completed:
            continue
        env = {
            **os.environ,
            "OHMC_CURRICULUM_STAGE": stage,
            "OHMC_NUM_STEPS_PER_ENV": str(recipe["algorithm"]["num_steps_per_env"]),
            "OHMC_SAVE_INTERVAL": str(recipe["algorithm"]["save_interval"]),
        }
        command = [
            sys.executable,
            str(train_script),
            "--task",
            "OHMC-X2-RGBD-Rough-v0",
            "--external_callback",
            "ohmc_x2.register",
            "--num_envs",
            str(num_envs),
            "--seed",
            str(seed),
            "--max_iterations",
            str(iterations),
            "--experiment_name",
            "ohmc_x2_rgbd_rough",
            "--device",
            str(recipe["compute"]["device"]),
            "--viz",
            "none",
            "--enable_cameras",
            "--export_io_descriptors",
        ]
        if checkpoint is not None:
            command.extend(["--resume", "--checkpoint", str(checkpoint)])
        _run(command, env)
        checkpoint = _latest_checkpoint(run_dir)
        if checkpoint is None:
            raise RuntimeError(f"stage {stage} did not emit a checkpoint")
        completed.append(stage)
        _atomic_json(
            progress_path,
            {
                "schema": "ohmc.curriculum_progress/v0.1",
                "completed_stages": completed,
                "checkpoint": str(checkpoint.relative_to(run_dir)),
                "initialization": "random",
            },
        )

    if checkpoint is None:
        raise RuntimeError("no training checkpoint is available")
    shutil.copy2(checkpoint, run_dir / "checkpoint.pt")

    play_env = {**os.environ, "OHMC_CURRICULUM_STAGE": CURRICULUM[-1][0]}
    _run(
        [
            sys.executable,
            str(play_script),
            "--task",
            "OHMC-X2-RGBD-Rough-Play-v0",
            "--external_callback",
            "ohmc_x2.register",
            "--checkpoint",
            str(checkpoint),
            "--num_envs",
            "1",
            "--device",
            str(recipe["compute"]["device"]),
            "--viz",
            "none",
            "--enable_cameras",
            "--video",
            "--video_length",
            "500",
        ],
        play_env,
    )

    export_dir = checkpoint.parent / "exported"
    policy = export_dir / "policy.onnx"
    if not policy.is_file():
        raise RuntimeError("Isaac Lab play completed without policy.onnx")
    shutil.copy2(policy, run_dir / "policy.onnx")
    videos = list((checkpoint.parent / "videos" / "play").glob("*.mp4"))
    if not videos:
        raise RuntimeError("Isaac Lab play completed without an MP4 preview")
    preview = max(videos, key=lambda path: path.stat().st_mtime)
    shutil.copy2(preview, run_dir / "preview.mp4")
    _atomic_json(
        run_dir / "backend-result.json",
        {
            "schema": "ohmc.isaaclab_backend_result/v0.1",
            "checkpoint": "checkpoint.pt",
            "policy": "policy.onnx",
            "preview": "preview.mp4",
            "curriculum": [stage for stage, _ in CURRICULUM],
            "authority": "simulation_only_unevaluated",
        },
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
