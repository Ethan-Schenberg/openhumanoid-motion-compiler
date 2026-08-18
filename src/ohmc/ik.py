"""Solver-neutral IK contracts and a deterministic MuJoCo reference solver."""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator

from .canonical import object_sha256
from .errors import OhmcError


def _schema_issues(document: dict[str, Any], schema: dict[str, Any]) -> list[str]:
    issues: list[str] = []
    for error in sorted(
        Draft202012Validator(schema).iter_errors(document),
        key=lambda item: list(item.path),
    ):
        location = ".".join(str(part) for part in error.absolute_path) or "$"
        issues.append(f"{location}: {error.message}")
    return issues


def validate_ik_task_map(
    document: dict[str, Any], schema: dict[str, Any]
) -> list[str]:
    issues = _schema_issues(document, schema)
    if issues:
        return issues
    task_ids = [task["id"] for task in document["tasks"]]
    if len(task_ids) != len(set(task_ids)):
        issues.append("tasks: ids must be unique")
    for field in (
        "damping",
        "max_step_rad",
        "neutral_weight",
        "temporal_weight",
    ):
        if not math.isfinite(float(document["solver"][field])):
            issues.append(f"solver.{field}: must be finite")
    for index, task in enumerate(document["tasks"]):
        for field in ("weight", "tolerance_m", "scale"):
            if not math.isfinite(float(task[field])):
                issues.append(f"tasks.{index}.{field}: must be finite")
    return issues


def validate_ik_problem(
    document: dict[str, Any], schema: dict[str, Any]
) -> list[str]:
    issues = _schema_issues(document, schema)
    if issues:
        return issues
    variable_names = [variable["name"] for variable in document["variables"]]
    if len(variable_names) != len(set(variable_names)):
        issues.append("variables: names must be unique")
    if not math.isfinite(float(document["source"]["rate_hz"])):
        issues.append("source.rate_hz: must be finite")
    for index, variable in enumerate(document["variables"]):
        lower = float(variable["lower"])
        upper = float(variable["upper"])
        initial = float(variable["initial"])
        if not all(math.isfinite(value) for value in (lower, upper, initial)):
            issues.append(f"variables.{index}: bounds and initial value must be finite")
        elif lower >= upper:
            issues.append(f"variables.{index}: lower must be less than upper")
        elif not lower <= initial <= upper:
            issues.append(f"variables.{index}.initial: must be within joint limits")
    task_ids = [task["id"] for task in document["tasks"]]
    if len(task_ids) != len(set(task_ids)):
        issues.append("tasks: ids must be unique")
    for index, task in enumerate(document["tasks"]):
        task_values = [task["weight"], task["tolerance_m"], task["scale"]]
        task_values.extend(task["source_reference_m"])
        task_values.extend(task["target_reference_m"])
        if any(not math.isfinite(float(value)) for value in task_values):
            issues.append(f"tasks.{index}: numeric values must be finite")
    for field in (
        "damping",
        "max_step_rad",
        "neutral_weight",
        "temporal_weight",
    ):
        if not math.isfinite(float(document["solver"][field])):
            issues.append(f"solver.{field}: must be finite")
    expected = set(task_ids)
    previous_time: float | None = None
    for frame_index, frame in enumerate(document["frames"]):
        timestamp = float(frame["time"])
        if not math.isfinite(timestamp):
            issues.append(f"frames.{frame_index}.time: must be finite")
        elif previous_time is not None and timestamp <= previous_time:
            issues.append(f"frames.{frame_index}.time: must be strictly increasing")
        previous_time = timestamp
        actual = [target["task_id"] for target in frame["targets"]]
        if len(actual) != len(set(actual)) or set(actual) != expected:
            issues.append(
                f"frames.{frame_index}.targets: must contain each task exactly once"
            )
        if any(
            not math.isfinite(float(value))
            for target in frame["targets"]
            for value in target["position_m"]
        ):
            issues.append(f"frames.{frame_index}.targets: positions must be finite")
    return issues


def validate_ik_result(
    document: dict[str, Any], schema: dict[str, Any]
) -> list[str]:
    issues = _schema_issues(document, schema)
    if issues:
        return issues
    variable_count = len(document["variables"])
    variable_names = set(document["variables"])
    solved = 0
    peak = 0.0
    previous_time: float | None = None
    expected_tasks: set[str] | None = None
    for index, frame in enumerate(document["frames"]):
        if len(frame["positions"]) != variable_count:
            issues.append(
                f"frames.{index}.positions: expected {variable_count} values, "
                f"got {len(frame['positions'])}"
            )
        if any(not math.isfinite(float(value)) for value in frame["positions"]):
            issues.append(f"frames.{index}.positions: must be finite")
        if not math.isfinite(float(frame["max_residual_m"])):
            issues.append(f"frames.{index}.max_residual_m: must be finite")
        if any(
            not math.isfinite(float(value))
            for value in frame["task_residuals_m"].values()
        ):
            issues.append(f"frames.{index}.task_residuals_m: must be finite")
        timestamp = float(frame["time"])
        if not math.isfinite(timestamp):
            issues.append(f"frames.{index}.time: must be finite")
        elif previous_time is not None and timestamp <= previous_time:
            issues.append(f"frames.{index}.time: must be strictly increasing")
        previous_time = timestamp
        actual_tasks = set(frame["task_residuals_m"])
        if expected_tasks is None:
            expected_tasks = actual_tasks
        elif actual_tasks != expected_tasks:
            issues.append(f"frames.{index}.task_residuals_m: task keys changed")
        unknown_limits = sorted(set(frame["active_joint_limits"]) - variable_names)
        if unknown_limits:
            issues.append(
                f"frames.{index}.active_joint_limits: unknown variables {unknown_limits}"
            )
        if frame["status"] == "solved":
            solved += 1
        peak = max(peak, float(frame["max_residual_m"]))
    summary = document["summary"]
    frame_count = len(document["frames"])
    if summary["frame_count"] != frame_count:
        issues.append("summary.frame_count does not match frames")
    if summary["solved_frame_count"] != solved:
        issues.append("summary.solved_frame_count does not match frames")
    if summary["failed_frame_count"] != frame_count - solved:
        issues.append("summary.failed_frame_count does not match frames")
    if not math.isclose(
        float(summary["peak_residual_m"]), peak, rel_tol=0.0, abs_tol=1e-12
    ):
        issues.append("summary.peak_residual_m does not match frames")
    expected_status = "pass" if solved == frame_count else "fail"
    if document["status"] != expected_status:
        issues.append(f"status must be {expected_status!r} for the frame results")
    return issues


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_mujoco() -> Any:
    try:
        import mujoco
    except ImportError as exc:  # pragma: no cover - depends on optional install
        raise OhmcError(
            "MuJoCo IK requires the optional dependency; install with "
            "python -m pip install -e '.[mujoco]'"
        ) from exc
    return mujoco


def build_ik_problem(
    canonical_motion: dict[str, Any],
    profile: dict[str, Any],
    task_map: dict[str, Any],
    model_path: Path,
) -> dict[str, Any]:
    """Compile canonical reference-delta tasks into a solver-neutral problem."""
    mujoco = _load_mujoco()
    try:
        model = mujoco.MjModel.from_xml_path(str(model_path))
    except (ValueError, OSError) as exc:
        raise OhmcError(f"failed to load MuJoCo model {model_path}: {exc}") from exc
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    source_joints = canonical_motion["skeleton"]["joints"]
    source_index = {joint["name"]: index for index, joint in enumerate(source_joints)}
    semantic_to_joint = profile["semantics"]
    variables: list[dict[str, Any]] = []
    for semantic in task_map["variables"]:
        if semantic not in semantic_to_joint:
            raise OhmcError(f"IK variable semantic is absent from profile: {semantic}")
        name = semantic_to_joint[semantic]
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise OhmcError(f"IK variable joint is absent from MuJoCo model: {name}")
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            raise OhmcError(f"IK variable must be a scalar hinge joint: {name}")
        limit = profile["joint_limits"][name]
        if bool(model.jnt_limited[joint_id]):
            model_lower, model_upper = (
                float(value) for value in model.jnt_range[joint_id]
            )
            if not math.isclose(
                float(limit["lower"]), model_lower, rel_tol=0.0, abs_tol=1e-6
            ) or not math.isclose(
                float(limit["upper"]), model_upper, rel_tol=0.0, abs_tol=1e-6
            ):
                raise OhmcError(
                    f"IK profile/model limit mismatch for {name}: profile "
                    f"[{limit['lower']}, {limit['upper']}], model "
                    f"[{model_lower}, {model_upper}]"
                )
        qpos_address = int(model.jnt_qposadr[joint_id])
        initial = min(
            float(limit["upper"]),
            max(float(limit["lower"]), float(data.qpos[qpos_address])),
        )
        variables.append(
            {
                "name": name,
                "lower": float(limit["lower"]),
                "upper": float(limit["upper"]),
                "initial": initial,
            }
        )

    first_sample = canonical_motion["samples"][0]
    tasks: list[dict[str, Any]] = []
    for config in task_map["tasks"]:
        source_name = config["source_joint"]
        if source_name not in source_index:
            raise OhmcError(f"IK source joint is absent from canonical motion: {source_name}")
        body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, config["target_frame"]
        )
        if body_id < 0:
            raise OhmcError(
                f"IK target frame is absent from MuJoCo model: {config['target_frame']}"
            )
        tasks.append(
            {
                "id": config["id"],
                "kind": "frame_position",
                "source_joint": source_name,
                "target_frame": config["target_frame"],
                "weight": float(config["weight"]),
                "tolerance_m": float(config["tolerance_m"]),
                "source_reference_m": list(
                    first_sample["world_positions_m"][source_index[source_name]]
                ),
                "target_reference_m": [
                    float(value) for value in data.xpos[body_id]
                ],
                "scale": float(config["scale"]),
            }
        )

    frames = []
    for sample in canonical_motion["samples"]:
        targets = []
        for task in tasks:
            source_position = sample["world_positions_m"][source_index[task["source_joint"]]]
            position = [
                task["target_reference_m"][axis]
                + task["scale"]
                * (source_position[axis] - task["source_reference_m"][axis])
                for axis in range(3)
            ]
            targets.append({"task_id": task["id"], "position_m": position})
        frames.append({"time": sample["time"], "targets": targets})

    if len(frames) > 1:
        rate_hz = 1.0 / (float(frames[1]["time"]) - float(frames[0]["time"]))
    else:
        normalization_pass = canonical_motion["passes"][-1]
        rate_hz = float(normalization_pass.get("metrics", {}).get("target_rate_hz", 1.0))
    solver = {"method": "damped_least_squares", **task_map["solver"]}
    solver_model_sha256 = _sha256_file(model_path)
    return {
        "schema": "ohmc.ik_problem/v0.1",
        "source": {
            "canonical_motion_sha256": object_sha256(
                {
                    "frames": canonical_motion["frames"],
                    "skeleton": canonical_motion["skeleton"],
                    "samples": canonical_motion["samples"],
                }
            ),
            "rate_hz": rate_hz,
        },
        "robot": {
            "profile": profile["id"],
            "profile_model_sha256": profile["model_evidence"]["model_sha256"],
            "solver_model_sha256": solver_model_sha256,
        },
        "variables": variables,
        "tasks": tasks,
        "frames": frames,
        "solver": solver,
        "provenance": {
            "task_map": task_map["id"],
            "config_sha256": object_sha256(
                {"task_map": task_map, "solver_model_sha256": solver_model_sha256}
            ),
            "hardware_commands_sent": False,
        },
    }


def solve_ik_problem(problem: dict[str, Any], model_path: Path) -> dict[str, Any]:
    """Solve position tasks with bounded damped least squares and no fallback."""
    mujoco = _load_mujoco()
    try:
        import numpy as np
    except ImportError as exc:  # pragma: no cover - MuJoCo installs NumPy
        raise OhmcError("MuJoCo IK requires NumPy") from exc
    actual_sha256 = _sha256_file(model_path)
    if actual_sha256 != problem["robot"]["solver_model_sha256"]:
        raise OhmcError(
            "IK solver model SHA-256 mismatch: "
            f"expected {problem['robot']['solver_model_sha256']}, got {actual_sha256}"
        )
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    qpos_addresses: list[int] = []
    dof_addresses: list[int] = []
    for variable in problem["variables"]:
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, variable["name"]
        )
        if joint_id < 0:
            raise OhmcError(f"IK variable joint is absent from model: {variable['name']}")
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            raise OhmcError(
                f"IK variable must resolve to a scalar hinge joint: {variable['name']}"
            )
        qpos_addresses.append(int(model.jnt_qposadr[joint_id]))
        dof_addresses.append(int(model.jnt_dofadr[joint_id]))
    task_body_ids = {
        task["id"]: mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, task["target_frame"]
        )
        for task in problem["tasks"]
    }
    if any(body_id < 0 for body_id in task_body_ids.values()):
        raise OhmcError("IK target frame is absent from solver model")

    lower = np.asarray([item["lower"] for item in problem["variables"]], dtype=float)
    upper = np.asarray([item["upper"] for item in problem["variables"]], dtype=float)
    neutral = np.asarray([item["initial"] for item in problem["variables"]], dtype=float)
    previous = neutral.copy()
    solver = problem["solver"]
    frame_results: list[dict[str, Any]] = []
    for frame in problem["frames"]:
        q = previous.copy()
        target_by_id = {
            target["task_id"]: np.asarray(target["position_m"], dtype=float)
            for target in frame["targets"]
        }
        final_residuals: dict[str, float] = {}
        status = "failed"
        updates = 0
        for iteration in range(int(solver["max_iterations"]) + 1):
            for index, address in enumerate(qpos_addresses):
                data.qpos[address] = q[index]
            mujoco.mj_forward(model, data)
            jacobian_blocks = []
            error_blocks = []
            final_residuals = {}
            converged = True
            for task in problem["tasks"]:
                body_id = task_body_ids[task["id"]]
                error = target_by_id[task["id"]] - data.xpos[body_id]
                residual = float(np.linalg.norm(error))
                final_residuals[task["id"]] = residual
                if residual > float(task["tolerance_m"]):
                    converged = False
                jacobian = np.zeros((3, model.nv), dtype=float)
                rotational = np.zeros((3, model.nv), dtype=float)
                mujoco.mj_jacBody(model, data, jacobian, rotational, body_id)
                weight = math.sqrt(float(task["weight"]))
                jacobian_blocks.append(jacobian[:, dof_addresses] * weight)
                error_blocks.append(error * weight)
            if converged:
                status = "solved"
                break
            if iteration == int(solver["max_iterations"]):
                break
            jacobian = np.vstack(jacobian_blocks)
            error = np.concatenate(error_blocks)
            damping = float(solver["damping"])
            neutral_weight = float(solver["neutral_weight"])
            temporal_weight = float(solver["temporal_weight"])
            matrix = jacobian.T @ jacobian
            matrix += (
                damping * damping + neutral_weight + temporal_weight
            ) * np.eye(len(q))
            gradient = jacobian.T @ error
            gradient += neutral_weight * (neutral - q)
            gradient += temporal_weight * (previous - q)
            delta = np.linalg.solve(matrix, gradient)
            max_step = float(solver["max_step_rad"])
            delta = np.clip(delta, -max_step, max_step)
            candidate = np.clip(q + delta, lower, upper)
            updates += 1
            if np.array_equal(candidate, q):
                q = candidate
                break
            q = candidate

        active_limits = [
            problem["variables"][index]["name"]
            for index in range(len(q))
            if math.isclose(float(q[index]), float(lower[index]), abs_tol=1e-12)
            or math.isclose(float(q[index]), float(upper[index]), abs_tol=1e-12)
        ]
        max_residual = max(final_residuals.values())
        frame_results.append(
            {
                "time": frame["time"],
                "status": status,
                "iterations": updates,
                "positions": [float(value) for value in q],
                "max_residual_m": max_residual,
                "task_residuals_m": final_residuals,
                "active_joint_limits": active_limits,
            }
        )
        previous = q

    solved_count = sum(frame["status"] == "solved" for frame in frame_results)
    peak_residual = max(frame["max_residual_m"] for frame in frame_results)
    return {
        "schema": "ohmc.ik_result/v0.1",
        "problem_sha256": object_sha256(problem),
        "status": "pass" if solved_count == len(frame_results) else "fail",
        "variables": [variable["name"] for variable in problem["variables"]],
        "frames": frame_results,
        "summary": {
            "frame_count": len(frame_results),
            "solved_frame_count": solved_count,
            "failed_frame_count": len(frame_results) - solved_count,
            "peak_residual_m": peak_residual,
        },
        "hardware_commands_sent": False,
    }


def ik_result_to_motion_ir(
    problem: dict[str, Any],
    result: dict[str, Any],
    canonical_motion: dict[str, Any],
    profile: dict[str, Any],
) -> dict[str, Any]:
    """Convert an entirely solved IK result into robot-joint Motion IR."""
    if result["status"] != "pass":
        raise OhmcError("refusing to compile a failed IK result into Motion IR")
    task_sources = {task["source_joint"] for task in problem["tasks"]}
    source_joints = {joint["name"] for joint in canonical_motion["skeleton"]["joints"]}
    unmapped_source = sorted(source_joints - task_sources)
    warnings = ["hardware transport remains disabled by the selected robot profile"]
    if unmapped_source:
        warnings.insert(
            0,
            "IK task coverage is partial; unmapped canonical joints: "
            + ", ".join(unmapped_source),
        )
    return {
        "schema": "ohmc.motion_ir/v0.1",
        "source": canonical_motion["source"],
        "robot": {
            "profile": profile["id"],
            "model_sha256": profile["model_evidence"]["model_sha256"],
            "vendor": profile["vendor"],
            "model": profile["model"],
        },
        "frames": {
            "convention": "right_handed_x_forward_y_left_z_up",
            "world": "world",
            "base": "pelvis",
        },
        "trajectory": {
            "rate_hz": problem["source"]["rate_hz"],
            "joints": result["variables"],
            "samples": [
                {
                    "time": frame["time"],
                    "position_targets": frame["positions"],
                    "solver_status": frame["status"],
                    "residual": frame["max_residual_m"],
                }
                for frame in result["frames"]
            ],
        },
        "passes": [
            {
                "name": "constrained_reference_ik",
                "version": "0.1.0",
                "config_sha256": problem["provenance"]["config_sha256"],
                "warnings": warnings,
            }
        ],
        "validation": {"status": "warning", "issues": warnings},
    }
