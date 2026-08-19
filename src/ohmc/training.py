"""Versioned training runs, evidence gates, and simulation-only policy bundles."""

from __future__ import annotations

import hashlib
import json
import math
import os
import platform
import re
import shutil
import sqlite3
import subprocess
import sys
import tempfile
import uuid
from collections.abc import Callable
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from xml.etree import ElementTree

import yaml
from jsonschema import Draft202012Validator

from .errors import OhmcError
from .profiles import load_yaml_object, validate_robot_profile

RUN_STATES = (
    "created",
    "preflight",
    "training",
    "evaluating",
    "sim2sim",
    "awaiting_hardware_review",
    "hardware_candidate",
    "failed",
    "cancelled",
)

RUN_TRANSITIONS: dict[str, set[str]] = {
    "created": {"preflight", "failed", "cancelled"},
    "preflight": {"training", "failed", "cancelled"},
    "training": {"evaluating", "failed", "cancelled"},
    "evaluating": {"sim2sim", "failed", "cancelled"},
    "sim2sim": {"awaiting_hardware_review", "failed", "cancelled"},
    "awaiting_hardware_review": {"hardware_candidate", "failed", "cancelled"},
    "hardware_candidate": {"failed", "cancelled"},
    "failed": set(),
    "cancelled": set(),
}

EXECUTION_STATUSES = {
    "idle",
    "ready",
    "queued",
    "running",
    "paused",
    "blocked",
    "complete",
    "failed",
    "cancelled",
}

_ENV_TOKEN = re.compile(r"^\$\{([A-Z][A-Z0-9_]*)\}$")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except FileNotFoundError as exc:
        raise OhmcError(f"file not found: {path}") from exc
    return digest.hexdigest()


def stable_sha256(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def load_json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise OhmcError(f"file not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise OhmcError(f"invalid JSON in {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise OhmcError(f"expected a JSON object in {path}")
    return value


def schema_issues(document: dict[str, Any], schema: dict[str, Any]) -> list[str]:
    issues: list[str] = []
    for error in sorted(
        Draft202012Validator(schema).iter_errors(document),
        key=lambda item: list(item.path),
    ):
        location = ".".join(str(part) for part in error.absolute_path) or "$"
        issues.append(f"{location}: {error.message}")
    return issues


def load_training_recipe(
    path: Path, schema_path: Path
) -> tuple[dict[str, Any], list[str]]:
    recipe = load_yaml_object(path)
    schema = load_json_object(schema_path)
    issues = schema_issues(recipe, schema)
    if not issues and recipe["backend"]["id"] == "fixture":
        if recipe["backend"]["runtime"] != "fixture":
            issues.append("backend.runtime must be fixture for the fixture backend")
    elif not issues:
        if recipe["backend"]["runtime"] != "external":
            issues.append("backend.runtime must be external for Isaac Lab")
        if not recipe["backend"]["command"]:
            issues.append("backend.command must define the external Isaac Lab launcher")
    if not issues:
        required_actor = {
            "base_angular_velocity",
            "projected_gravity",
            "velocity_command",
            "joint_position_error_29",
            "joint_velocity_29",
            "previous_action_29",
            "rgb_latent_32",
            "depth_latent_32",
            "image_age",
        }
        missing = sorted(required_actor - set(recipe["observation"]["actor"]))
        if missing:
            issues.append("observation.actor is missing: " + ", ".join(missing))
        required_privileged = {
            "base_linear_velocity",
            "contact_forces",
            "terrain_height",
        }
        missing = sorted(
            required_privileged - set(recipe["observation"]["critic_privileged"])
        )
        if missing:
            issues.append(
                "observation.critic_privileged is missing: " + ", ".join(missing)
            )
        terrain = [stage["terrain"] for stage in recipe["curriculum"]]
        expected = ["stand", "flat", "slope", "uneven", "low_obstacle", "stairs"]
        if terrain != expected:
            issues.append("curriculum terrain order must be: " + ", ".join(expected))
        gate_ids = [gate["id"] for gate in recipe["acceptance"]["gates"]]
        if len(set(gate_ids)) != len(gate_ids):
            issues.append("acceptance gate ids must be unique")
        levels = {gate["level"] for gate in recipe["acceptance"]["gates"]}
        for required in ("controller_sim", "sim2sim", "runtime_fault"):
            if required not in levels:
                issues.append(f"acceptance gates must include level {required}")
    return recipe, issues


def _atomic_write_json(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}-", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(document, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        temporary.replace(path)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise


def _run_id() -> str:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"{stamp}-{uuid.uuid4().hex[:8]}"


class TrainingStore:
    """SQLite index with an atomic, human-readable manifest for every run."""

    def __init__(
        self,
        root: Path,
        *,
        run_schema_path: Path,
    ) -> None:
        self.root = root.expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.database = self.root / "runs.sqlite3"
        self.run_schema_path = run_schema_path.expanduser().resolve()
        self.run_schema = load_json_object(self.run_schema_path)
        self._initialize()

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.database, timeout=30)
        connection.execute("PRAGMA journal_mode=WAL")
        connection.execute("PRAGMA foreign_keys=ON")
        return connection

    def _initialize(self) -> None:
        with self._connect() as connection:
            connection.execute(
                """
                CREATE TABLE IF NOT EXISTS runs (
                    run_id TEXT PRIMARY KEY,
                    state TEXT NOT NULL,
                    execution_status TEXT NOT NULL,
                    manifest_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )
                """
            )
            connection.execute(
                """
                CREATE TABLE IF NOT EXISTS events (
                    sequence INTEGER PRIMARY KEY AUTOINCREMENT,
                    run_id TEXT NOT NULL,
                    at TEXT NOT NULL,
                    kind TEXT NOT NULL,
                    message TEXT NOT NULL,
                    FOREIGN KEY(run_id) REFERENCES runs(run_id)
                )
                """
            )

    def run_dir(self, run_id: str) -> Path:
        if not re.fullmatch(r"[0-9]{8}T[0-9]{6}Z-[0-9a-f]{8}", run_id):
            raise OhmcError(f"invalid run id: {run_id}")
        return self.root / run_id

    def create(self, recipe: dict[str, Any], recipe_path: Path) -> dict[str, Any]:
        run_id = _run_id()
        run_dir = self.run_dir(run_id)
        run_dir.mkdir(parents=False)
        recipe_copy = run_dir / "recipe.yaml"
        recipe_copy.write_text(
            yaml.safe_dump(recipe, sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )
        now = utc_now()
        manifest: dict[str, Any] = {
            "schema": "ohmc.run_manifest/v0.1",
            "run_id": run_id,
            "recipe": {
                "id": recipe["id"],
                "sha256": sha256_file(recipe_copy),
                "path": "recipe.yaml",
            },
            "state": "created",
            "execution": {"status": "idle", "pid": None, "last_error": None},
            "created_at": now,
            "updated_at": now,
            "history": [
                {
                    "state": "created",
                    "at": now,
                    "message": f"run created from {recipe_path.expanduser().resolve()}",
                }
            ],
            "artifacts": {},
            "authority": {
                "hardware_transport": False,
                "automatic_hardware_promotion": False,
            },
        }
        self._validate_manifest(manifest)
        _atomic_write_json(run_dir / "manifest.json", manifest)
        with self._connect() as connection:
            connection.execute(
                "INSERT INTO runs VALUES (?, ?, ?, ?, ?, ?)",
                (
                    run_id,
                    manifest["state"],
                    manifest["execution"]["status"],
                    json.dumps(manifest, sort_keys=True),
                    now,
                    now,
                ),
            )
            connection.execute(
                "INSERT INTO events(run_id, at, kind, message) VALUES (?, ?, ?, ?)",
                (run_id, now, "state", "run created"),
            )
        return manifest

    def _validate_manifest(self, manifest: dict[str, Any]) -> None:
        issues = schema_issues(manifest, self.run_schema)
        if issues:
            raise OhmcError("invalid run manifest: " + "; ".join(issues))

    def get(self, run_id: str) -> dict[str, Any]:
        with self._connect() as connection:
            row = connection.execute(
                "SELECT manifest_json FROM runs WHERE run_id = ?", (run_id,)
            ).fetchone()
        if row is None:
            raise OhmcError(f"unknown run: {run_id}")
        return json.loads(row[0])

    def list(self, limit: int = 100) -> list[dict[str, Any]]:
        with self._connect() as connection:
            rows = connection.execute(
                "SELECT manifest_json FROM runs ORDER BY created_at DESC LIMIT ?",
                (limit,),
            ).fetchall()
        return [json.loads(row[0]) for row in rows]

    def recover_orphaned_runs(self) -> list[str]:
        """Mark interrupted training processes as recoverable without rerunning them."""

        recovered: list[str] = []
        for manifest in self.list(limit=10_000):
            execution = manifest["execution"]
            if manifest["state"] != "training" or execution["status"] not in {
                "queued",
                "running",
                "paused",
            }:
                continue
            pid = execution["pid"]
            if pid is not None and _process_matches_run(
                pid, self.run_dir(manifest["run_id"])
            ):
                continue
            self.set_execution(
                manifest["run_id"],
                "blocked",
                error="training process disappeared before completion",
                message="interrupted training detected; resume from the recorded curriculum checkpoint",
            )
            recovered.append(manifest["run_id"])
        return recovered

    def events(self, run_id: str, after: int = 0) -> list[dict[str, Any]]:
        self.get(run_id)
        with self._connect() as connection:
            rows = connection.execute(
                """
                SELECT sequence, at, kind, message FROM events
                WHERE run_id = ? AND sequence > ? ORDER BY sequence
                """,
                (run_id, after),
            ).fetchall()
        return [
            {"sequence": row[0], "at": row[1], "kind": row[2], "message": row[3]}
            for row in rows
        ]

    def _save(self, manifest: dict[str, Any], kind: str, message: str) -> None:
        manifest["updated_at"] = utc_now()
        self._validate_manifest(manifest)
        run_id = manifest["run_id"]
        _atomic_write_json(self.run_dir(run_id) / "manifest.json", manifest)
        with self._connect() as connection:
            cursor = connection.execute(
                """
                UPDATE runs SET state = ?, execution_status = ?, manifest_json = ?,
                    updated_at = ? WHERE run_id = ?
                """,
                (
                    manifest["state"],
                    manifest["execution"]["status"],
                    json.dumps(manifest, sort_keys=True),
                    manifest["updated_at"],
                    run_id,
                ),
            )
            if cursor.rowcount != 1:
                raise OhmcError(f"unknown run: {run_id}")
            connection.execute(
                "INSERT INTO events(run_id, at, kind, message) VALUES (?, ?, ?, ?)",
                (run_id, manifest["updated_at"], kind, message),
            )

    def transition(self, run_id: str, target: str, message: str) -> dict[str, Any]:
        manifest = self.get(run_id)
        current = manifest["state"]
        if target not in RUN_TRANSITIONS[current]:
            raise OhmcError(f"invalid run transition: {current} -> {target}")
        now = utc_now()
        manifest["state"] = target
        manifest["history"].append({"state": target, "at": now, "message": message})
        manifest["updated_at"] = now
        self._save(manifest, "state", f"{current} -> {target}: {message}")
        return manifest

    def set_execution(
        self,
        run_id: str,
        status: str,
        *,
        pid: int | None = None,
        error: str | None = None,
        message: str | None = None,
    ) -> dict[str, Any]:
        if status not in EXECUTION_STATUSES:
            raise OhmcError(f"unknown execution status: {status}")
        manifest = self.get(run_id)
        manifest["execution"] = {
            "status": status,
            "pid": pid,
            "last_error": error,
        }
        self._save(manifest, "execution", message or f"execution status: {status}")
        return manifest

    def add_artifact(self, run_id: str, role: str, path: Path) -> dict[str, Any]:
        manifest = self.get(run_id)
        run_dir = self.run_dir(run_id)
        resolved = path.expanduser().resolve()
        try:
            relative = resolved.relative_to(run_dir)
        except ValueError as exc:
            raise OhmcError(
                f"run artifact must stay inside {run_dir}: {resolved}"
            ) from exc
        manifest["artifacts"][role] = {
            "path": relative.as_posix(),
            "sha256": sha256_file(resolved),
        }
        self._save(manifest, "artifact", f"registered {role}: {relative.as_posix()}")
        return manifest


def _check(
    checks: list[dict[str, Any]],
    name: str,
    status: str,
    detail: str,
    fix: str | None = None,
) -> None:
    item: dict[str, Any] = {"name": name, "status": status, "detail": detail}
    if fix:
        item["fix"] = fix
    checks.append(item)


def _command_output(command: list[str]) -> tuple[int, str]:
    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 1, str(exc)
    output = (result.stdout or result.stderr).strip()
    return result.returncode, output


def _pid_is_running(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _process_matches_run(pid: int, run_dir: Path) -> bool:
    """Avoid mistaking a recycled PID for the run's former training process."""

    if not _pid_is_running(pid):
        return False
    proc_cmdline = Path(f"/proc/{pid}/cmdline")
    try:
        command = (
            proc_cmdline.read_bytes()
            .replace(b"\0", b" ")
            .decode("utf-8", errors="replace")
        )
    except OSError:
        code, command = _command_output(["ps", "-p", str(pid), "-o", "command="])
        if code != 0:
            return False
    return str(run_dir) in command


def _nvidia_memory_gib() -> float | None:
    executable = shutil.which("nvidia-smi")
    if executable is None:
        return None
    code, output = _command_output(
        [executable, "--query-gpu=memory.total", "--format=csv,noheader,nounits"]
    )
    if code != 0:
        return None
    try:
        return float(output.splitlines()[0]) / 1024.0
    except (ValueError, IndexError):
        return None


def expand_backend_command(
    recipe: dict[str, Any], *, recipe_path: Path, run_dir: Path
) -> list[str]:
    expanded: list[str] = []
    for token in recipe["backend"]["command"]:
        match = _ENV_TOKEN.fullmatch(token)
        if match:
            name = match.group(1)
            value = os.environ.get(name)
            if not value:
                raise OhmcError(f"required environment variable is not set: {name}")
            expanded.append(value)
            continue
        expanded.append(
            token.replace("{recipe}", str(recipe_path)).replace(
                "{run_dir}", str(run_dir)
            )
        )
    return expanded


def training_doctor_report(
    recipe: dict[str, Any] | None = None,
    *,
    recipe_path: Path | None = None,
    profile_path: Path | None = None,
    profile_schema_path: Path | None = None,
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    fixture_mode = recipe is not None and recipe["backend"]["id"] == "fixture"
    python_ok = sys.version_info >= (3, 10)
    _check(
        checks,
        "python",
        "pass" if python_ok else "fail",
        platform.python_version(),
        None if python_ok else "Install Python 3.10 or newer.",
    )
    release = platform.release().lower()
    proc_version = ""
    try:
        proc_version = Path("/proc/version").read_text(encoding="utf-8").lower()
    except (FileNotFoundError, OSError):
        pass
    is_wsl = "microsoft" in release or "microsoft" in proc_version
    _check(
        checks,
        "wsl2",
        "pass" if is_wsl else "warning",
        "WSL detected" if is_wsl else f"current host is {platform.system()}",
        None if is_wsl else "Run production training inside the supported WSL2 host.",
    )
    memory_gib = _nvidia_memory_gib()
    _check(
        checks,
        "nvidia_gpu",
        "pass" if memory_gib is not None else "warning" if fixture_mode else "fail",
        f"{memory_gib:.1f} GiB detected"
        if memory_gib is not None
        else "nvidia-smi unavailable",
        None
        if memory_gib is not None
        else "Install the Windows NVIDIA driver and expose the GPU to WSL2; do not install a Linux display driver inside WSL.",
    )
    docker = shutil.which("docker")
    _check(
        checks,
        "docker",
        "pass" if docker else "warning",
        docker or "docker executable unavailable",
        None
        if docker
        else "Install Docker Engine and NVIDIA Container Toolkit if using the container route.",
    )

    recommended_env_count: int | None = None
    if recipe is not None:
        if memory_gib is not None and memory_gib >= float(
            recipe["compute"]["high_memory_min_gib"]
        ):
            recommended_env_count = int(recipe["compute"]["high_memory_env_count"])
        else:
            recommended_env_count = int(recipe["compute"]["default_env_count"])
        backend = recipe["backend"]
        if backend["id"] == "fixture":
            _check(checks, "training_backend", "pass", "deterministic test fixture")
        else:
            try:
                command = expand_backend_command(
                    recipe,
                    recipe_path=recipe_path or Path("recipe.yaml"),
                    run_dir=Path("run"),
                )
                executable = command[0]
                exists = (
                    Path(executable).is_file() or shutil.which(executable) is not None
                )
                _check(
                    checks,
                    "training_backend",
                    "pass" if exists else "fail",
                    executable,
                    None
                    if exists
                    else "Set OHMC_ISAACLAB_PYTHON to the Isaac Lab launcher or Python executable.",
                )
                if exists:
                    code, runtime_output = _command_output(
                        [
                            executable,
                            "-c",
                            (
                                "import sys; from importlib.metadata import version; "
                                "import ohmc_x2; "
                                "print(f'OHMC_RUNTIME={sys.version_info.major}."
                                '{sys.version_info.minor}|{version("rsl-rl-lib")}\')'
                            ),
                        ]
                    )
                    runtime_match = re.search(
                        r"OHMC_RUNTIME=([0-9]+\.[0-9]+)\|([^\s]+)",
                        runtime_output,
                    )
                    expected_rsl = str(backend["versions"].get("rsl_rl", ""))
                    runtime_ok = (
                        code == 0
                        and runtime_match is not None
                        and runtime_match.group(1) == "3.12"
                        and runtime_match.group(2) == expected_rsl
                    )
                    _check(
                        checks,
                        "isaac_python_runtime",
                        "pass" if runtime_ok else "fail",
                        runtime_match.group(0) if runtime_match else runtime_output,
                        None
                        if runtime_ok
                        else (
                            "Install integrations/isaaclab into the pinned Isaac Lab "
                            f"Python 3.12 runtime with rsl-rl-lib {expected_rsl}."
                        ),
                    )
            except OhmcError as exc:
                _check(
                    checks,
                    "training_backend",
                    "fail",
                    str(exc),
                    "Set OHMC_ISAACLAB_PYTHON to the verified Isaac Lab runtime.",
                )
            isaac_root_value = os.environ.get("OHMC_ISAACLAB_ROOT")
            expected_revision = backend["versions"].get("isaac_lab_revision")
            if not isaac_root_value:
                _check(
                    checks,
                    "isaac_lab_revision",
                    "fail",
                    "OHMC_ISAACLAB_ROOT is not set",
                    "Set OHMC_ISAACLAB_ROOT to the pinned Isaac Lab checkout.",
                )
            else:
                isaac_root = Path(isaac_root_value).expanduser().resolve()
                code, revision = _command_output(
                    ["git", "-C", str(isaac_root), "rev-parse", "HEAD"]
                )
                revision_ok = code == 0 and (
                    expected_revision is None or revision == expected_revision
                )
                _check(
                    checks,
                    "isaac_lab_revision",
                    "pass" if revision_ok else "fail",
                    revision
                    if code == 0
                    else f"cannot inspect {isaac_root}: {revision}",
                    None
                    if revision_ok
                    else f"Checkout the recipe revision {expected_revision}.",
                )

    if profile_path is not None and profile_schema_path is not None:
        try:
            profile = load_yaml_object(profile_path)
            issues = validate_robot_profile(
                profile, load_json_object(profile_schema_path)
            )
            if len(profile["control"]["joint_order"]) != 29:
                issues.append("locomotion profile must contain exactly 29 joints")
            excluded = {item["name"] for item in profile["control"]["excluded_joints"]}
            if not {"head_yaw_joint", "head_pitch_joint"}.issubset(excluded):
                issues.append("locomotion profile must exclude both head joints")
            _check(
                checks,
                "robot_profile",
                "pass" if not issues else "fail",
                profile["id"] if not issues else "; ".join(issues),
            )
            if recipe is not None and not fixture_mode:
                urdf_value = os.environ.get("OHMC_X2_URDF")
                if not urdf_value:
                    _check(
                        checks,
                        "x2_urdf",
                        "fail",
                        "OHMC_X2_URDF is not set",
                        "Point OHMC_X2_URDF at the verified official x2_ultra.urdf.",
                    )
                else:
                    urdf_path = Path(urdf_value).expanduser().resolve()
                    if not urdf_path.is_file():
                        _check(
                            checks,
                            "x2_urdf",
                            "fail",
                            f"file not found: {urdf_path}",
                            "Sync the pinned AgiBot X2 URDF and update OHMC_X2_URDF.",
                        )
                    else:
                        actual_hash = sha256_file(urdf_path)
                        expected_hash = profile["model_evidence"]["model_sha256"]
                        _check(
                            checks,
                            "x2_urdf",
                            "pass" if actual_hash == expected_hash else "fail",
                            f"{urdf_path} sha256={actual_hash}",
                            None
                            if actual_hash == expected_hash
                            else "Use the exact URDF revision recorded by the locomotion profile.",
                        )
                        try:
                            root = ElementTree.parse(urdf_path).getroot()
                            missing_meshes = []
                            for mesh in root.iter("mesh"):
                                filename = mesh.attrib.get("filename")
                                if not filename:
                                    continue
                                if filename.startswith(
                                    ("package://", "http://", "https://")
                                ):
                                    missing_meshes.append(filename)
                                    continue
                                mesh_path = (urdf_path.parent / filename).resolve()
                                if not mesh_path.is_file():
                                    missing_meshes.append(filename)
                            _check(
                                checks,
                                "x2_model_resources",
                                "pass" if not missing_meshes else "fail",
                                (
                                    "all referenced meshes are present"
                                    if not missing_meshes
                                    else f"missing {len(missing_meshes)} mesh references"
                                ),
                                None
                                if not missing_meshes
                                else "Use the complete pinned X2 URDF repository, not a standalone URDF file.",
                            )
                        except ElementTree.ParseError as exc:
                            _check(
                                checks,
                                "x2_model_resources",
                                "fail",
                                f"invalid URDF XML: {exc}",
                                "Re-sync the pinned official X2 model repository.",
                            )
        except OhmcError as exc:
            _check(checks, "robot_profile", "fail", str(exc))

    critical = [item for item in checks if item["status"] == "fail"]
    return {
        "schema": "ohmc.training_doctor/v0.1",
        "generated_at": utc_now(),
        "ready": not critical,
        "recommended_env_count": recommended_env_count,
        "checks": checks,
    }


def prepare_run(
    store: TrainingStore,
    recipe: dict[str, Any],
    recipe_path: Path,
    *,
    profile_path: Path,
    profile_schema_path: Path,
) -> tuple[dict[str, Any], dict[str, Any]]:
    manifest = store.create(recipe, recipe_path)
    run_id = manifest["run_id"]
    manifest = store.transition(run_id, "preflight", "environment preflight started")
    report = training_doctor_report(
        recipe,
        recipe_path=store.run_dir(run_id) / "recipe.yaml",
        profile_path=profile_path,
        profile_schema_path=profile_schema_path,
    )
    report_path = store.run_dir(run_id) / "doctor.json"
    _atomic_write_json(report_path, report)
    store.add_artifact(run_id, "doctor", report_path)
    if report["ready"]:
        manifest = store.set_execution(
            run_id, "ready", message="preflight passed; training may start"
        )
    else:
        manifest = store.set_execution(
            run_id,
            "blocked",
            error="environment preflight failed",
            message="preflight blocked; inspect doctor.json",
        )
    return manifest, report


def _fixture_training(store: TrainingStore, run_id: str) -> int:
    run_dir = store.run_dir(run_id)
    (run_dir / "checkpoint.fixture").write_text(
        "OHMC deterministic training fixture; not a learned controller.\n",
        encoding="utf-8",
    )
    (run_dir / "policy.fixture.onnx").write_bytes(b"OHMC-FIXTURE-NOT-ONNX\n")
    store.add_artifact(run_id, "checkpoint_fixture", run_dir / "checkpoint.fixture")
    store.add_artifact(run_id, "policy_fixture", run_dir / "policy.fixture.onnx")
    return 0


def execute_run(
    store: TrainingStore,
    run_id: str,
    *,
    process_started: Callable[[subprocess.Popen[Any]], None] | None = None,
    resume: bool = False,
) -> int:
    manifest = store.get(run_id)
    if resume:
        if (
            manifest["state"] != "training"
            or manifest["execution"]["status"] != "blocked"
        ):
            raise OhmcError(f"run {run_id} is not an interrupted recoverable run")
    else:
        if manifest["state"] != "preflight":
            raise OhmcError(f"run {run_id} is not at preflight")
        if manifest["execution"]["status"] not in {"ready", "queued"}:
            raise OhmcError(f"run {run_id} preflight is not ready")
    run_dir = store.run_dir(run_id)
    recipe_path = run_dir / "recipe.yaml"
    recipe = load_yaml_object(recipe_path)
    if not resume:
        store.transition(run_id, "training", "training backend started")
    store.set_execution(
        run_id,
        "running",
        message="training process resumed" if resume else "training process running",
    )
    log_path = run_dir / "training.log"
    if recipe["backend"]["id"] == "fixture":
        code = _fixture_training(store, run_id)
        log_path.write_text(
            "Fixture backend completed. This is not reinforcement learning.\n",
            encoding="utf-8",
        )
    else:
        command = expand_backend_command(
            recipe, recipe_path=recipe_path, run_dir=run_dir
        )
        try:
            doctor_path = run_dir / "doctor.json"
            doctor = load_json_object(doctor_path)
            env_count = doctor.get("recommended_env_count")
            with log_path.open("ab") as log:
                process = subprocess.Popen(
                    command,
                    cwd=run_dir,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    env={
                        **os.environ,
                        "OHMC_RUN_ID": run_id,
                        "OHMC_RUN_DIR": str(run_dir),
                        "OHMC_RECIPE": str(recipe_path),
                        "OHMC_ENV_COUNT": str(
                            env_count or recipe["compute"]["default_env_count"]
                        ),
                    },
                )
                store.set_execution(
                    run_id,
                    "running",
                    pid=process.pid,
                    message=f"training process pid={process.pid}",
                )
                if process_started:
                    process_started(process)
                code = process.wait()
        except OSError as exc:
            code = 127
            with log_path.open("a", encoding="utf-8") as log:
                log.write(f"launcher error: {exc}\n")
    store.add_artifact(run_id, "training_log", log_path)
    if store.get(run_id)["state"] == "cancelled":
        return 130
    if code == 0:
        for role, name in (
            ("checkpoint", "checkpoint.pt"),
            ("policy_onnx", "policy.onnx"),
            ("action_preview", "preview.mp4"),
            ("curriculum_progress", "curriculum-progress.json"),
            ("backend_result", "backend-result.json"),
        ):
            artifact_path = run_dir / name
            if artifact_path.is_file():
                store.add_artifact(run_id, role, artifact_path)
        store.set_execution(run_id, "complete", message="training backend completed")
        store.transition(
            run_id, "evaluating", "training completed; evaluation required"
        )
        return 0
    error = f"training backend exited with status {code}"
    store.set_execution(run_id, "failed", error=error, message=error)
    store.transition(run_id, "failed", error)
    return code


def _metric(metrics: dict[str, Any], dotted: str) -> float | None:
    value: Any = metrics
    for component in dotted.split("."):
        if not isinstance(value, dict) or component not in value:
            return None
        value = value[component]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    numeric = float(value)
    return numeric if math.isfinite(numeric) else None


def _gate_passes(actual: float | None, operator: str, expected: float) -> bool:
    if actual is None:
        return False
    if operator == "ge":
        return actual >= expected
    if operator == "le":
        return actual <= expected
    if operator == "eq":
        return actual == expected
    raise OhmcError(f"unknown gate operator: {operator}")


def evaluate_run(
    store: TrainingStore,
    run_id: str,
    metrics_path: Path,
    *,
    metrics_schema_path: Path,
    evidence_schema_path: Path,
) -> dict[str, Any]:
    manifest = store.get(run_id)
    if manifest["state"] != "evaluating":
        raise OhmcError(f"run {run_id} is not awaiting evaluation")
    recipe = load_yaml_object(store.run_dir(run_id) / "recipe.yaml")
    metrics = load_json_object(metrics_path)
    issues = schema_issues(metrics, load_json_object(metrics_schema_path))
    if issues:
        raise OhmcError("invalid evaluation metrics: " + "; ".join(issues))
    if metrics["episodes"] < recipe["acceptance"]["minimum_episodes"]:
        raise OhmcError(
            f"evaluation has {metrics['episodes']} episodes; "
            f"at least {recipe['acceptance']['minimum_episodes']} are required"
        )
    destination_dir = store.run_dir(run_id) / "evidence"
    destination_dir.mkdir(exist_ok=True)
    metrics_destination = destination_dir / "metrics.json"
    if metrics_path.expanduser().resolve() != metrics_destination.resolve():
        shutil.copy2(metrics_path, metrics_destination)
    gates: list[dict[str, Any]] = []
    for definition in recipe["acceptance"]["gates"]:
        actual = _metric(metrics["metrics"], definition["metric"])
        passed = _gate_passes(
            actual, definition["operator"], float(definition["value"])
        )
        gates.append(
            {
                "id": definition["id"],
                "level": definition["level"],
                "metric": definition["metric"],
                "operator": definition["operator"],
                "expected": float(definition["value"]),
                "actual": actual,
                "status": "pass" if passed else "fail",
            }
        )
    controller_pass = all(
        gate["status"] == "pass"
        for gate in gates
        if gate["level"] in {"controller_sim", "runtime_fault"}
    )
    sim2sim_pass = all(
        gate["status"] == "pass" for gate in gates if gate["level"] == "sim2sim"
    )
    overall = controller_pass and sim2sim_pass
    evidence = {
        "schema": "ohmc.evidence_bundle/v0.1",
        "run_id": run_id,
        "created_at": utc_now(),
        "status": "pass" if overall else "fail",
        "metrics": {
            "path": "metrics.json",
            "sha256": sha256_file(metrics_destination),
            "episodes": metrics["episodes"],
        },
        "gates": gates,
        "levels": {
            "controller_sim": "pass" if controller_pass else "fail",
            "sim2sim": (
                "pass"
                if sim2sim_pass and controller_pass
                else "not_reached"
                if not controller_pass
                else "fail"
            ),
            "hardware": "not_tested",
        },
        "authority": {
            "label": "simulation_passed" if overall else "simulation_failed",
            "hardware_transport": False,
            "operator_review_required": True,
        },
    }
    issues = schema_issues(evidence, load_json_object(evidence_schema_path))
    if issues:
        raise OhmcError("generated invalid evidence bundle: " + "; ".join(issues))
    evidence_path = destination_dir / "evidence.json"
    _atomic_write_json(evidence_path, evidence)
    store.add_artifact(run_id, "evaluation_metrics", metrics_destination)
    store.add_artifact(run_id, "evidence", evidence_path)
    if not overall:
        store.transition(run_id, "failed", "one or more evaluation gates failed")
        return evidence
    store.transition(run_id, "sim2sim", "controller simulation gates passed")
    store.transition(
        run_id,
        "awaiting_hardware_review",
        "independent Sim2Sim gates passed; operator review is still required",
    )
    return evidence


def prepare_policy_bundle(
    store: TrainingStore,
    run_id: str,
    *,
    policy_path: Path,
    output_dir: Path,
    profile_path: Path,
    profile_schema_path: Path,
    policy_schema_path: Path,
    extra_artifacts: list[tuple[str, Path]] | None = None,
) -> dict[str, Any]:
    manifest = store.get(run_id)
    if manifest["state"] != "awaiting_hardware_review":
        raise OhmcError("policy preparation requires a run at awaiting_hardware_review")
    output = output_dir.expanduser().resolve()
    if output.exists():
        raise OhmcError(f"refusing to overwrite existing output: {output}")
    policy = policy_path.expanduser().resolve()
    if not policy.is_file():
        raise OhmcError(f"policy file not found: {policy}")
    profile = load_yaml_object(profile_path)
    profile_issues = validate_robot_profile(
        profile, load_json_object(profile_schema_path)
    )
    if profile_issues:
        raise OhmcError("invalid locomotion profile: " + "; ".join(profile_issues))
    joint_order = profile["control"]["joint_order"]
    if len(joint_order) != 29:
        raise OhmcError("policy bundle requires exactly 29 controlled joints")
    recipe_path = store.run_dir(run_id) / "recipe.yaml"
    recipe = load_yaml_object(recipe_path)
    evidence_source = store.run_dir(run_id) / "evidence" / "evidence.json"
    evidence = load_json_object(evidence_source)
    if evidence.get("status") != "pass":
        raise OhmcError("policy bundle requires passing E3 Sim2Sim evidence")

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=f".{output.name}-", dir=output.parent))
    try:
        artifacts: list[dict[str, Any]] = []

        def copy_artifact(
            role: str, source: Path, destination_name: str | None = None
        ) -> None:
            destination = temporary / (destination_name or source.name)
            shutil.copy2(source, destination)
            artifacts.append(
                {
                    "role": role,
                    "path": destination.name,
                    "sha256": sha256_file(destination),
                }
            )

        copy_artifact("policy_onnx", policy, "policy.onnx")
        copy_artifact("evidence", evidence_source, "evidence.json")
        for role, source in extra_artifacts or []:
            if role not in {
                "checkpoint",
                "normalization",
                "camera_calibration",
                "test_vectors",
            }:
                raise OhmcError(f"unsupported policy artifact role: {role}")
            copy_artifact(role, source.expanduser().resolve())

        bundle = {
            "schema": "ohmc.policy_bundle/v0.1",
            "bundle_id": f"{run_id}-x2-rgbd",
            "created_at": utc_now(),
            "source_run_id": run_id,
            "robot_profile": profile["id"],
            "training_recipe_sha256": sha256_file(recipe_path),
            "initialization": recipe["initialization"],
            "joint_order": joint_order,
            "observation": recipe["observation"],
            "action": {
                "type": recipe["control"]["action"],
                "dimensions": recipe["control"]["action_dimensions"],
                "scale": recipe["control"]["action_scale"],
            },
            "timing": {
                "policy_rate_hz": recipe["control"]["policy_rate_hz"],
                "command_rate_hz": recipe["control"]["command_rate_hz"],
                "interpolation": recipe["control"]["interpolation"],
            },
            "perception": recipe["perception"],
            "artifacts": artifacts,
            "evidence": {"level": "E3_sim2sim", "status": "pass"},
            "authority": {
                "hardware_execution": False,
                "automatic_promotion": False,
                "operator_review_required": True,
            },
        }
        issues = schema_issues(bundle, load_json_object(policy_schema_path))
        if issues:
            raise OhmcError("generated invalid policy bundle: " + "; ".join(issues))
        _atomic_write_json(temporary / "manifest.json", bundle)
        temporary.replace(output)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return bundle


def verify_policy_bundle(
    directory: Path, *, policy_schema_path: Path
) -> dict[str, Any]:
    root = directory.expanduser().resolve()
    manifest = load_json_object(root / "manifest.json")
    issues = schema_issues(manifest, load_json_object(policy_schema_path))
    for artifact in manifest.get("artifacts", []):
        relative = Path(artifact["path"])
        if relative.is_absolute() or ".." in relative.parts:
            issues.append(f"unsafe artifact path: {artifact['path']}")
            continue
        path = root / relative
        if not path.is_file():
            issues.append(f"missing artifact: {artifact['path']}")
        elif sha256_file(path) != artifact["sha256"]:
            issues.append(f"artifact hash mismatch: {artifact['path']}")
    if manifest.get("authority", {}).get("hardware_execution") is not False:
        issues.append("policy bundle must not grant hardware execution authority")
    return {
        "schema": "ohmc.policy_bundle_verification/v0.1",
        "status": "pass" if not issues else "fail",
        "issues": issues,
        "authority": "simulation-only; operator review required",
    }
