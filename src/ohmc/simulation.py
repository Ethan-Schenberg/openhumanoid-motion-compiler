"""Reproducible one-command offline simulation bundles."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import stat
import tempfile
from typing import Any
import zipfile

from jsonschema import Draft202012Validator

from .adapters import encode_vendor_fixture
from .bvh import bvh_to_motion_ir, load_bvh
from .canonical import bvh_to_canonical_motion, validate_canonical_motion
from .errors import OhmcError
from .ir import load_json, validate_motion_ir
from .ik import (
    build_ik_problem,
    ik_result_to_motion_ir,
    solve_ik_problem,
    validate_ik_problem,
    validate_ik_result,
    validate_ik_task_map,
)
from .normalization import normalize_canonical_motion
from .landmarks import landmark_coverage_report, validate_landmark_coverage
from .profiles import (
    load_yaml_object,
    map_motion_ir,
    validate_robot_profile,
    validate_semantic_map,
)
from .quality import (
    derive_motion_kinematics,
    trajectory_quality_report,
    validate_quality_report,
)
from .replay import replay_mujoco
from .vendor import (
    Component,
    artifact_cache_path,
    component_status,
    git_cache_path,
    iter_components,
    load_vendor_lock,
    sha256_file,
)


MAX_ARCHIVE_UNCOMPRESSED_BYTES = 1_000_000_000


def _json_bytes(value: Any) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8")


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(_json_bytes(value))


def _object_sha256(value: Any) -> str:
    return hashlib.sha256(_json_bytes(value)).hexdigest()


def _schema_issues(document: dict[str, Any], schema: dict[str, Any]) -> list[str]:
    issues: list[str] = []
    for error in sorted(
        Draft202012Validator(schema).iter_errors(document),
        key=lambda item: list(item.path),
    ):
        location = ".".join(str(part) for part in error.absolute_path) or "$"
        issues.append(f"{location}: {error.message}")
    return issues


def load_simulation_target(
    registry_path: Path, schema_path: Path, target_name: str
) -> dict[str, Any]:
    registry = load_yaml_object(registry_path)
    issues = _schema_issues(registry, load_json(schema_path))
    if issues:
        raise OhmcError("invalid simulation target registry: " + "; ".join(issues))
    targets = registry["targets"]
    if target_name not in targets:
        available = ", ".join(sorted(targets))
        raise OhmcError(
            f"unknown simulation target {target_name!r}; available targets: {available}"
        )
    return targets[target_name]


def _safe_relative_path(value: str) -> Path:
    path = PurePosixPath(value)
    if path.is_absolute() or not path.parts or any(part == ".." for part in path.parts):
        raise OhmcError(f"unsafe relative path in simulation target: {value!r}")
    return Path(*path.parts)


def _find_component(
    lock: dict[str, Any], vendor_name: str, component_name: str
) -> Component:
    matches = [
        component
        for component in iter_components(lock, vendor_name)
        if component.name == component_name
    ]
    if not matches:
        raise OhmcError(
            f"vendor lock has no component {vendor_name}.{component_name}"
        )
    return matches[0]


def _extract_verified_zip(
    archive: Path, cache_dir: Path, component: Component
) -> Path:
    archive_sha256 = sha256_file(archive)
    destination = (
        cache_dir
        / "extracted"
        / component.vendor
        / component.name
        / archive_sha256
    )
    marker = destination / ".ohmc-extracted-sha256"
    if marker.is_file() and marker.read_text(encoding="utf-8").strip() == archive_sha256:
        return destination
    if destination.exists():
        raise OhmcError(
            f"incomplete extracted vendor cache at {destination}; remove that exact "
            "directory and rerun after inspecting it"
        )

    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(
        tempfile.mkdtemp(prefix=".extract-", dir=str(destination.parent))
    )
    try:
        try:
            with zipfile.ZipFile(archive) as handle:
                total_size = sum(info.file_size for info in handle.infolist())
                if total_size > MAX_ARCHIVE_UNCOMPRESSED_BYTES:
                    raise OhmcError(
                        f"archive expands to {total_size} bytes, above the "
                        f"{MAX_ARCHIVE_UNCOMPRESSED_BYTES}-byte safety limit"
                    )
                for info in handle.infolist():
                    relative = _safe_relative_path(info.filename)
                    file_type = (info.external_attr >> 16) & 0o170000
                    if file_type == stat.S_IFLNK:
                        raise OhmcError(
                            f"archive contains unsupported symbolic link: {info.filename}"
                        )
                    target = temporary / relative
                    if info.is_dir():
                        target.mkdir(parents=True, exist_ok=True)
                        continue
                    target.parent.mkdir(parents=True, exist_ok=True)
                    with handle.open(info) as source, target.open("wb") as output:
                        shutil.copyfileobj(source, output)
        except zipfile.BadZipFile as exc:
            raise OhmcError(f"invalid ZIP artifact {archive}: {exc}") from exc
        marker_path = temporary / marker.name
        marker_path.write_text(archive_sha256 + "\n", encoding="utf-8")
        temporary.replace(destination)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return destination


def resolve_simulation_model(
    model_config: dict[str, Any],
    *,
    project_root: Path,
    lock: dict[str, Any],
    cache_dir: Path,
) -> Path:
    kind = model_config["kind"]
    relative = _safe_relative_path(model_config["path"])
    if kind == "workspace_file":
        model_path = project_root / relative
    else:
        vendor_name = model_config["vendor"]
        component_name = model_config["component"]
        component = _find_component(lock, vendor_name, component_name)
        status = component_status(cache_dir, component)
        if status.state not in {"verified", "system"}:
            raise OhmcError(
                f"simulation dependency {vendor_name}.{component_name} is "
                f"{status.state}: {status.detail}"
            )
        if kind == "git_component":
            model_path = git_cache_path(cache_dir, component) / relative
        elif kind == "zip_component":
            archive = artifact_cache_path(cache_dir, component)
            model_path = _extract_verified_zip(archive, cache_dir, component) / relative
        else:  # pragma: no cover - schema prevents this
            raise OhmcError(f"unsupported simulation model locator: {kind}")

    if not model_path.is_file():
        raise OhmcError(f"simulation model not found: {model_path}")
    actual = sha256_file(model_path)
    expected = model_config["expected_sha256"]
    if actual != expected:
        raise OhmcError(
            f"simulation model SHA-256 mismatch for {model_path}: "
            f"expected {expected}, got {actual}"
        )
    return model_path


def build_simulation_bundle(
    *,
    source_path: Path,
    source_license: str,
    output_dir: Path,
    target_name: str,
    registry_path: Path,
    registry_schema_path: Path,
    vendor_lock_path: Path,
    cache_dir: Path,
    project_root: Path,
    motion_schema_path: Path,
    profile_schema_path: Path,
    mapping_schema_path: Path,
    fixture_schema_path: Path,
    bundle_schema_path: Path,
    canonical_schema_path: Path,
    source_convention: str,
    source_length_unit: str,
    quality_schema_path: Path,
    ik_task_map_schema_path: Path,
    ik_problem_schema_path: Path,
    ik_result_schema_path: Path,
    landmark_schema_path: Path,
) -> dict[str, Any]:
    """Compile, map, replay, and encode an offline evidence bundle atomically."""
    output_dir = output_dir.expanduser().resolve()
    if output_dir.exists():
        raise OhmcError(f"refusing to overwrite existing build directory: {output_dir}")
    source_path = source_path.expanduser().resolve()
    try:
        source_bytes = source_path.read_bytes()
    except FileNotFoundError as exc:
        raise OhmcError(f"file not found: {source_path}") from exc

    target = load_simulation_target(
        registry_path, registry_schema_path, target_name
    )
    lock = load_vendor_lock(vendor_lock_path)
    model_path = resolve_simulation_model(
        target["simulator"]["model"],
        project_root=project_root,
        lock=lock,
        cache_dir=cache_dir.expanduser().resolve(),
    )
    profile_path = project_root / _safe_relative_path(target["robot_profile"])
    mapping_path = project_root / _safe_relative_path(target["semantic_mapping"])
    profile = load_yaml_object(profile_path)
    mapping = load_yaml_object(mapping_path)
    profile_issues = validate_robot_profile(profile, load_json(profile_schema_path))
    if profile_issues:
        raise OhmcError("invalid robot profile: " + "; ".join(profile_issues))
    mapping_issues = validate_semantic_map(mapping, load_json(mapping_schema_path))
    if mapping_issues:
        raise OhmcError("invalid semantic mapping: " + "; ".join(mapping_issues))

    bvh_motion = load_bvh(source_path)
    canonical_source = bvh_to_canonical_motion(
        bvh_motion,
        source_bytes=source_bytes,
        source_name=source_path.name,
        source_license=source_license,
        source_convention=source_convention,
        source_length_unit=source_length_unit,
    )
    canonical_issues = validate_canonical_motion(
        canonical_source, load_json(canonical_schema_path)
    )
    if canonical_issues:
        raise OhmcError(
            "generated invalid canonical motion: " + "; ".join(canonical_issues)
        )
    normalization = target["normalization"]
    canonical_motion = normalize_canonical_motion(
        canonical_source,
        morphology_scale=float(normalization["morphology_scale"]),
        rate_hz=float(normalization["rate_hz"]),
    )
    normalized_issues = validate_canonical_motion(
        canonical_motion, load_json(canonical_schema_path)
    )
    if normalized_issues:
        raise OhmcError(
            "generated invalid normalized canonical motion: "
            + "; ".join(normalized_issues)
        )
    source_motion = derive_motion_kinematics(
        bvh_to_motion_ir(
            bvh_motion,
            source_bytes=source_bytes,
            source_name=source_path.name,
            source_license=source_license,
        )
    )
    motion_schema = load_json(motion_schema_path)
    source_issues = validate_motion_ir(source_motion, motion_schema)
    if source_issues:
        raise OhmcError("generated invalid source Motion IR: " + "; ".join(source_issues))
    ik_problem = None
    ik_result = None
    ik_task_map_path = None
    ik_task_map = None
    if "ik_task_map" in target:
        ik_task_map_path = project_root / _safe_relative_path(target["ik_task_map"])
        ik_task_map = load_yaml_object(ik_task_map_path)
        task_map_issues = validate_ik_task_map(
            ik_task_map, load_json(ik_task_map_schema_path)
        )
        if task_map_issues:
            raise OhmcError("invalid IK task map: " + "; ".join(task_map_issues))
        ik_problem = build_ik_problem(
            canonical_motion, profile, ik_task_map, model_path
        )
        problem_issues = validate_ik_problem(
            ik_problem, load_json(ik_problem_schema_path)
        )
        if problem_issues:
            raise OhmcError(
                "generated invalid IK problem: " + "; ".join(problem_issues)
            )
        ik_result = solve_ik_problem(ik_problem, model_path)
        result_issues = validate_ik_result(
            ik_result, load_json(ik_result_schema_path)
        )
        if result_issues:
            raise OhmcError(
                "generated invalid IK result: " + "; ".join(result_issues)
            )
        if ik_result["status"] != "pass":
            raise OhmcError(
                "IK failed: "
                f"{ik_result['summary']['failed_frame_count']}/"
                f"{ik_result['summary']['frame_count']} frames, "
                f"peak residual {ik_result['summary']['peak_residual_m']:.9g} m"
            )
        mapped_motion = derive_motion_kinematics(
            ik_result_to_motion_ir(
                ik_problem, ik_result, canonical_motion, profile
            )
        )
    else:
        mapped_motion = map_motion_ir(source_motion, profile, mapping)
    landmark_report = landmark_coverage_report(canonical_motion, ik_task_map)
    landmark_issues = validate_landmark_coverage(
        landmark_report, load_json(landmark_schema_path)
    )
    if landmark_issues:
        raise OhmcError(
            "generated invalid landmark report: " + "; ".join(landmark_issues)
        )
    mapped_issues = validate_motion_ir(mapped_motion, motion_schema)
    if mapped_issues:
        raise OhmcError("generated invalid mapped Motion IR: " + "; ".join(mapped_issues))
    quality_report = trajectory_quality_report(mapped_motion, profile)
    quality_issues = validate_quality_report(
        quality_report, load_json(quality_schema_path)
    )
    if quality_issues:
        raise OhmcError(
            "generated invalid trajectory quality report: "
            + "; ".join(quality_issues)
        )
    if quality_report["status"] == "fail":
        raise OhmcError(
            f"trajectory quality failed with {len(quality_report['violations'])} "
            "limit violations"
        )
    replay_report = replay_mujoco(mapped_motion, model_path)
    fixture = encode_vendor_fixture(mapped_motion, profile, target["adapter"])
    fixture_issues = _schema_issues(fixture, load_json(fixture_schema_path))
    if fixture_issues:
        raise OhmcError("generated invalid vendor fixture: " + "; ".join(fixture_issues))

    output_dir.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(
        tempfile.mkdtemp(prefix=f".{output_dir.name}-", dir=str(output_dir.parent))
    )
    try:
        artifacts = {
            "canonical_source": "canonical.source.json",
            "canonical_motion": "canonical-motion.json",
            "source_motion": "motion.source.json",
            "motion": "motion.json",
            "replay_report": "replay-report.json",
            "quality_report": "quality-report.json",
            "landmark_report": "landmark-report.json",
            "interface_fixture": "interface-fixture.json",
            "robot_profile": "configs/robot-profile.yaml",
            "semantic_mapping": "configs/semantic-mapping.yaml",
        }
        if ik_problem is not None and ik_result is not None and ik_task_map_path:
            artifacts.update(
                {
                    "ik_task_map": "configs/ik-task-map.yaml",
                    "ik_problem": "ik-problem.json",
                    "ik_result": "ik-result.json",
                }
            )
        _write_json(temporary / artifacts["canonical_source"], canonical_source)
        _write_json(temporary / artifacts["canonical_motion"], canonical_motion)
        _write_json(temporary / artifacts["source_motion"], source_motion)
        _write_json(temporary / artifacts["motion"], mapped_motion)
        _write_json(temporary / artifacts["replay_report"], replay_report)
        _write_json(temporary / artifacts["quality_report"], quality_report)
        _write_json(temporary / artifacts["landmark_report"], landmark_report)
        _write_json(temporary / artifacts["interface_fixture"], fixture)
        if ik_problem is not None and ik_result is not None and ik_task_map_path:
            _write_json(temporary / artifacts["ik_problem"], ik_problem)
            _write_json(temporary / artifacts["ik_result"], ik_result)
        (temporary / "configs").mkdir(parents=True, exist_ok=True)
        shutil.copy2(profile_path, temporary / artifacts["robot_profile"])
        shutil.copy2(mapping_path, temporary / artifacts["semantic_mapping"])
        if ik_task_map_path is not None:
            shutil.copy2(ik_task_map_path, temporary / artifacts["ik_task_map"])

        artifact_hashes = {
            name: sha256_file(temporary / relative)
            for name, relative in artifacts.items()
        }
        manifest = {
            "schema": "ohmc.simulation_bundle/v0.1",
            "target": target_name,
            "fidelity": target["fidelity"],
            "result": {
                "replay": replay_report["status"],
                "motion_validation": mapped_motion["validation"]["status"],
                "motion_quality": quality_report["status"],
                "landmark_coverage": landmark_report["status"],
                "ik": "pass" if ik_result is not None else "not_run",
                "hardware_commands_sent": False,
            },
            "capabilities": {
                "canonical_source_kinematics": True,
                "morphology_scaling": True,
                "canonical_timeline_resampling": True,
                "trajectory_derivatives": True,
                "mapping_completeness_report": True,
                "landmark_coverage_report": True,
                "semantic_joint_mapping": True,
                "headless_kinematic_replay": True,
                "vendor_interface_fixture": True,
                "constrained_partial_body_ik": ik_result is not None,
                "constrained_whole_body_ik": False,
                "dynamic_controller_simulation": False,
                "hardware_transport": False,
            },
            "inputs": {
                "source_sha256": hashlib.sha256(source_bytes).hexdigest(),
                "source_license": source_license,
                "source_convention": source_convention,
                "source_length_unit": source_length_unit,
                "morphology_scale": normalization["morphology_scale"],
                "normalization_rate_hz": normalization["rate_hz"],
                "simulation_model_sha256": sha256_file(model_path),
                "target_config_sha256": _object_sha256(target),
            },
            "artifacts": {
                name: {"path": relative, "sha256": artifact_hashes[name]}
                for name, relative in artifacts.items()
            },
            "warnings": mapped_motion["validation"]["issues"]
            + quality_report["warnings"]
            + landmark_report["warnings"]
            + [
                "replay is kinematic mj_forward validation, not closed-loop physics",
                "simulation success is not evidence of physical-robot safety",
            ],
        }
        manifest_issues = _schema_issues(manifest, load_json(bundle_schema_path))
        if manifest_issues:
            raise OhmcError(
                "generated invalid simulation manifest: " + "; ".join(manifest_issues)
            )
        _write_json(temporary / "manifest.json", manifest)
        temporary.replace(output_dir)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return manifest
