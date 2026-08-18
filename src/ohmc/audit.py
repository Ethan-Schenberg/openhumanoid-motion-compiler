"""Independent integrity verification for simulation evidence artifacts."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path, PurePosixPath
from typing import Any

from jsonschema import Draft202012Validator

from .simulation import validate_simulation_matrix


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _schema_issues(document: dict[str, Any], schema: dict[str, Any]) -> list[str]:
    issues = []
    for error in sorted(
        Draft202012Validator(schema).iter_errors(document),
        key=lambda item: list(item.path),
    ):
        location = ".".join(str(part) for part in error.absolute_path) or "$"
        issues.append(f"{location}: {error.message}")
    return issues


def _safe_child(root: Path, relative_value: str) -> Path | None:
    relative = PurePosixPath(relative_value)
    if relative.is_absolute() or not relative.parts or ".." in relative.parts:
        return None
    path = root.joinpath(*relative.parts)
    try:
        path.resolve().relative_to(root.resolve())
    except ValueError:
        return None
    return path


def _load_json(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return None, f"missing manifest: {path.name}"
    except (OSError, json.JSONDecodeError) as exc:
        return None, f"cannot read manifest {path.name}: {exc}"
    if not isinstance(value, dict):
        return None, f"manifest {path.name} must contain a JSON object"
    return value, None


def _verify_bundle(
    root: Path, manifest_schema: dict[str, Any]
) -> tuple[list[str], int, str | None]:
    manifest_path = root / "manifest.json"
    manifest, load_issue = _load_json(manifest_path)
    if load_issue:
        return [load_issue], 0, None
    assert manifest is not None
    issues = _schema_issues(manifest, manifest_schema)
    manifest_sha256 = _sha256_file(manifest_path)
    if issues:
        return [f"manifest: {issue}" for issue in issues], 0, manifest_sha256
    checked = 0
    for name, artifact in manifest["artifacts"].items():
        path = _safe_child(root, artifact["path"])
        if path is None:
            issues.append(f"artifact {name}: unsafe path {artifact['path']!r}")
            continue
        if not path.is_file():
            issues.append(f"artifact {name}: missing file {artifact['path']}")
            continue
        checked += 1
        actual = _sha256_file(path)
        if actual != artifact["sha256"]:
            issues.append(
                f"artifact {name}: SHA-256 mismatch; "
                f"expected {artifact['sha256']}, got {actual}"
            )
    return issues, checked, manifest_sha256


def verify_evidence(
    root: Path,
    *,
    bundle_schema: dict[str, Any],
    matrix_schema: dict[str, Any],
) -> dict[str, Any]:
    """Verify a bundle or matrix without executing a simulator or hardware."""
    root = root.expanduser().resolve()
    bundle_manifest = root / "manifest.json"
    matrix_manifest = root / "matrix-manifest.json"
    issues: list[str] = []
    checked_bundles = 0
    checked_artifacts = 0
    root_sha256 = None

    if bundle_manifest.is_file():
        kind = "simulation_bundle"
        bundle_issues, checked_artifacts, root_sha256 = _verify_bundle(
            root, bundle_schema
        )
        issues.extend(bundle_issues)
        checked_bundles = 1
    elif matrix_manifest.is_file():
        kind = "simulation_matrix"
        matrix, load_issue = _load_json(matrix_manifest)
        if load_issue:
            issues.append(load_issue)
        else:
            assert matrix is not None
            root_sha256 = _sha256_file(matrix_manifest)
            matrix_issues = validate_simulation_matrix(matrix, matrix_schema)
            issues.extend(f"matrix: {issue}" for issue in matrix_issues)
            if not matrix_issues:
                for row in matrix["targets"]:
                    if row["status"] != "pass":
                        continue
                    bundle_root = _safe_child(root, row["bundle"])
                    if bundle_root is None:
                        issues.append(
                            f"target {row['target']}: unsafe bundle path "
                            f"{row['bundle']!r}"
                        )
                        continue
                    child_manifest = bundle_root / "manifest.json"
                    if not child_manifest.is_file():
                        issues.append(
                            f"target {row['target']}: missing child manifest"
                        )
                        continue
                    actual_manifest_sha256 = _sha256_file(child_manifest)
                    if actual_manifest_sha256 != row["manifest_sha256"]:
                        issues.append(
                            f"target {row['target']}: child manifest SHA-256 "
                            f"mismatch; expected {row['manifest_sha256']}, "
                            f"got {actual_manifest_sha256}"
                        )
                    child_issues, artifact_count, _ = _verify_bundle(
                        bundle_root, bundle_schema
                    )
                    checked_bundles += 1
                    checked_artifacts += artifact_count
                    issues.extend(
                        f"target {row['target']}: {issue}"
                        for issue in child_issues
                    )
    else:
        kind = "unknown"
        issues.append("evidence directory has no manifest.json or matrix-manifest.json")

    return {
        "schema": "ohmc.evidence_audit/v0.1",
        "kind": kind,
        "status": "fail" if issues else "pass",
        "root_manifest_sha256": root_sha256,
        "checked_bundle_count": checked_bundles,
        "checked_artifact_count": checked_artifacts,
        "issues": issues,
        "hardware_commands_sent": False,
    }


def validate_evidence_audit(
    document: dict[str, Any], schema: dict[str, Any]
) -> list[str]:
    issues = _schema_issues(document, schema)
    if issues:
        return issues
    expected = "fail" if document["issues"] else "pass"
    if document["status"] != expected:
        issues.append(f"status must be {expected!r} for issues")
    return issues
