"""Robot profile validation and offline semantic joint mapping."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator
import yaml

from .errors import OhmcError


def load_yaml_object(path: Path) -> dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise OhmcError(f"file not found: {path}") from exc
    except yaml.YAMLError as exc:
        raise OhmcError(f"invalid YAML in {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise OhmcError(f"expected a YAML object in {path}")
    return data


def _schema_issues(document: dict[str, Any], schema: dict[str, Any]) -> list[str]:
    issues: list[str] = []
    for error in sorted(
        Draft202012Validator(schema).iter_errors(document),
        key=lambda item: list(item.path),
    ):
        location = ".".join(str(part) for part in error.absolute_path) or "$"
        issues.append(f"{location}: {error.message}")
    return issues


def validate_robot_profile(
    profile: dict[str, Any], schema: dict[str, Any]
) -> list[str]:
    issues = _schema_issues(profile, schema)
    if issues:
        return issues

    joint_order = profile["control"]["joint_order"]
    joint_set = set(joint_order)
    limit_set = set(profile["joint_limits"])
    semantic_values = list(profile["semantics"].values())
    if limit_set != joint_set:
        missing = sorted(joint_set - limit_set)
        extra = sorted(limit_set - joint_set)
        issues.append(f"joint_limits keys must match joint_order; missing={missing}, extra={extra}")
    if len(set(semantic_values)) != len(semantic_values):
        issues.append("semantic targets must be unique")
    if set(semantic_values) != joint_set:
        missing = sorted(joint_set - set(semantic_values))
        extra = sorted(set(semantic_values) - joint_set)
        issues.append(f"semantics must cover joint_order; missing={missing}, extra={extra}")

    excluded_names = [item["name"] for item in profile["control"]["excluded_joints"]]
    if len(set(excluded_names)) != len(excluded_names):
        issues.append("excluded joint names must be unique")
    overlap = sorted(joint_set & set(excluded_names))
    if overlap:
        issues.append(f"excluded joints also appear in joint_order: {overlap}")

    grouped: list[str] = []
    for group_name, names in profile["groups"].items():
        unknown = sorted(set(names) - joint_set)
        if unknown:
            issues.append(f"groups.{group_name}: unknown joints {unknown}")
        grouped.extend(names)
    if len(set(grouped)) != len(grouped):
        issues.append("a joint appears in more than one group")
    if set(grouped) != joint_set:
        missing = sorted(joint_set - set(grouped))
        issues.append(f"groups do not cover joint_order: {missing}")

    for name, limit in profile["joint_limits"].items():
        lower = float(limit["lower"])
        upper = float(limit["upper"])
        if not math.isfinite(lower) or not math.isfinite(upper):
            issues.append(f"joint_limits.{name}: bounds must be finite")
        elif lower >= upper:
            issues.append(f"joint_limits.{name}: lower must be less than upper")
    return issues


def validate_semantic_map(
    mapping: dict[str, Any], schema: dict[str, Any]
) -> list[str]:
    issues = _schema_issues(mapping, schema)
    if issues:
        return issues
    sources = [rule["source"] for rule in mapping["rules"]]
    targets = [rule["target"] for rule in mapping["rules"]]
    if len(set(sources)) != len(sources):
        issues.append("semantic map source names must be unique")
    if len(set(targets)) != len(targets):
        issues.append("semantic map target semantics must be unique")
    for index, rule in enumerate(mapping["rules"]):
        for field in ("scale", "offset"):
            value = float(rule.get(field, 0.0))
            if not math.isfinite(value):
                issues.append(f"rules.{index}.{field}: must be finite")
    return issues


def _stable_hash(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def map_motion_ir(
    document: dict[str, Any],
    profile: dict[str, Any],
    mapping: dict[str, Any],
) -> dict[str, Any]:
    """Map source joint channels to a vendor model without invoking IK."""
    source_joints = document["trajectory"]["joints"]
    source_index = {name: index for index, name in enumerate(source_joints)}
    semantic_to_joint = profile["semantics"]
    profile_order = profile["control"]["joint_order"]
    profile_index = {name: index for index, name in enumerate(profile_order)}

    resolved: list[dict[str, Any]] = []
    for rule in mapping["rules"]:
        source_name = rule["source"]
        semantic = rule["target"]
        if source_name not in source_index:
            raise OhmcError(f"semantic map source is absent from Motion IR: {source_name}")
        if semantic not in semantic_to_joint:
            raise OhmcError(
                f"semantic map target is absent from robot profile: {semantic}"
            )
        target_name = semantic_to_joint[semantic]
        resolved.append(
            {
                "source": source_name,
                "source_index": source_index[source_name],
                "target": target_name,
                "target_index": profile_index[target_name],
                "scale": float(rule["scale"]),
                "offset": float(rule.get("offset", 0.0)),
            }
        )
    resolved.sort(key=lambda item: item["target_index"])

    target_joints = [item["target"] for item in resolved]
    mapped_samples: list[dict[str, Any]] = []
    for sample_index, sample in enumerate(document["trajectory"]["samples"]):
        mapped_sample: dict[str, Any] = {"time": sample["time"]}
        positions = []
        for item in resolved:
            value = (
                float(sample["position_targets"][item["source_index"]]) * item["scale"]
                + item["offset"]
            )
            limit = profile["joint_limits"][item["target"]]
            if value < float(limit["lower"]) or value > float(limit["upper"]):
                raise OhmcError(
                    f"sample {sample_index} mapped joint {item['target']} violates profile "
                    f"range [{limit['lower']}, {limit['upper']}]: {value}"
                )
            positions.append(value)
        mapped_sample["position_targets"] = positions
        for field in ("velocity_targets", "acceleration_targets"):
            if field in sample:
                mapped_sample[field] = [
                    float(sample[field][item["source_index"]]) * item["scale"]
                    for item in resolved
                ]
        mapped_samples.append(mapped_sample)

    unmapped = sorted(set(source_joints) - {item["source"] for item in resolved})
    warnings = [
        "semantic joint mapping only; canonical skeleton retargeting and IK have not been applied",
        "hardware transport remains disabled by the selected robot profile",
    ]
    if unmapped:
        warnings.append("unmapped source joints: " + ", ".join(unmapped))

    output = deepcopy(document)
    output["robot"] = {
        "profile": profile["id"],
        "model_sha256": profile["model_evidence"]["model_sha256"],
        "vendor": profile["vendor"],
        "model": profile["model"],
    }
    output["trajectory"] = {
        "rate_hz": document["trajectory"]["rate_hz"],
        "joints": target_joints,
        "samples": mapped_samples,
    }
    output["passes"].append(
        {
            "name": "semantic_joint_map",
            "version": "0.1.0",
            "config_sha256": _stable_hash(
                {"profile": profile["id"], "mapping": mapping, "resolved": resolved}
            ),
            "warnings": warnings,
        }
    )
    existing_issues = list(output["validation"].get("issues", []))
    output["validation"] = {
        "status": "warning",
        "issues": existing_issues + warnings,
    }
    return output
