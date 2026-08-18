"""OHMC command-line interface."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path
import tempfile

from jsonschema import Draft202012Validator

from .adapters import SUPPORTED_ADAPTERS, encode_vendor_fixture
from .audit import validate_evidence_audit, verify_evidence
from .bvh import bvh_to_motion_ir, load_bvh
from .canonical import (
    LENGTH_SCALES,
    SUPPORTED_SOURCE_CONVENTIONS,
    bvh_to_canonical_motion,
    validate_canonical_motion,
)
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
from .simulation import build_simulation_bundle, build_simulation_matrix
from .vendor import (
    default_cache_dir,
    import_official_artifact,
    doctor_report,
    load_vendor_lock,
    status_all,
    sync_git_vendor,
)


def default_project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="ohmc")
    subcommands = parser.add_subparsers(dest="command", required=True)

    validate_ir = subcommands.add_parser("validate-ir", help="validate Motion IR")
    validate_ir.add_argument("document", type=Path)
    validate_ir.add_argument(
        "--schema",
        type=Path,
        default=default_project_root() / "schemas" / "motion-ir-v0.1.schema.json",
    )

    verify_evidence_parser = subcommands.add_parser(
        "verify-evidence",
        help="verify bundle or matrix schemas, paths, and SHA-256 integrity",
    )
    verify_evidence_parser.add_argument("directory", type=Path)
    verify_evidence_parser.add_argument("--report", type=Path)
    verify_evidence_parser.add_argument("--force", action="store_true")
    verify_evidence_parser.add_argument(
        "--bundle-schema",
        type=Path,
        default=default_project_root()
        / "schemas"
        / "simulation-bundle-v0.1.schema.json",
    )
    verify_evidence_parser.add_argument(
        "--matrix-schema",
        type=Path,
        default=default_project_root()
        / "schemas"
        / "simulation-matrix-v0.1.schema.json",
    )
    verify_evidence_parser.add_argument(
        "--audit-schema",
        type=Path,
        default=default_project_root() / "schemas" / "evidence-audit-v0.1.schema.json",
    )

    inspect_source = subcommands.add_parser(
        "inspect-source", help="inspect a BVH source without compiling it"
    )
    inspect_source.add_argument("document", type=Path)

    canonicalize_bvh = subcommands.add_parser(
        "canonicalize-bvh",
        help="evaluate a BVH skeleton in canonical coordinates",
    )
    canonicalize_bvh.add_argument("document", type=Path)
    canonicalize_bvh.add_argument("--output", type=Path, required=True)
    canonicalize_bvh.add_argument("--source-license", required=True)
    canonicalize_bvh.add_argument(
        "--source-convention",
        choices=SUPPORTED_SOURCE_CONVENTIONS,
        required=True,
    )
    canonicalize_bvh.add_argument(
        "--source-length-unit",
        choices=tuple(LENGTH_SCALES),
        required=True,
    )
    canonicalize_bvh.add_argument("--force", action="store_true")
    canonicalize_bvh.add_argument(
        "--schema",
        type=Path,
        default=default_project_root()
        / "schemas"
        / "canonical-motion-v0.1.schema.json",
    )

    normalize_canonical = subcommands.add_parser(
        "normalize-canonical",
        help="scale and resample canonical skeleton motion with deterministic FK",
    )
    normalize_canonical.add_argument("document", type=Path)
    normalize_canonical.add_argument("--output", type=Path, required=True)
    normalize_canonical.add_argument("--morphology-scale", type=float, default=1.0)
    normalize_canonical.add_argument("--rate-hz", type=float, required=True)
    normalize_canonical.add_argument("--force", action="store_true")
    normalize_canonical.add_argument(
        "--schema",
        type=Path,
        default=default_project_root()
        / "schemas"
        / "canonical-motion-v0.1.schema.json",
    )

    retarget_ik = subcommands.add_parser(
        "retarget-ik",
        help="compile and solve a constrained offline IK evidence bundle",
    )
    retarget_ik.add_argument("document", type=Path, help="canonical motion JSON")
    retarget_ik.add_argument("--robot", type=Path, required=True)
    retarget_ik.add_argument("--task-map", type=Path, required=True)
    retarget_ik.add_argument("--model", type=Path, required=True)
    retarget_ik.add_argument("--output", type=Path, required=True)
    retarget_ik.add_argument(
        "--canonical-schema",
        type=Path,
        default=default_project_root()
        / "schemas"
        / "canonical-motion-v0.1.schema.json",
    )

    landmark_report = subcommands.add_parser(
        "landmark-report",
        help="report canonical source and IK task landmark coverage",
    )
    landmark_report.add_argument("document", type=Path)
    landmark_report.add_argument("--task-map", type=Path)
    landmark_report.add_argument("--output", type=Path, required=True)
    landmark_report.add_argument("--force", action="store_true")
    landmark_report.add_argument("--require-full-source", action="store_true")
    landmark_report.add_argument("--require-full-tasks", action="store_true")
    landmark_report.add_argument(
        "--canonical-schema",
        type=Path,
        default=default_project_root()
        / "schemas"
        / "canonical-motion-v0.1.schema.json",
    )
    landmark_report.add_argument(
        "--task-map-schema",
        type=Path,
        default=default_project_root() / "schemas" / "ik-task-map-v0.1.schema.json",
    )
    landmark_report.add_argument(
        "--report-schema",
        type=Path,
        default=default_project_root()
        / "schemas"
        / "landmark-coverage-v0.1.schema.json",
    )
    retarget_ik.add_argument(
        "--profile-schema",
        type=Path,
        default=default_project_root() / "schemas" / "robot-profile-v0.1.schema.json",
    )
    retarget_ik.add_argument(
        "--task-map-schema",
        type=Path,
        default=default_project_root() / "schemas" / "ik-task-map-v0.1.schema.json",
    )
    retarget_ik.add_argument(
        "--problem-schema",
        type=Path,
        default=default_project_root() / "schemas" / "ik-problem-v0.1.schema.json",
    )
    retarget_ik.add_argument(
        "--result-schema",
        type=Path,
        default=default_project_root() / "schemas" / "ik-result-v0.1.schema.json",
    )
    retarget_ik.add_argument(
        "--motion-schema",
        type=Path,
        default=default_project_root() / "schemas" / "motion-ir-v0.1.schema.json",
    )
    import_bvh = subcommands.add_parser(
        "import-bvh", help="import BVH rotation channels into prototype Motion IR"
    )
    import_bvh.add_argument("document", type=Path)
    import_bvh.add_argument("--output", type=Path, required=True)
    import_bvh.add_argument(
        "--source-license",
        required=True,
        help="SPDX identifier or other explicit license assertion for the BVH source",
    )
    import_bvh.add_argument("--force", action="store_true")
    import_bvh.add_argument(
        "--schema",
        type=Path,
        default=default_project_root() / "schemas" / "motion-ir-v0.1.schema.json",
    )

    derive_kinematics = subcommands.add_parser(
        "derive-kinematics",
        help="derive velocity and acceleration targets from Motion IR timestamps",
    )
    derive_kinematics.add_argument("document", type=Path)
    derive_kinematics.add_argument("--output", type=Path, required=True)
    derive_kinematics.add_argument("--force", action="store_true")
    derive_kinematics.add_argument(
        "--schema",
        type=Path,
        default=default_project_root() / "schemas" / "motion-ir-v0.1.schema.json",
    )

    replay = subcommands.add_parser(
        "replay", help="perform offline replay validation of Motion IR"
    )
    replay.add_argument("document", type=Path)
    replay.add_argument("--backend", choices=["mujoco"], required=True)
    replay.add_argument("--model", type=Path, required=True)
    replay.add_argument("--report", type=Path)
    replay.add_argument(
        "--schema",
        type=Path,
        default=default_project_root() / "schemas" / "motion-ir-v0.1.schema.json",
    )

    inspect_robot = subcommands.add_parser(
        "inspect-robot", help="validate and summarize an offline robot profile"
    )
    inspect_robot.add_argument("profile", type=Path)
    inspect_robot.add_argument(
        "--profile-schema",
        type=Path,
        default=default_project_root() / "schemas" / "robot-profile-v0.1.schema.json",
    )

    quality_report = subcommands.add_parser(
        "quality-report",
        help="measure mapping coverage and enforce robot trajectory limits",
    )
    quality_report.add_argument("document", type=Path)
    quality_report.add_argument("--robot", type=Path, required=True)
    quality_report.add_argument("--output", type=Path, required=True)
    quality_report.add_argument("--require-complete-mapping", action="store_true")
    quality_report.add_argument("--require-dynamic-limits", action="store_true")
    quality_report.add_argument(
        "--schema",
        type=Path,
        default=default_project_root() / "schemas" / "motion-ir-v0.1.schema.json",
    )
    quality_report.add_argument(
        "--profile-schema",
        type=Path,
        default=default_project_root() / "schemas" / "robot-profile-v0.1.schema.json",
    )
    quality_report.add_argument(
        "--report-schema",
        type=Path,
        default=default_project_root()
        / "schemas"
        / "trajectory-quality-v0.1.schema.json",
    )

    map_joints = subcommands.add_parser(
        "map-joints", help="apply an offline semantic joint map to Motion IR"
    )
    map_joints.add_argument("document", type=Path)
    map_joints.add_argument("--robot", type=Path, required=True)
    map_joints.add_argument("--mapping", type=Path, required=True)
    map_joints.add_argument("--output", type=Path, required=True)
    map_joints.add_argument("--force", action="store_true")
    map_joints.add_argument(
        "--schema",
        type=Path,
        default=default_project_root() / "schemas" / "motion-ir-v0.1.schema.json",
    )
    map_joints.add_argument(
        "--profile-schema",
        type=Path,
        default=default_project_root() / "schemas" / "robot-profile-v0.1.schema.json",
    )
    map_joints.add_argument(
        "--mapping-schema",
        type=Path,
        default=default_project_root() / "schemas" / "semantic-map-v0.1.schema.json",
    )

    encode_fixture = subcommands.add_parser(
        "encode-fixture", help="encode a non-executable vendor interface-order fixture"
    )
    encode_fixture.add_argument("document", type=Path)
    encode_fixture.add_argument("--robot", type=Path, required=True)
    encode_fixture.add_argument("--adapter", choices=SUPPORTED_ADAPTERS, required=True)
    encode_fixture.add_argument("--output", type=Path, required=True)
    encode_fixture.add_argument("--force", action="store_true")
    encode_fixture.add_argument(
        "--schema",
        type=Path,
        default=default_project_root() / "schemas" / "motion-ir-v0.1.schema.json",
    )
    encode_fixture.add_argument(
        "--profile-schema",
        type=Path,
        default=default_project_root() / "schemas" / "robot-profile-v0.1.schema.json",
    )
    encode_fixture.add_argument(
        "--fixture-schema",
        type=Path,
        default=default_project_root()
        / "schemas"
        / "vendor-interface-fixture-v0.1.schema.json",
    )

    simulate = subcommands.add_parser(
        "simulate",
        help="compile, map, replay, and package an offline simulation in one command",
    )
    simulate.add_argument("document", type=Path, help="licensed BVH input")
    simulate.add_argument(
        "--target",
        required=True,
        help=(
            "target registry key, such as unitree-g1, agibot-x2-ultra, or all"
        ),
    )
    simulate.add_argument(
        "--source-license",
        required=True,
        help="SPDX identifier or other explicit license assertion",
    )
    simulate.add_argument(
        "--source-convention",
        choices=SUPPORTED_SOURCE_CONVENTIONS,
        required=True,
        help="explicit coordinate convention declared by the BVH source",
    )
    simulate.add_argument(
        "--source-length-unit",
        choices=tuple(LENGTH_SCALES),
        required=True,
        help="length unit used by BVH OFFSET and position channels",
    )
    simulate.add_argument(
        "--output", type=Path, required=True, help="new evidence-bundle directory"
    )
    simulate.add_argument(
        "--targets",
        type=Path,
        default=default_project_root() / "profiles" / "simulation_targets_v0.1.yaml",
    )
    simulate.add_argument(
        "--target-schema",
        type=Path,
        default=default_project_root()
        / "schemas"
        / "simulation-targets-v0.1.schema.json",
    )
    simulate.add_argument(
        "--lock",
        type=Path,
        default=default_project_root() / "vendor" / "vendor-lock.yaml",
    )
    simulate.add_argument(
        "--cache-dir",
        type=Path,
        default=default_cache_dir(),
        help="verified vendor dependency cache",
    )
    simulate.add_argument(
        "--schema",
        type=Path,
        default=default_project_root() / "schemas" / "motion-ir-v0.1.schema.json",
    )
    simulate.add_argument(
        "--profile-schema",
        type=Path,
        default=default_project_root() / "schemas" / "robot-profile-v0.1.schema.json",
    )
    simulate.add_argument(
        "--mapping-schema",
        type=Path,
        default=default_project_root() / "schemas" / "semantic-map-v0.1.schema.json",
    )
    simulate.add_argument(
        "--fixture-schema",
        type=Path,
        default=default_project_root()
        / "schemas"
        / "vendor-interface-fixture-v0.1.schema.json",
    )
    simulate.add_argument(
        "--bundle-schema",
        type=Path,
        default=default_project_root()
        / "schemas"
        / "simulation-bundle-v0.1.schema.json",
    )
    simulate.add_argument(
        "--canonical-schema",
        type=Path,
        default=default_project_root()
        / "schemas"
        / "canonical-motion-v0.1.schema.json",
    )
    simulate.add_argument(
        "--quality-schema",
        type=Path,
        default=default_project_root()
        / "schemas"
        / "trajectory-quality-v0.1.schema.json",
    )
    simulate.add_argument(
        "--ik-task-map-schema",
        type=Path,
        default=default_project_root() / "schemas" / "ik-task-map-v0.1.schema.json",
    )
    simulate.add_argument(
        "--ik-problem-schema",
        type=Path,
        default=default_project_root() / "schemas" / "ik-problem-v0.1.schema.json",
    )
    simulate.add_argument(
        "--ik-result-schema",
        type=Path,
        default=default_project_root() / "schemas" / "ik-result-v0.1.schema.json",
    )
    simulate.add_argument(
        "--landmark-schema",
        type=Path,
        default=default_project_root()
        / "schemas"
        / "landmark-coverage-v0.1.schema.json",
    )
    simulate.add_argument(
        "--matrix-schema",
        type=Path,
        default=default_project_root()
        / "schemas"
        / "simulation-matrix-v0.1.schema.json",
    )

    vendor = subcommands.add_parser("vendor", help="manage vendor SDK dependencies")
    vendor.add_argument(
        "--lock",
        type=Path,
        default=default_project_root() / "vendor" / "vendor-lock.yaml",
    )
    vendor.add_argument("--cache-dir", type=Path, default=default_cache_dir())
    vendor_commands = vendor.add_subparsers(dest="vendor_command", required=True)

    status = vendor_commands.add_parser("status", help="show dependency status")
    status.add_argument("vendor", nargs="?")

    verify = vendor_commands.add_parser("verify", help="verify installed dependencies")
    verify.add_argument("vendor", nargs="?")

    import_command = vendor_commands.add_parser(
        "import", help="import an official downloaded artifact"
    )
    import_command.add_argument("vendor")
    import_command.add_argument("artifact", type=Path)

    sync = vendor_commands.add_parser("sync", help="sync pinned Git dependencies")
    sync.add_argument("vendor")

    doctor = vendor_commands.add_parser(
        "doctor", help="print detailed vendor health report"
    )
    doctor.add_argument("vendor", nargs="?")
    doctor.add_argument(
        "--json", action="store_true", help="emit machine-readable doctor report"
    )
    return parser


def _print_doctor_report(report: dict) -> None:
    if not report["vendors"]:
        print("No matching vendor components.")
        return
    for vendor_name, vendor_data in report["vendors"].items():
        print(f"vendor: {vendor_name}")
        print(f"  critical: {vendor_data['critical']}")
        print(f"  warning: {vendor_data['warning']}")
        for component in vendor_data["components"]:
            print(
                f"  - {component['component']:<20} {component['state']:<16} "
                f"{component['acquisition']:<20} {component['detail']}"
            )
            for warning in component["warnings"]:
                print(f"    warning: {warning}")
            for error in component["errors"]:
                print(f"    error: {error}")


def _print_statuses(statuses: list, strict: bool) -> int:
    print(f"{'COMPONENT':<28} {'STATE':<20} DETAIL")
    failed = False
    for status in statuses:
        component_name = f"{status.component.vendor}.{status.component.name}"
        print(f"{component_name:<28} {status.state:<20} {status.detail}")
        if status.state not in {"verified", "system"}:
            failed = True
    return 1 if strict and failed else 0


def run(args: argparse.Namespace) -> int:
    if args.command == "validate-ir":
        document = load_json(args.document)
        schema = load_json(args.schema)
        issues = validate_motion_ir(document, schema)
        if issues:
            for issue in issues:
                print(f"ERROR {issue}")
            return 1
        print(f"valid Motion IR: {args.document}")
        return 0

    if args.command == "verify-evidence":
        audit = verify_evidence(
            args.directory,
            bundle_schema=load_json(args.bundle_schema),
            matrix_schema=load_json(args.matrix_schema),
        )
        audit_issues = validate_evidence_audit(audit, load_json(args.audit_schema))
        if audit_issues:
            raise OhmcError("generated invalid evidence audit: " + "; ".join(audit_issues))
        if args.report:
            report = args.report.expanduser().resolve()
            if report.exists() and not args.force:
                raise OhmcError(f"refusing to overwrite existing output: {report}")
            report.parent.mkdir(parents=True, exist_ok=True)
            report.write_text(json.dumps(audit, indent=2) + "\n", encoding="utf-8")
        print(
            f"evidence integrity: {audit['status']} "
            f"(kind={audit['kind']}, bundles={audit['checked_bundle_count']}, "
            f"artifacts={audit['checked_artifact_count']}, "
            f"issues={len(audit['issues'])})"
        )
        for issue in audit["issues"]:
            print(f"ERROR {issue}")
        return 0 if audit["status"] == "pass" else 1

    if args.command == "inspect-source":
        motion = load_bvh(args.document)
        print(f"source: {args.document}")
        print(f"joints: {len(motion.joints)}")
        print(f"channels: {len(motion.channel_bindings)}")
        print(f"frames: {len(motion.frames)}")
        print(f"frame_time_seconds: {motion.frame_time:.9g}")
        print(f"duration_seconds: {motion.duration:.9g}")
        return 0

    if args.command == "canonicalize-bvh":
        output = args.output.expanduser().resolve()
        if output.exists() and not args.force:
            raise OhmcError(f"refusing to overwrite existing output: {output}")
        source = args.document.expanduser().resolve()
        try:
            source_bytes = source.read_bytes()
        except FileNotFoundError as exc:
            raise OhmcError(f"file not found: {source}") from exc
        document = bvh_to_canonical_motion(
            load_bvh(source),
            source_bytes=source_bytes,
            source_name=args.document.name,
            source_license=args.source_license,
            source_convention=args.source_convention,
            source_length_unit=args.source_length_unit,
        )
        issues = validate_canonical_motion(document, load_json(args.schema))
        if issues:
            raise OhmcError("generated invalid canonical motion: " + "; ".join(issues))
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
        print(
            f"canonical motion: {output} "
            f"({len(document['skeleton']['joints'])} joints, "
            f"{len(document['samples'])} frames)"
        )
        return 0

    if args.command == "normalize-canonical":
        output = args.output.expanduser().resolve()
        if output.exists() and not args.force:
            raise OhmcError(f"refusing to overwrite existing output: {output}")
        document = load_json(args.document)
        schema = load_json(args.schema)
        issues = validate_canonical_motion(document, schema)
        if issues:
            raise OhmcError(
                "cannot normalize invalid canonical motion: " + "; ".join(issues)
            )
        normalized = normalize_canonical_motion(
            document,
            morphology_scale=args.morphology_scale,
            rate_hz=args.rate_hz,
        )
        output_issues = validate_canonical_motion(normalized, schema)
        if output_issues:
            raise OhmcError(
                "generated invalid normalized motion: " + "; ".join(output_issues)
            )
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(normalized, indent=2) + "\n", encoding="utf-8")
        print(
            f"normalized canonical motion: {output} "
            f"({len(normalized['samples'])} samples, rate={args.rate_hz:g} Hz, "
            f"scale={args.morphology_scale:g})"
        )
        return 0

    if args.command == "retarget-ik":
        output = args.output.expanduser().resolve()
        if output.exists():
            raise OhmcError(f"refusing to overwrite existing output: {output}")
        canonical = load_json(args.document)
        canonical_issues = validate_canonical_motion(
            canonical, load_json(args.canonical_schema)
        )
        if canonical_issues:
            raise OhmcError(
                "cannot retarget invalid canonical motion: "
                + "; ".join(canonical_issues)
            )
        profile = load_yaml_object(args.robot)
        profile_issues = validate_robot_profile(profile, load_json(args.profile_schema))
        if profile_issues:
            raise OhmcError("invalid robot profile: " + "; ".join(profile_issues))
        task_map = load_yaml_object(args.task_map)
        task_map_issues = validate_ik_task_map(
            task_map, load_json(args.task_map_schema)
        )
        if task_map_issues:
            raise OhmcError("invalid IK task map: " + "; ".join(task_map_issues))
        model = args.model.expanduser().resolve()
        problem = build_ik_problem(canonical, profile, task_map, model)
        problem_issues = validate_ik_problem(
            problem, load_json(args.problem_schema)
        )
        if problem_issues:
            raise OhmcError("generated invalid IK problem: " + "; ".join(problem_issues))
        result = solve_ik_problem(problem, model)
        result_issues = validate_ik_result(result, load_json(args.result_schema))
        if result_issues:
            raise OhmcError("generated invalid IK result: " + "; ".join(result_issues))

        motion = None
        if result["status"] == "pass":
            motion = derive_motion_kinematics(
                ik_result_to_motion_ir(problem, result, canonical, profile)
            )
            motion_issues = validate_motion_ir(motion, load_json(args.motion_schema))
            if motion_issues:
                raise OhmcError(
                    "generated invalid IK Motion IR: " + "; ".join(motion_issues)
                )
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = Path(
            tempfile.mkdtemp(prefix=f".{output.name}-", dir=str(output.parent))
        )
        try:
            (temporary / "ik-problem.json").write_text(
                json.dumps(problem, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            (temporary / "ik-result.json").write_text(
                json.dumps(result, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            if motion is not None:
                (temporary / "motion.json").write_text(
                    json.dumps(motion, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8",
                )
            temporary.replace(output)
        except Exception:
            shutil.rmtree(temporary, ignore_errors=True)
            raise
        print(
            f"IK bundle: {output} (status={result['status']}, "
            f"solved={result['summary']['solved_frame_count']}/"
            f"{result['summary']['frame_count']}, "
            f"peak_residual={result['summary']['peak_residual_m']:.9g} m)"
        )
        return 0 if result["status"] == "pass" else 1

    if args.command == "landmark-report":
        output = args.output.expanduser().resolve()
        if output.exists() and not args.force:
            raise OhmcError(f"refusing to overwrite existing output: {output}")
        canonical = load_json(args.document)
        canonical_issues = validate_canonical_motion(
            canonical, load_json(args.canonical_schema)
        )
        if canonical_issues:
            raise OhmcError(
                "cannot inspect invalid canonical motion: "
                + "; ".join(canonical_issues)
            )
        task_map = None
        if args.task_map:
            task_map = load_yaml_object(args.task_map)
            task_issues = validate_ik_task_map(
                task_map, load_json(args.task_map_schema)
            )
            if task_issues:
                raise OhmcError("invalid IK task map: " + "; ".join(task_issues))
        report = landmark_coverage_report(canonical, task_map)
        report_issues = validate_landmark_coverage(
            report, load_json(args.report_schema)
        )
        if report_issues:
            raise OhmcError(
                "generated invalid landmark report: " + "; ".join(report_issues)
            )
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        task_coverage = report["task_coverage"]
        task_text = (
            "not evaluated"
            if task_coverage is None
            else f"{task_coverage['present_count']}/{task_coverage['required_count']}"
        )
        print(
            f"landmark coverage: source="
            f"{report['source']['present_count']}/{report['source']['required_count']}, "
            f"tasks={task_text}, status={report['status']}"
        )
        strict_failure = (
            args.require_full_source and not report["source"]["complete"]
        ) or (
            args.require_full_tasks
            and (task_coverage is None or not task_coverage["complete"])
        )
        return 1 if strict_failure else 0

    if args.command == "import-bvh":
        output = args.output.expanduser().resolve()
        if output.exists() and not args.force:
            raise OhmcError(f"refusing to overwrite existing output: {output}")
        source = args.document.expanduser().resolve()
        try:
            source_bytes = source.read_bytes()
        except FileNotFoundError as exc:
            raise OhmcError(f"file not found: {source}") from exc
        motion = load_bvh(source)
        document = bvh_to_motion_ir(
            motion,
            source_bytes=source_bytes,
            source_name=args.document.name,
            source_license=args.source_license,
        )
        schema = load_json(args.schema)
        issues = validate_motion_ir(document, schema)
        if issues:
            raise OhmcError("generated invalid Motion IR: " + "; ".join(issues))
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
        print(f"imported BVH Motion IR: {output}")
        return 0

    if args.command == "derive-kinematics":
        output = args.output.expanduser().resolve()
        if output.exists() and not args.force:
            raise OhmcError(f"refusing to overwrite existing output: {output}")
        document = load_json(args.document)
        schema = load_json(args.schema)
        issues = validate_motion_ir(document, schema)
        if issues:
            raise OhmcError("cannot derive invalid Motion IR: " + "; ".join(issues))
        derived = derive_motion_kinematics(document)
        output_issues = validate_motion_ir(derived, schema)
        if output_issues:
            raise OhmcError(
                "generated invalid derived Motion IR: " + "; ".join(output_issues)
            )
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(derived, indent=2) + "\n", encoding="utf-8")
        print(
            f"derived kinematics: {output} "
            f"({len(derived['trajectory']['samples'])} samples)"
        )
        return 0

    if args.command == "replay":
        document = load_json(args.document)
        schema = load_json(args.schema)
        issues = validate_motion_ir(document, schema)
        if issues:
            raise OhmcError("cannot replay invalid Motion IR: " + "; ".join(issues))
        if args.backend == "mujoco":
            report = replay_mujoco(document, args.model.expanduser().resolve())
        else:  # pragma: no cover - argparse restricts the value
            raise OhmcError(f"unsupported replay backend: {args.backend}")
        if args.report:
            report_path = args.report.expanduser().resolve()
            report_path.parent.mkdir(parents=True, exist_ok=True)
            report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            print(f"replay report: {report_path}")
        print(
            f"replay passed: {report['frames_replayed']} frames, "
            f"{report['joints_mapped']} joints, backend={report['backend']}"
        )
        return 0

    if args.command == "inspect-robot":
        profile = load_yaml_object(args.profile)
        profile_schema = load_json(args.profile_schema)
        issues = validate_robot_profile(profile, profile_schema)
        if issues:
            raise OhmcError("invalid robot profile: " + "; ".join(issues))
        print(f"profile: {profile['id']}")
        print(f"vendor: {profile['vendor']}")
        print(f"model: {profile['model']}")
        print(f"controllable_joints: {len(profile['control']['joint_order'])}")
        print(f"excluded_joints: {len(profile['control']['excluded_joints'])}")
        print(f"hardware_transport: {profile['control']['hardware_transport']}")
        print(f"model_sha256: {profile['model_evidence']['model_sha256']}")
        return 0

    if args.command == "quality-report":
        motion = load_json(args.document)
        motion_issues = validate_motion_ir(motion, load_json(args.schema))
        if motion_issues:
            raise OhmcError("cannot analyze invalid Motion IR: " + "; ".join(motion_issues))
        profile = load_yaml_object(args.robot)
        profile_issues = validate_robot_profile(
            profile, load_json(args.profile_schema)
        )
        if profile_issues:
            raise OhmcError("invalid robot profile: " + "; ".join(profile_issues))
        report = trajectory_quality_report(motion, profile)
        report_issues = validate_quality_report(
            report, load_json(args.report_schema)
        )
        if report_issues:
            raise OhmcError(
                "generated invalid quality report: " + "; ".join(report_issues)
            )
        output = args.output.expanduser().resolve()
        if output.exists():
            raise OhmcError(f"refusing to overwrite existing output: {output}")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        mapping = report["mapping"]
        print(
            f"trajectory quality: {report['status']} "
            f"({mapping['mapped_joint_count']}/{mapping['controllable_joint_count']} "
            f"joints, {len(report['violations'])} violations)"
        )
        dynamic = report["dynamic_limit_coverage"]
        strict_failure = (
            report["status"] == "fail"
            or (args.require_complete_mapping and not mapping["complete"])
            or (
                args.require_dynamic_limits
                and (
                    dynamic["velocity_configured_joint_count"]
                    != dynamic["mapped_joint_count"]
                    or dynamic["acceleration_configured_joint_count"]
                    != dynamic["mapped_joint_count"]
                )
            )
        )
        return 1 if strict_failure else 0

    if args.command == "map-joints":
        output = args.output.expanduser().resolve()
        if output.exists() and not args.force:
            raise OhmcError(f"refusing to overwrite existing output: {output}")
        motion = load_json(args.document)
        motion_schema = load_json(args.schema)
        input_issues = validate_motion_ir(motion, motion_schema)
        if input_issues:
            raise OhmcError("cannot map invalid Motion IR: " + "; ".join(input_issues))
        profile = load_yaml_object(args.robot)
        profile_issues = validate_robot_profile(
            profile, load_json(args.profile_schema)
        )
        if profile_issues:
            raise OhmcError("invalid robot profile: " + "; ".join(profile_issues))
        mapping = load_yaml_object(args.mapping)
        mapping_issues = validate_semantic_map(
            mapping, load_json(args.mapping_schema)
        )
        if mapping_issues:
            raise OhmcError("invalid semantic map: " + "; ".join(mapping_issues))
        mapped = map_motion_ir(motion, profile, mapping)
        output_issues = validate_motion_ir(mapped, motion_schema)
        if output_issues:
            raise OhmcError("generated invalid Motion IR: " + "; ".join(output_issues))
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(mapped, indent=2) + "\n", encoding="utf-8")
        print(
            f"mapped Motion IR: {output} ({len(mapped['trajectory']['joints'])} joints, "
            f"profile={profile['id']})"
        )
        return 0

    if args.command == "encode-fixture":
        output = args.output.expanduser().resolve()
        if output.exists() and not args.force:
            raise OhmcError(f"refusing to overwrite existing output: {output}")
        motion = load_json(args.document)
        input_issues = validate_motion_ir(motion, load_json(args.schema))
        if input_issues:
            raise OhmcError(
                "cannot encode invalid Motion IR: " + "; ".join(input_issues)
            )
        profile = load_yaml_object(args.robot)
        profile_issues = validate_robot_profile(
            profile, load_json(args.profile_schema)
        )
        if profile_issues:
            raise OhmcError("invalid robot profile: " + "; ".join(profile_issues))
        fixture = encode_vendor_fixture(motion, profile, args.adapter)
        fixture_issues = []
        validator = Draft202012Validator(load_json(args.fixture_schema))
        for error in validator.iter_errors(fixture):
            location = ".".join(str(part) for part in error.absolute_path) or "$"
            fixture_issues.append(f"{location}: {error.message}")
        if fixture_issues:
            raise OhmcError("generated invalid vendor fixture: " + "; ".join(fixture_issues))
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(fixture, indent=2) + "\n", encoding="utf-8")
        print(
            f"encoded non-executable fixture: {output} "
            f"({len(fixture['frames'])} frames, adapter={args.adapter})"
        )
        return 0

    if args.command == "simulate":
        if args.target == "all":
            matrix = build_simulation_matrix(
                source_path=args.document,
                source_license=args.source_license,
                output_dir=args.output,
                registry_path=args.targets,
                registry_schema_path=args.target_schema,
                matrix_schema_path=args.matrix_schema,
                vendor_lock_path=args.lock,
                cache_dir=args.cache_dir,
                project_root=default_project_root(),
                motion_schema_path=args.schema,
                profile_schema_path=args.profile_schema,
                mapping_schema_path=args.mapping_schema,
                fixture_schema_path=args.fixture_schema,
                bundle_schema_path=args.bundle_schema,
                canonical_schema_path=args.canonical_schema,
                source_convention=args.source_convention,
                source_length_unit=args.source_length_unit,
                quality_schema_path=args.quality_schema,
                ik_task_map_schema_path=args.ik_task_map_schema,
                ik_problem_schema_path=args.ik_problem_schema,
                ik_result_schema_path=args.ik_result_schema,
                landmark_schema_path=args.landmark_schema,
            )
            summary = matrix["summary"]
            print(
                f"simulation matrix: {args.output.expanduser().resolve()} "
                f"(status={summary['status']}, passed={summary['passed_count']}/"
                f"{summary['target_count']})"
            )
            return 0 if summary["status"] == "pass" else 1
        manifest = build_simulation_bundle(
            source_path=args.document,
            source_license=args.source_license,
            output_dir=args.output,
            target_name=args.target,
            registry_path=args.targets,
            registry_schema_path=args.target_schema,
            vendor_lock_path=args.lock,
            cache_dir=args.cache_dir,
            project_root=default_project_root(),
            motion_schema_path=args.schema,
            profile_schema_path=args.profile_schema,
            mapping_schema_path=args.mapping_schema,
            fixture_schema_path=args.fixture_schema,
            bundle_schema_path=args.bundle_schema,
            canonical_schema_path=args.canonical_schema,
            source_convention=args.source_convention,
            source_length_unit=args.source_length_unit,
            quality_schema_path=args.quality_schema,
            ik_task_map_schema_path=args.ik_task_map_schema,
            ik_problem_schema_path=args.ik_problem_schema,
            ik_result_schema_path=args.ik_result_schema,
            landmark_schema_path=args.landmark_schema,
        )
        print(
            f"simulation bundle: {args.output.expanduser().resolve()} "
            f"(target={manifest['target']}, replay={manifest['result']['replay']})"
        )
        return 0

    lock = load_vendor_lock(args.lock)
    cache_dir = args.cache_dir.expanduser().resolve()
    if args.vendor_command == "status":
        return _print_statuses(status_all(lock, cache_dir, args.vendor), strict=False)
    if args.vendor_command == "verify":
        return _print_statuses(status_all(lock, cache_dir, args.vendor), strict=True)
    if args.vendor_command == "import":
        destination = import_official_artifact(
            lock, cache_dir, args.vendor, args.artifact.expanduser().resolve()
        )
        print(f"imported and verified: {destination}")
        return 0
    if args.vendor_command == "sync":
        for destination in sync_git_vendor(lock, cache_dir, args.vendor):
            print(f"synced and verified: {destination}")
        return 0
    if args.vendor_command == "doctor":
        report = doctor_report(lock, cache_dir, args.vendor)
        if args.json:
            print(json.dumps(report, indent=2))
        else:
            _print_doctor_report(report)
        return 1 if not report["healthy"] else 0
    raise OhmcError("unhandled command")


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return run(args)
    except OhmcError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
