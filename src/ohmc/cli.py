"""OHMC command-line interface."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import tempfile
from pathlib import Path

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
from .ik import (
    build_ik_problem,
    ik_result_to_motion_ir,
    solve_ik_problem,
    validate_ik_problem,
    validate_ik_result,
    validate_ik_task_map,
)
from .ir import load_json, validate_motion_ir
from .landmarks import landmark_coverage_report, validate_landmark_coverage
from .normalization import normalize_canonical_motion
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
from .training import (
    TrainingStore,
    evaluate_run,
    execute_run,
    load_training_recipe,
    prepare_policy_bundle,
    prepare_run,
    training_doctor_report,
    verify_policy_bundle,
)
from .vendor import (
    default_cache_dir,
    doctor_report,
    import_official_artifact,
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

    training_doctor = subcommands.add_parser(
        "doctor", help="check the local X2 training environment"
    )
    training_doctor.add_argument(
        "--recipe",
        type=Path,
        default=default_project_root()
        / "examples"
        / "training"
        / "x2_rgbd_rough_ppo_v1.yaml",
    )
    training_doctor.add_argument(
        "--robot",
        "--profile",
        type=Path,
        default=default_project_root()
        / "profiles"
        / "agibot_x2_ultra_locomotion_29dof_v1.yaml",
    )
    training_doctor.add_argument("--json", action="store_true")
    training_doctor.add_argument(
        "--recipe-schema",
        type=Path,
        default=default_project_root()
        / "schemas"
        / "training-recipe-v0.1.schema.json",
    )
    training_doctor.add_argument(
        "--profile-schema",
        type=Path,
        default=default_project_root() / "schemas" / "robot-profile-v0.1.schema.json",
    )

    train = subcommands.add_parser(
        "train", help="create, preflight, and run a versioned training recipe"
    )
    train.add_argument(
        "recipe",
        type=Path,
        nargs="?",
        help="TrainingRecipe YAML (random initialization only)",
    )
    train.add_argument(
        "--resume-run",
        help="resume an interrupted run from its own curriculum checkpoint",
    )
    train.add_argument(
        "--runs-dir", type=Path, default=default_project_root() / "build" / "training-runs"
    )
    train.add_argument(
        "--robot",
        "--profile",
        type=Path,
        default=default_project_root()
        / "profiles"
        / "agibot_x2_ultra_locomotion_29dof_v1.yaml",
    )
    train.add_argument(
        "--prepare-only",
        action="store_true",
        help="create and preflight the run without launching the backend",
    )
    train.add_argument(
        "--recipe-schema",
        type=Path,
        default=default_project_root()
        / "schemas"
        / "training-recipe-v0.1.schema.json",
    )
    train.add_argument(
        "--run-schema",
        type=Path,
        default=default_project_root() / "schemas" / "run-manifest-v0.1.schema.json",
    )
    train.add_argument(
        "--profile-schema",
        type=Path,
        default=default_project_root() / "schemas" / "robot-profile-v0.1.schema.json",
    )

    evaluate = subcommands.add_parser(
        "evaluate", help="apply controller, Sim2Sim, and runtime fault gates"
    )
    evaluate.add_argument("run_id")
    evaluate.add_argument("--metrics", type=Path, required=True)
    evaluate.add_argument(
        "--runs-dir", type=Path, default=default_project_root() / "build" / "training-runs"
    )
    evaluate.add_argument(
        "--run-schema",
        type=Path,
        default=default_project_root() / "schemas" / "run-manifest-v0.1.schema.json",
    )
    evaluate.add_argument(
        "--metrics-schema",
        type=Path,
        default=default_project_root()
        / "schemas"
        / "evaluation-metrics-v0.1.schema.json",
    )
    evaluate.add_argument(
        "--evidence-schema",
        type=Path,
        default=default_project_root()
        / "schemas"
        / "evidence-bundle-v0.1.schema.json",
    )

    deploy = subcommands.add_parser(
        "deploy", help="prepare or verify a simulation-only policy candidate"
    )
    deploy_commands = deploy.add_subparsers(dest="deploy_command", required=True)
    deploy_prepare = deploy_commands.add_parser(
        "prepare", help="build an audited policy bundle after E3 Sim2Sim"
    )
    deploy_prepare.add_argument("run_id")
    deploy_prepare.add_argument("--policy", type=Path, required=True)
    deploy_prepare.add_argument("--output", type=Path, required=True)
    deploy_prepare.add_argument(
        "--artifact",
        action="append",
        default=[],
        metavar="ROLE=PATH",
        help="add checkpoint, normalization, camera_calibration, or test_vectors",
    )
    deploy_prepare.add_argument(
        "--runs-dir", type=Path, default=default_project_root() / "build" / "training-runs"
    )
    deploy_prepare.add_argument(
        "--robot",
        "--profile",
        type=Path,
        default=default_project_root()
        / "profiles"
        / "agibot_x2_ultra_locomotion_29dof_v1.yaml",
    )
    deploy_prepare.add_argument(
        "--run-schema",
        type=Path,
        default=default_project_root() / "schemas" / "run-manifest-v0.1.schema.json",
    )
    deploy_prepare.add_argument(
        "--profile-schema",
        type=Path,
        default=default_project_root() / "schemas" / "robot-profile-v0.1.schema.json",
    )
    deploy_prepare.add_argument(
        "--policy-schema",
        type=Path,
        default=default_project_root() / "schemas" / "policy-bundle-v0.1.schema.json",
    )
    deploy_verify = deploy_commands.add_parser(
        "verify", help="verify policy-bundle schemas, paths, hashes, and authority"
    )
    deploy_verify.add_argument("directory", type=Path)
    deploy_verify.add_argument(
        "--policy-schema",
        type=Path,
        default=default_project_root() / "schemas" / "policy-bundle-v0.1.schema.json",
    )

    web = subcommands.add_parser(
        "web", help="open the local beginner training dashboard"
    )
    web.add_argument("--host", default="127.0.0.1")
    web.add_argument("--port", type=int, default=8000)
    web.add_argument(
        "--runs-dir", type=Path, default=default_project_root() / "build" / "training-runs"
    )
    web.add_argument(
        "--recipe",
        type=Path,
        default=default_project_root()
        / "examples"
        / "training"
        / "x2_rgbd_rough_ppo_v1.yaml",
    )
    web.add_argument(
        "--robot",
        "--profile",
        type=Path,
        default=default_project_root()
        / "profiles"
        / "agibot_x2_ultra_locomotion_29dof_v1.yaml",
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

    if args.command == "doctor":
        recipe, issues = load_training_recipe(args.recipe, args.recipe_schema)
        if issues:
            raise OhmcError("invalid training recipe: " + "; ".join(issues))
        report = training_doctor_report(
            recipe,
            recipe_path=args.recipe.expanduser().resolve(),
            profile_path=args.robot,
            profile_schema_path=args.profile_schema,
        )
        if args.json:
            print(json.dumps(report, indent=2))
        else:
            print(f"training environment: {'ready' if report['ready'] else 'not ready'}")
            if report["recommended_env_count"] is not None:
                print(f"recommended parallel environments: {report['recommended_env_count']}")
            for check in report["checks"]:
                print(f"{check['status'].upper():<7} {check['name']}: {check['detail']}")
                if check.get("fix"):
                    print(f"        fix: {check['fix']}")
        return 0 if report["ready"] else 1

    if args.command == "train":
        store = TrainingStore(args.runs_dir, run_schema_path=args.run_schema)
        if args.resume_run:
            store.recover_orphaned_runs()
            code = execute_run(store, args.resume_run, resume=True)
            print(f"run state: {store.get(args.resume_run)['state']}")
            return code
        if args.recipe is None:
            raise OhmcError("a TrainingRecipe path is required unless --resume-run is used")
        recipe, issues = load_training_recipe(args.recipe, args.recipe_schema)
        if issues:
            raise OhmcError("invalid training recipe: " + "; ".join(issues))
        if recipe["robot_profile"] != args.robot.stem:
            raise OhmcError(
                f"recipe expects robot profile {recipe['robot_profile']}, "
                f"but --robot points to {args.robot.stem}"
            )
        manifest, report = prepare_run(
            store,
            recipe,
            args.recipe,
            profile_path=args.robot,
            profile_schema_path=args.profile_schema,
        )
        run_id = manifest["run_id"]
        print(f"training run: {run_id}")
        print(f"run directory: {store.run_dir(run_id)}")
        print(f"preflight: {'pass' if report['ready'] else 'blocked'}")
        if args.prepare_only or not report["ready"]:
            return 0 if report["ready"] else 1
        code = execute_run(store, run_id)
        print(f"run state: {store.get(run_id)['state']}")
        return code

    if args.command == "evaluate":
        store = TrainingStore(args.runs_dir, run_schema_path=args.run_schema)
        evidence = evaluate_run(
            store,
            args.run_id,
            args.metrics,
            metrics_schema_path=args.metrics_schema,
            evidence_schema_path=args.evidence_schema,
        )
        passed = sum(gate["status"] == "pass" for gate in evidence["gates"])
        print(
            f"evaluation: {evidence['status']} "
            f"({passed}/{len(evidence['gates'])} gates passed)"
        )
        print(f"authority: {evidence['authority']['label']}; hardware not tested")
        return 0 if evidence["status"] == "pass" else 1

    if args.command == "deploy":
        if args.deploy_command == "verify":
            report = verify_policy_bundle(
                args.directory, policy_schema_path=args.policy_schema
            )
            print(f"policy bundle: {report['status']}")
            print(f"authority: {report['authority']}")
            for issue in report["issues"]:
                print(f"ERROR {issue}")
            return 0 if report["status"] == "pass" else 1
        extras: list[tuple[str, Path]] = []
        for definition in args.artifact:
            role, separator, raw_path = definition.partition("=")
            if not separator or not role or not raw_path:
                raise OhmcError("--artifact must use ROLE=PATH")
            extras.append((role, Path(raw_path)))
        store = TrainingStore(args.runs_dir, run_schema_path=args.run_schema)
        bundle = prepare_policy_bundle(
            store,
            args.run_id,
            policy_path=args.policy,
            output_dir=args.output,
            profile_path=args.robot,
            profile_schema_path=args.profile_schema,
            policy_schema_path=args.policy_schema,
            extra_artifacts=extras,
        )
        print(f"policy bundle: {args.output.expanduser().resolve()}")
        print(f"bundle id: {bundle['bundle_id']}")
        print("authority: simulation-only; operator review required")
        return 0

    if args.command == "web":
        try:
            import uvicorn

            from .web import create_app
        except ImportError as exc:
            raise OhmcError(
                "web dependencies are missing; install with "
                "python -m pip install -e '.[web]'"
            ) from exc
        app = create_app(
            runs_dir=args.runs_dir,
            default_recipe=args.recipe,
            robot_profile=args.robot,
            project_root=default_project_root(),
        )
        print(f"OHMC dashboard: http://{args.host}:{args.port}")
        print("hardware transport: disabled")
        uvicorn.run(app, host=args.host, port=args.port)
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
