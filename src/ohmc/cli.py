"""OHMC command-line interface."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from jsonschema import Draft202012Validator

from .adapters import SUPPORTED_ADAPTERS, encode_vendor_fixture
from .bvh import bvh_to_motion_ir, load_bvh
from .errors import OhmcError
from .ir import load_json, validate_motion_ir
from .profiles import (
    load_yaml_object,
    map_motion_ir,
    validate_robot_profile,
    validate_semantic_map,
)
from .replay import replay_mujoco
from .simulation import build_simulation_bundle
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

    inspect_source = subcommands.add_parser(
        "inspect-source", help="inspect a BVH source without compiling it"
    )
    inspect_source.add_argument("document", type=Path)

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
            "target registry key, such as unitree-g1-contract-fixture, "
            "unitree-g1, or agibot-x2-ultra"
        ),
    )
    simulate.add_argument(
        "--source-license",
        required=True,
        help="SPDX identifier or other explicit license assertion",
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

    if args.command == "inspect-source":
        motion = load_bvh(args.document)
        print(f"source: {args.document}")
        print(f"joints: {len(motion.joints)}")
        print(f"channels: {len(motion.channel_bindings)}")
        print(f"frames: {len(motion.frames)}")
        print(f"frame_time_seconds: {motion.frame_time:.9g}")
        print(f"duration_seconds: {motion.duration:.9g}")
        return 0

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
