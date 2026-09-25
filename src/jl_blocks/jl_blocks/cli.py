"""Command line: ``jl_blocks check missions/*.yaml``."""

from __future__ import annotations

import argparse
import sys

from . import library  # noqa: F401  (registers the shipped blocks)
from .core import MissionError, load_block_files, load_mission


def check(paths: list[str], blocks: list[str] | None = None) -> int:
    block_errors = load_block_files(blocks or [])
    if block_errors:
        print("FAIL blocks")
        for line in block_errors:
            print(f"  {line}")
        return 1
    failed = 0
    for path in paths:
        try:
            spec = load_mission(path)
        except FileNotFoundError:
            print(f"FAIL {path}\n  file not found")
            failed += 1
        except (OSError, UnicodeDecodeError) as err:
            print(f"FAIL {path}\n  cannot read file: {err}")
            failed += 1
        except MissionError as err:
            print(f"FAIL {path}")
            for line in err.errors:
                print(f"  {line}")
            failed += 1
        else:
            print(f"ok   {path}  ({spec.name}, {len(spec.steps)} steps)")
    return 1 if failed else 0


def flight(action: str, path: str, root: str) -> int:
    from .flight import check_flight, load_flight, packages, units

    try:
        config = load_flight(path, root)
    except (OSError, MissionError) as err:
        lines = err.errors if isinstance(err, MissionError) else [str(err)]
        print(f"FAIL {path}")
        for line in lines:
            print(f"  {line}")
        return 1
    if action == "check":
        errors = check_flight(config)
        if errors:
            print(f"FAIL {path}")
            for line in errors:
                print(f"  {line}")
            return 1
        print(
            f"ok   {path}  ({len(config.missions)} missions, helpers: "
            f"{', '.join(config.helpers) or 'none'})"
        )
    elif action == "units":
        print("\n".join(units(config)))
    elif action == "packages":
        print("\n".join(packages(config)))
    else:
        print(",".join(str(b) for b in config.blocks))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="jl_blocks", description="Jacob's Ladder mission blocks"
    )
    commands = parser.add_subparsers(dest="command", required=True)
    check_cmd = commands.add_parser(
        "check", help="check mission files without flying them"
    )
    check_cmd.add_argument(
        "--blocks",
        action="append",
        default=[],
        metavar="PATH",
        help="a .py file or folder of your own blocks (repeat for more)",
    )
    check_cmd.add_argument("files", nargs="+", help="mission YAML files")
    flight_cmd = commands.add_parser(
        "flight", help="check or read config/flight.yaml (used by deploy.sh)"
    )
    flight_cmd.add_argument("action", choices=["check", "units", "packages", "blocks"])
    flight_cmd.add_argument("file", help="the flight YAML, e.g. config/flight.yaml")
    flight_cmd.add_argument(
        "--root", default=".", help="repo root that mission paths are relative to"
    )
    args = parser.parse_args(argv)
    if args.command == "flight":
        return flight(args.action, args.file, args.root)
    return check(args.files, args.blocks)


if __name__ == "__main__":
    sys.exit(main())
