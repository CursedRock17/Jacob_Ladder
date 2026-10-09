"""python -m jl_flight: run and inspect missions.

run FILE [--airframe NAME] [--fresh-selection]   wait for the mode, then fly it
check FILE...                                    load each mission and validate it
name FILE                                        print the mission's name
helpers FILE [--airframe NAME]                   print the processes it needs, one per line
"""

from __future__ import annotations

import argparse
import shlex
import sys

from . import missions
from .errors import FlightError


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="jl_flight", description="run and inspect missions"
    )
    sub = parser.add_subparsers(dest="command", required=True)

    run = sub.add_parser(
        "run", help="wait for the mode to be selected, then fly the mission"
    )
    run.add_argument("file")
    run.add_argument("--airframe", default=None)
    run.add_argument(
        "--fresh-selection",
        action="store_true",
        help="ignore a mode that is already selected; wait for it to be selected again",
    )
    run.add_argument(
        "--report",
        type=float,
        default=None,
        metavar="SECONDS",
        help="print the setpoint, the drone and the targets this often",
    )

    check = sub.add_parser("check", help="load each mission and validate it")
    check.add_argument("files", nargs="+")
    check.add_argument("--airframe", default=None)

    name = sub.add_parser("name", help="print a mission's name")
    name.add_argument("file")

    helpers = sub.add_parser("helpers", help="print the processes a mission needs")
    helpers.add_argument("file")
    helpers.add_argument("--airframe", default=None)

    flight = sub.add_parser(
        "flight", help="check or read config/flight.yaml (used by deploy.sh)"
    )
    flight.add_argument(
        "action", choices=["check", "units", "packages", "names", "topics", "airframe"]
    )
    flight.add_argument("file", help="the flight file, e.g. config/flight.yaml")
    flight.add_argument(
        "--root", default=".", help="the repo root mission paths are relative to"
    )

    args = parser.parse_args(argv)
    if args.command == "flight":
        return _flight(args.action, args.file, args.root)
    try:
        if args.command == "check":
            failed = False
            for path in args.files:
                try:
                    spec = missions.load(path)
                    spec.check(args.airframe)
                    print(
                        f"ok   {path}  ({spec.name}, {len(spec.helpers(args.airframe))} helper(s))"
                    )
                except missions.MissionError as err:
                    failed = True
                    print(f"FAIL {err}", file=sys.stderr)
            return 1 if failed else 0
        spec = missions.load(args.file)
        if args.command == "name":
            print(spec.name)
        elif args.command == "helpers":
            spec.check(args.airframe)
            for helper in spec.helpers(args.airframe):
                print(shlex.join(helper.argv))
        else:
            missions.run(
                spec,
                airframe=args.airframe,
                fresh_selection=args.fresh_selection,
                report_every=args.report,
                log=lambda line: print(line, flush=True),
            )
    except missions.MissionError as err:
        print(f"jl_flight: {err}", file=sys.stderr)
        return 1
    except FlightError as err:
        print(f"jl_flight: flight ended: {err}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130
    return 0


def _flight(action: str, path: str, root: str) -> int:
    from . import flight_config as fc

    try:
        flight = fc.load_flight(path, root)
    except (missions.MissionError, OSError) as err:
        for line in str(err).splitlines():
            print(f"FAIL {line}")
        return 1
    specs, errors = fc.load_missions(flight)
    if action == "check":
        for line in errors:
            print(f"FAIL {line}")
        if not errors:
            names = ", ".join(spec.name for spec in specs)
            print(f"ok   {path}  (airframe {flight.airframe}: {names})")
        return 1 if errors else 0
    if errors:  # the other actions feed deploy.sh, which runs check first
        for line in errors:
            print(f"FAIL {line}", file=sys.stderr)
        return 1
    lines = {
        "units": lambda: fc.units(flight),
        "packages": lambda: fc.packages(flight, specs),
        "names": lambda: [spec.name for spec in specs],
        "topics": lambda: fc.topics(specs),
        "airframe": lambda: [fc.airframe_name(flight)],
    }[action]()
    for line in lines:
        print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
