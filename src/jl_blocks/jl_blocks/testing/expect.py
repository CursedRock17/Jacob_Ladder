"""What test/sitl_mission.sh checks, as plain Python so it is tested without ROS.

StepOrder follows the runner's /jl/NAME/events; BandHold follows where the
camera sees the target. report() turns both into PASS/FAIL lines.
"""

from __future__ import annotations

from ..core import Vector3

STARTED = ": started"
ABORT_MARKERS = ("no on_fail -> hold, then land", "aborted: held, now landing")
REACTIVATED = "activated: starting from step 1"


class StepOrder:
    """Did the expected steps start, in this order? Later repeats are ignored."""

    def __init__(self, expected: list[str]) -> None:
        self.expected = tuple(expected)
        self._next = 0
        self.finished = False
        self.aborted = False
        self.deactivated = False

    def feed(self, event: str) -> None:
        if event == REACTIVATED:
            # A re-activation starts the order over: step progress and any
            # end state from an earlier flight are cleared.
            self._next = 0
            self.finished = False
            self.aborted = False
            self.deactivated = False
            return
        if event == "mission finished":
            self.finished = True
        if event == "deactivated":
            self.deactivated = True
        if any(marker in event for marker in ABORT_MARKERS):
            self.aborted = True
        if not event.endswith(STARTED):
            return
        step = event[: -len(STARTED)]
        if self._next < len(self.expected) and step == self.expected[self._next]:
            self._next += 1

    @property
    def ok(self) -> bool:
        return self._next == len(self.expected)

    @property
    def over(self) -> bool:
        """The mission has ended: finished, aborted, or deactivated."""
        return self.finished or self.aborted or self.deactivated

    def missing(self) -> list[str]:
        return list(self.expected[self._next :])


class BandHold:
    """Was a position inside center +/- tolerance (per axis) for hold_s seconds?

    A gap of more than max_gap_s between samples restarts the clock, so a
    target seen only now and then never counts as held. Once held, it stays held.
    """

    def __init__(
        self, center: Vector3, tolerance: float, hold_s: float, max_gap_s: float = 0.5
    ) -> None:
        self.center = center
        self.tolerance = tolerance
        self.hold_s = hold_s
        self.max_gap_s = max_gap_s
        self.held = False
        self.last: Vector3 | None = None
        self._since: float | None = None
        self._last_t: float | None = None

    def feed(self, position: Vector3, t: float) -> None:
        inside = all(
            abs(p - c) <= self.tolerance for p, c in zip(position, self.center)
        )
        gap = self._last_t is not None and t - self._last_t > self.max_gap_s
        if not inside:
            self._since = None
        elif self._since is None or gap:
            self._since = t
        self._last_t = t
        self.last = position
        if self._since is not None and t - self._since >= self.hold_s - 1e-9:
            self.held = True


def feed_gated(order: StepOrder, band: BandHold, position: Vector3, t: float) -> None:
    """Feed band only once every expected step in order has started.

    Samples seen before the last expected step starts (e.g. while still
    taking off or searching) must not count toward the hold.
    """
    if order.ok:
        band.feed(position, t)


def _fmt(v: Vector3) -> str:
    return ", ".join(f"{x:.2f}" for x in v)


def report(
    order: StepOrder, need_finished: bool, band: BandHold | None
) -> tuple[list[str], bool]:
    lines: list[str] = []
    ok = True
    wanted = " ".join(order.expected)
    if order.ok:
        lines.append(f"PASS steps started in order: {wanted}")
    else:
        lines.append(
            f"FAIL steps started in order: {wanted} "
            f"(missing: {' '.join(order.missing())})"
        )
        ok = False
    if order.aborted:
        lines.append("FAIL mission did not abort")
        ok = False
    else:
        lines.append("PASS mission did not abort")
    if order.deactivated:
        lines.append("FAIL mission was not deactivated")
        ok = False
    else:
        lines.append("PASS mission was not deactivated")
    if need_finished:
        lines.append(f"{'PASS' if order.finished else 'FAIL'} mission finished")
        ok = ok and order.finished
    if band is not None:
        what = (
            f"camera saw the target within {band.tolerance:g} m of "
            f"({_fmt(band.center)}) for {band.hold_s:g} s"
        )
        if band.held:
            lines.append(f"PASS {what}")
        elif band.last is None:
            lines.append(f"FAIL {what} (never saw it)")
            ok = False
        else:
            lines.append(f"FAIL {what} (last seen at ({_fmt(band.last)}))")
            ok = False
    return lines, ok
