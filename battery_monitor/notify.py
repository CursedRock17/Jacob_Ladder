"""Alert delivery channels.

Every channel is best-effort and independently isolated: a logged-out desktop
session must never stop the remaining channels from firing, and must never
take down the monitor loop. Failures are logged once per channel rather than
on every alert, so a permanently-unavailable channel does not flood the
journal during a long flight.

journald is not a channel here -- the monitor writes every event to stdout and
systemd captures it, so the log path exists even with all channels disabled.
"""

from __future__ import annotations

import logging
import os
import queue
import shutil
import subprocess
import threading
import time
from pathlib import Path

from .config import LEVELS

log = logging.getLogger("battery_monitor.notify")

# Notification urgency per level.
_URGENCY = {"ok": "low", "warn": "normal", "land": "critical",
            "critical": "critical", "min": "critical"}


def level_at_least(level: str, minimum: str) -> bool:
    """True if `level` is at least as severe as `minimum`."""
    return LEVELS.index(level) >= LEVELS.index(minimum)


class Channel:
    """Base class: subclasses implement `_send`; failures are swallowed."""

    name = "channel"

    def __init__(self, settings: dict) -> None:
        self.settings = settings or {}
        self.enabled = bool(self.settings.get("enabled", False))
        self.min_level = self.settings.get("min_level", "warn")
        self._failed = False

    def available(self) -> tuple[bool, str]:
        """(usable, reason) -- checked once at startup so gaps surface early."""
        return True, ""

    def wants(self, level: str) -> bool:
        # Recovery notices ("ok") are routed explicitly by the monitor and are
        # not subject to min_level, which describes alert severity only.
        return self.enabled and (level == "ok" or level_at_least(level, self.min_level))

    def send(self, level: str, title: str, body: str) -> None:
        if not self.wants(level):
            return
        try:
            self._send(level, title, body)
            self._failed = False
        except Exception as exc:  # noqa: BLE001 - no channel may break the loop
            if not self._failed:
                log.warning("%s notification failed (further errors suppressed): %s",
                            self.name, exc)
                self._failed = True

    def _send(self, level: str, title: str, body: str) -> None:
        raise NotImplementedError


class DesktopChannel(Channel):
    """GNOME popup on the logged-in session.

    Under systemd this process has no session bus in its environment, so the
    address is reconstructed from the target user's runtime dir. `critical`
    urgency popups persist until dismissed, which is what we want for LAND.

    That persistence is also why the D-Bus path matters. `alerts.repeat_seconds`
    re-fires every 10 s at MIN, and each critical popup stays on screen, so
    plain notify-send would bury the display under dozens of identical
    never-dismissed popups during a sustained low-voltage event. Calling
    org.freedesktop.Notifications directly lets us pass `replaces_id`, so the
    alert updates in place and there is only ever one battery popup.

    notify-send is kept as a fallback (it cannot replace: 0.7.9 predates
    --replace-id), because a stacking popup still beats no popup.
    """

    name = "desktop"

    _URGENCY_BYTE = {"ok": 0, "warn": 1, "land": 2, "critical": 2, "min": 2}

    def __init__(self, settings: dict) -> None:
        super().__init__(settings)
        self._env = None
        self._bus = None
        self._notify_id = 0
        self._dbus_broken = False

    def _session_env(self) -> dict:
        # Only a *successful* lookup is memoized. At boot this service starts
        # long before the desktop session exists, so /run/user/<uid>/bus is
        # not there yet; caching that miss would leave the popup channel dead
        # for the whole uptime even after someone logs in. Re-resolving until
        # the bus appears costs a stat() per alert and makes it self-heal.
        if self._env is not None:
            return self._env
        env = dict(os.environ)
        if "DBUS_SESSION_BUS_ADDRESS" not in env:
            uid = os.getuid()
            user = self.settings.get("user")
            if user:
                import pwd
                try:
                    uid = pwd.getpwnam(str(user)).pw_uid
                except KeyError:
                    log.warning("desktop.user %r not found; using uid %d", user, uid)
            bus = Path(f"/run/user/{uid}/bus")
            if bus.exists():
                env["DBUS_SESSION_BUS_ADDRESS"] = f"unix:path={bus}"
            env.setdefault("XDG_RUNTIME_DIR", f"/run/user/{uid}")
        env.setdefault("DISPLAY", ":0")
        if "DBUS_SESSION_BUS_ADDRESS" in env:
            self._env = env
        return env

    def available(self) -> tuple[bool, str]:
        if shutil.which("notify-send") is None:
            return False, "notify-send not installed (apt install libnotify-bin)"
        if "DBUS_SESSION_BUS_ADDRESS" not in self._session_env():
            return False, ("no session bus at /run/user/<uid>/bus yet (no desktop "
                           "logged in) -- will retry on each alert")
        return True, ""

    def _connection(self):
        """Cached session-bus connection, reconnecting after a failure."""
        if self._bus is not None:
            return self._bus
        address = self._session_env().get("DBUS_SESSION_BUS_ADDRESS")
        if not address:
            return None
        import gi
        gi.require_version("Gio", "2.0")
        from gi.repository import Gio

        self._bus = Gio.DBusConnection.new_for_address_sync(
            address,
            Gio.DBusConnectionFlags.AUTHENTICATION_CLIENT
            | Gio.DBusConnectionFlags.MESSAGE_BUS_CONNECTION,
            None, None)
        return self._bus

    def _send_via_dbus(self, level: str, title: str, body: str) -> bool:
        """Notify with replaces_id. False if D-Bus is unusable here."""
        if self._dbus_broken:
            return False
        try:
            from gi.repository import GLib

            conn = self._connection()
            if conn is None:
                return False
            args = GLib.Variant(
                "(susssasa{sv}i)",
                ("Jacob's Ladder", self._notify_id, "battery-caution",
                 title, body, [],
                 {"urgency": GLib.Variant("y", self._URGENCY_BYTE.get(level, 1))},
                 -1),
            )
            result = conn.call_sync(
                "org.freedesktop.Notifications",
                "/org/freedesktop/Notifications",
                "org.freedesktop.Notifications",
                "Notify", args, GLib.VariantType("(u)"),
                0, 5000, None)
            self._notify_id = result.unpack()[0]
            return True
        except ImportError:
            # No PyGObject: fall back permanently, this will not change.
            self._dbus_broken = True
            return False
        except Exception as exc:  # noqa: BLE001
            # A dropped session (logout, GNOME restart) invalidates the cached
            # connection. Clear it so the next alert reconnects, and let this
            # one go out via notify-send rather than being lost.
            self._bus = None
            log.debug("desktop D-Bus notify failed, falling back: %s", exc)
            return False

    def _send(self, level: str, title: str, body: str) -> None:
        if self._send_via_dbus(level, title, body):
            return
        urgency = _URGENCY.get(level, "normal")
        cmd = [
            "notify-send",
            "--app-name=Jacob's Ladder",
            f"--urgency={urgency}",
            "--icon=battery-caution",
            title,
            body,
        ]
        subprocess.run(cmd, env=self._session_env(), check=True,
                       capture_output=True, timeout=10)


class WallChannel(Channel):
    """Broadcast to every attached terminal and SSH session."""

    name = "wall"

    def available(self) -> tuple[bool, str]:
        if shutil.which("wall") is None:
            return False, "wall not installed (apt install bsdutils)"
        return True, ""

    def _send(self, level: str, title: str, body: str) -> None:
        message = f"\n*** {title} ***\n{body}\n"
        # -n suppresses the banner; it is root-only, so fall back without it.
        try:
            subprocess.run(["wall", "-n"], input=message, text=True,
                           check=True, capture_output=True, timeout=10)
        except subprocess.CalledProcessError:
            subprocess.run(["wall"], input=message, text=True,
                           check=True, capture_output=True, timeout=10)


_CHANNEL_TYPES = {
    "desktop": DesktopChannel,
    "wall": WallChannel,
}


class Notifier:
    """Fans one alert out to every enabled channel, off the caller's thread.

    Delivery can block for a long time: both channels shell out or make a
    blocking D-Bus call, each with timeouts up to 10 s, and a wedged session
    bus or a stuck `wall` can hit them. Dispatching inline would stall the
    sampling loop by that much, and it would stall it *hardest* at the most
    severe levels, where alerts repeat every 10 s -- the monitor would go
    half-blind exactly when the readings matter most. So alerts go to a worker
    thread and sampling keeps its cadence regardless of how slow a channel is.

    The queue is bounded: if a channel wedges badly enough to back it up, new
    alerts are dropped with a log line rather than growing memory without
    limit on a flight computer.
    """

    def __init__(self, settings: dict, dispatch_async: bool = True) -> None:
        self.channels: list[Channel] = []
        for name, cls in _CHANNEL_TYPES.items():
            channel = cls(settings.get(name, {}))
            if channel.enabled:
                self.channels.append(channel)

        self._queue: queue.Queue | None = None
        self._thread: threading.Thread | None = None
        self._idle = threading.Event()
        self._idle.set()
        self._dropped = 0
        if dispatch_async:
            self._queue = queue.Queue(maxsize=16)
            self._thread = threading.Thread(
                target=self._worker, name="battery-notify", daemon=True)
            self._thread.start()

    def check_availability(self) -> list[str]:
        """Report enabled-but-unusable channels so gaps are visible at startup
        rather than discovered during the emergency they exist for."""
        problems = []
        for channel in self.channels:
            ok, reason = channel.available()
            if not ok:
                problems.append(f"{channel.name}: {reason}")
        return problems

    def send(self, level: str, title: str, body: str) -> None:
        if self._queue is None:
            self._dispatch(level, title, body)
            return
        try:
            self._queue.put_nowait((level, title, body))
            self._idle.clear()
        except queue.Full:
            self._dropped += 1
            log.warning("notification backlog full, dropped %s alert (%d total). "
                        "A channel is wedged; check the ones listed at startup.",
                        level.upper(), self._dropped)

    def _worker(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is None:
                    return
                self._dispatch(*item)
            finally:
                self._queue.task_done()
                if self._queue.empty():
                    self._idle.set()

    def _dispatch(self, level: str, title: str, body: str) -> None:
        for channel in self.channels:
            channel.send(level, title, body)

    def flush(self, timeout: float = 20.0) -> bool:
        """Block until queued alerts are delivered. True if fully drained."""
        if self._queue is None:
            return True
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._queue.empty() and self._idle.is_set():
                return True
            time.sleep(0.05)
        return False

    def close(self, timeout: float = 10.0) -> None:
        """Deliver what is queued, then stop the worker.

        Called on shutdown, so a LAND alert raised in the same instant as a
        SIGTERM still reaches the pilot instead of dying with the process.
        """
        if self._queue is None or self._thread is None:
            return
        self.flush(timeout)
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass
        self._thread.join(timeout=2.0)
