"""canbot_link.py — talk to the drive-base firmware's bring-up (text) protocol from Python.

This is the ONLY Python file that knows the wire format. bench.py and fit_feedforward.py
import it, so if the protocol ever changes there's exactly one place to update.

Design: a background thread reads the serial port continuously and sorts every line into
one of two places:
  - telemetry frames ("T,...")   -> a buffer you can drain at any time
  - replies ("OK" / "ERR,...")   -> a queue that command() waits on
Without the thread, a script waiting for "OK" could miss telemetry, or mistake a
telemetry line for the reply, because both arrive interleaved on the same port.
"""

from __future__ import annotations

import queue
import threading
import time
from collections import deque
from dataclasses import dataclass

import serial  # pip install pyserial

WHEELS = ("FL", "FR", "RL", "RR")


@dataclass
class Frame:
    """One telemetry line, decoded. All per-wheel lists are in FL, FR, RL, RR order."""

    t_ms: int
    counts: list[int]
    vel: list[float]      # measured wheel speed, rad/s
    target: list[float]   # commanded wheel speed, rad/s (0 in open-loop mode)
    duty: list[float]     # applied duty, -1..1
    faults: int
    battery_v: float
    loop_us_max: int

    @classmethod
    def parse(cls, line: str) -> "Frame | None":
        parts = line.split(",")
        if len(parts) != 21 or parts[0] != "T":
            return None
        try:
            return cls(
                t_ms=int(parts[1]),
                counts=[int(x) for x in parts[2:6]],
                vel=[float(x) for x in parts[6:10]],
                target=[float(x) for x in parts[10:14]],
                duty=[float(x) for x in parts[14:18]],
                faults=int(parts[18]),
                battery_v=float(parts[19]),
                loop_us_max=int(parts[20]),
            )
        except ValueError:
            return None


FAULT_NAMES = {
    1 << 0: "stall FL", 1 << 1: "stall FR", 1 << 2: "stall RL", 1 << 3: "stall RR",
    1 << 4: "E-stop", 1 << 5: "watchdog", 1 << 6: "loop overrun", 1 << 7: "hardware init",
}


def describe_faults(bits: int) -> str:
    names = [name for bit, name in FAULT_NAMES.items() if bits & bit]
    return ", ".join(names) if names else "none"


class CommandError(RuntimeError):
    pass


class BaseLink:
    """Context manager: `with BaseLink("/dev/ttyACM0") as link: ...` stops the wheels on exit."""

    def __init__(self, port: str, telemetry_hz: int = 50):
        # On the S3's native USB the baud rate is ignored, but pyserial still needs one.
        self._ser = serial.Serial(port, 115200, timeout=0.05)
        self._replies: queue.Queue[str] = queue.Queue()
        self._frames: deque[Frame] = deque(maxlen=20000)
        self._lock = threading.Lock()
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        time.sleep(0.2)
        self.command("S")
        self.command(f"T,{telemetry_hz}")

    # ---- plumbing --------------------------------------------------------------------
    def _read_loop(self) -> None:
        buf = b""
        while self._running:
            try:
                chunk = self._ser.read(256)
            except serial.SerialException:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", errors="replace").strip()
                if not line or line.startswith("#"):
                    continue  # comments are the board's help/config text
                if line.startswith("T,"):
                    frame = Frame.parse(line)
                    if frame:
                        with self._lock:
                            self._frames.append(frame)
                elif line == "OK" or line.startswith("ERR"):
                    self._replies.put(line)

    def command(self, text: str, timeout: float = 0.5) -> None:
        """Send one command and wait for the board's OK. Raises CommandError on ERR."""
        while not self._replies.empty():  # drop any stale reply
            self._replies.get_nowait()
        self._ser.write((text + "\n").encode("ascii"))
        try:
            reply = self._replies.get(timeout=timeout)
        except queue.Empty as exc:
            raise CommandError(f"no reply to {text!r} (is this the bring-up firmware?)") from exc
        if reply != "OK":
            raise CommandError(f"{text!r} -> {reply}")

    def send(self, text: str) -> None:
        """Fire-and-forget, for commands repeated at a fixed rate (keepalives)."""
        self._ser.write((text + "\n").encode("ascii"))

    # ---- telemetry -------------------------------------------------------------------
    def drain(self) -> list[Frame]:
        """Return every frame received since the last drain()."""
        with self._lock:
            frames = list(self._frames)
            self._frames.clear()
        return frames

    def latest(self, timeout: float = 1.0) -> Frame:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._frames:
                    return self._frames[-1]
            time.sleep(0.01)
        raise CommandError("no telemetry received")

    # ---- higher-level helpers ----------------------------------------------------------
    def hold(self, cmd: str, seconds: float, rate_hz: float = 20.0) -> list[Frame]:
        """Repeat `cmd` at rate_hz for `seconds`, collecting telemetry.

        Repeating is what keeps the board's command watchdog satisfied: a single
        "D,0.3,..." would stop after 300 ms by design.
        """
        self.drain()
        self.command(cmd)
        period = 1.0 / rate_hz
        end = time.monotonic() + seconds
        next_send = time.monotonic() + period
        while time.monotonic() < end:
            if time.monotonic() >= next_send:
                self.send(cmd)
                next_send += period
            time.sleep(0.002)
        return self.drain()

    def close(self) -> None:
        try:
            self.command("S")
            self.command("T,0")
        except (CommandError, serial.SerialException):
            pass
        self._running = False
        self._reader.join(timeout=1.0)
        self._ser.close()

    def __enter__(self) -> "BaseLink":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def wheel_vector(wheel: str, value: float) -> list[float]:
    """'FL' (or 'all') + a value -> a 4-element command vector."""
    wheel = wheel.upper()
    if wheel == "ALL":
        return [value] * 4
    vec = [0.0] * 4
    vec[WHEELS.index(wheel)] = value
    return vec


def fmt_cmd(letter: str, vec: list[float]) -> str:
    return letter + "," + ",".join(f"{v:.4f}" for v in vec)
