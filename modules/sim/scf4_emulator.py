#!/usr/bin/env python3

"""An SCF4 lens controller, without the lens, for one simulated vehicle.

The aircraft drives a Kurokesu SCF4-M over a USB serial port. The simulator
has no such device, so this answers the same G-code on a TCP socket that
pyserial reaches as ``socket://sim:<port>``. The companion runs the real
``umd_uas/zoom.py`` against it: the same homing, the same travel limits, the
same preset recalls, the same CameraInfo. Only the device is different.

Everything it models is something the driver can observe:

  * ``M230`` normal mode -- a ``G0`` moves the commanded step count and stops.
  * ``M231`` forced mode -- a ``G0`` runs until the photo-interrupter flag
    changes state, and the controller stops the motor there.
  * ``M232`` -- the flag is a comparator with a lower and an upper threshold.
    Above the upper it reads 1, below the lower it reads 0, and between them it
    holds its last reading.
  * ``M240`` -- a step interval, so larger is slower.
  * the step counters are unsigned 16-bit and wrap rather than clamp.
  * lost motion on a direction reversal, and hard stops the controller cannot
    see, because it has no encoder.

and the serial line itself: every reply is delayed by the time its bytes would
take at the configured baud rate. Without that a poll loop with no sleep in it
-- which is how ``Scf4._seek_edge`` runs -- spins as fast as the socket allows
instead of at the few hundred hertz a real wire gives it.

The lens is where the picture comes from. The zoom axis position maps to a
field of view through the framings in ``scripts/zoom.sh``, and every change is
written to stdout as one radian value per line. ``gz_zoom_publisher`` turns
those into the Gazebo topic the video streamer crops with, so the picture
follows the lens continuously: a half-finished preset recall looks like a
half-finished zoom, which is what it looks like on the aircraft.

    scf4_emulator.py --uas 11 \
        --presets "narrow=6.09 mid=20.13 wide=56.06" \
        --steps "narrow=29000,32380 mid=34500,32980 wide=40000,31140" \
        --travel "zoom=29000,40000 focus=29500,33500" \
        --datum "zoom=32000 focus=32000" \
        --port 5911 | gz_zoom_publisher --topic /uas11/camera/zoom

What it is not: a model of the optics. Focus does nothing to the picture, the
iris is not driven, and the ``M232`` thresholds are accepted and ignored --
the switch band here is in steps, which is the only unit anything downstream
measures it in.
"""

import argparse
import math
import random
import socket
import sys
import threading
import time

# tunables ------------------------------------------------------------------

#: The controller's step counters, unsigned 16-bit.
COUNTER_MIN = 0
COUNTER_MAX = 65535

#: Steps per second at the vendor's default ``M240`` interval. The interval's
#: real unit is not documented, so this fixes the one thing that matters: a
#: homing leg and a full-travel move both have to finish well inside the
#: driver's own timeouts (zoom.home.timeout_sec 15 s per leg,
#: zoom.move.timeout_sec 30 s), and a slower interval has to be slower.
STEPS_PER_SECOND_AT_DEFAULT = 6000.0
SPEED_DEFAULT = 600

#: How long the axis carries on after the controller has decided to stop it.
#: A time rather than a distance, so it shrinks with the drive rate -- which is
#: why homing measures its trip points at zoom.home.datum_speed and why raising
#: that value tightens the repeatability.
COAST_SEC = 0.004

#: Run-to-run variation in where the axis rests, in steps, one standard
#: deviation. It is what zoom.home.repeatability_steps measures, so zero here
#: would report a repeatability no mechanism has.
COAST_JITTER_STEPS = 1.0

#: Lost motion on a direction reversal. zoom.home.clear_steps (200) has to
#: exceed it or the datum starts depending on which way the axis came from.
BACKLASH_STEPS = 40

#: Distance between the flag's two trip points, centred on the datum. Bounded
#: by zoom.home.max_band_steps (4000) on the driver's side.
SWITCH_BAND_STEPS = 300

#: How far past each end of the measured travel the mechanism actually stops.
#: The travel is marked just inside the stops on the aircraft, so a command
#: clamped to it always arrives instead of grinding.
STOP_MARGIN_STEPS = 150

#: Which way each axis's flag goes 0 -> 1 as the step counter increases. It has
#: to match umd_uas/scf4.py's FLAG_RISES, because the driver picks its seek
#: direction from it and an axis whose flag rises the other way never trips.
FLAG_RISES = {"zoom": +1, "focus": -1}

#: Logical axis name -> the G-code letter, in SCF4 channel order.
AXES = ("zoom", "focus", "iris")
LETTER = {"zoom": "A", "focus": "B", "iris": "C"}

#: Bits on the wire for one byte, start and stop included.
BITS_PER_BYTE = 10

#: Smallest change in field of view worth putting on stdout, in radians, and
#: the fastest it is written. The video streamer re-crops on every message.
ZOOM_EPSILON_RAD = math.radians(0.02)
ZOOM_MAX_HZ = 30.0

#: How long a connected client may say nothing before the link is dropped. One
#: client at a time is the point -- the aircraft's controller is a serial port,
#: and two drivers cannot open one -- so a half-open socket left by a companion
#: that was killed rather than closed would otherwise hold the lens for good.
#: Well above the 2 Hz the driver polls at when it is doing nothing.
CLIENT_IDLE_TIMEOUT_SEC = 60.0


def parse_table(text):
    """A "key=value key=value" table from scripts/zoom.sh, as a dict."""
    table = {}
    for entry in (text or "").split():
        key, _, value = entry.partition("=")
        if key and value:
            table[key] = value
    return table


def parse_pair(text, what):
    """A "<a>,<b>" value, as two ints."""
    parts = [part.strip() for part in text.split(",")]
    if len(parts) != 2:
        raise SystemExit(f"{what} must be two numbers separated by a comma, not {text!r}")
    return int(parts[0]), int(parts[1])


class Axis:
    """One stepper channel, its mechanism, and the switch on it.

    ``counter`` is what the controller reports. ``position`` is where the
    mechanism actually is, in steps from the switch centre, which is where
    homing writes the datum. The two agree only after homing, and the
    difference between them is the whole reason homing exists.
    """

    def __init__(self, name, datum, travel_low, travel_high, rng):
        self.name = name
        self.letter = LETTER[name]
        self.rises = FLAG_RISES.get(name, +1)
        self._rng = rng

        # The travel, and the switch, in positions relative to the switch.
        self.datum = datum
        self.stop_low = travel_low - datum - STOP_MARGIN_STEPS
        self.stop_high = travel_high - datum + STOP_MARGIN_STEPS
        half_band = SWITCH_BAND_STEPS / 2.0
        self.trip_low, self.trip_high = (
            (-half_band, half_band) if self.rises > 0 else (half_band, -half_band))

        self.counter = COUNTER_MIN
        self.position = 0.0
        self.slack = 0.0
        self.ground = 0            # steps commanded into a hard stop
        self.flag = self._level(self.position, 0)

        self.forced = False
        self.interval = SPEED_DEFAULT
        self.pending = 0           # steps still to run, sign carrying direction
        self.direction = 0         # forced-mode run direction, 0 when not seeking

    # -- physics -------------------------------------------------------------

    def _level(self, position, held):
        """The comparator's reading at a position, given what it last read."""
        side = 1 if self.rises > 0 else -1
        if side * position > side * self.trip_high:
            return 1
        if side * position < side * self.trip_low:
            return 0
        return held

    def _step(self, direction):
        """One step of the motor, and what the mechanism does about it."""
        self.counter = (self.counter + direction) % (COUNTER_MAX + 1)
        if self.slack * direction < 0:
            self.slack = 0.0
        if abs(self.slack) < BACKLASH_STEPS:
            self.slack += direction
            return
        moved = self.position + direction
        if moved < self.stop_low or moved > self.stop_high:
            self.ground += 1
            return
        self.position = moved
        self.flag = self._level(self.position, self.flag)

    @property
    def rate(self):
        """Steps per second at the current M240 interval. Larger is slower."""
        return STEPS_PER_SECOND_AT_DEFAULT * SPEED_DEFAULT / max(self.interval, 1)

    @property
    def moving(self):
        return bool(self.pending or self.direction)

    def advance(self, elapsed):
        """Run the motor for ``elapsed`` seconds of wall clock.

        Integrated when the controller is next spoken to rather than on a
        thread of its own. Motion ends on the step count, on the flag or on
        M0, and each of those is reached at a computable number of steps, so
        one pass over the elapsed time lands in exactly the same place a
        continuous model would.
        """
        budget = int(self.rate * max(elapsed, 0.0))
        while budget > 0 and self.moving:
            if self.direction:
                # Forced mode. The controller ends the move itself, on the
                # flag, and what is left to run afterwards becomes an ordinary
                # step count -- so there is one thing that stops this axis, not
                # two, and a coast of zero steps ends the seek like any other.
                before = self.flag
                run = 0
                while run < budget and self.flag == before:
                    self._step(self.direction)
                    run += 1
                budget -= run
                if self.flag != before:
                    self.pending = self.direction * self._coast_steps()
                    self.direction = 0
            elif self.pending:
                direction = 1 if self.pending > 0 else -1
                run = min(budget, abs(self.pending))
                self._run(direction, run)
                self.pending -= direction * run
                budget -= run

    def _run(self, direction, steps):
        for _ in range(steps):
            self._step(direction)

    def _coast_steps(self):
        """How far the axis carries on after the controller stops driving it.

        A time turned into a distance, so it shrinks with the drive rate.
        That is what makes zoom.home.datum_speed the knob the driver says it
        is: measuring the trip points slowly is what makes them repeatable.
        """
        carried = COAST_SEC * self.rate + self._rng.gauss(0.0, COAST_JITTER_STEPS)
        return max(0, int(round(carried)))

    # -- commands ------------------------------------------------------------

    def start(self, value, absolute):
        """Act on a ``G0`` for this axis."""
        if self.forced:
            # "does not stop turning motor after specified step count is
            # reached, instead seeks corresponding port PIN_x status state
            # change". Only the sign of the step count is used.
            self.pending = 0
            self.direction = 1 if value > 0 else -1
        elif absolute:
            self.pending = int(value) - self.counter
        else:
            self.pending = int(value)

    def stop(self):
        self.pending = 0
        self.direction = 0

    def set_counter(self, value):
        self.counter = int(value) % (COUNTER_MAX + 1)


class Lens:
    """What the framings say the picture is, from where the zoom axis sits.

    The framings anchor a curve of field of view against step position. Between
    them the magnification is geometric in the step count, which is how a zoom
    lens is marked and what makes a half-finished recall look half zoomed.
    Outside them the curve is flat: the travel ends at the outermost framings.
    """

    def __init__(self, anchors):
        if not anchors:
            raise SystemExit("no framings: the lens has no field of view to report")
        self._steps = [step for step, _ in anchors]
        self._log_tangents = [math.log(math.tan(math.radians(hfov) / 2.0))
                              for _, hfov in anchors]

    def hfov_rad(self, step):
        """Field of view at a zoom step position, in radians."""
        steps, logs = self._steps, self._log_tangents
        if len(steps) == 1 or step <= steps[0]:
            return 2.0 * math.atan(math.exp(logs[0]))
        if step >= steps[-1]:
            return 2.0 * math.atan(math.exp(logs[-1]))
        upper = next(i for i in range(1, len(steps)) if steps[i] >= step)
        span = steps[upper] - steps[upper - 1]
        fraction = (step - steps[upper - 1]) / span if span else 0.0
        blended = logs[upper - 1] + fraction * (logs[upper] - logs[upper - 1])
        return 2.0 * math.atan(math.exp(blended))


class Controller:
    """One SCF4 board: three channels, the modes, and the wire protocol."""

    def __init__(self, axes, lens, baudrate, log):
        self.axes = axes
        self.lens = lens
        self.baudrate = baudrate
        self.log = log
        self.absolute = False
        self.night_filter = False
        self._last_tick = time.monotonic()
        self._lock = threading.Lock()

    def _tick(self):
        now = time.monotonic()
        elapsed, self._last_tick = now - self._last_tick, now
        for axis in self.axes.values():
            axis.advance(elapsed)

    def zoom_hfov_rad(self):
        return self.lens.hfov_rad(self.axes["zoom"].position + self.axes["zoom"].datum)

    def wire_seconds(self, request, reply):
        """How long these bytes take on the wire, both directions."""
        return (len(request) + len(reply) + 4) * BITS_PER_BYTE / float(self.baudrate)

    def handle(self, command):
        """One command in, one reply line out. Never raises."""
        with self._lock:
            self._tick()
            return self._dispatch(command.strip())

    def _dispatch(self, command):
        if not command:
            return "OK"
        head = command.split()[0].upper()

        if head == "!1":
            return ", ".join(
                str(value) for value in
                [self.axes[a].counter for a in AXES]
                + [self.axes[a].flag for a in AXES]
                + [int(self.axes[a].moving) for a in AXES])
        if head == "$S":
            return "EVB.1.0.2, SCF4-M RevB, Kurokesu, SIMULATED"
        if head in ("M230", "M231"):
            forced = head == "M231"
            for axis in self._named(command):
                axis.forced = forced
                if not forced:
                    axis.direction = 0
            return "OK"
        if head in ("G90", "G91"):
            self.absolute = head == "G90"
            return "OK"
        if head == "G92":
            for axis, value in self._arguments(command):
                axis.set_counter(value)
            return "OK"
        if head == "G0":
            for axis, value in self._arguments(command):
                axis.start(value, self.absolute)
            return "OK"
        if head == "M0":
            for axis in self._named(command):
                axis.stop()
            return "OK"
        if head == "M240":
            for axis, value in self._arguments(command):
                axis.interval = max(int(value), 1)
            return "OK"
        if head in ("M7", "M8"):
            self.night_filter = head == "M8"
            return "OK"
        # $B2, M232, M234, M235, M238, M239, M243: accepted, and none of them
        # changes anything this models. The M232 comparator thresholds are in
        # ADC counts; the switch band here is in steps, which is the unit
        # everything downstream measures it in.
        if head in ("$B2", "M232", "M234", "M235", "M238", "M239", "M243"):
            return "OK"
        self.log(f"unknown command {command!r}")
        return "ERR"

    def _named(self, command):
        """The axes a command names, or all of them when it names none."""
        letters = {token[0].upper() for token in command.split()[1:] if token}
        chosen = [a for a in self.axes.values() if a.letter in letters]
        return chosen or list(self.axes.values())

    def _arguments(self, command):
        """(axis, value) for each ``A123`` style argument a command carries."""
        pairs = []
        for token in command.split()[1:]:
            letter, value = token[0].upper(), token[1:]
            axis = next((a for a in self.axes.values() if a.letter == letter), None)
            if axis is None:
                continue
            try:
                pairs.append((axis, int(float(value))))
            except ValueError:
                self.log(f"unreadable argument {token!r} in {command!r}")
        return pairs


class ZoomOutput:
    """The lens position, as a field of view, on stdout.

    One radian value per line, which is what gz_zoom_publisher puts on the
    Gazebo topic the video streamer crops with. Written only when the picture
    would visibly change, and never faster than the streamer can act on it.
    """

    def __init__(self, stream=sys.stdout):
        self._stream = stream
        self._last_value = None
        self._last_write = 0.0

    def offer(self, hfov_rad, force=False):
        now = time.monotonic()
        if not force:
            if now - self._last_write < 1.0 / ZOOM_MAX_HZ:
                return
            if (self._last_value is not None
                    and abs(hfov_rad - self._last_value) < ZOOM_EPSILON_RAD):
                return
        self._last_value, self._last_write = hfov_rad, now
        self._stream.write(f"{hfov_rad:.6f}\n")
        self._stream.flush()


def serve(controller, zoom_output, host, port, log):
    """Answer one client at a time, forever.

    One at a time on purpose: the aircraft's controller is a serial port, and
    two drivers cannot open one. A dropped link is a reconnect, which is
    exactly what zoom.py's connect timer does when the lens is unplugged.
    """
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((host, port))
    listener.listen(1)
    log(f"listening on {host}:{port}")

    while True:
        client, peer = listener.accept()
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        client.settimeout(CLIENT_IDLE_TIMEOUT_SEC)
        log(f"connected: {peer[0]}:{peer[1]}")
        try:
            _session(controller, zoom_output, client)
        except socket.timeout:
            log(f"silent for {CLIENT_IDLE_TIMEOUT_SEC:g}s; dropping the link so "
                f"another driver can have it")
        except OSError as exc:
            log(f"link lost: {exc}")
        finally:
            client.close()
            log("disconnected")


def _session(controller, zoom_output, client):
    buffer = b""
    while True:
        chunk = client.recv(4096)
        if not chunk:
            return
        buffer += chunk
        while b"\n" in buffer:
            line, _, buffer = buffer.partition(b"\n")
            command = line.decode("utf-8", "replace").strip("\r")
            reply = controller.handle(command)
            time.sleep(controller.wire_seconds(command, reply))
            client.sendall((reply + "\r\n").encode("utf-8"))
            zoom_output.offer(controller.zoom_hfov_rad())


def build(args, log):
    """The controller and the lens, from the tables in scripts/zoom.sh."""
    hfovs = parse_table(args.presets)
    steps = parse_table(args.steps)
    travel = parse_table(args.travel)
    datum = parse_table(args.datum)

    axes = {}
    for name in AXES:
        seat = datum.get(name)
        span = travel.get(name)
        if seat is None or span is None:
            # The iris has no travel and no datum, and nothing commands it.
            # It still has to answer, so it gets a channel with no mechanism.
            axes[name] = Axis(name, 0, -1, 1, random.Random(0))
            continue
        low, high = parse_pair(span, f"--travel {name}")
        axes[name] = Axis(name, int(seat), low, high,
                          random.Random(f"{args.uas}:{name}"))

    anchors = []
    for preset, hfov in hfovs.items():
        if preset not in steps:
            log(f"framing '{preset}' has no entry in --steps; it has no position")
            continue
        zoom_step, _focus_step = parse_pair(steps[preset], f"--steps {preset}")
        anchors.append((zoom_step, float(hfov)))
    anchors.sort()
    for (step, hfov), (next_step, next_hfov) in zip(anchors, anchors[1:]):
        if next_hfov <= hfov:
            log(f"WARNING: the framings are not a lens. Step {step} sees "
                f"{hfov} degrees and step {next_step} sees {next_hfov}, so "
                f"the view narrows as the lens zooms out. The picture will "
                f"follow the table.")

    lens = Lens(anchors)
    # Where the lens was left. The widest framing, so the picture starts wide
    # and the operator sees the startup homing sweep for what it is.
    widest = max(anchors, key=lambda anchor: anchor[1])[0] if anchors else None
    if widest is not None:
        axes["zoom"].position = widest - axes["zoom"].datum
        axes["zoom"].flag = axes["zoom"]._level(axes["zoom"].position, 0)

    return Controller(axes, lens, args.baudrate, log)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uas", type=int, required=True, help="vehicle number")
    parser.add_argument("--port", type=int, required=True,
                        help="TCP port to answer G-code on")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--presets", required=True,
                        help="UAS_ZOOM_PRESETS, '<name>=<hfov degrees> ...'")
    parser.add_argument("--steps", required=True,
                        help="UAS_ZOOM_STEPS, '<name>=<zoom>,<focus> ...'")
    parser.add_argument("--travel", required=True,
                        help="UAS_ZOOM_TRAVEL, '<axis>=<low>,<high> ...'")
    parser.add_argument("--datum", required=True,
                        help="UAS_ZOOM_DATUM, '<axis>=<step> ...'")
    parser.add_argument("--baudrate", type=int, default=115200,
                        help="what to delay each reply by, so a poll loop with "
                             "no sleep in it runs at the rate a wire gives it")
    args = parser.parse_args()

    def log(message):
        print(f"[scf4 uas{args.uas}] {message}", file=sys.stderr, flush=True)

    controller = build(args, log)
    zoom_output = ZoomOutput()
    for name in ("zoom", "focus"):
        axis = controller.axes[name]
        log(f"{name}: travel {axis.stop_low + axis.datum} to "
            f"{axis.stop_high + axis.datum} steps, datum {axis.datum}, "
            f"switch band {SWITCH_BAND_STEPS} steps, backlash {BACKLASH_STEPS}")
    log(f"lens starts at {math.degrees(controller.zoom_hfov_rad()):.2f} degrees")
    zoom_output.offer(controller.zoom_hfov_rad(), force=True)
    serve(controller, zoom_output, args.host, args.port, log)


if __name__ == "__main__":
    main()
