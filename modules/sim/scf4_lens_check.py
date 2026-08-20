#!/usr/bin/env python3

"""Drive the emulated lens with the aircraft's own driver, over a real socket.

The point of the emulator is that the companion runs the real umd_uas/zoom.py
against it, so the only check worth having is one where the client IS that
code. This starts scf4_emulator.py, opens it with umd_uas.scf4.Scf4 through
socket://, and walks the sequence zoom.py walks: identify, initialize, seed the
counters, home both axes, recall each framing the way ZoomDriver._move_axes
does, jog, and drop the link and come back.

It needs no Docker, no GPU and no ROS -- only the two checkouts on this
machine. ROS2_WS_DIR says where the flight code is.

    python3 modules/sim/scf4_lens_check.py

What it does not check: anything above the serial line. zoom.py's own gates
(the travel limits, the datum, the CameraInfo it publishes) need ROS, and
verify/component/localize.py covers the localization at each framing.
"""
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
WORKSPACE = Path(os.environ.get("ROS2_WS_DIR", HERE.parents[1].parent / "ros2_ws"))
sys.path.insert(0, str(WORKSPACE / "src" / "5g_drone"))

from umd_uas.scf4 import Scf4  # noqa: E402

EMU = str(HERE / "scf4_emulator.py")
# The same table scripts/zoom.sh holds, so this checks the numbers the
# simulator actually flies with rather than a set of its own.
PRESETS = os.environ.get("UAS_ZOOM_PRESETS", "narrow=6.09 mid=20.13 wide=56.06")
STEPS = os.environ.get(
    "UAS_ZOOM_STEPS", "narrow=29000,32380 mid=34500,32980 wide=40000,31140")
TRAVEL = os.environ.get("UAS_ZOOM_TRAVEL", "zoom=29000,40000 focus=29500,33500")
DATUM = os.environ.get("UAS_ZOOM_DATUM", "zoom=32000 focus=32000")
PORT = int(os.environ.get("PORT", "5911"))

class Log:
    def debug(self, m): pass
    def warn(self, m): print("   warn:", m)
    def info(self, m): print("   info:", m)

def start():
    proc = subprocess.Popen(
        [sys.executable, EMU, "--uas", "11", "--port", str(PORT),
         "--host", "127.0.0.1", "--presets", PRESETS, "--steps", STEPS,
         "--travel", TRAVEL, "--datum", DATUM],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)
    zoom_values = []
    def drain_out():
        for line in proc.stdout:
            zoom_values.append(float(line.strip()))
    def drain_err():
        for line in proc.stderr:
            print("   emu:", line.rstrip())
    threading.Thread(target=drain_out, daemon=True).start()
    threading.Thread(target=drain_err, daemon=True).start()
    time.sleep(1.0)
    return proc, zoom_values

failures = []
def check(ok, msg):
    print(("  PASS  " if ok else "  FAIL  ") + msg)
    if not ok: failures.append(msg)

proc, zoom_values = start()
try:
    dev = Scf4(port=f"socket://127.0.0.1:{PORT}", baudrate=115200, timeout=1.0, logger=Log())
    t0 = time.monotonic()
    dev.open()
    print("1. connect and identify")
    version = dev.version()
    print("   $S ->", version)
    status = dev.status()
    print("   !1 ->", status)
    check(len(status.raw) == 9, "!1 answers nine integers")

    print("2. initialize(), the vendor recipe zoom.py sends on connect")
    dev.initialize(stepping=[0,0,6], power=[190,190,190], filter_power=90,
                   sleep_power=[120,120,120], speed=[600,600,600],
                   pi_low=[400,400,400], pi_high=[700,700,700], pi_leds=True)
    dev.ir_filter(False)
    check(True, "initialize() completed without an error")

    print("3. seed the counters (G92), as zoom.py does before any jog")
    for axis, seat in (("zoom", 32000), ("focus", 32000)):
        dev.set_coordinate(axis, seat)
    st = dev.status()
    check(st.position["zoom"] == 32000 and st.position["focus"] == 32000,
          f"G92 seeded both counters: {st.position}")

    print("4. home both axes, the real procedure")
    for axis in ("zoom", "focus"):
        start_t = time.monotonic()
        res = dev.home(axis, home_position=32000, timeout=15.0,
                       approach_bound=(11000 if axis == "zoom" else 4000) + 4000,
                       max_band_steps=4000, clear_steps=200, passes=2,
                       position_window=None, seek_speed=1200, datum_speed=8000)
        took = time.monotonic() - start_t
        print(f"   {axis}: band={res.band} spread={res.spread} centre={res.center} "
              f"in {took:.1f}s")
        check(took < 15.0 * 6, f"{axis} homed in {took:.1f}s")
        check(res.band < 4000, f"{axis} band {res.band} inside max_band_steps")
        check(res.spread is not None and res.spread <= 10,
              f"{axis} repeatability {res.spread} within zoom.home.repeatability_steps")
        landed = dev.status().position[axis]
        check(landed == 32000, f"{axis} datum reads {landed}")

    print("5. recall each framing the way ZoomDriver._move_axes does:")
    print("   rapid to 300 short of the target, then precise onto it, always")
    print("   from below, so the mechanism takes up its backlash the same way")
    table = {"narrow": (29000, 32380), "mid": (34500, 32980), "wide": (40000, 31140)}
    hfov  = {"narrow": 6.09, "mid": 20.13, "wide": 56.06}
    LIMITS = {"zoom": (29000, 40000), "focus": (29500, 33500)}

    def approach_from(axis, target):
        low, high = LIMITS[axis]
        below = min(max(target - 300, low), high)
        if below != target:
            return below
        return min(max(target + 300, low), high)

    def recall(name):
        targets = dict(zip(("zoom", "focus"), table[name]))
        pre = {a: approach_from(a, t) for a, t in targets.items()}
        current = {a: dev.status().position[a] for a in targets}
        if any(abs(pre[a] - current[a]) > 300 for a in targets):
            dev.set_speed({"zoom": 400, "focus": 400})
            dev.move_absolute(pre)
            for a, t in pre.items():
                dev.wait_idle(a, 30.0, target=t)
        dev.set_speed({"zoom": 1500, "focus": 1500})
        dev.move_absolute(targets)
        return all(dev.wait_idle(a, 30.0, target=t) for a, t in targets.items())

    for name in ("mid", "wide", "narrow", "mid", "narrow", "wide"):
        z, f = table[name]
        started = time.monotonic()
        settled = recall(name)
        took = time.monotonic() - started
        st = dev.status()
        check(settled, f"{name}: both axes settled on target in {took:.1f}s")
        check(st.position["zoom"] == z and st.position["focus"] == f,
              f"{name}: landed on {st.position['zoom']}/{st.position['focus']}")
        time.sleep(0.3)
        seen = math.degrees(zoom_values[-1]) if zoom_values else float("nan")
        # Backlash. The counter lands on the target exactly; the mechanism
        # sits half the lost motion away from it, and the picture follows the
        # mechanism. Half of 40 steps is 0.5% of a framing, and the ray error
        # it causes scales with the angle off axis, so it is centimetres on
        # the ground at the edge of frame. Bounded, and real.
        check(abs(seen - hfov[name]) < 0.01 * hfov[name],
              f"{name}: lens shows {seen:.3f} deg, calibration says {hfov[name]} "
              f"(backlash error {100 * abs(seen - hfov[name]) / hfov[name]:.2f}%)")

    print("6. the picture moves with the lens, not in one jump")
    recall("narrow")
    time.sleep(0.3)
    before = len(zoom_values)
    dev.set_speed({"zoom": 1500, "focus": 1500})
    dev.move_absolute({"zoom": 40000, "focus": 31140})
    dev.wait_idle("zoom", 30.0, target=40000)
    dev.wait_idle("focus", 30.0, target=31140)
    during = zoom_values[before:]
    check(len(during) > 5, f"{len(during)} field-of-view updates during one recall")
    check(all(b >= a - 1e-6 for a, b in zip(during, during[1:])),
          "the field of view only widened on a wide recall")

    print("7. a jog is a relative move, and the counter follows it")
    dev.motion_mode("zoom", forced=False)
    dev.move_relative({"zoom": -500})
    dev.wait_idle("zoom", 10.0)
    check(dev.status().position["zoom"] == 39500,
          f"jog -500 landed at {dev.status().position['zoom']}")

    print("8. focus to infinity, the high extent")
    dev.move_absolute({"focus": 33500})
    check(dev.wait_idle("focus", 30.0, target=33500), "focus reached 33500")

    print("9. the IR filter answers")
    dev.ir_filter(True); dev.ir_filter(False)
    check(True, "M8/M7 answered")

    print("10. re-home with the travel window in force, as zoom/home does")
    for axis, window in (("zoom", (29000, 40000)), ("focus", (29500, 33500))):
        res = dev.home(axis, home_position=32000, timeout=15.0,
                       approach_bound=15000, max_band_steps=4000,
                       clear_steps=200, passes=2, position_window=window,
                       seek_speed=1200, datum_speed=8000)
        check(dev.status().position[axis] == 32000,
              f"{axis} re-homed inside {window}, spread {res.spread}")

    print("11. the link drops and the driver reconnects, as _on_serial_failure does")
    dev.close()
    time.sleep(1.0)
    dev.open()
    dev.initialize(stepping=[0,0,6], power=[190,190,190], filter_power=90,
                   sleep_power=[120,120,120], speed=[600,600,600],
                   pi_low=[400,400,400], pi_high=[700,700,700], pi_leds=True)
    check(len(dev.status().raw) == 9, "the controller answers again after a reconnect")
    check(dev.status().position["zoom"] == 32000,
          "the counter survived the reconnect (the lens was not power cycled)")

    print(f"12. a whole session took {time.monotonic() - t0:.1f}s")
    dev.close()
finally:
    proc.terminate()
    proc.wait(timeout=5)

print()
if failures:
    print(f"FAILED ({len(failures)}):")
    for f in failures: print("  -", f)
    sys.exit(1)
print("all protocol checks passed")
