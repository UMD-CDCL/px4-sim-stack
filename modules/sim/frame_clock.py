#!/usr/bin/env python3
"""Forward frame capture times from the video streamer to the message bus.

Why this exists
---------------
A detection is only useful if you know where the drone was looking when the
frame was taken. That means the localizer needs the frame's capture time, and
H.264 over RTSP does not carry one:

  - MediaMTX does not pass through usable RTCP sender reports. A GStreamer
    client asking for `add-reference-timestamp-meta` gets nothing, so there is
    no NTP mapping to recover.
  - DeepStream stamps its output when nvmsgconv runs, which is after decode and
    inference. Measured on this stack, that is 60 to 90 ms after capture, and
    it moves with GPU load. At 15 m/s that is a metre of error, in a direction
    that changes.
  - `attach-sys-ts-as-ntp=0` does not help. Setting it changes nothing in the
    payload, which was confirmed by measuring both settings.

So the capture time travels out of band. gz_video_streamer reads the clock in
its image callback, before it does any encoding work, and sends one small UDP
datagram for each frame. This program forwards those to MQTT, where the ROS
side joins them back to detections.

UDP is deliberate: a lost capture record costs one frame of precision, and it
must never block the encoder.

Topic:   video/frames/<stream>
Payload: {"stream","seq","capture_unix_ns","sim_time_ns"}
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import sys
import time

import paho.mqtt.client as mqtt

running = True


def stop(*_):
    global running
    running = False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--listen", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=int(os.environ.get("FRAME_CLOCK_PORT", "5599")))
    ap.add_argument("--host", default=os.environ.get("MQTT_HOST", "message-bus"))
    ap.add_argument("--mqtt-port", type=int, default=int(os.environ.get("MQTT_PORT", "1883")))
    ap.add_argument("--prefix", default=os.environ.get("FRAME_TOPIC_PREFIX", "video/frames"))
    args = ap.parse_args()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # A big receive buffer, so a scheduling hiccup here does not drop records.
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    sock.bind((args.listen, args.port))
    sock.settimeout(0.5)

    if hasattr(mqtt, "CallbackAPIVersion"):
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="px4simstack-frameclock")
    else:
        client = mqtt.Client(client_id="px4simstack-frameclock")
    client.connect_async(args.host, args.mqtt_port, keepalive=30)
    client.loop_start()

    print(f"[frame-clock] udp {args.listen}:{args.port} -> "
          f"mqtt://{args.host}:{args.mqtt_port}/{args.prefix}/<stream>", flush=True)

    sent = bad = 0
    last_report = time.time()

    while running:
        try:
            data, _ = sock.recvfrom(1024)
        except socket.timeout:
            data = None
        except OSError:
            break

        if data:
            try:
                record = json.loads(data)
                stream = record["stream"]
            except (ValueError, KeyError):
                bad += 1
            else:
                client.publish(f"{args.prefix}/{stream}", data, qos=0)
                sent += 1

        now = time.time()
        if now - last_report > 60:
            rate = sent / (now - last_report)
            print(f"[frame-clock] {sent} records ({rate:.1f}/s), {bad} unparsed", flush=True)
            sent = bad = 0
            last_report = now

    client.loop_stop()
    client.disconnect()
    sock.close()
    print("[frame-clock] stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
