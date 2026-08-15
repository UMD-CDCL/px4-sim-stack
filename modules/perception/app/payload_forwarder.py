#!/usr/bin/env python3
"""Forward DeepStream payload files to MQTT.

Why this exists
---------------
DeepStream 8.0 ships an MQTT protocol adapter, and it does not work against
Mosquitto 2. The adapter creates its client with mosquitto_new(), which
defaults to MQTT 3.1.1, and then publishes with mosquitto_publish_v5(). The
broker answers with a protocol error and closes the connection:

    perception: mqtt connection success; ready to send data
    perception: failed to send the message. err(1)
    perception: Error sending repeat publish: The client is not currently connected
    mosquitto:  Client px4simstack-perception disconnected: protocol error.

The pipeline then stops, so the failure takes the whole detector down. Setting
new-api=1 does not help, because the render path stays the same.

So this stack does not use that adapter. nvmsgconv writes each payload to
debug-payload-dir, and this program publishes the files and deletes them. The
MQTT topic and the payload format are exactly what the adapter would have
produced, so nothing downstream can tell the difference.

Delete this program when a DeepStream release fixes the adapter, and set
enable=1 on the msgbroker sink in the config.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from pathlib import Path

import paho.mqtt.client as mqtt

running = True


def stop(*_):
    global running
    running = False


def make_client(host: str, port: int, client_id: str) -> mqtt.Client:
    if hasattr(mqtt, "CallbackAPIVersion"):
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    else:
        client = mqtt.Client(client_id=client_id)
    # The broker disconnects a client when another connects with the same id,
    # and publishes into the dead session vanish without an error. These two
    # lines make that fight visible in the logs.
    client.on_connect = lambda *_: print(f"[forwarder] connected as {client_id}", flush=True)
    client.on_disconnect = lambda *_: print(f"[forwarder] disconnected as {client_id}", flush=True)
    client.connect_async(host, port, keepalive=30)
    client.loop_start()
    return client


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", default=os.environ.get("PAYLOAD_DIR", "/tmp/ds-payloads"))
    ap.add_argument("--name", default="",
                    help="unique client id suffix; defaults to the basename of --dir")
    ap.add_argument("--host", default=os.environ.get("MQTT_HOST", "message-bus"))
    ap.add_argument("--port", type=int, default=int(os.environ.get("MQTT_PORT", "1883")))
    ap.add_argument("--topic", default=os.environ.get("MQTT_TOPIC", "perception/detections"))
    ap.add_argument("--poll", type=float, default=0.1, help="seconds between directory scans")
    ap.add_argument("--max-age", type=float, default=5.0,
                    help="drop a payload older than this, in seconds")
    args = ap.parse_args()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    directory = Path(args.dir)
    directory.mkdir(parents=True, exist_ok=True)

    name = args.name or directory.name
    client = make_client(args.host, args.port, f"px4simstack-forwarder-{name}")
    print(f"[forwarder] {directory} -> mqtt://{args.host}:{args.port}/{args.topic}", flush=True)

    sent = dropped = 0
    last_report = time.time()

    while running:
        # Sort by write time, not name: nvmsgconv's file names are not
        # promised to sort chronologically, and at full frame rate a pass
        # regularly holds more than one payload.
        def mtime(p):
            try:
                return p.stat().st_mtime_ns
            except OSError:
                return 0
        files = sorted(directory.glob("*"), key=mtime)
        consumed = 0
        for path in files:
            if not running:
                break
            try:
                # A payload only matters while it is fresh. If the broker or
                # this loop falls behind, throw the backlog away rather than
                # publish a stale detection to a planner.
                if time.time() - path.stat().st_mtime > args.max_age:
                    path.unlink(missing_ok=True)
                    dropped += 1
                    consumed += 1
                    continue
                data = path.read_bytes()
                path.unlink(missing_ok=True)
            except FileNotFoundError:
                continue
            except OSError as exc:
                print(f"[forwarder] cannot read {path}: {exc}", file=sys.stderr, flush=True)
                continue

            consumed += 1
            if data.strip():
                client.publish(args.topic, data, qos=0)
                sent += 1

        # Sleep whenever the pass consumed nothing. Sleeping only on an empty
        # directory lets one unreadable file turn this loop into a busy spin.
        if consumed == 0:
            time.sleep(args.poll)

        if time.time() - last_report > 60:
            print(f"[forwarder] {sent} published, {dropped} dropped as stale", flush=True)
            sent = dropped = 0
            last_report = time.time()

    client.loop_stop()
    client.disconnect()
    print("[forwarder] stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
