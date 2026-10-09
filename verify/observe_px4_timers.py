#!/usr/bin/env python3
"""Check source timestamps through the px4sim front door during a SITL run.

Listening directly to uORB distinguishes fresh source updates from MAVLink
periodically transmitting a cached message. Run alongside the mission verifier.
"""
import argparse
import json
import re
import subprocess
import time
from pathlib import Path

TOPICS = ['battery_status', 'system_power', 'cpuload', 'gimbal_device_attitude_status']
ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vehicle', type=int, default=11)
    parser.add_argument('--duration', type=float, default=400)
    parser.add_argument('--max-age', type=float, default=1)
    parser.add_argument('--interval', type=float, default=5)
    args = parser.parse_args()
    start = time.monotonic()
    failures = []
    samples = {topic: 0 for topic in TOPICS}
    maximum = {topic: 0.0 for topic in TOPICS}
    while time.monotonic() - start < args.duration:
        for topic in TOPICS:
            result = subprocess.run([str(ROOT / 'px4sim'), 'px4', str(args.vehicle),
                                     'listener', topic], cwd=ROOT, capture_output=True,
                                    text=True, timeout=15)
            match = re.search(r'^\s*timestamp: (\d+) \(([\d.]+) (seconds|us) ago\)',
                              result.stdout, re.MULTILINE)
            row = {'t': round(time.monotonic() - start, 3), 'topic': topic}
            if result.returncode or not match:
                row['error'] = (result.stdout + result.stderr).strip()[:300]
                failures.append(row)
            else:
                age = float(match[2]) * (1e-6 if match[3] == 'us' else 1)
                row.update(timestamp_us=int(match[1]), age_s=age)
                samples[topic] += 1
                maximum[topic] = max(maximum[topic], age)
                if age > args.max_age:
                    failures.append(row)
            print(json.dumps(row), flush=True)
        time.sleep(min(args.interval, max(0, args.duration - (time.monotonic() - start))))
    summary = {'samples': samples, 'max_age_s': maximum, 'failures': failures,
               'passed': not failures and all(samples.values())}
    print('RESULT ' + json.dumps(summary), flush=True)
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
