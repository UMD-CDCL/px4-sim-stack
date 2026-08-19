#!/usr/bin/env bash
# Wait for a container to say one thing or another, and never for a dead one.
#
#   await.sh <container> <deadline-s> <success-regex> [failure-regex] [since]
#
# Exits 0 on the success pattern, 1 on the failure pattern, 2 on the deadline
# and 3 if the container stopped. A wait that only watches for success spends
# its whole deadline staring at something that already died, which is the
# slowest way there is to learn nothing.
set -uo pipefail

container=${1:?name a container}
deadline=${2:?give a deadline in seconds}
success=${3:?give a success pattern}
failure=${4:-}
since=${5:-5m}
period=${AWAIT_PERIOD_S:-2}

waited=0
while :; do
	log=$(docker logs "$container" --since "$since" 2>&1) || { echo "no such container: $container" >&2; exit 3; }
	if hit=$(printf '%s' "$log" | grep -aiE "$success" | tail -1) && [ -n "$hit" ]; then
		printf '%s\n' "$hit"
		exit 0
	fi
	if [ -n "$failure" ] && hit=$(printf '%s' "$log" | grep -aiE "$failure" | tail -1) && [ -n "$hit" ]; then
		printf '%s\n' "$hit" >&2
		exit 1
	fi
	if [ "$(docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null)" != true ]; then
		echo "$container stopped after ${waited}s" >&2
		printf '%s\n' "$log" | tail -5 >&2
		exit 3
	fi
	if [ "$waited" -ge "$deadline" ]; then
		echo "$container said neither after ${deadline}s" >&2
		exit 2
	fi
	sleep "$period"
	waited=$((waited + period))
done
