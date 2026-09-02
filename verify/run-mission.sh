#!/usr/bin/env bash
# Run verify/component/mission.py inside a vehicle's companion, the way px4sim
# runs its other component tools: copied in rather than mounted, on the
# vehicle's ROS domain, over UDP-only DDS.
#
#   ./verify/run-mission.sh plan /logs/mission_test.plan --home=LAT,LON --casualties='E,N;E,N'
#   ./verify/run-mission.sh scenario a --push service
#
# The mission node needs mission.plan_file set for the `service` push path:
# ONBOARD_PARAMS_FILE=/logs/mission_test_params.yaml in .env, which the entry
# point loads last. See logs/onboard/mission_test_params.yaml.
set -euo pipefail
cd /home/user/px4-sim-stack
container=$(docker ps -q --filter "name=onboard11")
[ -n "$container" ] || { echo "onboard11 not running" >&2; exit 1; }
docker cp verify/component/mission.py "$container:/tmp/mission.py" >/dev/null
docker cp verify/component/udp-only.xml "$container:/tmp/udp-only.xml" >/dev/null
exec docker exec -e ROS_DOMAIN_ID=71 \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/udp-only.xml "$container" bash -lc '
  set -e
  . /opt/ros/$ROS_DISTRO/setup.bash
  . /home/user/ros2_ws/install/setup.bash
  exec python3 /tmp/mission.py "$@"' _ "$@"
