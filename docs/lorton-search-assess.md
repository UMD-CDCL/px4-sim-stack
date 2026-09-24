# Lorton search and assess run

This run uses UAS13 (the simulated UAS3 v2 template) as search and UAS11 (the
simulated UAS1 v3 template) as assess. UAS12 and UAS14 remain available with
the UAS2 and UAS4 templates.

For a known-state run, use the PX4Sim restart front door. It rebuilds the selected
containers, recreates the network, and starts the fleet from a clean PX4 state:

```bash
./px4sim restart
./px4sim place
```

```bash
cd /home/user/px4-sim-stack
SCENE=lorton SCENARIO=lorton_casualties COMPOSE_PROFILES=sim,offboard UAS_BASE=10 ./px4sim restart
./px4sim place
./px4sim fly 13 20
./px4sim fly 11 30
```

Take both vehicles airborne with `./px4sim uas <N> takeoff <altitude>` before uploading the plans. The plans begin with a hold waypoint rather than another takeoff command, so PX4 does not re-execute a second takeoff while entering mission mode. Upload `missions/lorton-search-v2.plan` to uas13 and `missions/lorton-assess-v3.plan` to uas11 through PX4Sim. The v2 plan has a zero-second delay immediately before the lawnmower legs, a -70 degree gimbal command, 20 m altitude, 1 m/s mission speed, and wide camera selection. The v3 plan uses 30 m for its stationary vantage marker and `NAV_DELAY=-1` to remain there.

PX4Sim has a repeatable upload helper, which also sets the vehicle DDS domain and pushes directly through MAVROS without changing Chimera mission behavior:

```bash
./px4sim mission upload 13 missions/lorton-search-v2.plan
./px4sim mission upload 11 missions/lorton-assess-v3.plan
./px4sim px4 13 commander mode auto:mission
./px4sim px4 11 commander mode auto:mission
```

The v2 mission publishes stationary target reports. Ground tracking merges accepted stationary fixes by the configured horizontal distance (`dedup_horizontal_m`), forwards the resulting ROI array, and the v3 mission transforms and queues it. If no ROI report arrives, the known casualty-location topic populates the same queue once. The v3 vehicle stays at its -1-second vantage while the queue is drained. Each plan ends with an unlimited `DO_JUMP` back to the waypoint immediately preceding its delay marker, so the search and assessment loops continue instead of landing.

Verify with `./px4sim state`, the mission cursor, gimbal status, and the ROI topic before recording. Record the verified run with:

```bash
./px4sim record bags
```

Stop recording with `./px4sim record stop`; the bag is under `logs/` and should be copied to the run archive with both plan files.

The standard `lorton_casualties` scenario is the source of truth for the scene and casualty locations. Do not substitute the feature-test scenario for a mission run.
