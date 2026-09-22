# Lorton search and assess run

This run uses the PX4Sim front door with a v2 search aircraft and a v3 assess aircraft.

```bash
cd /home/user/px4-sim-stack
UAS_FLEET='chimera_v2 chimera_v3' SCENE=lorton SCENARIO=lorton_roi_feature_test COMPOSE_PROFILES=sim,offboard UAS_BASE=10 ./px4sim restart
./px4sim place
./px4sim fly 11 20
./px4sim fly 12 20
```

Upload `missions/lorton-search-v2.plan` to uas11 and `missions/lorton-assess-v3.plan` to uas12 through QGroundControl. The v2 plan has a zero-second delay immediately before the lawnmower legs, a -70 degree gimbal command, 20 m altitude, 1 m/s mission speed, and wide camera selection. The v3 plan uses `NAV_DELAY=-1` as its stationary vantage marker.

PX4Sim has a repeatable upload helper, which also sets the vehicle DDS domain and pushes directly through MAVROS without changing Chimera mission behavior:

```bash
./px4sim mission upload 11 missions/lorton-search-v2.plan
./px4sim mission upload 12 missions/lorton-assess-v3.plan
./px4sim px4 11 commander mode auto:mission
./px4sim px4 12 commander mode auto:mission
```

The v2 mission publishes stationary target reports. Ground tracking merges accepted stationary fixes by the configured horizontal distance (`dedup_horizontal_m`), forwards the resulting ROI array, and the v3 mission transforms and queues it. If no ROI report arrives, the known casualty-location topic populates the same queue once. The v3 vehicle stays at its -1-second vantage while the queue is drained.

Verify with `./px4sim state`, the mission cursor, gimbal status, and the ROI topic before recording. Record the verified run with:

```bash
./px4sim record bags
```

Stop recording with `./px4sim record stop`; the bag is under `logs/` and should be copied to the run archive with both plan files.

During the 2026-09-22 attempt, plan upload passed (8/8 and 4/4), but execution was not accepted as verified: the search companion's `img_processing` process exited because its configured ReID model was absent, so continuous detection and mosaic did not start and no stationary ROI was forwarded to the assess vehicle. Do not record or claim a successful mission until `img_processing` is healthy and logs show static-target publication followed by assess ROI/gimbal activity.
