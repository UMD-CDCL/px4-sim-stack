# Exhaustive feature ledger

This ledger is seeded directly from the 27 entries in
`5g_drone/config/foxglove/chimera_sim.json`. The status is deliberately a
review field. Replace `UNKNOWN` only after tracing the current code and, where
possible, observing the live Foxglove protocol.

| Status | Layout feature | User action or output | Required code/config trace | Stage |
|---|---|---|---|---|
| `UNKNOWN` | `uas11_camera` | Live RGB image and camera info | camera stream, image bridge, layout Image panel | Bench/Sim |
| `UNKNOWN` | `uas11_scene` | 3D scene/model view | scene source, Foxglove 3D panel | Bench/Sim |
| `UNKNOWN` | `uas11_map` | Map and vehicle position | map/localization topics and site config | GPS/Sim |
| `UNKNOWN` | `uas11_click_point` | Click point service | service server and layout CallService config | Bench/GPS |
| `UNKNOWN` | `uas11_click_off` | Disable click mode | service/topic handler | Bench |
| `UNKNOWN` | `uas11_reassert` | Reassert gimbal ownership | gimbal topic/service path | Bench/Sim |
| `UNKNOWN` | `uas11_click_mode` | Inspect click mode messages | publisher/subscriber and schema | Bench |
| `UNKNOWN` | `uas11_scoring_rates` | Plot scoring rates | scoring publisher and plot fields | GPS/Sim |
| `UNKNOWN` | `uas11_position_error` | Plot position error | truth/localization publishers | GPS/Sim |
| `UNKNOWN` | `uas11_click_roi` | Click ROI service | ROI service server and consumer | Bench/Sim |
| `UNKNOWN` | `uas11_release` | Release target/ROI state | release publisher and consumer | Bench/Sim |
| `UNKNOWN` | `uas11_gimbal_pitch` | Command gimbal pitch | gimbal command subscriber and PX4 path | Bench/Sim |
| `UNKNOWN` | `uas11_gimbal_angle` | Command gimbal angle | gimbal command subscriber and MAVROS path | Bench/Sim |
| `UNKNOWN` | `uas11_gimbal_state` | Inspect gimbal state | gimbal state publisher | Bench/Sim |
| `UNKNOWN` | `uas11_gimbal_owner` | Inspect gimbal owner | ownership publisher/state | Bench/Sim |
| `UNKNOWN` | `uas11_zoom_wide` | Select wide zoom | zoom command and SCF4 state | Bench/Sim |
| `UNKNOWN` | `uas11_zoom_mid` | Select mid zoom | zoom command and SCF4 state | Bench/Sim |
| `UNKNOWN` | `uas11_zoom_narrow` | Select narrow zoom | zoom command and SCF4 state | Bench/Sim |
| `UNKNOWN` | `uas11_zoom_preset` | Inspect zoom preset state | preset publisher/state | Bench/Sim |
| `UNKNOWN` | `uas11_mosaic_capture` | Capture mosaic | mosaic service/topic and output | GPS/Sim |
| `UNKNOWN` | `uas11_start_survey` | Start survey | survey service/topic and mission state | GPS/Sim |
| `UNKNOWN` | `uas11_vlm_capture` | Capture VLM input | capture publisher and consumer | Bench/Sim |
| `UNKNOWN` | `uas11_fiducial_capture` | Capture fiducial input | capture publisher and consumer | Bench/GPS |
| `UNKNOWN` | `uas11_advance_mission` | Advance mission | mission command and PX4 state | GPS/Sim |
| `UNKNOWN` | `uas11_raw_roi` | Publish raw ROI | raw ROI publisher and consumer | Bench/Sim |
| `UNKNOWN` | `uas11_detection_on` | Enable detection | detection control path and DeepStream | Bench/Sim |
| `UNKNOWN` | `uas11_detection_off` | Disable detection | detection control path and DeepStream | Bench/Sim |

Add every non-layout function discovered during code tracing below this table.
Each addition must cite the source file and one of the four stage documents.
