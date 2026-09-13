# AI Usage Statement – UEH CRC 2026

## Summary

This file documents AI involvement in the development of the competition solution,
as required by the contest rules.

---

## Files Created by AI

| File | Status |
|---|---|
| `src/ueh_solution/package.xml` | AI-created (scaffold) |
| `src/ueh_solution/setup.py` | AI-created (scaffold) |
| `src/ueh_solution/setup.cfg` | AI-created (scaffold) |
| `src/ueh_solution/config/params.yaml` | AI-created; **tuning by participant** |
| `src/ueh_solution/ueh_solution/lane_node.py` | AI-created; **tested and tuned by participant** |
| `src/ueh_solution/ueh_solution/lidar_node.py` | AI-created; **tested by participant** |
| `src/ueh_solution/ueh_solution/sign_node.py` | AI-created; **tested by participant** |
| `src/ueh_solution/ueh_solution/light_node.py` | AI-created; **tested by participant** |
| `src/ueh_solution/ueh_solution/pedestrian_detect_node.py` | AI-created; **tested by participant** |
| `src/ueh_solution/ueh_solution/behavior_node.py` | AI-created; **tested and tuned by participant** |
| `src/ueh_solution/ueh_solution/data_logger.py` | AI-created |
| `src/ueh_solution/launch/run.launch.py` | AI-created |
| `analysis/plot_run.py` | AI-created |
| `analysis/plot_steering.py` | AI-created |
| `README.md` | AI-created (template); **completed by participant** |
| `VIDEO_SCRIPT.md` | AI-created (template); **adapted by participant** |
| `AI_USAGE.md` | AI-created; **updated by participant** |

---

## Files Not Modified by AI

| File | Notes |
|---|---|
| `src/crc_sim/**` | Simulator package — protected, not touched |

---

## Design Suggestions from AI

1. **Modular perception/behavior split** — Each perception concern (lane, sign, light, pedestrian, LiDAR) is in its own ROS node, communicating via topics. The behavior_node implements a single priority-ordered FSM. This architecture was suggested by AI to improve testability and comply with the competition rule that the code should be modular.

2. **CLAHE + adaptive threshold** for lane detection — AI suggested combining CLAHE histogram equalisation with adaptive thresholding to handle the tunnel's near-zero illumination without sacrificing daylight performance.

3. **Temporal filter on traffic light detection** — AI suggested requiring N consecutive frames before committing to a new light state, to avoid false positives from STOP signs and highway signs.

4. **AND logic for pedestrian detection** — AI suggested requiring both LiDAR (lateral sector) AND camera (purple blob) to agree before asserting "pedestrian blocking", reducing false positives from tunnel walls.

5. **Separate cooldown state for STOP signs** — AI designed the STOP FSM with an explicit cooldown period to prevent the robot from repeatedly stopping at the same sign on approach/departure.

---

## Changes Made After Testing

*(Participant to fill in after actual simulation runs)*

- [ ] Tuned `kp_steer` and `kd_steer` based on observed oscillation
- [ ] Adjusted `roi_top_frac` after observing ROI in debug images
- [ ] Tuned `stop_brake_dist` after observing actual stop behaviour
- [ ] Adjusted `ped_lidar_sector_lo/hi` after observing false positives
- [ ] Confirmed `light_confirm_frames` value after observing light timing

---

## Decisions Made by the Participant

- Selection of `base_speed` as the video demo parameter (safe and observable)
- Final tuning of all numeric parameters
- Verification that no forbidden topics are used (reviewed all subscriptions)
- Recording of all test runs used in the report
- Preparation and delivery of the video

---

## AI Usage Statement (for REPORT.pdf)

> This solution was developed with the assistance of an AI coding assistant
> (Google Antigravity / Gemini). The AI contributed the initial code structure,
> algorithm selection, and implementation for all nodes in the `ueh_solution`
> package. The participant then built and tested the solution in the Gazebo
> simulation, tuned all control and perception parameters based on observed
> behaviour, recorded the experimental data used in this report, and prepared
> the video submission. No simulation results in this report were fabricated;
> all figures were generated from real recorded CSV log files.
