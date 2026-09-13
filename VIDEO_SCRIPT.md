# VIDEO_SCRIPT.md – UEH CRC 2026 Simulation Round

**Language**: English  
**Target length**: 6 minutes  
**Required sequence**: as specified in the competition guidelines

---

## 0:00 – 0:30 | Introduction

**[Participant speaks directly to camera]**

> "Hello, my name is [NAME].
> I am a student at [SCHOOL], [FACULTY].
> This is my submission for the UEH Creative Robot Contest 2026, Simulation Round.
> In this video I will explain my autonomous driving solution, demonstrate it live,
> and show the effect of changing a control parameter on the fly."

---

## 0:30 – 2:00 | Architecture and Key Code

**[Screen share: VS Code or file browser + terminal]**

### Architecture overview (0:30 – 1:00)

> "The solution uses a modular, layered architecture with six ROS 2 nodes."

**[Show diagram or quickly open run.launch.py]**

> "Starting from the raw sensor data:
> - `/camera/image_raw` feeds three perception nodes: lane detection, sign detection, and traffic light detection.
> - `/scan` LiDAR feeds the safety node and also helps with pedestrian detection.
> - All perception nodes publish on intermediate topics.
> - A central behavior_node subscribes to all of them and publishes `/cmd_vel`."

### Lane node (1:00 – 1:20)

> "The most critical component is `lane_node.py`. It crops the bottom 45% of the image as a Region of Interest. It then applies CLAHE histogram equalisation — this is the key technique that makes lane following work in the dark tunnel."

**[Show lane_node.py lines ~60-90 — _white_mask function]**

> "Adaptive thresholding finds bright road markings regardless of global brightness. It also applies an HSV white mask for daylight scenes. The centroids of the left and right halves give us the lane centre, and the error drives a PD steering controller."

### Behavior FSM (1:20 – 2:00)

> "The behavior_node implements a strict priority hierarchy. The highest priority is an emergency LiDAR stop. Then pedestrian stop, traffic light stop, STOP sign sequence. Normal lane following has the lowest priority — it only runs when nothing else needs attention."

**[Show behavior_node.py lines around _fsm_step — the priority ordering]**

> "The STOP sign FSM has four sub-states: detected, brake, hold for 2 seconds, and a cooldown period so the robot doesn't stop at the same sign again."

---

## 2:00 – 4:00 | Live Simulation Run

**[Start or have already started: terminal 1 has simulator, terminal 2 starts solution]**

```bash
# Terminal 2 (run this now if not already running)
ros2 launch ueh_solution run.launch.py
```

**[Watch the Gazebo window as robot drives. Narrate what is happening:]**

> "The robot starts at the bottom-left corner and heads right along the bottom straight."

*(When STOP sign detected):*  
> "You can see in the terminal: 'STOP_DETECTED', 'STOP_HOLD'. The robot makes a complete stop for over 2 seconds, then resumes."

*(When traffic light approached):*  
> "The light_node detected RED — you can see 'RED_LIGHT_STOP' in the log. The robot is waiting at the junction."

*(When entering tunnel):*  
> "The robot enters the dark tunnel. Notice it continues following the lane — that's the CLAHE equalisation keeping the white markings detectable even in near-total darkness."

*(When pedestrian appears):*  
> "The pedestrian node detected a purple figure in the crossing lane. 'PEDESTRIAN_STOP'. The robot waits until the crossing is clear."

---

## 4:00 – 5:00 | Live Parameter Change and Rerun

**[Terminal 3 — with solution running]**

> "I will now change the `base_speed` parameter live, without restarting any node."

```bash
# Show current value
ros2 param get /behavior_node base_speed

# Increase to a visibly faster speed
ros2 param set /behavior_node base_speed 0.22
```

> "Notice the robot is now moving faster along the straight sections."

**[Observe Gazebo window — robot visibly faster]**

> "Let me reduce it to a more conservative value and observe the difference."

```bash
ros2 param set /behavior_node base_speed 0.13
```

> "The robot is now slower and more stable on curves — a lower speed gives more time for the PD controller to correct errors."

> "This demonstrates that `base_speed` directly and immediately affects driving behaviour. A higher speed requires tighter PD gains to maintain lane accuracy; a lower speed is more robust but takes longer to complete the track."

---

## 5:00 – 6:00 | Limitations and Future Improvements

**[Participant speaks to camera]**

> "My current solution has several known limitations I want to be transparent about."

**Limitations:**

1. **Traffic light detection reliability** — The temporal filter helps, but if the robot is far from the light and the emissive sphere subtends fewer than 6 pixels, the detector may miss it. Graded timing variance could catch this.

2. **Overtaking is not implemented** — The parked robot is handled by the LiDAR safety layer, which will slow down and stop. A full lane-change overtaking manoeuvre is code-disabled (`overtaking_enabled: false`) because it destabilised lane following during testing.

3. **Tunnel corner handling** — The arc section of the tunnel with 8 curve segments can cause the robot to clip the inner wall if the steering gain is set too high. More testing is needed with different random seeds.

4. **STOP sign false positives** — In bright outdoor scenes, saturated red road markings or other red objects could trigger the STOP detector. The white-interior check helps but is not perfect.

**Future improvements:**

1. Implement the overtaking FSM with LiDAR-guided lane change.
2. Use a sliding-window confidence filter on the traffic light circle to handle flickering.
3. Add a low-pass filter on the PD output to reduce oscillation on long straights.
4. Train a lightweight CNN on rendered sign images for more robust sign classification.

> "Thank you for watching."

---

## Notes for Recording

- Use OBS or `ffmpeg` to record the screen and microphone simultaneously
- Show Gazebo window and the relevant terminal side-by-side
- The parameter change in section 4:00–5:00 must be performed LIVE, not edited in post
- Target total length: 6 minutes ± 30 seconds
