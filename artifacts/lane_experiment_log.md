# Lane camera-only experiment log

## Test 1 — baseline

- Change: none; commit `852f489`; `lane_node.roi_top_frac=0.55`.
- Exact test: clean Docker simulator, `ros2 launch ueh_solution run.launch.py test_mode:=lane_camera_only`, 60 onboard-camera frames at 0.5 s intervals, and 10 Hz solution CSV telemetry.
- Straight result: robot left START but drifted to the right of the marked lane within about 5 s; it then drove nearly straight across unmarked/off-road surface.
- First curve result: failed. The curve/edge was detected only around 40 s, after the robot was already displaced.
- Second curve result: not reached on-road.
- Failure location: immediately after START/intersection; camera markings moved fully to the left side of the image.
- Lane error: near zero for long periods despite the road being visibly left; at about 40–42 s it jumped from approximately -33 px to -94 px.
- Angular z: followed lane error with the expected sign, from about -0.15 to -0.40 rad/s during the late detection; otherwise stayed near zero.
- Camera observation: the raw view showed road markings entirely left by about 5 s. The mask falsely treated horizon/building edges in the right half as a right lane boundary.
- Conclusion: perception failure (Case A) plus late response (Case C). Controller sign and response are not the primary fault.

## Test 2 — lower ROI

- Change: `lane_node.roi_top_frac` 0.55 → 0.60 only.
- Straight result: failed within the first 15 s.
- First curve result: not reached.
- Lane error: oscillated between about -108 px and +145 px, then froze at +157 px after markings were lost.
- Angular z: tracked the erroneous signal and settled near +0.66 rad/s with speed reduced to 0.10 m/s.
- Camera observation: the robot initially recovered toward the visible lane, but the lower ROI then lost usable markings.
- Conclusion: removing horizon clutter helped expose the single-line geometry, but indefinite previous-error hold caused a runaway turn. ROI-only tuning is insufficient.

## Test 3 — connected-component lane pixels

- Change: restore `roi_top_frac=0.55`; replace all-white-pixel centroid with selection of a compact component reaching the lower ROI.
- Straight result: failed within the first 15 s after a corrective turn.
- First curve result: not reached.
- Failure location: near the roadside STOP/traffic-light poles.
- Lane error: component selection produced a strong recovery command, then line loss froze it at exactly +119.51 px.
- Angular z: remained at +0.50 rad/s while error was frozen; speed was 0.10 m/s.
- Camera observation: markings moved back into view, but a close pole occluded much of the camera after the sustained turn.
- Conclusion: component selection prevents the false horizon centroid, but indefinite stale-error hold is independently unsafe.

## Test 4 — decay error on line loss

- Change: multiply the previous error by 0.85 on frames with no trusted lane component instead of holding it indefinitely.
- Straight result: stale error now decayed to approximately zero in about 2 s, but the robot still oversteered through the wide START/intersection geometry.
- First curve result: not reached on the intended lane.
- Lane error: no longer froze; it decayed from +90 px through +32, +6, +1 and approximately 0 px. Later single-line estimates repeatedly reached roughly -60 to -94 px.
- Angular z: followed the decaying error back to zero, confirming the controller and decay worked as intended.
- Camera observation: a close roadside pole crossed the view after the initial excessive correction; later the robot entered a broad intersection from the wrong heading.
- Conclusion: line-loss decay is retained, but fixed-width single-line geometry over-corrects nearby lines.

## Test 5 — perspective-aware single-line offset

- Change: scale the assumed half-lane width from 0.25 to 0.90 half-images according to the selected component's vertical depth.
- Straight result: failed; the robot turned across the broad intersection (`odom y` reached about +3.9 m).
- First curve result: not reached on the intended lane.
- Lane error: highly noisy, repeatedly changing sign and ranging from about -69 px to +138 px.
- Angular z: correctly tracked those changes, with peaks around +0.58 rad/s.
- Camera/root cause: a long diagonal marking crossed the image midpoint and was split into separate left/right masks, so one physical line was treated as two boundaries.

## Test 6 — classify full marking components

- Change: connected-component labeling now occurs on the full ROI before left/right classification.
- Straight result: improved consistency, but still oversteered left; after about 20 s odometry was approximately (3.98, +0.71) m relative to START.
- First curve result: not yet reached on the intended centreline.
- Lane error: coherent components were found, but corrections remained too large (roughly -167 to +98 px in the observed interval).
- Angular z: reached approximately -0.70 rad/s at the largest error.
- Conclusion: full-component classification is retained; the PD gains are too aggressive for its larger, cleaner error signal.

## Test 7 — retune PD for component error

- Change: `kp_steer` 0.0042 → 0.0012 and `kd_steer` 0.0008 → 0.0002.
- Straight result: failed; lower gains reduced angular commands but the robot still crossed the intersection, reaching odometry approximately (3.49, +3.13) m.
- First curve result: not reached on the intended lane.
- Lane error: remained extremely noisy, approximately -222 to +201 px.
- Angular z: reduced to about ±0.27 rad/s, proving gain changes reduced response but did not fix the perception signal.
- Conclusion: discard the component-selection branch and restore baseline gains.

## Test 8 — require dark-road support

- Change: restore baseline centroid and PD gains; retain verified no-line decay; reject thresholded pixels that are not below nearby dark road.
- Straight result: clear improvement. At about 39 s the robot remained on the long approach at odometry approximately (7.05, -0.18) m with mostly single-digit error.
- First curve result: failed. It passed the tunnel approach and continued off the board; later odometry was approximately (13.30, -0.86) m.
- Lane error: stable on the straight, but a shallow false right edge kept curve-approach error near zero; after markings disappeared, the verified decay safely returned error to zero.
- Camera observation: true markings occupied the left side and extended 8–14% down the ROI; the false right edge remained confined to the top 4–6%.
- Conclusion: dark-road support is retained. A depth-consistency test is needed to reject the shallow false boundary.

## Test 9 — reject shallow horizon boundary

- Change: reject a right candidate shallower than 6% of ROI when the left marking is deeper than 13%; cap this preview correction at 35 px. Reject a lone equally shallow right candidate.
- Straight result: failed by oversteering left; odometry reached approximately (3.37, +3.48) m.
- First curve result: not reached on the intended lane.
- Lane error: shallow-edge rejection worked, but exposed the fixed-width single-line fallback, producing +65 to +117 px corrections early in the run.
- Angular z: reached about +0.48 rad/s, then reversed as the robot crossed unrelated markings.
- Conclusion: retain shallow false-edge rejection, but single-line lane width must vary with perspective.

## Test 10 — perspective-aware single-line geometry

- Change: single-line half-width now grows from 0.50 to 1.00 half-images with vertical depth; clamp single-line error to ±60 px.
- Straight result: pass. The robot stayed near the approach centreline and reached odometry approximately (4.94, -0.19) m at 30 s.
- First curve result: pass. It shifted left toward the tunnel, entered the first straight tunnel, negotiated the joining quarter-curve, and emerged onto the vertical road near odom x=9.1 m.
- Early second curve/tunnel result: pass. At 65–70 s odometry progressed from approximately (9.14, 3.54) to (9.31, 4.40) m while the camera showed the vertical-road exit.
- Later failure: missed the following top U-turn around 75–80 s and continued beyond the board edge.
- Lane error/angular z: bounded single-line commands reached ±60 px / approximately ±0.25 rad/s; straight segments returned to near zero.
- Conclusion: immediate milestone achieved—camera-only lane following passed the early major tunnel curves. More turning margin is needed for the next U-turn.

## Test 11 — lower cruise speed

- Change: `base_speed` 0.18 → 0.12 m/s only.
- Straight result: initially stable, reaching odometry approximately (5.82, +0.14) m at 50 s.
- First curve result: failed before the tunnel. The robot settled into a tight right-hand loop near odom x=6.1 m.
- Lane error/angular z: error remained near -100 px and commanded about -0.42 rad/s while speed modulation reduced motion to roughly 0.07 m/s.
- Conclusion: reduced cruise speed is a regression because it makes transient false corrections too tight. Revert to the simulation-verified 0.18 m/s Test 10 state.
