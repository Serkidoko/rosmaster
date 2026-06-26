# Robot Calibration Notes

Last updated: 2026-06-25

This file records the currently stable chassis calibration and the rules future
AI/code changes should follow. The robot's real behavior is the source of truth:
if a log looks questionable but the user says the run is stable, keep that setup
unless a new real 0.5 m test proves otherwise.

## Current Stable Setup

Workspace:

```bash
/home/pi/chay/new
```

Straight movement calibration is separate from lateral movement. Do not mix them.

Current straight calibration:

```python
MOTOR_CALIBRATION = (1.043, 1.000, 1.010, 1.006)
```

Current stable lateral calibration:

```python
LATERAL_LEFT_MOTOR_CALIBRATION = (1.250, 0.930, 1.220, 0.900)
LATERAL_RIGHT_MOTOR_CALIBRATION = (1.250, 0.930, 1.220, 0.900)
LATERAL_LEFT_PROFILE = (-1, 1, 1, -1)
LATERAL_RIGHT_PROFILE = (1, -1, -1, 1)
LATERAL_BASE_MOTOR_COMMAND = 36
LATERAL_REFERENCE_SPEED_MPS = 0.16
LATERAL_WHEEL_SYNC_KP = 160.0
LATERAL_WHEEL_SYNC_KI = 8.0
LATERAL_WHEEL_SYNC_MAX_CORRECTION = 10.0
LATERAL_YAW_KP = 4.0
LATERAL_YAW_MAX_CORRECTION = 12.0
LATERAL_ABORT_YAW_ERROR_DEG = 10.0
MAX_LATERAL_SEGMENT_M = 0.10
LATERAL_SEGMENT_PAUSE_S = 0.15
```

Lateral movement is intentionally segmented by distance, not fixed to the 0.5 m
test command. For any lateral target, the robot should run at most 0.10 m, stop,
check/realign yaw if needed, then continue with the remaining distance. The last
segment can be shorter than 0.10 m.

On 2026-06-25, a 0.20 m segment test reached high yaw errors and right strafe
hit the abort threshold, so keep 0.10 m as the safer realign interval. Prefer
avoiding long lateral moves in the mission route; use lateral motion mainly for
short visual alignment or small correction moves.

## Important Logs

Best real-world stable run according to the user:

```bash
/home/pi/chay/lateral_left_050_segmented_cal2_20260624_184803.log
```

This run used the old shared lateral calibration, now copied into both
direction-specific values:

```python
LATERAL_LEFT_MOTOR_CALIBRATION = (1.250, 0.930, 1.220, 0.900)
LATERAL_RIGHT_MOTOR_CALIBRATION = (1.250, 0.930, 1.220, 0.900)
```

Useful earlier comparison log:

```bash
/home/pi/chay/lateral_left_050_segmented_20260624_184723.log
```

Bad experiment, do not restore:

```bash
/home/pi/chay/lateral_left_050_segmented_yawsign_20260624_184908.log
```

That run inverted lateral yaw correction and caused yaw runaway plus M2/M4 nearly
stopping. Keep the current yaw correction sign in `robot_driver.py`:

```python
signed_yaw = (1 if i in (0, 1) else -1) * yaw_correction
```

## Test Commands

Always test lateral calibration with at least 0.5 m. Short 0.04-0.05 m tests are
not enough to judge drift.

Dry run:

```bash
cd /home/pi/chay/new
python3 test_chassis_motion.py --direction left --distance-m 0.5 --debug
```

Real 0.5 m left test with log:

```bash
cd /home/pi/chay/new
ts=$(date +%Y%m%d_%H%M%S)
python3 test_chassis_motion.py --run --direction left --distance-m 0.5 --debug 2>&1 \
  | tee /home/pi/chay/lateral_left_050_${ts}.log
```

Run right strafe separately before assuming right is calibrated:

```bash
cd /home/pi/chay/new
ts=$(date +%Y%m%d_%H%M%S)
python3 test_chassis_motion.py --run --direction right --distance-m 0.5 --debug 2>&1 \
  | tee /home/pi/chay/lateral_right_050_${ts}.log
```

Direction-specific lateral wheel calibration:

```bash
cd /home/pi/chay/new
python3 calibrate_lateral_wheels.py --run --direction left --speed 32 --duration 0.8
python3 calibrate_lateral_wheels.py --run --direction right --speed 32 --duration 0.8
```

The calibration script writes a timestamped log automatically in `/home/pi/chay`
when `--run` is used, for example `lateral_calib_left_YYYYMMDD_HHMMSS.log`.
By default it sends equal raw command magnitude to all four wheels using the
lateral profile, like `set_motor(-32, 32, 32, -32)` for left. This avoids a bad
old lateral calibration hiding a weak wheel. Use `--use-current-calibration`
only for a deliberate iterative refinement after the raw identity test looks
healthy.

Apply the suggested value only to the matching config variable:

```python
LATERAL_LEFT_MOTOR_CALIBRATION = (...)
LATERAL_RIGHT_MOTOR_CALIBRATION = (...)
```

If the calibration script warns about command saturation, lower `--speed`
before trusting the suggestion.

Invalid calibration result, do not apply:

```text
2026-06-25 pasted run with old current-calibration command mode:
left  command=(-40.0, 29.8, 39.0, -28.8) abs_delta=[768, 35, 807, 3]
right command=(40.0, -29.8, -39.0, 28.8) abs_delta=[754, 12, 653, 12]

2026-06-25 identity-command auto-log run:
/home/pi/chay/lateral_calib_left_20260625_163249.log
left  command=(-32.0, 32.0, 32.0, -32.0) abs_delta=[285, 157, 425, 20]
/home/pi/chay/lateral_calib_right_20260625_163303.log
right command=(32.0, -32.0, -32.0, 32.0) abs_delta=[285, 120, 301, 16]
```

The first pair under-drove M2/M4 and produced absurd suggested multipliers such
as `10.715`, `120.975`, `27.726`, and `26.831`. The second pair used equal raw
commands and shows M4 still barely moving. Do not copy any values from these
runs into `config.py`.

Compile check:

```bash
cd /home/pi/chay/new
python3 -m py_compile config.py robot_driver.py test_chassis_motion.py
```

## Pick 1 Sequence Test

Successful route test from `home` to `pick_1`:

```bash
/home/pi/chay/home_to_pick1_20260624_185833.log
```

The configured route is:

```python
("home", "pick_1"): (
    ("forward", 1.8),
)
```

False-positive pick_1 live test, do not treat as successful:

```bash
/home/pi/chay/pick1_sequence_live_red_fullroi_20260624_190001.log
/home/pi/chay/pick1_sequence_live_red_fullroi_20260624_190001.jpg
/home/pi/chay/arm_stage_pick1_red_fullroi_20260624_190001.log
```

Command used for that false-positive run:

```bash
cd /home/pi/chay/new
python3 pick1_sequence_node.py --run \
  --gripper-config /home/pi/chay/pick1_gripper_box.json \
  --color red \
  --roi 0 0 1 0.8 \
  --debug \
  --save-debug /home/pi/chay/pick1_sequence_live_red_fullroi_20260624_190001.jpg \
  --arm-log /home/pi/chay/arm_stage_pick1_red_fullroi_20260624_190001.log
```

Important: the default `pick_1` ROI is left-side only. In that test, the object
started on the right side of the camera image, so the default ROI could not
detect it. The run used full-width ROI `0 0 1 0.8`, but the robot still did not
physically pick the object.

The old gripper config stores `"color": "yellow"`, but the false-positive test
object was red/pink. Override with `--color red` when using the red object.

Do not start camera detection while the arm is still moving. On 2026-06-24 the
arm command used `run_time_ms=10000`, but the code only waited `arm_settle_s=1s`,
so the camera could detect while the arm was rotating. `arm_stage_calibrator.py`
now waits for `run_time_ms / 1000 + settle_s` in real hardware mode before the
next FSM step.

On 2026-06-25, `visual_approach_node.py`, `pick1_sequence_node.py`, and
`home_to_pick1_pick_object1.py` gained a slot-scan layer before bbox approach:
detect `red/green/yellow`, choose the detection nearest the gripper target box
as the current slot, and move `arm-right` with `side_speed_mps > 0` if that slot
is the wrong target color. This has compile/dry-run/offline-image coverage only;
hardware scan across three physical objects is not yet confirmed.

Later on 2026-06-25, a red live test stopped early while still looking at the
yellow/green object because `wrong_slot_count` was based on color changes and
could count glare/flicker as multiple objects. The scan logic now counts a wrong
slot only after `--slot-stable-frames` consecutive frames and only after the
previous counted slot has left the gripper target area.

## Rules For Future AI Coding

1. Keep all active robot code in `/home/pi/chay/new`.
2. Do not edit straight movement calibration when tuning lateral movement.
   Tune `LATERAL_LEFT_MOTOR_CALIBRATION` and
   `LATERAL_RIGHT_MOTOR_CALIBRATION` separately instead.
3. Do not collapse lateral movement back into one long 0.5 m command.
4. Do not invert the lateral yaw correction sign again.
5. Do not trust raw encoder balancing alone; compare against real robot behavior.
6. Preserve user-proven values unless a new real 0.5 m log clearly improves them.
7. When changing calibration, record the exact values, command, and log path here.
8. Avoid changing unrelated mission, vision, or arm code while tuning chassis motion.

## Practical Diagnosis Notes

- If the robot moves too slowly or does not start, check motor command saturation
  and battery voltage before changing calibration.
- If lateral drift appears only on long movement, keep segmentation and tune per
  segment. Do not rely on 4 cm tests.
- If one wheel appears bad in combined lateral motion, test it individually before
  assuming motor hardware failure. M4 previously passed a single-motor test in
  `/home/pi/chay/single_motor_m4_20260624_183650.log`.
- If yaw exceeds `LATERAL_ABORT_YAW_ERROR_DEG`, stop and inspect the log instead
  of continuing to force the run.
