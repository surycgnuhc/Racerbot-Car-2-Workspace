# Sensor logging (`sensor_logger`)

> **Who this is for:** anyone who wants recorded IMU and LiDAR data from real maneuvers: turning, pulling away, stopping.
> **Read first:** [operations.md](operations.md#manual-driving-teleop). You drive the maneuvers yourself, by hand.
> **You'll be able to:** record a labelled test vector, check that it really contains the maneuver you meant, and find the files afterwards.
> **Time:** about 10 minutes for your first recording.

`sensor_logger` writes three of the car's sensors to plain CSV files. They are the IMU (accelerometer and gyro), the [LiDAR](glossary.md#lidar) (the laser that measures distance to walls), and wheel [odometry](glossary.md#odometry--odom) (speed and position worked out from the wheels).

Each recording is filed under the maneuver you said you were driving, such as `turn_left` or `stop`. Afterwards a checker tells you whether the recording actually contains that maneuver.

## Highlights

- **One folder per recording, named by maneuver.** For example, `data_logs/turn_left/20261016-141500/`. There are eight categories, and a typo in the name refuses to start rather than creating a stray folder.
- **Every recording is checked against its label.** `summarize_session` labels every odometry sample (idle, accelerating, braking, straight, turn_left, turn_right, reverse). It then fails a `turn_left` folder that is really straight-line driving.
- **Spots a dead IMU while you record.** It warns within 5 seconds if every IMU value is exactly zero. That is the documented state of this car's VESC IMU ([hardware-reference.md](hardware-reference.md#vesc-motor--steering-controller)).
- **Survives being switched off mid-run.** Every file is written one line at a time. A half-written last line is detected and skipped rather than read as a wrong number.
- **Plain CSV.** It opens in a spreadsheet, `pandas` or `numpy` with no ROS install.
- **Read-only by construction.** It subscribes to three topics and publishes to none, so it cannot move the car. A test fails if a publisher is ever added.
- **Limit: the labels come from odometry, not the IMU.** The IMU reads zero on this car, so "was it turning?" is answered from wheel speed and *commanded* steering. Wheelspin or a skid will not show up.

**Why it exists:** turning, starting and stopping data is what you need to tune a controller, fit a vehicle model or test an IMU once one works. Without labels, a folder of recordings soon becomes "which of these was the left turn?". With labels nobody checks, a mislabelled file poisons whatever is trained or tested on it later.

## What this actually is

Three things happen when you record:

1. You start the **driving stack** as usual (bringup + teleop), so you can drive the car with the gamepad.
2. You start **`sensor_logger`** in a third terminal, telling it which maneuver you are about to drive. It writes everything it hears to a new folder.
3. You **drive the maneuver**, press Ctrl-C in the logger's terminal, then run **`summarize_session`** on the folder.

The logger is a [node](glossary.md#node) (one running program that does one job). It only *listens* to [topics](glossary.md#topic) (named channels that nodes send messages over). Driving is still entirely yours: you hold LB and use the sticks, exactly as in manual driving.

### What gets recorded

| File | From topic | One row per | Columns |
|---|---|---|---|
| `imu.csv` | `/sensors/imu/raw` | IMU message (~50 Hz) | `stamp, recv_time, ax, ay, az, gx, gy, gz, qw, qx, qy, qz` |
| `scan.csv` | `/scan` | LiDAR sweep | `stamp, recv_time, angle_min, angle_increment, range_min, range_max, r0 … r1080` |
| `odom.csv` | `/odom` | odometry message | `stamp, recv_time, x, y, yaw, speed, yaw_rate` |
| `metadata.json` | — | — | category, your notes, the driving protocol, `source` (real car or simulator), units, row counts, IMU verdict |
| `labels.csv` | written by `summarize_session` | odometry sample | `stamp, label` |
| `summary.json` | written by `summarize_session` | — | pass/fail, seconds per label, IMU and LiDAR health |

`stamp` is the time inside the message, in seconds. `recv_time` is the computer's clock when the logger received it.

**Real car or simulator?** `metadata.json` has a `source` field. The logger fills it in by checking, once a second, which nodes (running ROS programs) publish the topics it records:

- `car`: only `urg_node`, `sick_node`, `vesc_to_odom_node` or `vesc_driver_node`, the real drivers.
- `sim`: only `gym_bridge_node`, from `racerbot_sim`.
- `mixed`: more than one kind during the session, for example a `ros2 bag play` on top of the live car. Don't trust that recording.
- `unknown`: nothing was publishing, or only something it doesn't recognise.

`source_publishers` lists the node names it saw, per topic.

**The IMU columns are not in the units `sensor_msgs/Imu` promises.** The VESC driver passes the VESC's own units through unchanged. Acceleration is in **g** (1.0 = gravity), and rotation rate is in **degrees per second**. They are not m/s² and rad/s. `metadata.json` records this under `units`. The evidence is in the deep dive below.

### The categories (test vectors)

Pick one per recording with `category:=<name>`. The protocol is what you drive. The check is what `summarize_session` then requires.

| Category | Protocol: what you drive | Check: what the data must show |
|---|---|---|
| `idle` | Car on the floor, bringup running, **don't hold LB**. 10 s. | Idle for ≥ 95 % of the recording |
| `start` | Stopped ≥ 1 s, then accelerate straight to ~1 m/s and hold it a second | Idle for the first 0.5 s, ≥ 0.3 s accelerating, reaches ≥ 0.5 m/s |
| `stop` | Straight at ~1 m/s, release the throttle to a full stop, stay stopped ≥ 1 s | ≥ 0.3 s braking, idle for the last 0.5 s, reached ≥ 0.5 m/s |
| `straight` | ~1 m/s, wheel centred, ≥ 3 s | ≥ 2 s straight, ≤ 0.5 s of either turn |
| `turn_left` | ~1 m/s, steering held left, ≥ 3 s | ≥ 2 s turn_left, ≤ 0.5 s turn_right |
| `turn_right` | ~1 m/s, steering held right, ≥ 3 s | ≥ 2 s turn_right, ≤ 0.5 s turn_left |
| `reverse` | Back up slowly and straight, ≥ 2 s | ≥ 1 s reverse, reaches ≥ 0.3 m/s |
| `free_drive` | Anything | Nothing: labelled, never failed. This is the default. |

Full lock on this car is a circle about 1.35 m in radius ([hardware-reference.md](hardware-reference.md#physical-dimensions-used-in-config)). A turn recording needs room for that.

## Before you start

- [ ] The workspace is built, including the new package: `colcon build --symlink-install --packages-select sensor_logger`
- [ ] You can already drive the car by hand ([operations.md](operations.md#manual-driving-teleop))
- [ ] Open floor, at least 3 m × 3 m for the turn recordings

## Recording a test vector

**Run these in order.** Every terminal starts with the two `source` lines from [concepts.md](concepts.md).

**Terminal 1**: the hardware. Leave it running.

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

**Working when:** the log settles and keeps running, with no `process has died`.

**Terminal 2**: manual control. Leave it running.

```bash
ros2 launch f1tenth_stack teleop_launch.py
```

**Working when:** holding LB and pushing the left stick turns the wheels. Lift the car first if you want to check this safely.

**Terminal 3**: the logger. Start it *just before* you drive. Pick the category for what you are about to do.

```bash
ros2 launch sensor_logger sensor_logger_launch.py category:=turn_left notes:='carpet, full battery'
```

**Working when:** it prints `recording category "turn_left" to …/data_logs/turn_left/<time>`, the protocol for that category, and then a line every 5 s that looks like this:

```
imu 250  scan 200 (rejected 0)  odom 250
```

All three numbers should climb. A number stuck at 0 means that topic isn't arriving, usually because terminal 1 isn't running.

**Expect this warning.** On this car it is correct, not a fault in the logger:

```
every IMU sample so far is exactly 0.0 -- the VESC IMU is off
```

**If it doesn't start:** `unknown category 'turnleft'; expected one of: …` means a typo. Use one of the names in the table above.

**Now drive the protocol.** Hold LB and drive the maneuver. Then press **Ctrl-C in terminal 3**. Leave the car stopped while you do.

**Terminal 3 again**: check the recording, using the folder it printed.

```bash
ros2 run sensor_logger summarize_session data_logs/turn_left/20261016-141500
```

**Working when:** you see something like this:

```
data_logs/turn_left/20261016-141500
  category  turn_left: PASS
  labels    turn_left 3.4s, accelerating 0.6s, idle 0.5s  (peak 1.08 m/s over 4.6s)
  imu       all_zero, 230 samples, mean |accel| 0.0 g
  scan      184 scans x 1081 beams, 40.0 Hz, finite 0.97
```

**If it says FAIL:** the lines underneath say exactly what was missing, for example `turn_left: 1.20s, needs at least 2.00s`. Drive it again and hold the maneuver longer. Delete the failed folder, or keep it and note that it failed. Nothing reads it automatically.

## Where the data goes

```
<workspace>/data_logs/
├── README.md              (tracked in git)
├── turn_left/
│   ├── 20261016-141500/   one recording
│   └── 20261016-141812/
└── stop/
    └── …
```

**Recordings are not committed to git.** `.gitignore` excludes everything in `data_logs/` except its README. A LiDAR sweep is about 7 kB per row, which is roughly 17 MB per minute. To share a recording, copy the folder.

To write somewhere else, pass `log_root:=/some/folder`. `<category>/<time>/` is still created underneath it.

## Using the data

The CSVs need nothing from ROS:

```python
import pandas as pd
run = 'data_logs/turn_left/20261016-141500'
odom = pd.read_csv(f'{run}/odom.csv')
labels = pd.read_csv(f'{run}/labels.csv')      # after summarize_session
scan = pd.read_csv(f'{run}/scan.csv')          # columns r0..r1080 are the beams
beams = scan.filter(regex=r'^r\d+$').to_numpy()  # shape: (sweeps, 1081)
```

Beam `i` points at angle `angle_min + i * angle_increment`, in radians, where 0 is straight ahead and positive is to the left. Ranges are metres. `inf` means nothing was hit.

<details>
<summary><b>Deep dive: how each sample gets its label</b>, and why these thresholds. Safe to skip unless a label surprises you or you are changing <code>Thresholds</code>.</summary>

`label_series()` in `src/sensor_logger/sensor_logger/maneuvers.py` labels each `/odom` sample. The checks run in this order, and the first match wins:

1. Speed or yaw rate is NaN or infinite → `invalid`.
2. Speed ≤ −0.10 m/s → `reverse`.
3. Speed changed by ≤ −0.5 m/s² over the last 0.3 s → `braking`.
4. Speed changed by ≥ +0.5 m/s² over the last 0.3 s → `accelerating`.
5. |speed| < 0.10 m/s → `idle`.
6. Path curvature (yaw rate ÷ speed) ≥ 0.25 1/m → `turn_left`, or ≤ −0.25 → `turn_right`.
7. Otherwise → `straight`.

Why these numbers:

- **0.10 m/s for "stopped".** `vesc_to_odom` rounds any speed under 0.05 m/s to exactly 0 (`vesc_to_odom.cpp`). Anything under 0.10 is that rounding, or creep.
- **Acceleration over 0.3 s, not one step.** Differencing two odometry messages 20 ms apart is mostly noise. Until half a window (0.15 s) of history exists, acceleration is treated as unknown and the sample is labelled from speed alone.
- **Curvature, not yaw rate.** Curvature doesn't change with speed: full lock is 0.74 1/m at any speed. A yaw-rate threshold tuned at 1 m/s would miss a full-lock turn at 0.5 m/s. 0.25 1/m is a 4 m radius, about 5° of steering on the 0.36 m wheelbase.
- **Left is positive.** ROS ([REP-103](https://www.ros.org/reps/rep-0103.html)) puts z up, so positive yaw rate is counter-clockwise, which is a left turn when driving forwards.

**A limit worth knowing.** `/odom`'s yaw rate is not measured. `vesc_to_odom` computes it as `speed × tan(commanded steering) / wheelbase`.

So `turn_left` means "drove forwards with the wheel turned left", not "the car actually rotated". A car skidding straight on with the wheel turned is still labelled a turn. A working gyro would fix this ([localization.md](localization.md)).

The thresholds live in the `Thresholds` dataclass, and every `summary.json` records the values it used. Each one has a boundary test in `test/test_maneuvers.py`, and each test was seen to fail with the threshold changed by 10 %.

</details>

<details>
<summary><b>Deep dive: why the IMU units are g and deg/s</b>. Skip unless you are about to use <code>imu.csv</code>.</summary>

`sensor_msgs/Imu` specifies m/s² and rad/s. The VESC driver doesn't convert to them:

- `src/f1tenth_system/vesc/vesc_driver/src/vesc_packet.cpp`: `VescPacketImu::acc_x()` returns `acc_x_` commented `// g/s`, and `gyr_x()` is commented `// deg/s`. These are the VESC firmware's own units.
- `src/f1tenth_system/vesc/vesc_driver/src/vesc_driver.cpp`: copies those values straight into the `sensor_msgs/Imu` it publishes on `/sensors/imu/raw`, with no scaling.

So when the IMU is enabled, a car at rest should read about **1.0** on one accelerometer axis, not 9.81. This has not been measured on this car, because its IMU currently reports zeros.

`summarize_session` handles either case. A mean |accel| near 1.0 with `imu_accel_unit: g` is `ok`. A mean near 9.81 is reported as `unit_mismatch`: the source has changed units, so update `config/sensor_logger.yaml`.

This also means the re-check procedure in [hardware-reference.md](hardware-reference.md#vesc-motor--steering-controller) ("roughly 9.81 on one axis at rest") probably expects the wrong number. `TODO(verify):` confirm on a car with the IMU enabled.

</details>

## Safety

**The logger cannot move the car, but you still drive it.** Every maneuver here is manual driving, with the same rules: LB held, open space, someone ready to let go.

The logger only subscribes. `test/test_node_static.py` fails if a publisher, or any of `/drive`, `/ackermann_cmd` or `/commands/`, ever appears in the package.

If you ever add a publisher to it, it becomes driving code and needs the full LB deadman contract ([writing-your-own-node.md](writing-your-own-node.md)).

**Mind the stick scale when recording.** Full left stick is 5 m/s in `joy_teleop.yaml`. The protocols ask for about 1 m/s, which is a light push.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `could not find the workspace root above the installed sensor_logger package` | Package installed somewhere without `src/` and `install/` side by side | Pass `log_root:=<folder>` |
| `imu 0` stays at 0 | VESC driver not running | Start terminal 1; check `ros2 topic hz /sensors/imu/raw` |
| `scan … (rejected N)` with N > 0 | A sweep arrived with a different beam count from the first | Check `urg_node`'s config; the rejected sweeps are counted, not written |
| `imu  unit_mismatch` | The IMU source publishes m/s², not g | Set `imu_accel_unit` in `config/sensor_logger.yaml` |
| `odom.csv: timestamps must be non-decreasing` (exit 2) | Two clocks mixed, e.g. a sim and the real car on one graph | Re-record on one graph |
| A start/stop recording FAILs on `not idle for the first 0.50s` | The logger started while the car was already moving | Start the logger first, wait a second, then drive |

## See also

- [run-diagnostics.md](run-diagnostics.md): a full rosbag of a run plus pipeline health. Use it to debug an autonomy run rather than to collect labelled maneuvers.
- [src/sensor_logger/README.md](../src/sensor_logger/README.md): the code layout and how to run the tests.
- [hardware-reference.md](hardware-reference.md): the IMU's state, and the car's measured geometry.
