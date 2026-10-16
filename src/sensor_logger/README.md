# sensor_logger

> **Who this is for:** someone reading or changing this package's code.
> **Read first:** [docs/sensor-logging.md](../../docs/sensor-logging.md) for the recording workflow and the categories.
> **What's in it:** the module layout, where each rule lives, and how to test it. Subscribe-only: this package publishes to no topic and cannot move the car.

Records `/sensors/imu/raw`, `/scan` and `/odom` to CSV, one folder per recording under `<workspace>/data_logs/<category>/<time>/`. Afterwards it checks each recording against the maneuver its category claims.

## Highlights

- **All the logic runs without ROS.** File formats, labelling and checks are plain Python. The suite runs with nothing sourced: 168 tests, every one collected.
- **Mutation-tested.** 54 deliberate breakages were each caught by at least one test: flipped comparisons, negated signs, constants off by 10 %, swapped arguments, deleted checks.
- **Read-only, enforced by a test.** `test_node_static.py` fails if a publisher appears, or if the config and the node disagree on any parameter.
- **Limit: the ROS node itself has no automated test here.** `rclpy` doesn't import on the dev machine this was written on. See [Testing on the car](#testing-on-the-car).

## Use

Full walkthrough, terminal by terminal: [docs/sensor-logging.md](../../docs/sensor-logging.md#recording-a-test-vector). In short, with bringup and teleop already running:

**Terminal 3**: record, then Ctrl-C; then check.

```bash
ros2 launch sensor_logger sensor_logger_launch.py category:=turn_left notes:='carpet'
ros2 run sensor_logger summarize_session data_logs/turn_left/<time>      # --json for scripts
```

**Working when:** the launch prints the session folder and climbing `imu`/`scan`/`odom` counts. The summary prints `turn_left: PASS`.

`summarize_session` exits 0 on PASS (or for the unchecked `free_drive`), 1 on FAIL, and 2 if the folder isn't a readable session.

## What's in here

| File | Role |
|---|---|
| `sensor_logger/maneuvers.py` | The test-vector catalogue (`CATEGORIES`), per-sample labelling (`label_series`, `Thresholds`) and the per-category check (`check_session`). No rclpy. |
| `sensor_logger/session.py` | Folder layout, workspace discovery, the CSV writers (`CsvLog`, `ScanLog`), the dead-IMU check (`ImuStats`). No rclpy. |
| `sensor_logger/summarize_session.py` | The post-run CLI: writes `labels.csv` and `summary.json`. No rclpy. |
| `sensor_logger/sensor_logger_node.py` | The ROS node. Converts messages to numbers and hands them to `session.py`. Nothing else. |
| `launch/sensor_logger_launch.py` | `category`, `notes`, `log_root` launch arguments, forced to strings. |
| `config/sensor_logger.yaml` | Topics, IMU units, status period. |

**Adding a category:** add an entry to `CATEGORIES` in `maneuvers.py`, add a matching closed-form vector to `VECTORS` in `test/test_maneuvers.py`, and add it to the launch argument's description. `test_every_checked_category_has_a_vector` and `test_launch_help_lists_exactly_the_catalogue` fail until they agree. Then add it to the table in `docs/sensor-logging.md`.

## Tests

```bash
python3 -m pytest src/sensor_logger/test/ -v      # no sourcing, no build
```

**Working when:** it ends with `168 passed`. Compare that with `grep -c '^def test' src/sensor_logger/test/*.py`. There are fewer functions than tests, because many are parametrized.

The test vectors in `test_maneuvers.py` are closed-form speed and curvature profiles (curvature is 1 ÷ turn radius). Every expected label follows from `Thresholds`, not from copying the code's output.

The central test is `test_each_vector_passes_its_own_category_and_no_other`. A folder name is only trustworthy if a left turn *fails* every other category.

## Testing on the car

The node can't be exercised on a machine without a working `rclpy`, so check it once on the Jetson, following the workspace's usual order:

1. **Static, nothing else running:** `ros2 launch sensor_logger sensor_logger_launch.py category:=idle`. It should print the session folder, then `imu 0  scan 0 … odom 0` every 5 s. Ctrl-C should leave `metadata.json` containing `ended_at` and `counts`, and `"source": "unknown"` (nothing was publishing).
2. **Bringup only, car parked:** the counts should climb, and you should see the all-zero IMU warning. `summarize_session` on that folder should report `idle: PASS` and `source    car (urg_node, vesc_driver_node, vesc_to_odom_node)`. Under `racerbot_sim` it should say `source    sim (gym_bridge_node)`.
3. **Driving:** one recording per category, per [docs/sensor-logging.md](../../docs/sensor-logging.md).
