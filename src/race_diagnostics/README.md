# race_diagnostics

> **Who this is for:** someone reading or changing this package's code.
> **Read first:** [docs/run-diagnostics.md](../../docs/run-diagnostics.md) for the recording and analysis workflow.
> **What's in it:** the recorder and analyzer internals. Read-only — this package publishes to no topic and cannot move the car.

Read-only run recorder and post-run analyzer for this car's driving stack.

**This package only ever subscribes.** It publishes to no topic and cannot
influence how the car drives, which puts it in the support/tooling
category rather than driving code (see
[docs/adding-your-own-code.md](../../docs/adding-your-own-code.md)) — the
same category as `web_dashboard`. It is safe to leave running alongside
anything, including a race. If it ever gains a publisher it becomes
driving code and inherits the LB deadman policy in full.

Full workflow, rationale, and the AI-agent prompt template:
**[docs/run-diagnostics.md](../../docs/run-diagnostics.md)**

## Use

```bash
# Terminal 1 -- start FIRST so nothing is missed
ros2 launch race_diagnostics record_run.py

# Terminal 2 -- the driving stack, with tee (see the path it printed)
ros2 launch racerbot_launch auto_map_race_launch.py 2>&1 | tee <dir>/launch.log

# afterwards
ros2 run race_diagnostics summarize_run <dir>

# your own questions, answered in a few lines (one-time pip install first)
ros2 run race_diagnostics bag_stats <dir>
ros2 run race_diagnostics bag_find <dir> /scan "min(ranges) < 0.4" --min-duration 0.05
```

The `bag_*` commands, their one-time setup and the expression syntax are in
[docs/run-diagnostics.md#asking-questions-about-a-recording](../../docs/run-diagnostics.md#asking-questions-about-a-recording).

## What's in here

| File | Role |
|---|---|
| `race_diagnostics/race_diag_node.py` | The ROS probe. Watches every link of the pipeline; records `events.jsonl`. |
| `race_diagnostics/run_events.py` | Pure logic: log classification, lap-gate parsing, run timeline. No rclpy — unit-testable with a bare `pytest`. |
| `race_diagnostics/filter_log.py` | Follows a tee'd launch log, prints only the lines that matter. |
| `race_diagnostics/summarize_run.py` | Turns a run directory into one page of findings (`--json` for agents). |
| `race_diagnostics/bag_tools.py` | The `bag_stats`, `bag_window`, `bag_extract` and `bag_find` commands: questions about a rosbag, answered in a few lines. Not a ROS node. |
| `race_diagnostics/bag_query.py` | Pure logic for those: topic timing, episodes, bucketing, compacting a message. No rclpy, no mcap. |
| `race_diagnostics/field_expr.py` | The safe expression language (`min(ranges) < 0.4`). Walks a whitelisted `ast`; never `eval()`. |
| `race_diagnostics/bag_io.py` | Opens MCAP recordings (split files merged) with the pure-Python `mcap` library. The only file that touches the recording. |
| `test/fake_recording.py` | Writes a noisy 60 s fake recording with five known events, for the `bag_*` tests. |
| `launch/record_run.py` | Probe + rosbag into one timestamped directory. |
| `config/race_diagnostics.yaml` | Topics, reporting cadence, pose-lag alert threshold. |

## Tests

`run_events.py` and the `bag_*` logic are deliberately free of ROS
dependencies, so no build and no [sourcing](../../docs/glossary.md#sourcing)
(loading ROS into the terminal) are needed. The `bag_*` tests do
need the two `mcap` libraries installed first:

```bash
pip install --user --break-system-packages mcap==1.6.0 mcap-ros2-support==0.6.0   # once
python3 -m pytest src/race_diagnostics/test/ -v     # no sourcing, no build
```

Without those libraries, collection fails on `test_bag_tools.py` and the
error names the install command. That failure is deliberate: a skipped test
would let the suite look green while testing nothing.

The test fixtures are real log lines from the 2026-07-27 session, so the
classifier is verified against output the car actually produced rather
than output we imagined it would.

## The one number to look at

**Worst pose lag.** Localization staleness is invisible in a topic's
message rate — `auto_map_race_node` republishes SLAM's transform at a
fixed 40 Hz whatever its age, so a frozen transform arrives just as
punctually as a live one. Only the header stamp reveals it, and a stale
pose is what put the car into a wall. Under 0.15 s is healthy; over 0.5 s
and `pure_pursuit` will hard-stop through the stalls.
