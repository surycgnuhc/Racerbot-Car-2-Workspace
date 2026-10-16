# data_logs

> **Who this is for:** anyone looking for recorded sensor data.
> **Read first:** [docs/sensor-logging.md](../docs/sensor-logging.md) for how recordings get here.
> **What's in it:** one folder per maneuver category, one timestamped folder per recording.

This is where `sensor_logger` writes recordings:

```
data_logs/<category>/<YYYYmmdd-HHMMSS>/
    metadata.json   what was recorded, by whom, in which units
    imu.csv         /sensors/imu/raw
    scan.csv        /scan, one column per beam
    odom.csv        /odom
    labels.csv      per-sample labels (after summarize_session)
    summary.json    PASS/FAIL against the category (after summarize_session)
```

**Nothing in here except this README is committed to git.** A minute of LiDAR (laser distance sweeps) is roughly 17 MB, so to share a recording, copy its folder.
