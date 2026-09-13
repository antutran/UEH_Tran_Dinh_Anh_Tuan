# Analysis Scripts

Two Python scripts to generate publication-ready figures from recorded runs.

## Requirements

```bash
pip install matplotlib pandas numpy
```

## Usage

### 1. Single-run analysis

```bash
python3 analysis/plot_run.py /tmp/crc_logs/run_20260913_120000.csv
```

Generates five figures alongside the CSV:
- `*_speed.png`      – commanded linear and angular velocity vs time
- `*_lane_error.png` – lane error (pixels) vs time
- `*_lidar.png`      – front LiDAR distance vs time
- `*_events.png`     – STOP sign / traffic light / pedestrian event timeline
- `*_path.png`       – x-y robot path from odometry

Also prints a summary table:
```
=== Run Summary ===
  Duration:         142.3 s
  Distance:          8.47 m
  Mean cmd_v:        0.164 m/s
  Lane error mean:  18.3 px
  Lane error max:   94.1 px
  STOP detections:  47 frames
  Min front dist:    0.28 m
```

### 2. Multi-run comparison (for the video demo)

```bash
python3 analysis/plot_steering.py \
    /tmp/crc_logs/run_slow.csv \
    /tmp/crc_logs/run_fast.csv
```

Generates in `analysis/`:
- `steering_comparison.png` – overlaid lane errors
- `speed_comparison.png`    – overlaid speeds
- `error_histogram.png`     – distribution of |error| per run

## Data source

All figures are generated from **real recorded data only**.
The data_logger node writes `/tmp/crc_logs/run_<timestamp>.csv` during every
solution run. No figures are fabricated.
