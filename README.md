# Pedestrian Detection & Collision Warning (CARLA + YOLOv8)

A real-time pipeline that detects pedestrians using YOLOv8, estimates
their distance using CARLA's depth camera, computes Time-To-Collision
(TTC), and triggers an Autonomous Emergency Braking (AEB) response when
TTC drops below a safety threshold — mirroring the structure of Euro
NCAP AEB pedestrian test evaluations.

## Pipeline Overview

```
CARLA World (synchronous mode)
      │
      ├── Ego Vehicle ── RGB Camera ────┐
      │                  Depth Camera ──┤
      │                                  ▼
      │                          Frame Sync (sensors.py)
      │                                  │
      │                    ┌─────────────┴──────────────┐
      │                    ▼                             ▼
      │            YOLOv8 Detection              Depth Map (meters)
      │           (detector.py)                 (sensors.py)
      │                    │                             │
      │                    └──────────┬──────────────────┘
      │                               ▼
      │                    Distance Estimation (using depth info from camera)
      │                    (distance_estimation.py)
      │                               │
      │                               ▼
      │                    TTC Computation (ttc.py)
      │                               │
      │                               ▼
      │                    AEB Decision (controller.py)
      │                               │
      └───────────── apply_control() ◄┘
                               │
                               ▼
                    Visualization (visualizer.py)
```

## Project Structure

| File                      | Purpose                                                            |
|---------------------------|---------------------------------------------------------------------|
| `config.py`               | All tunable parameters (thresholds, sensor settings, colors, etc.) |
| `spawn_actors.py`         | Connects to CARLA, enables sync mode, spawns ego vehicle, cameras, and pedestrians |
| `sensors.py`              | Synchronizes RGB + depth frames per tick, decodes depth to meters   |
| `detector.py`             | YOLOv8 wrapper, filters detections to the "person" class            |
| `distance_estimation.py`  | Samples the depth map per bounding box, smooths distance over time (EMA) and does simple frame-to-frame association |
| `ttc.py`                  | Computes closing speed and Time-To-Collision per tracked pedestrian |
| `controller.py`           | AEB decision logic (safe / warning / brake) and vehicle control     |
| `visualizer.py`           | Draws bounding boxes, distance/TTC labels, and HUD                  |
| `main.py`                 | Live demo loop with an OpenCV display window                       |
| `save_vid.py`             | Headless-friendly version that records the annotated run to MP4    |

## Setup

1. Start the CARLA server (e.g. `./CarlaUE4.sh` or `CarlaUE4.exe`).
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
   Note: the `carla` PyPI package version must match your CARLA server
   version (this scaffold assumes 0.9.15 — adjust `requirements.txt` if
   your server differs).
3. (Optional) Download a YOLOv8 weights file — `yolov8n.pt` will be
   auto-downloaded by `ultralytics` on first run if not present.

## Running

**Live demo with display window:**
```bash
python main.py
```
Press `q` in the OpenCV window (or `Ctrl+C`) to stop.

"""
main.py
========
Main synchronous simulation loop:
  1. Connect to CARLA, spawn ego vehicle + cameras + pedestrians.
  2. Each tick:
       a. world.tick() to advance the simulation
       b. Retrieve the matching RGB + depth frames
       c. Run YOLOv8 pedestrian detection on the RGB frame
       d. Estimate distance for each detection from the depth map
       e. Smooth distances + associate detections across frames (tracking)
       f. Compute TTC for each tracked pedestrian
       g. Decide AEB action (safe / warning / brake) and apply control
       h. Draw overlays, show the frame, and save it to video
  3. Clean up all actors and restore world settings on exit.
Press 'q' in the OpenCV window (or Ctrl+C in the terminal) to stop.
"""

## How It Works

### 1. Synchronous Mode
CARLA's `world.tick()` advances the simulation by exactly
`FIXED_DELTA_SECONDS`. We retrieve the RGB and depth frame that share
the same `frame_id` so the depth map exactly matches what the YOLO
detector sees — this is essential for accurate distance estimates.

### 2. Depth Decoding
CARLA's depth camera encodes distance into the BGRA buffer:
```
normalized = (R + G*256 + B*256^2) / (256^3 - 1)
depth_meters = normalized * 1000.0
```

### 3. Distance Estimation
For each YOLO bounding box, we sample a small patch of the depth map
around the box center and take the median (`DEPTH_SAMPLE_MODE =
"median_box"`), which is more robust than a single pixel against edge
noise and partial occlusion.

### 4. Smoothing & Lightweight Tracking
Distances are smoothed with an exponential moving average
(`DISTANCE_EMA_ALPHA`). Detections are associated frame-to-frame by
nearest bounding-box center (a simple greedy matcher) so each pedestrian
keeps a stable `track_id` and distance history. For production use,
swap this for ByteTrack/DeepSORT.

### 5. TTC Computation
```
closing_speed = (distance_prev - distance_curr) / dt
TTC = distance / closing_speed   (if closing_speed > MIN_CLOSING_SPEED)
TTC = inf                         (otherwise — not approaching)
```

### 6. AEB Decision
| Condition                       | Action                          |
|----------------------------------|----------------------------------|
| `TTC <= TTC_BRAKE_THRESHOLD`     | Full emergency brake             |
| `TTC <= TTC_WARNING_THRESHOLD`   | Cautionary slowdown + warning HUD|
| Otherwise                        | Maintain target cruise speed     |

Default thresholds (`config.py`) follow Euro NCAP AEB pedestrian test
ranges: warning at 2.0s TTC, braking at 1.2s TTC.

## Visualization

- **Green** box: safe (TTC above warning threshold)
- **Yellow** box: warning zone
- **Red** box: braking zone
- Each box is labeled with track ID, estimated distance (m), and TTC (s)

## Results
![Town01 Demo](output/sim.gif)
