# Camera Phase Relay Test

## Overview

- SBC side (`camera_phase_relay_sbc.py`):
  - Runs camera detection with the same logic path as `main.py` (`library/detect_corn.py` -> `detector.detect_cone()`).
  - Includes phase4-6 control loop and the same motor/LED behavior as `main.py` (camera searching -> approaching -> goal).
  - Reads BNO055 and GPS in parallel and relays them.
  - Adds camera debug overlay (bbox, centroid, phase, GOAL sign).
  - Sends telemetry and compressed camera frames to PC.
- PC side (`camera_phase_monitor_pc.py`):
  - Receives relay packets.
  - Shows realtime camera video.
  - Plots realtime BNO055 and GPS graphs.

## Resource split

- SBC:
  - Sensor I/O
  - Cone detection
  - JPEG compression only (320x240, low quality)
- PC:
  - Decode/visualize video
  - Realtime graph rendering
  - UI/event loop

## Run

1. Start PC monitor:

```bash
python tests/camera_phase_monitor_pc.py --host 0.0.0.0 --port 5001 --history-sec 30
```

1. Start SBC relay (replace with your PC IP):

```bash
python tests/camera_phase_relay_sbc.py --pc-host 192.168.1.10 --pc-port 5001 --tx-hz 10 --video-every 2 --jpeg-quality 55 --start-phase 4
```

## Notes

- If ROI image is missing, default red-color detection fallback is used (same as `main.py`).
- `--exit-on-goal` を付けると、phase6到達時に停止します（付けなければGOAL表示を維持して配信継続）。
- Reduce SBC load further:
  - Increase `--video-every` (send frames less often).
  - Lower `--jpeg-quality`.
  - Lower `--tx-hz`.
- Required packages (both machines as needed): `opencv-python`, `numpy`, `matplotlib`, `pynmea2`, `pyserial`.
