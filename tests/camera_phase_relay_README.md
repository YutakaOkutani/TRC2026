# Camera Phase Relay Test

## Overview

- SBC side (`camera_phase_relay_sbc.py`):
  - Calls `cansat_mission/phases/phase4.py`, `phase5.py`, `phase6.py` handlers directly for phase behavior.
  - Uses shared managers (`SensorManager`, `MotorManager`, `LedManager`) instead of duplicating phase logic.
  - Keeps only SBC-specific realtime debug relay (overlay + telemetry transport).
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
python camera_phase_monitor_pc.py --host 0.0.0.0 --port 5001 --history-sec 30
```

1. Start SBC relay (replace with your PC IP):

```bash
python camera_phase_relay_sbc.py --pc-host 100.100.219.60 --pc-port 5001 --tx-hz 10 --video-every 2 --jpeg-quality 55 --start-phase 4
```

## Notes

- If ROI image is missing, default red-color detection fallback is used (same as `main.py`).
- `--exit-on-goal` を付けると、phase6到達時に停止します（付けなければGOAL表示を維持して配信継続）。
- Reduce SBC load further:
  - Increase `--video-every` (send frames less often).
  - Lower `--jpeg-quality`.
  - Lower `--tx-hz`.
- Required packages (both machines as needed): `opencv-python`, `numpy`, `matplotlib`, `pynmea2`, `pyserial`.
