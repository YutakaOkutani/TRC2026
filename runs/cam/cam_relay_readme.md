# Camera Phase Relay Test

## Overview

- SBC side (`cam_relay_sbc.py`):
  - Calls `csmn/phs/p4.py`, `p5.py`, `p6.py` handlers directly for phase behavior.
  - Uses shared managers (`sns_mgr.py`, `mtr_mgr.py`, `led_mgr.py`) instead of duplicating phase logic.
  - Keeps only SBC-specific realtime debug relay (overlay + telemetry transport).
  - Adds camera debug overlay (bbox, centroid, phase, GOAL sign).
  - Sends telemetry and compressed camera frames to PC.
- PC side (`cam_relay_pc.py`):
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
python3 cam_relay_pc.py
```

1. Start SBC relay:

```bash
python3 cam_relay_sbc.py
```

## Notes

- If ROI image is missing, default red-color detection fallback is used (same as `main.py`).
- Runtime defaults (host/port/tx rate/JPEG quality/start phase) are embedded in each script and can be changed in the constants near the top of each file.
- `--exit-on-goal` を付けると、p6到達時に停止します（付けなければGOAL表示を維持して配信継続）。
- Reduce SBC load further:
  - Increase `--video-every` (send frames less often).
  - Lower `--jpeg-quality`.
  - Lower `--tx-hz`.
- Required packages (both machines as needed): `opencv-python`, `numpy`, `matplotlib`, `pynmea2`, `pyserial`.
