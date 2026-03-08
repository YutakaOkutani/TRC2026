import argparse
import csv
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from csmn.const import (
    CONE_PROBABILITY_THRESHOLD_PHASE4,
    CONE_PROBABILITY_THRESHOLD_PHASE5,
)
from csmn.mgr.hw_mgr import HardwareManager
from lib import detect_corn as dc

DEFAULT_OUT_BASE = PROJECT_ROOT / "runs" / "log"
WINDOW_NAME = "Detector Debug (Production detect_corn.py)"


class _RoiLoader(HardwareManager):
    pass


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Run production cone detector on live camera and dump debug artifacts. "
            "This imports the same detector code used in mission runtime."
        )
    )
    parser.add_argument("--phase", type=int, choices=[4, 5], default=4, help="Phase threshold preset (4 or 5).")
    parser.add_argument(
        "--frames",
        type=int,
        default=0,
        help="Number of frames to process. 0 means run until 'q' key.",
    )
    parser.add_argument("--sleep", type=float, default=0.05, help="Sleep seconds between frames.")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Disable GUI window. Artifacts and CSV logs are still saved.",
    )
    parser.add_argument(
        "--save-every",
        type=int,
        default=1,
        help="Save image artifacts every N frames.",
    )
    parser.add_argument(
        "--outdir",
        type=str,
        default="",
        help="Output directory. Default: /home/pi/TRC2026/runs/log/camera_debug_<timestamp>.",
    )
    return parser


def _make_run_outdir(base_dir: Path) -> Path:
    ts = time.strftime("%Y%m%d_%H%M%S")
    subsec = int((time.time() % 1.0) * 1000)
    outdir = base_dir / f"camera_debug_{ts}_{subsec:03d}"
    suffix = 0
    while outdir.exists():
        suffix += 1
        outdir = base_dir / f"camera_debug_{ts}_{subsec:03d}_{suffix:02d}"
    outdir.mkdir(parents=True, exist_ok=False)
    return outdir


def _display_available() -> bool:
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def _overlay(detector, prob_thresh, frame_idx):
    img = detector.input_img
    if img is None:
        return None
    vis = img.copy()
    h, w = vis.shape[:2]

    bbox = getattr(detector, "detected", None)
    if bbox is not None:
        x, y, bw, bh = [int(v) for v in bbox]
        cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 2)

    cent = getattr(detector, "centroids", None)
    if cent is not None:
        cx, cy = int(cent[0]), int(cent[1])
        cv2.circle(vis, (cx, cy), 5, (0, 255, 255), -1)

    prob = float(getattr(detector, "probability", 0.0))
    is_reached = bool(getattr(detector, "is_reached", False))
    method = str(getattr(detector, "debug_method", "unknown"))
    detected = prob >= prob_thresh
    status = f"DET={int(detected)} REACH={int(is_reached)} p={prob:.3f} thr={prob_thresh:.2f} f={frame_idx}"
    cv2.putText(vis, status, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.57, (0, 255, 0) if detected else (0, 0, 255), 2)
    cv2.putText(vis, method, (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 0), 1)

    if detector.binarized_img is not None:
        mask = detector.binarized_img.astype(np.uint8)
        mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_small = cv2.resize(mask_rgb, (w // 3, h // 3))
        mh, mw = mask_small.shape[:2]
        vis[0:mh, w - mw : w] = mask_small
    return vis


def main():
    args = build_parser().parse_args()
    prob_thresh = (
        float(CONE_PROBABILITY_THRESHOLD_PHASE4)
        if args.phase == 4
        else float(CONE_PROBABILITY_THRESHOLD_PHASE5)
    )

    outdir = Path(args.outdir).expanduser().resolve() if args.outdir else _make_run_outdir(DEFAULT_OUT_BASE)
    if args.outdir:
        outdir.mkdir(parents=True, exist_ok=True)
    csv_path = outdir / "debug.csv"

    detector = dc.detector()
    try:
        roi_loader = _RoiLoader()
        roi_images = roi_loader._load_roi_images()
        roi_input = roi_images if roi_images else None
        detector.set_roi_img(roi_input)
    except Exception as exc:
        print(f"ROI setup warning: {exc}")

    gui_enabled = (not args.headless) and _display_available()
    if (not args.headless) and (not gui_enabled):
        print("GUI disabled: DISPLAY/WAYLAND_DISPLAY is not set. Running in save-only mode.")
    if gui_enabled:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    frame_idx = 0
    detect_count = 0
    reached_count = 0
    t_start = time.time()

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "frame",
                "time_sec",
                "probability",
                "is_detected_by_phase_threshold",
                "is_reached",
                "occupancy",
                "frame_red_occupancy",
                "cone_direction",
                "method",
                "shape_score",
                "cone_shape_score",
                "sv_score",
                "hue_redness_score",
                "roi_support_ratio",
            ]
        )

        while True:
            frame_idx += 1
            ok = detector.detect_cone()
            now = time.time()
            if not ok:
                print("camera capture failed")
                time.sleep(max(0.0, args.sleep))
                if args.frames > 0 and frame_idx >= args.frames:
                    break
                continue

            prob = float(detector.probability)
            is_reached = bool(detector.is_reached)
            is_det = prob >= prob_thresh
            detect_count += 1 if is_det else 0
            reached_count += 1 if is_reached else 0

            scores = getattr(detector, "debug_scores", {}) or {}
            writer.writerow(
                [
                    frame_idx,
                    now - t_start,
                    prob,
                    int(is_det),
                    int(is_reached),
                    float(getattr(detector, "occupancy", 0.0)),
                    float(getattr(detector, "frame_red_occupancy", 0.0)),
                    float(getattr(detector, "cone_direction", 0.5)),
                    str(getattr(detector, "debug_method", "")),
                    float(scores.get("shape", 0.0)),
                    float(scores.get("cone_shape", 0.0)),
                    float(scores.get("sv", 0.0)),
                    float(scores.get("hue", 0.0)),
                    float(scores.get("roi_support", 0.0)),
                ]
            )
            f.flush()

            if frame_idx % max(1, args.save_every) == 0:
                stem = f"frame_{frame_idx:06d}"
                if detector.input_img is not None:
                    cv2.imwrite(str(outdir / f"{stem}_input.png"), detector.input_img)
                if detector.projected_img is not None:
                    proj = detector.projected_img
                    if proj.ndim == 2:
                        cv2.imwrite(str(outdir / f"{stem}_projected.png"), proj.astype(np.uint8))
                    else:
                        cv2.imwrite(str(outdir / f"{stem}_projected.png"), proj)
                if detector.binarized_img is not None:
                    cv2.imwrite(str(outdir / f"{stem}_mask.png"), detector.binarized_img.astype(np.uint8))

            if gui_enabled:
                vis = _overlay(detector, prob_thresh, frame_idx)
                if vis is not None:
                    cv2.imshow(WINDOW_NAME, vis)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("s"):
                    stem = f"manual_{frame_idx:06d}"
                    if detector.input_img is not None:
                        cv2.imwrite(str(outdir / f"{stem}_input.png"), detector.input_img)
                    if detector.binarized_img is not None:
                        cv2.imwrite(str(outdir / f"{stem}_mask.png"), detector.binarized_img.astype(np.uint8))

            if args.frames > 0 and frame_idx >= args.frames:
                break
            time.sleep(max(0.0, args.sleep))

    elapsed = max(1e-6, time.time() - t_start)
    print(f"Output dir: {outdir}")
    print(f"CSV: {csv_path}")
    print(
        f"Frames={frame_idx}, Detect(phase{args.phase})={detect_count} ({100.0 * detect_count / max(1, frame_idx):.1f}%), "
        f"Reached={reached_count}, FPS={frame_idx / elapsed:.2f}"
    )

    try:
        detector.close()
    except Exception:
        pass
    if gui_enabled:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
