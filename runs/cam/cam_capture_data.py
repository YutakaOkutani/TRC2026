import argparse
import sys
import time
from pathlib import Path

from picamera2 import Picamera2


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "runs" / "log"


def build_parser():
    parser = argparse.ArgumentParser(
        description="Capture one or more camera frames for cone/grass dataset collection."
    )
    parser.add_argument("--count", type=int, default=1, help="Number of images to capture.")
    parser.add_argument(
        "--interval",
        type=float,
        default=0.0,
        help="Seconds to wait between captures (ignored when count=1).",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=2.0,
        help="Camera warmup seconds before the first capture.",
    )
    parser.add_argument("--width", type=int, default=640, help="Capture width (mission default: 640).")
    parser.add_argument("--height", type=int, default=480, help="Capture height (mission default: 480).")
    parser.add_argument(
        "--prefix",
        default="scene",
        help="Filename prefix (e.g. grass, cone_p4, false_positive).",
    )
    parser.add_argument(
        "--outdir",
        default=str(DEFAULT_OUTPUT_DIR),
        help="Base output directory (default: ./runs/log). A run subfolder is created automatically.",
    )
    parser.add_argument(
        "--session-name",
        default="",
        help="Optional suffix for the run folder name (e.g. phase4_grass).",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Use preview configuration (same style as detector).",
    )
    return parser


def main():
    args = build_parser().parse_args()
    if args.count <= 0:
        print("ERROR: --count must be >= 1")
        return 2
    if args.width <= 0 or args.height <= 0:
        print("ERROR: --width/--height must be >= 1")
        return 2

    base_outdir = Path(args.outdir).expanduser().resolve()
    base_outdir.mkdir(parents=True, exist_ok=True)

    picam2 = None
    try:
        picam2 = Picamera2()
        if args.preview:
            config = picam2.create_preview_configuration(
                main={"size": (args.width, args.height), "format": "BGR888"}
            )
        else:
            # Still use BGR888 so saved frames match detector input assumptions.
            config = picam2.create_still_configuration(
                main={"size": (args.width, args.height), "format": "BGR888"}
            )
        picam2.configure(config)
        picam2.start()

        print(f"Camera started: {args.width}x{args.height}, warmup={args.warmup:.1f}s")
        time.sleep(max(0.0, args.warmup))

        ts = time.strftime("%Y%m%d_%H%M%S")
        session_suffix = f"_{args.session_name}" if args.session_name else ""
        outdir = base_outdir / f"capture_{ts}{session_suffix}"
        outdir.mkdir(parents=True, exist_ok=True)
        print(f"Run folder: {outdir}")

        for idx in range(args.count):
            file_path = outdir / f"{args.prefix}_{ts}_{idx + 1:03d}.jpg"
            picam2.capture_file(str(file_path))
            print(f"[{idx + 1}/{args.count}] saved: {file_path}")
            if idx + 1 < args.count and args.interval > 0:
                time.sleep(args.interval)

        print(f"Done. Images saved under: {outdir}")
        return 0
    except Exception as exc:
        print(f"Capture failed: {exc}")
        return 1
    finally:
        if picam2 is not None:
            try:
                picam2.stop()
            except Exception:
                pass
            try:
                picam2.close()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
