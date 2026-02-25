from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

try:
    from cansat_mission.constants import LOG_HEADER
except Exception:
    LOG_HEADER = []


COLUMN_GROUPS = [
    {
        "key": "time_and_phase",
        "title": "Time & Mission Phase",
        "cols": ["ElapsedSec", "Phase", "MissionElapsedSec", "MissionEndReason", "MissionTotalTimeout"],
    },
    {
        "key": "imu_accel",
        "title": "IMU - Acceleration",
        "cols": ["AccX", "AccY", "AccZ"],
    },
    {
        "key": "imu_gyro",
        "title": "IMU - Gyroscope",
        "cols": ["GyroX", "GyroY", "GyroZ"],
    },
    {
        "key": "imu_mag",
        "title": "IMU - Magnetometer",
        "cols": ["MagX", "MagY", "MagZ"],
    },
    {
        "key": "gps_position",
        "title": "GPS - Position & Quality",
        "cols": ["LAT", "LNG", "GpsSpeedMps", "GPSFixQual", "GPSSats", "GPSHdop"],
    },
    {
        "key": "altitude_pressure",
        "title": "Altitude & Pressure",
        "cols": ["ALT", "Pres"],
    },
    {
        "key": "navigation",
        "title": "Navigation",
        "cols": ["Distance", "Azimuth", "TargetLat", "TargetLng", "Angle", "Direction", "AngleValid"],
    },
    {
        "key": "vision_detection",
        "title": "Vision / Detection",
        "cols": ["ConeDir", "ConeProb", "ConeMethod", "ObstacleDist"],
    },
    {
        "key": "fall_and_sensor_health",
        "title": "Fall & Sensor Health",
        "cols": ["Fall", "BNOStaleSec"],
    },
    {
        "key": "motor_commands",
        "title": "Motor Commands",
        "cols": [
            "MotorCmdType",
            "MotorCmdUpdatedElapsedSec",
            "Motor1CmdSpeed",
            "Motor1CmdForward",
            "Motor2CmdSpeed",
            "Motor2CmdForward",
        ],
    },
]


def find_latest_log() -> Path:
    candidates = []
    home = Path.home()
    for target_dir in (home / "Download", home / "Downloads"):
        if not target_dir.exists():
            continue
        candidates.extend(target_dir.glob("robust_log_*.csv"))

    if not candidates:
        raise FileNotFoundError("No robust_log_*.csv found in ~/Download or ~/Downloads")
    return max(candidates, key=lambda p: p.stat().st_mtime)


def prepare_output_dir(log_path: Path) -> Path:
    out_dir = REPO_ROOT / "analyzer" / "outputs" / log_path.stem
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir


def _safe_numeric_series(df: pd.DataFrame, col: str) -> pd.Series:
    return pd.to_numeric(df[col], errors="coerce")


def write_group_csvs(df: pd.DataFrame, out_dir: Path) -> None:
    for group in COLUMN_GROUPS:
        present_cols = [c for c in group["cols"] if c in df.columns]
        if not present_cols:
            continue
        export_cols = present_cols if "ElapsedSec" in present_cols else (["ElapsedSec"] + present_cols if "ElapsedSec" in df.columns else present_cols)
        df[export_cols].to_csv(out_dir / f"{group['key']}.csv", index=False)


def write_coverage_reports(df: pd.DataFrame, out_dir: Path) -> dict:
    grouped_cols = [c for g in COLUMN_GROUPS for c in g["cols"]]
    grouped_set = set(grouped_cols)
    expected_set = set(LOG_HEADER) if LOG_HEADER else set(df.columns)
    actual_set = set(df.columns)

    duplicate_group_cols = sorted({c for c in grouped_cols if grouped_cols.count(c) > 1})
    expected_missing_in_groups = sorted(expected_set - grouped_set)
    grouped_not_in_expected = sorted(grouped_set - expected_set) if LOG_HEADER else []
    actual_missing_from_csv = sorted(expected_set - actual_set)
    actual_extra_in_csv = sorted(actual_set - expected_set) if LOG_HEADER else []

    lines = [
        f"Source CSV: {df.attrs.get('source_path', '')}",
        f"Row count: {len(df)}",
        f"Column count: {len(df.columns)}",
        "",
        "[Coverage vs LOG_HEADER]",
        f"Duplicate columns across groups: {duplicate_group_cols or 'None'}",
        f"LOG_HEADER columns missing from groups: {expected_missing_in_groups or 'None'}",
        f"Grouped columns not in LOG_HEADER: {grouped_not_in_expected or 'None'}",
        f"LOG_HEADER columns missing from CSV: {actual_missing_from_csv or 'None'}",
        f"CSV columns not in LOG_HEADER: {actual_extra_in_csv or 'None'}",
        "",
        "[Group Definitions]",
    ]
    for g in COLUMN_GROUPS:
        lines.append(f"- {g['key']}: {', '.join(g['cols'])}")

    (out_dir / "coverage_report.txt").write_text("\n".join(lines), encoding="utf-8")

    summary_rows = []
    for g in COLUMN_GROUPS:
        for c in g["cols"]:
            summary_rows.append(
                {
                    "group_key": g["key"],
                    "group_title": g["title"],
                    "column": c,
                    "in_csv": c in df.columns,
                    "in_log_header": (c in LOG_HEADER) if LOG_HEADER else None,
                }
            )
    pd.DataFrame(summary_rows).to_csv(out_dir / "column_group_mapping.csv", index=False)

    return {
        "duplicate_group_cols": duplicate_group_cols,
        "expected_missing_in_groups": expected_missing_in_groups,
        "grouped_not_in_expected": grouped_not_in_expected,
        "actual_missing_from_csv": actual_missing_from_csv,
        "actual_extra_in_csv": actual_extra_in_csv,
    }


def write_basic_summaries(df: pd.DataFrame, out_dir: Path) -> None:
    df.describe(include="all").transpose().to_csv(out_dir / "summary_describe_all.csv")

    categorical_rows = []
    for col in df.columns:
        if col == "ElapsedSec":
            continue
        numeric = pd.to_numeric(df[col], errors="coerce")
        if numeric.notna().sum() > 0 and numeric.notna().sum() >= max(1, int(len(df) * 0.8)):
            continue
        counts = df[col].astype(str).value_counts(dropna=False).head(20)
        for value, count in counts.items():
            categorical_rows.append({"column": col, "value": value, "count": int(count)})
    if categorical_rows:
        pd.DataFrame(categorical_rows).to_csv(out_dir / "categorical_value_counts_top20.csv", index=False)


def plot_integrated_timeseries(df: pd.DataFrame, out_dir: Path) -> None:
    if "ElapsedSec" not in df.columns:
        return

    t = _safe_numeric_series(df, "ElapsedSec")
    plot_groups = []
    for group in COLUMN_GROUPS:
        numeric_cols = []
        for col in group["cols"]:
            if col == "ElapsedSec" or col not in df.columns:
                continue
            s = _safe_numeric_series(df, col)
            if s.notna().any():
                numeric_cols.append(col)
        if numeric_cols:
            plot_groups.append((group["title"], numeric_cols))

    if not plot_groups:
        return

    fig, axes = plt.subplots(len(plot_groups), 1, figsize=(14, max(4, 2.8 * len(plot_groups))), sharex=True)
    if len(plot_groups) == 1:
        axes = [axes]

    for ax, (title, cols) in zip(axes, plot_groups):
        for col in cols:
            ax.plot(t, _safe_numeric_series(df, col), label=col, alpha=0.85, linewidth=1.0)
        ax.set_title(title, fontweight="bold")
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.legend(loc="upper right", fontsize="x-small", ncol=2)

    axes[-1].set_xlabel("Elapsed Seconds [s]")
    fig.tight_layout()
    fig.savefig(out_dir / "integrated_sensor_log.png", dpi=150)
    plt.close(fig)


def plot_trajectory(df: pd.DataFrame, out_dir: Path) -> None:
    if not {"LAT", "LNG"}.issubset(df.columns):
        return

    lat = _safe_numeric_series(df, "LAT")
    lng = _safe_numeric_series(df, "LNG")
    valid = lat.notna() & lng.notna() & (lat != 0) & (lng != 0)
    if not valid.any():
        return

    lat_v = lat[valid]
    lng_v = lng[valid]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(lng_v, lat_v, color="blue", label="Actual Path")
    ax.scatter(lng_v.iloc[0], lat_v.iloc[0], color="green", label="Start")
    ax.scatter(lng_v.iloc[-1], lat_v.iloc[-1], color="orange", label="End")

    if {"TargetLat", "TargetLng"}.issubset(df.columns):
        tgt_lat = _safe_numeric_series(df, "TargetLat").dropna()
        tgt_lng = _safe_numeric_series(df, "TargetLng").dropna()
        if not tgt_lat.empty and not tgt_lng.empty:
            ax.scatter(tgt_lng.iloc[0], tgt_lat.iloc[0], color="red", marker="*", s=180, label="Target")

    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.set_title("CanSat Trajectory Map")
    ax.legend()
    ax.grid(True, linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(out_dir / "trajectory_map.png", dpi=150)
    plt.close(fig)


def analyze_cansat_log(file_path: str | Path | None = None) -> Path:
    log_path = Path(file_path).resolve() if file_path else find_latest_log()
    out_dir = prepare_output_dir(log_path)

    df = pd.read_csv(log_path)
    df.attrs["source_path"] = str(log_path)

    coverage = write_coverage_reports(df, out_dir)
    write_group_csvs(df, out_dir)
    write_basic_summaries(df, out_dir)
    plot_integrated_timeseries(df, out_dir)
    plot_trajectory(df, out_dir)

    print(f"[INFO] Source log: {log_path}")
    print(f"[INFO] Output dir : {out_dir}")
    print(f"[INFO] Rows/Cols   : {len(df)} / {len(df.columns)}")
    if any(coverage.values()):
        print("[WARN] Column coverage mismatch detected. See coverage_report.txt")
    else:
        print("[INFO] Column coverage OK (vs LOG_HEADER and group definitions).")

    return out_dir


if __name__ == "__main__":
    arg_path = sys.argv[1] if len(sys.argv) > 1 else None
    analyze_cansat_log(arg_path)
