from __future__ import annotations

import argparse
import shlex
import subprocess
import sys
from pathlib import Path

PI_IP = "100.107.201.122"
PI_USER = "pi"
SSH_PORT = 22

PI_LOG_DIRS = ("/home/pi/TRC2026/log", "/home/pi/TRC2026/tests/log")
PC_DEST_DIR = "~/Downloads"
LOG_PATTERN = "robust_log_*.csv"

PC_TO_PI_IDENTITY_FILE = "~/.ssh/id_ed25519_pi"
SSH_CONNECT_TIMEOUT_SEC = 8
SSH_CMD_TIMEOUT_SEC = 30
SCP_CMD_TIMEOUT_SEC = 120


def _run(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    timeout = SCP_CMD_TIMEOUT_SEC if cmd and cmd[0] == "scp" else SSH_CMD_TIMEOUT_SEC
    return subprocess.run(cmd, check=True, text=True, capture_output=True, timeout=timeout)


def _ssh_common_opts(identity_file: Path) -> list[str]:
    return [
        "-i",
        str(identity_file),
        "-o",
        "BatchMode=yes",
        "-o",
        "IdentitiesOnly=yes",
        "-o",
        f"ConnectTimeout={SSH_CONNECT_TIMEOUT_SEC}",
        "-o",
        "ServerAliveInterval=5",
        "-o",
        "ServerAliveCountMax=2",
        "-o",
        "StrictHostKeyChecking=accept-new",
    ]


def find_latest_remote_log(
    host: str,
    user: str,
    port: int,
    remote_dirs: tuple[str, ...],
    pattern: str,
    ssh_opts: list[str],
) -> str:
    quoted_dirs = " ".join(shlex.quote(d) for d in remote_dirs)
    quoted_pattern = shlex.quote(pattern)

    remote_cmd = (
        "set -eu; "
        f"pattern={quoted_pattern}; "
        f"for d in {quoted_dirs}; do "
        '  [ -d "$d" ] || continue; '
        '  find "$d" -maxdepth 1 -type f -name "$pattern" -printf "%T@\\t%p\\n" 2>/dev/null || true; '
        "done | sort -nr | head -n 1 | cut -f2-"
    )

    cmd = ["ssh", *ssh_opts, "-p", str(port), f"{user}@{host}", remote_cmd]
    result = _run(cmd)
    latest = result.stdout.strip()
    if not latest:
        fallback_cmd = (
            "set -eu; "
            f"pattern={quoted_pattern}; "
            'root="/home/pi/TRC2026"; '
            '[ -d "$root" ] || exit 0; '
            'find "$root" -type f -name "$pattern" -printf "%T@\\t%p\\n" 2>/dev/null | sort -nr | head -n 1 | cut -f2-'
        )
        fallback = _run(["ssh", *ssh_opts, "-p", str(port), f"{user}@{host}", fallback_cmd]).stdout.strip()
        latest = fallback
    if not latest:
        raise FileNotFoundError(
            "No remote file matched "
            f"{pattern} in any of: {', '.join(remote_dirs)} on {user}@{host}."
        )
    return latest


def pull_from_remote(
    host: str,
    user: str,
    port: int,
    remote_path: str,
    local_dir: Path,
    ssh_opts: list[str],
) -> Path:
    local_dir.mkdir(parents=True, exist_ok=True)
    src = f"{user}@{host}:{remote_path}"
    cmd = ["scp", *ssh_opts, "-P", str(port), src, str(local_dir)]
    _run(cmd)
    downloaded = local_dir / Path(remote_path).name
    if not downloaded.exists():
        raise FileNotFoundError(f"SCP finished but file was not found locally: {downloaded}")
    return downloaded


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Pull latest robust_log_*.csv from Raspberry Pi to this PC."
    )
    parser.add_argument("--reason", default="", help="Optional tag for run logs.")
    parser.add_argument("--dest", default=PC_DEST_DIR, help="Destination directory on this PC.")
    return parser


def main() -> int:
    args = build_parser().parse_args()

    try:
        identity_file = Path(PC_TO_PI_IDENTITY_FILE).expanduser()
        if not identity_file.exists():
            raise FileNotFoundError(f"Identity file not found: {identity_file}")
        ssh_opts = _ssh_common_opts(identity_file)
        tag = f", reason={args.reason}" if args.reason else ""
        print(f"[INFO] Mode: pull_only (pi={PI_USER}@{PI_IP}, key={identity_file}{tag})")

        latest_remote = find_latest_remote_log(PI_IP, PI_USER, SSH_PORT, PI_LOG_DIRS, LOG_PATTERN, ssh_opts)
        local_dest = Path(args.dest).expanduser().resolve()
        print(f"[INFO] Remote latest: {latest_remote}")
        print(f"[INFO] Pulling to: {local_dest}")
        downloaded = pull_from_remote(PI_IP, PI_USER, SSH_PORT, latest_remote, local_dest, ssh_opts)
        print(f"[INFO] Pulled latest log: {latest_remote} -> {downloaded}")
        return 0
    except subprocess.TimeoutExpired as exc:
        print(f"[ERROR] Command timed out after {exc.timeout}s: {exc.cmd}", file=sys.stderr)
        return 1
    except subprocess.CalledProcessError as exc:
        stderr = (exc.stderr or "").strip()
        stdout = (exc.stdout or "").strip()
        print("[ERROR] SSH/SCP command failed.", file=sys.stderr)
        if stdout:
            print(f"[ERROR] stdout: {stdout}", file=sys.stderr)
        if stderr:
            print(f"[ERROR] stderr: {stderr}", file=sys.stderr)
        return exc.returncode or 1
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
