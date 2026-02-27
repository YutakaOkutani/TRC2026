from __future__ import annotations

import argparse
import platform
import re
import shlex
import socket
import subprocess
import sys
from pathlib import Path

PI_IP = "100.107.201.122"
PC_IP = "100.100.219.60"
PI_USER = "pi"
PC_USER = "okku0"
SSH_PORT = 22

PI_LOG_DIRS = ("~/TRC2026/log", "~/TRC2026/tests/log")
PC_DEST_DIR = "~/Downloads"
LOG_PATTERN = "robust_log_*.csv"


def _run(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, check=True, text=True, capture_output=True)


def get_local_ipv4s() -> set[str]:
    ips: set[str] = set()

    try:
        r = subprocess.run(["tailscale", "ip", "-4"], text=True, capture_output=True, check=False)
        if r.returncode == 0:
            for line in r.stdout.splitlines():
                line = line.strip()
                if re.fullmatch(r"\d+\.\d+\.\d+\.\d+", line):
                    ips.add(line)
    except Exception:
        pass

    try:
        host = socket.gethostname()
        for info in socket.getaddrinfo(host, None, socket.AF_INET):
            ip = info[4][0]
            if ip and ip != "127.0.0.1":
                ips.add(ip)
    except Exception:
        pass

    try:
        if platform.system().lower().startswith("win"):
            r = subprocess.run(["ipconfig"], text=True, capture_output=True, check=False)
            text = r.stdout
        else:
            r = subprocess.run(["ip", "-4", "addr", "show"], text=True, capture_output=True, check=False)
            text = r.stdout
        ips.update(re.findall(r"\b(?:\d{1,3}\.){3}\d{1,3}\b", text))
    except Exception:
        pass

    return {ip for ip in ips if ip != "127.0.0.1"}


def detect_role() -> str:
    ips = get_local_ipv4s()
    if PI_IP in ips:
        return "push"
    if PC_IP in ips:
        return "pull"

    system_name = platform.system().lower()
    if system_name.startswith("win"):
        return "pull"
    if "linux" in system_name:
        return "push"

    raise RuntimeError(
        "Could not detect role automatically. "
        f"Local IPs={sorted(ips)} and OS={platform.system()} are not recognized."
    )


def find_latest_local_log(local_dirs: tuple[str, ...], pattern: str) -> Path:
    candidates: list[Path] = []
    for local_dir in local_dirs:
        base = Path(local_dir).expanduser()
        if not base.exists():
            continue
        candidates.extend(base.glob(pattern))

    if not candidates:
        raise FileNotFoundError(
            f"No local file matched {pattern} in any of: {', '.join(local_dirs)}"
        )
    return max(candidates, key=lambda p: p.stat().st_mtime)


def find_latest_remote_log(host: str, user: str, port: int, remote_dirs: tuple[str, ...], pattern: str) -> str:
    quoted_dirs = " ".join(shlex.quote(d) for d in remote_dirs)
    quoted_pattern = shlex.quote(pattern)

    remote_cmd = (
        "set -euo pipefail; "
        f"pattern={quoted_pattern}; "
        f"for d in {quoted_dirs}; do "
        '  for f in "$d"/"$pattern"; do '
        '    [ -e "$f" ] || continue; '
        '    mt=$(stat -c %Y "$f" 2>/dev/null || stat -f %m "$f" 2>/dev/null || echo 0); '
        '    printf "%s\t%s\n" "$mt" "$f"; '
        "  done; "
        "done | sort -nr | head -n 1 | cut -f2-"
    )

    cmd = ["ssh", "-p", str(port), f"{user}@{host}", remote_cmd]
    result = _run(cmd)
    latest = result.stdout.strip()
    if not latest:
        raise FileNotFoundError(
            "No remote file matched "
            f"{pattern} in any of: {', '.join(remote_dirs)} on {user}@{host}."
        )
    return latest


def push_to_remote(local_path: Path, host: str, user: str, port: int, remote_dest_dir: str) -> None:
    dst = f"{user}@{host}:{remote_dest_dir}"
    cmd = ["scp", "-P", str(port), str(local_path), dst]
    _run(cmd)


def pull_from_remote(host: str, user: str, port: int, remote_path: str, local_dir: Path) -> Path:
    local_dir.mkdir(parents=True, exist_ok=True)
    src = f"{user}@{host}:{remote_path}"
    cmd = ["scp", "-P", str(port), src, str(local_dir)]
    _run(cmd)
    downloaded = local_dir / Path(remote_path).name
    if not downloaded.exists():
        raise FileNotFoundError(f"SCP finished but file was not found locally: {downloaded}")
    return downloaded


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Auto sync latest robust_log_*.csv between Pi and PC using fixed Tailscale IPs."
    )
    parser.add_argument("--reason", default="", help="Optional tag for logs when triggered automatically.")
    parser.add_argument("--force", choices=["push", "pull"], help="Override auto role detection")
    return parser


def main() -> int:
    args = build_parser().parse_args()

    try:
        role = args.force or detect_role()
        tag = f", reason={args.reason}" if args.reason else ""
        print(f"[INFO] Role: {role} (auto={args.force is None}, os={platform.system()}, ips={sorted(get_local_ipv4s())}{tag})")

        if role == "push":
            latest_local = find_latest_local_log(PI_LOG_DIRS, LOG_PATTERN)
            push_to_remote(latest_local, PC_IP, PC_USER, SSH_PORT, PC_DEST_DIR)
            print(f"[INFO] Sent latest log: {latest_local} -> {PC_USER}@{PC_IP}:{PC_DEST_DIR}")
        else:
            latest_remote = find_latest_remote_log(PI_IP, PI_USER, SSH_PORT, PI_LOG_DIRS, LOG_PATTERN)
            local_dest = Path(PC_DEST_DIR).expanduser().resolve()
            downloaded = pull_from_remote(PI_IP, PI_USER, SSH_PORT, latest_remote, local_dest)
            print(f"[INFO] Pulled latest log: {latest_remote} -> {downloaded}")
        return 0
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
