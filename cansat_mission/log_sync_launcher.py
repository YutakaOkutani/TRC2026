from __future__ import annotations

import os
import platform
import subprocess
import sys
from pathlib import Path


def trigger_async_log_sync(reason: str = "") -> None:
    """Launch analyzer/fetch_latest_robust_log.py in background, never blocking mission flow."""
    if platform.system().lower() != "linux":
        return

    repo_root = Path(__file__).resolve().parents[1]
    script_path = repo_root / "analyzer" / "fetch_latest_robust_log.py"
    if not script_path.exists():
        return

    cmd = [sys.executable, str(script_path)]
    if reason:
        cmd.extend(["--reason", reason])

    try:
        kwargs = {
            "stdin": subprocess.DEVNULL,
            "stdout": subprocess.DEVNULL,
            "stderr": subprocess.DEVNULL,
        }
        if os.name == "nt":
            flags = getattr(subprocess, "DETACHED_PROCESS", 0) | getattr(subprocess, "CREATE_NO_WINDOW", 0)
            kwargs["creationflags"] = flags
        else:
            kwargs["start_new_session"] = True
        subprocess.Popen(cmd, **kwargs)
    except Exception:
        # Log sync must never impact mission execution.
        pass
