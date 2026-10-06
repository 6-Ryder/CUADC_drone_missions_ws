"""Exercise the real launcher's preflight prefix without running any ROS/flight commands."""

import fcntl
import os
from pathlib import Path
import subprocess

import pytest

SCRIPT = Path(__file__).resolve().parents[3] / "scripts/start_quick_mission_real.sh"


def _prefix(tmp_path):
    source = SCRIPT.read_text().split("# ---- 环境自愈", 1)[0]
    script = tmp_path / "scripts" / "prefix.sh"
    script.parent.mkdir(exist_ok=True)
    script.write_text(source + '\nprintf "PREFLIGHT_PREFIX_REACHED\\n"\n')
    return script


@pytest.mark.parametrize("port", ["55444", "55445"])
def test_second_quick_instance_exits_before_any_old_process_cleanup(tmp_path, port):
    script = _prefix(tmp_path)
    env = dict(os.environ, TMPDIR=str(tmp_path), DDS_PORT=port)
    lock_path = tmp_path / f"drone_quick_real_{os.getuid()}.lock"
    with lock_path.open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        result = subprocess.run(["bash", str(script)], env=env,
                                capture_output=True, text=True, timeout=5)
    assert result.returncode != 0
    assert "PREFLIGHT_PREFIX_REACHED" not in result.stdout
    assert "已有 quick 实例运行" in result.stdout


def test_quick_runs_always_get_distinct_log_directories(tmp_path):
    script = _prefix(tmp_path)
    # Pin date so this detects second-resolution collisions even across
    # a wall-clock second boundary; separate invocations still have new PIDs.
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    date = bin_dir / "date"
    date.write_text('#!/bin/sh\nprintf "20261006_161615\\n"\n')
    date.chmod(0o755)
    env = dict(os.environ, TMPDIR=str(tmp_path), DDS_PORT="55444",
               PATH=str(bin_dir) + os.pathsep + os.environ["PATH"])
    for _ in range(2):
        result = subprocess.run(["bash", str(script)], env=env,
                                capture_output=True, text=True, timeout=5)
        assert result.returncode == 0, result.stderr
    directories = list((tmp_path / "log").glob("quick_real_mission_*"))
    assert len(directories) == 2
