"""Run lerobot-train with one Windows fix.

LeRobot marks the newest checkpoint with a `checkpoints/last` symlink, and
Windows only lets admins create symlinks. A directory junction works the same
way without admin rights, so we swap that one function and run training as usual.

    python -m roarm.lerobot_train_win --dataset.repo_id=... (same flags as lerobot-train)
"""
import os
import sys
from pathlib import Path

from lerobot.scripts import lerobot_train


def _update_last_checkpoint(checkpoint_dir):
    last = Path(checkpoint_dir).parent / "last"
    if last.is_junction() or last.is_symlink():
        os.rmdir(last) if last.is_junction() else last.unlink()
    if sys.platform == "win32":
        import _winapi

        _winapi.CreateJunction(str(Path(checkpoint_dir).resolve()), str(last))
    else:
        last.symlink_to(Path(checkpoint_dir).name)


lerobot_train.update_last_checkpoint = _update_last_checkpoint

if __name__ == "__main__":
    lerobot_train.main()
