"""Run lerobot-train (LeRobot 0.6.1) with two fixes.

1. Speed: ACT reads 100 future actions per sample. LeRobot fetches them with
   `hf_dataset["action"][rows]`, which in `datasets` 4.x still loads whole rows,
   camera images included (~400 ms per sample). Selecting the column first takes
   ~3 ms, so training stops waiting on the CPU.
2. Windows: LeRobot marks the newest checkpoint with a `checkpoints/last`
   symlink, which needs admin rights on Windows. A directory junction works
   the same way without them.

    python -m roarm.train_lerobot --dataset.repo_id=... (same flags as lerobot-train)
"""
import os
import sys
from pathlib import Path

import torch
from lerobot.datasets import dataset_reader
from lerobot.scripts import lerobot_train


def _query_hf_dataset(self, query_indices):
    cache = self.__dict__.setdefault("_column_cache", {})
    result = {}
    for key, q_idx in query_indices.items():
        if key in self._meta.video_keys:
            continue
        rows = (
            q_idx
            if self._absolute_to_relative_idx is None
            else [self._absolute_to_relative_idx[i] for i in q_idx]
        )
        cache_key = (id(self.hf_dataset), key)
        if cache_key not in cache:
            cache[cache_key] = self.hf_dataset.select_columns([key])
        result[key] = torch.stack(cache[cache_key][rows][key])
    return result


def _update_last_checkpoint(checkpoint_dir):
    last = Path(checkpoint_dir).parent / "last"
    if last.is_junction() or last.is_symlink():
        os.rmdir(last) if last.is_junction() else last.unlink()
    if sys.platform == "win32":
        import _winapi

        _winapi.CreateJunction(str(Path(checkpoint_dir).resolve()), str(last))
    else:
        last.symlink_to(Path(checkpoint_dir).name)


dataset_reader.DatasetReader._query_hf_dataset = _query_hf_dataset
lerobot_train.update_last_checkpoint = _update_last_checkpoint

if __name__ == "__main__":
    lerobot_train.main()
