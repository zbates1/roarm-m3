"""Train ACT on a recorded dataset from the terminal (e.g. on a faster PC).

    uv run python scripts/train.py data/electrode_pick outputs/act_electrode_pick
    uv run python scripts/train.py data/electrode_pick outputs/act_big --batch-size 32 --workers 8

Copy the dataset folder over first, then copy
<output>/checkpoints/last/pretrained_model back to run it on the arm.
"""
import argparse

from roarm import teach

p = argparse.ArgumentParser()
p.add_argument("dataset", help="dataset folder, e.g. data/electrode_pick")
p.add_argument("output", help="new folder for checkpoints, e.g. outputs/act_electrode_pick")
p.add_argument("--repo-id", default="local/electrode_pick")
p.add_argument("--steps", type=int, default=20_000)
p.add_argument("--batch-size", type=int, default=8)
p.add_argument("--workers", type=int, default=2)
a = p.parse_args()

teach.train(a.dataset, a.repo_id, a.output, steps=a.steps, batch_size=a.batch_size, num_workers=a.workers)
