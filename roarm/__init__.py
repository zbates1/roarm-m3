"""Waveshare RoArm-M3 control and imitation-learning helpers."""
import os
from pathlib import Path

# Hugging Face caches every dataset it loads. Keep that cache next to the repo
# (C: on this PC is nearly full) unless you've already chosen a location.
os.environ.setdefault("HF_HOME", str(Path(__file__).resolve().parent.parent / ".cache" / "huggingface"))
