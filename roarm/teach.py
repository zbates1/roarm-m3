"""Teach the arm by hand, train a LeRobot ACT policy, and let it run.

Demonstrations are kinesthetic: the motors go limp, you move the arm through
the task, and it records joint angles + camera frames. The action for each
frame is the next frame's joint angles, so the policy learns "where to go next".
"""
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

from .arm import JOINTS

ROBOT_TYPE = "roarm_m3"


def make_features(camera_names, image_size):
    h, w = image_size
    joints = {"dtype": "float32", "shape": (len(JOINTS),), "names": list(JOINTS)}
    features = {"observation.state": joints, "action": dict(joints)}
    for name in camera_names:
        features[f"observation.images.{name}"] = {
            "dtype": "image", "shape": (h, w, 3), "names": ["height", "width", "channels"],
        }
    return features


def open_dataset(root, repo_id, fps, camera_names, image_size):
    """Create the dataset, or reopen it to add more episodes. Call dataset.finalize() when done."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = Path(root)
    threads = 4 * max(1, len(camera_names))
    if (root / "meta" / "info.json").exists():
        dataset = LeRobotDataset.resume(repo_id, root=root, image_writer_threads=threads)
        print(f"Reopened {root} ({dataset.num_episodes} episodes so far)")
        return dataset
    return LeRobotDataset.create(
        repo_id, fps, make_features(camera_names, image_size), root=root,
        robot_type=ROBOT_TYPE, use_videos=False, image_writer_threads=threads,
    )


def record_episode(arm, cameras, dataset, task, fps, image_size, max_seconds=60):
    """Record one demonstration. Returns True if it was saved."""
    input("Take the electrode out of the gripper, put it somewhere new, then press Enter to send the arm home...")
    arm.go_home()
    input("Hold the arm (it is about to go limp), then press Enter to start recording...")

    states, frames = [], {name: [] for name in cameras}
    stop = threading.Event()

    def capture():
        t_end = time.time() + max_seconds
        while not stop.is_set() and time.time() < t_end:
            t0 = time.perf_counter()
            states.append(arm.state())
            for name, cam in cameras.items():
                frames[name].append(cam.read(image_size))
            time.sleep(max(0.0, 1 / fps - (time.perf_counter() - t0)))

    arm.torque(False)
    worker = threading.Thread(target=capture)
    worker.start()
    try:
        input("Recording. Do the task, then press Enter to stop...")
    finally:
        stop.set()
        worker.join()
        arm.hold()

    n = len(states)
    print(f"Recorded {n} frames ({n / fps:.1f} s).")
    if n < 2 or input("Keep this episode? [Y/n] ").strip().lower() == "n":
        print("Discarded.")
        return False
    for t in range(n):
        frame = {
            "observation.state": states[t],
            "action": states[min(t + 1, n - 1)],
            "task": task,
        }
        for name in cameras:
            frame[f"observation.images.{name}"] = frames[name][t]
        dataset.add_frame(frame)
    dataset.save_episode()
    print(f"Saved episode {dataset.num_episodes - 1}.")
    return True


def train(dataset_root, repo_id, output_dir, steps=20_000, batch_size=8):
    """Train ACT with lerobot-train. Prints the log as it goes."""
    if Path(output_dir).exists():
        raise FileExistsError(f"{output_dir} exists. Pick a new output_dir or delete the old one.")
    cmd = [
        sys.executable, "-m", "roarm.lerobot_train_win",
        f"--dataset.repo_id={repo_id}",
        f"--dataset.root={dataset_root}",
        "--policy.type=act",
        "--policy.device=cuda",
        "--policy.push_to_hub=false",
        f"--output_dir={output_dir}",
        "--job_name=act_roarm",
        f"--steps={steps}",
        f"--batch_size={batch_size}",
        f"--save_freq={max(1, steps // 4)}",
        "--num_workers=2",
        "--wandb.enable=false",
    ]
    print(" ".join(cmd[1:]))
    subprocess.run(cmd, check=True)


def load_policy(checkpoint):
    """Load a trained policy plus the normalization steps saved with it."""
    import torch
    from lerobot.policies import get_policy_class, make_pre_post_processors
    from lerobot.configs.policies import PreTrainedConfig

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    config = PreTrainedConfig.from_pretrained(checkpoint)
    config.device = device.type
    policy = get_policy_class(config.type).from_pretrained(checkpoint, config=config)
    policy.to(device).eval()
    pre, post = make_pre_post_processors(
        config, pretrained_path=str(checkpoint),
        preprocessor_overrides={"device_processor": {"device": device.type}},
    )
    return policy, pre, post, device


def run_policy(arm, cameras, checkpoint, task, fps, image_size, seconds=30, max_step=0.1):
    """Let the policy drive. max_step caps how far any joint moves per step (radians).

    Interrupt the cell (the stop button) to stop early; the arm locks in place.
    """
    from lerobot.common.control_utils import predict_action

    policy, pre, post, device = load_policy(checkpoint)
    policy.reset()
    arm.go_home()
    input("Clear the area, then press Enter to start the policy...")
    try:
        t_end = time.time() + seconds
        while time.time() < t_end:
            t0 = time.perf_counter()
            state = arm.state()
            obs = {"observation.state": state}
            for name, cam in cameras.items():
                obs[f"observation.images.{name}"] = cam.read(image_size)
            action = predict_action(obs, policy, device, pre, post, use_amp=False,
                                    task=task, robot_type=ROBOT_TYPE)
            target = np.asarray(action.detach().cpu()).reshape(-1)
            arm.move(state + np.clip(target - state, -max_step, max_step))
            time.sleep(max(0.0, 1 / fps - (time.perf_counter() - t0)))
    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        arm.hold()


def preview(cameras, image_size=None):
    """Show one frame from each camera."""
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, len(cameras), figsize=(5 * len(cameras), 4), squeeze=False)
    for ax, (name, cam) in zip(axes[0], cameras.items()):
        ax.imshow(cam.read(image_size))
        ax.set_title(name)
        ax.axis("off")
    plt.show()
