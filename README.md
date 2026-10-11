# roarm-m3

Control and imitation learning for a [Waveshare RoArm-M3](https://www.waveshare.com/wiki/RoArm-M3). The goal: teach it by hand to pick up small electrodes, using [LeRobot](https://github.com/huggingface/lerobot)'s ACT policy.

```
imitation_learning.ipynb   record demos -> train -> run (start here)
roarm/arm.py               talk to the arm over USB or WiFi (JSON commands)
roarm/camera.py            USB cameras via OpenCV
roarm/teach.py             recording, training and running the policy
roarm/train_lerobot.py     lerobot-train plus two fixes (speed, Windows)
scripts/check_arm.py       connection test; doesn't move the arm
scripts/train.py           train from a terminal, e.g. on a faster PC
docs/cameras.md            which cameras to buy and how to mount them
```

## Setup

```powershell
$env:UV_CACHE_DIR = "D:\.cache\uv"   # C: is nearly full; keep the ~5 GB download cache on D:
uv sync                               # Python 3.12, LeRobot 0.6.1, PyTorch 2.11 (CUDA 12.6)
uv run python scripts/check_arm.py    # should print the six joint angles
```

Open `imitation_learning.ipynb` in VS Code and pick the `.venv` kernel.

PyTorch is pinned to the CUDA 12.6 build because newer CUDA builds dropped support for the GTX 1060. That build also runs on newer cards like the RTX 3090.

## Training

ACT, 20,000 steps, batch 8, two 240×320 cameras: about **3.6 steps/s on the GTX 1060 (≈1.5 h)**, using 3.5 GB of GPU memory.

To train on a faster PC (e.g. the 3090):

1. On that PC: install [uv](https://docs.astral.sh/uv/) and git, clone this repo, run `uv sync`.
2. Copy the `data/electrode_pick` folder over (same path inside the repo).
3. Run `uv run python scripts/train.py data/electrode_pick outputs/act_electrode_pick`. With 24 GB of memory you can also try `--batch-size 16 --workers 8`.
4. Copy `outputs/act_electrode_pick/checkpoints/last/pretrained_model` (about 200 MB) back to the same path on this PC, then run step 4 of the notebook.

## This arm

| | |
|---|---|
| Power | 12 V 5 A supply, switch on the driver board |
| WiFi | Joins the home network at **192.168.1.203** (reserved in the router for MAC `EC:C9:FF:F1:3F:64`); keeps its own `RoArm-M3` hotspot (`12345678`, 192.168.4.1) as a backup |
| USB | Only **one** of the board's two USB-C ports talks to the arm (COM14 on the desktop). `RoArm()` finds it automatically. Use a port on the back of the PC; the front ones gave "Device Descriptor Request Failed". |
| Web app | http://192.168.1.203 |

## Commands used here

All commands are JSON. Send them over USB serial (115200 baud) or `http://<ip>/js?json=<command>`. Angles are in radians.

| Command | Does |
|---|---|
| `{"T":105}` | Read joints (`b s e t r g`) and XYZ. Replies with `{"T":1051,...}` |
| `{"T":102,"base":0,"shoulder":0,"elbow":1.57,"wrist":0,"roll":0,"hand":3.14,"spd":0,"acc":10}` | Move all joints (`spd` in steps/s, 4096 per turn; 0 = max) |
| `{"T":210,"cmd":0}` / `{"T":210,"cmd":1}` | All motors off (limp) / on. Any move command turns them back on. |
| `{"T":100}` | Go to the power-on pose |
| `{"T":405}` / `{"T":406}` | Show WiFi status / save current WiFi settings |
| `{"T":404,"ap_ssid":...,"ap_password":...,"sta_ssid":...,"sta_password":...}` | Join a network and keep the hotspot |

Joint limits: base ±3.14, shoulder ±1.57, elbow 0–3.14, wrist ±1.57, roll ±3.14, gripper 1.08 (open) – 3.14 (closed).

## Things that weren't obvious

- **Reading joints over WiFi:** an HTTP `{"T":105}` request returns only `{"ok":1}`. The actual reply arrives on the WebSocket at `ws://<ip>/ws`. Sending JSON *into* the WebSocket does nothing. Round trips are about 70 ms with spikes to 300 ms, so the notebook defaults to USB, which is steadier.
- **USB:** set RTS and DTR low before opening the port, or the ESP32 resets. `arm.py` does this.
- **`{"T":405}` scrambles** its password and hotspot-name fields. It's a display bug; the saved settings are fine. The garbage can include characters that are illegal in JSON, so parse its reply with `json.loads(line, strict=False)`.
- **The arm doesn't rejoin WiFi on its own after a dropout.** `{"T":405}` then shows `"ip":"0.0.0.0"`. Power-cycle it, or over USB send `{"T":407,"mode":3,"ap_ssid":"RoArm-M3","ap_password":"12345678","sta_ssid":"<network>","sta_password":"<password>"}`, which saves a clean WiFi config and reconnects.
- **"Bytes received: … / Invalid data, ignore"** on the serial port is the arm ignoring wireless broadcasts (ESP-NOW) from some other device nearby. It's harmless.
- **LeRobot 0.6.1 training was 80× slower than it should be:**
  - ACT reads 100 future actions per sample, and LeRobot fetched them in a way that also loaded every camera image in those rows (357 ms per sample, about 2 s per step).
  - `roarm/train_lerobot.py` selects just the action column instead (4 ms per sample, same data), so training is GPU-bound.
- **Windows + LeRobot:**
  - `lerobot-train` marks the newest checkpoint with a symlink, which needs admin rights on Windows. `roarm/train_lerobot.py` uses a directory junction instead.
  - The `torchcodec` DLL warning during training is harmless: datasets here store images, not video.
- **Disk:** Hugging Face caches every dataset it loads. `roarm/__init__.py` points that cache at `.cache/` in this repo. Each training checkpoint is about 600 MB.

## Getting good demos

- Keep the electrode positions varied, but your motion consistent.
- Hold the arm low (upper arm or elbow) so your hand stays out of the wrist camera's view.
- If the policy gets confused by seeing your hand: do the demo by hand with the cameras off, put the electrode back, and replay the recorded joint path with the motors on while recording images. The images then show only the arm. (Not built into the notebook yet.)
- A second RoArm-M3 as a leader arm (Waveshare's [AI kit](https://www.waveshare.com/wiki/RoArm-M3-AI-Kit) setup) avoids the problem entirely.

## Links

- [Waveshare wiki](https://www.waveshare.com/wiki/RoArm-M3): [JSON commands](https://www.waveshare.com/wiki/RoArm-M3-S_Robotic_Arm_Control), [WiFi](https://www.waveshare.com/wiki/RoArm-M3-S_WIFI_Configuration), [STEP files](https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_STEP_260310.zip), [CP2102 driver](https://files.waveshare.com/wiki/common/CP210x_USB_TO_UART.zip)
- [waveshareteam/roarm_ws](https://github.com/waveshareteam/roarm_ws): ROS 2 Humble + MoveIt
- [metrox-eth/lerobot_robot_roarm_m3](https://github.com/metrox-eth/lerobot_robot_roarm_m3): a LeRobot plugin for this arm (follower only, needs a separate teleop device)
