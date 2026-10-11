# roarm-m3

Control and imitation learning for a [Waveshare RoArm-M3](https://www.waveshare.com/wiki/RoArm-M3). The goal: teach it by hand to pick up small electrodes, using [LeRobot](https://github.com/huggingface/lerobot)'s ACT policy.

```
imitation_learning.ipynb   record demos -> train -> run (start here)
roarm/arm.py               talk to the arm over USB or WiFi (JSON commands)
roarm/kinematics.py        the firmware's IK/FK in Python, to check paths against joint limits
roarm/motion.py            smooth streamed moves, limit checks, and settle() sag correction
roarm/camera.py            USB cameras via OpenCV
roarm/teach.py             recording, training and running the policy
roarm/train_lerobot.py     lerobot-train plus two fixes (speed, Windows)
scripts/check_arm.py       connection test; doesn't move the arm
scripts/train.py           train from a terminal, e.g. on a faster PC
tmp_scripts/               quick motion tests: square_dips.py, dip_repeatability.py
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

## Control tuning (2026-10-11)

`tmp_scripts/autotune.py` ran unattended experiments around the ready pose (x=180, z=−40, gripper down); results and logs are in `tmp_scripts/results/`. What it found:

- **Friction bands** (how far a joint's resting point depends on which way it last moved), in encoder steps (1 step = 0.088°):

  | Joint | Stock P=16 | P=48 | **P=80** |
  |---|---|---|---|
  | Base | 18 | 6 | **5** |
  | Shoulder | 31 | 17 | **8** |
  | Elbow | 18 | 7 | **4** |
  | Wrist | 10 | 3 | **3** |

- **Integral gain is unusable.** Even I=2 oscillates: the integral builds while friction holds the joint, it breaks free, overshoots, and repeats. P up to 80 was stable on every joint, at rest and after moves.
- **Gravity feedforward.** Each loaded joint's sag is fitted as a linear function of its gravity lever arms (`motion.GravityModel`) and added to the joint commands. On poses not used for fitting, at P=80: 1.58 mm off without feedforward → **0.72 mm** with it (height error −1.37 → +0.07 mm). Stock control lands ~3 mm off.
- **To use it:**
  ```python
  from roarm import motion, tuning
  gains, gravity = tuning.load()
  tuning.apply_gains(arm, gains)   # until the arm restarts
  motion.move_tcp_joints(arm, a, b, pitch, roll, grip, rate, gravity=gravity)
  ```
- **Limits:**
  - The model was fitted for the gripper pointing down, x 160–235 mm, z −65..−15 mm, with poses reached from above.
  - Higher P also means the arm pushes up to 5× harder against anything that blocks it.
  - The notebook still uses stock gains.

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

- **Reading joints over WiFi:**
  - An HTTP `{"T":105}` request returns only `{"ok":1}`. The actual reply arrives on the WebSocket at `ws://<ip>/ws`, whose handler ignores anything sent into it.
  - After **every** queued command, moves included, the arm rebroadcasts its *last* position report. A script that streams moves and then reads will see stale positions unless it clears the backlog first. `arm.py` does that and takes the last report to arrive.
  - Some replies (`T:405`) contain bytes that aren't valid text, which makes Python's `websockets` drop the connection. Read frames as bytes (`recv(decode=False)`).
  - Readings take about 100–300 ms; ping alone averages ~100 ms, probably the ESP32's WiFi power saving, which only firmware can turn off. The notebook and the tests default to USB.
- **USB:** set RTS and DTR low before opening the port, or the ESP32 resets. `arm.py` does this.
- **Measured sag (2026-10-11, USB, `square_dips.py`, gripper straight down around x=180–200, z=−60):** the tip settles 2.5–4.2 mm below the commanded height, and the gripper pitch 0.9–1.3° past vertical. These come from the arm's own joint sensors, so they show how far the joints fall short of their targets, not flex or calibration error.
- **Why the arm dips when a move starts:**
  - The servos hold position with a small sag under gravity.
  - Every `T:105` reading (the web app sends them constantly) overwrites the firmware's stored "last position" with the measured, sagged pose.
  - `T:104` smooth moves start from that stored position, so the first step drops the target by the sag.
  - For smooth motion, plan the path yourself and stream small `T:1041` steps, which have no firmware smoothing, from your own targets. See `tmp_scripts/square_dips.py`.
- **Coordinates:** x is forward and +y is the arm's own left (the firmware's "base + → left"). z = 0 is the shoulder pivot, about 126 mm above the base (`ARM_L1_LENGTH_MM`), not the table. `tit` is the gripper's pitch in the world frame (elbow + shoulder + wrist − π/2), which is what `T:1041` expects as `t`.
- **`{"T":405}` scrambles** its password and hotspot-name fields. It's a display bug; the saved settings are fine. The garbage can include characters that are illegal in JSON, so parse its reply with `json.loads(line, strict=False)`.
- **The firmware saves a corrupted WiFi password.** `T:406` and `T:407` both write garbage into `sta_password` in `/wifiConfig.json`, so at startup the arm fails to join ("STA connection timeout") and falls back to hotspot-only.
  - Workaround: the arm runs `boot.mission` at every startup, and this arm's has the connect command added as step 1:
    ```
    {"T":222,"name":"boot","step":"{\"T\":404,\"ap_ssid\":\"RoArm-M3\",\"ap_password\":\"12345678\",\"sta_ssid\":\"<network>\",\"sta_password\":\"<password>\"}"}
    ```
  - Check it with `{"T":221,"name":"boot"}`. If the WiFi password changes, delete the step with `{"T":231,"name":"boot","stepNum":1}` and add a new one.
- **The arm doesn't rejoin WiFi on its own after a dropout while running.** `{"T":405}` then shows `"ip":"0.0.0.0"`. Restart it, or send the `T:404` command over USB.
- **Restarting needs the USB cable unplugged too.** USB powers the controller, so the power switch alone only cuts the motors. With USB connected, a reset pulse also restarts it: RTS high for 0.15 s with DTR low.
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
