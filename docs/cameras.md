# Cameras for imitation learning on the RoArm-M3

## What Waveshare documents

- The arm "comes with a camera holder", and Waveshare says you can mount "a camera with the corresponding size or other cameras with an adapter" ([product page](https://www.waveshare.com/roarm-m3.htm)).
- Their own LeRobot kit ([RoArm-M3 AI Kit](https://www.waveshare.com/roarm-m3-s-ai-kit.htm)) ships **two USB cameras with holders**, but they don't publish the camera model, resolution or field of view.
- The [bracket drawing](https://files.waveshare.com/wiki/common/Camera%20mounting%20bracket%20drawing.zip) (a DXF file) has **four Ø2.4 mm holes (M2 screws) on a 21.0 × 13.5 mm rectangle**. A camera board with that hole pattern bolts straight on. For anything else (e.g. a 32 × 32 or 38 × 38 mm board), print an adapter; the arm's 3D model is in the [STEP files](https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_STEP_260310.zip).
- The arm's frame is two 1020 aluminium extrusions, so a fixed camera can also clamp to the frame with boat nuts and M4 screws.

## Use two cameras

| Camera | Job | Why |
|---|---|---|
| **Wrist** (on the gripper) | Final approach and grasp | A small electrode fills the frame up close. This is the camera that makes precise picks work. |
| **Front/overhead** (fixed) | Where is the electrode on the table | Gives the policy the big picture before the wrist camera can see the target. |

The notebook is already set up for two (`CAMERAS = {"wrist": 0, "front": 1}`). One camera works too; just delete an entry.

## What matters (roughly in order)

1. **Focus that never changes.** Autofocus hunting makes images differ between recording and running, which confuses the policy. Pick fixed-focus or manual-focus (M12 lens) cameras. Set the wrist camera's focus for roughly 5–20 cm.
2. **Exposure and white balance you can lock.** Same reason. The author of a RoArm-M3 LeRobot plugin freezes these after connecting so "record and eval see the same images" ([lerobot_robot_roarm_m3](https://github.com/metrox-eth/lerobot_robot_roarm_m3)).
3. **MJPG at 640×480, 30 fps.** Two cameras on USB 2.0 need compressed MJPG, which `roarm/camera.py` asks for. Frames are shrunk to 320×240 before saving, so capturing above 640×480 gains little.
4. **Wide view on the wrist (~90–120°)**, so the electrode stays in frame as the gripper closes in.
5. **Light and flexible.** Payload is only 200 g at 0.5 m. Use a small board camera (around 10–20 g) and a thin, flexible cable. Leave slack at every joint and strain-relieve the cable so it can't snag or drag the wrist.
6. **Global shutter (nice to have)** stops motion blur and the "skew" rolling-shutter cameras show while the wrist moves.

**Lighting matters as much as the camera.** Use a fixed desk lamp with diffuse light (electrodes can glare) and a matte background that contrasts with the electrodes. Record and run under the same light.

## Options

| | Camera | Notes |
|---|---|---|
| **Wrist, cheap** | Small UVC USB board camera, M12 lens, 90–120°, MJPG | Generic "USB camera module" boards from ELP, Arducam, etc. Check the hole pattern against 21 × 13.5 mm M2, or print an adapter. |
| **Wrist, better** | [Arducam B0385](https://www.arducam.com/100fps-global-shutter-color-usb-camera-board-1mp-ov9782-uvc-webcam-module-with-low-distortion-m12-lens-without-microphones-for-computer-laptop-android-device-and-raspberry-pi-arducam.html): OV9782, colour **global shutter**, 1280×800, MJPG up to 100 fps, M12 lens, 38 × 38 mm | Stock lens is 70°; an M12 lens swap gets wider. Needs an adapter for the bracket. |
| **Wrist, research-grade** | [Intel RealSense D405](https://www.generationrobots.com/en/403953-intel-realsense-depth-camera-d405.html): 42 × 42 × 23 mm, 60 g, global shutter, depth from 7 cm, roughly $340–440 | The wrist camera on ALOHA 2 ([paper](https://arxiv.org/pdf/2405.02292)). 60 g is a big share of this arm's payload, and ACT doesn't use depth by default, so it's overkill here. |
| **Front** | Any good UVC webcam where autofocus can be turned off, e.g. Logitech C920/C922 | The original ALOHA used Logitech webcams. Mount it rigidly: if it moves between recording and running, the policy breaks. |

## Before buying, check

- [ ] It's UVC (plug-and-play on Windows) and supports MJPG
- [ ] Fixed or manual focus
- [ ] Mounting: matches 21 × 13.5 mm M2, or you're OK printing an adapter
- [ ] Cable length reaches from the gripper to the PC with slack at every joint (add a USB 2.0 extension if needed)
