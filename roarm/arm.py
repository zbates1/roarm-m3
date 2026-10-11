"""Talk to a RoArm-M3 over USB serial or WiFi using its JSON commands.

    arm = RoArm()                 # find the arm on USB automatically
    arm = RoArm("COM14")          # a specific serial port
    arm = RoArm("192.168.1.203")  # over WiFi

Joint order everywhere: base, shoulder, elbow, wrist, roll, gripper (radians).
"""
import json
import time
import urllib.parse
import urllib.request

import numpy as np

JOINTS = ("base", "shoulder", "elbow", "wrist", "roll", "gripper")

# Joint limits from the Waveshare wiki (radians). Gripper: 3.14 = closed, smaller = open.
LOWER = np.array([-3.14, -1.57, 0.00, -1.57, -3.14, 1.08])
UPPER = np.array([3.14, 1.57, 3.14, 1.57, 3.14, 3.14])

# The pose the arm goes to on power-up (T:100).
HOME = np.array([0.0, 0.0, 1.57, 0.0, 0.0, 3.14])

# T:105 feedback keys, in JOINTS order.
_FEEDBACK_KEYS = ("b", "s", "e", "t", "r", "g")


class _Serial:
    def __init__(self, port):
        import serial

        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = 115200
        self.ser.timeout = 0.5
        # Keep RTS/DTR low before opening, or the ESP32 resets.
        self.ser.rts = False
        self.ser.dtr = False
        self.ser.open()
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    def send(self, cmd):
        self.ser.write((json.dumps(cmd) + "\n").encode())

    def query(self, cmd, reply_type, timeout=1.0):
        self.ser.reset_input_buffer()
        self.send(cmd)
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self.ser.readline().decode(errors="replace").strip()
            if line.startswith('{"T":%d' % reply_type):
                return json.loads(line)
        raise TimeoutError(f"no T:{reply_type} reply on {self.ser.port}")

    def close(self):
        self.ser.close()


class _Wifi:
    """Commands go over HTTP; the arm sends replies back on its WebSocket."""

    def __init__(self, host):
        from websockets.sync.client import connect

        self.host = host
        self.ws = connect(f"ws://{host}/ws", open_timeout=5, ping_interval=None)

    def send(self, cmd):
        url = f"http://{self.host}/js?json=" + urllib.parse.quote(json.dumps(cmd))
        urllib.request.urlopen(url, timeout=2).read()

    def query(self, cmd, reply_type, timeout=1.0):
        self.send(cmd)
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                msg = self.ws.recv(timeout=max(0.01, deadline - time.time()))
            except TimeoutError:
                break
            if msg.startswith('{"T":%d' % reply_type):
                return json.loads(msg)
        raise TimeoutError(f"no T:{reply_type} reply from {self.host}")

    def close(self):
        self.ws.close()


def find_usb_port():
    """Return the serial port the arm answers on.

    The driver board has two USB-C ports with identical USB chips, so we ask
    each one for feedback and keep the one that replies.
    """
    from serial.tools import list_ports

    for p in list_ports.comports():
        if "CP210" not in (p.description or "") and p.vid != 0x10C4:
            continue
        try:
            link = _Serial(p.device)
            try:
                link.query({"T": 105}, 1051)
                return p.device
            finally:
                link.close()
        except Exception:
            continue
    raise RuntimeError(
        "Arm not found on USB. Check the 12 V switch is on, use the USB-C port "
        "that talks to the arm, and plug into a USB port on the back of the PC."
    )


class RoArm:
    def __init__(self, target=None):
        if target is None:
            target = find_usb_port()
        is_serial = target.upper().startswith("COM") or target.startswith("/dev/")
        self.link = _Serial(target) if is_serial else _Wifi(target)
        self.target = target

    def state(self):
        """Current joint angles (radians), in JOINTS order."""
        fb = self.link.query({"T": 105}, 1051)
        return np.array([fb[k] for k in _FEEDBACK_KEYS], dtype=np.float32)

    def move(self, joints, spd=0, acc=10):
        """Send all six joint targets (radians). spd 0 = max speed, in steps/s (4096 per turn)."""
        j = [round(float(x), 4) for x in np.clip(np.asarray(joints, dtype=float), LOWER, UPPER)]
        self.link.send({
            "T": 102, "base": j[0], "shoulder": j[1], "elbow": j[2],
            "wrist": j[3], "roll": j[4], "hand": j[5], "spd": spd, "acc": acc,
        })

    def move_and_wait(self, joints, spd=600, acc=20, tol=0.05, timeout=8.0):
        """Move slowly to a pose and wait until the arm gets there."""
        target = np.clip(np.asarray(joints, dtype=float), LOWER, UPPER)
        self.move(target, spd=spd, acc=acc)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if np.max(np.abs(self.state() - target)) < tol:
                return
            time.sleep(0.1)

    def go_home(self):
        self.move_and_wait(HOME)

    def torque(self, on):
        """Turn all joint motors on or off. Off = the arm goes limp, so hold it first."""
        self.link.send({"T": 210, "cmd": 1 if on else 0})

    def hold(self):
        """Lock the arm where it is now (turns torque back on without a jump)."""
        self.move(self.state(), spd=300, acc=10)

    def close(self):
        self.link.close()
