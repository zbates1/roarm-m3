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
    """Commands go over HTTP; the arm sends replies back on its WebSocket.

    Two firmware quirks shape this:
    - After every queued command (moves included), the arm rebroadcasts its
      *last* T:1051 report, so the socket fills with stale copies. A query
      clears the backlog, then takes the last report to arrive once things
      go quiet: the arm handles commands in order, so ours comes last.
    - Some replies (e.g. T:405) contain bytes that aren't valid UTF-8, so
      frames are read as bytes; decoding them as text would drop the link.
    """

    def __init__(self, host):
        from websockets.sync.client import connect

        self.host = host
        self.ws = connect(f"ws://{host}/ws", open_timeout=5, ping_interval=None, legacy=True)

    def send(self, cmd):
        url = f"http://{self.host}/js?json=" + urllib.parse.quote(json.dumps(cmd))
        urllib.request.urlopen(url, timeout=2).read()

    def _recv(self, timeout):
        try:
            return self.ws.recv(timeout=timeout, decode=False).decode(errors="replace")
        except TimeoutError:
            return None

    def query(self, cmd, reply_type, timeout=1.0, quiet=0.06, attempts=2):
        # The arm's web server sometimes drops a broadcast, so ask again once.
        prefix = '{"T":%d' % reply_type
        for _ in range(attempts):
            while self._recv(0) is not None:
                pass
            self.send(cmd)
            reply, last, deadline = None, 0.0, time.time() + timeout
            while time.time() < deadline:
                wait = last + quiet - time.time() if reply else deadline - time.time()
                if reply and wait <= 0:
                    break
                msg = self._recv(max(0.005, wait))
                if msg and msg.startswith(prefix):
                    reply, last = msg, time.time()
            if reply is not None:
                return json.loads(reply, strict=False)
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

    def feedback(self):
        """Everything T:105 reports: joints b s e t r g (rad), gripper tip x y z (mm),
        tit (gripper pitch, rad), loads tB..tG, torque flags torswitchB..G, v (0.01 V)."""
        return self.link.query({"T": 105}, 1051)

    def state(self):
        """Current joint angles (radians), in JOINTS order."""
        fb = self.feedback()
        return np.array([fb[k] for k in _FEEDBACK_KEYS], dtype=np.float32)

    def move_tcp(self, x, y, z, pitch, roll, gripper):
        """Jump the gripper tip to x, y, z (mm) with no smoothing (T:1041).

        pitch is the gripper's angle in the world frame (feedback "tit").
        For smooth motion, call this many times with small steps.
        """
        self.link.send({
            "T": 1041, "x": round(float(x), 2), "y": round(float(y), 2), "z": round(float(z), 2),
            "t": round(float(pitch), 4), "r": round(float(roll), 4), "g": round(float(gripper), 4),
        })

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
