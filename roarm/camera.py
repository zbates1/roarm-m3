"""USB cameras via OpenCV, read on a background thread so frames are never stale."""
import sys
import threading
import time

import cv2
import numpy as np

_BACKEND = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY


def find_cameras(max_index=6):
    """Indices of the cameras OpenCV can open."""
    found = []
    for i in range(max_index):
        cap = cv2.VideoCapture(i, _BACKEND)
        if cap.isOpened() and cap.read()[0]:
            found.append(i)
        cap.release()
    return found


class Camera:
    def __init__(self, index, width=640, height=480, fps=30):
        self.cap = cv2.VideoCapture(index, _BACKEND)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera {index}")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.index = index
        self._frame = None
        self._lock = threading.Lock()
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()
        deadline = time.time() + 3
        while self._frame is None and time.time() < deadline:
            time.sleep(0.02)
        if self._frame is None:
            raise RuntimeError(f"Camera {index} opened but sent no frames")

    def _loop(self):
        while self._running:
            ok, bgr = self.cap.read()
            if ok:
                with self._lock:
                    self._frame = bgr

    def read(self, size=None):
        """Latest frame as RGB uint8 (H, W, 3). size=(height, width) resizes it."""
        with self._lock:
            bgr = self._frame.copy()
        if size is not None:
            bgr = cv2.resize(bgr, (size[1], size[0]), interpolation=cv2.INTER_AREA)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def close(self):
        self._running = False
        time.sleep(0.1)
        self.cap.release()
