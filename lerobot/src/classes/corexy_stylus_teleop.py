# lerobot/src/classes/corexy_stylus_teleop.py  (replace entire file)

from __future__ import annotations
import logging, threading, time
from typing import Dict, Tuple

import cv2, numpy as np

from ...common.cameras import make_cameras_from_configs
from ...common.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from ...common.teleoperators import Teleoperator
from .corexy_stylus_teleop_config import CoreXYStylusTeleopConfig

logger = logging.getLogger(__name__)


class CoreXYStylusTeleop(Teleoperator):
    """
    Streams the iPad HDMI feed, lets a user click in the window, and converts
    pixel positions into (x_mm, y_mm, tap) actions.
    """
    config_class = CoreXYStylusTeleopConfig
    name         = "corexy_stylus_teleop"

    # ── interface spec ──────────────────────────────────────────────────────
    def action_features(self) -> Dict[str, type]:
        return {"x": float, "y": float, "tap": int}

    @property
    def feedback_features(self) -> Dict[str, type]:
        return {}

    # ── init ───────────────────────────────────────────────────────────────
    def __init__(self, config: CoreXYStylusTeleopConfig):
        super().__init__(config)
        self.config   = config
        self.cameras  = make_cameras_from_configs(config.cameras)

        self._latest_click_px: Tuple[int, int] | None = None
        self._latest_frame   : np.ndarray | None      = None
        self._roi            : np.ndarray | slice | None = None
        self._roi_w_h        : Tuple[int, int] | None    = None

        self._lock = threading.Lock()          # protects _latest_click_px/_frame
        self._running = False                  # turned on in connect()

        self._is_connected  = False
        self._is_calibrated = False

    # ── status props ────────────────────────────────────────────────────────
    @property
    def is_connected(self) -> bool:
        return self._is_connected and all(cam.is_connected for cam in self.cameras.values())

    @property
    def is_calibrated(self) -> bool:
        return self._is_calibrated

    # teleop doesn’t need calibration
    def calibrate(self) -> None: self._is_calibrated = True
    def configure(self) -> None: pass

    # ── connect / disconnect ───────────────────────────────────────────────
    def connect(self, calibrate: bool = False) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        logger.info("Connecting %d camera(s)…", len(self.cameras))
        for cam in self.cameras.values():
            cam.connect()

        # single frame → ROI
        frame  = next(iter(self.cameras.values())).read()
        self._roi = self._compute_roi(frame)
        logger.info("ROI computed: %s", self._roi)

        self._running      = True
        self._is_connected = True

        # background capture thread
        threading.Thread(target=self._capture_loop, daemon=True).start()
        # background GUI thread (creates window + pumps HighGUI)
        threading.Thread(target=self._gui_loop, daemon=True).start()

        logger.info("%s connected.", self)

    def disconnect(self) -> None:
        # if not self.is_connected:
        #     raise DeviceNotConnectedError(f"{self} is not connected")

        self._running = False      # tells worker threads to exit
        for cam in self.cameras.values():
            cam.disconnect()

        # must destroy the window from the GUI thread – we just flag running=False
        self._is_connected = False
        logger.info("%s disconnected.", self)

    # ── action output ───────────────────────────────────────────────────────
    def get_action(self) -> Dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        with self._lock:
            click = self._latest_click_px
            self._latest_click_px = None   # consume

        if click is None:
            return {"x": 0.0, "y": 0.0, "tap": 0}

        px, py = click
        w, h   = self._roi_w_h
        x_mm, y_mm = self._px_to_mm(px, w, py, h)
        return {"x": x_mm, "y": y_mm, "tap": 1}

    def send_feedback(self, feedback: Dict[str, float]) -> None:
        pass  # nothing to send yet

    # ── helpers ─────────────────────────────────────────────────────────────
    def _compute_roi(self, frame: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ys, xs = np.where(gray > 20)        # threshold cheaply removes black bars
        y0, y1 = ys.min(), ys.max()
        x0, x1 = xs.min(), xs.max()
        self._roi_w_h = (x1 - x0 + 1, y1 - y0 + 1)
        return np.s_[y0 : y1 + 1, x0 : x1 + 1]

    # TODO: rename - teleop will report normalized coords, robot will convert to mm
    def _px_to_mm(self, px: int, w: int, py: int, h: int) -> Tuple[float, float]:
        return (
            px / w,# * self.config.bed_width_mm,
            py / h # * self.config.bed_height_mm,
        )

    # ── mouse callback (runs in GUI thread) ────────────────────────────────
    def _on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            with self._lock:
                self._latest_click_px = (x, y)
                w, h = self._roi_w_h
                print(f"Mouse clicked at {x}/{w} ({x/w:.3f}), {y}/{h} ({y/h:.3f})")

    # ── worker threads ──────────────────────────────────────────────────────
    def _capture_loop(self):
        """Grabs frames from the first camera and stores the most recent one."""
        cam = next(iter(self.cameras.values()))
        while self._running:
            frame = cam.read()[self._roi]
            with self._lock:
                self._latest_frame = frame
            time.sleep(1 / cam.fps)

    def _gui_loop(self):
        """Creates the window and pumps the HighGUI event loop."""
        cv2.namedWindow(self.config.window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow(self.config.window_name, self._roi_w_h[0], self._roi_w_h[1])
        cv2.setMouseCallback(self.config.window_name, self._on_mouse)

        while self._running:
            with self._lock:
                frame = self._latest_frame
            if frame is not None:
                cv2.imshow(self.config.window_name, frame)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:   # ESC
                self._running = False
                break
            time.sleep(0.001)

        cv2.destroyWindow(self.config.window_name)
