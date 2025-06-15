# lerobot/src/classes/corexy_stylus_robot.py

import logging
import time
from functools import cached_property
from typing import Any, Dict

from lerobot.common.cameras.utils import make_cameras_from_configs
from lerobot.common.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.common.robots import Robot
from lerobot.common.motors.marlin.marlin_gcode_controller import MarlinGCodeController
from lerobot.common.robots.corexy_follower.config_corexy_follower import CoreXYFollowerConfig

import cv2
import numpy as np

logger = logging.getLogger(__name__)


class CoreXYFollower(Robot):
    """
    LeRobot-compatible wrapper around the Core-XY iPad tapping rig.
    Action:  {'x': float, 'y': float, 'tap': int}
    Obs:     {'x': float, 'y': float}
    """

    config_class = CoreXYFollowerConfig
    name = "corexy_follower"

    # ── init / connection ────────────────────────────────────────────────────
    def __init__(self, config: CoreXYFollowerConfig):
        super().__init__(config)

        self.config = config

        self.bus = MarlinGCodeController(
            port=config.port,
            baudrate=getattr(config, 'baudrate', 115200),
            timeout=getattr(config, 'timeout', 1.0)
        )

        self.cameras = make_cameras_from_configs(config.cameras)

        self._last_action: Dict[str, Any] = {"x": 0.0, "y": 0.0, "tap": 0}
        self._calibrated: bool = False

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {"x": float, "y": float, "tap": int }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras
        }
    
    @cached_property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")
        
        self.bus.connect()
        if not self.is_calibrated and calibrate:
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        logger.info(f"{self} connected.")
        
    @property
    def is_calibrated(self) -> bool:
        return self._calibrated
    
    def calibrate(self) -> None:
        # Sensorless homing
        self.bus.write("G28 X Y")
        self._calibrated = True

    def configure(self) -> None:
        cam = next(iter(self.cameras.values()))
        # Remove black perimeter bars from iPad screen mirror
        frame = cam.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        thr = 20
        ys, xs = np.where(gray > thr)

        y0, y1 = ys.min(), ys.max()
        x0, x1 = xs.min(), xs.max()
        self.roi = np.s_[y0:y1+1, x0:x1+1]
        self.Hc, self.Wc = y1 - y0 + 1, x1 - x0 + 1

    def setup_motors(self) -> None:
        # self.calibrate does sensorless homing
        pass

    def get_observation(self) -> Dict[str, Any]:
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected.")

        # Read X/Y/Tap positions
        obs_dict: Dict[str, Any] = self._last_action.copy()
        #obs_dict = {f"{motor}": val for motor, val in obs_dict.items()}

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            frame = cam.async_read()
            # Apply ROI to remove black perimeter bars
            frame = frame[self.roi]
            obs_dict[cam_key] = frame

        return obs_dict

    def send_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
        x = action["x"]
        y = action["y"]
        tap = int(action.get("tap", 0))

        # Convert normalized coords to mm
        x = x * self.config.bed_width_mm
        y = y * self.config.bed_height_mm

        # Clip to iPad screen dimensions
        x = max(0, min(x, self.config.bed_width_mm))
        y = max(0, min(y, self.config.bed_height_mm))

        # Move
        self._move(x, y)

        # Tap
        if tap:
            # Cannot tap mid-move, else tap location will be incorrect
            self._wait_for_move_complete()
            self._tap()

        action = {"x": x, "y": y, "tap": tap}
        self._last_action.update(action)
        return action
    
    def disconnect(self):
        self.bus.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")

    # Helper methods
    def _move(self, x: int, y: int, speed: int = 3000) -> None:
        self.bus.write(f"G0 X{x} Y{y} F{speed}")

    def _wait_for_move_complete(self) -> None:
        self.bus.write("M400")

    def _tap(self, tap_time=300):
        # solenoid on pin 41
        self.bus.write('M42 P41 S255')
        time.sleep(tap_time / 1000)
        self.bus.write('M42 P41 S0')

        # Dampening pulse, to avoid stylus from bouncing/double tapping
        time.sleep(0.005)        
        self.bus.write('M42 P41 S255')
        time.sleep(0.005)
        self.bus.write('M42 P41 S0')