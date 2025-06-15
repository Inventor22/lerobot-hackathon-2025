# lerobot/src/classes/corexy_stylus_robot.py

from __future__ import annotations
import time
from typing import Any, Dict

from ....lerobot.common.cameras import make_cameras_from_configs
from ....lerobot.common.robots import Robot

from .marlin_gcode_motor_bus import MarlinGCodeMotorBus
from .corexy_stylus_config import CoreXYStylusConfig

import cv2
import numpy as np


class CoreXYStylus(Robot):
    """
    LeRobot-compatible wrapper around the Core-XY iPad tapping rig.
    Action:  {'x': int, 'y': int, 'tap': int}
    Obs:     {'x': int, 'y': int}
    """

    config_class = CoreXYStylusConfig
    name = "corexy_stylus_robot"

    # ── init / connection ────────────────────────────────────────────────────
    def __init__(self, config: CoreXYStylusConfig):
        super().__init__(config)
        self.bus = MarlinGCodeMotorBus(port=config.port)
        self.cameras = make_cameras_from_configs(config.cameras)
        self._last_action: Dict[str, int] = {"x": 0, "y": 0}
        self._calibrated: bool = False

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    def connect(self, calibrate: bool = False) -> None:
        self.bus.connect()

        for cam in self.cameras.values():
            cam.connect()

        if not self.is_calibrated and calibrate:
            self.calibrate()

    def disconnect(self) -> None:
        self.bus.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()

    # ── API required by LeRobot ─────────────────────────────────────────────
    @property
    def _motors_ft(self) -> dict[str, type]:
        return {
            "x": int,
            "y": int,
            "tap": int,
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras
        }

    @property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}
        
    @property
    def is_calibrated(self) -> bool:
        return self._calibrated

    # def action_features(self) -> Dict[str, type]:
    #     return {"x": int, "y": int, "tap": int}

    def action_features(self) -> dict:
        return self._motors_ft

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
        x = int(action["x"])
        y = int(action["y"])
        tap = int(action.get("tap", 0))

        # Clip to iPad screen dimensions
        x = max(0, min(x, self.config.bed_width_mm))
        y = max(0, min(y, self.config.bed_height_mm))

        # Move
        self.move(x, y)

        # Tap
        if tap:
            # Cannot tap mid-move, else tap location will be incorrect
            self.wait_for_move_complete()
            self.tap()

        self._last_action.update({"x": x, "y": y, "tap": tap})
        return {"x": x, "y": y, "tap": tap}
    
    # Helper methods
    def move(self, x: int, y: int, speed: int = 3000) -> None:
        self.bus.send_command(f"G0 X{x} Y{y} F{speed}")

    def wait_for_move_complete(self) -> None:
        self.bus.send_command("M400")

    def tap(self, tap_time=300):
        # solenoid on pin 41
        self.send_command('M42 P41 S255')
        time.sleep(tap_time / 1000)
        self.send_command('M42 P41 S0')

        # Dampening pulse, to avoid stylus from bouncing/double tapping
        time.sleep(0.005)        
        self.send_command('M42 P41 S255')
        time.sleep(0.005)
        self.send_command('M42 P41 S0')

    def calibrate(self) -> None:
        # Sensorless homing
        self.bus.send_command("G28 X Y")
        self._calibrated = True

    def configure(self) -> None:
        for cam in self.cameras.values():
            # Remove black perimeter bars from iPad screen mirror
            frame = cam.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            thr = 20
            ys, xs = np.where(gray > thr)

            y0, y1 = ys.min(), ys.max()
            x0, x1 = xs.min(), xs.max()
            self.roi = np.s_[y0:y1+1, x0:x1+1]
            self.Hc, self.Wc = y1 - y0 + 1, x1 - x0 + 1