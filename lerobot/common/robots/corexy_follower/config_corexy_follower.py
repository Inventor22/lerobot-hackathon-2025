# lerobot/src/classes/corexy_stylus_robot_config.py

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict

import cv2

from lerobot.common.cameras import CameraConfig
from lerobot.common.cameras.opencv import OpenCVCameraConfig
from lerobot.common.robots.config import RobotConfig


@RobotConfig.register_subclass("corexy_follower")
@dataclass
class CoreXYFollowerConfig(RobotConfig):
    """Config dataclass for the Core-XY stylus rig."""
    port: str = "COM5"
    bed_width_mm: float = 260.0   # X travel
    bed_height_mm: float = 198.0  # Y travel
    cameras: Dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "ipad_hdmi": OpenCVCameraConfig(
                index_or_path=0,
                fps=30,
                width=1920,
                height=1080,
                backend=cv2.CAP_DSHOW
            )
        }
    )
