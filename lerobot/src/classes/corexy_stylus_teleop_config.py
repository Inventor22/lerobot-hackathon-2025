# lerobot/src/classes/corexy_stylus_teleop_config.py
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict

import cv2

from ...common.cameras import CameraConfig, ColorMode
from ...common.cameras.opencv import OpenCVCameraConfig
from ...common.robots import RobotConfig


@RobotConfig.register_subclass("corexy_stylus_teleop")
@dataclass
class CoreXYStylusTeleopConfig(RobotConfig):
    """
    Config for the Core-XY stylus teleoperator.
    """
    cameras: Dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "ipad_hdmi": OpenCVCameraConfig(
                index_or_path=0,
                color_mode=ColorMode.BGR,
                fps=30,
                width=1920,
                height=1080,
                backend=cv2.CAP_DSHOW
            )
        }
    )
    bed_width_mm: float = 260.0
    bed_height_mm: float = 198.0
    window_name: str = "CoreXY Teleop"
