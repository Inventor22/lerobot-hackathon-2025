# lerobot/src/classes/corexy_stylus_teleop_config.py
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict

import cv2

from ...cameras import CameraConfig, ColorMode
from ...cameras.opencv import OpenCVCameraConfig
from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("corexy_leader")
@dataclass
class CoreXYLeaderConfig(TeleoperatorConfig):
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
    window_name: str = "corexy_leader"
