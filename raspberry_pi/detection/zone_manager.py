"""
Zone Manager
=============
Defines the shared track section as a rectangular region-of-interest (ROI)
in the camera frame, and checks whether a detection bounding box overlaps it.

How to calibrate:
    Run calibrate_zone.py once — it opens the camera and lets you draw the
    zone with your mouse. Saves zone coords to config.py automatically.
"""

import logging
from dataclasses import dataclass
from typing import Tuple, List

logger = logging.getLogger(__name__)


@dataclass
class Zone:
    """A rectangular zone in pixel coordinates."""
    x1: int
    y1: int
    x2: int
    y2: int
    name: str = "shared_zone"

    def contains_box(self, box_x1: int, box_y1: int, box_x2: int, box_y2: int,
                     overlap_threshold: float = 0.2) -> bool:
        """
        Returns True if the bounding box overlaps this zone by at least
        overlap_threshold fraction of the box area.

        Args:
            box_x1, box_y1, box_x2, box_y2: detection bounding box corners
            overlap_threshold: 0.0–1.0, fraction of box that must be inside zone
        """
        # Intersection rectangle
        ix1 = max(self.x1, box_x1)
        iy1 = max(self.y1, box_y1)
        ix2 = min(self.x2, box_x2)
        iy2 = min(self.y2, box_y2)

        if ix2 <= ix1 or iy2 <= iy1:
            return False  # No overlap

        intersection_area = (ix2 - ix1) * (iy2 - iy1)
        box_area = max(1, (box_x2 - box_x1) * (box_y2 - box_y1))
        overlap = intersection_area / box_area
        return overlap >= overlap_threshold

    def as_tuple(self) -> Tuple[int, int, int, int]:
        return (self.x1, self.y1, self.x2, self.y2)


class ZoneManager:
    """Manages one or more detection zones on the camera frame."""

    def __init__(self, zones: List[Zone]):
        self.zones = zones

    def any_zone_triggered(self, detections: list) -> bool:
        """
        Returns True if any detection bounding box overlaps any zone.

        Args:
            detections: list of dicts with keys x1, y1, x2, y2, confidence, label
        """
        for det in detections:
            for zone in self.zones:
                if zone.contains_box(det["x1"], det["y1"], det["x2"], det["y2"]):
                    logger.info(
                        f"Train detected in zone '{zone.name}' "
                        f"(conf={det.get('confidence', 0):.2f})"
                    )
                    return True
        return False

    def draw_zones(self, frame):
        """Draw zone rectangles on an OpenCV frame (for debug display)."""
        import cv2
        for zone in self.zones:
            cv2.rectangle(
                frame,
                (zone.x1, zone.y1),
                (zone.x2, zone.y2),
                (0, 165, 255),  # Orange
                2,
            )
            cv2.putText(
                frame, zone.name,
                (zone.x1, zone.y1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (0, 165, 255), 2,
            )
        return frame
