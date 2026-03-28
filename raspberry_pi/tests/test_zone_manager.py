"""
tests/test_zone_manager.py
===========================
Tests for the Zone detection logic.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from detection.zone_manager import Zone, ZoneManager


class TestZone:

    def setup_method(self):
        # Zone: pixel box (100,100) → (400,300)
        self.zone = Zone(100, 100, 400, 300)

    def test_detection_fully_inside_zone(self):
        assert self.zone.contains_box(150, 150, 350, 250)

    def test_detection_outside_zone(self):
        assert not self.zone.contains_box(10, 10, 90, 90)

    def test_detection_partially_overlapping(self):
        # Large overlap — should trigger
        assert self.zone.contains_box(50, 50, 200, 200, overlap_threshold=0.1)

    def test_tiny_overlap_below_threshold(self):
        # Just barely touching — should NOT trigger with high threshold
        assert not self.zone.contains_box(395, 295, 500, 400, overlap_threshold=0.5)

    def test_exact_boundary_match(self):
        assert self.zone.contains_box(100, 100, 400, 300)


class TestZoneManager:

    def setup_method(self):
        self.mgr = ZoneManager([
            Zone(100, 100, 400, 300, name="shared_section")
        ])

    def test_no_detections_returns_false(self):
        assert not self.mgr.any_zone_triggered([])

    def test_detection_in_zone_returns_true(self):
        dets = [{"x1": 150, "y1": 150, "x2": 300, "y2": 250,
                 "confidence": 0.9, "label": "train"}]
        assert self.mgr.any_zone_triggered(dets)

    def test_detection_outside_zone_returns_false(self):
        dets = [{"x1": 500, "y1": 400, "x2": 600, "y2": 480,
                 "confidence": 0.9, "label": "train"}]
        assert not self.mgr.any_zone_triggered(dets)

    def test_multiple_detections_one_in_zone(self):
        dets = [
            {"x1": 500, "y1": 400, "x2": 600, "y2": 480,  # outside
             "confidence": 0.9, "label": "train"},
            {"x1": 150, "y1": 150, "x2": 300, "y2": 250,  # inside
             "confidence": 0.85, "label": "train"},
        ]
        assert self.mgr.any_zone_triggered(dets)
