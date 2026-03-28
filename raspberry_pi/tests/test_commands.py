"""
tests/test_commands.py
=======================
Unit tests for the LionChief command builder.
Run: pytest tests/
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from lionchief.commands import (
    STOP_CMD, MEDIUM_CMD, FORWARD_CMD, REVERSE_CMD,
    HORN_ON_CMD, HORN_OFF_CMD, BELL_ON_CMD, BELL_OFF_CMD,
    LIGHT_ON_CMD, LIGHT_OFF_CMD,
    build_speed_cmd, build_volume_cmd,
    SERVICE_UUID, CHARACTERISTIC_UUID,
)


class TestCommandBytes:
    """Verify every command byte is correct against Property404's decoded protocol."""

    def test_stop_command(self):
        assert list(STOP_CMD) == [0x00, 0x45, 0x00], "STOP must be [0x00, 0x45, 0x00]"

    def test_medium_speed(self):
        assert list(MEDIUM_CMD) == [0x00, 0x45, 0x07]

    def test_forward(self):
        assert list(FORWARD_CMD) == [0x00, 0x46, 0x01]

    def test_reverse(self):
        assert list(REVERSE_CMD) == [0x00, 0x46, 0x02]

    def test_horn_on(self):
        assert list(HORN_ON_CMD) == [0x00, 0x48, 0x01]

    def test_horn_off(self):
        assert list(HORN_OFF_CMD) == [0x00, 0x48, 0x00]

    def test_bell_on(self):
        assert list(BELL_ON_CMD) == [0x00, 0x47, 0x01]

    def test_bell_off(self):
        assert list(BELL_OFF_CMD) == [0x00, 0x47, 0x00]

    def test_lights_on(self):
        assert list(LIGHT_ON_CMD) == [0x00, 0x51, 0x01]

    def test_lights_off(self):
        assert list(LIGHT_OFF_CMD) == [0x00, 0x51, 0x00]


class TestBuildSpeedCmd:

    def test_zero_speed_is_stop(self):
        assert list(build_speed_cmd(0)) == [0x00, 0x45, 0x00]

    def test_max_speed_clamped(self):
        cmd = build_speed_cmd(100)
        assert cmd[2] == 31, "Speed above 31 should clamp to 31"

    def test_negative_clamped_to_zero(self):
        cmd = build_speed_cmd(-5)
        assert cmd[2] == 0

    def test_arbitrary_speed(self):
        cmd = build_speed_cmd(15)
        assert list(cmd) == [0x00, 0x45, 0x0F]


class TestBuildVolumeCmd:

    def test_volume_range(self):
        for v in range(8):
            cmd = build_volume_cmd(v)
            assert cmd[2] == v

    def test_volume_clamped_high(self):
        cmd = build_volume_cmd(99)
        assert cmd[2] == 7

    def test_volume_clamped_low(self):
        cmd = build_volume_cmd(-1)
        assert cmd[2] == 0


class TestUUIDs:

    def test_service_uuid_matches_peter_train(self):
        """Peter confirmed this UUID from his train."""
        assert SERVICE_UUID.lower() == "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"

    def test_characteristic_uuid(self):
        assert CHARACTERISTIC_UUID.lower() == "08590f7e-db05-467e-8757-72f6faeb13d4"
