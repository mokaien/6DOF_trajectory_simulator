"""Tests for configuration classes."""

import pytest
from trajectory_sim.config.vehicle import VehicleConfig
from trajectory_sim.config.mission import MissionConfig
import numpy as np


def test_vehicle_config():
    """Test vehicle configuration."""
    vehicle = VehicleConfig(
        mass=100.0,
        Ixx=10.0,
        Iyy=20.0,
        Izz=30.0,
        ref_area=1.0,
        ref_length=2.0,
    )
    
    assert vehicle.mass == 100.0
    assert vehicle.Ixx == 10.0
    assert vehicle.ref_area == 1.0


def test_mission_config_quaternion():
    """Test mission configuration with quaternion."""
    mission = MissionConfig(
        pos_I=(0, 0, 0),
        vel_I=(0, 0, 0),
        quat_BI=(1.0, 0.0, 0.0, 0.0),
    )
    
    quat = mission.get_quaternion()
    assert np.allclose(quat, [1.0, 0.0, 0.0, 0.0])


def test_mission_config_euler():
    """Test mission configuration with Euler angles."""
    mission = MissionConfig(
        pos_I=(0, 0, 0),
        vel_I=(0, 0, 0),
        euler_angles=(0.0, 0.0, 0.0),
    )
    
    quat = mission.get_quaternion()
    # Identity quaternion (approximately)
    assert np.allclose(quat, [1.0, 0.0, 0.0, 0.0], atol=1e-6)
