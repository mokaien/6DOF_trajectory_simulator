"""Tests for State class."""

import pytest
import numpy as np
from trajectory_sim.core.state import State


def test_state_initialization():
    """Test state initialization."""
    state = State(
        pos_I=[1.0, 2.0, 3.0],
        vel_I=[4.0, 5.0, 6.0],
        quat_BI=[1.0, 0.0, 0.0, 0.0],
        omega_B=[0.0, 0.0, 0.0],
        mass=100.0,
    )
    
    assert np.allclose(state.pos_I, [1.0, 2.0, 3.0])
    assert np.allclose(state.vel_I, [4.0, 5.0, 6.0])
    assert np.allclose(state.quat_BI, [1.0, 0.0, 0.0, 0.0])
    assert state.mass == 100.0


def test_state_copy():
    """Test state copying."""
    state1 = State(
        pos_I=[1.0, 2.0, 3.0],
        vel_I=[4.0, 5.0, 6.0],
        quat_BI=[1.0, 0.0, 0.0, 0.0],
        omega_B=[0.0, 0.0, 0.0],
        mass=100.0,
    )
    
    state2 = state1.copy()
    
    assert np.allclose(state1.pos_I, state2.pos_I)
    assert state1.mass == state2.mass
    
    # Modify copy and verify original unchanged
    state2.mass = 200.0
    assert state1.mass == 100.0
    assert state2.mass == 200.0


def test_quaternion_normalization():
    """Test quaternion normalization."""
    # Unnormalized quaternion
    state = State(
        pos_I=[0, 0, 0],
        vel_I=[0, 0, 0],
        quat_BI=[2.0, 0.0, 0.0, 0.0],  # Should normalize to [1, 0, 0, 0]
        omega_B=[0, 0, 0],
        mass=100.0,
    )
    
    assert np.allclose(state.quat_BI, [1.0, 0.0, 0.0, 0.0])


def test_get_speed():
    """Test speed calculation."""
    state = State(
        pos_I=[0, 0, 0],
        vel_I=[3.0, 4.0, 0.0],  # Speed should be 5.0
        quat_BI=[1, 0, 0, 0],
        omega_B=[0, 0, 0],
        mass=100.0,
    )
    
    assert np.isclose(state.get_speed(), 5.0)
