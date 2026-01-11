"""Tests for model classes."""

import pytest
import numpy as np
from trajectory_sim.models.coefficient_models import SimpleAeroModel
from trajectory_sim.models.atmosphere import ExponentialAtmosphere, NoWind
from trajectory_sim.models.thrust import NoThrust, ConstantThrust


def test_simple_aero_model():
    """Test simple aerodynamic model."""
    model = SimpleAeroModel(Cx0=0.5, Cz_alpha=5.0, Cm_alpha=-2.0)
    
    coeffs = model.evaluate(
        alpha=0.1,  # 0.1 rad
        beta=0.0,
        mach=0.5,
        control_p=0.0,
        control_q=0.0,
        control_r=0.0,
        control_i=0.0,
    )
    
    assert "Cx" in coeffs
    assert "Cz" in coeffs
    assert "Cmy" in coeffs
    assert np.isclose(coeffs["Cz"], 5.0 * 0.1)  # Cz_alpha * alpha


def test_exponential_atmosphere():
    """Test exponential atmosphere model."""
    atm = ExponentialAtmosphere(wind_model=NoWind())
    
    # Sea level
    state = atm.get_atmosphere(0.0)
    assert state["density"] > 0
    assert state["pressure"] > 0
    assert state["temperature"] > 0
    assert state["speed_of_sound"] > 0
    
    # High altitude (should have lower density)
    state_high = atm.get_atmosphere(10000.0)
    assert state_high["density"] < state["density"]


def test_constant_thrust():
    """Test constant thrust model."""
    thrust = ConstantThrust(
        thrust_vector=(1000.0, 0.0, 0.0),
        mass_flow=-1.0,
    )
    
    assert np.allclose(thrust.get_thrust(0.0), [1000.0, 0.0, 0.0])
    assert thrust.get_mass_flow_rate(0.0) == -1.0
    assert thrust.is_active(0.0) is True


def test_no_thrust():
    """Test no thrust model."""
    thrust = NoThrust()
    
    assert np.allclose(thrust.get_thrust(0.0), [0.0, 0.0, 0.0])
    assert thrust.get_mass_flow_rate(0.0) == 0.0
    assert thrust.is_active(0.0) is False
