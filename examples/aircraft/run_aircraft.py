"""Example: Simple aircraft simulation."""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from trajectory_sim.config.vehicle import VehicleConfig
from trajectory_sim.config.mission import MissionConfig
from trajectory_sim.models.atmosphere import ExponentialAtmosphere, ConstantWind
from trajectory_sim.models.coefficient_models import SimpleAeroModel
from trajectory_sim.models.thrust import ConstantThrust
from trajectory_sim.control.controller import NullController
from trajectory_sim.core.api import SimulationConfig, SimulationRunner
from trajectory_sim.visualization.plots_2d import plot_time_history
from trajectory_sim.visualization.trajectory_3d import plot_trajectory_3d


def main():
    """Run aircraft simulation."""
    # Vehicle configuration (aircraft)
    vehicle = VehicleConfig(
        mass=1000.0,  # kg
        Ixx=2000.0, Iyy=5000.0, Izz=6000.0,  # kg*m^2
        ref_area=20.0,  # m^2
        ref_length=5.0,  # m
    )
    
    # Mission configuration
    mission = MissionConfig(
        pos_I=(0.0, 0.0, 1000.0),  # Start at 1 km altitude
        vel_I=(100.0, 0.0, 0.0),  # Initial forward velocity
        euler_angles=(0.0, 0.1, 0.0),  # Slight pitch up
        t_start=0.0,
        t_end=60.0,  # 60 seconds
        dt=0.01,  # 10 ms time step
        gravity=(0.0, 0.0, -9.81),  # Gravity in -Z direction
    )
    
    # Atmosphere model with wind
    atmosphere = ExponentialAtmosphere(
        wind_model=ConstantWind((5.0, 0.0, 0.0))  # 5 m/s headwind
    )
    
    # Aerodynamic model (with lift)
    aero_model = SimpleAeroModel(Cx0=0.05, Cz_alpha=4.0, Cm_alpha=-1.0)
    
    # Constant thrust forward
    thrust = ConstantThrust(
        thrust_vector=(5000.0, 0.0, 0.0),  # 5 kN forward
        mass_flow=0.0,  # No mass change
    )
    
    # No control
    controller = NullController()
    
    # Create simulation configuration
    config = SimulationConfig(
        vehicle=vehicle,
        mission=mission,
        atmosphere=atmosphere,
        aero_model=aero_model,
        thrust=thrust,
        controller=controller,
    )
    
    # Run simulation
    print("Running aircraft simulation...")
    runner = SimulationRunner(config)
    runner.run()
    
    # Get results
    results = runner.get_results()
    print(f"Simulation completed. {len(results['states'])} time steps.")
    
    # Plot results
    print("Plotting results...")
    plot_time_history(results)
    plot_trajectory_3d(results)
    
    # Print final state
    final_state = runner.get_final_state()
    import numpy as np
    speed = np.linalg.norm(final_state.vel_I)
    print(f"\nFinal altitude: {-final_state.pos_I[2]:.2f} m")
    print(f"Final speed: {speed:.2f} m/s")


if __name__ == "__main__":
    main()
