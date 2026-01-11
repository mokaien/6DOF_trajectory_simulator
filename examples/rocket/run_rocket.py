"""Example: Rocket simulation with thrust."""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from trajectory_sim.config.vehicle import VehicleConfig
from trajectory_sim.config.mission import MissionConfig
from trajectory_sim.models.atmosphere import ExponentialAtmosphere, NoWind
from trajectory_sim.models.coefficient_models import SimpleAeroModel
from trajectory_sim.models.thrust import ConstantThrust
from trajectory_sim.control.controller import NullController
from trajectory_sim.core.api import SimulationConfig, SimulationRunner
from trajectory_sim.visualization.plots_2d import plot_time_history
from trajectory_sim.visualization.trajectory_3d import plot_trajectory_3d


def main():
    """Run rocket simulation."""
    # Vehicle configuration (rocket)
    vehicle = VehicleConfig(
        mass=1000.0,  # kg
        Ixx=100.0, Iyy=100.0, Izz=10.0,  # kg*m^2
        ref_area=0.5,  # m^2
        ref_length=5.0,  # m
    )
    
    # Mission configuration
    mission = MissionConfig(
        pos_I=(0.0, 0.0, 0.0),  # Start at ground level
        vel_I=(0.0, 0.0, 0.0),  # Initial velocity zero
        euler_angles=(0.0, 0.0, 0.0),  # Pointing up
        t_start=0.0,
        t_end=30.0,  # 30 seconds
        dt=0.01,  # 10 ms time step
        gravity=(0.0, 0.0, -9.81),  # Gravity in -Z direction
    )
    
    # Atmosphere model
    atmosphere = ExponentialAtmosphere(wind_model=NoWind())
    
    # Aerodynamic model
    aero_model = SimpleAeroModel(Cx0=0.3, Cz_alpha=0.0, Cm_alpha=0.0)
    
    # Constant thrust upward
    thrust = ConstantThrust(
        thrust_vector=(0.0, 0.0, 15000.0),  # 15 kN upward
        mass_flow=-5.0,  # 5 kg/s mass loss
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
    print("Running rocket simulation...")
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
    print(f"Final mass: {final_state.mass:.2f} kg")


if __name__ == "__main__":
    main()
