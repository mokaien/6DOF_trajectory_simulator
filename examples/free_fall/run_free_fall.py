"""Example: Free-fall object simulation."""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from trajectory_sim.config.vehicle import VehicleConfig
from trajectory_sim.config.mission import MissionConfig
from trajectory_sim.models.atmosphere import ExponentialAtmosphere, NoWind
from trajectory_sim.models.coefficient_models import SimpleAeroModel
from trajectory_sim.models.thrust import NoThrust
from trajectory_sim.control.controller import NullController
from trajectory_sim.core.api import SimulationConfig, SimulationRunner
from trajectory_sim.visualization.plots_2d import plot_time_history
from trajectory_sim.visualization.trajectory_3d import plot_trajectory_3d


def main():
    """Run free-fall simulation."""
    # Vehicle configuration (simple object)
    vehicle = VehicleConfig(
        mass=100.0,  # kg
        Ixx=10.0, Iyy=10.0, Izz=10.0,  # kg*m^2
        ref_area=1.0,  # m^2
        ref_length=1.0,  # m
    )
    
    # Mission configuration
    mission = MissionConfig(
        pos_I=(0.0, 0.0, 10000.0),  # Start at 10 km altitude
        vel_I=(0.0, 0.0, 0.0),  # Initial velocity zero
        euler_angles=(0.0, 0.0, 0.0),  # No initial rotation
        t_start=0.0,
        t_end=60.0,  # 60 seconds
        dt=0.01,  # 10 ms time step
        gravity=(0.0, 0.0, -9.81),  # Gravity in -Z direction
    )
    
    # Atmosphere model
    atmosphere = ExponentialAtmosphere(wind_model=NoWind())
    
    # Aerodynamic model (simple drag)
    aero_model = SimpleAeroModel(Cx0=0.8, Cz_alpha=0.0, Cm_alpha=0.0)
    
    # No thrust
    thrust = NoThrust()
    
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
    
    # Calculate terminal velocity at initial altitude
    print("Running free-fall simulation...")
    runner = SimulationRunner(config)
    
    # Calculate terminal velocity at sea level (for comparison)
    import numpy as np
    v_terminal_sea = runner.simulator.calculate_terminal_velocity(altitude=0.0)
    v_terminal_initial = runner.simulator.calculate_terminal_velocity(altitude=10000.0)
    print(f"Terminal velocity at sea level: {v_terminal_sea:.2f} m/s")
    print(f"Terminal velocity at 10 km: {v_terminal_initial:.2f} m/s")
    print()
    
    runner.run()
    
    # Get results
    results = runner.get_results()
    print(f"Simulation completed. {len(results['states'])} time steps.")
    
    # Check if object hit ground
    final_state = runner.get_final_state()
    final_altitude = -final_state.pos_I[2]
    if final_altitude <= 0:
        print("Object reached ground level.")
    
    # Plot results
    print("Plotting results...")
    plot_time_history(results)
    plot_trajectory_3d(results)
    
    # Print final state
    speed = np.linalg.norm(final_state.vel_I)
    print(f"\nFinal altitude: {final_altitude:.2f} m")
    print(f"Final speed: {speed:.2f} m/s")
    
    # Compare with terminal velocity
    if final_altitude > 0:
        v_terminal_final = runner.simulator.calculate_terminal_velocity(altitude=final_altitude)
        print(f"Terminal velocity at final altitude: {v_terminal_final:.2f} m/s")
        print(f"Speed / Terminal velocity ratio: {speed / v_terminal_final:.3f}")


if __name__ == "__main__":
    main()
