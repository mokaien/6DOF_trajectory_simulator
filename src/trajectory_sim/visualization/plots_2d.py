"""2D plotting utilities for time history and trajectory."""

import numpy as np
import matplotlib.pyplot as plt
from typing import List, Dict, Optional
from trajectory_sim.core.state import State


def plot_time_history(
    history: Dict[str, List],
    figsize: tuple = (12, 8),
    save_path: Optional[str] = None,
):
    """Plot time history of state variables.
    
    Args:
        history: Dictionary with 'states' and 'times' keys
        figsize: Figure size
        save_path: Optional path to save figure
    """
    states = history["states"]
    times = history["times"]
    
    fig, axes = plt.subplots(3, 3, figsize=figsize)
    
    # Position
    pos_data = np.array([s.pos_I for s in states])
    axes[0, 0].plot(times, pos_data[:, 0], label="x")
    axes[0, 0].plot(times, pos_data[:, 1], label="y")
    axes[0, 0].plot(times, pos_data[:, 2], label="z")
    axes[0, 0].set_xlabel("Time (s)")
    axes[0, 0].set_ylabel("Position (m)")
    axes[0, 0].set_title("Position")
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    
    # Velocity
    vel_data = np.array([s.vel_I for s in states])
    axes[0, 1].plot(times, vel_data[:, 0], label="vx")
    axes[0, 1].plot(times, vel_data[:, 1], label="vy")
    axes[0, 1].plot(times, vel_data[:, 2], label="vz")
    axes[0, 1].set_xlabel("Time (s)")
    axes[0, 1].set_ylabel("Velocity (m/s)")
    axes[0, 1].set_title("Velocity")
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    
    # Speed
    speeds = [s.get_speed() for s in states]
    axes[0, 2].plot(times, speeds)
    axes[0, 2].set_xlabel("Time (s)")
    axes[0, 2].set_ylabel("Speed (m/s)")
    axes[0, 2].set_title("Speed")
    axes[0, 2].grid(True)
    
    # Euler angles
    euler_data = np.array([s.get_euler_angles() for s in states])
    axes[1, 0].plot(times, np.degrees(euler_data[:, 0]), label="Roll")
    axes[1, 0].plot(times, np.degrees(euler_data[:, 1]), label="Pitch")
    axes[1, 0].plot(times, np.degrees(euler_data[:, 2]), label="Yaw")
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_ylabel("Angle (deg)")
    axes[1, 0].set_title("Euler Angles")
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    
    # Angular velocity
    omega_data = np.array([s.omega_B for s in states])
    axes[1, 1].plot(times, omega_data[:, 0], label="p")
    axes[1, 1].plot(times, omega_data[:, 1], label="q")
    axes[1, 1].plot(times, omega_data[:, 2], label="r")
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].set_ylabel("Angular Velocity (rad/s)")
    axes[1, 1].set_title("Angular Velocity")
    axes[1, 1].legend()
    axes[1, 1].grid(True)
    
    # Mass
    masses = [s.mass for s in states]
    axes[1, 2].plot(times, masses)
    axes[1, 2].set_xlabel("Time (s)")
    axes[1, 2].set_ylabel("Mass (kg)")
    axes[1, 2].set_title("Mass")
    axes[1, 2].grid(True)
    
    # Altitude
    altitudes = [-s.pos_I[2] for s in states]  # Assuming NED frame
    axes[2, 0].plot(times, altitudes)
    axes[2, 0].set_xlabel("Time (s)")
    axes[2, 0].set_ylabel("Altitude (m)")
    axes[2, 0].set_title("Altitude")
    axes[2, 0].grid(True)
    
    # Trajectory (X-Y)
    axes[2, 1].plot(pos_data[:, 0], pos_data[:, 1])
    axes[2, 1].set_xlabel("X (m)")
    axes[2, 1].set_ylabel("Y (m)")
    axes[2, 1].set_title("Trajectory (X-Y)")
    axes[2, 1].grid(True)
    axes[2, 1].axis("equal")
    
    # Trajectory (X-Z)
    axes[2, 2].plot(pos_data[:, 0], pos_data[:, 2])
    axes[2, 2].set_xlabel("X (m)")
    axes[2, 2].set_ylabel("Z (m)")
    axes[2, 2].set_title("Trajectory (X-Z)")
    axes[2, 2].grid(True)
    axes[2, 2].axis("equal")
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path)
    else:
        plt.show()


def plot_trajectory_2d(
    history: Dict[str, List],
    projection: str = "xy",
    figsize: tuple = (8, 6),
    save_path: Optional[str] = None,
):
    """Plot 2D trajectory projection.
    
    Args:
        history: Dictionary with 'states' and 'times' keys
        projection: Projection plane ('xy', 'xz', or 'yz')
        figsize: Figure size
        save_path: Optional path to save figure
    """
    states = history["states"]
    pos_data = np.array([s.pos_I for s in states])
    
    fig, ax = plt.subplots(figsize=figsize)
    
    if projection == "xy":
        ax.plot(pos_data[:, 0], pos_data[:, 1])
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
    elif projection == "xz":
        ax.plot(pos_data[:, 0], pos_data[:, 2])
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Z (m)")
    elif projection == "yz":
        ax.plot(pos_data[:, 1], pos_data[:, 2])
        ax.set_xlabel("Y (m)")
        ax.set_ylabel("Z (m)")
    else:
        raise ValueError(f"Invalid projection: {projection}")
    
    ax.set_title(f"Trajectory ({projection.upper()} projection)")
    ax.grid(True)
    ax.axis("equal")
    
    if save_path:
        plt.savefig(save_path)
    else:
        plt.show()
