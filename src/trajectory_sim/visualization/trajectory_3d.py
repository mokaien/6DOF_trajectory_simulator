"""3D trajectory visualization."""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import List, Dict, Optional


def plot_trajectory_3d(
    history: Dict[str, List],
    figsize: tuple = (10, 8),
    save_path: Optional[str] = None,
):
    """Plot 3D trajectory.
    
    Args:
        history: Dictionary with 'states' and 'times' keys
        figsize: Figure size
        save_path: Optional path to save figure
    """
    states = history["states"]
    pos_data = np.array([s.pos_I for s in states])
    
    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection="3d")
    
    ax.plot(pos_data[:, 0], pos_data[:, 1], pos_data[:, 2])
    ax.scatter(pos_data[0, 0], pos_data[0, 1], pos_data[0, 2], 
               color="green", s=100, label="Start")
    ax.scatter(pos_data[-1, 0], pos_data[-1, 1], pos_data[-1, 2], 
               color="red", s=100, label="End")
    
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("3D Trajectory")
    ax.legend()
    
    if save_path:
        plt.savefig(save_path)
    else:
        plt.show()
