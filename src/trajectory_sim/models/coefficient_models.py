"""Aerodynamic coefficient models."""

from abc import ABC, abstractmethod
from typing import Dict


class AeroCoefficientModel(ABC):
    """Abstract base class for aerodynamic coefficient models."""
    
    @abstractmethod
    def evaluate(
        self,
        alpha: float,
        beta: float,
        mach: float,
        control_p: float,
        control_q: float,
        control_r: float,
        control_i: float,
    ) -> Dict[str, float]:
        """Evaluate aerodynamic coefficients.
        
        Args:
            alpha: Angle of attack (rad)
            beta: Sideslip angle (rad)
            mach: Mach number
            control_p: Roll control input
            control_q: Pitch control input
            control_r: Yaw control input
            control_i: Brake/drag control input
        
        Returns:
            Dictionary with keys: Cx, Cy, Cz, Cmx, Cmy, Cmz
        """
        pass


class SimpleAeroModel(AeroCoefficientModel):
    """Simple linear aerodynamic model."""
    
    def __init__(
        self,
        Cx0: float = 0.5,
        Cz_alpha: float = 5.0,
        Cm_alpha: float = -2.0,
    ):
        """Initialize simple aerodynamic model.
        
        Args:
            Cx0: Base drag coefficient
            Cz_alpha: Lift curve slope
            Cm_alpha: Pitch moment slope
        """
        self.Cx0 = Cx0
        self.Cz_alpha = Cz_alpha
        self.Cm_alpha = Cm_alpha
    
    def evaluate(
        self,
        alpha: float,
        beta: float,
        mach: float,
        control_p: float,
        control_q: float,
        control_r: float,
        control_i: float,
    ) -> Dict[str, float]:
        """Evaluate aerodynamic coefficients."""
        return {
            "Cx": self.Cx0 + control_i * 0.5,
            "Cy": 0.0,
            "Cz": self.Cz_alpha * alpha,
            "Cmx": 0.0,
            "Cmy": self.Cm_alpha * alpha,
            "Cmz": 0.0,
        }
