"""
Orbit mechanics calculations and data structures for Keplerian orbits,
conic sections, anomaly conversions, and patched-conics mission design.
"""

from enum import Enum, auto
import numpy as np


class OrbitType(Enum):
    CIRCULAR = auto()
    ELLIPTICAL = auto()
    PARABOLIC = auto()
    HYPERBOLIC = auto()


class Body:
    """
    Represents a central gravitating body (e.g., Sun, Earth, Moon, Mars).
    Useful for patched conics when switching spheres of influence.
    """
    def __init__(self, name: str, mu: float, radius: float = 0.0, soi_radius: float = 0.0):
        self.name = name
        self.mu = mu                  # Standard gravitational parameter (km^3 / s^2)
        self.radius = radius          # Mean equatorial radius (km)
        self.soi_radius = soi_radius  # Sphere of influence radius (km)
        self.rotation_rate = 7.292115090e-5

    def __repr__(self):
        return f"Body({self.name}, mu={self.mu:.4e} km^3/s^2)"


# Common central bodies (km^3 / s^2 and km)
SUN = Body("Sun", mu=1.32712440018e11, radius=696340.0)
EARTH = Body("Earth", mu=398600.4418, radius=6378.137, soi_radius=925000.0)
MOON = Body("Moon", mu=4902.8000, radius=1737.4, soi_radius=66100.0)
MARS = Body("Mars", mu=42828.3752, radius=3389.5, soi_radius=577000.0)
TITAN = Body("Titan", mu=8978.5, radius=2574.7)

class Orbit:
    def __init__(
        self,
        mu: float,
        semi_major_axis: float = None,
        eccentricity: float = 0.0,
        inclination: float = 0.0,
        raan: float = 0.0,
        arg_of_periapsis: float = 0.0,
        body: Body = None,
    ):
        self.mu = mu
        self.semi_major_axis = semi_major_axis
        self.eccentricity = eccentricity
        self.inclination = inclination
        self.raan = raan
        self.arg_of_periapsis = arg_of_periapsis
        self.body = body

    @property
    def orbit_type(self) -> OrbitType:
        e = self.eccentricity
        if np.isclose(e, 0.0, atol=1e-8):
            return OrbitType.CIRCULAR
        elif np.isclose(e, 1.0, atol=1e-8):
            return OrbitType.PARABOLIC
        elif e < 1.0:
            return OrbitType.ELLIPTICAL
        else:
            return OrbitType.HYPERBOLIC

    @property
    def is_closed(self) -> bool:
        return (self.orbit_type == OrbitType.CIRCULAR or self.orbit_type == OrbitType.ELLIPTICAL)

    
    @property
    def semi_latus_rectum(self) -> float:
        p = self.semi_major_axis * (1 - self.eccentricity**2)
        return p

    @property
    def semi_minor_axis(self) -> float:
        if self.orbit_type == OrbitType.HYPERBOLIC:
            b = abs(self.semi_major_axis) * np.sqrt(self.eccentricity**2 - 1)
        else:
            b = abs(self.semi_major_axis) * np.sqrt(1 - self.eccentricity**2)
        return b

    @property
    def periapsis_radius(self) -> float:
        r_p = self.semi_major_axis * (1 - self.eccentricity)
        return r_p

    @property
    def apoapsis_radius(self) -> float:
        if not self.is_closed:
            return np.inf
        r_a = self.semi_major_axis * (1 + self.eccentricity)
        return r_a

    @property
    def specific_angular_momentum(self) -> float:
        h = np.sqrt(self.mu * self.semi_latus_rectum)
        return h

    @property
    def specific_energy(self) -> float:
        epsilon = (self.eccentricity**2 - 1) * (self.mu**2) / (2 * self.specific_angular_momentum**2)
        return epsilon
    
    @property
    def mean_motion(self) -> float:
        n = np.sqrt(self.mu / self.semi_major_axis**3)
        return n

    @property
    def period(self) -> float:
        if not self.is_closed:
            return np.inf
        period = 2 * np.pi / self.mean_motion
        return period

    @property
    def v_infinity(self) -> float:
        if self.orbit_type != OrbitType.HYPERBOLIC:
            return 0.0
        v_infinity = np.sqrt(2 * self.specific_energy)
        return v_infinity

    @property
    def nu_infinity(self) -> float:
        if self.orbit_type != OrbitType.HYPERBOLIC:
            return 0.0
        nu_infinity = np.acos(- 1 / self.eccentricity)
        return nu_infinity

    @property
    def flyby_angle(self) -> float:
        if self.orbit_type != OrbitType.HYPERBOLIC:
            return 0.0
        delta = 2 * np.asin(1/self.eccentricity)
        return delta
    
    def calc_radius_mag(self, nu) -> float:
        r = self.semi_latus_rectum / (1 + self.eccentricity * np.cos(nu))
        return r
    
    def calc_velocity_mag(self, radius) -> float:
        v = np.sqrt(2 * (self.specific_energy + self.mu / radius))
        return v
    
    def get_state_rth(self, nu):
        r_mag = self.calc_radius_mag(nu)
        r_rth = np.array([r_mag, 0, 0])

        v_mag = self.calc_velocity_mag(r_mag)
        fpa = np.atan2(self.eccentricity * np.sin(nu), (1 + self.eccentricity*np.cos(nu)))
        v_rth = v_mag * np.array([np.sin(fpa), np.cos(fpa), 0])
        state_rth = np.concatenate((r_rth, v_rth))
        return state_rth

    def __repr__(self):
        type_str = self.orbit_type.name.capitalize()
        if self.semi_major_axis is not None:
            return f"Orbit({type_str}, a={self.semi_major_axis:.1f} km, e={self.eccentricity:.4f})"
        else:
            return f"Orbit({type_str}, p={self._p:.1f} km, e={self.eccentricity:.4f})"
