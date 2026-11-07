import numpy as np
from orbitCalculations import Orbit, EARTH

a = 6700
e = 0
inc = 50*np.pi/180

# Target
target_lla = np.array([10*np.pi/180, 10*np.pi/180, a-EARTH.radius])
target_time = 1000

# Right ascension of the target point
target_ra = target_lla[1] + EARTH.rotation_rate * target_time

# Argument of latitude
aol = np.asin(np.sin(target_lla[0]) / np.sin(inc))

raan = target_ra - np.atan2(np.cos(inc) * np.sin(aol), np.cos(aol))

