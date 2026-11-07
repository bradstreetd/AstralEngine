# Imports

import numpy as np
from orbitCalculations import Orbit, EARTH
from frames import Quaternion, FrameTree
from spacecraft import Spacecraft
from eventHelper import *

# Set up initial frames
ft = FrameTree(root_frame='eci')
ft.add_frame('ecf', 'eci', q_from_parent=Quaternion(0, 0, 0, 1))

# Starting orbit
initial_orbit = Orbit(
    mu=EARTH.mu,
    semi_major_axis=EARTH.radius + 800,
    eccentricity=0.0,
    inclination=50*np.pi/180,
    raan=75*np.pi/180,
    arg_of_periapsis=140*np.pi/180,
    body=EARTH
)

# Target orbit
target = {
    'a': initial_orbit.semi_major_axis,
    'e': 0,
    'i': initial_orbit.inclination,
    'lat': 10*np.pi/180,
    'lon': 10*np.pi/180,
    'time': 8000
}

# Target LLA
target_lla = [target['lat'], target['lon'], target['a'] - EARTH.radius]

# Convert LLA to ECI
target_ecf = np.array([target['a']*np.cos(target['lat']) * np.cos(target['lon']), target['a']*np.cos(target['lat']) * np.sin(target['lon']), target['a']*np.sin(target['lat'])])
q_ecf_eci = Quaternion.from_axis_angle([0, 0, 1], target['time']*EARTH.rotation_rate)
ft.update_frame('ecf', q_from_parent=q_ecf_eci)
target_eci = ft.transform(target_ecf, 'ecf', 'eci')

# With knowing a, e, i, we can get the argument of latitude
# Knowing these allow us to construct a spherical triangle
aol = np.asin(np.sin(target['lat']) / np.sin(target['i']))

# When making page on this, make visual of this math
# length of animation
anim_length = 10
num_points = 200

anim_time = np.linspace(0, anim_length, num_points)

# Set up target point array
r_eci = np.zeros((num_points, 3))
r_lla = np.zeros((num_points, 3))
for k in range(num_points):
    r_eci[k, :] = target_eci
    r_lla[k, :] = target_lla

# Have orbit plane rotate to its specific inclination
basis_pqw_eci = np.zeros((num_points, 3, 3))
num_points_inc = int(num_points/2)
ft.add_frame('pqw', 'eci', Quaternion(0,0,0,1))
for k in range(num_points_inc):
    inc = 0 + target['i']/num_points_inc * k
    q_pqw_eci = Quaternion.from_axis_angle([1,0,0], inc)
    ft.update_frame('pqw', q_from_parent=q_pqw_eci)
    basis_pqw_eci[k, :, :] = ft.get_basis('pqw', 'eci')

# Have orbit plane rotate with raan
end_raan = 20*np.pi/180
num_points_raan = int(num_points/2)
ft.add_frame('pqw', 'eci', Quaternion(0,0,0,1))
for k in range(num_points_raan):
    idx = k + num_points_inc
    raan = 0 + end_raan/num_points_raan * k
    q_raan_eci = Quaternion.from_axis_angle([0,0,1], raan)
    q_pqw_raan = Quaternion.from_axis_angle([1,0,0], target['i'])
    q_pqw_eci = q_pqw_raan * q_raan_eci
    ft.update_frame('pqw', q_from_parent=q_pqw_eci)
    basis_pqw_eci[idx, :, :] = ft.get_basis('pqw', 'eci')

# Raan


# Argument of latitude


# Set propagation time
tspan = [0, 10000]

# Satelite 1
start_nu = 0
sat_1 = Spacecraft('Satelite', initial_orbit=initial_orbit, initial_nu=start_nu, useSTM=True)
sat_1.register_frames(ft) # TODO make this part of init
sat_1.propagate(tspan, ft)

basis_eci_eci = ft.get_basis('eci', 'eci')

all_basis = {
    'ECI': basis_eci_eci,
    'Orbit Plane': basis_pqw_eci,
}

# Send trajectory to Unity 3D visualizer (if active)
from unity_bridge import send_orbit
send_orbit(
    name='Target Point',
    time=anim_time,
    r_eci=r_eci,
    r_lla=r_lla,
    central_body=EARTH,
    all_basis=all_basis,
    scale_km_per_unit=1000.0,
)