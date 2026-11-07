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
target['aol'] = np.asin(np.sin(target['lat']) / np.sin(target['i']))

# Now calculate RAAN
total_raan = np.atan2(np.cos(target['i']) * np.sin(target['aol']), np.cos(target['aol']))
target['raan'] = total_raan - target['lon'] + target['time']*EARTH.rotation_rate

# Set propagation time
tspan = [0, 10000]

target_orbit = Orbit(
    mu=EARTH.mu,
    semi_major_axis=target['a'],
    eccentricity=target['e'],
    inclination=target['i'],
    raan=target['raan'],
    arg_of_periapsis=target['aol'],
    body=EARTH
)

temp_1 = Spacecraft('temp1', initial_orbit=initial_orbit, initial_nu=0)
temp_1.register_frames(ft)
r_1_eci = temp_1.get_initial_state_eci(ft)

temp_2 = Spacecraft('temp2', initial_orbit=target_orbit, initial_nu=0)
temp_2.register_frames(ft)
r_2_eci = temp_2.get_initial_state_eci(ft)

# Lambert solution


# # Satelite 1
# start_nu = 0
# sat_1 = Spacecraft('Satelite', initial_orbit=initial_orbit, initial_nu=start_nu, useSTM=True)
# sat_1.register_frames(ft) # TODO make this part of init
# sat_1.propagate(tspan, ft)

# basis_eci_eci = ft.get_basis('eci', 'eci')

# all_basis = {
#     'ECI': basis_eci_eci,
#     'Orbit Plane': basis_pqw_eci,
# }

# # Send trajectory to Unity 3D visualizer (if active)
# from unity_bridge import send_orbit
# send_orbit(
#     name='Target Point',
#     time=anim_time,
#     r_eci=r_eci,
#     r_lla=r_lla,
#     central_body=EARTH,
#     all_basis=all_basis,
#     scale_km_per_unit=1000.0,
# )