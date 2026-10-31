'''
This script is to make an earth orbit to be visualized in Unity.
Prior to this script, the Unity visualizer is basic with showing trajectory and frame data with time controls
Through this script, the goal is to implement and create clean UI visuals in Unity for viewing different aspects and values of an orbit.
Importantly, this script will produce no direct visuals, instead send_orbit will be untilized to send all relavent and necessary information to Unity.

Example things to view for an orbit over time:
position away from central body (r_mag)
velocity magnitude
true anomaly

Potential addition for ground tracks and lat/lon calculations can be done, though is much lower priority
Future scripts will use this base code and visual implementation to begin visualization of hohmann transfers
'''

# Imports
import numpy as np
from orbitCalculations import Orbit, EARTH
from frames import Quaternion, FrameTree
from scipy.integrate import solve_ivp

# Minimal orbit
earth_orbit = Orbit(
    mu = EARTH.mu,
    semi_major_axis = EARTH.radius + 800,
    eccentricity = 0.0,
    inclination = 50*np.pi/180,
    raan = 75*np.pi/180,
    arg_of_periapsis = 140*np.pi/180,
    body = EARTH
)

def orbit_propagate_cart(time, state, mu):
    # Split state
    r_vec_cart = state[0:3]
    v_vec_cart = state[3:6]

    r_mag_km = np.linalg.norm(r_vec_cart)

    f_0 = np.concatenate((v_vec_cart, -mu / r_mag_km**3 * r_vec_cart))

    state_dot = f_0
    return state_dot

# Define starting true anomaly
start_nu = 90 *np.pi/180

initial_state_rth = earth_orbit.get_state_rth(start_nu)
# Break into r and v
r_0_rth = initial_state_rth[:3]
v_0_rth = initial_state_rth[3:]

# Set up frames
ft = FrameTree(root_frame='eci')

# pqw / eci relation
q_c1_eci = Quaternion.from_axis_angle([0,0,1], earth_orbit.raan) # raan
q_c2_c1 = Quaternion.from_axis_angle([1,0,0], earth_orbit.inclination) # inclination
q_pqw_c2 = Quaternion.from_axis_angle([0,0,1], earth_orbit.arg_of_periapsis) # aop
q_pqw_eci = q_pqw_c2 * q_c2_c1 * q_c1_eci
ft.add_frame('pqw', 'eci', q_from_parent=q_pqw_eci)

# rth / pqw relation
q_rth_pqw_0 = Quaternion.from_axis_angle([0,0,1], start_nu)
ft.add_frame('rth_0', 'pqw', q_from_parent=q_rth_pqw_0)

# Convert initial state into eci
r_0_eci = ft.transform(r_0_rth, 'rth_0', 'eci')
v_0_eci = ft.transform(v_0_rth, 'rth_0', 'eci')
state_0_eci = np.concatenate((r_0_eci, v_0_eci))

# Get orbit period for time to propagate
period = earth_orbit.period
time = np.linspace(0, period*10, 3000)

sol = solve_ivp(orbit_propagate_cart, [time[0], time[-1]], state_0_eci, method='RK45', t_eval=time, atol=1e-12, rtol=1e-12, args=(EARTH.mu,))

# Extract solution
r_eci = sol.y[:3]
v_eci = sol.y[3:6]

r_ecf = np.zeros(np.shape(r_eci))
v_ecf = np.zeros(np.shape(v_eci))

# Calculate Basis Vectors
basis_eci_eci = ft.get_basis('eci', 'eci') # Doesn't change with time
basis_pqw_eci = ft.get_basis('pqw', 'eci') # Doesn't change with time
basis_rth_eci = np.zeros((len(time), 3, 3))
for k in range(len(time)):
    # Calculate r_hat
    r_hat = r_eci[:,k] / np.linalg.norm(r_eci[:,k])
    h_vec = np.cross(r_eci[:,k], v_eci[:,k])
    h_hat = h_vec / np.linalg.norm(h_vec)
    e_hat = np.cross(h_hat, r_hat)
    basis_rth_eci[k, :, :] = [r_hat.T, e_hat.T, h_hat.T]

# ECF transformation
basis_ecf_eci = np.zeros((len(time), 3, 3))
ft.add_frame('ecf', 'eci', q_from_parent=Quaternion(0,0,0,1))
for k in range(len(time)):
    q_ecf_eci = Quaternion.from_axis_angle([0,0,1], EARTH.rotation_rate*time[k])
    ft.update_frame('ecf', q_from_parent=q_ecf_eci)
    r_ecf[:, k] = ft.transform(r_eci[:,k], 'eci', 'ecf')
    v_ecf[:, k] = ft.transform(v_eci[:,k], 'eci', 'ecf')
    basis_ecf_eci[k, :, :] = ft.get_basis('ecf', 'eci')

# LLA transformation (Spherical Earth)
r_lla = np.zeros(np.shape(r_eci))
for k in range(len(time)):
    r_mag = np.linalg.norm(r_ecf[:,k])
    lat = np.asin(r_ecf[2,k] / r_mag)
    lon = np.atan2(r_ecf[1,k], r_ecf[0,k])
    alt = r_mag - EARTH.radius
    r_lla[:,k] = [lat, lon, alt]


# Compile basis vectors into a dictionary for Unity
all_basis = {
    'ECI': basis_eci_eci,
    'PQW': basis_pqw_eci,
    'RTH': basis_rth_eci,
    'ECF': basis_ecf_eci,
}

# Send trajectory to Unity 3D visualizer (if active)
from unity_bridge import send_orbit
send_orbit(
    name="Earth Simple Orbit",
    time=time,
    r_eci=r_eci,
    v_eci=v_eci,
    r_lla=r_lla,
    central_body=EARTH,
    all_basis=all_basis,
    scale_km_per_unit=1000.0,
)

