# Script for testing orbitCalculations
# Not meant for much besides initial code setup and testing, along with some starting visual Unity workflow
import numpy as np
from orbitCalculations import Orbit, EARTH, SUN, MARS, TITAN
from frames import Quaternion, FrameTree
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# I want to first define a regular orbit minimally
semi_major_axis = 10000
eccentricity = 0.3

# Now I want to calculate all relevant parameters that describe this orbit in the orbital plane
earth_orbit = Orbit(
    mu = EARTH.mu,
    semi_major_axis = 10000.0,
    eccentricity = 0.3
)

# Given altitude and velocity
altitude = 979
velocity = 5.8

# Periapsis radius
radius_periapsis = altitude + TITAN.radius
theta_dot = velocity / radius_periapsis # v / r
specific_angular_momentum = radius_periapsis**2 * theta_dot
semi_latus_rectum = specific_angular_momentum**2 / TITAN.mu
specific_energy = velocity**2 / 2 - TITAN.mu / radius_periapsis
eccentricity = np.sqrt(1 + (2 * specific_energy * specific_angular_momentum**2) / TITAN.mu**2)
semi_major_axis = semi_latus_rectum / (1 - eccentricity**2)

hyperbolic_orbit = Orbit(
    mu = TITAN.mu,
    semi_major_axis = semi_major_axis,
    eccentricity = eccentricity
)

raan = 0 * np.pi/180
inclination = 0 * np.pi/180
arg_peri = 0 * np.pi/180
nu_0 = -np.pi/2

# Get the start at 90 degrees true anomaly (or negative for the direction its going)
start_radius = hyperbolic_orbit.calc_radius_mag(nu_0) # Should be the same as semi latus rectum
start_velocity = hyperbolic_orbit.calc_velocity_mag(start_radius)

initial_state_rth = hyperbolic_orbit.get_state_rth(nu_0)

# Define frame tree to translate this into titan frame (with 0 raan, inc, aop)
ft = FrameTree(root_frame='tci')

q_t1_tci = Quaternion.from_axis_angle([0,0,1], raan)
q_t2_t1 = Quaternion.from_axis_angle([1,0,0], inclination)
q_pqw_t2 = Quaternion.from_axis_angle([0,0,1], arg_peri)
q_pqw_tci = q_pqw_t2 * q_t2_t1 * q_t1_tci

ft.add_frame('pqw', parent='tci', q_from_parent=q_pqw_tci)

# Set up propagator to propagate it for some amount of time
def orbit_propagate_cart(time, state, mu):
    # Split state
    r_vec_cart = state[0:3]
    v_vec_cart = state[3:6]

    r_mag_km = np.linalg.norm(r_vec_cart)

    f_0 = np.concatenate((v_vec_cart, -mu / r_mag_km**3 * r_vec_cart))

    state_dot = f_0
    return state_dot

time = np.linspace(0, 20e3, 1000)

# Initial state in tci frame
q_rth_pqw_0 = Quaternion.from_axis_angle([0,0,1], nu_0) # Initial relationship between rth and pqw
ft.add_frame('rth', parent='pqw', q_from_parent=q_rth_pqw_0)

r_0_tci = ft.transform(initial_state_rth[:3], from_frame='rth', to_frame='tci')
v_0_tci = ft.transform(initial_state_rth[3:], from_frame='rth', to_frame='tci')
state_0_tci = np.concatenate((r_0_tci, v_0_tci))

# Propagate!
sol = solve_ivp(orbit_propagate_cart, [time[0], time[-1]], state_0_tci, method='RK45', t_eval=time, atol=1e-12, rtol=1e-12, args=(TITAN.mu,))

# Extract solution
r_eci = sol.y[:3]
v_eci = sol.y[3:6]

# Calculate Basis Vectors
basis_tci_tci = ft.get_basis('tci', 'tci') # Doesn't change with time
basis_pqw_tci = ft.get_basis('pqw', 'tci') # Doesn't change with time
basis_rth_tci = np.zeros((len(time), 3, 3))
for k in range(len(time)):
    # Calculate r_hat
    r_hat = r_eci[:,k] / np.linalg.norm(r_eci[:,k])
    h_vec = np.cross(r_eci[:,k], v_eci[:,k])
    h_hat = h_vec / np.linalg.norm(h_vec)
    e_hat = np.cross(h_hat, r_hat)
    basis_rth_tci[k, :, :] = [r_hat.T, e_hat.T, h_hat.T]

# Send trajectory to Unity 3D visualizer (if active)
from unity_bridge import send_orbit
send_orbit(
    name="Titan Hyperbolic Flyby",
    time=time,
    r_eci=r_eci,
    v_eci=v_eci,
    central_body=TITAN,
    basis_tci=basis_tci_tci,
    basis_pqw=basis_pqw_tci,
    basis_rth=basis_rth_tci,
    scale_km_per_unit=1000.0,
)

# ==============================================================================
# Plot Trajectory (r_eci) and Orbit Evolution
# ==============================================================================
plt.style.use('dark_background')
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6.5))

# ------------------------------------------------------------------------------
# 1. 2D Trajectory in Central Body Frame (TCI / ECI)
# ------------------------------------------------------------------------------
# Plot trajectory path
ax1.plot(r_eci[0], r_eci[1], color='#38bdf8', lw=2.2, label='Hyperbolic Trajectory')

# Draw central body (Titan) to scale
titan_body = plt.Circle((0, 0), TITAN.radius, color='#f59e0b', alpha=0.45, label=f'Titan (R = {TITAN.radius:.0f} km)')
ax1.add_patch(titan_body)
ax1.plot(0, 0, 'o', color='#fbbf24', markersize=6, label='Titan Center')

# Start and end position markers
ax1.plot(r_eci[0, 0], r_eci[1, 0], 'o', color='#34d399', markersize=8, label=f'Start (t=0s, $\\nu={np.rad2deg(nu_0):.0f}^\\circ$)')
ax1.plot(r_eci[0, -1], r_eci[1, -1], 'o', color='#f43f5e', markersize=8, label=f'End (t={time[-1]:.0f}s)')

# Velocity vector arrow at start
v_scale = 1500  # Scale arrow length for visibility
ax1.arrow(
    r_eci[0, 0], r_eci[1, 0],
    v_eci[0, 0] * v_scale, v_eci[1, 0] * v_scale,
    color='#34d399', width=200, head_width=800, head_length=1200,
    length_includes_head=True, alpha=0.8, label=r'Initial Velocity $\vec{v}_0$'
)

# Reference axes
ax1.axhline(0, color='#64748b', linestyle=':', alpha=0.4)
ax1.axvline(0, color='#64748b', linestyle=':', alpha=0.4)

ax1.set_title('Trajectory in Central Body Frame ($r_{eci}$)', fontsize=12, fontweight='bold', pad=12)
ax1.set_xlabel('X [km]', fontsize=11)
ax1.set_ylabel('Y [km]', fontsize=11)
ax1.set_aspect('equal', 'datalim')
ax1.grid(True, linestyle='--', alpha=0.25)
ax1.legend(loc='best', fontsize=9, framealpha=0.8)

# ------------------------------------------------------------------------------
# 2. Altitude and Velocity Magnitude vs Time
# ------------------------------------------------------------------------------
r_mag = np.linalg.norm(r_eci, axis=0)
altitude_km = r_mag - TITAN.radius
speed_kms = np.linalg.norm(v_eci, axis=0)

color_alt = '#38bdf8'
ax2.plot(time, altitude_km, color=color_alt, lw=2.2, label='Altitude')
ax2.set_xlabel('Time [s]', fontsize=11)
ax2.set_ylabel('Altitude above Titan [km]', color=color_alt, fontsize=11)
ax2.tick_params(axis='y', labelcolor=color_alt)
ax2.grid(True, linestyle='--', alpha=0.25)

# Secondary axis for speed
ax2_twin = ax2.twinx()
color_spd = '#fbbf24'
ax2_twin.plot(time, speed_kms, color=color_spd, lw=2.0, linestyle='--', label='Speed')
ax2_twin.set_ylabel('Speed [km/s]', color=color_spd, fontsize=11)
ax2_twin.tick_params(axis='y', labelcolor=color_spd)

ax2.set_title('Altitude & Speed vs Time', fontsize=12, fontweight='bold', pad=12)

# Combined legend for ax2
lines_1, labels_1 = ax2.get_legend_handles_labels()
lines_2, labels_2 = ax2_twin.get_legend_handles_labels()
ax2.legend(lines_1 + lines_2, labels_1 + labels_2, loc='center right', fontsize=9, framealpha=0.8)

plt.tight_layout()
plt.show()