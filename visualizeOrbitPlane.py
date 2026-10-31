# Visualizing orbit plane around Earth
# Goal: Animate a satellite in orbit around Earth in 3D ECI space with unit vectors for ECI, PQW, and RTH frames.

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from frames import Quaternion, FrameTree

num_points = 100

# First, define the orbit in the rth frame
semi_major_axis = 10000
eccentricity = 0.3

# Additional info for ECI relative framing
RAAN = 45 * np.pi/180
inclination = 30 * np.pi/180
arg_of_periapsis = 20 * np.pi/180

true_anomaly_sweep = np.linspace(0, 2*np.pi, num_points)

# Initialize arrays
r_rth = np.zeros((3, num_points))
r_pqw = np.zeros((3, num_points))
r_eci = np.zeros((3, num_points))

# Initialize Frame Tree
ft = FrameTree(root_frame='eci')

# Unit vectors in ECI coordinates:
# Base identity triad
unit_vec_base = np.eye(3)

# 1. ECI unit vectors in ECI frame (stationary at Earth center)
unit_vec_eci_in_eci = np.eye(3)

# 2. Perifocal (PQW) unit vectors in ECI frame (stationary at Earth center)
unit_vec_pqw_in_eci = np.zeros((3, 3))

# 3. Rotating (RTH) unit vectors in ECI frame (moving with satellite)
unit_vec_rth_in_eci = np.zeros((3, 3, num_points))

# Rotation of ECI -> pqw (3-1-3 Euler sequence: RAAN -> inc -> aop)
A_q_eci = Quaternion.from_axis_angle(np.array([0, 0, 1]), RAAN)
B_q_A = Quaternion.from_axis_angle(np.array([1, 0, 0]), inclination)
pqw_q_B = Quaternion.from_axis_angle(np.array([0, 0, 1]), arg_of_periapsis)
pqw_Q_eci = pqw_q_B * B_q_A * A_q_eci

ft.add_frame('pqw', parent='eci', q_from_parent=pqw_Q_eci)
ft.add_frame('rth', parent='pqw', q_from_parent=Quaternion.identity())

unit_vec_pqw_in_eci[:, :] = ft.get_basis('pqw', in_frame='eci')

for k, true_anomaly in enumerate(true_anomaly_sweep):
    # R in RTH frame
    p = semi_major_axis * (1 - eccentricity**2)
    r_mag = p / (1 + eccentricity*np.cos(true_anomaly))
    r_rth[:, k] = np.array([r_mag, 0, 0])

    # Rotation of pqw -> rth (Would function as r_rth = rth_C_pqw * r_pqw)
    rth_Q_pqw = Quaternion.from_axis_angle(np.array([0, 0, 1]), true_anomaly)

    ft.update_frame('rth', q_from_parent=rth_Q_pqw)

    r_eci[:, k] = ft.transform(r_rth[:, k], from_frame='rth', to_frame='eci')

    unit_vec_rth_in_eci[:,:,k] = ft.get_basis('rth', in_frame='eci')

# ==============================================================================
# 3D Dark-Mode Plotting & Animation in ECI Frame
# ==============================================================================
plt.style.use('dark_background')
fig = plt.figure(figsize=(9, 9))
ax = fig.add_subplot(111, projection='3d')

# Scale lengths for unit vector visualization [km]
L_eci = 6000
L_pqw = 6500
L_rth = 2200
r_earth = 6378.137  # Earth equatorial radius [km]

# -------------------------------------------------------------
# 1. Earth 3D Sphere & Equatorial Reference
# -------------------------------------------------------------
u_e = np.linspace(0, 2 * np.pi, 30)
v_e = np.linspace(0, np.pi, 20)
x_earth = r_earth * np.outer(np.cos(u_e), np.sin(v_e))
y_earth = r_earth * np.outer(np.sin(u_e), np.sin(v_e))
z_earth = r_earth * np.outer(np.ones_like(u_e), np.cos(v_e))
ax.plot_surface(x_earth, y_earth, z_earth, color='#1e3a8a', alpha=0.25, edgecolor='#3b82f6', linewidth=0.2)

# Equatorial plane ring (Z=0)
theta_eq = np.linspace(0, 2*np.pi, 100)
ax.plot(r_earth * np.cos(theta_eq), r_earth * np.sin(theta_eq), np.zeros_like(theta_eq),
        color='#330293', linestyle=':', lw=1.0, alpha=0.5, label='Equatorial Plane')

# -------------------------------------------------------------
# 2. Static Orbit Trajectory & Critical Points
# -------------------------------------------------------------
ax.plot(r_eci[0, :], r_eci[1, :], r_eci[2, :], color='#38bdf8', lw=2.0, alpha=0.85, label='Orbit Trajectory ($r_{eci}$)')

# Periapsis and Apoapsis points
ax.plot([r_eci[0, 0]], [r_eci[1, 0]], [r_eci[2, 0]], 'o', color='#f43f5e', markersize=6, label=f'Periapsis ({r_mag:.0f} km)')
apo_idx = num_points // 2
ax.plot([r_eci[0, apo_idx]], [r_eci[1, apo_idx]], [r_eci[2, apo_idx]], 'o', color='#c084fc', markersize=6, label='Apoapsis')

# Line of Nodes (Vector to Ascending Node in equatorial plane)
node_vec = np.array([np.cos(RAAN), np.sin(RAAN), 0.0]) * (r_earth + 2500)
ax.plot([0, node_vec[0]], [0, node_vec[1]], [0, node_vec[2]], color='#eab308', linestyle='--', lw=1.5, label=r'Line of Nodes ($\Omega$)')

# -------------------------------------------------------------
# 3. ECI Coordinate Frame Unit Vectors (Stationary at Origin)
# -------------------------------------------------------------
eci_colors = ['#94a3b8', '#94a3b8', '#cbd5e1']
eci_labels = [r'$\hat{i}_{ECI}$', r'$\hat{j}_{ECI}$', r'$\hat{k}_{ECI}$ (North)']
for j in range(3):
    vec = unit_vec_eci_in_eci[j, :] * L_eci
    ax.plot([0, vec[0]], [0, vec[1]], [0, vec[2]], color=eci_colors[j], linestyle=':', lw=1.4)
    ax.text(vec[0]*1.08, vec[1]*1.08, vec[2]*1.08, eci_labels[j], color=eci_colors[j], fontsize=9)

# -------------------------------------------------------------
# 4. Perifocal (PQW) Frame Unit Vectors (Stationary at Origin)
# -------------------------------------------------------------
pqw_colors = ['#f87171', '#34d399', '#a78bfa']
pqw_labels = [r'$\hat{p}$ (Periapsis)', r'$\hat{q}$ ($+90^\circ$)', r'$\hat{h}$ ($\vec{h}$ Normal)']
for j in range(3):
    vec = unit_vec_pqw_in_eci[j, :] * L_pqw
    ax.plot([0, vec[0]], [0, vec[1]], [0, vec[2]], color=pqw_colors[j], linestyle='--', lw=1.6)
    ax.text(vec[0]*1.08, vec[1]*1.08, vec[2]*1.08, pqw_labels[j], color=pqw_colors[j], fontsize=9, fontweight='bold')

# -------------------------------------------------------------
# 5. Dynamic / Animated Elements (Satellite & Rotating RTH Frame)
# -------------------------------------------------------------
sat_dot, = ax.plot([], [], [], 'o', color='#fbbf24', markersize=9, markeredgecolor='white', markeredgewidth=1.2, label='Satellite')
pos_line, = ax.plot([], [], [], color='#fbbf24', linestyle=':', lw=1.2, alpha=0.6)

# Moving RTH triad attached to satellite
r_vec_line, = ax.plot([], [], [], color='#f43f5e', lw=2.2, label=r'$\hat{R}$ (Radial)')
t_vec_line, = ax.plot([], [], [], color='#2dd4bf', lw=2.2, label=r'$\hat{T}$ (Transverse)')
h_vec_line, = ax.plot([], [], [], color='#c084fc', lw=2.2, label=r'$\hat{H}$ (Normal)')

# Real-time HUD info text overlay
hud_text = ax.text2D(0.03, 0.94, '', transform=ax.transAxes, fontsize=10,
                     bbox=dict(boxstyle='round,pad=0.5', facecolor='#0f172a', edgecolor='#334155', alpha=0.85))

# -------------------------------------------------------------
# 6. Plot Configuration & Limits
# -------------------------------------------------------------
ax.set_title('3D Orbit Plane & Multi-Frame Orientation in ECI', fontsize=13, fontweight='bold', pad=15)
ax.set_xlabel('$X_{ECI}$ [km]', fontsize=10, labelpad=8)
ax.set_ylabel('$Y_{ECI}$ [km]', fontsize=10, labelpad=8)
ax.set_zlabel('$Z_{ECI}$ [km]', fontsize=10, labelpad=8)

max_range = 10500
ax.set_xlim([-max_range, max_range])
ax.set_ylim([-max_range, max_range])
ax.set_zlim([-max_range, max_range])
ax.set_box_aspect([1, 1, 1])
ax.grid(True, linestyle='--', alpha=0.2)
ax.view_init(elev=26, azim=48)
ax.legend(loc='lower left', bbox_to_anchor=(0.01, 0.01), fontsize=8, framealpha=0.75)

# -------------------------------------------------------------
# 7. Animation Update Function
# -------------------------------------------------------------
def update(frame):
    # Current satellite position in ECI
    pos = r_eci[:, frame]
    nu_deg = np.rad2deg(true_anomaly_sweep[frame])
    r_current = np.linalg.norm(pos)
    alt_current = r_current - r_earth

    # Update satellite dot and radius line
    sat_dot.set_data([pos[0]], [pos[1]])
    sat_dot.set_3d_properties([pos[2]])

    pos_line.set_data([0, pos[0]], [0, pos[1]])
    pos_line.set_3d_properties([0, pos[2]])

    # Update RTH unit vectors originating from current satellite position
    r_dir = unit_vec_rth_in_eci[0, :, frame] * L_rth
    t_dir = unit_vec_rth_in_eci[1, :, frame] * L_rth
    h_dir = unit_vec_rth_in_eci[2, :, frame] * L_rth

    r_vec_line.set_data([pos[0], pos[0] + r_dir[0]], [pos[1], pos[1] + r_dir[1]])
    r_vec_line.set_3d_properties([pos[2], pos[2] + r_dir[2]])

    t_vec_line.set_data([pos[0], pos[0] + t_dir[0]], [pos[1], pos[1] + t_dir[1]])
    t_vec_line.set_3d_properties([pos[2], pos[2] + t_dir[2]])

    h_vec_line.set_data([pos[0], pos[0] + h_dir[0]], [pos[1], pos[1] + h_dir[1]])
    h_vec_line.set_3d_properties([pos[2], pos[2] + h_dir[2]])

    # Update HUD text
    hud_text.set_text(
        f"Orbit State:\n"
        f"  True Anomaly ($\\nu$): {nu_deg:5.1f}°\n"
        f"  Radius ($r$):          {r_current:5.0f} km\n"
        f"  Altitude ($h$):        {alt_current:5.0f} km\n"
        f"Orbital Elements:\n"
        f"  $a = {semi_major_axis}$ km, $e = {eccentricity}$\n"
        f"  $i = {np.rad2deg(inclination):.0f}^\\circ$, $\\Omega = {np.rad2deg(RAAN):.0f}^\\circ$, $\\omega = {np.rad2deg(arg_of_periapsis):.0f}^\\circ$"
    )

    return sat_dot, pos_line, r_vec_line, t_vec_line, h_vec_line, hud_text

anim = FuncAnimation(fig, update, frames=num_points, interval=25, blit=False, repeat=True)

plt.tight_layout()
plt.show()