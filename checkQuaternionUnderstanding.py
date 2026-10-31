# Rotation Matrix / Quaternion Checks
import matplotlib.pyplot as plt
import numpy as np

def calcQuaternion(axis_vec, angle):
    """
    Returns unit quaternion [x, y, z, w] representing a rotation of `angle`
    about `axis_vec` (Hamilton convention, scalar w is last).
    """
    axis = np.asarray(axis_vec, dtype=float)
    norm = np.linalg.norm(axis)
    if norm > 0:
        axis = axis / norm

    s = np.sin(angle / 2.0)
    w = np.cos(angle / 2.0)
    q = np.array([axis[0] * s, axis[1] * s, axis[2] * s, w])

    # Enforce canonical form (w >= 0)
    if q[3] < 0:
        q *= -1

    return q

def quat_conjugate(q):
    """Quaternion conjugate (inverse for unit quaternion)"""
    return q * np.array([-1.0, -1.0, -1.0, 1.0])

def quat_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    v1 = np.array([x1, y1, z1])
    v2 = np.array([x2, y2, z2])

    v = w1 * v2 + w2 * v1 + np.cross(v1, v2)
    w = w1 * w2 - np.dot(v2, v1)

    return np.append(v, w)

def quat_rotate(q, v):
    arg1 = quat_mult(quat_conjugate(q), np.append(v, 0))

    v_new_frame = quat_mult(arg1, q)[:3]
    return v_new_frame

num_points = 1000

# Define an orbit in the orbital plane
semi_major_axis = 7000
eccentricity = 0.3

true_anomaly_sweep = np.linspace(0, 1*np.pi, num_points)

# Initialize lists
r_rth = np.zeros((3, num_points))
r_pqw = np.zeros((3, num_points))
r_inc = np.zeros((3, num_points))

for k, true_anomaly in enumerate(true_anomaly_sweep):
    # R in RTH frame
    p = semi_major_axis * (1 - eccentricity**2)
    r_mag = p / (1 + eccentricity*np.cos(true_anomaly))
    r_rth[:, k] = np.array([r_mag, 0, 0])

    # Rotation of pqw -> rth (Would function as r_rth = rth_C_pqw * r_pqw)
    rth_Q_pqw = calcQuaternion(np.array([0, 0, 1]), true_anomaly)
    pqw_Q_inc = calcQuaternion(np.array([1, 0, 0]), 15*np.pi/180)

    # Conjugate to get rotation to convert INTO pqw
    pqw_Q_rth = quat_conjugate(rth_Q_pqw)
    inc_Q_rth = quat_conjugate(quat_mult(rth_Q_pqw, pqw_Q_inc))

    # Get r in pqw frame
    r_pqw[:, k] = quat_rotate(pqw_Q_rth, r_rth[:,k])
    r_inc[:, k] = quat_rotate(inc_Q_rth, r_rth[:, k])
    
# ==========================================
# Plotting with Matplotlib (Dark Mode)
# ==========================================
plt.style.use('dark_background')
breakpoint()
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.5))

# -------------------------------------------------------------
# 1. Ephemeris / Perifocal Frame: 2D Elliptical Orbit
# -------------------------------------------------------------
ax1.plot(r_pqw[0, :], r_pqw[1, :], label='Orbit Trajectory ($r_{eph}$)', color='#38bdf8', lw=2.2)
ax1.plot(0, 0, 'o', color='#fbbf24', markersize=10, markeredgecolor='white', markeredgewidth=1.2, label='Central Body (Focus)')
ax1.plot(r_pqw[0, 0], r_pqw[1, 0], 'o', color='#f43f5e', markersize=7, label=f'Periapsis ({r_pqw[0, 0]:.0f} km)')
apo_idx = num_points // 2
ax1.plot(r_pqw[0, apo_idx], r_pqw[1, apo_idx], 'o', color='#c084fc', markersize=7, label=f'Apoapsis ({-r_pqw[0, apo_idx]:.0f} km)')

# Draw frame axes at origin
ax1.axhline(0, color='#64748b', linestyle=':', alpha=0.5)
ax1.axvline(0, color='#64748b', linestyle=':', alpha=0.5)

ax1.set_title('Orbit in Perifocal Frame ($r_{eph}$)', fontsize=12, fontweight='bold', pad=12)
ax1.set_xlabel(r'$\hat{e}$ [km]', fontsize=11)
ax1.set_ylabel(r'$\hat{p}$ [km]', fontsize=11)
ax1.set_aspect('equal', 'datalim')
ax1.grid(True, linestyle='--', alpha=0.25)
ax1.legend(loc='upper right', framealpha=0.85)

# -------------------------------------------------------------
# 2. RTH Frame: Components vs True Anomaly
# -------------------------------------------------------------
nu_deg = np.rad2deg(true_anomaly_sweep)
ax2.plot(nu_deg, r_rth[0, :], label='Radial ($R$)', color='#f43f5e', lw=2.2)
ax2.plot(nu_deg, r_rth[1, :], label='Tangential ($T = 0$)', color='#2dd4bf', linestyle='--', lw=1.8)
ax2.plot(nu_deg, r_rth[2, :], label='Normal ($H = 0$)', color='#94a3b8', linestyle=':', lw=1.8)

ax2.set_title('Components in Rotating RTH Frame ($r_{rth}$)', fontsize=12, fontweight='bold', pad=12)
ax2.set_xlabel('True Anomaly $\\nu$ [deg]', fontsize=10)
ax2.set_ylabel('Position Magnitude [km]', fontsize=10)
ax2.set_xlim(0, 360)
ax2.set_xticks(np.arange(0, 361, 45))
ax2.grid(True, linestyle='--', alpha=0.25)
ax2.legend(loc='center right', framealpha=0.85)

plt.tight_layout()
plt.show()