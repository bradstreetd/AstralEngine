'''
This script is for the development of adding maneuvers into the propagation method used (solve_ivp).
Maneuvers are impulsive
'''

# Imports
import numpy as np
from orbitCalculations import Orbit, EARTH
from frames import Quaternion, FrameTree
from spacecraft import Spacecraft
from eventHelper import EventScheduler, TimeTrigger, Maneuver


# Set up frames
ft = FrameTree(root_frame='eci')
ft.add_frame('ecf', 'eci', q_from_parent=Quaternion(0, 0, 0, 1))

# Basic earth orbit
earth_orbit = Orbit(
    mu = EARTH.mu,
    semi_major_axis = EARTH.radius + 800,
    eccentricity = 0.0,
    inclination = 50*np.pi/180,
    raan = 75*np.pi/180,
    arg_of_periapsis = 140*np.pi/180,
    body = EARTH
)

# Get orbit period for time to propagate
period = earth_orbit.period
tspan = [0, period*10]

# Spacecraft 1
start_nu = 90 *np.pi/180
spacecraft_1 = Spacecraft('1', earth_orbit, start_nu)
spacecraft_1.register_frames(ft)
spacecraft_1.add_event(Maneuver(
    'Burn 1',
    trigger=TimeTrigger(target_time=10000),
    delta_v=np.array([0,0.05,0]),
    frame='rth'))
spacecraft_1.propagate(tspan, ft)

# Spacecraft 2 (No Maneuver)
spacecraft_2 = Spacecraft('2', earth_orbit, start_nu)
spacecraft_2.register_frames(ft)
spacecraft_2.propagate(tspan, ft)

# Example to show getting basis for all frames
# r_lla = spacecraft_1.get_state('lla', ft)
# _, basis_ecf_eci = spacecraft_1.get_state('ecf', ft, return_basis=True)
# _, basis_rth_eci = spacecraft_1.get_state('rth', ft, return_basis=True)
# _, basis_pqw_eci = spacecraft_1.get_state('pqw', ft, return_basis=True)
# basis_eci_eci = ft.get_basis('eci', 'eci')

# # Compile basis vectors into a dictionary for Unity
# all_basis = {
#     'ECI': basis_eci_eci,
#     'PQW': basis_pqw_eci,
#     'RTH': basis_rth_eci,
#     'ECF': basis_ecf_eci,
# }

def custom_send_orbit(sc: Spacecraft, ft: FrameTree):
    r_lla = spacecraft_1.get_state('lla', ft)
    _, basis_ecf_eci = spacecraft_1.get_state('ecf', ft, return_basis=True)
    _, basis_ecf_eci = spacecraft_1.get_state('ecf', ft, return_basis=True)
    _, basis_rth_eci = spacecraft_1.get_state('rth', ft, return_basis=True)
    basis_eci_eci = ft.get_basis('eci', 'eci')

    all_basis = {
        'ECI': basis_eci_eci,
        'RTH': basis_rth_eci,
        'ECF': basis_ecf_eci,
    }

    # Send trajectory to Unity 3D visualizer (if active)
    from unity_bridge import send_orbit
    send_orbit(
        name=sc.name,
        time=sc.t,
        r_eci=sc.r_eci,
        v_eci=sc.v_eci,
        r_lla=sc.get_state('lla', ft),
        central_body=EARTH,
        all_basis=all_basis,
        markers=spacecraft_1.markers,
        scale_km_per_unit=1000.0,
    )

custom_send_orbit(spacecraft_1, ft)
custom_send_orbit(spacecraft_2, ft)