# Class for describing a satellite

# State history should be attached to the object, in respective frames
# Maneuver planning
# Frames
# Something like .propagate() and .apply_maneuver() methods

# TODO: Change eci to inertial to keep things generalized

import numpy as np
from scipy.integrate import solve_ivp
from frames import Quaternion, FrameTree
from orbitCalculations import Orbit

class Spacecraft:
    # INITIALIZATION METHODS
    def __init__(self, name: str, initial_orbit: Orbit, initial_nu=0):
        self.name = name
        self.initial_orbit = initial_orbit
        self.initial_nu = initial_nu

        # Frames for spacecraft 
        # Note: Technically should be pqw_0 since it can change with time.
        #   I think later updates should maybe consider frame handling with time, so either this is just index [0] or something
        self.local_frames = {
            'pqw': f'{self.name.lower()}_pqw',
            'rth_0': f'{self.name.lower()}_rth_0'
        }

        # Propagation and Trajectory information
        # Note: Make this dynamic. Something like do everything in inertial, then if I request spacecraft.get_state('r', 'ecf')
        #   'r' in that example could instead just be indices of state [0:3] for r.
        #   Markers and basis could work similarly, or be updated once adding logic for event scheduling.
        self.t = None
        self.r_eci = None
        self.v_eci = None
        self.basis = {}
        self.markers = []

    def register_frames(self, ft: FrameTree):
        # pqw / eci relation
        q_c1_eci = Quaternion.from_axis_angle([0,0,1], self.initial_orbit.raan) # raan
        q_c2_c1 = Quaternion.from_axis_angle([1,0,0], self.initial_orbit.inclination) # inclination
        q_pqw_c2 = Quaternion.from_axis_angle([0,0,1], self.initial_orbit.arg_of_periapsis) # aop
        q_pqw_eci = q_pqw_c2 * q_c2_c1 * q_c1_eci
        ft.add_frame(self.local_frames['pqw'], 'eci', q_from_parent=q_pqw_eci)

        # rth / pqw
        q_rth_pqw = Quaternion.from_axis_angle([0,0,1], self.initial_nu) # true anomaly
        ft.add_frame(self.local_frames['rth_0'], self.local_frames['pqw'], q_from_parent=q_rth_pqw)

    def get_initial_state_eci(self, ft: FrameTree):
        self.initial_state_rth = self.initial_orbit.get_state_rth(self.initial_nu)
        r_0_rth = self.initial_state_rth[:3]
        v_0_rth = self.initial_state_rth[3:]

        # Transform using local rth frame
        r_0_eci = ft.transform(r_0_rth, self.local_frames['rth_0'], 'eci')
        v_0_eci = ft.transform(v_0_rth, self.local_frames['rth_0'], 'eci')
        return np.concatenate((r_0_eci, v_0_eci))
        
    # OTHER METHODS
    
    # Propagation with maneuvers
    # Notes for improvements and future
    #   Generalize this using a schedule for events. Ideally handles different types of event triggers & N number of events
    #   state_0 is passed in currently for this first setup. In future we should have the state of the vehicle, so no need to pass it into this function
    #   mu is assumed to be a single body. This is a specific to the current dynamics model and should be changed/adapted later when adding multibody differential equations of motion
    def propagate_with_maneuvers(self, t_span, state_0, target_time: float, dv_mag: float, num_points: int = 3000):
        def maneuver_event(time, state, mu):
            return time - target_time
        maneuver_event.terminal = True
        maneuver_event.direction = 0

        sol1 = solve_ivp(
            self._dynamics, t_span, state_0,
            method='RK45', atol=1e-12, rtol=1e-12,
            args=(self.initial_orbit.mu,), events=maneuver_event, dense_output=True)

        if sol1.status == 1:
            # Get final state
            r_eci_final = sol1.y[:3, -1]
            v_eci_final = sol1.y[3:6, -1]

            # Get final time
            t_burn = sol1.t[-1]

            # Get direction of velocity
            v_hat = v_eci_final / np.linalg.norm(v_eci_final)

            v_after_burn = v_eci_final + v_hat * dv_mag

            # Construct state to continue propagation
            state_eci = np.concatenate((r_eci_final, v_after_burn))

            sol2 = solve_ivp(
                self._dynamics, [t_burn, t_span[-1]], state_eci,
                method='RK45', atol=1e-12, rtol=1e-12,
                args=(self.initial_orbit.mu,), dense_output=True)

            if num_points is not None:
                # Sample continuous polynomial solutions onto a uniform linear time grid
                time_grid = np.linspace(t_span[0], t_span[-1], num_points)
                mask1 = time_grid <= t_burn
                mask2 = time_grid > t_burn

                y_full = np.zeros((6, len(time_grid)))
                y_full[:, mask1] = sol1.sol(time_grid[mask1])
                y_full[:, mask2] = sol2.sol(time_grid[mask2])
                self.t = time_grid
                self.r_eci = y_full[:3]
                self.v_eci = y_full[3:6]
                burn_idx = int(np.searchsorted(time_grid, t_burn))
            else:
                self.t = np.concatenate((sol1.t, sol2.t))
                y_full = np.concatenate((sol1.y, sol2.y), axis=1)
                self.r_eci = y_full[:3]
                self.v_eci = y_full[3:6]
                burn_idx = int(np.searchsorted(self.t, t_burn))

            # Add markers for visualization
            self.markers = [
                {"label": "Start", "index": 0},
                {"label": f"ΔV (+{int(dv_mag * 1000)} m/s)", "index": burn_idx},
                {"label": "End", "index": len(self.t) - 1},
            ]
        else:
            self.t = sol1.t
            self.r_eci = sol1.y[:3]
            self.v_eci = sol1.y[3:6]

    @staticmethod
    # Define simple dynamics
    def _dynamics(time, state, mu):
        # Split state
        r_vec_cart = state[0:3]
        v_vec_cart = state[3:6]

        r_mag_km = np.linalg.norm(r_vec_cart)

        f_0 = np.concatenate((v_vec_cart, -mu / r_mag_km**3 * r_vec_cart))

        state_dot = f_0
        return state_dot

    # Getting different frame positional information
    def get_state(self, target_frame: str, ft: FrameTree, index=None, return_basis=False):

        if index is not None:
            t = self.t[index]
            r_inertial = self.r_eci[:, index]
            v_inertial = self.v_eci[:, index]

            if 'ecf' in ft.frames:
                q_ecf_eci = Quaternion.from_axis_angle([0,0,1], self.initial_orbit.body.rotation_rate*t)
                ft.update_frame('ecf', q_from_parent=q_ecf_eci)

            if target_frame == 'lla':
                r_ecf = ft.transform(r_inertial, 'eci', 'ecf')

                r_mag = np.linalg.norm(r_ecf)
                lat = np.asin(r_ecf[2] / r_mag)
                lon = np.atan2(r_ecf[1], r_ecf[0])
                alt = r_mag - self.initial_orbit.body.radius
                r_lla = np.stack([lat, lon, alt])

                if return_basis:
                    raise ValueError('LLA does not have a basis. Set return_basis=False')

                return r_lla

            if target_frame == 'rth':
                r_mag = np.linalg.norm(r_inertial)
                r_hat = r_inertial / r_mag
                h_vec = np.cross(r_inertial, v_inertial)
                h_hat = h_vec / np.linalg.norm(h_vec)
                e_hat = np.cross(h_hat, r_hat)
                basis_rth_inertial = np.stack([r_hat, e_hat, h_hat])
                r_rth = basis_rth_inertial @ r_inertial
                v_rth = basis_rth_inertial @ v_inertial
                state_target = np.concatenate((r_rth, v_rth))

                if return_basis:
                    return state_target, basis_rth_inertial
                return state_target
            
            # Future updates to pqw will need to account for _0, _1 frame naming
            # Conceptual future design:
            # self.pqw_frames = [
            #     {"t_start": 0.0, "t_end": 10000.0, "frame_name": "1_pqw_0"},
            #     {"t_start": 10000.0, "t_end": np.inf, "frame_name": "1_pqw_1"}
            # ]
            if target_frame == 'pqw':
                target_frame = self.local_frames['pqw']

            r_new = ft.transform(r_inertial, 'eci', target_frame)
            v_new = ft.transform(v_inertial, 'eci', target_frame)
            state_target = np.concatenate((r_new, v_new))

            if return_basis:
                return state_target, ft.get_basis(target_frame, in_frame='eci')
            else:
                return state_target
        
        else:
            r_inertial = self.r_eci
            v_inertial = self.v_eci
            N = len(self.t)

            state_dim = 3 if target_frame == 'lla' else 6
            states = np.zeros((state_dim, N))
            bases = np.zeros((N, 3, 3)) if return_basis else None

            for k in range(N):
                if return_basis:
                    st, bs = self.get_state(target_frame, ft, index=k, return_basis=True)
                    states[:,k] = st
                    bases[k, :, :] = bs
                else:
                    st = self.get_state(target_frame, ft, index=k)
                    states[:,k] = st
                
            if return_basis:
                return states, bases
            else:
                return states
    