"""
Coordinate Frame Tree and Quaternion System for Orbital Mechanics.
Provides FrameTree for managing coordinate transformations and basis vectors
across arbitrary reference frames (ECI, PQW, RTH, ECEF, BODY, etc.).
"""

import numpy as np


class Quaternion:
    """
    Unit Quaternion [x, y, z, w] representing a spatial rotation (Hamilton convention).
    Scalar w is stored last.
    """

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0):
        self.q = np.array([x, y, z, w], dtype=float)

    @classmethod
    def identity(cls) -> "Quaternion":
        """Returns the identity quaternion (no rotation)."""
        return cls(0.0, 0.0, 0.0, 1.0)

    @classmethod
    def from_axis_angle(cls, axis, angle: float) -> "Quaternion":
        """
        Creates a unit quaternion from an axis vector and rotation angle (in radians).
        Active rotation of angle about axis.
        """
        axis = np.asarray(axis, dtype=float)
        norm = np.linalg.norm(axis)
        if norm > 0:
            axis = axis / norm
        s = np.sin(angle / 2.0)
        c = np.cos(angle / 2.0)
        return cls(axis[0] * s, axis[1] * s, axis[2] * s, c)

    @classmethod
    def from_euler_313(cls, raan: float, inc: float, aop: float) -> "Quaternion":
        """
        Keplerian orbital plane rotation: converts ECI -> PQW coordinates.
        3-1-3 sequence: R3(aop) * R1(inc) * R3(raan).
        Returns the quaternion representing rotation from PQW to ECI (Q_eci_pqw).
        """
        q_raan = cls.from_axis_angle([0, 0, 1], raan)
        q_inc = cls.from_axis_angle([1, 0, 0], inc)
        q_aop = cls.from_axis_angle([0, 0, 1], aop)
        pqw_Q_eci = q_aop * q_inc * q_raan
        return ~pqw_Q_eci  # Returns Q_eci_pqw (to_parent)

    def conjugate(self) -> "Quaternion":
        """Returns the quaternion conjugate / inverse: [-x, -y, -z, w]."""
        return Quaternion(-self.q[0], -self.q[1], -self.q[2], self.q[3])

    def __invert__(self) -> "Quaternion":
        """Allows ~q syntax for quaternion conjugate/inverse."""
        return self.conjugate()

    def __mul__(self, other: "Quaternion") -> "Quaternion":
        """
        Hamilton quaternion composition: q_combined = q1 * q2
        Applies q2 first, then q1.
        """
        px, py, pz, pw = other.q
        qx, qy, qz, qw = self.q

        pv = np.array([px, py, pz])
        qv = np.array([qx, qy, qz])

        rv = pw*qv + qw*pv + np.cross(pv, qv)
        rw = pw*qw - np.dot(qv, pv)

        return Quaternion(rv[0], rv[1], rv[2], rw)
    
    def rotate(self, v) -> np.ndarray:
        v_new = self * Quaternion(v[0], v[1], v[2], 0) * ~self
        return v_new.q[:3]

    def __matmul__(self, v) -> np.ndarray:
        """
        Rotates vector v using Rodrigues' rotation formula: 
otated_v = q @ v.
        Supports:
          - Single 3D vector shape (3,)
          - 3x3 basis matrix shape (3, 3) (rotates each basis vector row)
          - Batch of vectors shape (3, N)
        """
        qv = self.q[:3]
        w = self.q[3]
        v_arr = np.asarray(v, dtype=float)

        # Handle 3x3 basis matrix
        if v_arr.ndim == 2 and v_arr.shape == (3, 3):
            return np.array([self @ row for row in v_arr])

        # Handle (3, N) batch
        if v_arr.ndim == 2 and v_arr.shape[0] == 3:
            t = 2.0 * np.cross(qv, v_arr, axis=0)
            return v_arr + w * t + np.cross(qv, t, axis=0)

        return self.rotate(v_arr)

    def basis(self) -> np.ndarray:
        """
        Returns the 3 unit basis vectors [e1, e2, e3] rotated by this quaternion.
        Shape is (3, 3) where:
          row 0: rotated e1 [1, 0, 0]
          row 1: rotated e2 [0, 1, 0]
          row 2: rotated e3 [0, 0, 1]
        """
        return self @ np.eye(3)

    def to_dcm(self) -> np.ndarray:
        """Converts quaternion to a 3x3 Direction Cosine Matrix (DCM)."""
        x, y, z, w = self.q
        return np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
        ])

    def __repr__(self) -> str:
        return f"Quaternion(x={self.q[0]:.5f}, y={self.q[1]:.5f}, z={self.q[2]:.5f}, w={self.q[3]:.5f})"


class FrameTree:
    """
    Hierarchical Coordinate Frame Tree / Graph.
    Manages transformations and basis vectors across arbitrary reference frames.
    """

    def __init__(self, root_frame: str = "eci"):
        self.root = root_frame.lower()
        self.frames = {
            self.root: {
                "parent": None,
                "q_to_parent": Quaternion.identity(),
            }
        }

    def add_frame(
        self,
        name: str,
        parent: str,
        q_to_parent: Quaternion = None,
        q_from_parent: Quaternion = None,
    ):
        """
        Adds a new frame to the tree.
        Specify either q_to_parent (Q_parent_child) or q_from_parent (Q_child_parent).
        """
        name = name.lower()
        parent = parent.lower()

        if parent not in self.frames:
            raise ValueError(f"Parent frame '{parent}' is not registered in FrameTree.")

        if q_to_parent is not None:
            q = q_to_parent
        elif q_from_parent is not None:
            q = ~q_from_parent
        else:
            q = Quaternion.identity()

        self.frames[name] = {"parent": parent, "q_to_parent": q}

    def update_frame(
        self,
        name: str,
        q_to_parent: Quaternion = None,
        q_from_parent: Quaternion = None,
    ):
        """Updates the rotation of an existing frame relative to its parent."""
        name = name.lower()
        if name not in self.frames:
            raise ValueError(f"Frame '{name}' is not registered in FrameTree.")

        if q_to_parent is not None:
            self.frames[name]["q_to_parent"] = q_to_parent
        elif q_from_parent is not None:
            self.frames[name]["q_to_parent"] = ~q_from_parent

    def _get_path_to_root(self, frame: str):
        path = []
        curr = frame.lower()
        while curr is not None:
            path.append(curr)
            curr = self.frames[curr]["parent"]
        return path

    def get_rotation(self, from_frame: str, to_frame: str) -> Quaternion:
        '''
        Calculates the quaternion rotation from 'from_frame' to 'to_frame'.
        It goes up the tree from both frames to their Lowest Common Ancestor (LCA), then conbimes the paths.
        '''
        from_f = from_frame.lower()
        to_f = to_frame.lower()

        if from_f not in self.frames or to_f not in self.frames:
            raise KeyError('One or both frames not found in FrameTree.')

        # If given the same frames for some reason
        if from_f == to_f:
            return Quaternion.identity()

        # Find the shared ancestor
        from_path = self._get_path_to_root(from_f)
        to_path_set = set(self._get_path_to_root(to_f))

        # Find Lowest Common Ancestor (LCA)
        lca = next((f for f in from_path if f in to_path_set), None)

        if not lca:
            raise ValueError(f"No common ancestor between frames '{from_frame}' and '{to_frame}'.")

        # Walk from from_frame up to LCA: Q_lca_from
        q_lca_from = Quaternion.identity()
        curr = from_f
        while curr != lca:
            parent = self.frames[curr]["parent"]
            q_parent_curr = self.frames[curr]["q_to_parent"]
            q_lca_from = q_parent_curr * q_lca_from
            curr = parent

        # Walk from to_frame up to LCA: Q_lca_to
        q_lca_to = Quaternion.identity()
        curr = to_f
        while curr != lca:
            parent = self.frames[curr]["parent"]
            q_parent_curr = self.frames[curr]["q_to_parent"]
            q_lca_to = q_parent_curr * q_lca_to
            curr = parent

        # Composed rotation: Q_to_from = (~Q_lca_to) * Q_lca_from
        return (~q_lca_to) * q_lca_from

    def transform(self, v, from_frame: str, to_frame: str) -> np.ndarray:
        """
        Transforms a vector (or array of vectors) from from_frame to to_frame.
        """
        q = self.get_rotation(from_frame=from_frame, to_frame=to_frame)
        return q @ v

    def get_basis(self, frame: str, in_frame: str = "eci") -> np.ndarray:
        """
        Returns the 3 basis vectors [e1, e2, e3] in in_frame coordinates.
        Shape: (3, 3) where:
          row 0: basis vector 1 (e.g. R_hat or P_hat)
          row 1: basis vector 2 (e.g. T_hat or Q_hat)
          row 2: basis vector 3 (e.g. H_hat or W_hat)
        """
        q = self.get_rotation(from_frame=frame, to_frame=in_frame)
        return q.basis()
