"""
Unity Bridge for Orbit Mechanics Visualization.
Streams precomputed or live orbit trajectory batches to a Unity TCP server.

Packet Framing:
  [4-byte Big-Endian Length Header] + [UTF-8 JSON Payload]
"""

import json
import socket
from typing import Any, Dict, List, Optional, Union
import numpy as np


class NumpyEncoder(json.JSONEncoder):
    """Encodes NumPy arrays and numerical scalars to standard JSON types."""

    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.floating, np.float32, np.float64)):
            return float(obj)
        if isinstance(obj, (np.integer, np.int32, np.int64)):
            return int(obj)
        return super().default(obj)


def send_payload(
    payload: Dict[str, Any],
    host: str = "127.0.0.1",
    port: int = 5050,
    timeout: float = 1.5,
) -> bool:
    """
    Sends a JSON payload dictionary to Unity via TCP with a 4-byte length prefix.

    Returns:
        bool: True if successfully delivered to Unity, False if Unity is offline.
    """
    try:
        json_str = json.dumps(payload, cls=NumpyEncoder)
        json_bytes = json_str.encode("utf-8")
        header = len(json_bytes).to_bytes(4, byteorder="big")

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(timeout)
            s.connect((host, port))
            s.sendall(header + json_bytes)

        print(f"[UnityBridge] Successfully sent orbit payload to Unity ({len(json_bytes) / 1024:.1f} KB).")
        return True

    except (ConnectionRefusedError, TimeoutError, OSError):
        print(f"[UnityBridge] Notice: Unity is not listening on {host}:{port}. (Start Play mode in Unity to view in 3D).")
        return False


# Tracks whether send_orbit has already been called in the current Python script execution.
# First call clears previous run's orbits in Unity; subsequent calls append additional orbits.
_sent_in_current_process = False


def send_orbit(
    name: str,
    time: Union[np.ndarray, List[float]],
    r_eci: np.ndarray,
    v_eci: Optional[np.ndarray] = None,
    r_lla: Optional[np.ndarray] = None,
    central_body: Optional[Any] = None,
    all_basis: Optional[Dict[str, np.ndarray]] = None,
    basis_tci: Optional[np.ndarray] = None,
    basis_pqw: Optional[np.ndarray] = None,
    basis_rth: Optional[np.ndarray] = None,
    markers: Optional[List[Dict[str, Any]]] = None,
    color: Optional[str] = None,
    clear_existing: Optional[bool] = None,
    scale_km_per_unit: float = 1000.0,
    host: str = "127.0.0.1",
    port: int = 5050,
) -> bool:
    """
    Convenience wrapper to package and transmit full orbit trajectory data to Unity.
    Calling send_orbit() multiple times in a script adds multiple satellites/orbits to the scene.

    Parameters:
        name: Name of mission or trajectory (e.g., "Orbit 1")
        time: 1D array of simulation time steps (seconds)
        r_eci: Position vectors in ECI/inertial frame. Shape (3, N) or (N, 3) in km.
        v_eci: Optional velocity vectors in ECI frame. Shape (3, N) or (N, 3) in km/s.
        r_lla: Optional Geodetic/Spherical LLA coordinates [lat (rad), lon (rad), alt (km)]. Shape (3, N) or (N, 3).
        central_body: Optional Body object or dict with 'name', 'radius', 'mu', 'color'.
        all_basis: Optional dict mapping custom frame names (e.g. 'ECI', 'PQW', 'RTH', 'ECF')
                   to static (3, 3) or dynamic (N, 3, 3) / (3, 3, N) basis arrays.
        basis_tci: Optional legacy static TCI/ECI inertial basis vectors. Shape (3, 3).
        basis_pqw: Optional legacy static PQW basis vectors. Shape (3, 3).
        basis_rth: Optional legacy dynamic RTH basis vectors over time. Shape (3, 3, N) or (N, 3, 3).
        markers: Optional list of labeled points (e.g. [{'label': 'Periapsis', 'index': 0}]).
        color: Optional hex color string (e.g. '#38bdf8'). If omitted, Unity assigns a distinct default color.
        clear_existing: If True, clears existing orbits in Unity before adding this one.
                        Defaults to True on the first send_orbit() call of a script run, and False on subsequent calls.
        scale_km_per_unit: Distance scaling factor for Unity world space (default: 1000 km = 1 unit).
        host: Unity listener IP address (default: localhost).
        port: Unity listener port (default: 5050).
    """
    global _sent_in_current_process
    if clear_existing is None:
        clear_existing = not _sent_in_current_process
    _sent_in_current_process = True

    r_arr = np.asarray(r_eci, dtype=float)
    # Ensure r is (N, 3) for clean serialization
    if not len(r_arr.shape) == 1 and r_arr.shape[0] == 3 and r_arr.shape[1] != 3:
        r_coords = r_arr.T
    else:
        r_coords = r_arr

    v_coords = None
    if v_eci is not None:
        v_arr = np.asarray(v_eci, dtype=float)
        if not len(v_arr.shape) == 1 and v_arr.shape[0] == 3 and v_arr.shape[1] != 3:
            v_coords = v_arr.T
        else:
            v_coords = v_arr

    lla_coords = None
    if r_lla is not None:
        lla_arr = np.asarray(r_lla, dtype=float)
        if not len(lla_arr.shape) == 1 and lla_arr.shape[0] == 3 and lla_arr.shape[1] != 3:
            lla_coords = lla_arr.T
        else:
            lla_coords = lla_arr

    # Length consistency validation
    t_len = len(time)
    r_len = len(r_coords)
    if t_len != r_len:
        print(f"[UnityBridge] WARNING: Time array length ({t_len}) does not match trajectory positions ({r_len})!")
    if lla_coords is not None and len(lla_coords) != r_len:
        print(f"[UnityBridge] WARNING: LLA array length ({len(lla_coords)}) does not match trajectory positions ({r_len})!")
    if v_coords is not None and len(v_coords) != r_len:
        print(f"[UnityBridge] WARNING: Velocity array length ({len(v_coords)}) does not match trajectory positions ({r_len})!")

    # Body information
    body_data = {}
    if central_body is not None:
        if hasattr(central_body, "name"):
            body_data = {
                "name": getattr(central_body, "name", "CentralBody"),
                "radius_km": getattr(central_body, "radius", 6378.137),
                "mu": getattr(central_body, "mu", 398600.4418),
            }
        elif isinstance(central_body, dict):
            body_data = central_body

    # Format arbitrary named basis frames
    def _normalize_basis_array(arr: np.ndarray) -> np.ndarray:
        b = np.asarray(arr, dtype=float)
        if b.ndim == 2:
            # Static (3, 3) -> (1, 3, 3)
            return b[np.newaxis, :, :]
        if b.ndim == 3 and b.shape[0] == 3 and b.shape[1] == 3 and b.shape[2] != 3:
            # Dynamic (3, 3, N) -> (N, 3, 3)
            return np.transpose(b, (2, 0, 1))
        return b

    basis_frames = []
    if all_basis is not None:
        for frame_name, b_arr in all_basis.items():
            if b_arr is not None:
                basis_frames.append({
                    "name": str(frame_name),
                    "matrices": _normalize_basis_array(b_arr),
                })
    else:
        # Fallback if legacy individual arguments were used
        if basis_tci is not None:
            basis_frames.append({"name": "TCI", "matrices": _normalize_basis_array(basis_tci)})
        if basis_pqw is not None:
            basis_frames.append({"name": "PQW", "matrices": _normalize_basis_array(basis_pqw)})
        if basis_rth is not None:
            basis_frames.append({"name": "RTH", "matrices": _normalize_basis_array(basis_rth)})

    # Default markers if none provided
    if markers is None:
        markers = [
            {"label": "Start", "index": 0},
            {"label": "End", "index": len(r_coords) - 1},
        ]

    payload = {
        "mission_name": name,
        "color_hex": color,
        "clear_existing": bool(clear_existing),
        "scale_km_per_unit": scale_km_per_unit,
        "central_body": body_data,
        "time": np.asarray(time, dtype=float),
        "trajectory_eci": r_coords,
        "velocity_eci": v_coords,
        "trajectory_lla": lla_coords,
        "basis_frames": basis_frames,
        "markers": markers,
    }

    return send_payload(payload, host=host, port=port)


if __name__ == "__main__":
    # Self-test: Generate a simple sample orbit payload and attempt transmission
    t = np.linspace(0, 360, 36)
    dummy_r = np.array([7000.0 * np.cos(np.radians(t)), 7000.0 * np.sin(np.radians(t)), np.zeros_like(t)])
    print("[UnityBridge] Running self-test transmission...")
    send_orbit(
        name="SelfTest Orbit",
        time=t,
        r_eci=dummy_r,
        central_body={"name": "Earth", "radius_km": 6378.137, "mu": 398600.4},
        scale_km_per_unit=1000.0,
    )
