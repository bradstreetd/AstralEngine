# Orbit Mechanics & Unity 3D Visualizer

A modular astrodynamics computation suite paired with a real-time, interactive Unity 3D orbit visualizer. High-precision numerical propagation and coordinate frame transformations are executed in Python and streamed live to Unity over a low-latency TCP bridge.

---

## Architecture Overview

```
 ┌────────────────────────────────────────┐             TCP Socket (Port 5050)
 │          Python Astrodynamics          │ ──────────────────────────────────────────────┐
 │  - Numerical Integrator (RK45)         │      [4-byte Length Header] + [JSON Payload]  │
 │  - Keplerian Orbit & Frame Tree Engine │                                               │
 │  - LLA / Ground Track Computation      │                                               ▼
 └────────────────────────────────────────┘                                 ┌───────────────────────────┐
                                                                            │   Unity 3D Visualizer     │
                                                                            │  - Real-time 3D Trajectory│
                                                                            │  - N-Body / Multi-Sat     │
                                                                            │  - Basis Vector Triads    │
                                                                            │  - 2D Ground Track HUD    │
                                                                            └───────────────────────────┘
```

- **Python Core**:
  - `orbitCalculations.py`: Keplerian orbital elements, anomalies, energy, vis-viva, and celestial body constants (Earth, Titan, Mars, etc.).
  - `frames.py`: Quaternion math, direction cosine matrices (DCM), and a tree-based reference frame manager (`FrameTree`) supporting ECI/TCI, PQW, RTH, and ECF frames.
  - `unity_bridge.py`: Streaming serialization layer packaging trajectory states, multiple satellites, reference frame triads, and geodetic LLA coordinates to Unity via TCP.
  - Simulation scripts:
    - `simpleEarthOrbit.py`: Multi-orbit / multi-frame propagation with ECF rotation and LLA tracking.
    - `createOrbits.py`: Hyperbolic flyby trajectory analysis around Titan.
    - `visualizeOrbitPlane.py`: Matplotlib reference views.

- **Unity Visualizer (`OrbitVisualization/`)**:
  - `OrbitReceiver.cs`: Thread-safe TCP server receiving and queuing incoming telemetry packets.
  - `OrbitManager.cs`: Central orchestrator managing $N$ satellites, physically accurate celestial body scaling, line renderers, and time scrub / playback.
  - `OrbitUIController.cs`: Lightweight HUD with logarithmic simulation speed controls (up to 100,000x), named reference frame toggles, and an embedded 2D Ground Track (Latitude / Longitude) display with fading trails.
  - `BlenderCamera.cs`: Smooth orbit and zoom camera with gimbal lock prevention, logarithmic distance zoom, and target switching (`Central Body` $\leftrightarrow$ `Satellites`).
  - `BasisVectorTriad.cs`: Camera-distance-adaptive basis vector triads (Red = X/R, Green = Y/T, Blue = Z/H).

---

## Prerequisites

- **Python**: 3.10+
  - Dependencies: `numpy`, `scipy`, `matplotlib`
- **Unity**: Unity 2022.3 LTS (or Unity 6) with Universal Render Pipeline (URP) and the new Input System package.

---

## Getting Started

### 1. Python Environment Setup
```bash
# Clone the repository
git clone https://github.com/<username>/<repo-name>.git
cd <repo-name>

# Create and activate a virtual environment
python -m venv .venv
source .venv/bin/activate   # On Windows: .venv\Scripts\Activate.ps1

# Install requirements
pip install -r requirements.txt
```

### 2. Launch Unity Visualizer
1. Open Unity Hub and add the `OrbitVisualization` folder as a project.
2. Open the project in Unity.
3. Open `Assets/Scenes/SampleScene.unity`.
4. Press **Play** in the Unity Editor. The console will display:
   ```
   [OrbitReceiver] TCP Server started on 127.0.0.1:5050. Waiting for Python trajectories...
   ```

### 3. Run a Simulation
In a terminal, execute any orbit simulation script:
```bash
python simpleEarthOrbit.py
```
Unity will immediately receive the telemetry, spawn the satellite(s), draw the trajectory path, display the active reference frames, and plot the 2D ground track.

---

## Visualizer Controls

| Control | Action |
| :--- | :--- |
| **Right Mouse Drag** | Orbit camera around current focus target |
| **Mouse Scroll** | Logarithmic camera zoom (Shift for turbo, Ctrl/Alt for fine zoom) |
| **C / F** | Cycle focus target between Central Body and Satellites |
| **Spacebar** | Toggle simulation Play / Pause |
| **R** | Restart simulation from $t = 0$ |
| **Tab** | Toggle HUD UI visibility |
