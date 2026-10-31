using System;
using UnityEngine;

namespace OrbitVisualization.Core
{
    /// <summary>
    /// Data contract matching the JSON payload sent by unity_bridge.py
    /// </summary>
    [Serializable]
    public class OrbitPayload
    {
        public string mission_name;
        public string color_hex;
        public bool clear_existing = true;
        public float scale_km_per_unit = 1000f; // Default: 1000 km = 1 Unity Unit
        public CentralBodyData central_body;
        public float[] time;
        public float[][] trajectory_eci;    // N x 3 positions in km [x, y, z]
        public float[][] velocity_eci;      // N x 3 velocities in km/s [vx, vy, vz]
        public float[][] trajectory_lla;    // N x 3 LLA [lat (rad), lon (rad), alt (km)]
        public BasisFrameData[] basis_frames;
        public BasisVectorsData basis_vectors;
        public MarkerData[] markers;
    }

    [Serializable]
    public class CentralBodyData
    {
        public string name = "CentralBody";
        public float radius_km = 6378.137f;
        public float mu = 398600.4418f;
        public string color_hex = "#38bdf8";
    }

    [Serializable]
    public class BasisFrameData
    {
        public string name;
        // 1 x 3 x 3 for static frames, or N x 3 x 3 for time-varying frames
        public float[][][] matrices;
    }

    [Serializable]
    public class BasisVectorsData
    {
        // Legacy fallback fields
        public float[][] tci;
        public float[][] pqw;
        public float[][][] rth;
    }

    [Serializable]
    public class MarkerData
    {
        public string label;
        public int index;
    }

    /// <summary>
    /// Helper utilities to convert between Astrodynamics ECI coordinates (Right-Handed)
    /// and Unity 3D World coordinates (Left-Handed).
    /// </summary>
    public static class CoordinateConverter
    {
        /// <summary>
        /// Converts Right-Handed Astrodynamic ECI coordinates to Left-Handed Unity world space.
        /// 
        /// In Orbital Mechanics:
        ///   X = Vernal Equinox / Reference Direction (In-plane)
        ///   Y = In-plane Quadrature (+90 deg from X)
        ///   Z = Orbit Normal / Polar Axis (Up out of equatorial plane)
        /// 
        /// In Unity:
        ///   X = Right
        ///   Y = Up (World Up)
        ///   Z = Forward (World Depth)
        /// 
        /// Mapping:
        ///   Unity.X =  ECI.X / scale
        ///   Unity.Y =  ECI.Z / scale   (ECI Normal/Polar axis aligns with Unity Up)
        ///   Unity.Z =  ECI.Y / scale   (ECI Y aligns with Unity Depth)
        /// </summary>
        public static Vector3 EciToUnity(float x, float y, float z, float scaleKmPerUnit)
        {
            float scale = Mathf.Max(scaleKmPerUnit, 1e-4f);
            return new Vector3(x / scale, z / scale, y / scale);
        }

        public static Vector3 EciToUnity(float[] coords, float scaleKmPerUnit)
        {
            if (coords == null || coords.Length < 3) return Vector3.zero;
            return EciToUnity(coords[0], coords[1], coords[2], scaleKmPerUnit);
        }

        /// <summary>
        /// Converts a directional basis vector (unit vector) without applying distance scaling.
        /// </summary>
        public static Vector3 DirectionEciToUnity(float[] dir)
        {
            if (dir == null || dir.Length < 3) return Vector3.forward;
            return new Vector3(dir[0], dir[2], dir[1]).normalized;
        }
    }
}
