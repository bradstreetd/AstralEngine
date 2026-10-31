using UnityEngine;

namespace OrbitVisualization.Visuals
{
    /// <summary>
    /// Renders a 3-axis basis vector triad in the Unity Game view using adaptive LineRenderers
    /// that scale dynamically with camera distance so they never disappear when zoomed out.
    /// Follows standard color coding:
    ///   Axis 1 (X / I / P / R): Red
    ///   Axis 2 (Y / J / Q / T): Green
    ///   Axis 3 (Z / K / W / H): Blue
    /// </summary>
    public class BasisVectorTriad : MonoBehaviour
    {
        [Header("Triad Settings")]
        public float axisLength = 15.0f;
        public float baseLineWidth = 0.25f;

        [Tooltip("If true, line width scales with camera distance so lines stay visible at massive zoom distances.")]
        public bool adaptiveLineWidth = true;

        [Header("Standard Colors (X=Red, Y=Green, Z=Blue)")]
        public Color axis1Color = new Color(1.00f, 0.18f, 0.18f, 1f); // Red   (X / I / P / R)
        public Color axis2Color = new Color(0.18f, 0.95f, 0.25f, 1f); // Green (Y / J / Q / T)
        public Color axis3Color = new Color(0.20f, 0.55f, 1.00f, 1f); // Blue  (Z / K / W / H)

        public Material lineMaterial;

        private LineRenderer _line1;
        private LineRenderer _line2;
        private LineRenderer _line3;

        private Vector3 _currentOrigin = Vector3.zero;
        private Vector3 _v1 = Vector3.right;
        private Vector3 _v2 = Vector3.forward;
        private Vector3 _v3 = Vector3.up;
        private float _currentLength = 15.0f;
        private bool _isVisible = true;

        private void Awake()
        {
            InitializeLines();
        }

        private void InitializeLines()
        {
            if (_line1 != null) return;

            _line1 = CreateLineRenderer("Axis_1_Red_X", axis1Color);
            _line2 = CreateLineRenderer("Axis_2_Green_Y", axis2Color);
            _line3 = CreateLineRenderer("Axis_3_Blue_Z", axis3Color);
        }

        private LineRenderer CreateLineRenderer(string childName, Color color)
        {
            GameObject child = new GameObject(childName);
            child.transform.SetParent(transform, false);

            LineRenderer lr = child.AddComponent<LineRenderer>();
            lr.positionCount = 2;
            lr.startWidth = baseLineWidth;
            lr.endWidth = baseLineWidth * 0.7f;
            lr.useWorldSpace = true;

            // Use provided material or create an unlit colored material
            if (lineMaterial != null)
            {
                lr.material = new Material(lineMaterial);
            }
            else
            {
                Shader shader = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Sprites/Default");
                lr.material = new Material(shader);
            }

            lr.material.color = color;
            lr.startColor = color;
            lr.endColor = color;

            return lr;
        }

        public void SetColors(Color c1, Color c2, Color c3)
        {
            axis1Color = c1;
            axis2Color = c2;
            axis3Color = c3;

            if (_line1 != null) { _line1.startColor = c1; _line1.endColor = c1; _line1.material.color = c1; }
            if (_line2 != null) { _line2.startColor = c2; _line2.endColor = c2; _line2.material.color = c2; }
            if (_line3 != null) { _line3.startColor = c3; _line3.endColor = c3; _line3.material.color = c3; }
        }

        /// <summary>
        /// Updates the origin and directional vectors of the triad.
        /// </summary>
        public void SetVectors(Vector3 origin, Vector3 v1, Vector3 v2, Vector3 v3, float length = -1f)
        {
            if (_line1 == null) InitializeLines();

            _currentOrigin = origin;
            _v1 = v1.normalized;
            _v2 = v2.normalized;
            _v3 = v3.normalized;
            _currentLength = length > 0 ? length : axisLength;

            UpdateLinePositions();
        }

        private void LateUpdate()
        {
            if (!_isVisible) return;

            // Adapt line thickness dynamically based on distance to main camera
            if (adaptiveLineWidth && Camera.main != null)
            {
                float camDist = Vector3.Distance(_currentOrigin, Camera.main.transform.position);
                // Keep thickness proportional to screen size (at least baseLineWidth)
                float dynamicWidth = Mathf.Max(baseLineWidth, camDist * 0.0045f);

                if (_line1 != null) { _line1.startWidth = dynamicWidth; _line1.endWidth = dynamicWidth * 0.7f; }
                if (_line2 != null) { _line2.startWidth = dynamicWidth; _line2.endWidth = dynamicWidth * 0.7f; }
                if (_line3 != null) { _line3.startWidth = dynamicWidth; _line3.endWidth = dynamicWidth * 0.7f; }
            }

            UpdateLinePositions();
        }

        private void UpdateLinePositions()
        {
            if (_line1 == null) return;

            _line1.SetPosition(0, _currentOrigin);
            _line1.SetPosition(1, _currentOrigin + _v1 * _currentLength);

            _line2.SetPosition(0, _currentOrigin);
            _line2.SetPosition(1, _currentOrigin + _v2 * _currentLength);

            _line3.SetPosition(0, _currentOrigin);
            _line3.SetPosition(1, _currentOrigin + _v3 * _currentLength);
        }

        /// <summary>
        /// Toggles visibility of all three axes.
        /// </summary>
        public void SetVisible(bool isVisible)
        {
            _isVisible = isVisible;
            if (_line1 == null) InitializeLines();
            if (_line1 != null) _line1.enabled = isVisible;
            if (_line2 != null) _line2.enabled = isVisible;
            if (_line3 != null) _line3.enabled = isVisible;
        }
    }
}
