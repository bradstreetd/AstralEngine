using UnityEngine;
using UnityEngine.InputSystem;
using OrbitVisualization.Visuals;

namespace OrbitVisualization.CameraControl
{
    /// <summary>
    /// Smooth 3D Orbit & Zoom Camera Controller with gimbal clamp (never flips over),
    /// scene-scale proportional zoom, and dynamic target centering (Central Body vs Satellite).
    /// </summary>
    public class BlenderCamera : MonoBehaviour
    {
        public enum CameraFocusTarget
        {
            CentralBody,
            Satellite
        }

        [Header("Orbit Rotation Settings")]
        [Tooltip("Mouse sensitivity for orbiting around the target.")]
        public float orbitSpeed = 0.25f;

        [Tooltip("Minimum vertical pitch angle (prevents flipping underneath).")]
        public float minPitch = -85f;

        [Tooltip("Maximum vertical pitch angle (prevents flipping over the top).")]
        public float maxPitch = 85f;

        [Header("Zoom Settings (Scene-Scale Proportional)")]
        [Tooltip("Base zoom ratio per scroll notch (e.g. 1.35 = 35% distance change per tick).")]
        [Range(1.05f, 2.5f)]
        public float zoomRatio = 1.35f;

        [Tooltip("Multiplier applied to zoom sensitivity.")]
        public float zoomSensitivity = 1.0f;

        [Tooltip("Zoom speed multiplier when holding Shift (Turbo zoom).")]
        public float shiftSpeedMultiplier = 3.5f;

        [Tooltip("Zoom speed multiplier when holding Ctrl/Alt (Precision fine zoom).")]
        public float fineSpeedMultiplier = 0.25f;

        [Tooltip("Closest distance the camera can get to the target (in Unity units).")]
        public float minDistance = 0.5f;

        [Tooltip("Farthest distance the camera can get from the target.")]
        public float maxDistance = 25000.0f;

        [Tooltip("Smooth damping speed for zoom interpolation.")]
        public float zoomSmoothTime = 18f;

        [Header("Target Tracking & Framing")]
        [Tooltip("Current object the camera is centered on.")]
        public CameraFocusTarget focusTarget = CameraFocusTarget.CentralBody;

        [Tooltip("Smooth speed for moving the pivot when following a moving target.")]
        public float trackingSmoothSpeed = 15f;

        [Tooltip("Default camera distance when focusing on the satellite.")]
        public float satelliteFramingDistance = 12.0f;

        public Transform centralBodyTarget;
        public Transform satelliteTarget;

        // Internal State
        private Transform _camTransform;
        private float _targetDistance = 80f;
        private float _currentDistance = 80f;
        private float _yaw = 0f;
        private float _pitch = 25f;

        private OrbitManager _orbitManager;

        private void Start()
        {
            _orbitManager = FindAnyObjectByType<OrbitManager>();

            Camera cam = GetComponentInChildren<Camera>();
            if (cam != null)
            {
                cam.farClipPlane = 100000.0f; // 100,000 units prevents basis vectors and distant orbits from clipping
                cam.nearClipPlane = 0.1f;

                _camTransform = cam.transform;
                _currentDistance = Mathf.Abs(_camTransform.localPosition.z);
                if (_currentDistance < minDistance || _currentDistance > maxDistance)
                {
                    _currentDistance = Mathf.Clamp(_currentDistance, 10f, 300f);
                }
                _targetDistance = _currentDistance;
            }

            Vector3 euler = transform.eulerAngles;
            _yaw = euler.y;
            _pitch = euler.x;
            if (_pitch > 180f) _pitch -= 360f;
            _pitch = Mathf.Clamp(_pitch, minPitch, maxPitch);
        }

        private void Update()
        {
            HandleRotation();
            HandleZoom();
            HandleTargetToggle();
        }

        private void LateUpdate()
        {
            HandleTargetTracking();
            ApplyCameraDistance();
        }

        private void HandleRotation()
        {
            if (Mouse.current == null) return;

            // Right mouse button orbits the camera around the current pivot
            if (Mouse.current.rightButton.isPressed)
            {
                Vector2 delta = Mouse.current.delta.ReadValue();
                _yaw += delta.x * orbitSpeed;
                _pitch -= delta.y * orbitSpeed;

                // Gimbal Clamp: Prevents flipping over the top or under the bottom
                _pitch = Mathf.Clamp(_pitch, minPitch, maxPitch);
            }

            transform.rotation = Quaternion.Euler(_pitch, _yaw, 0f);
        }

        private void HandleZoom()
        {
            if (Mouse.current == null || _camTransform == null) return;

            float scroll = Mouse.current.scroll.ReadValue().y;
            if (Mathf.Abs(scroll) > 0.001f)
            {
                // Normalize scroll ticks (standard notch is +/- 120 in new Input System)
                float scrollTicks = Mathf.Sign(scroll) * Mathf.Max(Mathf.Abs(scroll) / 120f, 1.0f);

                // Speed modifier: Shift = Turbo, Ctrl/Alt = Fine
                float speedMultiplier = 1.0f;
                if (Keyboard.current != null)
                {
                    if (Keyboard.current.leftShiftKey.isPressed || Keyboard.current.rightShiftKey.isPressed)
                    {
                        speedMultiplier = shiftSpeedMultiplier;
                    }
                    else if (Keyboard.current.leftCtrlKey.isPressed || Keyboard.current.leftAltKey.isPressed)
                    {
                        speedMultiplier = fineSpeedMultiplier;
                    }
                }

                // Geometric/Power Scaling: Scales proportionally with the current distance
                // Zooming in divides by ratio; Zooming out multiplies by ratio
                float exponent = -scrollTicks * zoomSensitivity * speedMultiplier;
                float multiplier = Mathf.Pow(zoomRatio, exponent);

                _targetDistance = Mathf.Clamp(_targetDistance * multiplier, minDistance, maxDistance);
            }
        }

        private void ApplyCameraDistance()
        {
            if (_camTransform == null) return;

            // Smoothly interpolate camera zoom distance
            _currentDistance = Mathf.Lerp(_currentDistance, _targetDistance, Time.deltaTime * zoomSmoothTime);
            _camTransform.localPosition = new Vector3(0f, 0f, -_currentDistance);
        }

        private void HandleTargetToggle()
        {
            if (Keyboard.current == null) return;

            // Press C or F to toggle camera focus between Central Body and Satellite
            if (Keyboard.current.cKey.wasPressedThisFrame || Keyboard.current.fKey.wasPressedThisFrame)
            {
                ToggleFocusTarget();
            }
        }

        public void ToggleFocusTarget()
        {
            if (_orbitManager != null && _orbitManager.trackedOrbits.Count > 0)
            {
                if (focusTarget == CameraFocusTarget.CentralBody)
                {
                    _orbitManager.SelectOrbit(0);
                    SetFocusTarget(CameraFocusTarget.Satellite);
                }
                else
                {
                    int nextIdx = _orbitManager.activeOrbitIndex + 1;
                    if (nextIdx < _orbitManager.trackedOrbits.Count)
                    {
                        _orbitManager.SelectOrbit(nextIdx);
                        SetFocusTarget(CameraFocusTarget.Satellite);
                    }
                    else
                    {
                        SetFocusTarget(CameraFocusTarget.CentralBody);
                    }
                }
            }
            else
            {
                SetFocusTarget(focusTarget == CameraFocusTarget.CentralBody
                    ? CameraFocusTarget.Satellite
                    : CameraFocusTarget.CentralBody);
            }
        }

        public void SetFocusTarget(CameraFocusTarget target)
        {
            focusTarget = target;

            // Automatically adapt zoom framing when switching focus targets
            if (focusTarget == CameraFocusTarget.Satellite)
            {
                // If currently zoomed far out, frame closer to the satellite
                if (_targetDistance > satelliteFramingDistance * 3.0f)
                {
                    _targetDistance = satelliteFramingDistance;
                }
            }
            else if (focusTarget == CameraFocusTarget.CentralBody)
            {
                // If currently super close, frame the central body
                float bodyRadius = 5.0f;
                if (centralBodyTarget != null)
                {
                    bodyRadius = centralBodyTarget.localScale.x * 0.5f;
                }
                if (_targetDistance < bodyRadius * 2.0f)
                {
                    _targetDistance = Mathf.Max(bodyRadius * 4.0f, 40.0f);
                }
            }
        }

        private void HandleTargetTracking()
        {
            if (_orbitManager != null)
            {
                centralBodyTarget = _orbitManager.CentralBodyTransform;
                satelliteTarget = _orbitManager.SatelliteTransform;
            }

            Vector3 desiredPosition = Vector3.zero;

            if (focusTarget == CameraFocusTarget.Satellite && satelliteTarget != null)
            {
                desiredPosition = satelliteTarget.position;
            }
            else if (centralBodyTarget != null)
            {
                desiredPosition = centralBodyTarget.position;
            }

            // Smoothly glide the pivot to follow the focused target
            transform.position = Vector3.Lerp(transform.position, desiredPosition, Time.deltaTime * trackingSmoothSpeed);
        }
    }
}