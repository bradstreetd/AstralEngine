using System.Collections.Generic;
using UnityEngine;
using OrbitVisualization.Core;

namespace OrbitVisualization.Visuals
{
    [System.Serializable]
    public class RuntimeBasisFrame
    {
        public string name;
        public bool isVisible = true;
        public bool anchorToSatellite = false;
        public float lengthMultiplier = 1.0f;
        [HideInInspector] public BasisVectorTriad triad;
    }

    [System.Serializable]
    public class TrackedOrbit
    {
        public string name;
        public Color color;
        public bool isVisible = true;
        public OrbitPayload payload;
        public float[] times;
        public Vector3[] unityTrajectory;
        public Vector3[] unityVelocities;
        public Vector3[] llaData; // x = Lat (deg), y = Lon (deg), z = Alt (km)
        public Vector3 currentLLA;
        public int currentSampleIdx;
        public float currentSampleAlpha;
        public GameObject satelliteInstance;
        public LineRenderer pathLineRenderer;
        public Dictionary<string, float[][][]> basisMatrices = new Dictionary<string, float[][][]>();

        public bool HasLLAData => llaData != null && llaData.Length > 1;
    }

    /// <summary>
    /// Central manager for the orbit scene.
    /// Supports N simultaneous satellites/orbits with distinct colors applied to SatGlow Base & Emission maps,
    /// dynamic named basis vector triads, and synchronized playback time.
    /// </summary>
    public class OrbitManager : MonoBehaviour
    {
        [Header("Visual Prefabs & References")]
        public GameObject centralBodyObject;
        public GameObject satellitePrefab;
        public Material orbitLineMaterial;
        public Material centralBodyMaterial;

        [Header("Playback Controls")]
        public bool isPlaying = true;
        [Range(0.01f, 100f)]
        public float playbackSpeed = 1.0f;
        public bool loop = true;

        [Header("Visual Settings")]
        public bool showOrbitPath = true;
        public float basisVectorLength = 15.0f;

        [Header("Tracked Orbits & Basis Frames")]
        public List<TrackedOrbit> trackedOrbits = new List<TrackedOrbit>();
        public int activeOrbitIndex = 0;
        public List<RuntimeBasisFrame> basisFrames = new List<RuntimeBasisFrame>();

        // Distinct default palette for N objects
        private static readonly Color[] DefaultOrbitColors = new Color[]
        {
            new Color(0.22f, 0.88f, 0.98f, 1f), // 0: Cyan
            new Color(1.00f, 0.45f, 0.22f, 1f), // 1: Coral / Orange
            new Color(0.25f, 0.95f, 0.52f, 1f), // 2: Emerald Green
            new Color(0.98f, 0.82f, 0.20f, 1f), // 3: Gold / Amber
            new Color(0.78f, 0.40f, 1.00f, 1f), // 4: Purple / Violet
            new Color(1.00f, 0.32f, 0.64f, 1f), // 5: Rose / Pink
            new Color(0.32f, 0.58f, 1.00f, 1f), // 6: Electric Blue
            new Color(0.68f, 0.98f, 0.22f, 1f), // 7: Lime
        };

        // Internal State
        private float _currentSimTime = 0f;
        private float _minStartTime = 0f;
        private float _maxEndTime = 0f;

        private GameObject _orbitsContainer;
        private GameObject _triadsContainer;

        public TrackedOrbit ActiveOrbit =>
            (trackedOrbits.Count > 0 && activeOrbitIndex >= 0 && activeOrbitIndex < trackedOrbits.Count)
                ? trackedOrbits[activeOrbitIndex]
                : null;

        public OrbitPayload CurrentPayload => ActiveOrbit?.payload;
        public float CurrentSimTime => _currentSimTime;
        public float TotalDuration => Mathf.Max(0f, _maxEndTime - _minStartTime);
        public float NormalizedProgress => TotalDuration > 0 ? Mathf.Clamp01((_currentSimTime - _minStartTime) / TotalDuration) : 0f;
        public Transform CentralBodyTransform => centralBodyObject != null ? centralBodyObject.transform : null;
        public Transform SatelliteTransform => ActiveOrbit?.satelliteInstance != null ? ActiveOrbit.satelliteInstance.transform : null;

        public bool HasLLAData
        {
            get
            {
                for (int i = 0; i < trackedOrbits.Count; i++)
                {
                    if (trackedOrbits[i].HasLLAData) return true;
                }
                return false;
            }
        }

        public Vector3 CurrentLLA => ActiveOrbit != null ? ActiveOrbit.currentLLA : Vector3.zero;

        private void OnEnable()
        {
            OrbitReceiver.OnOrbitReceived += HandleNewOrbit;
        }

        private void OnDisable()
        {
            OrbitReceiver.OnOrbitReceived -= HandleNewOrbit;
        }

        private void Start()
        {
            EnsureSceneComponents();
        }

        private void EnsureSceneComponents()
        {
            if (centralBodyObject == null)
            {
                centralBodyObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                centralBodyObject.name = "CentralBody";
                centralBodyObject.transform.position = Vector3.zero;

                if (centralBodyMaterial != null)
                {
                    centralBodyObject.GetComponent<Renderer>().material = centralBodyMaterial;
                }
            }

            if (centralBodyObject.transform.localScale.x > 50f || centralBodyObject.transform.localScale.x < 0.1f)
            {
                float defaultDiameter = (2574.7f / 1000f) * 2.0f;
                centralBodyObject.transform.localScale = Vector3.one * defaultDiameter;
            }

            if (_orbitsContainer == null)
            {
                _orbitsContainer = new GameObject("TrackedOrbits");
                _orbitsContainer.transform.SetParent(transform, false);
            }

            if (_triadsContainer == null)
            {
                _triadsContainer = new GameObject("BasisTriads");
                _triadsContainer.transform.SetParent(transform, false);
            }
        }

        public static Color GetDefaultOrbitColor(int index)
        {
            if (index >= 0 && index < DefaultOrbitColors.Length)
                return DefaultOrbitColors[index];

            // Golden-ratio hue stepping for arbitrary N > 8
            float hue = (index * 0.61803398875f) % 1.0f;
            return Color.HSVToRGB(hue, 0.78f, 1.0f);
        }

        /// <summary>
        /// Invoked when a new orbit trajectory payload arrives over TCP from Python.
        /// Supports adding N orbits across consecutive send_orbit() calls.
        /// </summary>
        public void HandleNewOrbit(OrbitPayload payload)
        {
            if (payload == null || payload.trajectory_eci == null || payload.trajectory_eci.Length == 0) return;

            EnsureSceneComponents();

            // Clear previous session's orbits if requested (true on the first send_orbit call of a script run)
            if (payload.clear_existing)
            {
                ClearAllOrbits();
            }

            float scale = payload.scale_km_per_unit > 0 ? payload.scale_km_per_unit : 1000f;

            // 1. Update Central Body Scale
            if (payload.central_body != null && payload.central_body.radius_km > 0)
            {
                float bodyRadiusUnits = payload.central_body.radius_km / scale;
                float bodyDiameterUnits = bodyRadiusUnits * 2.0f;
                centralBodyObject.transform.localScale = Vector3.one * bodyDiameterUnits;
                centralBodyObject.name = payload.central_body.name ?? "CentralBody";

                basisVectorLength = Mathf.Max(bodyDiameterUnits * 2.5f, 15.0f);
            }

            // 2. Determine Color & Name for this Orbit
            int orbitIdx = trackedOrbits.Count;
            Color orbitColor = GetDefaultOrbitColor(orbitIdx);
            if (!string.IsNullOrEmpty(payload.color_hex) && ColorUtility.TryParseHtmlString(payload.color_hex, out Color parsedCol))
            {
                orbitColor = parsedCol;
            }

            string orbitName = !string.IsNullOrEmpty(payload.mission_name) ? payload.mission_name : $"Orbit {orbitIdx + 1}";
            // Ensure unique display name if duplicate names are sent in the same batch
            int dupCount = 1;
            string baseName = orbitName;
            while (trackedOrbits.Exists(o => o.name == orbitName))
            {
                dupCount++;
                orbitName = $"{baseName} ({dupCount})";
            }

            int totalPts = payload.trajectory_eci.Length;
            TrackedOrbit orbit = new TrackedOrbit
            {
                name = orbitName,
                color = orbitColor,
                isVisible = true,
                payload = payload,
                times = payload.time,
                unityTrajectory = new Vector3[totalPts]
            };

            // 3. Convert ECI trajectory & velocity to Unity world space
            for (int i = 0; i < totalPts; i++)
            {
                orbit.unityTrajectory[i] = CoordinateConverter.EciToUnity(payload.trajectory_eci[i], scale);
            }

            if (payload.velocity_eci != null && payload.velocity_eci.Length == totalPts)
            {
                orbit.unityVelocities = new Vector3[totalPts];
                for (int i = 0; i < totalPts; i++)
                {
                    orbit.unityVelocities[i] = CoordinateConverter.DirectionEciToUnity(payload.velocity_eci[i]);
                }
            }

            // 4. Parse LLA (Latitude, Longitude, Altitude) if provided
            if (payload.trajectory_lla != null && payload.trajectory_lla.Length == totalPts)
            {
                orbit.llaData = new Vector3[totalPts];
                bool isRadians = true;
                for (int i = 0; i < totalPts; i++)
                {
                    if (payload.trajectory_lla[i] != null && payload.trajectory_lla[i].Length >= 2)
                    {
                        if (Mathf.Abs(payload.trajectory_lla[i][0]) > (Mathf.PI * 0.5f + 0.05f) ||
                            Mathf.Abs(payload.trajectory_lla[i][1]) > (Mathf.PI + 0.05f))
                        {
                            isRadians = false;
                            break;
                        }
                    }
                }

                float angleScale = isRadians ? Mathf.Rad2Deg : 1.0f;
                for (int i = 0; i < totalPts; i++)
                {
                    float[] pt = payload.trajectory_lla[i];
                    if (pt != null && pt.Length >= 3)
                    {
                        orbit.llaData[i] = new Vector3(pt[0] * angleScale, pt[1] * angleScale, pt[2]);
                    }
                }
            }

            // 5. Create Orbit Path LineRenderer
            GameObject lineObj = new GameObject($"OrbitPath_{orbitName}");
            lineObj.transform.SetParent(_orbitsContainer.transform, false);
            LineRenderer lr = lineObj.AddComponent<LineRenderer>();
            lr.useWorldSpace = true;
            lr.startWidth = 0.35f;
            lr.endWidth = 0.35f;

            if (orbitLineMaterial != null)
            {
                lr.material = new Material(orbitLineMaterial);
            }
            else
            {
                Shader shader = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Sprites/Default");
                lr.material = new Material(shader);
            }

            lr.positionCount = totalPts;
            lr.SetPositions(orbit.unityTrajectory);
            lr.enabled = showOrbitPath;
            orbit.pathLineRenderer = lr;

            // 6. Spawn Satellite Instance & Apply Color to SatGlow Base + Emission Maps
            GameObject satInstance;
            if (satellitePrefab != null)
            {
                satInstance = Instantiate(satellitePrefab, _orbitsContainer.transform);
            }
            else
            {
                satInstance = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                satInstance.transform.SetParent(_orbitsContainer.transform, false);
                satInstance.transform.localScale = Vector3.one * 0.8f;
            }
            satInstance.name = $"Satellite_{orbitName}";
            orbit.satelliteInstance = satInstance;

            ApplyColorToOrbitVisuals(orbit);

            // 7. Store Basis Frames for this Orbit & Register in Scene Basis List
            ExtractOrbitBasisFrames(orbit, payload);

            trackedOrbits.Add(orbit);
            if (activeOrbitIndex >= trackedOrbits.Count) activeOrbitIndex = 0;

            RecalculateTimeBounds();
            if (trackedOrbits.Count == 1)
            {
                _currentSimTime = _minStartTime;
            }

            UpdateSatelliteState();
        }

        /// <summary>
        /// Applies the orbit's assigned color directly to the SatGlow material's Base Map (_BaseColor),
        /// Emission Map (_EmissionColor), TrailRenderer gradient, and Orbit Path LineRenderer.
        /// </summary>
        private void ApplyColorToOrbitVisuals(TrackedOrbit orbit)
        {
            Color c = orbit.color;
            // Bright core tint for Base Map, and HDR intensity (1.6x) for Emission Map glow
            Color baseTint = Color.Lerp(c, Color.white, 0.25f);
            Color hdrEmission = new Color(c.r * 1.6f, c.g * 1.6f, c.b * 1.6f, 1.0f);

            if (orbit.satelliteInstance != null)
            {
                // 1. Satellite MeshRenderer(s) (SatGlow material)
                foreach (var mr in orbit.satelliteInstance.GetComponentsInChildren<MeshRenderer>())
                {
                    Material mat = mr.material;
                    if (mat.HasProperty("_BaseColor")) mat.SetColor("_BaseColor", baseTint);
                    if (mat.HasProperty("_Color")) mat.SetColor("_Color", baseTint);
                    mat.EnableKeyword("_EMISSION");
                    if (mat.HasProperty("_EmissionColor")) mat.SetColor("_EmissionColor", hdrEmission);
                }

                // 2. Satellite TrailRenderer(s) (SatGlow / Trail material + vertex color gradient)
                foreach (var tr in orbit.satelliteInstance.GetComponentsInChildren<TrailRenderer>())
                {
                    Material tMat = tr.material;
                    if (tMat.HasProperty("_BaseColor")) tMat.SetColor("_BaseColor", baseTint);
                    if (tMat.HasProperty("_Color")) tMat.SetColor("_Color", baseTint);
                    tMat.EnableKeyword("_EMISSION");
                    if (tMat.HasProperty("_EmissionColor")) tMat.SetColor("_EmissionColor", hdrEmission);

                    Gradient grad = new Gradient();
                    grad.SetKeys(
                        new GradientColorKey[]
                        {
                            new GradientColorKey(c, 0.0f),
                            new GradientColorKey(Color.Lerp(c, Color.white, 0.5f), 1.0f)
                        },
                        new GradientAlphaKey[]
                        {
                            new GradientAlphaKey(1.0f, 0.0f),
                            new GradientAlphaKey(0.0f, 1.0f)
                        }
                    );
                    tr.colorGradient = grad;
                }
            }

            // 3. Orbit Trajectory Path LineRenderer
            if (orbit.pathLineRenderer != null)
            {
                Color lineCol = new Color(c.r, c.g, c.b, 0.85f);
                Material lMat = orbit.pathLineRenderer.material;
                if (lMat.HasProperty("_BaseColor")) lMat.SetColor("_BaseColor", lineCol);
                if (lMat.HasProperty("_Color")) lMat.SetColor("_Color", lineCol);
                if (lMat.HasProperty("_EmissionColor"))
                {
                    lMat.EnableKeyword("_EMISSION");
                    lMat.SetColor("_EmissionColor", c * 1.15f);
                }
                orbit.pathLineRenderer.startColor = lineCol;
                orbit.pathLineRenderer.endColor = lineCol;
            }
        }

        private void ExtractOrbitBasisFrames(TrackedOrbit orbit, OrbitPayload payload)
        {
            List<BasisFrameData> incoming = new List<BasisFrameData>();

            if (payload.basis_frames != null && payload.basis_frames.Length > 0)
            {
                incoming.AddRange(payload.basis_frames);
            }
            else if (payload.basis_vectors != null)
            {
                if (payload.basis_vectors.tci != null)
                    incoming.Add(new BasisFrameData { name = "TCI", matrices = new float[][][] { payload.basis_vectors.tci } });
                if (payload.basis_vectors.pqw != null)
                    incoming.Add(new BasisFrameData { name = "PQW", matrices = new float[][][] { payload.basis_vectors.pqw } });
                if (payload.basis_vectors.rth != null)
                    incoming.Add(new BasisFrameData { name = "RTH", matrices = payload.basis_vectors.rth });
            }

            Color redAxis = new Color(1.00f, 0.18f, 0.18f, 1f);
            Color greenAxis = new Color(0.18f, 0.95f, 0.25f, 1f);
            Color blueAxis = new Color(0.20f, 0.55f, 1.00f, 1f);
            float[] lengthMultipliers = { 0.95f, 1.12f, 1.02f, 0.88f };

            foreach (var data in incoming)
            {
                if (data == null || string.IsNullOrEmpty(data.name) || data.matrices == null || data.matrices.Length == 0)
                    continue;

                orbit.basisMatrices[data.name] = data.matrices;

                // Ensure a RuntimeBasisFrame triad exists for this frame name
                RuntimeBasisFrame existing = basisFrames.Find(f => f.name == data.name);
                if (existing == null)
                {
                    int idx = basisFrames.Count;
                    float lenMult = lengthMultipliers[idx % lengthMultipliers.Length];

                    GameObject triadObj = new GameObject($"Triad_{data.name}");
                    triadObj.transform.SetParent(_triadsContainer.transform, false);
                    BasisVectorTriad triad = triadObj.AddComponent<BasisVectorTriad>();
                    triad.axisLength = basisVectorLength * lenMult;
                    triad.baseLineWidth = 0.26f;
                    triad.SetColors(redAxis, greenAxis, blueAxis);
                    triad.SetVisible(true);

                    basisFrames.Add(new RuntimeBasisFrame
                    {
                        name = data.name,
                        isVisible = true,
                        anchorToSatellite = false,
                        lengthMultiplier = lenMult,
                        triad = triad
                    });
                }
            }
        }

        public void ClearAllOrbits()
        {
            foreach (var orb in trackedOrbits)
            {
                if (orb.satelliteInstance != null) Destroy(orb.satelliteInstance);
                if (orb.pathLineRenderer != null) Destroy(orb.pathLineRenderer.gameObject);
            }
            trackedOrbits.Clear();
            activeOrbitIndex = 0;
        }

        private void RecalculateTimeBounds()
        {
            if (trackedOrbits.Count == 0)
            {
                _minStartTime = 0f;
                _maxEndTime = 0f;
                return;
            }

            _minStartTime = float.MaxValue;
            _maxEndTime = float.MinValue;

            foreach (var orb in trackedOrbits)
            {
                if (orb.times != null && orb.times.Length > 0)
                {
                    if (orb.times[0] < _minStartTime) _minStartTime = orb.times[0];
                    if (orb.times[orb.times.Length - 1] > _maxEndTime) _maxEndTime = orb.times[orb.times.Length - 1];
                }
            }

            if (_minStartTime > _maxEndTime)
            {
                _minStartTime = 0f;
                _maxEndTime = 0f;
            }
        }

        private void Update()
        {
            if (trackedOrbits.Count == 0 || !isPlaying) return;

            _currentSimTime += Time.deltaTime * playbackSpeed;

            if (_currentSimTime > _maxEndTime)
            {
                if (loop)
                {
                    _currentSimTime = _minStartTime;
                    foreach (var orb in trackedOrbits)
                    {
                        var trail = orb.satelliteInstance?.GetComponentInChildren<TrailRenderer>();
                        if (trail != null) trail.Clear();
                    }
                }
                else
                {
                    _currentSimTime = _maxEndTime;
                    isPlaying = false;
                }
            }

            UpdateSatelliteState();
        }

        /// <summary>
        /// Interpolates positions and LLA coordinates for all N satellites, and updates basis triads for the active satellite.
        /// </summary>
        public void UpdateSatelliteState()
        {
            if (trackedOrbits.Count == 0) return;

            // 1. Update every tracked satellite
            foreach (var orb in trackedOrbits)
            {
                if (orb.unityTrajectory == null || orb.unityTrajectory.Length < 2 || orb.satelliteInstance == null)
                    continue;

                orb.satelliteInstance.SetActive(orb.isVisible);
                if (orb.pathLineRenderer != null)
                {
                    orb.pathLineRenderer.enabled = showOrbitPath && orb.isVisible;
                }

                float startTime = orb.times[0];
                float endTime = orb.times[orb.times.Length - 1];
                float clampedTime = Mathf.Clamp(_currentSimTime, startTime, endTime);

                // Binary search for sample interval [idx, idx+1]
                int low = 0;
                int high = orb.unityTrajectory.Length - 2;
                int idx = 0;
                while (low <= high)
                {
                    int mid = (low + high) >> 1;
                    if (clampedTime < orb.times[mid])
                        high = mid - 1;
                    else if (clampedTime > orb.times[mid + 1])
                        low = mid + 1;
                    else
                    {
                        idx = mid;
                        break;
                    }
                }
                if (low > high) idx = Mathf.Clamp(low, 0, orb.unityTrajectory.Length - 2);

                float dt = Mathf.Max(orb.times[idx + 1] - orb.times[idx], 1e-6f);
                float alpha = Mathf.Clamp01((clampedTime - orb.times[idx]) / dt);

                orb.currentSampleIdx = idx;
                orb.currentSampleAlpha = alpha;

                Vector3 currentPos = Vector3.Lerp(orb.unityTrajectory[idx], orb.unityTrajectory[idx + 1], alpha);
                orb.satelliteInstance.transform.position = currentPos;

                if (orb.HasLLAData)
                {
                    Vector3 lla0 = orb.llaData[idx];
                    Vector3 lla1 = orb.llaData[Mathf.Min(idx + 1, orb.llaData.Length - 1)];
                    float lat = Mathf.Lerp(lla0.x, lla1.x, alpha);
                    float lon = Mathf.LerpAngle(lla0.y, lla1.y, alpha);
                    lon = Mathf.Repeat(lon + 180f, 360f) - 180f;
                    float alt = Mathf.Lerp(lla0.z, lla1.z, alpha);
                    orb.currentLLA = new Vector3(lat, lon, alt);
                }
            }

            // 2. Update Basis Triads (using ActiveOrbit's frame data and position)
            TrackedOrbit activeOrb = ActiveOrbit;
            if (activeOrb == null) return;

            Vector3 activeSatPos = activeOrb.satelliteInstance != null ? activeOrb.satelliteInstance.transform.position : Vector3.zero;
            int activeIdx = activeOrb.currentSampleIdx;
            float activeAlpha = activeOrb.currentSampleAlpha;

            foreach (var frame in basisFrames)
            {
                if (frame.triad == null) continue;

                // Look up matrices on ActiveOrbit first, or fallback to any orbit that defines this frame
                float[][][] matrices = null;
                TrackedOrbit sourceOrbit = activeOrb;
                if (!activeOrb.basisMatrices.TryGetValue(frame.name, out matrices))
                {
                    foreach (var orb in trackedOrbits)
                    {
                        if (orb.basisMatrices.TryGetValue(frame.name, out matrices))
                        {
                            sourceOrbit = orb;
                            break;
                        }
                    }
                }

                if (!frame.isVisible || matrices == null || matrices.Length == 0)
                {
                    frame.triad.SetVisible(false);
                    continue;
                }

                Vector3 origin = frame.anchorToSatellite ? activeSatPos : Vector3.zero;
                int matIdx = matrices.Length == 1 ? 0 : Mathf.Clamp(sourceOrbit.currentSampleIdx, 0, matrices.Length - 1);
                float[][] mat0 = matrices[matIdx];

                if (mat0 != null && mat0.Length >= 3)
                {
                    Vector3 v0 = CoordinateConverter.DirectionEciToUnity(mat0[0]);
                    Vector3 v1 = CoordinateConverter.DirectionEciToUnity(mat0[1]);
                    Vector3 v2 = CoordinateConverter.DirectionEciToUnity(mat0[2]);

                    if (matrices.Length > 1 && matIdx + 1 < matrices.Length)
                    {
                        float[][] mat1 = matrices[matIdx + 1];
                        if (mat1 != null && mat1.Length >= 3)
                        {
                            v0 = Vector3.Slerp(v0, CoordinateConverter.DirectionEciToUnity(mat1[0]), sourceOrbit.currentSampleAlpha).normalized;
                            v1 = Vector3.Slerp(v1, CoordinateConverter.DirectionEciToUnity(mat1[1]), sourceOrbit.currentSampleAlpha).normalized;
                            v2 = Vector3.Slerp(v2, CoordinateConverter.DirectionEciToUnity(mat1[2]), sourceOrbit.currentSampleAlpha).normalized;
                        }
                    }

                    frame.triad.SetVectors(origin, v0, v1, v2, basisVectorLength * frame.lengthMultiplier);
                    frame.triad.SetVisible(true);
                }
            }
        }

        // --- Public UI Controller Methods ---

        public void TogglePlay() => isPlaying = !isPlaying;
        public void Play() => isPlaying = true;
        public void Pause() => isPlaying = false;

        public void ScrubTo(float normalizedProgress)
        {
            if (trackedOrbits.Count == 0) return;
            _currentSimTime = _minStartTime + Mathf.Clamp01(normalizedProgress) * TotalDuration;
            UpdateSatelliteState();
        }

        public void SetPlaybackSpeed(float speed) => playbackSpeed = Mathf.Max(speed, 0.01f);

        public void SelectOrbit(int index)
        {
            if (trackedOrbits.Count == 0) return;
            activeOrbitIndex = Mathf.Clamp(index, 0, trackedOrbits.Count - 1);
            UpdateSatelliteState();
        }

        public void SetOrbitVisibility(TrackedOrbit orbit, bool isVisible)
        {
            if (orbit == null) return;
            orbit.isVisible = isVisible;
            UpdateSatelliteState();
        }

        public void SetBasisVisibility(RuntimeBasisFrame frame, bool isVisible)
        {
            if (frame == null) return;
            frame.isVisible = isVisible;
            UpdateSatelliteState();
        }

        public void SetBasisAnchor(RuntimeBasisFrame frame, bool atSatellite)
        {
            if (frame == null) return;
            frame.anchorToSatellite = atSatellite;
            UpdateSatelliteState();
        }

        public void ToggleOrbitPath(bool isVisible)
        {
            showOrbitPath = isVisible;
            UpdateSatelliteState();
        }
    }
}
