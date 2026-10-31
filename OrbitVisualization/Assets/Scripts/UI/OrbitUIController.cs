using UnityEngine;
using UnityEngine.InputSystem;
using OrbitVisualization.Core;
using OrbitVisualization.Visuals;
using OrbitVisualization.CameraControl;

namespace OrbitVisualization.UI
{
    [RequireComponent(typeof(OrbitManager))]
    public class OrbitUIController : MonoBehaviour
    {
        [Header("HUD Visibility")]
        public bool showHUD = true;

        [Header("Ground Track Settings")]
        [Tooltip("Fraction of total trajectory points before the ground track tail fades completely to 0.")]
        [Range(0.05f, 1.0f)]
        public float groundTrackTrailFraction = 0.18f;

        private OrbitManager _manager;
        private BlenderCamera _cameraController;

        // Custom Styles
        private GUIStyle _widgetStyle;
        private GUIStyle _headerStyle;
        private GUIStyle _labelStyle;
        private GUIStyle _smallLabelStyle;
        private GUIStyle _valueStyle;
        private GUIStyle _buttonStyle;
        private GUIStyle _toggleStyle;
        private GUIStyle _sliderBgStyle;
        private GUIStyle _sliderThumbStyle;

        private Font _regularFont;
        private Font _boldFont;
        private Font _semiBoldFont;
        private bool _stylesInitialized = false;

        // Widget toggles
        private bool _showPlaybackWidget = true;
        private bool _showVisualsWidget = true;
        private bool _showGroundTrackWidget = true;

        // Ground Track Plot Texture & Buffers (2:1 aspect ratio for [-180..180] x [-90..90])
        private const int PlotWidth = 320;
        private const int PlotHeight = 160;
        private Texture2D _groundTrackTex;
        private Color32[] _groundTrackBgPixels;
        private Color32[] _groundTrackWorkPixels;
        private float _lastPlottedSimTime = -1f;
        private int _lastPlottedOrbitCount = -1;

        private void Awake()
        {
            _manager = GetComponent<OrbitManager>();
            _cameraController = FindAnyObjectByType<BlenderCamera>();

            // Load Lora fonts from Resources/static/
            _regularFont = Resources.Load<Font>("static/Lora-Regular");
            _boldFont = Resources.Load<Font>("static/Lora-Bold");
            _semiBoldFont = Resources.Load<Font>("static/Lora-SemiBold");

            // Fallbacks
            if (_boldFont == null) _boldFont = _regularFont;
            if (_semiBoldFont == null) _semiBoldFont = _regularFont;

            InitializeGroundTrackTexture();
        }

        private void Update()
        {
            if (Keyboard.current != null)
            {
                if (Keyboard.current.tabKey.wasPressedThisFrame) showHUD = !showHUD;
                if (Keyboard.current.spaceKey.wasPressedThisFrame) _manager.TogglePlay();
                if (Keyboard.current.rKey.wasPressedThisFrame) _manager.ScrubTo(0f);
            }
        }

        private void InitializeStyles()
        {
            if (_stylesInitialized) return;

            // Widget Background (Smaller, darker, fainter/more transparent)
            _widgetStyle = new GUIStyle(GUI.skin.box);
            Texture2D bgTex = CreateRoundedTexture(64, 64, 10, new Color(0.04f, 0.05f, 0.06f, 0.45f));
            _widgetStyle.normal.background = bgTex;
            _widgetStyle.border = new RectOffset(10, 10, 10, 10);
            _widgetStyle.padding = new RectOffset(12, 12, 10, 10);
            _widgetStyle.margin = new RectOffset(5, 5, 8, 8);

            // Header Style
            _headerStyle = new GUIStyle(GUI.skin.label)
            {
                fontSize = 13,
                fontStyle = _boldFont != null ? FontStyle.Normal : FontStyle.Bold,
                alignment = TextAnchor.MiddleLeft,
                font = _boldFont
            };
            _headerStyle.normal.textColor = new Color(0.9f, 0.9f, 0.9f);

            // Standard Label Style
            _labelStyle = new GUIStyle(GUI.skin.label)
            {
                fontSize = 11,
                alignment = TextAnchor.MiddleLeft,
                font = _regularFont,
                richText = true
            };
            _labelStyle.normal.textColor = new Color(0.8f, 0.8f, 0.8f);

            // Small Axis / Hint Label Style
            _smallLabelStyle = new GUIStyle(GUI.skin.label)
            {
                fontSize = 9,
                alignment = TextAnchor.MiddleCenter,
                font = _regularFont
            };
            _smallLabelStyle.normal.textColor = new Color(0.6f, 0.64f, 0.68f);

            // Value Label Style (For numbers/stats)
            _valueStyle = new GUIStyle(GUI.skin.label)
            {
                fontSize = 12,
                fontStyle = _semiBoldFont != null ? FontStyle.Normal : FontStyle.Bold,
                alignment = TextAnchor.MiddleRight,
                font = _semiBoldFont,
                richText = true
            };
            _valueStyle.normal.textColor = new Color(0.35f, 0.85f, 0.98f);

            // Button Style
            _buttonStyle = new GUIStyle(GUI.skin.button);
            _buttonStyle.normal.background = CreateRoundedTexture(32, 32, 6, new Color(0.15f, 0.2f, 0.25f, 0.6f));
            _buttonStyle.hover.background = CreateRoundedTexture(32, 32, 6, new Color(0.25f, 0.3f, 0.35f, 0.7f));
            _buttonStyle.active.background = CreateRoundedTexture(32, 32, 6, new Color(0.1f, 0.12f, 0.15f, 0.8f));
            _buttonStyle.border = new RectOffset(6, 6, 6, 6);
            _buttonStyle.normal.textColor = Color.white;
            _buttonStyle.hover.textColor = Color.white;
            _buttonStyle.active.textColor = Color.gray;
            _buttonStyle.fontSize = 11;
            _buttonStyle.font = _semiBoldFont ?? _regularFont;
            _buttonStyle.alignment = TextAnchor.MiddleCenter;
            _buttonStyle.richText = true;

            // Toggle Style
            _toggleStyle = new GUIStyle(GUI.skin.toggle);
            _toggleStyle.font = _regularFont;
            _toggleStyle.fontSize = 11;
            _toggleStyle.normal.textColor = new Color(0.85f, 0.85f, 0.85f);
            _toggleStyle.onNormal.textColor = Color.white;
            _toggleStyle.richText = true;

            // Sliders
            _sliderBgStyle = new GUIStyle(GUI.skin.horizontalSlider);
            _sliderBgStyle.normal.background = CreateSolidTexture(2, 2, new Color(0.1f, 0.1f, 0.1f, 0.6f));

            _sliderThumbStyle = new GUIStyle(GUI.skin.horizontalSliderThumb);
            _sliderThumbStyle.normal.background = CreateRoundedTexture(12, 12, 6, new Color(0.35f, 0.85f, 0.98f, 0.9f));
            _sliderThumbStyle.hover.background = CreateRoundedTexture(12, 12, 6, new Color(0.5f, 0.95f, 1.0f, 1f));
            _sliderThumbStyle.fixedWidth = 12;
            _sliderThumbStyle.fixedHeight = 12;

            _stylesInitialized = true;
        }

        private Texture2D CreateRoundedTexture(int width, int height, int radius, Color color)
        {
            Texture2D tex = new Texture2D(width, height, TextureFormat.ARGB32, false);
            Color[] pixels = new Color[width * height];
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    bool isInside = true;
                    if (x < radius && y < radius)
                        isInside = (x - radius) * (x - radius) + (y - radius) * (y - radius) <= radius * radius;
                    else if (x > width - radius - 1 && y < radius)
                        isInside = (x - (width - radius - 1)) * (x - (width - radius - 1)) + (y - radius) * (y - radius) <= radius * radius;
                    else if (x < radius && y > height - radius - 1)
                        isInside = (x - radius) * (x - radius) + (y - (height - radius - 1)) * (y - (height - radius - 1)) <= radius * radius;
                    else if (x > width - radius - 1 && y > height - radius - 1)
                        isInside = (x - (width - radius - 1)) * (x - (width - radius - 1)) + (y - (height - radius - 1)) * (y - (height - radius - 1)) <= radius * radius;

                    pixels[y * width + x] = isInside ? color : Color.clear;
                }
            }
            tex.SetPixels(pixels);
            tex.Apply();
            return tex;
        }

        private Texture2D CreateSolidTexture(int width, int height, Color color)
        {
            Texture2D tex = new Texture2D(width, height);
            Color[] pixels = new Color[width * height];
            for (int i = 0; i < pixels.Length; i++) pixels[i] = color;
            tex.SetPixels(pixels);
            tex.Apply();
            return tex;
        }

        private void OnGUI()
        {
            if (!showHUD) return;
            InitializeStyles();

            // Left Column: Top Bar, Playback Controls, Visualization Settings
            GUILayout.BeginArea(new Rect(15, 15, 250, Screen.height - 30));
            DrawTopBar();
            if (_showPlaybackWidget) DrawPlaybackWidget();
            if (_showVisualsWidget) DrawVisualsWidget();
            GUILayout.EndArea();

            // Right Side: Ground Track (Lat / Lon) Widget
            if (_showGroundTrackWidget && _manager.HasLLAData)
            {
                float rightWidth = 376f;
                GUILayout.BeginArea(new Rect(Screen.width - rightWidth - 15f, 15f, rightWidth, 275f));
                DrawGroundTrackWidget();
                GUILayout.EndArea();
            }
        }

        private void DrawTopBar()
        {
            GUILayout.BeginVertical(_widgetStyle);

            GUILayout.BeginHorizontal();
            string mission = "AWAITING TELEMETRY";
            if (_manager.trackedOrbits.Count == 1)
            {
                mission = _manager.trackedOrbits[0].name;
            }
            else if (_manager.trackedOrbits.Count > 1)
            {
                mission = $"{_manager.trackedOrbits.Count} ORBITS ACTIVE";
            }
            GUILayout.Label(mission.ToUpper(), _headerStyle);
            GUILayout.EndHorizontal();

            GUILayout.Space(8);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button(_showPlaybackWidget ? "Hide Playback" : "Show Playback", _buttonStyle, GUILayout.Height(20)))
                _showPlaybackWidget = !_showPlaybackWidget;
            if (GUILayout.Button(_showVisualsWidget ? "Hide Visuals" : "Show Visuals", _buttonStyle, GUILayout.Height(20)))
                _showVisualsWidget = !_showVisualsWidget;
            GUILayout.EndHorizontal();

            if (_manager.HasLLAData)
            {
                GUILayout.Space(4);
                if (GUILayout.Button(_showGroundTrackWidget ? "Hide Ground Track" : "Show Ground Track", _buttonStyle, GUILayout.Height(20)))
                    _showGroundTrackWidget = !_showGroundTrackWidget;
            }

            GUILayout.EndVertical();
        }

        private void DrawPlaybackWidget()
        {
            GUILayout.BeginVertical(_widgetStyle);

            GUILayout.Label("Playback Controls", _headerStyle);
            GUILayout.Space(6);

            // Time Readouts
            GUILayout.BeginHorizontal();
            GUILayout.Label("Time", _labelStyle);
            GUILayout.Label($"{_manager.CurrentSimTime:F1}s / {_manager.TotalDuration:F1}s", _valueStyle);
            GUILayout.EndHorizontal();

            GUILayout.Space(6);

            // Play / Pause & Reset
            GUILayout.BeginHorizontal();
            string playBtnText = _manager.isPlaying ? "Pause" : "Play";
            if (GUILayout.Button(playBtnText, _buttonStyle, GUILayout.Height(22))) _manager.TogglePlay();
            if (GUILayout.Button("Restart", _buttonStyle, GUILayout.Height(22))) _manager.ScrubTo(0f);
            GUILayout.EndHorizontal();

            GUILayout.Space(8);

            // Time Scrub Slider
            GUILayout.Label("Scrub Timeline", _labelStyle);
            float currentProgress = _manager.NormalizedProgress;
            float newProgress = GUILayout.HorizontalSlider(currentProgress, 0f, 1f, _sliderBgStyle, _sliderThumbStyle);
            if (Mathf.Abs(newProgress - currentProgress) > 0.001f)
            {
                _manager.ScrubTo(newProgress);
            }

            GUILayout.Space(8);

            // Playback Speed
            GUILayout.BeginHorizontal();
            GUILayout.Label("Speed", _labelStyle);

            string speedLabel = _manager.playbackSpeed >= 1000f ? $"{_manager.playbackSpeed / 1000f:F1}k x" : $"{_manager.playbackSpeed:F1}x";
            GUILayout.Label(speedLabel, _valueStyle);
            GUILayout.EndHorizontal();

            // Logarithmic slider (0.1x to 100,000x)
            float logSpeed = Mathf.Log10(Mathf.Max(0.1f, _manager.playbackSpeed));
            logSpeed = GUILayout.HorizontalSlider(logSpeed, -1f, 5f, _sliderBgStyle, _sliderThumbStyle);
            _manager.playbackSpeed = Mathf.Pow(10f, logSpeed);

            GUILayout.EndVertical();
        }

        private void DrawVisualsWidget()
        {
            GUILayout.BeginVertical(_widgetStyle);

            GUILayout.Label("Visualization Settings", _headerStyle);
            GUILayout.Space(6);

            // Camera Focus
            if (_cameraController != null)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label("Focus", _labelStyle);

                string targetName = "Central Body";
                if (_cameraController.focusTarget == BlenderCamera.CameraFocusTarget.Satellite)
                {
                    var activeOrb = _manager.ActiveOrbit;
                    if (activeOrb != null)
                    {
                        string hex = ColorUtility.ToHtmlStringRGB(activeOrb.color);
                        targetName = $"<color=#{hex}>{activeOrb.name}</color>";
                    }
                    else
                    {
                        targetName = "Satellite";
                    }
                }

                if (GUILayout.Button(targetName, _buttonStyle, GUILayout.Height(20)))
                {
                    _cameraController.ToggleFocusTarget();
                }
                GUILayout.EndHorizontal();
            }

            // Multi-Satellite List (shown when more than 1 orbit is loaded)
            if (_manager.trackedOrbits.Count > 1)
            {
                GUILayout.Space(6);
                for (int i = 0; i < _manager.trackedOrbits.Count; i++)
                {
                    var orb = _manager.trackedOrbits[i];
                    string hex = ColorUtility.ToHtmlStringRGB(orb.color);
                    bool isSelected = (i == _manager.activeOrbitIndex);

                    GUILayout.BeginHorizontal();
                    bool newVis = GUILayout.Toggle(orb.isVisible, $" <color=#{hex}>● {orb.name}</color>", _toggleStyle);
                    if (newVis != orb.isVisible)
                    {
                        _manager.SetOrbitVisibility(orb, newVis);
                        _lastPlottedSimTime = -1f; // Force Ground Track refresh
                    }

                    GUILayout.FlexibleSpace();
                    string selLabel = isSelected ? "Active" : "Select";
                    if (GUILayout.Button(selLabel, _buttonStyle, GUILayout.Width(50), GUILayout.Height(18)))
                    {
                        _manager.SelectOrbit(i);
                        if (_cameraController != null && _cameraController.focusTarget == BlenderCamera.CameraFocusTarget.Satellite)
                        {
                            _cameraController.SetFocusTarget(BlenderCamera.CameraFocusTarget.Satellite);
                        }
                    }
                    GUILayout.EndHorizontal();
                    GUILayout.Space(2);
                }
            }

            GUILayout.Space(8);

            // Dynamic Basis Vector Toggles from Python all_basis dict
            if (_manager.basisFrames != null && _manager.basisFrames.Count > 0)
            {
                foreach (var frame in _manager.basisFrames)
                {
                    GUILayout.BeginHorizontal();
                    bool newVis = GUILayout.Toggle(frame.isVisible, $" Show {frame.name} Frame", _toggleStyle);
                    if (newVis != frame.isVisible)
                    {
                        _manager.SetBasisVisibility(frame, newVis);
                    }

                    if (frame.isVisible)
                    {
                        GUILayout.FlexibleSpace();
                        string anchorText = frame.anchorToSatellite ? "At Sat" : "At Body";
                        if (GUILayout.Button(anchorText, _buttonStyle, GUILayout.Width(54), GUILayout.Height(18)))
                        {
                            _manager.SetBasisAnchor(frame, !frame.anchorToSatellite);
                        }
                    }
                    GUILayout.EndHorizontal();
                    GUILayout.Space(2);
                }
            }

            GUILayout.Space(4);

            bool newShowPath = GUILayout.Toggle(_manager.showOrbitPath, " Show Orbit Paths", _toggleStyle);
            if (newShowPath != _manager.showOrbitPath) _manager.ToggleOrbitPath(newShowPath);

            GUILayout.Space(8);
            GUILayout.Label("Press [Tab] to hide UI", _smallLabelStyle);

            GUILayout.EndVertical();
        }

        // =====================================================================
        // Ground Track (Latitude / Longitude) 2D Plot Widget
        // =====================================================================

        private void InitializeGroundTrackTexture()
        {
            _groundTrackTex = new Texture2D(PlotWidth, PlotHeight, TextureFormat.RGBA32, false)
            {
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp
            };

            _groundTrackBgPixels = new Color32[PlotWidth * PlotHeight];
            _groundTrackWorkPixels = new Color32[PlotWidth * PlotHeight];

            Color32 bgCol = new Color32(8, 10, 14, 140);          // Faint dark background
            Color32 minorGridCol = new Color32(255, 255, 255, 18); // Very faint 30/60 deg grid lines
            Color32 majorGridCol = new Color32(255, 255, 255, 42); // Slightly clearer Equator & Prime Meridian
            Color32 borderCol = new Color32(255, 255, 255, 55);    // Outer plot frame

            for (int i = 0; i < _groundTrackBgPixels.Length; i++)
            {
                _groundTrackBgPixels[i] = bgCol;
            }

            int[] lonGridDegs = { -120, -60, 0, 60, 120 };
            foreach (int lon in lonGridDegs)
            {
                int px = LonToPixelX(lon);
                Color32 col = (lon == 0) ? majorGridCol : minorGridCol;
                for (int y = 0; y < PlotHeight; y++)
                {
                    _groundTrackBgPixels[y * PlotWidth + px] = col;
                }
            }

            int[] latGridDegs = { -60, -30, 0, 30, 60 };
            foreach (int lat in latGridDegs)
            {
                int py = LatToPixelY(lat);
                Color32 col = (lat == 0) ? majorGridCol : minorGridCol;
                for (int x = 0; x < PlotWidth; x++)
                {
                    _groundTrackBgPixels[py * PlotWidth + x] = col;
                }
            }

            for (int x = 0; x < PlotWidth; x++)
            {
                _groundTrackBgPixels[x] = borderCol;
                _groundTrackBgPixels[(PlotHeight - 1) * PlotWidth + x] = borderCol;
            }
            for (int y = 0; y < PlotHeight; y++)
            {
                _groundTrackBgPixels[y * PlotWidth] = borderCol;
                _groundTrackBgPixels[y * PlotWidth + (PlotWidth - 1)] = borderCol;
            }
        }

        private void DrawGroundTrackWidget()
        {
            if (Mathf.Abs(_manager.CurrentSimTime - _lastPlottedSimTime) > 1e-4f || _manager.trackedOrbits.Count != _lastPlottedOrbitCount)
            {
                UpdateGroundTrackTexture();
                _lastPlottedSimTime = _manager.CurrentSimTime;
                _lastPlottedOrbitCount = _manager.trackedOrbits.Count;
            }

            GUILayout.BeginVertical(_widgetStyle);

            // Title & Live Readout Row (uses ActiveOrbit's color and coordinates)
            GUILayout.BeginHorizontal();
            GUILayout.Label("Ground Track", _headerStyle);
            var activeOrb = _manager.ActiveOrbit;
            if (activeOrb != null && activeOrb.HasLLAData)
            {
                Vector3 curLLA = activeOrb.currentLLA;
                string hex = ColorUtility.ToHtmlStringRGB(activeOrb.color);
                GUILayout.Label($"<color=#{hex}>Lat {curLLA.x:+0.0;-0.0;0.0}°   Lon {curLLA.y:+0.0;-0.0;0.0}°</color>", _valueStyle);
            }
            GUILayout.EndHorizontal();

            GUILayout.Space(6);

            // Plot Area with Y-Axis Labels on the Left
            GUILayout.BeginHorizontal();

            GUILayout.BeginVertical(GUILayout.Width(24), GUILayout.Height(PlotHeight));
            GUIStyle rightSmall = new GUIStyle(_smallLabelStyle) { alignment = TextAnchor.UpperRight };
            GUILayout.Label("+90°", rightSmall);
            GUILayout.FlexibleSpace();
            rightSmall.alignment = TextAnchor.MiddleRight;
            GUILayout.Label("0°", rightSmall);
            GUILayout.FlexibleSpace();
            rightSmall.alignment = TextAnchor.LowerRight;
            GUILayout.Label("-90°", rightSmall);
            GUILayout.EndVertical();

            GUILayout.Space(4);

            Rect plotRect = GUILayoutUtility.GetRect(PlotWidth, PlotHeight, GUILayout.ExpandWidth(false), GUILayout.ExpandHeight(false));
            GUI.DrawTexture(plotRect, _groundTrackTex);

            GUILayout.EndHorizontal();

            // Bottom X-axis labels (-180, 0, +180)
            GUILayout.BeginHorizontal();
            GUILayout.Space(28);
            GUIStyle leftSmall = new GUIStyle(_smallLabelStyle) { alignment = TextAnchor.UpperLeft };
            GUIStyle centerSmall = new GUIStyle(_smallLabelStyle) { alignment = TextAnchor.UpperCenter };
            GUIStyle rightBottomSmall = new GUIStyle(_smallLabelStyle) { alignment = TextAnchor.UpperRight };

            GUILayout.Label("-180°", leftSmall, GUILayout.Width(50));
            GUILayout.FlexibleSpace();
            GUILayout.Label("0° Lon", centerSmall, GUILayout.Width(60));
            GUILayout.FlexibleSpace();
            GUILayout.Label("+180°", rightBottomSmall, GUILayout.Width(50));
            GUILayout.EndHorizontal();

            GUILayout.EndVertical();
        }

        private void UpdateGroundTrackTexture()
        {
            if (_groundTrackTex == null) InitializeGroundTrackTexture();

            // Reset working buffer to cached faint grid background
            System.Array.Copy(_groundTrackBgPixels, _groundTrackWorkPixels, _groundTrackBgPixels.Length);

            // Render every visible orbit's ground track in its own color
            foreach (var orb in _manager.trackedOrbits)
            {
                if (!orb.isVisible || !orb.HasLLAData) continue;

                Vector3[] lla = orb.llaData;
                int currentIdx = Mathf.Clamp(orb.currentSampleIdx, 0, lla.Length - 1);

                int trailWindow = Mathf.Max(15, Mathf.RoundToInt(lla.Length * Mathf.Clamp(groundTrackTrailFraction, 0.05f, 1.0f)));
                int startIdx = Mathf.Max(0, currentIdx - trailWindow);

                Color32 baseCol32 = orb.color;

                // 1. Draw fading trail segments from startIdx up to currentIdx
                for (int i = startIdx; i < currentIdx; i++)
                {
                    float progressInTail = (float)(i - startIdx + 1) / (trailWindow + 1);
                    float fadeAlpha = progressInTail * progressInTail;
                    byte alphaByte = (byte)Mathf.Clamp(Mathf.RoundToInt(fadeAlpha * 255f), 0, 255);
                    if (alphaByte <= 2) continue;

                    Color32 segCol = new Color32(baseCol32.r, baseCol32.g, baseCol32.b, alphaByte);
                    DrawGroundTrackSegment(lla[i].y, lla[i].x, lla[i + 1].y, lla[i + 1].x, segCol);
                }

                // 2. Draw partial interpolated segment from lla[currentIdx] to currentLLA at full opacity
                Vector3 curLLA = orb.currentLLA;
                DrawGroundTrackSegment(lla[currentIdx].y, lla[currentIdx].x, curLLA.y, curLLA.x, new Color32(baseCol32.r, baseCol32.g, baseCol32.b, 255));

                // 3. Draw Current Satellite Marker (colored halo + bright white core)
                int curPx = LonToPixelX(curLLA.y);
                int curPy = LatToPixelY(curLLA.x);
                DrawFilledCircle(curPx, curPy, 4, new Color32(baseCol32.r, baseCol32.g, baseCol32.b, 225));
                DrawFilledCircle(curPx, curPy, 2, new Color32(255, 255, 255, 255));
            }

            _groundTrackTex.SetPixels32(_groundTrackWorkPixels);
            _groundTrackTex.Apply(false);
        }

        private void DrawGroundTrackSegment(float lon1, float lat1, float lon2, float lat2, Color32 color)
        {
            float dLon = lon2 - lon1;

            if (Mathf.Abs(dLon) <= 180f)
            {
                DrawLinePixels(
                    LonToPixelX(lon1), LatToPixelY(lat1),
                    LonToPixelX(lon2), LatToPixelY(lat2),
                    color
                );
                return;
            }

            if (dLon > 180f)
            {
                float wrappedDLon = (lon2 - 360f) - lon1;
                float tCross = Mathf.Abs(wrappedDLon) > 1e-5f ? (-180f - lon1) / wrappedDLon : 0.5f;
                float latCross = Mathf.Lerp(lat1, lat2, Mathf.Clamp01(tCross));

                DrawLinePixels(LonToPixelX(lon1), LatToPixelY(lat1), LonToPixelX(-180f), LatToPixelY(latCross), color);
                DrawLinePixels(LonToPixelX(180f), LatToPixelY(latCross), LonToPixelX(lon2), LatToPixelY(lat2), color);
            }
            else
            {
                float wrappedDLon = (lon2 + 360f) - lon1;
                float tCross = Mathf.Abs(wrappedDLon) > 1e-5f ? (180f - lon1) / wrappedDLon : 0.5f;
                float latCross = Mathf.Lerp(lat1, lat2, Mathf.Clamp01(tCross));

                DrawLinePixels(LonToPixelX(lon1), LatToPixelY(lat1), LonToPixelX(180f), LatToPixelY(latCross), color);
                DrawLinePixels(LonToPixelX(-180f), LatToPixelY(latCross), LonToPixelX(lon2), LatToPixelY(lat2), color);
            }
        }

        private int LonToPixelX(float lonDeg)
        {
            float norm = Mathf.Clamp01((lonDeg + 180f) / 360f);
            return Mathf.Clamp(Mathf.RoundToInt(norm * (PlotWidth - 1)), 0, PlotWidth - 1);
        }

        private int LatToPixelY(float latDeg)
        {
            float norm = Mathf.Clamp01((latDeg + 90f) / 180f);
            return Mathf.Clamp(Mathf.RoundToInt(norm * (PlotHeight - 1)), 0, PlotHeight - 1);
        }

        private void DrawLinePixels(int x0, int y0, int x1, int y1, Color32 color)
        {
            int dx = Mathf.Abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
            int dy = -Mathf.Abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
            int err = dx + dy;

            while (true)
            {
                SetThickPixel(x0, y0, color);
                if (x0 == x1 && y0 == y1) break;
                int e2 = 2 * err;
                if (e2 >= dy) { err += dy; x0 += sx; }
                if (e2 <= dx) { err += dx; y0 += sy; }
            }
        }

        private void SetThickPixel(int x, int y, Color32 fg)
        {
            BlendPixel(x, y, fg);
            BlendPixel(x, y + 1, fg);
        }

        private void BlendPixel(int x, int y, Color32 fg)
        {
            if (x < 0 || x >= PlotWidth || y < 0 || y >= PlotHeight) return;

            int idx = y * PlotWidth + x;
            Color32 dst = _groundTrackWorkPixels[idx];
            float a = fg.a / 255f;

            byte r = (byte)Mathf.Clamp(Mathf.RoundToInt(fg.r * a + dst.r * (1f - a)), 0, 255);
            byte g = (byte)Mathf.Clamp(Mathf.RoundToInt(fg.g * a + dst.g * (1f - a)), 0, 255);
            byte b = (byte)Mathf.Clamp(Mathf.RoundToInt(fg.b * a + dst.b * (1f - a)), 0, 255);
            byte outA = (byte)Mathf.Max(dst.a, fg.a);

            _groundTrackWorkPixels[idx] = new Color32(r, g, b, outA);
        }

        private void DrawFilledCircle(int cx, int cy, int radius, Color32 color)
        {
            int r2 = radius * radius;
            for (int dy = -radius; dy <= radius; dy++)
            {
                for (int dx = -radius; dx <= radius; dx++)
                {
                    if (dx * dx + dy * dy <= r2)
                    {
                        BlendPixel(cx + dx, cy + dy, color);
                    }
                }
            }
        }
    }
}
