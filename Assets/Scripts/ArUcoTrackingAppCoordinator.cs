using System;
using System.Collections;
using System.Collections.Generic;
using Meta.XR;
using UnityEngine;

namespace QuestArucoTracker
{
    public class ArUcoTrackingAppCoordinator : MonoBehaviour
    {
        /// <summary>
        /// Serializable class for mapping marker IDs to GameObjects in the Inspector.
        /// </summary>
        [Serializable]
        public class MarkerGameObjectPair
        {
            /// <summary>
            /// The unique ID of the AR marker to track.
            /// </summary>
            public int markerId;

            /// <summary>
            /// The GameObject to associate with this marker.
            /// </summary>
            public GameObject gameObject;
        }

        private class MarkerEntry
        {
            public GameObject Root;
            public bool Visible;
        }

        [Header("Passthrough Camera")]
        [SerializeField]
        private PassthroughCameraAccess m_passthroughCameraAccess;

        [Header("Marker Tracking")]
        [SerializeField] private ArUcoMarkerTracking m_arucoMarkerTracking;
        [SerializeField, Tooltip("Optional: marker IDs mapped to your own GameObjects. Any other ID in the dictionary gets an automatically created object the first time it is seen.")]
        private List<MarkerGameObjectPair> m_markerGameObjectPairs = new List<MarkerGameObjectPair>();
        [SerializeField] MeshRenderer m_debugRenderer;

        [Header("Marker Visuals")]
        [SerializeField, Tooltip("Draw an outline and ID label on each tracked marker.")]
        private bool m_showMarkerOutlines = true;
        [SerializeField, Tooltip("Hide the renderers of the GameObjects assigned above (only the outline is shown).")]
        private bool m_hideAssignedObjectRenderers = true;
        [SerializeField, Tooltip("Color of the marker square and ID number.")]
        private Color m_outlineColor = new Color(1f, 0.1f, 0.85f, 1f);
        [SerializeField, Range(0f, 60f), Tooltip("Optional extra glide toward the tracked pose (1/s). Keep 0: the tracker already smooths and predicts poses every frame, and any glide adds lag.")]
        private float m_followSpeed = 0f;

        private readonly Dictionary<int, MarkerEntry> m_markers = new Dictionary<int, MarkerEntry>();
        private readonly List<int> m_trackedIds = new List<int>();
        private Material m_markerOutlineMaterial;
        private Material m_labelBackingMaterial;
        private Font m_labelFont;
        private bool m_showRecogResult = false;

        private static readonly Color LabelBackingColor = new Color(0f, 0f, 0f, 1f);

        /// <summary>
        /// Waits for the passthrough camera, then initializes marker tracking.
        /// </summary>
        private IEnumerator Start()
        {
            if (m_passthroughCameraAccess == null)
            {
                Debug.LogError("PassthroughCameraAccess reference is missing.");
                yield break;
            }

            while (!m_passthroughCameraAccess.IsPlaying)
                yield return null;
            yield return null; // Wait one frame to ensure camera is fully initialized

            InitializeMarkerTracking();

            if (m_debugRenderer != null)
                m_debugRenderer.gameObject.SetActive(m_showRecogResult);
        }

        private void Update()
        {
            if (m_passthroughCameraAccess == null || !m_passthroughCameraAccess.IsPlaying || !m_arucoMarkerTracking.IsReady)
                return;

            HandleVisualizationToggle();

            //======================================================================================
            // CORE FUNCTIONALITY: hand the newest camera frame (and the camera pose it was taken
            // from) to the tracker. Detection runs asynchronously on a background thread; this call
            // returns immediately and applies results whenever they are ready.
            //======================================================================================
            Texture cameraTexture = m_passthroughCameraAccess.GetTexture();
            m_arucoMarkerTracking.UpdateTracking(
                cameraTexture,
                m_passthroughCameraAccess.GetCameraPose(),
                m_passthroughCameraAccess.Timestamp,
                m_passthroughCameraAccess.IsUpdatedThisFrame);

            UpdateMarkerObjects();

            if (m_showRecogResult && m_debugRenderer != null && m_debugRenderer.material.mainTexture != cameraTexture)
                m_debugRenderer.material.mainTexture = cameraTexture;
        }

        /// <summary>
        /// Handles button input to toggle between the camera image and AR marker objects.
        /// </summary>
        private void HandleVisualizationToggle()
        {
            if (m_debugRenderer == null)
                return;

            if (OVRInput.GetDown(OVRInput.Button.One))
            {
                m_showRecogResult = !m_showRecogResult;
                m_debugRenderer.gameObject.SetActive(m_showRecogResult);
            }
        }

        /// <summary>
        /// Moves each tracked marker's GameObject to its pose and shows/hides objects.
        /// </summary>
        private void UpdateMarkerObjects()
        {
            m_arucoMarkerTracking.GetTrackedMarkerIds(m_trackedIds);
            foreach (int markerId in m_trackedIds)
            {
                if (!m_arucoMarkerTracking.TryGetMarkerPose(markerId, out Pose pose))
                    continue;

                MarkerEntry entry = GetOrCreateMarker(markerId);
                if (entry == null)
                    continue;

                // The tracker already smooths poses, so by default objects go straight to them.
                // An optional glide can hide the step between detections. Snap on (re)appear.
                Transform t = entry.Root.transform;
                if (!entry.Visible || m_followSpeed <= 0f)
                {
                    t.SetPositionAndRotation(pose.position, pose.rotation);
                }
                else
                {
                    float k = 1f - Mathf.Exp(-m_followSpeed * Time.deltaTime);
                    t.SetPositionAndRotation(
                        Vector3.Lerp(t.position, pose.position, k),
                        Quaternion.Slerp(t.rotation, pose.rotation, k));
                }
            }

            foreach (var marker in m_markers)
            {
                bool visible = !m_showRecogResult && m_arucoMarkerTracking.IsMarkerTracked(marker.Key);
                if (marker.Value.Visible == visible)
                    continue;

                marker.Value.Visible = visible;
                marker.Value.Root.SetActive(visible);
            }
        }

        /// <summary>
        /// Reads camera intrinsics, initializes the tracker and registers the configured marker objects.
        /// </summary>
        private void InitializeMarkerTracking()
        {
            var intrinsics = m_passthroughCameraAccess.Intrinsics;
            var cx = intrinsics.PrincipalPoint.x;
            var cy = intrinsics.PrincipalPoint.y;
            var fx = intrinsics.FocalLength.x;
            var fy = intrinsics.FocalLength.y;
            var width = intrinsics.SensorResolution.x;
            var height = intrinsics.SensorResolution.y;

            // Scale parameters to match the current camera resolution
            var currentResolution = m_passthroughCameraAccess.CurrentResolution;

            if (currentResolution.x != width || currentResolution.y != height)
            {
                float scaleX = (float)currentResolution.x / width;
                float scaleY = (float)currentResolution.y / height;
                fx *= scaleX;
                fy *= scaleY;
                cx *= scaleX;
                cy *= scaleY;
                width = currentResolution.x;
                height = currentResolution.y;
            }

            m_arucoMarkerTracking.Initialize(width, height, cx, cy, fx, fy);

            CreateOutlineMaterial();
            foreach (var pair in m_markerGameObjectPairs)
            {
                if (pair.gameObject == null || m_markers.ContainsKey(pair.markerId))
                    continue;

                if (m_hideAssignedObjectRenderers)
                {
                    foreach (Renderer existingRenderer in pair.gameObject.GetComponentsInChildren<Renderer>(true))
                        existingRenderer.enabled = false;
                }

                RegisterMarker(pair.markerId, pair.gameObject);
            }
        }

        private MarkerEntry GetOrCreateMarker(int markerId)
        {
            if (m_markers.TryGetValue(markerId, out MarkerEntry entry))
                return entry;

            if (markerId < 0 || markerId >= m_arucoMarkerTracking.DictionarySize)
                return null;

            // Created lazily the first time this ID is seen, so every ID in the dictionary works
            // without having to set up hundreds of objects in the scene.
            return RegisterMarker(markerId, new GameObject($"ARGameObject_{markerId}"));
        }

        private MarkerEntry RegisterMarker(int markerId, GameObject root)
        {
            if (m_showMarkerOutlines)
                CreateMarkerVisual(markerId, root.transform);

            root.SetActive(false);
            var entry = new MarkerEntry { Root = root, Visible = false };
            m_markers[markerId] = entry;
            return entry;
        }

        private void CreateOutlineMaterial()
        {
            if (m_markerOutlineMaterial != null)
                return;

            Shader outlineShader = Shader.Find("Universal Render Pipeline/Unlit");
            if (outlineShader == null)
                outlineShader = Shader.Find("Sprites/Default");
            if (outlineShader == null)
                throw new InvalidOperationException("Unable to find a shader for the marker outline.");

            m_markerOutlineMaterial = new Material(outlineShader) { color = m_outlineColor };
            m_labelBackingMaterial = new Material(outlineShader) { color = LabelBackingColor };
            m_labelFont = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        }

        private void CreateMarkerVisual(int markerId, Transform parent)
        {
            float markerSize = m_arucoMarkerTracking.MarkerLength;
            float halfSize = markerSize * 0.5f;

            GameObject visual = new GameObject($"Marker {markerId} Outline");
            visual.transform.SetParent(parent, false);

            LineRenderer outline = visual.AddComponent<LineRenderer>();
            outline.useWorldSpace = false;
            outline.loop = true;
            outline.positionCount = 4;
            outline.SetPosition(0, new Vector3(-halfSize, -halfSize, 0f));
            outline.SetPosition(1, new Vector3(-halfSize, halfSize, 0f));
            outline.SetPosition(2, new Vector3(halfSize, halfSize, 0f));
            outline.SetPosition(3, new Vector3(halfSize, -halfSize, 0f));
            outline.widthMultiplier = Mathf.Max(markerSize * 0.12f, 0.002f);
            outline.alignment = LineAlignment.View;
            outline.numCornerVertices = 2;
            outline.startColor = m_outlineColor;
            outline.endColor = m_outlineColor;
            outline.sharedMaterial = m_markerOutlineMaterial;
            outline.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            outline.receiveShadows = false;

            // Dark plate in the middle of the square so the number stays readable over the marker pattern.
            GameObject backing = GameObject.CreatePrimitive(PrimitiveType.Quad);
            backing.name = $"Marker {markerId} Label Backing";
            Destroy(backing.GetComponent<Collider>());
            backing.transform.SetParent(visual.transform, false);
            backing.transform.localPosition = new Vector3(0f, 0f, -markerSize * 0.005f);
            backing.transform.localScale = new Vector3(markerSize * 0.62f, markerSize * 0.5f, 1f);
            MeshRenderer backingRenderer = backing.GetComponent<MeshRenderer>();
            backingRenderer.sharedMaterial = m_labelBackingMaterial;
            backingRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            backingRenderer.receiveShadows = false;

            string text = markerId.ToString();
            GameObject labelObject = new GameObject($"Marker {markerId} Label");
            labelObject.transform.SetParent(visual.transform, false);
            labelObject.transform.localPosition = new Vector3(0f, 0f, -markerSize * 0.01f);
            labelObject.transform.localScale = Vector3.one * markerSize * (text.Length >= 3 ? 0.03f : 0.042f);

            TextMesh label = labelObject.AddComponent<TextMesh>();
            label.font = m_labelFont;
            label.fontSize = 100;
            label.fontStyle = FontStyle.Bold;
            label.characterSize = 1f;
            label.anchor = TextAnchor.MiddleCenter;
            label.alignment = TextAlignment.Center;
            label.color = m_outlineColor;
            label.text = text;
            label.GetComponent<MeshRenderer>().sharedMaterial = m_labelFont.material;
        }

        private void OnDestroy()
        {
            if (m_markerOutlineMaterial != null)
                Destroy(m_markerOutlineMaterial);
            if (m_labelBackingMaterial != null)
                Destroy(m_labelBackingMaterial);
        }
    }
}
