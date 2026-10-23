using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace QuestArucoTracker
{
    /// <summary>
    /// Detects ArUco markers and estimates their poses using the OpenCV Android SDK.
    ///
    /// Pipeline (nothing blocks the main thread):
    ///   1. GPU: Blit each new passthrough frame into an R8 (grayscale, already flipped) render texture.
    ///   2. GPU -> CPU: AsyncGPUReadback (no stall, arrives 1-2 frames later).
    ///   3. Java worker thread: OpenCV detection + solvePnP. Frames are handed over as soon as they
    ///      arrive; if the detector is still busy, the waiting frame is replaced by the newer one.
    ///   4. Main thread: poll the result and convert it to world space using the camera pose
    ///      captured together with the frame, so markers do not drift when you move your head.
    ///      Poses are smoothed with a One Euro filter: strong smoothing when a marker is still,
    ///      almost none when it moves, so there is little jitter and little lag.
    /// Readback of the next frame runs while the detector works on the current one.
    /// </summary>
    public class ArUcoMarkerTracking : MonoBehaviour
    {
        public enum ArUcoDictionary
        {
            DICT_4X4_50 = 0,
            DICT_4X4_100 = 1,
            DICT_4X4_250 = 2,
            DICT_4X4_1000 = 3
        }

        [Header("Markers")]
        [SerializeField] private ArUcoDictionary _dictionaryId = ArUcoDictionary.DICT_4X4_50;
        [SerializeField, Tooltip("Physical side length of the black marker square, in meters (2 cm = 0.02).")]
        private float _markerLength = 0.02f;

        [Header("Detection")]
        [SerializeField, Range(0.5f, 30f), Tooltip("Maximum camera frames per second sent to the detector. 30 = every camera frame. Lower = less CPU/GPU load.")]
        private float _detectionsPerSecond = 30f;
        [SerializeField, Tooltip("Downscale factor applied before detection. 1 = full camera resolution (needed for small markers), 2 = half (faster, shorter range).")]
        private int _divideNumber = 1;
        [SerializeField, Range(0.01f, 0.2f), Tooltip("Smallest marker to detect: perimeter as a fraction of the largest image side. Lower = detects smaller/further markers but slower.")]
        private float _minMarkerPerimeterRate = 0.04f;
        [SerializeField, Tooltip("Sub-pixel corner refinement. More stable poses, slightly slower.")]
        private bool _cornerRefinement = true;
        [SerializeField, Tooltip("Flip the image vertically before detection (Unity readbacks are bottom-up). Only change this if no markers are found at all.")]
        private bool _flipVertically = true;

        [Header("Pose Smoothing (One Euro filter)")]
        [SerializeField, Tooltip("Smooth poses. Off = raw detections (lowest lag, most jitter).")]
        private bool _smoothPoses = true;
        [SerializeField, Range(0.1f, 10f), Tooltip("Smoothing cutoff (Hz) while a marker is still. Lower = less jitter when still, but slower to start following.")]
        private float _minCutoff = 1.5f;
        [SerializeField, Range(0f, 100f), Tooltip("How quickly smoothing is reduced as the marker moves (per m/s). Higher = less lag when moving.")]
        private float _positionBeta = 20f;
        [SerializeField, Range(0f, 20f), Tooltip("How quickly smoothing is reduced as the marker rotates (per rad/s). Higher = less lag when rotating.")]
        private float _rotationBeta = 2f;
        [SerializeField, Tooltip("If a marker moves more than this (meters) between detections, snap instead of smoothing.")]
        private float _snapDistance = 0.05f;
        [SerializeField, Tooltip("Keep a marker visible this many seconds after it was last detected (avoids flicker).")]
        private float _lostTimeout = 0.25f;

        // Cutoff for smoothing the speed estimate itself (standard One Euro value).
        private const float DerivativeCutoff = 1f;
        // Allow a capture slightly early so 30/s on a 30 fps camera doesn't skip every other frame.
        private const float CaptureIntervalTolerance = 0.75f;
        // Frames that can be "in flight" (readback + Java pending + Java busy) at once, with margin.
        private const int FrameRingSize = 8;

        private struct TrackedMarker
        {
            public Vector3 Position;
            public Quaternion Rotation;
            public Vector3 Velocity;
            public float AngularSpeed;
            public float SampleTime; // capture time of the frame this pose came from
            public float LastSeen;   // Time.time when the detection arrived
        }

        private struct FrameInfo
        {
            public int Id;
            public Pose CameraPose;
            public float CaptureTime;
        }

        private readonly Dictionary<int, TrackedMarker> _trackedMarkers = new Dictionary<int, TrackedMarker>();
        private readonly List<int> _markerIdBuffer = new List<int>();
        private readonly FrameInfo[] _frames = new FrameInfo[FrameRingSize];

        private int _imageWidth;
        private int _imageHeight;
        private float _cx;
        private float _cy;
        private float _fx;
        private float _fy;
        private bool _isReady;

        private RenderTexture _frameTexture;
        private Material _grayscaleMaterial;
        private int _frameWidth;
        private int _frameHeight;
        private int _channels;
        private byte[] _frameBuffer;

        private int _generation;
        private int _nextFrameId = 1;
        private int _lastResultFrameId = -1;
        private bool _readbackInFlight;
        private Pose _readbackPose;
        private float _readbackTime;
        private float _nextCaptureTime;

#if UNITY_ANDROID && !UNITY_EDITOR
        private AndroidJavaClass _openCvBridge;
#endif

        // With slow detection a marker must survive the gap between detections (plus one missed one).
        private float LostTimeout => Mathf.Max(_lostTimeout, 1.5f / Mathf.Max(0.1f, _detectionsPerSecond) + 0.1f);

        public int DivideNumber => _divideNumber;
        public float MarkerLength => _markerLength;
        public bool IsReady => _isReady;
        public float LastDetectionMs { get; private set; }

        public int DictionarySize
        {
            get
            {
                switch (_dictionaryId)
                {
                    case ArUcoDictionary.DICT_4X4_100: return 100;
                    case ArUcoDictionary.DICT_4X4_250: return 250;
                    case ArUcoDictionary.DICT_4X4_1000: return 1000;
                    default: return 50;
                }
            }
        }

        public void Initialize(int imageWidth, int imageHeight, float cx, float cy, float fx, float fy)
        {
            ReleaseResources();

            if (imageWidth <= 0 || imageHeight <= 0 || _divideNumber <= 0 || _markerLength <= 0)
            {
                Debug.LogError("ArUco tracking requires positive image dimensions, marker size, and divide number.");
                return;
            }

            if (!SystemInfo.supportsAsyncGPUReadback)
            {
                Debug.LogError("ArUco tracking requires AsyncGPUReadback support.");
                return;
            }

            _imageWidth = imageWidth;
            _imageHeight = imageHeight;
            _cx = cx;
            _cy = cy;
            _fx = fx;
            _fy = fy;
            _frameWidth = Mathf.Max(1, imageWidth / _divideNumber);
            _frameHeight = Mathf.Max(1, imageHeight / _divideNumber);

            CreateFrameTexture();

#if UNITY_ANDROID && !UNITY_EDITOR
            _openCvBridge = new AndroidJavaClass("com.questmarkertracking.ArucoTrackingBridge");
            _openCvBridge.CallStatic("initialize", (int)_dictionaryId, _minMarkerPerimeterRate, _cornerRefinement);
            _isReady = true;
#else
            Debug.LogWarning("The OpenCV Android SDK tracker is available in an Android player build only.");
#endif
        }

        /// <summary>
        /// Call once per frame with the latest camera texture and the camera pose for that texture.
        /// <paramref name="isNewCameraFrame"/> should be true only on frames where the camera delivered
        /// a new image, so the same image isn't detected twice.
        /// </summary>
        public void UpdateTracking(Texture cameraTexture, Pose cameraPose, bool isNewCameraFrame = true)
        {
            if (!_isReady)
                return;

            PollDetectionResult();

            float now = Time.realtimeSinceStartup;
            if (!_readbackInFlight && isNewCameraFrame && cameraTexture != null && now >= _nextCaptureTime)
            {
                _nextCaptureTime = now + CaptureIntervalTolerance / Mathf.Max(0.1f, _detectionsPerSecond);
                RequestReadback(cameraTexture, cameraPose, now);
            }

            RemoveLostMarkers();
        }

        /// <summary>True if the marker was detected within the last <c>_lostTimeout</c> seconds.</summary>
        public bool IsMarkerTracked(int markerId)
        {
            return _trackedMarkers.TryGetValue(markerId, out TrackedMarker marker)
                   && Time.time - marker.LastSeen <= LostTimeout;
        }

        /// <summary>Kept for compatibility with older code.</summary>
        public bool IsMarkerDetected(int markerId) => IsMarkerTracked(markerId);

        /// <summary>World-space pose of a tracked marker.</summary>
        public bool TryGetMarkerPose(int markerId, out Pose pose)
        {
            if (_trackedMarkers.TryGetValue(markerId, out TrackedMarker marker)
                && Time.time - marker.LastSeen <= LostTimeout)
            {
                pose = new Pose(marker.Position, marker.Rotation);
                return true;
            }

            pose = default;
            return false;
        }

        /// <summary>Fills <paramref name="markerIds"/> with all currently tracked marker IDs.</summary>
        public void GetTrackedMarkerIds(List<int> markerIds)
        {
            markerIds.Clear();
            markerIds.AddRange(_trackedMarkers.Keys);
        }

        public void Dispose()
        {
            ReleaseResources();
        }

        private void OnDestroy()
        {
            ReleaseResources();
        }

        private void CreateFrameTexture()
        {
            Shader grayscaleShader = Resources.Load<Shader>("ArucoGrayscale");
            bool useGrayscale = grayscaleShader != null
                                && grayscaleShader.isSupported
                                && SystemInfo.SupportsRenderTextureFormat(RenderTextureFormat.R8);

            if (useGrayscale)
            {
                _grayscaleMaterial = new Material(grayscaleShader);
                // Flip on the GPU while converting, so OpenCV doesn't have to.
                _grayscaleMaterial.SetFloat("_FlipY", _flipVertically ? 1f : 0f);
                _channels = 1;
                _frameTexture = new RenderTexture(_frameWidth, _frameHeight, 0, RenderTextureFormat.R8, RenderTextureReadWrite.Linear);
            }
            else
            {
                Debug.LogWarning("[ArUco] Grayscale shader unavailable, falling back to RGBA readback (slower).");
                _channels = 4;
                _frameTexture = new RenderTexture(_frameWidth, _frameHeight, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Default);
            }

            _frameTexture.filterMode = FilterMode.Bilinear;
            _frameTexture.wrapMode = TextureWrapMode.Clamp;
            _frameTexture.Create();
        }

        private void RequestReadback(Texture cameraTexture, Pose cameraPose, float captureTime)
        {
            if (_grayscaleMaterial != null)
                Graphics.Blit(cameraTexture, _frameTexture, _grayscaleMaterial);
            else
                Graphics.Blit(cameraTexture, _frameTexture);

            _readbackPose = cameraPose;
            _readbackTime = captureTime;
            _readbackInFlight = true;
            int generation = _generation;
            AsyncGPUReadback.Request(_frameTexture, 0, request => OnReadbackComplete(request, generation));
        }

        private void OnReadbackComplete(AsyncGPUReadbackRequest request, int generation)
        {
            if (generation != _generation || !_isReady)
                return;

            _readbackInFlight = false;
            if (request.hasError)
                return;

            var data = request.GetData<byte>();
            int expectedLength = _frameWidth * _frameHeight * _channels;
            if (data.Length != expectedLength)
            {
                Debug.LogWarning($"[ArUco] Unexpected readback size {data.Length}, expected {expectedLength}.");
                return;
            }

            if (_frameBuffer == null || _frameBuffer.Length != expectedLength)
                _frameBuffer = new byte[expectedLength];
            data.CopyTo(_frameBuffer);

            // Hand it over right away. The Java side keeps only the newest waiting frame,
            // and the call copies the array, so _frameBuffer can be reused immediately.
            SubmitFrame(_readbackPose, _readbackTime);
        }

        private void SubmitFrame(Pose cameraPose, float captureTime)
        {
#if UNITY_ANDROID && !UNITY_EDITOR
            float scaleX = (float)_frameWidth / _imageWidth;
            float scaleY = (float)_frameHeight / _imageHeight;
            int frameId = _nextFrameId;
            _nextFrameId = (_nextFrameId + 1) & 0xFFFFF; // stays exactly representable as a float

            bool accepted = _openCvBridge.CallStatic<bool>(
                "submitFrame",
                _frameBuffer,
                _frameWidth,
                _frameHeight,
                _channels,
                _flipVertically && _channels != 1, // the grayscale shader already flipped it
                _fx * scaleX,
                _fy * scaleY,
                _cx * scaleX,
                _cy * scaleY,
                _markerLength,
                frameId);

            if (accepted)
            {
                _frames[frameId % FrameRingSize] = new FrameInfo
                {
                    Id = frameId,
                    CameraPose = cameraPose,
                    CaptureTime = captureTime
                };
            }
#endif
        }

        private void PollDetectionResult()
        {
#if UNITY_ANDROID && !UNITY_EDITOR
            if (!_openCvBridge.CallStatic<bool>("hasResult"))
                return;

            float[] result = _openCvBridge.CallStatic<float[]>("takeResult");
            if (result == null || result.Length < 3)
                return;

            int frameId = Mathf.RoundToInt(result[0]);
            FrameInfo frame = _frames[frameId % FrameRingSize];
            if (frame.Id != frameId || frameId == _lastResultFrameId)
                return; // stale result from before a re-initialize, or already applied
            _lastResultFrameId = frameId;

            LastDetectionMs = result[1];
            int markerCount = Mathf.RoundToInt(result[2]);
            float now = Time.time;

            for (int i = 0; i < markerCount; i++)
            {
                int offset = 3 + i * 7;
                if (offset + 6 >= result.Length)
                    break;

                int markerId = Mathf.RoundToInt(result[offset]);
                Vector3 cameraSpacePosition = new Vector3(result[offset + 1], -result[offset + 2], result[offset + 3]);
                Quaternion cameraSpaceRotation = ConvertRvecToUnityRotation(result[offset + 4], result[offset + 5], result[offset + 6]);

                // Use the pose the camera had when this frame was captured, not the current one.
                Vector3 position = frame.CameraPose.position + frame.CameraPose.rotation * cameraSpacePosition;
                Quaternion rotation = frame.CameraPose.rotation * cameraSpaceRotation;

                _trackedMarkers[markerId] = FilterPose(markerId, position, rotation, frame.CaptureTime, now);
            }
#endif
        }

        /// <summary>
        /// One Euro filter (Casiez et al. 2012): the smoothing cutoff rises with the marker's speed,
        /// so a still marker is heavily smoothed and a moving marker follows almost without lag.
        /// </summary>
        private TrackedMarker FilterPose(int markerId, Vector3 position, Quaternion rotation, float sampleTime, float now)
        {
            var fresh = new TrackedMarker
            {
                Position = position,
                Rotation = rotation,
                Velocity = Vector3.zero,
                AngularSpeed = 0f,
                SampleTime = sampleTime,
                LastSeen = now
            };

            if (!_smoothPoses
                || !_trackedMarkers.TryGetValue(markerId, out TrackedMarker previous)
                || now - previous.LastSeen > LostTimeout
                || (position - previous.Position).sqrMagnitude > _snapDistance * _snapDistance)
            {
                return fresh;
            }

            float dt = sampleTime - previous.SampleTime;
            if (dt <= 1e-4f)
                dt = 1f / 30f;

            float derivativeAlpha = SmoothingAlpha(DerivativeCutoff, dt);

            Vector3 rawVelocity = (position - previous.Position) / dt;
            Vector3 velocity = Vector3.Lerp(previous.Velocity, rawVelocity, derivativeAlpha);
            float positionAlpha = SmoothingAlpha(_minCutoff + _positionBeta * velocity.magnitude, dt);

            float rawAngularSpeed = Quaternion.Angle(previous.Rotation, rotation) * Mathf.Deg2Rad / dt;
            float angularSpeed = Mathf.Lerp(previous.AngularSpeed, rawAngularSpeed, derivativeAlpha);
            float rotationAlpha = SmoothingAlpha(_minCutoff + _rotationBeta * angularSpeed, dt);

            fresh.Position = Vector3.Lerp(previous.Position, position, positionAlpha);
            fresh.Rotation = Quaternion.Slerp(previous.Rotation, rotation, rotationAlpha);
            fresh.Velocity = velocity;
            fresh.AngularSpeed = angularSpeed;
            return fresh;
        }

        private static float SmoothingAlpha(float cutoffHz, float dt)
        {
            float tau = 1f / (2f * Mathf.PI * cutoffHz);
            return 1f / (1f + tau / dt);
        }

        private void RemoveLostMarkers()
        {
            float now = Time.time;
            _markerIdBuffer.Clear();
            foreach (var marker in _trackedMarkers)
            {
                if (now - marker.Value.LastSeen > LostTimeout)
                    _markerIdBuffer.Add(marker.Key);
            }

            foreach (int markerId in _markerIdBuffer)
                _trackedMarkers.Remove(markerId);
        }

        private void ReleaseResources()
        {
            _generation++;
            _isReady = false;
            _readbackInFlight = false;
            _trackedMarkers.Clear();
            _frameBuffer = null;
            for (int i = 0; i < _frames.Length; i++)
                _frames[i] = default;

            if (_frameTexture != null)
            {
                _frameTexture.Release();
                Destroy(_frameTexture);
                _frameTexture = null;
            }
            if (_grayscaleMaterial != null)
            {
                Destroy(_grayscaleMaterial);
                _grayscaleMaterial = null;
            }

#if UNITY_ANDROID && !UNITY_EDITOR
            if (_openCvBridge != null)
            {
                _openCvBridge.CallStatic("shutdown");
                _openCvBridge.Dispose();
                _openCvBridge = null;
            }
#endif
        }

        private static Quaternion ConvertRvecToUnityRotation(float x, float y, float z)
        {
            float angle = Mathf.Sqrt(x * x + y * y + z * z);
            float cosine = Mathf.Cos(angle);
            float sine = Mathf.Sin(angle);
            float oneMinusCosine = 1f - cosine;
            float r01;
            float r02;
            float r11;
            float r12;
            float r21;
            float r22;

            if (angle < 1e-6f)
            {
                r11 = r22 = 1f;
                r01 = r02 = r12 = r21 = 0f;
            }
            else
            {
                float inverseAngle = 1f / angle;
                float axisX = x * inverseAngle;
                float axisY = y * inverseAngle;
                float axisZ = z * inverseAngle;
                r01 = axisX * axisY * oneMinusCosine - axisZ * sine;
                r02 = axisX * axisZ * oneMinusCosine + axisY * sine;
                r11 = cosine + axisY * axisY * oneMinusCosine;
                r12 = axisY * axisZ * oneMinusCosine - axisX * sine;
                r21 = axisZ * axisY * oneMinusCosine + axisX * sine;
                r22 = cosine + axisZ * axisZ * oneMinusCosine;
            }

            Vector3 unityUp = new Vector3(r01, -r11, r21);
            Vector3 unityForward = new Vector3(-r02, r12, -r22);
            return Quaternion.LookRotation(unityForward, unityUp);
        }
    }
}
