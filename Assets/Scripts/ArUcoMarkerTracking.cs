using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.Rendering;

namespace QuestArucoTracker
{
    /// <summary>
    /// Detects ArUco markers and estimates their poses with a native OpenCV plugin (libarucobridge.so).
    ///
    /// Pipeline (nothing blocks the main thread):
    ///   1. GPU -> CPU: each new passthrough camera image is read back with AsyncGPUReadback straight
    ///      into one of a few reusable buffers. Reading the camera texture directly (instead of
    ///      blitting it first) guarantees the image matches the camera pose and timestamp.
    ///   2. Native worker thread: grayscale conversion, ArUco detection (only around known markers,
    ///      with regular full-frame scans) and pose estimation. The plugin reads Unity's buffer
    ///      directly: no copies, no Java.
    ///   3. Main thread: results are placed in world space with the camera pose of that image, then
    ///      run through a predictive filter that estimates each marker's velocity.
    ///   4. Every frame, poses are extrapolated from the camera's capture time to the time the frame is
    ///      shown. That hides the camera and processing delay and makes motion fluid at the display
    ///      rate instead of stepping at the camera rate.
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
        [SerializeField, Tooltip("Downscale factor applied before detection. 1 = full camera resolution (needed for small markers), 2 = half (faster, shorter range).")]
        private int _divideNumber = 1;
        [SerializeField, Range(0.01f, 0.2f), Tooltip("Smallest marker to detect: perimeter as a fraction of the largest image side. Lower = detects smaller/further markers but slower.")]
        private float _minMarkerPerimeterRate = 0.04f;
        [SerializeField, Tooltip("Sub-pixel corner refinement. More stable poses, slightly slower.")]
        private bool _cornerRefinement = true;
        [SerializeField, Range(1, 30), Tooltip("Once markers are found, only the area around them is searched (much faster). Every this many frames the whole image is scanned to find new markers. 1 = always scan the whole image.")]
        private int _fullScanInterval = 6;
        [SerializeField, Range(1, 6), Tooltip("CPU threads OpenCV may use for detection.")]
        private int _detectionThreads = 3;
        [SerializeField, Tooltip("Flip the image vertically before detection (Unity readbacks are bottom-up).")]
        private bool _flipVertically = true;
        [SerializeField, Tooltip("If no markers are found for a while, also try the image the other way up and keep whichever works.")]
        private bool _autoOrientation = true;

        [Header("Motion Prediction")]
        [SerializeField, Tooltip("Extrapolate marker motion to the moment the frame is displayed. Removes the drag behind moving markers.")]
        private bool _predictMotion = true;
        [SerializeField, Range(0f, 1f), Tooltip("How much of the delay to predict away. 1 = all of it (least drag on smooth moves, can overshoot on fast shakes), 0 = none.")]
        private float _predictionStrength = 0.5f;
        [SerializeField, Range(0f, 0.1f), Tooltip("Extra prediction (seconds) for the time between Update and the frame reaching the display.")]
        private float _displayLatency = 0.02f;
        [SerializeField, Range(0f, 0.3f), Tooltip("Never predict further ahead than this (seconds). Limits overshoot when a marker stops suddenly or is lost.")]
        private float _maxPrediction = 0.15f;
        [SerializeField, Range(0f, 0.2f), Tooltip("Never move a marker more than this (meters) ahead of its last detection. Stops fast motion from throwing it far off.")]
        private float _maxPredictionDistance = 0.05f;
        [SerializeField, Range(0f, 90f), Tooltip("Never rotate a marker more than this (degrees) ahead of its last detection.")]
        private float _maxPredictionAngle = 25f;

        [Header("Smoothing")]
        [SerializeField, Range(0.0002f, 0.01f), Tooltip("Typical position noise (meters) of a still marker 30 cm away. Movements smaller than this are smoothed out; larger ones are followed immediately.")]
        private float _positionNoise = 0.0015f;
        [SerializeField, Range(0.1f, 10f), Tooltip("Typical rotation noise (degrees) of a still marker 30 cm away.")]
        private float _rotationNoise = 1.5f;
        [SerializeField, Range(0.05f, 1f), Tooltip("How much a new detection is trusted while the marker is still. Lower = steadier, higher = more responsive.")]
        private float _stillResponsiveness = 0.25f;
        [SerializeField, Range(0.05f, 1f), Tooltip("How quickly the estimated speed reacts to new detections. Lower = calmer prediction during fast or jerky motion, higher = prediction kicks in sooner.")]
        private float _velocityResponsiveness = 0.5f;
        [SerializeField, Tooltip("If a marker jumps more than this (meters) between detections, snap instead of smoothing.")]
        private float _snapDistance = 0.1f;
        [SerializeField, Tooltip("Keep a marker visible this many seconds after it was last detected (avoids flicker).")]
        private float _lostTimeout = 0.25f;

        private const int BufferCount = 4;          // readback + waiting + processing, with one spare
        private const int MaxReadbacksInFlight = 2;
        private const int FrameRingSize = 16;
        private const int ResultHeader = 3;
        private const int ValuesPerMarker = 7;
        private const float MovingGain = 0.9f;      // detection weight while clearly moving
        private const float DepthNoiseRatio = 1f / 3f; // depth from a small marker is ~3x noisier than x/y
        private const float NoiseReferenceDistance = 0.3f;
        private const double FallbackCameraLatency = 0.045;
        private const float MaxSpeed = 3f;                 // m/s, faster than a hand-held marker moves
        private const float MaxAcceleration = 30f;         // m/s^2, limits how fast the speed estimate can change
        private const float MaxAngularSpeed = 15f;         // rad/s
        private const float MaxAngularAcceleration = 150f; // rad/s^2
        private const float JumpAngleDegrees = 45f;        // a sudden rotation this large must be confirmed
        private const float StaleFalloff = 0.05f;          // seconds; how quickly prediction fades once detections stop
        private const float BrakingResponsiveness = 0.8f;  // velocity gain when the marker slows down
        private const float TrustMissStart = 1.0f;         // prediction misses (relative to expected motion) below this are fine
        private const float TrustMissEnd = 3.0f;           // at this miss, prediction is switched off
        private const float SpeedDeadZone = 0.02f;         // m/s below which a marker counts as still
        private const float SpinDeadZone = 0.3f;           // rad/s below which a marker counts as not rotating

        private struct TrackedMarker
        {
            public Vector3 Position;
            public Vector3 Velocity;
            public Quaternion Rotation;
            public Vector3 AngularVelocity; // world space, radians per second
            public double SampleTime;       // capture time of the image this state belongs to
            public float LastSeen;          // Time.time when the detection arrived
            // 0..1: how well the motion model has been predicting lately. Prediction is scaled by it,
            // so it fades out while a marker speeds up, stops or changes direction.
            public float PositionTrust;
            public float RotationTrust;
            // A detection far from where the marker should be is held back until the next one agrees.
            public bool HasPendingJump;
            public Vector3 JumpPosition;
            public Quaternion JumpRotation;
        }

        private struct FrameInfo
        {
            public int Id;
            public Pose CameraPose;
            public double SampleTime;
        }

        private readonly Dictionary<int, TrackedMarker> _trackedMarkers = new Dictionary<int, TrackedMarker>();
        private readonly List<int> _markerIdBuffer = new List<int>();
        private readonly FrameInfo[] _frames = new FrameInfo[FrameRingSize];
        private readonly NativeArray<byte>[] _buffers = new NativeArray<byte>[BufferCount];
        private readonly bool[] _bufferReading = new bool[BufferCount];
        private readonly FrameInfo[] _bufferFrame = new FrameInfo[BufferCount];
        private readonly int[] _bufferGeneration = new int[BufferCount];
        private Action<AsyncGPUReadbackRequest>[] _readbackCallbacks;
        private float[] _resultBuffer;

        private int _imageWidth;
        private int _imageHeight;
        private float _cx;
        private float _cy;
        private float _fx;
        private float _fy;
        private bool _isReady;
        private int _generation;
        private int _nextFrameId = 1;
        private int _readbacksInFlight;
        private long _timeBaseTicks;
        private bool _cameraClockValid = true;
        private float _typicalResultAge = 0.1f; // how old an image usually is when its result arrives

        // With slow detection a marker must survive the gap between detections (plus one missed one).
        private float LostTimeout => Mathf.Max(_lostTimeout, 0.15f);

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

#if UNITY_ANDROID && !UNITY_EDITOR
        private const string PluginName = "arucobridge";

        [DllImport(PluginName)]
        private static extern int aruco_initialize(int dictionaryId, float minMarkerPerimeterRate, int cornerRefinement,
            int numThreads, int fullScanInterval, int autoOrientation);

        [DllImport(PluginName)]
        private static extern void aruco_shutdown();

        [DllImport(PluginName)]
        private static extern int aruco_submit(IntPtr pixels, int width, int height, int channels, int flipVertically,
            int divide, float fx, float fy, float cx, float cy, float markerLength, int frameId, int bufferIndex);

        [DllImport(PluginName)]
        private static extern int aruco_is_buffer_in_use(int bufferIndex);

        [DllImport(PluginName)]
        private static extern int aruco_take_result([Out] float[] output, int capacity);
#endif

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
            _timeBaseTicks = DateTime.UtcNow.Ticks;
            _cameraClockValid = true;

#if UNITY_ANDROID && !UNITY_EDITOR
            int byteCount = imageWidth * imageHeight * 4;
            for (int i = 0; i < BufferCount; i++)
                _buffers[i] = new NativeArray<byte>(byteCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

            if (_readbackCallbacks == null)
            {
                _readbackCallbacks = new Action<AsyncGPUReadbackRequest>[BufferCount];
                for (int i = 0; i < BufferCount; i++)
                {
                    int index = i;
                    _readbackCallbacks[i] = request => OnReadbackComplete(request, index);
                }
            }

            _resultBuffer = new float[ResultHeader + ValuesPerMarker * DictionarySize];

            if (aruco_initialize((int)_dictionaryId, _minMarkerPerimeterRate, _cornerRefinement ? 1 : 0,
                    _detectionThreads, _fullScanInterval, _autoOrientation ? 1 : 0) == 0)
            {
                Debug.LogError("[ArUco] Native detector failed to start.");
                ReleaseResources();
                return;
            }
            _isReady = true;
#else
            Debug.LogWarning("The native ArUco tracker is available in an Android player build only.");
#endif
        }

        /// <summary>
        /// Call once per frame with the camera texture, the camera pose and timestamp of the latest image,
        /// and whether the camera delivered a new image this frame.
        /// </summary>
        public void UpdateTracking(Texture cameraTexture, Pose cameraPose, DateTime cameraTimestamp, bool isNewCameraFrame)
        {
            if (!_isReady)
                return;

            PollDetectionResult();

            if (isNewCameraFrame && cameraTexture != null && _readbacksInFlight < MaxReadbacksInFlight
                && cameraTexture.width == _imageWidth && cameraTexture.height == _imageHeight)
            {
                int bufferIndex = FindFreeBuffer();
                if (bufferIndex >= 0)
                    RequestReadback(bufferIndex, cameraTexture, cameraPose, CameraTimeToSeconds(cameraTimestamp));
            }

            RemoveLostMarkers();
        }

        /// <summary>Kept for compatibility: assumes the image is new and was captured ~45 ms ago.</summary>
        public void UpdateTracking(Texture cameraTexture, Pose cameraPose, bool isNewCameraFrame = true)
        {
            UpdateTracking(cameraTexture, cameraPose, DateTime.MinValue, isNewCameraFrame);
        }

        /// <summary>True if the marker was detected within the last <c>_lostTimeout</c> seconds.</summary>
        public bool IsMarkerTracked(int markerId)
        {
            return _trackedMarkers.TryGetValue(markerId, out TrackedMarker marker)
                   && Time.time - marker.LastSeen <= LostTimeout;
        }

        /// <summary>Kept for compatibility with older code.</summary>
        public bool IsMarkerDetected(int markerId) => IsMarkerTracked(markerId);

        /// <summary>
        /// World-space pose of a tracked marker, predicted to the moment the current frame is displayed.
        /// </summary>
        public bool TryGetMarkerPose(int markerId, out Pose pose)
        {
            if (_trackedMarkers.TryGetValue(markerId, out TrackedMarker marker)
                && Time.time - marker.LastSeen <= LostTimeout)
            {
                pose = PredictPose(marker, (float)(NowSeconds() - marker.SampleTime));
                return true;
            }

            pose = default;
            return false;
        }

        /// <summary>Pose of <paramref name="marker"/> extrapolated from its image's capture time by <paramref name="age"/> seconds.</summary>
        private Pose PredictPose(TrackedMarker marker, float age)
        {
            if (!_predictMotion)
                return new Pose(marker.Position, marker.Rotation);

            float horizon = Mathf.Clamp(age + _displayLatency, 0f, _maxPrediction) * _predictionStrength;

            // Detections stopped arriving (blur, occlusion): fade the prediction out instead of
            // carrying the marker further and further along its last direction.
            float expectedAge = _typicalResultAge + 1f / 30f;
            float freshness = age > expectedAge ? Mathf.Exp(-(age - expectedAge) / StaleFalloff) : 1f;

            // Speeds at noise level are not real motion: don't extrapolate them.
            float moving = Mathf.SmoothStep(0f, 1f, Mathf.InverseLerp(SpeedDeadZone, 3f * SpeedDeadZone, marker.Velocity.magnitude));
            float spinning = Mathf.SmoothStep(0f, 1f, Mathf.InverseLerp(SpinDeadZone, 3f * SpinDeadZone, marker.AngularVelocity.magnitude));

            Vector3 offset = marker.Velocity * (horizon * marker.PositionTrust * freshness * moving);
            offset = Vector3.ClampMagnitude(offset, _maxPredictionDistance);

            Vector3 spin = marker.AngularVelocity * (horizon * marker.RotationTrust * freshness * spinning);
            spin = Vector3.ClampMagnitude(spin, _maxPredictionAngle * Mathf.Deg2Rad);

            return new Pose(marker.Position + offset, Integrate(marker.Rotation, spin, 1f));
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

        // ---- Time --------------------------------------------------------------------------------------

        // The camera timestamp is wall-clock time (microseconds since the Unix epoch), the same clock as
        // DateTime.UtcNow, so the age of an image is simply "now - timestamp".
        private double NowSeconds() => (DateTime.UtcNow.Ticks - _timeBaseTicks) * 1e-7;

        private double CameraTimeToSeconds(DateTime cameraTimestamp)
        {
            double now = NowSeconds();
            if (_cameraClockValid && cameraTimestamp != DateTime.MinValue)
            {
                double sample = (cameraTimestamp.Ticks - _timeBaseTicks) * 1e-7;
                double age = now - sample;
                if (age >= 0.0 && age < 0.5)
                    return sample;
                _cameraClockValid = false; // clocks don't line up: estimate instead
            }
            return now - FallbackCameraLatency;
        }

        // ---- Readback -----------------------------------------------------------------------------------

        private int FindFreeBuffer()
        {
#if UNITY_ANDROID && !UNITY_EDITOR
            for (int i = 0; i < BufferCount; i++)
            {
                if (!_bufferReading[i] && aruco_is_buffer_in_use(i) == 0)
                    return i;
            }
#endif
            return -1;
        }

        private void RequestReadback(int bufferIndex, Texture cameraTexture, Pose cameraPose, double sampleTime)
        {
            _bufferReading[bufferIndex] = true;
            _bufferGeneration[bufferIndex] = _generation;
            _bufferFrame[bufferIndex] = new FrameInfo { CameraPose = cameraPose, SampleTime = sampleTime };
            _readbacksInFlight++;

            AsyncGPUReadback.RequestIntoNativeArray(ref _buffers[bufferIndex], cameraTexture, 0, _readbackCallbacks[bufferIndex]);
        }

        private unsafe void OnReadbackComplete(AsyncGPUReadbackRequest request, int bufferIndex)
        {
            if (_bufferGeneration[bufferIndex] != _generation || !_isReady)
                return;

            _bufferReading[bufferIndex] = false;
            _readbacksInFlight = Mathf.Max(0, _readbacksInFlight - 1);
            if (request.hasError)
                return;

#if UNITY_ANDROID && !UNITY_EDITOR
            int frameId = _nextFrameId;
            _nextFrameId = (_nextFrameId + 1) & 0xFFFFF; // stays exactly representable as a float
            if (_nextFrameId == 0)
                _nextFrameId = 1;

            FrameInfo frame = _bufferFrame[bufferIndex];
            frame.Id = frameId;

            float scale = 1f / _divideNumber;
            IntPtr pixels = (IntPtr)NativeArrayUnsafeUtility.GetUnsafeReadOnlyPtr(_buffers[bufferIndex]);
            int accepted = aruco_submit(pixels, _imageWidth, _imageHeight, 4, _flipVertically ? 1 : 0, _divideNumber,
                _fx * scale, _fy * scale, _cx * scale, _cy * scale, _markerLength, frameId, bufferIndex);
            if (accepted != 0)
                _frames[frameId % FrameRingSize] = frame;
#endif
        }

        // ---- Results ------------------------------------------------------------------------------------

        private void PollDetectionResult()
        {
#if UNITY_ANDROID && !UNITY_EDITOR
            int count = aruco_take_result(_resultBuffer, _resultBuffer.Length);
            if (count < ResultHeader)
                return;

            int frameId = Mathf.RoundToInt(_resultBuffer[0]);
            FrameInfo frame = _frames[frameId % FrameRingSize];
            if (frame.Id != frameId)
                return; // stale result from before a re-initialize

            LastDetectionMs = _resultBuffer[1];
            int markerCount = Mathf.RoundToInt(_resultBuffer[2]);
            float now = Time.time;

            float resultAge = (float)(NowSeconds() - frame.SampleTime);
            if (resultAge > 0f && resultAge < 0.5f)
                _typicalResultAge = Mathf.Lerp(_typicalResultAge, resultAge, 0.1f);

            for (int i = 0; i < markerCount; i++)
            {
                int offset = ResultHeader + i * ValuesPerMarker;
                if (offset + ValuesPerMarker > count)
                    break;

                int markerId = Mathf.RoundToInt(_resultBuffer[offset]);
                Vector3 cameraSpacePosition = new Vector3(_resultBuffer[offset + 1], -_resultBuffer[offset + 2], _resultBuffer[offset + 3]);
                Quaternion cameraSpaceRotation = ConvertRvecToUnityRotation(_resultBuffer[offset + 4], _resultBuffer[offset + 5], _resultBuffer[offset + 6]);

                // Use the pose the camera had when this image was captured, not the current one.
                Vector3 position = frame.CameraPose.position + frame.CameraPose.rotation * cameraSpacePosition;
                Quaternion rotation = frame.CameraPose.rotation * cameraSpaceRotation;

                _trackedMarkers[markerId] = Filter(markerId, position, rotation, frame, cameraSpacePosition.magnitude, now);
            }
#endif
        }

        /// <summary>
        /// Predictive alpha-beta filter. Each marker keeps a position, velocity, rotation and spin. A new
        /// detection is compared with where the marker was expected to be: differences within the noise
        /// level are mostly ignored (steady when still), larger ones are followed almost fully and update
        /// the velocity (no lag when moving). The velocity is what lets poses be predicted forward.
        /// </summary>
        private TrackedMarker Filter(int markerId, Vector3 position, Quaternion rotation, FrameInfo frame, float distance, float now)
        {
            var fresh = new TrackedMarker
            {
                Position = position,
                Velocity = Vector3.zero,
                Rotation = rotation,
                AngularVelocity = Vector3.zero,
                SampleTime = frame.SampleTime,
                LastSeen = now
            };

            if (!_trackedMarkers.TryGetValue(markerId, out TrackedMarker previous)
                || now - previous.LastSeen > LostTimeout)
            {
                return fresh;
            }

            double rawDt = frame.SampleTime - previous.SampleTime;
            if (rawDt <= 0.0)
                return previous; // older than what we already have

            float dt = Mathf.Clamp((float)rawDt, 1f / 120f, 0.25f);

            // Where the marker should be now if it kept moving the same way.
            Vector3 predictedPosition = previous.Position + previous.Velocity * dt;
            Quaternion predictedRotation = Integrate(previous.Rotation, previous.AngularVelocity, dt);

            Vector3 residual = position - predictedPosition;
            Vector3 rotationResidual = RotationVector(rotation * Quaternion.Inverse(predictedRotation));
            float residualDegrees = rotationResidual.magnitude * Mathf.Rad2Deg;

            // ---- Glitch rejection ---------------------------------------------------------------------
            // A detection far away from where the marker should be is usually a blurred or flipped
            // detection, not real motion. Hold it back; accept it only if the next detection agrees.
            bool jump = residual.sqrMagnitude > _snapDistance * _snapDistance || residualDegrees > JumpAngleDegrees;
            if (jump)
            {
                bool confirmed = previous.HasPendingJump
                                 && (position - previous.JumpPosition).sqrMagnitude < _snapDistance * _snapDistance * 0.25f
                                 && Quaternion.Angle(rotation, previous.JumpRotation) < JumpAngleDegrees * 0.5f;
                if (confirmed)
                    return fresh; // it really moved: start over from here

                previous.HasPendingJump = true;
                previous.JumpPosition = position;
                previous.JumpRotation = rotation;
                previous.LastSeen = now;
                return previous;
            }

            float distanceScale = Mathf.Max(1f, distance / NoiseReferenceDistance);
            float positionNoise = _positionNoise * distanceScale * distanceScale;
            float rotationNoise = _rotationNoise * distanceScale;

            // ---- Position -------------------------------------------------------------------------------
            // Judge the residual in camera space, where depth is noisier than x/y.
            Vector3 cameraResidual = Quaternion.Inverse(frame.CameraPose.rotation) * residual;
            cameraResidual.z *= DepthNoiseRatio;
            float positionError = cameraResidual.magnitude / positionNoise;
            float positionGain = Gain(positionError);
            float positionBeta = Mathf.Min(positionGain * positionGain / (2f - positionGain), _velocityResponsiveness);
            // Slowing down must react quickly (that is where overshoot comes from); speeding up may be gradual.
            if (Vector3.Dot(residual, previous.Velocity) < 0f)
                positionBeta = Mathf.Max(positionBeta, BrakingResponsiveness * positionGain);

            Vector3 velocityChange = Vector3.ClampMagnitude(residual * (positionBeta / dt), MaxAcceleration * dt);
            fresh.Position = predictedPosition + residual * positionGain;
            fresh.Velocity = Vector3.ClampMagnitude(previous.Velocity + velocityChange, MaxSpeed);

            // ---- Rotation -------------------------------------------------------------------------------
            float rotationError = residualDegrees / rotationNoise;
            float rotationGain = Gain(rotationError);
            float rotationBeta = Mathf.Min(rotationGain * rotationGain / (2f - rotationGain), _velocityResponsiveness);
            if (Vector3.Dot(rotationResidual, previous.AngularVelocity) < 0f)
                rotationBeta = Mathf.Max(rotationBeta, BrakingResponsiveness * rotationGain);

            Vector3 spinChange = Vector3.ClampMagnitude(rotationResidual * (rotationBeta / dt), MaxAngularAcceleration * dt);
            fresh.Rotation = Integrate(predictedRotation, rotationResidual * rotationGain, 1f);
            fresh.AngularVelocity = Vector3.ClampMagnitude(previous.AngularVelocity + spinChange, MaxAngularSpeed);

            // A still marker should not drift: let noise-level velocity die out quickly.
            if (positionError < 1f)
                fresh.Velocity *= 0.7f;
            if (rotationError < 1f)
                fresh.AngularVelocity *= 0.7f;

            // ---- Prediction trust -----------------------------------------------------------------------
            // Compare the miss with how far the marker was expected to move. Small misses mean the
            // motion is steady and prediction helps; big misses mean it is speeding up, stopping or
            // turning, where prediction would overshoot.
            float expectedMove = previous.Velocity.magnitude * dt + 3f * positionNoise;
            fresh.PositionTrust = UpdateTrust(previous.PositionTrust, residual.magnitude / expectedMove);

            float expectedTurn = previous.AngularVelocity.magnitude * Mathf.Rad2Deg * dt + 3f * rotationNoise;
            fresh.RotationTrust = UpdateTrust(previous.RotationTrust, residualDegrees / expectedTurn);

            return fresh;
        }

        // Trust drops quickly when the prediction misses and recovers gradually when it hits.
        private static float UpdateTrust(float trust, float miss)
        {
            float target = Mathf.Clamp01(1f - (miss - TrustMissStart) / (TrustMissEnd - TrustMissStart));
            return Mathf.Lerp(trust, target, target < trust ? 0.8f : 0.35f);
        }

        // Shortest-path rotation as an axis * angle vector (radians).
        private static Vector3 RotationVector(Quaternion delta)
        {
            if (delta.w < 0f)
                delta = new Quaternion(-delta.x, -delta.y, -delta.z, -delta.w);
            delta.ToAngleAxis(out float angleDegrees, out Vector3 axis);
            if (angleDegrees > 180f)
                angleDegrees -= 360f;
            if (!IsFinite(axis) || Mathf.Abs(angleDegrees) < 1e-4f)
                return Vector3.zero;
            return axis * (angleDegrees * Mathf.Deg2Rad);
        }

        // 0..1: how much of the difference to the expected pose is accepted.
        private float Gain(float normalizedError)
        {
            float t = Mathf.Clamp01((normalizedError - 1f) / 3f);
            return Mathf.SmoothStep(_stillResponsiveness, MovingGain, t);
        }

        private static Quaternion Integrate(Quaternion rotation, Vector3 angularVelocity, float time)
        {
            Vector3 rotationVector = angularVelocity * time;
            float angle = rotationVector.magnitude;
            if (angle < 1e-6f)
                return rotation;
            return Quaternion.AngleAxis(angle * Mathf.Rad2Deg, rotationVector / angle) * rotation;
        }

        private static bool IsFinite(Vector3 v)
        {
            return !(float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z)
                     || float.IsInfinity(v.x) || float.IsInfinity(v.y) || float.IsInfinity(v.z));
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
            bool wasReady = _isReady;
            _isReady = false;
            _trackedMarkers.Clear();

#if UNITY_ANDROID && !UNITY_EDITOR
            // Stop the worker first: once this returns, the plugin no longer reads any buffer.
            if (wasReady)
                aruco_shutdown();
#endif
            // Buffers must not be freed while the GPU may still write into them.
            if (_readbacksInFlight > 0)
                AsyncGPUReadback.WaitAllRequests();
            _readbacksInFlight = 0;

            for (int i = 0; i < BufferCount; i++)
            {
                _bufferReading[i] = false;
                if (_buffers[i].IsCreated)
                    _buffers[i].Dispose();
            }
            for (int i = 0; i < _frames.Length; i++)
                _frames[i] = default;
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
