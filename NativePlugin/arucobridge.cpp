// Native ArUco detection bridge for Unity on Quest (arm64-v8a).
//
// Replaces the old Java bridge. Unity reads the passthrough camera texture back into its own
// NativeArray buffers and hands us a pointer. We never copy that buffer on the Unity thread:
// a worker thread converts it to grayscale, detects markers and estimates their poses.
//
// Threading:
//   - aruco_submit() is called from Unity's main thread. It only stores the frame description.
//     If a frame is already waiting, it is replaced (the newest frame always wins).
//   - The worker takes the waiting frame, processes it and publishes a result.
//   - aruco_is_buffer_in_use() tells Unity which of its buffers we may still read, so it never
//     overwrites a buffer we are working on.
//
// Speed:
//   - Region-of-interest tracking: once markers are found, only the area around them is searched.
//     A full-frame scan still runs every `fullScanInterval` frames to pick up new markers, and
//     immediately whenever the region search loses everything.
//
// Reliability:
//   - IPPE_SQUARE pose estimation returns two candidate poses for a square. For small markers the
//     wrong one is sometimes ranked first, which shows up as a sudden orientation flip. When the
//     two are close in error, we keep the one closest to that marker's previous orientation.
//   - Auto orientation: if no markers are found for a while, the vertically flipped image is
//     tried once. If that finds markers, the orientation is switched.
//
// Result layout (floats): [frameId, detectionMs, markerCount, (id, tx, ty, tz, rx, ry, rz) * count]
// Translation and Rodrigues rotation are in OpenCV camera space (x right, y down, z forward).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include <sys/resource.h>
#include <unistd.h>

#include <opencv2/core.hpp>
#include <opencv2/geometry/3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

#define ARUCO_EXPORT extern "C" __attribute__((visibility("default")))

namespace {

constexpr int kHeaderSize = 3;
constexpr int kValuesPerMarker = 7;
// Empty frames in a row before the other image orientation is tried.
constexpr int kOrientationProbeInterval = 10;
// If the second IPPE solution's error is within this factor of the best, the result is ambiguous.
constexpr double kAmbiguityRatio = 4.0;
// A marker's previous orientation is only trusted for this many frames.
constexpr int kRotationMemoryFrames = 15;

struct Frame {
    const uint8_t* pixels = nullptr;
    int width = 0;
    int height = 0;
    int channels = 0;
    int flip = 0;
    int divide = 1;
    float fx = 0, fy = 0, cx = 0, cy = 0;
    float markerLength = 0;
    int frameId = 0;
    int bufferIndex = -1;
};

struct RotationMemory {
    cv::Matx33d rotation;
    int frameId;
};

// ---- Shared state, guarded by gMutex -----------------------------------------------------------
std::mutex gMutex;
std::condition_variable gWake;
std::thread gWorker;
bool gRunning = false;
bool gHasPending = false;
Frame gPending;
int gProcessingBuffer = -1;
std::vector<float> gResult;
bool gHasResult = false;

// ---- Settings (written before the worker starts, read-only while it runs) ------------------------
cv::aruco::DetectorParameters gBaseParams;
cv::aruco::Dictionary gDictionary;
int gFullScanInterval = 6;
bool gAutoOrientation = true;

// ---- Worker-only state --------------------------------------------------------------------------
struct WorkerState {
    cv::aruco::ArucoDetector detector;
    cv::aruco::DetectorParameters roiParams;
    cv::Mat gray;
    cv::Mat grayTemp;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> roiCorners;
    std::vector<int> roiIds;
    std::vector<std::vector<cv::Point2f>> rejected;
    std::unordered_map<int, RotationMemory> rotations;
    cv::Rect roi;            // empty = no region to track
    int lastMarkerCount = 0;
    int framesSinceFullScan = 1 << 20;
    int emptyFrames = 0;
    int orientation = -1;    // -1 = use the flip Unity asked for; 0/1 = decided by auto orientation
    cv::Mat objectPoints;
    float objectPointsLength = -1.f;
};

void EnsureObjectPoints(WorkerState& s, float markerLength) {
    if (s.objectPointsLength == markerLength) return;
    const float half = markerLength * 0.5f;
    // Order required by SOLVEPNP_IPPE_SQUARE, matching ArUco corner order (TL, TR, BR, BL).
    const std::vector<cv::Point3f> points{{-half, half, 0}, {half, half, 0}, {half, -half, 0}, {-half, -half, 0}};
    cv::Mat(points, true).copyTo(s.objectPoints);
    s.objectPointsLength = markerLength;
}

// Converts the raw frame to grayscale in OpenCV's top-down orientation.
void PrepareGray(const Frame& f, bool flip, cv::Mat& out, cv::Mat& temp) {
    const int type = f.channels == 4 ? CV_8UC4 : CV_8UC1;
    cv::Mat src(f.height, f.width, type, const_cast<uint8_t*>(f.pixels), static_cast<size_t>(f.width) * f.channels);

    cv::Mat& target = f.divide > 1 ? temp : out;
    if (f.channels == 4) {
        cv::cvtColor(src, target, cv::COLOR_RGBA2GRAY);
        if (flip) cv::flip(target, target, 0);
    } else if (flip) {
        cv::flip(src, target, 0);
    } else {
        src.copyTo(target);
    }

    if (f.divide > 1) {
        cv::resize(temp, out, cv::Size(f.width / f.divide, f.height / f.divide), 0, 0, cv::INTER_AREA);
    }
}

// Runs detection on the whole image, or only inside `roi` (corners are returned in full-image coordinates).
void Detect(WorkerState& s, const cv::Mat& gray, const cv::Rect& roi) {
    s.corners.clear();
    s.ids.clear();
    s.rejected.clear();

    if (roi.empty()) {
        s.detector.setDetectorParameters(gBaseParams);
        s.detector.detectMarkers(gray, s.corners, s.ids, s.rejected);
        return;
    }

    // minMarkerPerimeterRate is relative to the input's largest side, so rescale it to keep the
    // same minimum marker size in pixels inside the smaller region.
    const double fullSide = std::max(gray.cols, gray.rows);
    const double roiSide = std::max(roi.width, roi.height);
    s.roiParams = gBaseParams;
    s.roiParams.minMarkerPerimeterRate = std::min(gBaseParams.minMarkerPerimeterRate * fullSide / roiSide, 3.9);
    s.roiParams.maxMarkerPerimeterRate = 4.0;
    s.detector.setDetectorParameters(s.roiParams);

    s.roiCorners.clear();
    s.roiIds.clear();
    s.detector.detectMarkers(gray(roi), s.roiCorners, s.roiIds, s.rejected);

    const cv::Point2f offset(static_cast<float>(roi.x), static_cast<float>(roi.y));
    for (auto& quad : s.roiCorners) {
        for (auto& p : quad) p += offset;
    }
    s.corners.swap(s.roiCorners);
    s.ids.swap(s.roiIds);
}

// Region around all detected markers, generously padded so they are still inside next frame.
cv::Rect ComputeTrackingRegion(const WorkerState& s, const cv::Size& imageSize) {
    if (s.corners.empty()) return cv::Rect();

    float minX = 1e9f, minY = 1e9f, maxX = -1e9f, maxY = -1e9f, largestSide = 0.f;
    for (const auto& quad : s.corners) {
        float qMinX = 1e9f, qMinY = 1e9f, qMaxX = -1e9f, qMaxY = -1e9f;
        for (const auto& p : quad) {
            qMinX = std::min(qMinX, p.x); qMaxX = std::max(qMaxX, p.x);
            qMinY = std::min(qMinY, p.y); qMaxY = std::max(qMaxY, p.y);
        }
        largestSide = std::max(largestSide, std::max(qMaxX - qMinX, qMaxY - qMinY));
        minX = std::min(minX, qMinX); maxX = std::max(maxX, qMaxX);
        minY = std::min(minY, qMinY); maxY = std::max(maxY, qMaxY);
    }

    // Covers fast head turns at 30 fps (~50 px per frame at 1280 px) plus the marker's own size.
    const float margin = std::max(imageSize.width * 0.08f, largestSide * 1.5f);
    cv::Rect region(cv::Point(static_cast<int>(std::floor(minX - margin)), static_cast<int>(std::floor(minY - margin))),
                    cv::Point(static_cast<int>(std::ceil(maxX + margin)), static_cast<int>(std::ceil(maxY + margin))));
    region &= cv::Rect(0, 0, imageSize.width, imageSize.height);

    // Not worth it when the region covers most of the image anyway.
    if (region.area() > imageSize.area() * 0.6) return cv::Rect();
    return region;
}

double RotationDifference(const cv::Matx33d& a, const cv::Matx33d& b) {
    const cv::Matx33d d = a.t() * b;
    const double c = std::max(-1.0, std::min(1.0, (cv::trace(d) - 1.0) * 0.5));
    return std::acos(c);
}

std::vector<float> Process(WorkerState& s, const Frame& f) {
    const auto start = std::chrono::steady_clock::now();

    bool flip = s.orientation >= 0 ? s.orientation == 1 : f.flip != 0;
    PrepareGray(f, flip, s.gray, s.grayTemp);

    // Region tracking, with a full scan every gFullScanInterval frames.
    const bool useRoi = gFullScanInterval > 1 && !s.roi.empty() && s.framesSinceFullScan < gFullScanInterval - 1;
    if (useRoi) {
        Detect(s, s.gray, s.roi);
        s.framesSinceFullScan++;
        if (s.ids.empty()) {
            // Lost everything inside the region: search the whole frame right away.
            Detect(s, s.gray, cv::Rect());
            s.framesSinceFullScan = 0;
        } else if (static_cast<int>(s.ids.size()) < s.lastMarkerCount) {
            // Lost some markers: do a full scan on the next frame.
            s.framesSinceFullScan = gFullScanInterval;
        }
    } else {
        Detect(s, s.gray, cv::Rect());
        s.framesSinceFullScan = 0;
    }

    // Auto orientation: after a run of empty frames, try the image the other way up once.
    if (s.ids.empty() && gAutoOrientation) {
        if (++s.emptyFrames >= kOrientationProbeInterval) {
            s.emptyFrames = 0;
            PrepareGray(f, !flip, s.gray, s.grayTemp);
            Detect(s, s.gray, cv::Rect());
            if (!s.ids.empty()) {
                flip = !flip;
                s.orientation = flip ? 1 : 0;
                s.rotations.clear();
            }
        }
    }
    if (!s.ids.empty()) s.emptyFrames = 0;

    s.lastMarkerCount = static_cast<int>(s.ids.size());
    s.roi = ComputeTrackingRegion(s, s.gray.size());

    // ---- Pose estimation --------------------------------------------------------------------------
    EnsureObjectPoints(s, f.markerLength);
    const cv::Matx33d cameraMatrix(f.fx, 0, f.cx, 0, f.fy, f.cy, 0, 0, 1);

    const int count = static_cast<int>(std::min(s.ids.size(), s.corners.size()));
    std::vector<float> out(kHeaderSize + static_cast<size_t>(count) * kValuesPerMarker);
    int written = 0;

    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat errors, errors64;
    cv::Mat r64[2], t64;
    for (int i = 0; i < count; i++) {
        rvecs.clear();
        tvecs.clear();
        const cv::Mat imagePoints(s.corners[i], false);
        int solutions = 0;
        try {
            solutions = cv::solvePnPGeneric(s.objectPoints, imagePoints.reshape(2, 4), cameraMatrix, cv::noArray(),
                                            rvecs, tvecs, false, cv::SOLVEPNP_IPPE_SQUARE,
                                            cv::noArray(), cv::noArray(), errors);
        } catch (const cv::Exception&) {
            continue;
        }
        if (solutions <= 0 || rvecs.empty() || tvecs.empty()) continue;
        solutions = std::min<int>(solutions, static_cast<int>(std::min(rvecs.size(), tvecs.size())));

        // OpenCV returns float or double depending on the inputs; work in double.
        rvecs[0].convertTo(r64[0], CV_64F);
        if (solutions > 1) rvecs[1].convertTo(r64[1], CV_64F);

        const int id = s.ids[i];
        int chosen = 0;
        cv::Matx33d chosenRotation;
        cv::Rodrigues(r64[0], chosenRotation);

        if (solutions > 1 && errors.total() >= 2) {
            errors.convertTo(errors64, CV_64F);
            const double e0 = errors64.at<double>(0);
            const double e1 = errors64.at<double>(1);
            const bool ambiguous = e1 < e0 * kAmbiguityRatio + 1e-3;
            auto memory = s.rotations.find(id);
            if (ambiguous && memory != s.rotations.end() && f.frameId - memory->second.frameId <= kRotationMemoryFrames) {
                cv::Matx33d second;
                cv::Rodrigues(r64[1], second);
                if (RotationDifference(memory->second.rotation, second) <
                    RotationDifference(memory->second.rotation, chosenRotation)) {
                    chosen = 1;
                    chosenRotation = second;
                }
            }
        }
        s.rotations[id] = RotationMemory{chosenRotation, f.frameId};

        tvecs[chosen].convertTo(t64, CV_64F);
        const double* r = r64[chosen].ptr<double>();
        const double* t = t64.ptr<double>();
        const size_t o = kHeaderSize + static_cast<size_t>(written) * kValuesPerMarker;
        out[o] = static_cast<float>(id);
        out[o + 1] = static_cast<float>(t[0]);
        out[o + 2] = static_cast<float>(t[1]);
        out[o + 3] = static_cast<float>(t[2]);
        out[o + 4] = static_cast<float>(r[0]);
        out[o + 5] = static_cast<float>(r[1]);
        out[o + 6] = static_cast<float>(r[2]);
        written++;
    }

    out.resize(kHeaderSize + static_cast<size_t>(written) * kValuesPerMarker);
    out[0] = static_cast<float>(f.frameId);
    out[1] = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - start).count();
    out[2] = static_cast<float>(written);
    return out;
}

void WorkerLoop() {
    // Slightly above normal so detection finishes quickly; Unity's render thread stays higher.
    setpriority(PRIO_PROCESS, static_cast<id_t>(gettid()), -2);

    WorkerState state;
    state.detector = cv::aruco::ArucoDetector(gDictionary, gBaseParams);

    while (true) {
        Frame frame;
        {
            std::unique_lock<std::mutex> lock(gMutex);
            gWake.wait(lock, [] { return !gRunning || gHasPending; });
            if (!gRunning) return;
            frame = gPending;
            gHasPending = false;
            gProcessingBuffer = frame.bufferIndex;
        }

        std::vector<float> result;
        try {
            result = Process(state, frame);
        } catch (...) {
            result = {static_cast<float>(frame.frameId), 0.f, 0.f};
        }

        {
            std::lock_guard<std::mutex> lock(gMutex);
            gProcessingBuffer = -1;
            gResult.swap(result);
            gHasResult = true;
        }
    }
}

void StopWorker() {
    {
        std::lock_guard<std::mutex> lock(gMutex);
        gRunning = false;
        gHasPending = false;
    }
    gWake.notify_all();
    if (gWorker.joinable()) gWorker.join();
    std::lock_guard<std::mutex> lock(gMutex);
    gProcessingBuffer = -1;
    gHasResult = false;
    gResult.clear();
}

}  // namespace

// Creates the detector and starts the worker thread. Calling it again restarts with new settings.
ARUCO_EXPORT int aruco_initialize(int dictionaryId, float minMarkerPerimeterRate, int cornerRefinement,
                                  int numThreads, int fullScanInterval, int autoOrientation) {
    if (dictionaryId < cv::aruco::DICT_4X4_50 || dictionaryId > cv::aruco::DICT_4X4_1000) return 0;

    StopWorker();

    try {
        cv::setNumThreads(std::max(1, numThreads));

        cv::aruco::DetectorParameters p;
        // Two adaptive-threshold passes (window 7 and 17) instead of three (3, 13, 23):
        // ~1.5-2x faster with the same detection rate for small markers.
        p.adaptiveThreshWinSizeMin = 7;
        p.adaptiveThreshWinSizeMax = 17;
        p.adaptiveThreshWinSizeStep = 10;
        p.minMarkerPerimeterRate = minMarkerPerimeterRate;
        p.minDistanceToBorder = 3;
        p.cornerRefinementMethod = cornerRefinement ? cv::aruco::CORNER_REFINE_SUBPIX : cv::aruco::CORNER_REFINE_NONE;
        p.cornerRefinementWinSize = 3;
        p.useAruco3Detection = false;  // downsamples internally and loses small markers

        gBaseParams = p;
        gDictionary = cv::aruco::getPredefinedDictionary(dictionaryId);
        gFullScanInterval = std::max(1, fullScanInterval);
        gAutoOrientation = autoOrientation != 0;
    } catch (...) {
        return 0;
    }

    {
        std::lock_guard<std::mutex> lock(gMutex);
        gRunning = true;
        gHasPending = false;
        gHasResult = false;
    }
    gWorker = std::thread(WorkerLoop);
    return 1;
}

// Stops the worker. When this returns, none of Unity's buffers are in use any more.
ARUCO_EXPORT void aruco_shutdown() {
    StopWorker();
}

// Queues a frame. `pixels` must stay valid until aruco_is_buffer_in_use(bufferIndex) returns 0.
ARUCO_EXPORT int aruco_submit(const void* pixels, int width, int height, int channels, int flipVertically,
                              int divide, float fx, float fy, float cx, float cy, float markerLength,
                              int frameId, int bufferIndex) {
    if (pixels == nullptr || width <= 0 || height <= 0 || (channels != 1 && channels != 4) ||
        fx <= 0 || fy <= 0 || markerLength <= 0) {
        return 0;
    }

    {
        std::lock_guard<std::mutex> lock(gMutex);
        if (!gRunning) return 0;
        Frame& f = gPending;
        f.pixels = static_cast<const uint8_t*>(pixels);
        f.width = width;
        f.height = height;
        f.channels = channels;
        f.flip = flipVertically;
        f.divide = std::max(1, divide);
        f.fx = fx;
        f.fy = fy;
        f.cx = cx;
        f.cy = cy;
        f.markerLength = markerLength;
        f.frameId = frameId;
        f.bufferIndex = bufferIndex;
        gHasPending = true;  // replaces any frame still waiting
    }
    gWake.notify_one();
    return 1;
}

// 1 if the worker is processing, or still has queued, the frame stored in this buffer.
ARUCO_EXPORT int aruco_is_buffer_in_use(int bufferIndex) {
    std::lock_guard<std::mutex> lock(gMutex);
    return (gProcessingBuffer == bufferIndex || (gHasPending && gPending.bufferIndex == bufferIndex)) ? 1 : 0;
}

// Copies the newest result into `out`. Returns the number of floats written, or 0 if there is none.
ARUCO_EXPORT int aruco_take_result(float* out, int capacity) {
    std::lock_guard<std::mutex> lock(gMutex);
    if (!gHasResult || out == nullptr || capacity < kHeaderSize) return 0;
    gHasResult = false;

    int markers = gResult.size() >= kHeaderSize ? static_cast<int>(gResult[2]) : 0;
    markers = std::min(markers, (capacity - kHeaderSize) / kValuesPerMarker);
    const int n = kHeaderSize + markers * kValuesPerMarker;
    std::copy(gResult.begin(), gResult.begin() + std::min<size_t>(n, gResult.size()), out);
    out[2] = static_cast<float>(markers);
    return n;
}
