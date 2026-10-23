package com.questmarkertracking;

import android.util.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.geometry.Geometry;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.ArucoDetector;
import org.opencv.objdetect.DetectorParameters;
import org.opencv.objdetect.Dictionary;
import org.opencv.objdetect.Objdetect;
import org.opencv.objdetect.RefineParameters;

/**
 * ArUco detection + pose estimation on a dedicated background thread.
 *
 * Unity (main thread) calls submitFrame() with a grayscale (or RGBA) frame; the worker thread
 * runs detection and pose estimation; Unity polls hasResult()/takeResult() each frame.
 *
 * Result layout: [frameId, detectionMs, markerCount, (id, tx, ty, tz, rx, ry, rz) * markerCount]
 * Translation and Rodrigues rotation are in OpenCV camera space (x right, y down, z forward).
 */
public final class ArucoTrackingBridge {
    private static final String TAG = "ArucoTrackingBridge";
    private static final int HEADER_SIZE = 3;
    private static final int VALUES_PER_MARKER = 7;

    static {
        System.loadLibrary("opencv_java5");
    }

    private static final class Frame {
        byte[] pixels;
        int width;
        int height;
        int channels;
        boolean flip;
        float fx;
        float fy;
        float cx;
        float cy;
        float markerLength;
        int frameId;
    }

    // ---- Shared state, guarded by LOCK -------------------------------------------------------
    private static final Object LOCK = new Object();
    private static ArucoDetector detector;
    private static Thread worker;
    private static Frame pending;
    private static float[] latestResult;

    // ---- Worker-thread-only state ------------------------------------------------------------
    private static final Mat input = new Mat();
    private static final Mat gray = new Mat();
    private static final Mat ids = new Mat();
    private static final Mat cameraMatrix = Mat.zeros(3, 3, CvType.CV_64FC1);
    private static final MatOfDouble distortion = new MatOfDouble(0, 0, 0, 0);
    private static final MatOfPoint2f imagePoints = new MatOfPoint2f();
    private static final Mat rvec = new Mat();
    private static final Mat tvec = new Mat();
    private static final List<Mat> corners = new ArrayList<>();
    private static final float[] cornerValues = new float[8];
    private static final double[] rvecValues = new double[3];
    private static final double[] tvecValues = new double[3];
    private static MatOfPoint3f objectPoints;
    private static float objectPointsLength = -1f;

    private ArucoTrackingBridge() {}

    /**
     * Creates the detector and starts the worker thread. Safe to call again to change settings.
     */
    public static void initialize(int dictionaryId, float minMarkerPerimeterRate, boolean cornerRefinement) {
        if (dictionaryId < Objdetect.DICT_4X4_50 || dictionaryId > Objdetect.DICT_4X4_1000) {
            throw new IllegalArgumentException("Only the DICT_4X4_* dictionaries are supported.");
        }

        Dictionary dictionary = Objdetect.getPredefinedDictionary(dictionaryId);
        DetectorParameters parameters = new DetectorParameters();
        // Two adaptive-threshold passes (window 7 and 17) instead of the default three (3, 13, 23).
        // In benchmarks this is ~1.5-2x faster with the same detection rate for small markers.
        parameters.set_adaptiveThreshWinSizeMin(7);
        parameters.set_adaptiveThreshWinSizeMax(17);
        parameters.set_adaptiveThreshWinSizeStep(10);
        // Smallest marker perimeter as a fraction of the largest image side.
        // 0.04 at 1280 px = ~13 px per marker side.
        parameters.set_minMarkerPerimeterRate(minMarkerPerimeterRate);
        parameters.set_minDistanceToBorder(3);
        parameters.set_cornerRefinementMethod(
                cornerRefinement ? Objdetect.CORNER_REFINE_SUBPIX : Objdetect.CORNER_REFINE_NONE);
        // The default 5 px window is wide compared to a ~13 px marker and can pull corners
        // toward neighbouring edges. OpenCV also shrinks the window for small markers on its own
        // (relativeCornerRefinmentWinSize); this only caps it for larger ones.
        parameters.set_cornerRefinementWinSize(3);
        // Aruco3 "fast" mode downsamples the image and loses small markers. Keep it off.
        parameters.set_useAruco3Detection(false);

        // OpenCV would otherwise spread work across every core, competing with Unity's threads.
        Core.setNumThreads(2);

        ArucoDetector newDetector = new ArucoDetector(dictionary, parameters, new RefineParameters(10f, 3f, true));

        synchronized (LOCK) {
            detector = newDetector;
            pending = null;
            latestResult = null;
            if (worker == null) {
                Thread thread = new Thread(new Runnable() {
                    @Override
                    public void run() {
                        runWorker();
                    }
                }, "ArucoDetection");
                thread.setDaemon(true);
                worker = thread;
                thread.start();
            }
        }
    }

    /** Stops the worker thread. */
    public static void shutdown() {
        synchronized (LOCK) {
            worker = null;
            pending = null;
            latestResult = null;
            LOCK.notifyAll();
        }
    }

    /**
     * Queues a frame for detection. The worker always takes the newest frame: if a frame is already
     * waiting while the detector is busy, it is replaced (dropped) by this one, so the detector is
     * never idle and never works on a stale frame. Returns false only if the tracker isn't running.
     * pixels: tightly packed rows, 1 (gray) or 4 (RGBA) bytes per pixel.
     */
    public static boolean submitFrame(
            byte[] pixels,
            int width,
            int height,
            int channels,
            boolean flipVertically,
            float fx,
            float fy,
            float cx,
            float cy,
            float markerLength,
            int frameId) {
        if (pixels == null || width <= 0 || height <= 0 || (channels != 1 && channels != 4)
                || pixels.length != (long) width * height * channels) {
            throw new IllegalArgumentException("Frame must contain width*height*channels bytes (channels = 1 or 4).");
        }
        if (fx <= 0 || fy <= 0 || markerLength <= 0) {
            throw new IllegalArgumentException("Camera focal lengths and marker length must be positive.");
        }

        synchronized (LOCK) {
            if (worker == null || detector == null) {
                return false;
            }
            Frame frame = new Frame();
            frame.pixels = pixels; // Unity hands us a fresh Java array per call, so no copy is needed.
            frame.width = width;
            frame.height = height;
            frame.channels = channels;
            frame.flip = flipVertically;
            frame.fx = fx;
            frame.fy = fy;
            frame.cx = cx;
            frame.cy = cy;
            frame.markerLength = markerLength;
            frame.frameId = frameId;
            pending = frame;
            LOCK.notifyAll();
            return true;
        }
    }

    public static boolean hasResult() {
        synchronized (LOCK) {
            return latestResult != null;
        }
    }

    public static float[] takeResult() {
        synchronized (LOCK) {
            float[] result = latestResult;
            latestResult = null;
            return result != null ? result : new float[0];
        }
    }

    private static void runWorker() {
        // Normal priority. THREAD_PRIORITY_BACKGROUND puts the thread in Android's background group,
        // which often runs it on the slow cores and makes every detection take longer. Unity's
        // main and render threads run at a higher priority than this, so rendering still wins.
        android.os.Process.setThreadPriority(android.os.Process.THREAD_PRIORITY_DEFAULT);
        Thread self = Thread.currentThread();
        while (true) {
            Frame frame;
            ArucoDetector activeDetector;
            synchronized (LOCK) {
                while (worker == self && pending == null) {
                    try {
                        LOCK.wait();
                    } catch (InterruptedException e) {
                        return;
                    }
                }
                if (worker != self) {
                    return;
                }
                frame = pending;
                pending = null;
                activeDetector = detector;
            }

            float[] result;
            try {
                result = process(activeDetector, frame);
            } catch (Throwable t) {
                Log.e(TAG, "Marker detection failed", t);
                result = new float[] { frame.frameId, 0f, 0f };
            }

            synchronized (LOCK) {
                if (worker == self) {
                    latestResult = result;
                }
            }
        }
    }

    private static float[] process(ArucoDetector activeDetector, Frame frame) {
        long start = System.nanoTime();

        if (frame.channels == 1) {
            gray.create(frame.height, frame.width, CvType.CV_8UC1);
            gray.put(0, 0, frame.pixels);
        } else {
            input.create(frame.height, frame.width, CvType.CV_8UC4);
            input.put(0, 0, frame.pixels);
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGBA2GRAY);
        }
        if (frame.flip) {
            Core.flip(gray, gray, 0);
        }

        cameraMatrix.put(0, 0,
                frame.fx, 0, frame.cx,
                0, frame.fy, frame.cy,
                0, 0, 1);
        ensureObjectPoints(frame.markerLength);

        releaseCorners();
        try {
            activeDetector.detectMarkers(gray, corners, ids);

            int count = Math.min((int) ids.total(), corners.size());
            int[] idValues = new int[count];
            if (count > 0) {
                // OpenCV 5 returns ids as a 1-D Mat. Reading it with ids.get(i, 0) only works for i = 0,
                // which made every marker after the first come back as ID 0. Reshape to a count x 1
                // 2-D Mat and read all IDs at once.
                Mat idColumn = ids.reshape(1, count);
                idColumn.get(0, 0, idValues);
                idColumn.release();
            }

            float[] out = new float[HEADER_SIZE + count * VALUES_PER_MARKER];
            int written = 0;
            Point[] points = new Point[4];
            for (int i = 0; i < count; i++) {
                Mat corner = corners.get(i);
                Mat flatCorner = corner.reshape(1, 8);
                int read = flatCorner.get(0, 0, cornerValues);
                flatCorner.release();
                if (read < 8) {
                    continue;
                }

                for (int p = 0; p < 4; p++) {
                    points[p] = new Point(cornerValues[p * 2], cornerValues[p * 2 + 1]);
                }
                imagePoints.fromArray(points);

                // IPPE_SQUARE is designed for square markers: faster and more stable than ITERATIVE.
                if (!Geometry.solvePnP(objectPoints, imagePoints, cameraMatrix, distortion, rvec, tvec,
                        false, Geometry.SOLVEPNP_IPPE_SQUARE)) {
                    continue;
                }
                rvec.get(0, 0, rvecValues);
                tvec.get(0, 0, tvecValues);

                int offset = HEADER_SIZE + written * VALUES_PER_MARKER;
                out[offset] = idValues[i];
                out[offset + 1] = (float) tvecValues[0];
                out[offset + 2] = (float) tvecValues[1];
                out[offset + 3] = (float) tvecValues[2];
                out[offset + 4] = (float) rvecValues[0];
                out[offset + 5] = (float) rvecValues[1];
                out[offset + 6] = (float) rvecValues[2];
                written++;
            }

            if (written != count) {
                out = Arrays.copyOf(out, HEADER_SIZE + written * VALUES_PER_MARKER);
            }
            out[0] = frame.frameId;
            out[1] = (System.nanoTime() - start) / 1_000_000f;
            out[2] = written;
            return out;
        } finally {
            releaseCorners();
        }
    }

    private static void ensureObjectPoints(float markerLength) {
        if (objectPoints != null && objectPointsLength == markerLength) {
            return;
        }
        if (objectPoints != null) {
            objectPoints.release();
        }
        double half = markerLength / 2.0;
        // Order required by SOLVEPNP_IPPE_SQUARE, matching ArUco corner order (TL, TR, BR, BL).
        objectPoints = new MatOfPoint3f(
                new Point3(-half, half, 0),
                new Point3(half, half, 0),
                new Point3(half, -half, 0),
                new Point3(-half, -half, 0));
        objectPointsLength = markerLength;
    }

    private static void releaseCorners() {
        for (Mat corner : corners) {
            corner.release();
        }
        corners.clear();
    }
}
