# Quest ArUco Tracking

Tracks multiple ArUco markers (4x4 dictionary) on Meta Quest 3 / 3S in passthrough mode, using Meta's Passthrough Camera API and the OpenCV Android SDK.

---

## 1. What you need

- **Unity 6000.3.26f1** (Unity 6.3) with **Android Build Support** installed
  (in Unity Hub: *Installs → ⚙ → Add modules → Android Build Support*, including OpenJDK and Android SDK & NDK)
- A **Meta Quest 3 or 3S**
- A **USB-C cable** to connect the Quest to your PC
- **SideQuest** (Advanced Installer) from [sidequestvr.com/setup-howto](https://sidequestvr.com/setup-howto)
- Printed **4x4 ArUco markers**. The scene expects 2 cm markers (`DICT_4X4_50`). You can generate them at [chev.me/arucogen](https://chev.me/arucogen/).

Everything else (Meta XR SDK, OpenCV) is already included in the project.

---

## 2. Build the APK

1. Put the project in a folder with a **short path**, for example `C:\Projects\QuestArucoTracking`. Very long paths make the Android build fail on Windows.
2. Open **Unity Hub → Add → Add project from disk** and select this `QuestArucoTracking` folder.
3. Open the project. The first time takes a few minutes while Unity imports everything.
4. Go to **File → Build Profiles**, select **Android**, and click **Switch Platform** (skip this if Android is already active).
5. In the top menu, click **Build → Build Quest ARM64 APK**.
6. When it finishes, the app is at:

   ```
   QuestArucoTracking/Builds/ArucoTracker.apk
   ```

---

## 3. Install on Quest 3 with SideQuest

### One-time setup: enable Developer Mode

1. Install the **Meta Horizon** app on your phone and sign in with the account the Quest uses.
2. Create a developer organization (free) at [developers.meta.com](https://developers.meta.com/horizon/manage/organizations/create/) if you haven't already.
3. In the phone app: **Devices → your Quest → Headset settings → Developer Mode → On**.
4. Restart the Quest.

### Install the app

1. Open **SideQuest** on your PC.
2. Connect the Quest to the PC with the USB cable.
3. Put on the headset and select **Allow** on the *"Allow USB debugging?"* prompt (tick *Always allow from this computer*).
4. In SideQuest, the dot in the top-left should turn **green** (connected).
5. **Drag and drop `ArucoTracker.apk`** onto the SideQuest window.
   (Or click the *Install APK file from folder* icon at the top and select the APK.)
6. Wait for *"Installed"* in SideQuest's task list.

### Run it

1. On the Quest, open the **Library** and change the filter to **Unknown Sources**.
2. Launch **Aruco Tracker**.
3. Select **Allow** when it asks for camera access (needed to see the markers).
4. Look at a printed marker. An outline with its ID appears on top of it.

**Controls:** press **A** on the right controller to toggle the camera image view.

---

## Updating the app

Build again and drag the new APK into SideQuest. It replaces the old version.

## How tracking works

1. Each new camera image is copied from the GPU into a reusable buffer (`AsyncGPUReadback`).
2. A native C++ plugin (`libarucobridge.so`, built on OpenCV) reads that buffer directly and detects markers on its own thread. Once markers are found, it only searches the area around them, with a full scan every few frames to find new ones.
3. Poses are placed in the world using the headset pose at the exact moment the image was taken.
4. Every frame, each marker's pose is predicted forward to the moment the frame is shown. This hides the camera's built-in delay, so moving markers don't trail behind.

## Tuning

Select `ArUcoTrackingAppCoordinator` in the scene and change these on the `ArUcoMarkerTracking` component:

| Setting | What it does |
| --- | --- |
| **Full Scan Interval** | How often the whole image is searched for new markers. Lower = new markers appear sooner, higher = faster tracking. |
| **Predict Motion** | Predicts where moving markers are right now. Turn off to see the raw (delayed) tracking. |
| **Prediction Strength** | How much of the camera delay to predict away (default 0.5). Higher = less drag on smooth moves, lower = calmer on fast shakes and sudden stops. |
| **Max Prediction Distance / Angle** | The furthest a marker may be moved or rotated ahead of its last detection. Lower them if fast moves still overshoot. |
| **Velocity Responsiveness** | How quickly the speed estimate reacts. Lower = calmer during jerky motion, higher = prediction starts sooner. |
| **Max Prediction** | How far ahead (seconds) it may predict at most. |
| **Position Noise / Rotation Noise** | Movement smaller than this counts as jitter and is smoothed out. Raise them if still markers shake, lower them if small movements feel sluggish. |
| **Still Responsiveness** | How steady a still marker is (lower = steadier). |

## Changing the native plugin

The built plugin is already in `Assets/Plugins/Android/arm64-v8a`. Rebuilding is only needed if you change `NativePlugin/arucobridge.cpp`. Run `NativePlugin/build.bat`. It uses the NDK that comes with Unity and the C++ headers from the [OpenCV 5 Android SDK](https://opencv.org/releases/) (by default it looks for `OpenCV-android-sdk` next to this folder, or set `OPENCV_SDK`).

## Troubleshooting

| Problem | Fix |
| --- | --- |
| SideQuest dot stays red/orange | Re-plug the cable, put the headset on and accept the USB debugging prompt. Check that Developer Mode is on. |
| App not in Library | Set the Library filter to **Unknown Sources**. |
| No markers detected | Make sure you allowed camera access (Settings → Apps → Permissions), the markers are from the 4x4 dictionary, and the room is well lit. |
| Marker position/size is off | Set **Marker Length** on the `ArUcoMarkerTracking` component to your printed marker's side length in meters (2 cm = `0.02`), then rebuild. |
| Build fails with "Cannot run program ... .bat" | The folder path is too long. Move the project to a short path like `C:\Projects\QuestArucoTracking`. |
| Build menu missing | Wait for Unity to finish compiling scripts, or check the Console for errors. |
