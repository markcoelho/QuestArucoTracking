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

1. Open **Unity Hub → Add → Add project from disk** and select this `QuestArucoTracking` folder.
2. Open the project. The first time takes a few minutes while Unity imports everything.
3. Go to **File → Build Profiles**, select **Android**, and click **Switch Platform** (skip this if Android is already active).
4. In the top menu, click **Build → Build Quest ARM64 APK**.
5. When it finishes, the app is at:

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

## Troubleshooting

| Problem | Fix |
| --- | --- |
| SideQuest dot stays red/orange | Re-plug the cable, put the headset on and accept the USB debugging prompt. Check that Developer Mode is on. |
| App not in Library | Set the Library filter to **Unknown Sources**. |
| No markers detected | Make sure you allowed camera access (Settings → Apps → Permissions), the markers are from the 4x4 dictionary, and the room is well lit. |
| Marker position/size is off | Set **Marker Length** on the `ArUcoMarkerTracking` component to your printed marker's side length in meters (2 cm = `0.02`), then rebuild. |
| Build menu missing | Wait for Unity to finish compiling scripts, or check the Console for errors. |
