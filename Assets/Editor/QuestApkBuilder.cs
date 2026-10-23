using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

public static class QuestApkBuilder
{
    [MenuItem("Build/Build Quest ARM64 APK")]
    public static void BuildQuestApk()
    {
        if (EditorUserBuildSettings.activeBuildTarget != BuildTarget.Android)
            throw new InvalidOperationException("Switch the active build platform to Android before building the Quest APK.");

        string[] scenes = EditorBuildSettings.scenes
            .Where(scene => scene.enabled)
            .Select(scene => scene.path)
            .ToArray();
        if (scenes.Length == 0)
            throw new InvalidOperationException("Enable the ArUco tracking scene in Build Settings before building.");

        PlayerSettings.Android.targetArchitectures = AndroidArchitecture.ARM64;

        string projectDirectory = Directory.GetParent(Application.dataPath).FullName;
        string outputDirectory = Path.Combine(projectDirectory, "Builds");
        Directory.CreateDirectory(outputDirectory);
        string outputPath = Path.Combine(outputDirectory, "ArucoTracker.apk");

        BuildReport report = BuildPipeline.BuildPlayer(scenes, outputPath, BuildTarget.Android, BuildOptions.None);
        if (report.summary.result != BuildResult.Succeeded)
            throw new InvalidOperationException($"Quest APK build failed: {report.summary.result}.");

        Debug.Log($"Quest APK built: {outputPath}");
    }
}
