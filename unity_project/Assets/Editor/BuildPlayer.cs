using UnityEditor;
using UnityEngine;

namespace TerrainNavigation.EditorTools
{
    public class BuildPlayer
    {
        [MenuItem("TerrainNav/Build Linux Standalone Player")]
        public static void PerformBuild()
        {
            BuildScene.CreateMainScene();

            string[] scenes = new[] { "Assets/Scenes/MainScene.unity" };
            string buildPath = "Build/UnityTerrainSim.x86_64";

            BuildPlayerOptions buildPlayerOptions = new BuildPlayerOptions
            {
                scenes = scenes,
                locationPathName = buildPath,
                target = BuildTarget.StandaloneLinux64,
                options = BuildOptions.None
            };

            var report = BuildPipeline.BuildPlayer(buildPlayerOptions);
            var summary = report.summary;

            if (summary.result == UnityEditor.Build.Reporting.BuildResult.Succeeded)
            {
                Debug.Log($"Build succeeded: {summary.totalSize} bytes");
            }
            else if (summary.result == UnityEditor.Build.Reporting.BuildResult.Failed)
            {
                Debug.LogError($"Build failed with {summary.totalErrors} errors");
            }
        }
    }
}
