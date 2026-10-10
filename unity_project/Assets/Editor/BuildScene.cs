using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using TerrainNavigation.Network;

namespace TerrainNavigation.EditorTools
{
    public class BuildScene
    {
        [MenuItem("TerrainNav/Build Main Scene")]
        public static void CreateMainScene()
        {
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // Directional Light
            var lightGo = new GameObject("Directional Light");
            var light = lightGo.AddComponent<Light>();
            light.type = LightType.Directional;
            lightGo.transform.rotation = Quaternion.Euler(50, -30, 0);

            // Camera
            var cameraGo = new GameObject("Main Camera");
            var camera = cameraGo.AddComponent<Camera>();
            cameraGo.tag = "MainCamera";
            cameraGo.transform.position = new Vector3(0, 5, -10);
            cameraGo.transform.LookAt(Vector3.zero);

            // Ground Plane
            var plane = GameObject.CreatePrimitive(PrimitiveType.Plane);
            plane.name = "Ground";
            plane.transform.localScale = new Vector3(5, 1, 5);

            // TurtleBot3 (includes AGVController, LaserScan, TF and clock publishers)
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/TurtleBot3ManualConfig.prefab");
            if (prefab != null)
            {
                var robot = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
                robot.transform.position = new Vector3(0, 0.05f, 0);
            }
            else
            {
                Debug.LogWarning("TurtleBot3ManualConfig.prefab not found");
            }

            // ROS Test Publisher GameObject
            var rosTestGo = new GameObject("ROSTestPublisher");
            rosTestGo.AddComponent<ROSTestPublisher>();

            // Save scene
            if (!AssetDatabase.IsValidFolder("Assets/Scenes"))
            {
                AssetDatabase.CreateFolder("Assets", "Scenes");
            }
            EditorSceneManager.SaveScene(scene, "Assets/Scenes/MainScene.unity");

            // Set as active scene in Build Settings
            EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene("Assets/Scenes/MainScene.unity", true) };
            Debug.Log("MainScene created and set in Build Settings successfully!");
        }
    }
}
