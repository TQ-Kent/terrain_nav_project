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
            BuildBase(new Vector3(0, 0.05f, 0), new Vector3(5, 1, 5), new Vector3(0, 5, -10));
            SaveScene("Assets/Scenes/MainScene.unity");
        }

        // Light, camera, ground, robot and ROS publisher shared by every scene. Starts a new scene.
        public static void BuildBase(Vector3 robotPosition, Vector3 groundScale, Vector3 cameraPosition)
        {
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // Directional Light
            var lightGo = new GameObject("Directional Light");
            var light = lightGo.AddComponent<Light>();
            light.type = LightType.Directional;
            lightGo.transform.rotation = Quaternion.Euler(50, -30, 0);

            // Camera
            var cameraGo = new GameObject("Main Camera");
            var camera = cameraGo.AddComponent<Camera>();
            cameraGo.tag = "MainCamera";
            cameraGo.transform.position = cameraPosition;
            cameraGo.transform.LookAt(Vector3.zero);

            // Ground Plane
            var plane = GameObject.CreatePrimitive(PrimitiveType.Plane);
            plane.name = "Ground";
            plane.transform.localScale = groundScale;

            // TurtleBot3 (includes AGVController, LaserScan, TF and clock publishers)
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/TurtleBot3ManualConfig.prefab");
            if (prefab != null)
            {
                var robot = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
                robot.transform.position = robotPosition;
            }
            else
            {
                Debug.LogWarning("TurtleBot3ManualConfig.prefab not found");
            }

            // ROS Test Publisher GameObject
            var rosTestGo = new GameObject("ROSTestPublisher");
            rosTestGo.AddComponent<ROSTestPublisher>();
        }

        public static void SaveScene(string path, bool setAsBuildScene = true)
        {
            if (!AssetDatabase.IsValidFolder("Assets/Scenes"))
            {
                AssetDatabase.CreateFolder("Assets", "Scenes");
            }
            EditorSceneManager.SaveScene(EditorSceneManager.GetActiveScene(), path);

            // Set as active scene in Build Settings
            if (setAsBuildScene)
                EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(path, true) };
            Debug.Log(path + (setAsBuildScene ? " created and set in Build Settings successfully!" : " created (Build Settings unchanged)."));
        }
    }
}
