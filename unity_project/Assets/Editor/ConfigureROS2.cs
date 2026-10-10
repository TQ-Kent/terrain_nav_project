using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

namespace TerrainNavigation.EditorTools
{
    public class ConfigureROS2
    {
        [MenuItem("TerrainNav/Configure ROS 2")]
        public static void SetROS2DefineSymbolsAndPrefab()
        {
            var buildTargetGroup = BuildTargetGroup.Standalone;
            string currentDefines = PlayerSettings.GetScriptingDefineSymbolsForGroup(buildTargetGroup);
            List<string> definesList = currentDefines.Split(';').Select(d => d.Trim()).Where(d => !string.IsNullOrEmpty(d)).ToList();

            if (!definesList.Contains("ROS2"))
            {
                definesList.Add("ROS2");
                string newDefines = string.Join(";", definesList);
                PlayerSettings.SetScriptingDefineSymbolsForGroup(buildTargetGroup, newDefines);
                Debug.Log($"Added ROS2 scripting define symbol: '{newDefines}'");
            }
            else
            {
                Debug.Log("ROS2 scripting define symbol already exists.");
            }

            // Create Assets/Resources/ROSConnectionPrefab.prefab
            if (!Directory.Exists("Assets/Resources"))
            {
                Directory.CreateDirectory("Assets/Resources");
            }

            GameObject sceneObj = new GameObject("ROSConnection");
            ROSConnection rosConn = sceneObj.AddComponent<ROSConnection>();
            rosConn.RosIPAddress = "127.0.0.1";
            rosConn.RosPort = 10000;
            rosConn.ConnectOnStart = true;

            PrefabUtility.SaveAsPrefabAsset(sceneObj, "Assets/Resources/ROSConnectionPrefab.prefab");
            Object.DestroyImmediate(sceneObj);
            Debug.Log("Saved Assets/Resources/ROSConnectionPrefab.prefab successfully.");
        }
    }
}
