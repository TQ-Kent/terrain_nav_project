using UnityEngine;

namespace TerrainNavigation.Tools
{
    /// <summary>
    /// Maze run timer and path trace. Put it on the robot. The timer starts when the robot first moves
    /// and stops when it is within goalTolerance (m, horizontal) of the goal; a line is drawn along the
    /// robot's path. Press R to clear the trace and reset the timer.
    /// </summary>
    public class RunTracker : MonoBehaviour
    {
        public Transform goal;                  // defaults to the object named "GoalMarker"
        public float goalTolerance = 0.25f;     // same as tolerance.xy in maze_test.yaml
        public float startMoveDistance = 0.05f; // movement that starts the timer
        public float sampleDistance = 0.05f;    // minimum spacing of trace points
        public float lineWidth = 0.06f;
        public Color lineColor = Color.red;

        Transform tracked;                      // the transform that really moves (see Start)
        LineRenderer line;
        Vector3 startPos, lastPoint;
        float startTime, elapsed, pathLength;
        bool running, finished;

        void Start()
        {
            // The robot is an ArticulationBody chain: only the articulation root's transform moves, and it can
            // be a child of the object this script sits on. Track that one, else the timer never starts.
            tracked = transform;
            foreach (var b in GetComponentsInChildren<ArticulationBody>())
                if (b.isRoot) { tracked = b.transform; break; }
            Debug.Log($"RunTracker: tracking '{tracked.name}'");

            if (goal == null)
            {
                var g = GameObject.Find("GoalMarker");
                if (g != null) goal = g.transform;
                else Debug.LogWarning("RunTracker: no goal assigned and no GoalMarker in the scene; timer will not stop.");
            }

            // Separate object, so the trace stays in world space and does not move with the robot.
            var go = new GameObject("PathTrace");
            line = go.AddComponent<LineRenderer>();
            line.useWorldSpace = true;
            line.widthMultiplier = lineWidth;
            line.numCornerVertices = 2;
            line.material = new Material(Shader.Find("Sprites/Default"));
            line.startColor = line.endColor = lineColor;
            line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            ResetRun();
        }

        void ResetRun()
        {
            running = finished = false;
            elapsed = pathLength = 0f;
            startPos = lastPoint = tracked.position;
            line.positionCount = 0;
            AddPoint(tracked.position);
        }

        void AddPoint(Vector3 p)
        {
            p.y = 0.08f; // just above the floor; the walls are 1 m tall, so the line is visible from above
            line.positionCount++;
            line.SetPosition(line.positionCount - 1, p);
        }

        void Update()
        {
            if (Input.GetKeyDown(KeyCode.R)) ResetRun();
            if (finished) return;

            Vector3 pos = tracked.position;
            if (!running && (pos - startPos).magnitude > startMoveDistance)
            {
                running = true;
                startTime = Time.time;
            }
            if (!running) return;

            elapsed = Time.time - startTime;
            if ((pos - lastPoint).magnitude >= sampleDistance)
            {
                pathLength += (pos - lastPoint).magnitude;
                lastPoint = pos;
                AddPoint(pos);
            }

            if (goal != null)
            {
                var d = pos - goal.position;
                d.y = 0;
                if (d.magnitude <= goalTolerance)
                {
                    finished = true;
                    AddPoint(pos);
                    Debug.Log($"RunTracker: goal reached in {elapsed:F2} s, path {pathLength:F2} m");
                }
            }
        }

        // Styled like the ROS-TCP-Connector HUD: GUI.skin.box, 300 px wide, top-left, placed directly under it.
        const float PanelWidth = 300f;
        const float PanelTop = 28f;      // height of the ROS HUD header; raise it if the two boxes overlap

        void OnGUI()
        {
            string state = finished ? "GOAL REACHED" : running ? "RUNNING" : "WAITING FOR MOVEMENT";
            var bold = new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold };
            var status = new GUIStyle(bold);
            status.normal.textColor = finished ? Color.green : running ? Color.yellow : Color.white;

            GUILayout.BeginArea(new Rect(0, PanelTop, PanelWidth, 200));
            GUILayout.BeginVertical(GUI.skin.box, GUILayout.Width(PanelWidth));
            GUILayout.Label($"Time: {elapsed:F2} s   |   Path: {pathLength:F1} m", bold);
            GUILayout.Label($"{state}   (R = Reset)", status);
            GUILayout.EndVertical();
            GUILayout.EndArea();
        }
    }
}
