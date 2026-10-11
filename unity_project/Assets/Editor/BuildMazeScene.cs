using System.Collections.Generic;
using System.Text;
using UnityEditor;
using UnityEngine;

namespace TerrainNavigation.EditorTools
{
    // Builds Assets/Scenes/MazeScene.unity: a fixed (seeded) maze for the algorithm comparison.
    // Same seed -> same maze, so every algorithm is tested on identical geometry.
    public static class BuildMazeScene
    {
        const int Cells = 6;            // maze is Cells x Cells
        const int CellChars = 4;        // 3 open characters + 1 wall character per cell
        const float Char = 0.5f;        // metres per character -> 1.5 m corridors, 0.5 m walls, 2 m pitch
        const float WallHeight = 1.0f;
        const int Seed = 42;
        const int ExtraOpenings = 4;    // knocked-out walls so the maze has loops (alternative routes)

        [MenuItem("TerrainNav/Build Maze Scene")]
        public static void CreateMazeScene()
        {
            int size = Cells * CellChars + 1;
            var wall = new bool[size, size];
            for (int x = 0; x < size; x++)
                for (int z = 0; z < size; z++)
                    wall[x, z] = true;

            var rng = new System.Random(Seed);
            Carve(wall, rng);

            // World position of a character centre; cell (0,0) is centred on the origin (robot start).
            float Pos(int c) => (c + 0.5f) * Char - 2.5f * Char;
            float mazeSize = size * Char;
            float mid = Pos(size / 2);

            BuildScene.BuildBase(new Vector3(0, 0.05f, 0), new Vector3(1, 1, 1), new Vector3(mid, 20, mid));

            var ground = GameObject.Find("Ground");
            ground.transform.position = new Vector3(mid, 0, mid);
            ground.transform.localScale = Vector3.one * (mazeSize / 10f + 1f);

            var cam = GameObject.Find("Main Camera");
            cam.transform.rotation = Quaternion.Euler(90, 0, 0);
            cam.GetComponent<Camera>().orthographic = true;
            cam.GetComponent<Camera>().orthographicSize = mazeSize / 2f + 1f;

            if (!AssetDatabase.IsValidFolder("Assets/Scenes"))
                AssetDatabase.CreateFolder("Assets", "Scenes");
            var root = new GameObject("Maze");
            var material = new Material(Shader.Find("Standard")) { color = new Color(0.55f, 0.55f, 0.6f) };
            AssetDatabase.CreateAsset(material, "Assets/Scenes/MazeWall.mat");

            // Merge horizontal runs of wall cells into one box each to keep the object count low.
            int boxes = 0;
            for (int z = 0; z < size; z++)
            {
                int x = 0;
                while (x < size)
                {
                    if (!wall[x, z]) { x++; continue; }
                    int start = x;
                    while (x < size && wall[x, z]) x++;
                    int len = x - start;
                    var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    box.name = $"Wall_{boxes++}";
                    box.transform.SetParent(root.transform);
                    box.transform.position = new Vector3((Pos(start) + Pos(x - 1)) / 2f, WallHeight / 2f, Pos(z));
                    box.transform.localScale = new Vector3(len * Char, WallHeight, Char);
                    box.GetComponent<Renderer>().sharedMaterial = material;
                    box.isStatic = true;
                }
            }

            // Visual-only goal marker in the far corner cell (no collider, so the lidar ignores it).
            int goalChar = (Cells - 1) * CellChars + 2;
            var goal = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            goal.name = "GoalMarker";
            Object.DestroyImmediate(goal.GetComponent<Collider>());
            goal.transform.position = new Vector3(Pos(goalChar), 0.01f, Pos(goalChar));
            goal.transform.localScale = new Vector3(0.6f, 0.01f, 0.6f);
            goal.GetComponent<Renderer>().sharedMaterial = new Material(Shader.Find("Standard")) { color = Color.green };

            BuildScene.SaveScene("Assets/Scenes/MazeScene.unity");

            // Unity (x right, z forward) -> ROS (x forward, y left): ros_x = z, ros_y = -x.
            Debug.Log($"Maze: {boxes} walls, {mazeSize} m square, robot start (ROS) 0,0. " +
                      $"Goal cell centre (ROS) x={Pos(goalChar):F2}, y={-Pos(goalChar):F2}\n" + Layout(wall, size));
        }

        static void Carve(bool[,] wall, System.Random rng)
        {
            // Recursive backtracker over the cell grid.
            var visited = new bool[Cells, Cells];
            var stack = new Stack<Vector2Int>();
            var dirs = new[] { new Vector2Int(1, 0), new Vector2Int(-1, 0), new Vector2Int(0, 1), new Vector2Int(0, -1) };
            visited[0, 0] = true;
            OpenCell(wall, 0, 0);
            stack.Push(Vector2Int.zero);
            while (stack.Count > 0)
            {
                var c = stack.Peek();
                var options = new List<Vector2Int>();
                foreach (var d in dirs)
                {
                    int nx = c.x + d.x, nz = c.y + d.y;
                    if (nx >= 0 && nz >= 0 && nx < Cells && nz < Cells && !visited[nx, nz])
                        options.Add(d);
                }
                if (options.Count == 0) { stack.Pop(); continue; }
                var pick = options[rng.Next(options.Count)];
                var n = c + pick;
                visited[n.x, n.y] = true;
                OpenCell(wall, n.x, n.y);
                OpenPassage(wall, c, pick);
                stack.Push(n);
            }

            // Extra openings between neighbouring cells create loops.
            for (int i = 0; i < ExtraOpenings; i++)
            {
                var c = new Vector2Int(rng.Next(Cells), rng.Next(Cells));
                var d = dirs[rng.Next(dirs.Length)];
                var n = c + d;
                if (n.x >= 0 && n.y >= 0 && n.x < Cells && n.y < Cells)
                    OpenPassage(wall, c, d);
            }
        }

        static void OpenCell(bool[,] wall, int cx, int cz)
        {
            for (int a = 1; a <= 3; a++)
                for (int b = 1; b <= 3; b++)
                    wall[cx * CellChars + a, cz * CellChars + b] = false;
        }

        static void OpenPassage(bool[,] wall, Vector2Int c, Vector2Int d)
        {
            // The wall column/row between two cells, across the 3 open characters.
            int wx = c.x * CellChars + 2 + d.x * 2;
            int wz = c.y * CellChars + 2 + d.y * 2;
            for (int k = -1; k <= 1; k++)
            {
                if (d.x != 0) wall[wx, wz + k] = false;
                else wall[wx + k, wz] = false;
            }
        }

        static string Layout(bool[,] wall, int size)
        {
            var sb = new StringBuilder("Layout (# wall, top row = far z / ROS +x end):\n");
            for (int z = size - 1; z >= 0; z--)
            {
                for (int x = 0; x < size; x++) sb.Append(wall[x, z] ? '#' : ' ');
                sb.Append('\n');
            }
            return sb.ToString();
        }
    }
}
