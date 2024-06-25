using System.Collections.Generic;
using UnityEngine;
using System;
using Oculus.Interaction.Input;
using Oculus.Interaction;
using Oculus.Interaction.HandGrab;


[Serializable]
public class ComponentType
{
    public string componentName;
}
public class ShapeDrawer : MonoBehaviour
{
    public Handedness Handedness; // Left or Right hand
    public LineRenderer LineRenderer;
    private List<Vector3> points = new List<Vector3>();
    private bool isDrawing = false;
    public OVRPassthroughLayer PassthroughLayer;
    public Camera MainCamera;
    //public List<ComponentType> ComponentsToAdd = new List<ComponentType>();

    void Start()
    {
        // Ensure the LineRenderer is properly configured
        LineRenderer = GetComponent<LineRenderer>();
        PassthroughLayer = FindObjectOfType<OVRPassthroughLayer>();

        if (LineRenderer == null)
        {
            LineRenderer = gameObject.AddComponent<LineRenderer>();
            LineRenderer.startWidth = 0.01f;
            LineRenderer.endWidth = 0.01f;
            LineRenderer.material = new Material(Shader.Find("Unlit/Color"));
            LineRenderer.material.color = Color.white;
            LineRenderer.useWorldSpace = true;
        }
    }

    void Update()
    {
        // Check trigger button state
        bool triggerPressed = OVRInput.Get(OVRInput.Button.PrimaryIndexTrigger, Handedness == Handedness.Left ? OVRInput.Controller.LTouch : OVRInput.Controller.RTouch);

        if (triggerPressed)
        {
            if (!isDrawing)
            {
                StartDrawing();
            }
            Draw();
        }
        else if (isDrawing)
        {
            EndDrawing();
        }
    }

    void StartDrawing()
    {
        isDrawing = true;
        points.Clear();
        LineRenderer.positionCount = 0;
    }

    void Draw()
    {
        Vector3 newPoint = transform.position;
        if (points.Count == 0 || Vector3.Distance(points[points.Count - 1], newPoint) > 0.01f)
        {
            points.Add(newPoint);
            LineRenderer.positionCount = points.Count;
            LineRenderer.SetPosition(points.Count - 1, newPoint);
        }
    }

    void EndDrawing()
    {
        isDrawing = false;
        CreateMeshFromPoints();
        points.Clear();
        LineRenderer.positionCount = 0;
    }

    //void CreateMeshFromPoints()
    //{
    //    if (points.Count < 3)
    //    {
    //        points.Clear();
    //        LineRenderer.positionCount = 0;
    //        return; // A valid shape needs at least 3 points
    //    }

    //    // Create a 2D shape and convert it to a mesh
    //    Vector2[] points2D = new Vector2[points.Count];
    //    for (int i = 0; i < points.Count; i++)
    //    {
    //        points2D[i] = new Vector2(points[i].x, points[i].y); // Assuming drawing on the XY plane
    //    }

    //    // Use Triangulator or other method to generate triangles
    //    Triangulator triangulator = new Triangulator(points2D);
    //    int[] indices = triangulator.Triangulate();

    //    Vector3[] vertices = new Vector3[points.Count];
    //    for (int i = 0; i < vertices.Length; i++)
    //    {
    //        //vertices[i] = new Vector3(points2D[i].x, 0, points2D[i].y);
    //        vertices[i] = new Vector3(points2D[i].x, points2D[i].y, 0);
    //    }

    //    Mesh mesh = new Mesh
    //    {
    //        vertices = vertices,
    //        triangles = indices
    //    };
    //    mesh.RecalculateNormals();
    //    mesh.RecalculateBounds();

    //    GameObject shape = new GameObject("Shape", typeof(MeshFilter), typeof(MeshRenderer));
    //    //shape.transform.SetPositionAndRotation(MainCamera.transform.position + Vector3.forward, MainCamera.transform.rotation);
    //    shape.GetComponent<MeshFilter>().mesh = mesh;
    //    shape.GetComponent<MeshRenderer>().enabled = false;

    //    BoxCollider boxCollider = shape.AddComponent<BoxCollider>();
    //    Bounds bounds = mesh.bounds;
    //    boxCollider.center = bounds.center;
    //    boxCollider.size = bounds.size;

    //    Rigidbody rb = shape.AddComponent<Rigidbody>();
    //    rb.isKinematic = true;
    //    rb.useGravity = false;

    //    Grabbable grabbable = shape.AddComponent<Grabbable>();
    //    grabbable.InjectOptionalRigidbody(rb);
    //    HandGrabInteractable handGrabInteractable = shape.AddComponent<HandGrabInteractable>();
    //    handGrabInteractable.InjectOptionalPointableElement(grabbable);
    //    GrabInteractable grabInteractable = shape.AddComponent<GrabInteractable>();
    //    grabInteractable.InjectOptionalPointableElement(grabbable);

    //    //var col = shape.AddComponent<BoxCollider>();
    //    //col.
    //    PassthroughLayer.AddSurfaceGeometry(shape, true);
    //    //shape.GetComponent<MeshRenderer>().material = new Material(Shader.Find("Standard"));
    //}

    void CreateMeshFromPoints()
    {
        if (points.Count < 3)
        {
            points.Clear();
            LineRenderer.positionCount = 0;
            return; // A valid shape needs at least 3 points
        }

        // Create a 2D shape and convert it to a mesh
        Vector2[] points2D = new Vector2[points.Count];
        for (int i = 0; i < points.Count; i++)
        {
            points2D[i] = new Vector2(points[i].x, points[i].y); // Assuming drawing on the XY plane
        }

        // Use Triangulator or other method to generate triangles
        Triangulator triangulator = new Triangulator(points2D);
        int[] indices = triangulator.Triangulate();

        Vector3[] vertices = new Vector3[points.Count];
        for (int i = 0; i < vertices.Length; i++)
        {
            vertices[i] = new Vector3(points2D[i].x, points2D[i].y, points[i].z);
        }

        Mesh mesh = new Mesh
        {
            vertices = vertices,
            triangles = indices
        };
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        // Create the shape GameObject
        GameObject shape = new GameObject("Shape", typeof(MeshFilter), typeof(MeshRenderer));
        shape.GetComponent<MeshFilter>().mesh = mesh;
        shape.GetComponent<MeshRenderer>().enabled = false;

        // Position the shape GameObject at the center of the drawn shape
        shape.transform.position = GetShapeCenter();
        shape.transform.rotation = Quaternion.identity;

        // Adjust the vertices to be relative to the shape GameObject's position
        Vector3 center = shape.transform.position;
        for (int i = 0; i < vertices.Length; i++)
        {
            vertices[i] -= center;
        }
        mesh.vertices = vertices;

        // Update the collider
        BoxCollider boxCollider = shape.AddComponent<BoxCollider>();
        Bounds bounds = mesh.bounds;
        boxCollider.center = bounds.center;
        boxCollider.size = bounds.size;

        // Add components for interaction
        Rigidbody rb = shape.AddComponent<Rigidbody>();
        rb.isKinematic = true;
        rb.useGravity = false;

        Grabbable grabbable = shape.AddComponent<Grabbable>();
        grabbable.InjectOptionalRigidbody(rb);
        HandGrabInteractable handGrabInteractable = shape.AddComponent<HandGrabInteractable>();
        handGrabInteractable.InjectOptionalPointableElement(grabbable);
        GrabInteractable grabInteractable = shape.AddComponent<GrabInteractable>();
        grabInteractable.InjectOptionalPointableElement(grabbable);

        PassthroughLayer.AddSurfaceGeometry(shape, true);
    }

    Vector3 GetShapeCenter()
    {
        Vector3 sum = Vector3.zero;
        foreach (Vector3 point in points)
        {
            sum += point;
        }
        return sum / points.Count;
    }
}

public class Triangulator
{
    private List<Vector2> m_points = new List<Vector2>();

    public Triangulator(Vector2[] points)
    {
        m_points = new List<Vector2>(points);
    }

    public int[] Triangulate()
    {
        List<int> indices = new List<int>();

        int n = m_points.Count;
        if (n < 3)
            return indices.ToArray();

        int[] V = new int[n];
        if (Area() > 0)
        {
            for (int v = 0; v < n; v++)
                V[v] = v;
        }
        else
        {
            for (int v = 0; v < n; v++)
                V[v] = (n - 1) - v;
        }

        int nv = n;
        int count = 2 * nv;
        for (int v = nv - 1; nv > 2;)
        {
            if ((count--) <= 0)
                return indices.ToArray();

            int u = v;
            if (nv <= u)
                u = 0;
            v = u + 1;
            if (nv <= v)
                v = 0;
            int w = v + 1;
            if (nv <= w)
                w = 0;

            if (Snip(u, v, w, nv, V))
            {
                int a, b, c, s, t;
                a = V[u];
                b = V[v];
                c = V[w];
                indices.Add(a);
                indices.Add(b);
                indices.Add(c);
                for (s = v, t = v + 1; t < nv; s++, t++)
                    V[s] = V[t];
                nv--;
                count = 2 * nv;
            }
        }
        indices.Reverse();
        return indices.ToArray();
    }

    private float Area()
    {
        int n = m_points.Count;
        float A = 0.0f;
        for (int p = n - 1, q = 0; q < n; p = q++)
        {
            Vector2 pval = m_points[p];
            Vector2 qval = m_points[q];
            A += pval.x * qval.y - qval.x * pval.y;
        }
        return (A * 0.5f);
    }

    private bool Snip(int u, int v, int w, int n, int[] V)
    {
        int p;
        Vector2 A = m_points[V[u]];
        Vector2 B = m_points[V[v]];
        Vector2 C = m_points[V[w]];
        if (Mathf.Epsilon > (((B.x - A.x) * (C.y - A.y)) - ((B.y - A.y) * (C.x - A.x))))
            return false;
        for (p = 0; p < n; p++)
        {
            if ((p == u) || (p == v) || (p == w))
                continue;
            Vector2 P = m_points[V[p]];
            if (InsideTriangle(A, B, C, P))
                return false;
        }
        return true;
    }

    private bool InsideTriangle(Vector2 A, Vector2 B, Vector2 C, Vector2 P)
    {
        float ax, ay, bx, by, cx, cy;
        float apx, apy, bpx, bpy, cpx, cpy;
        float cCROSSap, bCROSScp, aCROSSbp;

        ax = C.x - B.x; ay = C.y - B.y;
        bx = A.x - C.x; by = A.y - C.y;
        cx = B.x - A.x; cy = B.y - A.y;
        apx = P.x - A.x; apy = P.y - A.y;
        bpx = P.x - B.x; bpy = P.y - B.y;
        cpx = P.x - C.x; cpy = P.y - C.y;

        aCROSSbp = ax * bpy - ay * bpx;
        cCROSSap = cx * apy - cy * apx;
        bCROSScp = bx * cpy - by * cpx;

        return ((aCROSSbp >= 0.0) && (bCROSScp >= 0.0) && (cCROSSap >= 0.0));
    }
}

