using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GeometryConstant
{
    static public float Tolerance =  1e-5f;
}

public class Vertex
{
    public Loop PLoop;
    public Vector2 Point;

    public Vertex()
    {
        Point = new Vector2(0, 0);
    }
    public Vertex(float x, float y)
    {
        Point = new Vector2(x, y);
    }
    public Vertex(Vector2 point)
    {
        Point = point;
    }

    public void SetPoint(Vector2 point)
    {
        Point = point;
        //Update();
    }

    public void SetPoint(float x, float y)
    {
        Point = new Vector2(x, y);
        //Update();
    }

    /*private void Update()
    {
        PLoop.Update();
    }*/
}

public class Edge
{
    public Vertex VertexA;
    public Vertex VertexB;

    public Vector2 getVector
    {
        get
        {
            return VertexB.Point - VertexA.Point;
        }
    }
}

public class Loop
{
    public Mesh PMesh;
    public List<Vertex> Vertices = new List<Vertex>();
    public List<Edge> Edges = new List<Edge>();

    public void AddVertex(Vertex vertex)
    {
        vertex.PLoop = this;
        // 判断是否和第一个点重合
        if (Vertices.Count > 0 && Vector2.Distance(vertex.Point, Vertices[0].Point) <= GeometryConstant.Tolerance)
        {
            return;
        }
        // 不重合，方才添加进去
        Vertices.Add(vertex);
    }

    public void AddEdge(Vertex a, Vertex b)
    {
        Edge newEdge = new Edge();
        newEdge.VertexA = a;
        newEdge.VertexB = b;
        Edges.Add(newEdge);
    }

    /*public void Update()
    {
        PMesh.Update();
    }*/

    public Vector3[] GetPositions()
    {
        Vector3[] positions = new Vector3[Vertices.Count + 1];
        for (int i = 0; i < positions.Length; i++)
        {
            positions[i] = Vertices[i % Vertices.Count].Point;
        }
        
        return positions;
    }
}

public class Mesh:MonoBehaviour
{
    public bool needUpdate = false;
    
    public List<LineRenderer> LineRenderers = new List<LineRenderer>();
    
    public List<Loop> Loops = new List<Loop>();
    public void AddLoop(Loop loop)
    {
        loop.PMesh = this;
        Loops.Add(loop);
        //Update();
    }
}
