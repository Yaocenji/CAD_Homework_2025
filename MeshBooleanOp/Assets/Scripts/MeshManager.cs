using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using TMPro;
using UnityEditor.Experimental.GraphView;
using UnityEngine;
using UnityEngine.UI;

public class MeshManager : MonoBehaviour
{
    static public MeshManager Instance;

    public TMP_InputField meshAPath;
    public TMP_InputField meshBPath;
    
    public Mesh meshA;
    public Mesh meshB;
    public Mesh meshC;
    
    private string meshAText;
    private string meshBText;

    private void Awake()
    {
        Instance = this;
        
        /*_meshA = new Mesh();
        _meshB = new Mesh();*/
        
        /*Loop loop = new Loop();
        loop.AddVertex(new Vertex(0f, 0f));
        loop.AddVertex(new Vertex(1f, 0f));
        loop.AddVertex(new Vertex(0.5f, 1.2f));
        
        _meshA.AddLoop(loop);

        _meshA.Update();*/
    }

    public void LoadMeshA()
    {
        meshAText = File.ReadAllText(meshAPath.text);
        FromTextToMesh(meshAText, ref meshA);
    }
    public void LoadMeshB()
    {
        meshBText = File.ReadAllText(meshBPath.text);
        FromTextToMesh(meshBText, ref meshB);
    }

    private void FromTextToMesh(string text, ref Mesh mesh)
    {
        // 分行
        string[] lines = text.Split('\n');
        
        // 临时loop
        Loop tmpLoop = null;
        
        for (int i = 0; i < lines.Length; i++)
        {
            // 若末位\R，去之
            if (lines[i].EndsWith("\r"))
            {
                lines[i] = lines[i].Substring(0, lines[i].Length - 1);
            }

            if (lines[i].Equals("") || lines[i].Equals(" "))    // 空行，忽略之
            {
                continue;
            }
            else if (lines[i].Equals("#loop"))
            {
                // 如果不空，那就加进去
                if (tmpLoop is not null && tmpLoop.Vertices.Count > 0)
                {
                    // loop生成边
                    for (int j = 0; j < tmpLoop.Vertices.Count; j++)
                    {
                        tmpLoop.AddEdge(tmpLoop.Vertices[j % tmpLoop.Vertices.Count], tmpLoop.Vertices[(j + 1) % tmpLoop.Vertices.Count]);
                    }
                    // 将loop加入
                    mesh.AddLoop(tmpLoop);
                }
                // 刷新
                tmpLoop = new Loop();
            }
            else
            {
                // 添加点
                string[] pos = lines[i].Split(' ');
                if (pos.Length == 2 && tmpLoop is not null)
                {
                    tmpLoop.AddVertex(new Vertex(float.Parse(pos[0]), float.Parse(pos[1])));
                }
            }
        }
        // 如果不空，那就加进去
        if (tmpLoop is not null && tmpLoop.Vertices.Count > 0)
        {
            // loop生成边
            for (int j = 0; j < tmpLoop.Vertices.Count; j++)
            {
                tmpLoop.AddEdge(tmpLoop.Vertices[j % tmpLoop.Vertices.Count], tmpLoop.Vertices[(j + 1) % tmpLoop.Vertices.Count]);
            }
            // 将loop加入
            mesh.AddLoop(tmpLoop);
        }

        // debug内容
        /*foreach (Loop loop in mesh.Loops)
        {
            Debug.Log("edges in loop:");
            foreach (Edge edge in loop.Edges)
            {
                Debug.Log(edge.VertexA.Point + "  " + edge.VertexB.Point);
            }
        }*/
        
        mesh.needUpdate = true;
    }

    // Start is called before the first frame update
    void Start()
    {
        // 在这里测试一些列操作
        Edge a = new Edge();
        a.VertexA = new Vertex(new Vector2(-1, 1));
        a.VertexB = new Vertex(new Vector2(0, 1));
        Edge b = new Edge();
        b.VertexA = new Vertex(new Vector2(0, -1));
        b.VertexB = new Vertex(new Vector2(0, 2));

        BooleanOperation.IntersectionEdgeEdgeResult ANS;
        BooleanOperation.IntersectionEdgeEdge(b, a, out ANS);
        if (ANS is not null)
        {
            Debug.Log(ANS.iType + "  " + ANS.pType + "  " + ANS.spsr + "  " + ANS.hTType + "  " + ANS.tSType
                      + "\n" + ANS.intPoint + "  " + ANS.intEdge.VertexA.Point + "  " + ANS.intEdge.VertexB.Point);
        }
        else
            Debug.Log("无交点");
    }

    public void Intersection()
    {
        BooleanOperation.Intersection(ref meshA, ref meshB, ref meshC);
    }
}
