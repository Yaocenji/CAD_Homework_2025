using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class MeshManager : MonoBehaviour
{
    static public MeshManager Instance;

    public TMP_InputField meshAPath;
    public TMP_InputField meshBPath;

    public Toggle ToggleA;
    public Toggle ToggleB;
    public Toggle ToggleC;
    public Toggle ToggleD;
    public Toggle ToggleE;
    public Toggle ToggleF;
    
    public Mesh meshA;
    public Mesh meshB;
    public Mesh meshC;
    public Mesh meshD;
    public Mesh meshE;
    public Mesh meshF;

    private Mesh reverseMeshA;
    private Mesh reverseMeshB;
    
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
        ToggleA.isOn = true;
    }
    public void LoadMeshB()
    {
        meshBText = File.ReadAllText(meshBPath.text);
        FromTextToMesh(meshBText, ref meshB);
        ToggleB.isOn = true;
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
                    //Debug.Log(pos[0] + " " + pos[1]);
                    tmpLoop.AddVertex(new Vertex(float.Parse(pos[0]), float.Parse(pos[1])));
                }
            }
        }
        // 如果不空，那就加进去
        if (tmpLoop is not null && tmpLoop.Vertices.Count > 0)
        {
            // loop生成边
            tmpLoop.GenerateEdges();
            // 将loop加入
            mesh.AddLoop(tmpLoop);
        }
        
        mesh.OptimizeTopo();
        
        mesh.needUpdate = true;
    }

    // Start is called before the first frame update
    void Start()
    {
        // 在这里测试一些列操作
        /*Edge a = new Edge();
        a.VertexBegin = new Vertex(new Vector2(-1, 1));
        a.VertexEnd = new Vertex(new Vector2(0, 1));
        Edge b = new Edge();
        b.VertexBegin = new Vertex(new Vector2(0, -1));
        b.VertexEnd = new Vertex(new Vector2(0, 2));

        BooleanOperation.IntersectionEdgeEdgeResult ANS;
        BooleanOperation.IntersectionEdgeEdge(b, a, out ANS);
        if (ANS is not null)
        {
            Debug.Log(ANS.iType + "  " + ANS.pType + "  " + ANS.spsr + "  " + ANS.hTType + "  " + ANS.tSType
                      + "\n" + ANS.intPoint + "  " + ANS.intEdge.VertexBegin.Point + "  " + ANS.intEdge.VertexEnd.Point);
        }
        else
            Debug.Log("无交点");*/
    }

    public void ClearAll()
    {
        meshA.ClearAll();
        meshB.ClearAll();
        meshC.ClearAll();
        meshD.ClearAll();
        meshE.ClearAll();
        meshF.ClearAll();
        
        reverseMeshA.ClearAll();
        reverseMeshB.ClearAll();
    }
    
    public void SetADraw()
    {
        if (ToggleA.isOn && meshA is not null)
            meshA.EnableDraw();
        else if  (!ToggleA.isOn && meshA is not null)
            meshA.DisableDraw();
    }

    public void SetBDraw()
    {
        if (ToggleB.isOn && meshB is not null)
            meshB.EnableDraw();
        else if  (!ToggleB.isOn && meshB is not null)
            meshB.DisableDraw();
    }

    public void SetCDraw()
    {
        if (ToggleC.isOn && meshC is not null)
            meshC.EnableDraw();
        else if  (!ToggleC.isOn && meshC is not null)
            meshC.DisableDraw();
    }

    public void SetDDraw()
    {
        if (ToggleD.isOn && meshD is not null)
            meshD.EnableDraw();
        else if  (!ToggleD.isOn && meshD is not null)
            meshD.DisableDraw();
    }

    public void SetEDraw()
    {
        if (ToggleE.isOn && meshE is not null)
            meshE.EnableDraw();
        else if  (!ToggleE.isOn && meshE is not null)
            meshE.DisableDraw();
    }

    public void SetFDraw()
    {
        if (ToggleF.isOn && meshF is not null)
            meshF.EnableDraw();
        else if  (!ToggleF.isOn && meshF is not null)
            meshF.DisableDraw();
    }
    
    public void Intersection()
    {
        meshC.ClearAll();
        BooleanOperation.Intersection(ref meshA, ref meshB, ref meshC);
        ToggleC.isOn = true;
    }
    
    public void Addition()
    {
        meshD.ClearAll();
        BooleanOperation.Addition(ref meshA, ref meshB, ref meshD);
        ToggleD.isOn = true;
    }

    public void AMinusB()
    {
        meshE.ClearAll();
        if (reverseMeshB is null)
        {
            GameObject go = new GameObject();
            go.name = "reverseMeshB";
            go.transform.parent = transform;
            reverseMeshB = go.AddComponent<Mesh>();
        }
        reverseMeshB.ClearAll();
        Mesh.GetReverse(meshB, ref reverseMeshB);
        
        // 多加一步操作，为 reverseMeshB 添加外边界
        // 首先，获取MeshA和MeshB的共享AABB
        // left right buttom top
        Vector4 aabb = new Vector4(float.MaxValue, float.MinValue, float.MaxValue, float.MinValue);
        foreach (var loop in Enumerable.Concat(meshA.Loops, meshB.Loops))
        {
            foreach (var vert in loop.Vertices)
            {
                if (vert.Point.x < aabb.x)
                    aabb.x = vert.Point.x;
                if (vert.Point.x > aabb.y)
                    aabb.y = vert.Point.x;
                
                if (vert.Point.y < aabb.z)
                    aabb.z = vert.Point.y;
                if (vert.Point.y > aabb.w)
                    aabb.w = vert.Point.y;
            }
        }
        // 包围盒向外扩展一层
        aabb += new Vector4(-1, 1, -1, 1);
        // 新建一个外包围loop加到reverseB中
        Loop outLineLoop = new Loop();
        outLineLoop.AddVertex(new Vertex(aabb.x, aabb.z));
        outLineLoop.AddVertex(new Vertex(aabb.y, aabb.z));
        outLineLoop.AddVertex(new Vertex(aabb.y, aabb.w));
        outLineLoop.AddVertex(new Vertex(aabb.x, aabb.w));
        outLineLoop.GenerateEdges();
        reverseMeshB.AddLoop(outLineLoop);
        
        BooleanOperation.Intersection(ref meshA, ref reverseMeshB, ref meshE);
        
        ToggleE.isOn = true;
    }
    public void BMinusA()
    {
        meshF.ClearAll();
        if (reverseMeshA is null)
        {
            GameObject go = new GameObject();
            go.name = "reverseMeshA";
            go.transform.parent = transform;
            reverseMeshA = go.AddComponent<Mesh>();
        }
        reverseMeshA.ClearAll();
        Mesh.GetReverse(meshA, ref reverseMeshA);
        
        // 多加一步操作，为 reverseMeshB 添加外边界
        // 首先，获取MeshA和MeshB的共享AABB
        // left right buttom top
        Vector4 aabb = new Vector4(float.MaxValue, float.MinValue, float.MaxValue, float.MinValue);
        foreach (var loop in Enumerable.Concat(meshA.Loops, meshB.Loops))
        {
            foreach (var vert in loop.Vertices)
            {
                if (vert.Point.x < aabb.x)
                    aabb.x = vert.Point.x;
                if (vert.Point.x > aabb.y)
                    aabb.y = vert.Point.x;
                
                if (vert.Point.y < aabb.z)
                    aabb.z = vert.Point.y;
                if (vert.Point.y > aabb.w)
                    aabb.w = vert.Point.y;
            }
        }
        // 包围盒向外扩展一层
        aabb += new Vector4(-1, 1, -1, 1);
        // 新建一个外包围loop加到reverseA中
        Loop outLineLoop = new Loop();
        outLineLoop.AddVertex(new Vertex(aabb.x, aabb.z));
        outLineLoop.AddVertex(new Vertex(aabb.y, aabb.z));
        outLineLoop.AddVertex(new Vertex(aabb.y, aabb.w));
        outLineLoop.AddVertex(new Vertex(aabb.x, aabb.w));
        outLineLoop.GenerateEdges();
        reverseMeshA.AddLoop(outLineLoop);
        
        BooleanOperation.Intersection(ref reverseMeshA, ref meshB, ref meshF);
        
        ToggleF.isOn = true;
    }
}
