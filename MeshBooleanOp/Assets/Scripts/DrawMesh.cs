using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;


public class DrawMesh : MonoBehaviour
{
    public Material materialA;
    public Material materialB;
    public Material materialC;
    public Material materialD;
    public Material materialE;
    public Material materialF;

    public float lineWidth = 0.02f;
    public float ToothInteval = 0.2f;
    public float ToothLength = 0.2f;
    
    public float fillLineWidth = 0.01f;
    public float fillLineInteval = 0.2f;
    
    private List<LineRenderer> _lineRenderers1;
    
    void Start()
    {
        _lineRenderers1 =  new List<LineRenderer>();
        
        //MeshManager.Instance.meshA.LineRenderers =  _lineRenderers1;
    }

    // Update is called once per frame
    void Update()
    {
        if (MeshManager.Instance.meshA is not null && MeshManager.Instance.meshA.needUpdate)
        {
            UpdateMeshAndLineRenderers(MeshManager.Instance.meshA, materialA, (Mathf.PI * 0.5f / 7f) * 1);
        }
        if (MeshManager.Instance.meshB is not null && MeshManager.Instance.meshB.needUpdate)
        {
            UpdateMeshAndLineRenderers(MeshManager.Instance.meshB, materialB, (Mathf.PI * 0.5f / 7f) * 2);
        }
        if (MeshManager.Instance.meshC is not null && MeshManager.Instance.meshC.needUpdate)
        {
            UpdateMeshAndLineRenderers(MeshManager.Instance.meshC, materialC, (Mathf.PI * 0.5f / 7f) * 3);
        }

        if (MeshManager.Instance.meshD is not null && MeshManager.Instance.meshD.needUpdate)
        {
            UpdateMeshAndLineRenderers(MeshManager.Instance.meshD, materialD, (Mathf.PI * 0.5f / 7f) * 4);
        }

        if (MeshManager.Instance.meshE is not null && MeshManager.Instance.meshE.needUpdate)
        {
            UpdateMeshAndLineRenderers(MeshManager.Instance.meshE, materialE, (Mathf.PI * 0.5f / 7f) * 5);
        }

        if (MeshManager.Instance.meshF is not null && MeshManager.Instance.meshF.needUpdate)
        {
            UpdateMeshAndLineRenderers(MeshManager.Instance.meshF, materialF, (Mathf.PI * 0.5f / 7f) * 6);
        }
    }
    
    public Vector3[] GetMeshDrawPositions(in Loop loop)
    {
        List<Vector3> positions = new List<Vector3>();
        for (int i = 0; i < loop.Vertices.Count + 1; i++)
        {
            /*
            // 假如当前List的数据量大于等于1
            if (positions.Count >= 1)
            {
                // 获取当前的最后两个点
                Vector2 lastPoint = loop.Vertices[i % loop.Vertices.Count].Point;
                Vector2 preLastPoint = positions[positions.Count - 1];
                // 这个相当于最后一条边
                // 添加内齿
                float length = (lastPoint - preLastPoint).magnitude;
                Debug.Log(length);
                
                Vector2 edgeVector = (lastPoint - preLastPoint).normalized;
                Vector2 ToothDir = new Vector2(edgeVector.y, -edgeVector.x);
                if (BooleanOperation.Cross(edgeVector, ToothDir) < 0)
                    ToothDir = -ToothDir;
                
                for (float j = ToothInteval; j < length; j += ToothInteval)
                {
                    Vector2 toothBeginPos = preLastPoint + edgeVector * j;
                    //Debug.Log(toothBeginPos);
                    Vector2 toothMidPos = toothBeginPos + ToothDir * ToothLength + edgeVector * lineWidth / 2.0f;
                    Vector3 toothEndPos = toothBeginPos + edgeVector * lineWidth;
                    
                    positions.Add(toothBeginPos);
                    positions.Add(toothMidPos);
                    positions.Add(toothEndPos);
                }
            }
            */
            // 添加结尾点
            positions.Add(loop.Vertices[i % loop.Vertices.Count].Point);
        }
        
        return positions.ToArray();
    }

    private void UpdateMeshAndLineRenderers(Mesh mesh , Material material, float fillLineThetaRad)
    {
        
        mesh.needUpdate = false;
        if (mesh.LineRenderers is not null)
        {
            foreach (LineRenderer lr in mesh.LineRenderers)
            {
                // 删去所有的lineRenderer，并删去挂接它的物体
                GameObject go = lr.gameObject;
                Destroy(lr);
                Destroy(go);
            }
            mesh.LineRenderers.Clear();
            
            for (int i = 0; i < mesh.Loops.Count; i++)
            {
                // 创建mesh的子物体，并挂接lineRenderer
                GameObject go = new GameObject();
                go.transform.parent = mesh.transform;
                LineRenderer newLineRenderer = go.AddComponent<LineRenderer>();
                
                // 修改lineRenderer
                newLineRenderer.material = material;
                
                newLineRenderer.startWidth = lineWidth;
                newLineRenderer.endWidth = lineWidth;
                newLineRenderer.loop = true;
                newLineRenderer.numCornerVertices = 10;

                var positions = GetMeshDrawPositions(mesh.Loops[i]); // mesh.Loops[i].GetPositions();
                newLineRenderer.positionCount = positions.Length;
                newLineRenderer.SetPositions(positions);
                
                mesh.LineRenderers.Add(newLineRenderer);
            }
        }
        
        // 接下来绘制内部填充线
        // 获取Mesh的AABB
        Vector4 aabb = new Vector4(float.MaxValue, float.MinValue, float.MaxValue, float.MinValue);
        foreach (var loop in mesh.Loops)
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
        // 扩展一圈
        aabb += new Vector4(-1, 1, -1, 1);
        // 填充线方向
        Vector2 dir = new Vector2(Mathf.Cos(fillLineThetaRad), Mathf.Sin(fillLineThetaRad));
        // 填充线画法：所有填充线从AABB的buttom下边缘起始，从左到右，沿着dir方向
        float fillLineBeginXBegin = aabb.x - (aabb.w - aabb.z) / Mathf.Tan(fillLineThetaRad);
        float fillLineBeginXEnd = aabb.y;
        float fillLineBeginY = aabb.z;
        float fillLineIntervalX = fillLineInteval / Mathf.Sin(fillLineThetaRad);
        float fillLineMaxLength = (aabb.w - aabb.z) / Mathf.Sin(fillLineThetaRad);
        
        // 绘制每一根线
        for (float fillLineBeginX = fillLineBeginXBegin; fillLineBeginX < fillLineBeginXEnd; fillLineBeginX += fillLineIntervalX)
        {
            Vector2 newLineBegin = new Vector2(fillLineBeginX, fillLineBeginY);
            Vector2 newLineEnd = newLineBegin + fillLineMaxLength * dir;
            
            Vertex newLineEdgeBegin = new Vertex(newLineBegin);
            Vertex newLineEdgeEnd = new Vertex(newLineEnd);
            
            Edge newLineEdge = new Edge();
            newLineEdge.VertexBegin = newLineEdgeBegin;
            newLineEdge.VertexEnd = newLineEdgeEnd;

            // 该填充线边和Mesh边求交结果
            List<BooleanOperation.IntersectionEdgeEdgeResult> results = new List<BooleanOperation.IntersectionEdgeEdgeResult>();
            foreach (var loop in mesh.Loops)
            {
                foreach (var edge in loop.Edges)
                {
                    BooleanOperation.IntersectionEdgeEdgeResult result = null;
                    BooleanOperation.IntersectionEdgeEdge(edge, newLineEdge, out result);

                    if (result is not null)
                    {
                        results.Add(result);
                    }
                }
            }
            
            // 没有交点或者存在重合边的、存在十字路口或T字的情况，都放弃
            if (results.Count == 0)
                continue;
            bool allIsSignelPoint = true;
            foreach (BooleanOperation.IntersectionEdgeEdgeResult result in results)
            {
                if (result.iType == BooleanOperation.IntersectionEdgeEdgeResult.intersectionType.SegmentOverlap)
                {
                    allIsSignelPoint = false;
                    break;
                }

                if (result.iType == BooleanOperation.IntersectionEdgeEdgeResult.intersectionType.SinglePoint &&
                    (result.spsr == BooleanOperation.IntersectionEdgeEdgeResult.singlePointSpecialRelationship
                         .HeadTail ||
                     result.spsr == BooleanOperation.IntersectionEdgeEdgeResult.singlePointSpecialRelationship.TShape))
                {
                    allIsSignelPoint = false;
                    break;
                }
            }
            if (!allIsSignelPoint)
                continue;
            
            // 一般不会出现但是万一result的数量是奇数，说明错了：
            if ((results.Count & 1) == 1)
                continue;
            
            // 现在result只有X型交点
            // 根据到起点的距离排序
            results.Sort((a, b) =>
            {
                float dist1 = Vector2.Distance(a.intPoint, newLineBegin);
                float dist2 = Vector2.Distance(b.intPoint, newLineBegin);
                if (dist1 > dist2)
                    return 1;
                else 
                    return -1;
            });

            for (int i = 0; i < results.Count; i += 2)
            {
                // 创建mesh的子物体，并挂接lineRenderer
                GameObject go = new GameObject();
                go.transform.parent = mesh.transform;
                LineRenderer newLineRenderer = go.AddComponent<LineRenderer>();
                
                // 修改lineRenderer
                newLineRenderer.material = material;
                
                newLineRenderer.startWidth = fillLineWidth;
                newLineRenderer.endWidth = fillLineWidth;
                newLineRenderer.loop = true;

                var positions = new Vector3[2];
                positions[0] = results[i].intPoint;
                positions[1] = results[i + 1].intPoint;
                
                newLineRenderer.positionCount = 2;
                newLineRenderer.SetPositions(positions);
                
                mesh.LineRenderers.Add(newLineRenderer);
            }
        }
    }

    public static void SetOrder(Mesh mesh, int order)
    {
        foreach (LineRenderer lr in mesh.LineRenderers)
        {
            lr.sortingOrder = order;
        }
    }
}
