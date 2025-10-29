using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BooleanOperation
{
    /*public struct Edge
    {
        public Vector2 VertexA.Point;
        public Vector2 VertexB.Point;
    }*/
    
    // 边边求交系统
    public class IntersectionEdgeEdgeResult
    {
        public IntersectionEdgeEdgeResult()
        {
            intEdge = new Edge();
            intEdge.VertexA = new Vertex();
            intEdge.VertexB = new Vertex();
        }
        
        // 边的相交类型：单个点或重合一段
        public enum intersectionType
        {
            SinglePoint,
            SegmentOverlap
        }
        public intersectionType iType;
        
        // 是否平行
        public enum parellelType
        {
            Parallel,
            NonParallel,
        }
        public parellelType pType;
        
        // 单点相交的时候的特殊关系
        public enum singlePointSpecialRelationship
        {
            None,
            HeadTail,   // 两条线段头尾相连交于一点
            TShape,     // 两条线段成T字型交于一点
        }
        public singlePointSpecialRelationship spsr;
        
        // 头尾相接型，那么可能是A尾和B头重合，也有可能是B尾和A头重合，也可能是头头/尾尾
        public enum headTailType
        {
            AToB,   // A尾连B头
            BToA,   // B尾连A头
            ABHead, // AB头头连接（相背）
            ABTail, // AB尾尾连接（相向）
        }
        public headTailType hTType;

        // T字型交点，有四种情形
        public enum tShapeType
        {
            TShape_AinB,
            TShape_AoutB,
            TShape_BinA,
            TShape_BoutA,
        }
        public tShapeType tSType;
        

        // 如果是单个交点，那么这是交点坐标
        public Vector2 intPoint;
        
        // 如果是一段重合，那么这是这一段的前后坐标
        public Edge intEdge;
        
        // 记录：被求交的两条边
        public Edge aEdge;
        public Edge bEdge;
    }
    
    // 点和线段边的距离
    static public float DistancePointEdge(Edge edge, Vector2 point, ref Vector2 closestPoint)
    {
        // Vector from VertexA.Point to VertexB.Point
        Vector2 lineDirection = edge.VertexB.Point - edge.VertexA.Point;
        
        float lineLengthSqr = lineDirection.sqrMagnitude;

        if (lineLengthSqr == 0.0f)
        {
            return Vector2.Distance(point, edge.VertexA.Point);
        }
        float t = Vector2.Dot(point - edge.VertexA.Point, lineDirection) / lineLengthSqr;
        t = Mathf.Clamp01(t);
        
        closestPoint = edge.VertexA.Point + t * lineDirection;

        return Vector2.Distance(point, closestPoint);
    }
    
    // Helper function for 2D cross product
    private static float Cross(Vector2 a, Vector2 b)
    {
        return a.x * b.y - b.y * a.x;
    }
    // 线段边求交
    static public void IntersectionEdgeEdge(Edge edgeA, Edge edgeB, out IntersectionEdgeEdgeResult intersectionEdge)
    {
        intersectionEdge = null;

        Vector2 p = edgeA.VertexA.Point;
        Vector2 r = edgeA.VertexB.Point - edgeA.VertexA.Point;
        Vector2 q = edgeB.VertexA.Point;
        Vector2 s = edgeB.VertexB.Point - edgeB.VertexA.Point;

        // Using 2D cross product for calculations. For V(x, y), cross product V.z = x*y' - y*x'
        float r_cross_s = r.x * s.y - r.y * s.x;
        Vector2 q_minus_p = q - p;
        float q_minus_p_cross_r = q_minus_p.x * r.y - q_minus_p.y * r.x;

        // Case 1: Lines are collinear or parallel.
        if (Mathf.Abs(r_cross_s) < GeometryConstant.Tolerance)
        {
            // Check if they are collinear (on the same infinite line).
            // If they are parallel but not on the same line, they don't intersect.
            if (Mathf.Abs(q_minus_p_cross_r) > GeometryConstant.Tolerance)
            {
                return; // Parallel and non-intersecting
            }

            // They are collinear. Now check for segment overlap.
            float r_dot_r = Vector2.Dot(r, r);
            float t0 = Vector2.Dot(q_minus_p, r) / r_dot_r;
            float t1 = t0 + Vector2.Dot(s, r) / r_dot_r;
            
            // The interval for edgeA is [0, 1].
            // We find the interval for edgeB projected onto edgeA.
            float overlap_start = Mathf.Max(0.0f, Mathf.Min(t0, t1));
            float overlap_end = Mathf.Min(1.0f, Mathf.Max(t0, t1));

            // Check if there is a valid overlap
            if (overlap_start > overlap_end + GeometryConstant.Tolerance)
            {
                return; // Collinear but no overlap
            }
            
            intersectionEdge = new IntersectionEdgeEdgeResult();
            intersectionEdge.aEdge = edgeA;
            intersectionEdge.bEdge = edgeB;
            
            // 走到这条条件分支，必然是平行类的
            intersectionEdge.pType = IntersectionEdgeEdgeResult.parellelType.Parallel;

            // Check if the overlap is just a single point
            if (Mathf.Abs(overlap_start - overlap_end) < GeometryConstant.Tolerance)
            {
                intersectionEdge.iType = IntersectionEdgeEdgeResult.intersectionType.SinglePoint;
                intersectionEdge.intPoint = p + overlap_start * r;
                
                // For collinear segments, a single point intersection must be a HeadTail connection.
                // 对于共线线段，单点相交必然是头尾相接。
                intersectionEdge.spsr = IntersectionEdgeEdgeResult.singlePointSpecialRelationship.HeadTail;
                
                // Determine the specific HeadTail type.
                // 确定具体的头尾相接类型。
                bool a_end_touches = Mathf.Abs(1.0f - overlap_start) < GeometryConstant.Tolerance;
                bool b_start_touches = (Mathf.Abs(t0 - overlap_start) < GeometryConstant.Tolerance && Vector2.Dot(r,s) > 0) || (Mathf.Abs(t1 - overlap_start) < GeometryConstant.Tolerance && Vector2.Dot(r,s) < 0);
                if(a_end_touches && b_start_touches)
                {
                    intersectionEdge.hTType = IntersectionEdgeEdgeResult.headTailType.AToB;
                }
                else
                {
                    intersectionEdge.hTType = IntersectionEdgeEdgeResult.headTailType.BToA;
                }
            }
            else // The overlap is a line segment
            {
                intersectionEdge.iType = IntersectionEdgeEdgeResult.intersectionType.SegmentOverlap;
                intersectionEdge.intEdge.VertexA.Point = p + overlap_start * r;
                intersectionEdge.intEdge.VertexB.Point = p + overlap_end * r;
            }
            return;
        }

        // Case 2: Lines are not parallel and intersect at one point.
        // We need to find if the intersection point lies on both segments.
        // We solve p + t*r = q + u*s for t and u.
        // t = (q - p) x s / (r x s)
        // u = (q - p) x r / (r x s)
        float t = (q_minus_p.x * s.y - q_minus_p.y * s.x) / r_cross_s;
        float u = q_minus_p_cross_r / r_cross_s;

        // The intersection point is on both segments if 0 <= t <= 1 and 0 <= u <= 1.
        if (t >= -GeometryConstant.Tolerance && t <= 1 + GeometryConstant.Tolerance && u >= -GeometryConstant.Tolerance && u <= 1 + GeometryConstant.Tolerance)
        {
            intersectionEdge = new IntersectionEdgeEdgeResult();
            intersectionEdge.aEdge = edgeA;
            intersectionEdge.bEdge = edgeB;
            intersectionEdge.iType = IntersectionEdgeEdgeResult.intersectionType.SinglePoint;
            intersectionEdge.intPoint = p + t * r;
            
            
             // --- Determine the special relationship for this single point intersection ---
            // --- 判断该单点相交的特殊关系 ---

            // Check if t or u are at the endpoints (0 or 1).
            // 检查 t 或 u 是否位于端点（0 或 1）。
            bool tIsEndpoint = t < GeometryConstant.Tolerance || t > 1 - GeometryConstant.Tolerance;
            bool uIsEndpoint = u < GeometryConstant.Tolerance || u > 1 - GeometryConstant.Tolerance;

            if (tIsEndpoint && uIsEndpoint)
            {
                // Both endpoints meet. It's a HeadTail connection.
                // 两个端点相遇，是头尾相接。
                intersectionEdge.spsr = IntersectionEdgeEdgeResult.singlePointSpecialRelationship.HeadTail;

                bool tIsEnd = t > 1 - GeometryConstant.Tolerance;
                bool uIsStart = u < GeometryConstant.Tolerance;

                if (tIsEnd && uIsStart)
                {
                    // A's tail meets B's head.
                    // A的尾部连接B的头部。
                    intersectionEdge.hTType = IntersectionEdgeEdgeResult.headTailType.AToB;
                }
                else
                {
                    // All other endpoint-to-endpoint cases are classified as BToA.
                    // 其他所有端点到端点的情况都归类为 BToA。
                    intersectionEdge.hTType = IntersectionEdgeEdgeResult.headTailType.BToA;
                }
            }
            else if (tIsEndpoint || uIsEndpoint)
            {
                // Only one endpoint is involved. It's a T-Shape connection.
                // 只有一个端点参与，是T字形连接。
                intersectionEdge.spsr = IntersectionEdgeEdgeResult.singlePointSpecialRelationship.TShape;
                
                if (tIsEndpoint) // Endpoint of A is on segment B
                {
                    // A的端点在B线段上
                    intersectionEdge.tSType = IntersectionEdgeEdgeResult.tShapeType.TShape_AinB;
                }
                else // Endpoint of B is on segment A
                {
                    // B的端点在A线段上
                    intersectionEdge.tSType = IntersectionEdgeEdgeResult.tShapeType.TShape_BinA;
                }
            }
            else
            {
                // The intersection is in the middle of both segments (a standard 'X' cross).
                // 交点在两条线段的中间（标准的'X'型交叉）。
                intersectionEdge.spsr = IntersectionEdgeEdgeResult.singlePointSpecialRelationship.None;
            }
        }
        
        // Otherwise, the lines intersect, but the segments do not.
        // In this case, intersectionEdge remains null.
    }
    
    // （广义）入出点信息
    public struct generalizedInOutPoint
    {
        // 所连的A的边
        Edge edgeA;
        // 所连的B的边
        Edge edgeB;
    }
    
    // 将A∩B结果存储于C
    static public void Intersection(ref Mesh meshA, ref Mesh meshB, ref Mesh meshC)
    {
        // 首先，拿到两个模型所有的边
        List<Edge> aEdges = new List<Edge>();
        List<Edge> bEdges = new List<Edge>();

        foreach (var loop in meshA.Loops)
        {
            aEdges.AddRange(loop.Edges);
        }
        foreach (var loop in meshB.Loops)
        {
            bEdges.AddRange(loop.Edges);
        } 
        
        // 求出所有的相交情况
        List<IntersectionEdgeEdgeResult> intersectionEdges = new List<IntersectionEdgeEdgeResult>();
        
        // 入点或广义入点（等价入点的边）
        List<IntersectionEdgeEdgeResult> generalizedIntoPoints = new List<IntersectionEdgeEdgeResult>();
        // 出点或广义出点
        List<IntersectionEdgeEdgeResult> generalizedOutoPoints = new List<IntersectionEdgeEdgeResult>();
        
        foreach (var aEdge in aEdges)
        {
            foreach (var bEdge in bEdges)
            {
                IntersectionEdgeEdgeResult currInt = null;
                IntersectionEdgeEdge(aEdge, bEdge, out currInt);
                
                // 真有相交情况
                if (currInt is not null)
                {
                    // 单点、段重合两种情况不同处理
                    if (currInt.iType == IntersectionEdgeEdgeResult.intersectionType.SinglePoint)
                    {
                        // 单点相交，那么判断B边的起点在A边的内外（左右）
                        // 用右手cross，A边向量和A尾-B头向量 叉乘，若值为负，则可以说这是入点，否则是出点
                        float inoutCriterion = Cross(aEdge.getVector, bEdge.VertexA.Point - aEdge.VertexB.Point);

                        if (inoutCriterion > 0)
                        {
                            // 这个点是个入点
                            //intoPoints
                        }
                    }
                }
            }
        }
    }
}
