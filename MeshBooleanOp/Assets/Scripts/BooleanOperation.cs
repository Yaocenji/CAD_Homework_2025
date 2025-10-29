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
            HeadTail, // 两条线段头尾相连交于一点
            TShape, // 两条线段成T字型交于一点
        }

        public singlePointSpecialRelationship spsr;

        // 头尾相接型，那么可能是A尾和B头重合，也有可能是B尾和A头重合，也可能是头头/尾尾
        public enum headTailType
        {
            HeadTail_AToB, // A尾连B头
            HeadTail_BToA, // B尾连A头
            HeadTail_ABHead, // AB头头连接（相背）
            HeadTail_ABTail, // AB尾尾连接（相向）
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
    static public void IntersectionEdgeEdge(Edge edgeA, Edge edgeB, out IntersectionEdgeEdgeResult intersectionResult)
    {
        // By default, there is no intersection.
        intersectionResult = null;

        // Represent the edges as parametric equations: P = P_start + t * direction
        Vector2 p = edgeA.VertexA.Point;
        Vector2 r = edgeA.VertexB.Point - edgeA.VertexA.Point;
        Vector2 q = edgeB.VertexA.Point;
        Vector2 s = edgeB.VertexB.Point - edgeB.VertexA.Point;

        // Calculate the 2D cross product of the direction vectors
        float r_cross_s = r.x * s.y - r.y * s.x;
        Vector2 q_minus_p = q - p;
        float q_minus_p_cross_r = q_minus_p.x * r.y - q_minus_p.y * r.x;

        // Case 1: The lines are parallel or collinear
        if (Mathf.Abs(r_cross_s) < GeometryConstant.Tolerance)
        {
            // Check if they are also collinear. If not, they are parallel and non-intersecting.
            if (Mathf.Abs(q_minus_p_cross_r) > GeometryConstant.Tolerance)
            {
                return; // Parallel and non-intersecting
            }

            // --- Lines are collinear, now check for segment overlap ---
            float r_dot_r = Vector2.Dot(r, r);

            // Handle case where edgeA is just a point
            if (r_dot_r < GeometryConstant.Tolerance)
            {
                // Check if this point lies on edgeB
                float s_dot_s = Vector2.Dot(s, s);
                if (s_dot_s < GeometryConstant.Tolerance) // Both are points
                {
                    if (Vector2.Distance(p, q) < GeometryConstant.Tolerance)
                    {
                        // Two points at the same location
                        intersectionResult = new IntersectionEdgeEdgeResult();
                        intersectionResult.pType = IntersectionEdgeEdgeResult.parellelType.Parallel;
                        intersectionResult.iType = IntersectionEdgeEdgeResult.intersectionType.SinglePoint;
                        intersectionResult.spsr = IntersectionEdgeEdgeResult.singlePointSpecialRelationship.HeadTail;
                        intersectionResult.hTType =
                            IntersectionEdgeEdgeResult.headTailType.HeadTail_ABHead; // Or any, it's ambiguous
                        intersectionResult.intPoint = p;
                    }

                    return;
                }

                float tmpu = Vector2.Dot(p - q, s) / s_dot_s;
                if (tmpu >= -GeometryConstant.Tolerance && tmpu <= 1 + GeometryConstant.Tolerance)
                {
                    // Point A lies on segment B
                    intersectionResult = new IntersectionEdgeEdgeResult();
                    intersectionResult.pType = IntersectionEdgeEdgeResult.parellelType.Parallel;
                    intersectionResult.iType = IntersectionEdgeEdgeResult.intersectionType.SinglePoint;
                    intersectionResult.intPoint = p;
                    intersectionResult.spsr = IntersectionEdgeEdgeResult.singlePointSpecialRelationship.TShape;
                    intersectionResult.tSType = IntersectionEdgeEdgeResult.tShapeType.TShape_AinB;
                }

                return;
            }

            float t0 = Vector2.Dot(q - p, r) / r_dot_r;
            float t1 = t0 + Vector2.Dot(s, r) / r_dot_r;

            float overlap_start = Mathf.Max(0.0f, Mathf.Min(t0, t1));
            float overlap_end = Mathf.Min(1.0f, Mathf.Max(t0, t1));

            // If the overlap interval is not valid, they are collinear but don't overlap
            if (overlap_start > overlap_end + GeometryConstant.Tolerance)
            {
                return;
            }

            // An intersection exists, so create the result object
            intersectionResult = new IntersectionEdgeEdgeResult();
            intersectionResult.aEdge = edgeA;
            intersectionResult.bEdge = edgeB;
            intersectionResult.pType = IntersectionEdgeEdgeResult.parellelType.Parallel;

            // Check if the overlap is just a single point
            if (Mathf.Abs(overlap_start - overlap_end) < GeometryConstant.Tolerance)
            {
                intersectionResult.iType = IntersectionEdgeEdgeResult.intersectionType.SinglePoint;
                intersectionResult.intPoint = p + overlap_start * r;

                // This is a collinear head-to-tail connection
                intersectionResult.spsr = IntersectionEdgeEdgeResult.singlePointSpecialRelationship.HeadTail;

                bool t0_is_0 = Mathf.Abs(t0) < GeometryConstant.Tolerance;
                bool t0_is_1 = Mathf.Abs(t0 - 1.0f) < GeometryConstant.Tolerance;
                bool t1_is_0 = Mathf.Abs(t1) < GeometryConstant.Tolerance;
                bool t1_is_1 = Mathf.Abs(t1 - 1.0f) < GeometryConstant.Tolerance;

                if (t0_is_1)
                    intersectionResult.hTType = IntersectionEdgeEdgeResult.headTailType.HeadTail_AToB; // A_end touches B_start
                else if (t1_is_0)
                    intersectionResult.hTType = IntersectionEdgeEdgeResult.headTailType.HeadTail_BToA; // B_end touches A_start
                else if (t0_is_0)
                    intersectionResult.hTType =
                        IntersectionEdgeEdgeResult.headTailType.HeadTail_ABHead; // A_start touches B_start
                else if (t1_is_1)
                    intersectionResult.hTType = IntersectionEdgeEdgeResult.headTailType.HeadTail_ABTail; // A_end touches B_end
            }
            else // The overlap is a line segment
            {
                intersectionResult.iType = IntersectionEdgeEdgeResult.intersectionType.SegmentOverlap;
                intersectionResult.intEdge.VertexA.Point = p + overlap_start * r;
                intersectionResult.intEdge.VertexB.Point = p + overlap_end * r;
            }

            return;
        }

        // Case 2: Lines are not parallel and intersect at a single point
        // Solve for parameters t and u: p + t*r = q + u*s
        float t = (q_minus_p.x * s.y - q_minus_p.y * s.x) / r_cross_s;
        float u = q_minus_p_cross_r / r_cross_s;

        // Check if the intersection point lies on both segments (t and u are between 0 and 1)
        if (t >= -GeometryConstant.Tolerance && t <= 1 + GeometryConstant.Tolerance &&
            u >= -GeometryConstant.Tolerance && u <= 1 + GeometryConstant.Tolerance)
        {
            // An intersection exists, create and populate the result object
            intersectionResult = new IntersectionEdgeEdgeResult();
            intersectionResult.aEdge = edgeA;
            intersectionResult.bEdge = edgeB;
            intersectionResult.pType = IntersectionEdgeEdgeResult.parellelType.NonParallel;
            intersectionResult.iType = IntersectionEdgeEdgeResult.intersectionType.SinglePoint;
            intersectionResult.intPoint = p + t * r;

            // --- Now, classify the single point intersection ---
            bool a_is_headpoint = Mathf.Abs(t) < GeometryConstant.Tolerance;
            bool a_is_tailpoint = Mathf.Abs(t - 1.0f) < GeometryConstant.Tolerance;
            bool b_is_headpoint = Mathf.Abs(u) < GeometryConstant.Tolerance;
            bool b_is_tailpoint = Mathf.Abs(u - 1.0f) < GeometryConstant.Tolerance;
            
            bool a_is_endpoint = a_is_headpoint || a_is_tailpoint;
            bool b_is_endpoint = b_is_headpoint || b_is_tailpoint;

            if (a_is_endpoint && b_is_endpoint)
            {
                intersectionResult.spsr = IntersectionEdgeEdgeResult.singlePointSpecialRelationship.HeadTail;

                bool a_is_head = Mathf.Abs(t) < GeometryConstant.Tolerance;
                bool b_is_head = Mathf.Abs(u) < GeometryConstant.Tolerance;

                if (!a_is_head && b_is_head)
                    intersectionResult.hTType = IntersectionEdgeEdgeResult.headTailType.HeadTail_AToB; // A-tail to B-head
                else if (a_is_head && !b_is_head)
                    intersectionResult.hTType = IntersectionEdgeEdgeResult.headTailType.HeadTail_BToA; // B-tail to A-head
                else if (a_is_head && b_is_head)
                    intersectionResult.hTType = IntersectionEdgeEdgeResult.headTailType.HeadTail_ABHead; // Heads connect
                else intersectionResult.hTType = IntersectionEdgeEdgeResult.headTailType.HeadTail_ABTail; // Tails connect
            }
            else if (a_is_endpoint && !b_is_endpoint)
            {
                intersectionResult.spsr = IntersectionEdgeEdgeResult.singlePointSpecialRelationship.TShape;
                if (a_is_tailpoint)
                    intersectionResult.tSType =
                        IntersectionEdgeEdgeResult.tShapeType.TShape_AinB; // Tail of A is on segment B
                else if (a_is_headpoint)
                    intersectionResult.tSType =
                        IntersectionEdgeEdgeResult.tShapeType.TShape_AoutB; // Head of A is on segment B
            }
            else if (!a_is_endpoint && b_is_endpoint)
            {
                intersectionResult.spsr = IntersectionEdgeEdgeResult.singlePointSpecialRelationship.TShape;
                if (b_is_tailpoint)
                    intersectionResult.tSType =
                        IntersectionEdgeEdgeResult.tShapeType.TShape_BinA; // Tail of B is on segment A
                else if (b_is_headpoint)
                    intersectionResult.tSType = 
                        IntersectionEdgeEdgeResult.tShapeType.TShape_BoutA; // Head of B is on segment A
            }
            else
            {
                // Standard "X" intersection where the point is not an endpoint for either segment
                intersectionResult.spsr = IntersectionEdgeEdgeResult.singlePointSpecialRelationship.None;
            }
        }
        // If the intersection point is not on both segments, intersectionResult remains null
    }

    // 入出点信息
    public class generalizedInOutPoint
    {
        // 所连的A的边，可能有两个，那么按照前后的顺序放
        Edge[] edgeA;

        // 所连的B的边，可能有两个，那么按照前后的顺序放
        Edge[] edgeB;

        public generalizedInOutPoint()
        {
            edgeA = new Edge[2];
            edgeB = new Edge[2];
        }
    }

    static public void getAllEdgeIntersectionOfTwoMesh(ref Mesh meshA, ref Mesh meshB, ref List<IntersectionEdgeEdgeResult> results)
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
        foreach (var aEdge in aEdges)
        {
            foreach (var bEdge in bEdges)
            {
                IntersectionEdgeEdgeResult currInt = null;
                IntersectionEdgeEdge(aEdge, bEdge, out currInt);

                // 真有相交情况
                if (currInt is not null)
                {
                    results.Add(currInt);
                }
            }
        }
    }

    // 将A∩B结果存储于C
    static public void Intersection(ref Mesh meshA, ref Mesh meshB, ref Mesh meshC)
    {
        
        // 拿到所有的交点信息，在本算法设计中，重合边可以放弃
        List<IntersectionEdgeEdgeResult> intersectionResults = new List<IntersectionEdgeEdgeResult>();

        // 求出所有的相交情况
        getAllEdgeIntersectionOfTwoMesh(ref meshA, ref meshB, ref intersectionResults);

        // 入点或广义入点（等价入点的边）
        List<generalizedInOutPoint> inPoints = new List<generalizedInOutPoint>();
        // 出点或广义出点
        List<generalizedInOutPoint> outPoints = new List<generalizedInOutPoint>();
        
        // 开始处理所有的求交结果
        // 第一步：放弃掉所有的重合类
        List<IntersectionEdgeEdgeResult> deletes = new List<IntersectionEdgeEdgeResult>();
        foreach (var result in intersectionResults)
        {
            if (result.iType == IntersectionEdgeEdgeResult.intersectionType.SegmentOverlap)
                deletes.Add(result);
        }
        foreach (var result in deletes)
        {
            intersectionResults.Remove(result);
        }
        // 处理过程中的标记
        bool[] visit = new bool[intersectionResults.Count];
        
        foreach (var result in intersectionResults)
        {
            Debug.Log(result.iType + "  " + result.pType + "  " + result.spsr + "  " + result.hTType + "  " + result.tSType
                      + "\n" + result.intPoint + "  " + result.intEdge.VertexA.Point + "  " + result.intEdge.VertexB.Point);
        }


        /*
         * // 单点、段重合两种情况不同处理
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
         */
    }
}