//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using BulletXNA.LinearMath;
//using Microsoft.Xna.Framework;
//using BulletXNA.BulletCollision.CollisionShapes.GImpact;

//namespace BulletXNA.BulletCollision.CollisionShapes.GImpact
//{
////! Structure for collision
//class GIM_TRIANGLE_CONTACT
//{
//    public const int MAX_TRI_CLIPPING = 16;

//    public float m_penetration_depth;
//    public int m_point_count;
//    public Vector4 m_separating_normal;
//    public Vector3[] m_points = new Vector3[MAX_TRI_CLIPPING];

//    public void CopyFrom(GIM_TRIANGLE_CONTACT other)
//    {
//        m_penetration_depth = other.m_penetration_depth;
//        m_separating_normal = other.m_separating_normal;
//        m_point_count = other.m_point_count;
//        int i = m_point_count;
//        while(i-- != 0)
//        {
//            m_points[i] = other.m_points[i];
//        }
//    }

//    GIM_TRIANGLE_CONTACT()
//    {
//    }

//    public GIM_TRIANGLE_CONTACT(GIM_TRIANGLE_CONTACT other)
//    {
//        CopyFrom(other);
//    }

//    //! classify points that are closer
//    public void MergePoints(ref Vector4 plane,float margin, ObjectArray<Vector3> points, int point_count);

//}



//public class PrimitiveTriangle
//{
//    public Vector3[] m_vertices= new Vector3[3];
//    public Vector4 m_plane;
//    public float m_margin;
//    //float m_dummy;
//    public PrimitiveTriangle()
//    {
//        m_margin = 0.01f;
//    }


//    public void BuildTriPlane()
//    {
//        Vector3 normal = Vector3.Cross(m_vertices[1]-m_vertices[0],m_vertices[2]-m_vertices[0]);
//        normal.Normalize();
//        m_plane = new Vector4(normal,Vector3.Dot(m_vertices[0],normal));
//    }

//    //! Test if triangles could collide
//    public bool OverlapTestConservative(PrimitiveTriangle other);

//    //! Calcs the plane which is paralele to the edge and perpendicular to the triangle plane
//    /*!
//    \pre this triangle must have its plane calculated.
//    */
//    public void GetEdgePlane(int edge_index, ref Vector4 plane)
//    {
//        Vector3 e0 = m_vertices[edge_index];
//        Vector3 e1 = m_vertices[(edge_index+1)%3];
//        GeometeryOperations.bt_edge_plane(ref e0,ref e1,ref m_plane,ref plane);
//    }

//    public void ApplyTransform(ref Matrix t)
//    {
//        Vector3.Transform(m_vertices,t);
//        //m_vertices[0] = t(m_vertices[0]);
//        //m_vertices[1] = t(m_vertices[1]);
//        //m_vertices[2] = t(m_vertices[2]);
//    }

//    //! Clips the triangle against this
//    /*!
//    \pre clipped_points must have MAX_TRI_CLIPPING size, and this triangle must have its plane calculated.
//    \return the number of clipped points
//    */
//    public int ClipTriangle(PrimitiveTriangle other, ObjectArray<Vector3> clipped_points );

//    //! Find collision using the clipping method
//    /*!
//    \pre this triangle and other must have their triangles calculated
//    */
//    public bool FindTriangleCollisionClipMethod(PrimitiveTriangle other, GIM_TRIANGLE_CONTACT contacts);
//};



////! Helper class for colliding Bullet Triangle Shapes
///*!
//This class implements a better getAabb method than the previous btTriangleShape class
//*/
//public class TriangleShapeEx: TriangleShape
//{
//    public TriangleShapeEx():base(Vector3.Zero,Vector3.Zero,Vector3.Zero)
//    {
//    }

//    public TriangleShapeEx(ref Vector3 p0,ref Vector3 p1,ref Vector3 p2):	base(ref p0,ref p1,ref p2)
//    {
//    }

//    public TriangleShapeEx(TriangleShapeEx other):	base(ref other.m_vertices1[0],ref other.m_vertices1[1],ref other.m_vertices1[2])
//    {
//    }

//    virtual void GetAabb(ref Matrix t,ref Vector3 aabbMin,ref Vector3 aabbMax)
//    {
//        Vector3 tv0 = Vector3.Transform(m_vertices1[0],t);
//        Vector3 tv1 = Vector3.Transform(m_vertices1[1],t);
//        Vector3 tv2 = Vector3.Transform(m_vertices1[2],t);

//        BoundingBox trianglebox = new BoundingBox(tv0,tv1,tv2,m_collisionMargin);
//        aabbMin = trianglebox.m_min;
//        aabbMax = trianglebox.m_max;
//    }

//    public void ApplyTransform(ref Matrix t)
//    {
//        m_vertices1[0] = Vector3.Transform(m_vertices1[0],t);
//        m_vertices1[1] = Vector3.Transform(m_vertices1[1],t);
//        m_vertices1[2] = Vector3.Transform(m_vertices1[2],t);
//    }

//    public void BuildTriPlane(Vector4 plane)
//    {
//        Vector3 normal = Vector3.Cross(m_vertices1[1]-m_vertices1[0],m_vertices1[2]-m_vertices1[0]);
//        normal.Normalize();
//        plane = new Vector4(normal,Vector3.Dot(m_vertices1[0],normal));
//    }

//    public bool OverlapTestConservative(TriangleShapeEx other);
//};

//}
