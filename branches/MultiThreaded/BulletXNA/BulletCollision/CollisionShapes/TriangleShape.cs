/*
 * C# / XNA  port of Bullet (c) 2011 Mark Neale <xexuxjy@hotmail.com>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

using System;
using System.Diagnostics;

using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
    public class TriangleShape : PolyhedralConvexShape , IDisposable
    {

        public TriangleShape()
        {
            m_shapeType = BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE;
        }

        public TriangleShape(IndexedVector3 p0, IndexedVector3 p1, IndexedVector3 p2)
            : this(ref p0, ref p1, ref p2)
        {
        }



	    public TriangleShape(ref IndexedVector3 p0,ref IndexedVector3 p1,ref IndexedVector3 p2) : base ()
        {
		    m_shapeType = BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE;
            m_vertices1[0] = p0;
            m_vertices1[1] = p1;
            m_vertices1[2] = p2;
        }

        public void Initialize(ref IndexedVector3 p0,ref IndexedVector3 p1,ref IndexedVector3 p2)
        {
            m_shapeType = BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE;
            m_vertices1[0] = p0;
            m_vertices1[1] = p1;
            m_vertices1[2] = p2;
        }

        public override void GetPlane(out IndexedVector3 planeNormal, out IndexedVector3 planeSupport, int i)
	    {
            GetPlaneEquation(i, out planeNormal, out planeSupport);
	    }

        public override int GetNumPlanes()
	    {
		    return 1;
	    }

        public void CalcNormal(out IndexedVector3 normal) 
	    {
            normal = IndexedVector3.Cross(m_vertices1[1]-m_vertices1[0],m_vertices1[2]-m_vertices1[0]);
		    normal.Normalize();
	    }

        public virtual void GetPlaneEquation(int i, out IndexedVector3 planeNormal, out IndexedVector3 planeSupport)
	    {
            CalcNormal(out planeNormal);
		    planeSupport = m_vertices1[0];
	    }

        public override void CalculateLocalInertia(float mass, out IndexedVector3 inertia)
	    {
		    Debug.Assert(false);
		    inertia = IndexedVector3.Zero;
	    }

        public override bool IsInside(ref IndexedVector3 pt, float tolerance)
	    {
		    IndexedVector3 normal;
		    CalcNormal(out normal);
		    //distance to plane
            float dist = IndexedVector3.Dot(ref pt, ref normal);
            float planeconst = IndexedVector3.Dot(ref m_vertices1[0], ref normal);
		    dist -= planeconst;
		    if (dist >= -tolerance && dist <= tolerance)
		    {
			    //inside check on edge-planes
			    int i;
			    for (i=0;i<3;i++)
			    {
				    IndexedVector3 pa, pb;
                    GetEdge(i, out pa, out pb);
				    IndexedVector3 edge = pb-pa;
                    IndexedVector3 edgeNormal = edge.Cross(ref normal);
				    edgeNormal.Normalize();
                    float dist2 = IndexedVector3.Dot(ref pt, ref edgeNormal);
                    float edgeConst = IndexedVector3.Dot(ref pa, ref edgeNormal);
				    dist2 -= edgeConst;
				    if (dist2 < -tolerance)
                    {
					    return false;
                    }
			    }
    			
			    return true;
		    }

		    return false;
	    }

        public override String GetName()
        {
            return "Triangle";
        }

        public override int GetNumPreferredPenetrationDirections()
	    {
		    return 2;
	    }

        public override void GetPreferredPenetrationDirection(int index, out IndexedVector3 penetrationVector)
        {
	        CalcNormal(out penetrationVector);
	        if (index > 0)
            {
		        penetrationVector *= -1f;
            }
        }

        public override int GetNumVertices()
        {
            return 3;
        }

        public virtual IndexedVector3[] GetVertexPtr(int i)
        {
            return m_vertices1;
        }

        public override void GetVertex(int i, out IndexedVector3 vert)
        {
            vert = m_vertices1[i];
        }

        public override int GetNumEdges()
        {
            return 3;
        }

        public override void GetEdge(int i, out IndexedVector3 pa, out IndexedVector3 pb)
        {
            GetVertex(i, out pa);
            GetVertex((i + 1) % 3, out pb);
        }

        public override void GetAabb(ref IndexedMatrix trans, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
        {
            GetAabbSlow(ref trans, out aabbMin, out aabbMax);
        }

        public override IndexedVector3 LocalGetSupportingVertexWithoutMargin(ref IndexedVector3 dir)
        {
            IndexedVector3 dots = new IndexedVector3(
                dir.Dot(ref m_vertices1[0]),
                dir.Dot(ref m_vertices1[1]),
                dir.Dot(ref m_vertices1[2]));
            return m_vertices1[MathUtil.MaxAxis(ref dots)];
        }

        public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IndexedVector3[] vectors, IndexedVector4[] supportVerticesOut, int numVectors)
        {
            for (int i = 0; i < numVectors; i++)
            {
                IndexedVector3 dir = vectors[i];
                IndexedVector3 dots = new IndexedVector3(
                    dir.Dot(ref m_vertices1[0]),
                    dir.Dot(ref m_vertices1[1]),
                    dir.Dot(ref m_vertices1[2]));
                supportVerticesOut[i] = new IndexedVector4(m_vertices1[MathUtil.MaxAxis(ref dots)], 0);
            }
        }

        public void Dispose()
        {
            BulletGlobals.TriangleShapePool.Free(this);
        }

        public IndexedVector3[] m_vertices1 = new IndexedVector3[3];
    }
}
