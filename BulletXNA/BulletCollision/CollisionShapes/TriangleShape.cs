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
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletCollision
{
    public class TriangleShape : PolyhedralConvexShape
    {

        public TriangleShape()
        {
            m_shapeType = BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE;
        }

        public TriangleShape(Vector3 p0, Vector3 p1, Vector3 p2)
            : this(ref p0, ref p1, ref p2)
        {
        }



	    public TriangleShape(ref Vector3 p0,ref Vector3 p1,ref Vector3 p2) : base ()
        {
		    m_shapeType = BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE;
            m_vertices1[0] = p0;
            m_vertices1[1] = p1;
            m_vertices1[2] = p2;
        }


        public override void GetPlane(out Vector3 planeNormal, out Vector3 planeSupport, int i)
	    {
            GetPlaneEquation(i, out planeNormal, out planeSupport);
	    }

        public override int GetNumPlanes()
	    {
		    return 1;
	    }

        public void CalcNormal(out Vector3 normal) 
	    {
            normal = Vector3.Cross(m_vertices1[1]-m_vertices1[0],m_vertices1[2]-m_vertices1[0]);
		    normal.Normalize();
	    }

        public virtual void GetPlaneEquation(int i, out Vector3 planeNormal, out Vector3 planeSupport)
	    {
            CalcNormal(out planeNormal);
		    planeSupport = m_vertices1[0];
	    }

        public override void CalculateLocalInertia(float mass, out Vector3 inertia)
	    {
		    Debug.Assert(false);
		    inertia = Vector3.Zero;
	    }

        public override bool IsInside(ref Vector3 pt, float tolerance)
	    {
		    Vector3 normal;
		    CalcNormal(out normal);
		    //distance to plane
            float dist;
            Vector3.Dot(ref pt,ref normal,out dist);
		    float planeconst;
            Vector3.Dot(ref m_vertices1[0],ref normal,out planeconst);
		    dist -= planeconst;
		    if (dist >= -tolerance && dist <= tolerance)
		    {
			    //inside check on edge-planes
			    int i;
			    for (i=0;i<3;i++)
			    {
				    Vector3 pa, pb;
                    GetEdge(i, out pa, out pb);
				    Vector3 edge = pb-pa;
                    Vector3 edgeNormal;
                    Vector3.Cross(ref edge,ref normal,out edgeNormal);
				    edgeNormal.Normalize();
                    float dist2;
                    Vector3.Dot(ref pt, ref edgeNormal,out dist2);
				    float edgeConst;
                    Vector3.Dot(ref pa, ref edgeNormal,out edgeConst);
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

        public override void GetPreferredPenetrationDirection(int index, out Vector3 penetrationVector)
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

        public virtual Vector3[] GetVertexPtr(int i)
        {
            return m_vertices1;
        }

        public override void GetVertex(int i, out Vector3 vert)
        {
            vert = m_vertices1[i];
        }

        public override int GetNumEdges()
        {
            return 3;
        }

        public override void GetEdge(int i, out Vector3 pa, out Vector3 pb)
        {
            GetVertex(i, out pa);
            GetVertex((i + 1) % 3, out pb);
        }

        public override void GetAabb(ref Matrix trans, out Vector3 aabbMin, out Vector3 aabbMax)
        {
            GetAabbSlow(ref trans, out aabbMin, out aabbMax);
        }

        public override Vector3 LocalGetSupportingVertexWithoutMargin(ref Vector3 dir)
	    {
            float a,b,c;
            Vector3.Dot(ref dir, ref m_vertices1[0], out a);
            Vector3.Dot(ref dir, ref m_vertices1[1], out b);
            Vector3.Dot(ref dir, ref m_vertices1[2], out c);
            Vector3 dots = new Vector3(a, b, c);
            return m_vertices1[MathUtil.MaxAxis(ref dots)];
	    }

	    public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors,Vector4[] supportVerticesOut,int numVectors)
	    {
		    for (int i=0;i<numVectors;i++)
		    {
			    Vector3 dir = vectors[i];
                float a, b, c;
                Vector3.Dot(ref dir, ref m_vertices1[0],out a);
                Vector3.Dot(ref dir, ref m_vertices1[1],out b);
                Vector3.Dot(ref dir, ref m_vertices1[2],out c);

                Vector3 dots = new Vector3(a, b, c);
                supportVerticesOut[i] = new Vector4(m_vertices1[MathUtil.MaxAxis(ref dots)],0);
		    }
	    }

        public Vector3[] m_vertices1 = new Vector3[3];
    }
}
