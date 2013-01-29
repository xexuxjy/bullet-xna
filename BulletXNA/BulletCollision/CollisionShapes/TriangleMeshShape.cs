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
    public class TriangleMeshShape : ConcaveShape
    {
        public TriangleMeshShape(StridingMeshInterface meshInterface)
        {
            m_inConstructor = true;
            m_meshInterface = meshInterface;
            m_shapeType = BroadphaseNativeTypes.TRIANGLE_MESH_SHAPE_PROXYTYPE;
            if (meshInterface.HasPremadeAabb())
            {
                meshInterface.GetPremadeAabb(out m_localAabbMin, out m_localAabbMax);
            }
            else
            {
                RecalcLocalAabb();
            }
            m_inConstructor = false;
        }


        public virtual IndexedVector3 LocalGetSupportingVertex(ref IndexedVector3 vec)
        {
	        IndexedVector3 supportVertex;

            IndexedMatrix ident = IndexedMatrix.Identity;

	        SupportVertexCallback supportCallback = new SupportVertexCallback(ref vec,ref ident);

	        IndexedVector3 aabbMax = MathUtil.MAX_VECTOR;
            IndexedVector3 aabbMin = MathUtil.MIN_VECTOR;

            // URGGHHH!
            if (m_inConstructor)
            {
                this.ProcessAllTrianglesCtor(supportCallback, ref aabbMin, ref aabbMax);
            }
            else
            {
                ProcessAllTriangles(supportCallback, ref aabbMin, ref aabbMax);
            }
            
	        supportVertex = supportCallback.GetSupportVertexLocal();
            supportCallback.Cleanup();
	        return supportVertex;

        }

	    public virtual IndexedVector3 LocalGetSupportingVertexWithoutMargin(ref IndexedVector3 vec)
	    {
		    Debug.Assert(false);
		    return LocalGetSupportingVertex(ref vec);
	    }

	    public void	RecalcLocalAabb()
        {
		    IndexedVector3 vec = new IndexedVector3(1,0,0);
		    IndexedVector3 tmp = LocalGetSupportingVertex(ref vec);
            m_localAabbMax.X = tmp.X + m_collisionMargin;
            vec = new IndexedVector3(-1, 0, 0);
            tmp = LocalGetSupportingVertex(ref vec);
            m_localAabbMin.X = tmp.X - m_collisionMargin;

            vec = new IndexedVector3(0, 1, 0);
            tmp = LocalGetSupportingVertex(ref vec);
            m_localAabbMax.Y = tmp.Y + m_collisionMargin;
            vec = new IndexedVector3(0, -1, 0);
            tmp = LocalGetSupportingVertex(ref vec);
            m_localAabbMin.Y = tmp.Y - m_collisionMargin;

            vec = new IndexedVector3(0, 0, 1);
            tmp = LocalGetSupportingVertex(ref vec);
            m_localAabbMax.Z = tmp.Z + m_collisionMargin;
            vec = new IndexedVector3(0, 0, -1);
            tmp = LocalGetSupportingVertex(ref vec);
            m_localAabbMin.Z = tmp.Z - m_collisionMargin;
        }

        public override void GetAabb(ref IndexedMatrix trans, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
        {
            IndexedVector3 localHalfExtents = 0.5f * (m_localAabbMax - m_localAabbMin);
            float margin = GetMargin();
            localHalfExtents += new IndexedVector3(margin);
            IndexedVector3 localCenter = 0.5f * (m_localAabbMax + m_localAabbMin);

            IndexedBasisMatrix abs_b = trans._basis.Absolute();

            IndexedVector3 center = trans * localCenter;

            IndexedVector3 extent = new IndexedVector3(abs_b._el0.Dot(ref localHalfExtents),
                   abs_b._el1.Dot(ref localHalfExtents),
                  abs_b._el2.Dot(ref localHalfExtents));

            aabbMin = center - extent;
            aabbMax = center + extent;

        }
        // yuck yuck yuck
        public void ProcessAllTrianglesCtor(ITriangleCallback callback, ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax)
        {
            FilteredCallback filterCallback = new FilteredCallback(callback, ref aabbMin, ref aabbMax);
            m_meshInterface.InternalProcessAllTriangles(filterCallback, ref aabbMin, ref aabbMax);
            filterCallback.Cleanup();
        }


        public override void ProcessAllTriangles(ITriangleCallback callback, ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax)
        {
        	FilteredCallback filterCallback = new FilteredCallback(callback,ref aabbMin,ref aabbMax);
            m_meshInterface.InternalProcessAllTriangles(filterCallback,ref aabbMin,ref aabbMax);
            filterCallback.Cleanup();
        }

	    public override void CalculateLocalInertia(float mass, out IndexedVector3 inertia)
        {
	        //moving concave objects not supported
	        Debug.Assert(false);
	        inertia = IndexedVector3.Zero;
        }

        public override void SetLocalScaling(ref IndexedVector3 scaling)
        {
            m_meshInterface.SetScaling(ref scaling);
            RecalcLocalAabb();
        }

        public override IndexedVector3 GetLocalScaling()
        {
            return m_meshInterface.GetScaling();
        }
	
	    public StridingMeshInterface GetMeshInterface()
	    {
		    return m_meshInterface;
	    }

	    public IndexedVector3 GetLocalAabbMin()
	    {
		    return m_localAabbMin;
	    }

        public IndexedVector3 GetLocalAabbMax()
	    {
		    return m_localAabbMax;
	    }

	    //debugging
	    public override String GetName()
        {
            return "TRIANGLEMESH";
        }

        private bool m_inConstructor;  // hacky attempt to get correct callback in construction
        protected IndexedVector3 m_localAabbMin;
        protected IndexedVector3 m_localAabbMax;
        protected StridingMeshInterface m_meshInterface;
    }


    public class SupportVertexCallback : ITriangleCallback
    {

	    private IndexedVector3 m_supportVertexLocal;
        public IndexedMatrix m_worldTrans;
	    public float m_maxDot;
	    public IndexedVector3 m_supportVecLocal;

        public virtual bool graphics()
        {
            return false;
        }


	    public SupportVertexCallback(ref IndexedVector3 supportVecWorld,ref IndexedMatrix trans)
	    {
            m_supportVertexLocal = IndexedVector3.Zero;
            m_worldTrans = trans;
            m_maxDot = -MathUtil.BT_LARGE_FLOAT;
            m_supportVecLocal = supportVecWorld * m_worldTrans._basis;
            //m_supportVecLocal = IndexedVector3.TransformNormal(supportVecWorld, m_worldTrans);
	    }

	    public virtual void ProcessTriangle(IndexedVector3[] triangle,int partId, int triangleIndex)
	    {
		    for (int i=0;i<3;i++)
		    {
                float dot = IndexedVector3.Dot(ref m_supportVecLocal, ref triangle[i]);
			    if (dot > m_maxDot)
			    {
				    m_maxDot = dot;
				    m_supportVertexLocal = triangle[i];
			    }
		    }
	    }

	    public IndexedVector3 GetSupportVertexWorldSpace()
	    {
            return m_worldTrans * m_supportVertexLocal;
	    }

	    public IndexedVector3 GetSupportVertexLocal()
	    {
		    return m_supportVertexLocal;
	    }

        public virtual void Cleanup()
        {
        }
    }

    public class FilteredCallback : IInternalTriangleIndexCallback
	{
		public ITriangleCallback m_callback;
		public IndexedVector3 m_aabbMin;
		public IndexedVector3 m_aabbMax;

		public FilteredCallback(ITriangleCallback callback,ref IndexedVector3 aabbMin,ref IndexedVector3 aabbMax)
		{
            m_callback = callback;
            m_aabbMin = aabbMin;
            m_aabbMax = aabbMax;
        }

		public virtual void InternalProcessTriangleIndex(IndexedVector3[] triangle,int partId,int triangleIndex)
		{

            //if (BulletGlobals.gDebugDraw != null)
            //{
            //    if ((int)(BulletGlobals.gDebugDraw.GetDebugMode() & DebugDrawModes.DBG_DrawNormals) != 0)
            //    {
            //        IndexedVector3 wv0, wv1, wv2;
            //        wv0 = triangle[0];
            //        wv1 = triangle[1];
            //        wv2 = triangle[2];


            //        IndexedVector3 center = (wv0 + wv1 + wv2) * (1f / 3f);
            //        IndexedVector3 normal = (wv1 - wv0).Cross(wv2 - wv0);
            //        normal.Normalize();
            //        IndexedVector3 normalColor = new IndexedVector3(1, 0, 1);
            //        BulletGlobals.gDebugDraw.DrawLine(center, center + normal, normalColor);
            //    }
            //}



            if (AabbUtil2.TestTriangleAgainstAabb2(triangle, ref m_aabbMin, ref m_aabbMax))
            {
                //check aabb in triangle-space, before doing this
                m_callback.ProcessTriangle(triangle, partId, triangleIndex);
            }
            //else
            //{
            //    AabbUtil2.TestTriangleAgainstAabb2(triangle, ref m_aabbMin, ref m_aabbMax);
            //}
			
		}
        public virtual void Cleanup()
        {
        }

	}
}
