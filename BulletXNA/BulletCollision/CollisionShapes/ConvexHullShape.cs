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
using System.Collections.Generic;
using System.Diagnostics;

using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
    public class ConvexHullShape : PolyhedralConvexAabbCachingShape
    {
        public ConvexHullShape(IList<IndexedVector3> points, int numPoints)
        {
            m_unscaledPoints = new List<IndexedVector3>(numPoints);
            m_shapeType = BroadphaseNativeTypes.CONVEX_HULL_SHAPE_PROXYTYPE;
            for (int i = 0; i < numPoints; ++i)
            {
                m_unscaledPoints.Add(points[i]);
            }
            RecalcLocalAabb();
        }

        
        public ConvexHullShape(IList<float>points,int numPoints)
        {
            m_unscaledPoints = new List<IndexedVector3>(numPoints);
            m_shapeType = BroadphaseNativeTypes.CONVEX_HULL_SHAPE_PROXYTYPE;
            for (int i = 0; i < numPoints/3; ++i)
            {
                m_unscaledPoints.Add(new IndexedVector3(points[i * 3], points[i * 3] + 1, points[i * 3] + 2));
            }
            RecalcLocalAabb();
        }

        public void AddPoint(ref IndexedVector3 point)
        {
            m_unscaledPoints.Add(point);
            RecalcLocalAabb();
        }

	
	    public IList<IndexedVector3> GetUnscaledPoints()
	    {
		    return m_unscaledPoints;
	    }


	    public IndexedVector3 GetScaledPoint(int i)
	    {
		    return m_unscaledPoints[i] * m_localScaling;
	    }

	    public int GetNumPoints()
	    {
		    return m_unscaledPoints.Count;
	    }

        public override IndexedVector3 LocalGetSupportingVertexWithoutMargin(ref IndexedVector3 vec)
        {
	        IndexedVector3 supVec = IndexedVector3.Zero;
	        float newDot,maxDot = float.MinValue;
#if DEBUG
			if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugConvexHull)
			{
				BulletGlobals.g_streamWriter.WriteLine("localGetSupportingVertexWithoutMargin");
				MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "vec", vec);
				for (int i = 0; i < m_unscaledPoints.Count; ++i)
				{
					MathUtil.PrintVector3(BulletGlobals.g_streamWriter, m_unscaledPoints[i]);
				}
			}
#endif

	        for (int i=0;i<m_unscaledPoints.Count;i++)
	        {
		        IndexedVector3 vtx = m_unscaledPoints[i] * m_localScaling;

		        newDot = IndexedVector3.Dot(vec,vtx);
		        if (newDot > maxDot)
		        {
			        maxDot = newDot;
			        supVec = vtx;
		        }
	        }
	        return supVec;
        }
        
        public override IndexedVector3 LocalGetSupportingVertex(ref IndexedVector3 vec)
        {
	        IndexedVector3 supVertex = LocalGetSupportingVertexWithoutMargin(ref vec);

	        if ( GetMargin()!=0f)
	        {
		        IndexedVector3 vecnorm = vec;
                if (vecnorm.LengthSquared() < (MathUtil.SIMD_EPSILON * MathUtil.SIMD_EPSILON))
		        {
			        vecnorm = new IndexedVector3(-1f);
		        } 
		        vecnorm.Normalize();
		        supVertex += GetMargin() * vecnorm;
	        }
	        return supVertex;
        }

        public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IndexedVector3[] vectors, IndexedVector4[] supportVerticesOut, int numVectors)
        {
	        float newDot;
	        //use 'w' component of supportVerticesOut?
	        {
		        for (int i=0;i<numVectors;i++)
		        {
                    IndexedVector4 temp = supportVerticesOut[i];
					temp.W = -MathUtil.BT_LARGE_FLOAT;
			        supportVerticesOut[i] = temp;
		        }
	        }
	        for (int i=0;i<m_unscaledPoints.Count;i++)
	        {
		        IndexedVector3 vtx = GetScaledPoint(i);

		        for (int j=0;j<numVectors;j++)
		        {
			        IndexedVector3 vec = vectors[j];
        			
			        newDot = IndexedVector3.Dot(vec,vtx);
			        if (newDot > supportVerticesOut[j].W)
			        {
				        //WARNING: don't swap next lines, the w component would get overwritten!
				        supportVerticesOut[j] = new IndexedVector4(vtx,newDot);
			        }
		        }
	        }
        }
	

    	//debugging
	    public override String GetName()
        {
            return "Convex";
        }

        public override int GetNumVertices()
        {
            return m_unscaledPoints.Count;
        }
        public override int GetNumEdges()
        {
            return m_unscaledPoints.Count;
        }
        public override void GetEdge(int i, out IndexedVector3 pa, out IndexedVector3 pb)
        {
            int index0 = i % m_unscaledPoints.Count;
            int index1 = (i + 1) % m_unscaledPoints.Count;
            pa = GetScaledPoint(index0);
            pb = GetScaledPoint(index1);
        }

        public override void GetVertex(int i, out IndexedVector3 vtx)
        {
            vtx = GetScaledPoint(i);
        }

        public override int GetNumPlanes()
        {
            return 0;
        }

        public override void GetPlane(out IndexedVector3 planeNormal, out IndexedVector3 planeSupport, int i)
        {
            Debug.Assert(false);
            planeNormal = IndexedVector3.Zero;
            planeSupport = IndexedVector3.Zero;
        }

        public override bool IsInside(ref IndexedVector3 pt, float tolerance)
        {
            Debug.Assert(false);
            return false;
        }

	    ///in case we receive negative scaling
        public override void SetLocalScaling(ref IndexedVector3 scaling)
        {
            m_localScaling = scaling;
            RecalcLocalAabb();
        }


        public void Project(ref IndexedMatrix trans, ref IndexedVector3 dir, ref float min, ref float max)
        {
        #if true
	        min = float.MaxValue;
	        max = float.MinValue;
            IndexedVector3 witnesPtMin;
            IndexedVector3 witnesPtMax;

	        int numVerts = m_unscaledPoints.Count;
	        for(int i=0;i<numVerts;i++)
	        {
                IndexedVector3 vtx = m_unscaledPoints[i] * m_localScaling;
                IndexedVector3 pt = trans * vtx;
		        float dp = pt.Dot(dir);
		        if(dp < min)	
		        {
			        min = dp;
			        witnesPtMin = pt;
		        }
		        if(dp > max)	
		        {
			        max = dp;
			        witnesPtMax=pt;
		        }
	        }
        #else
	        btVector3 localAxis = dir*trans.getBasis();
	        btVector3 vtx1 = trans(localGetSupportingVertex(localAxis));
	        btVector3 vtx2 = trans(localGetSupportingVertex(-localAxis));

	        min = vtx1.dot(dir);
	        max = vtx2.dot(dir);
        #endif

	        if(min>max)
	        {
		        float tmp = min;
		        min = max;
		        max = tmp;
	        }


        }


        private IList<IndexedVector3> m_unscaledPoints;
    }

}
