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
using BulletXNA.BulletCollision.BroadphaseCollision;
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletCollision.CollisionShapes
{
    public class ConvexHullShape : PolyhedralConvexAabbCachingShape
    {
        public ConvexHullShape(IList<Vector3> points, int numPoints)
        {
            m_unscaledPoints = new List<Vector3>(numPoints);
            m_shapeType = BroadphaseNativeTypes.CONVEX_HULL_SHAPE_PROXYTYPE;
            for (int i = 0; i < numPoints; ++i)
            {
                m_unscaledPoints.Add(points[i]);
            }
            RecalcLocalAabb();
        }

        
        public ConvexHullShape(IList<float>points,int numPoints)
        {
            m_unscaledPoints = new List<Vector3>(numPoints);
            m_shapeType = BroadphaseNativeTypes.CONVEX_HULL_SHAPE_PROXYTYPE;
            for (int i = 0; i < numPoints/3; ++i)
            {
                m_unscaledPoints.Add(new Vector3(points[i * 3], points[i * 3] + 1, points[i * 3] + 2));
            }
            RecalcLocalAabb();
        }

        public void AddPoint(ref Vector3 point)
        {
            m_unscaledPoints.Add(point);
            RecalcLocalAabb();
        }

	
	    public IList<Vector3> GetUnscaledPoints()
	    {
		    return m_unscaledPoints;
	    }


	    public Vector3 GetScaledPoint(int i)
	    {
		    return m_unscaledPoints[i] * m_localScaling;
	    }

	    public int GetNumPoints()
	    {
		    return m_unscaledPoints.Count;
	    }

        public override Vector3 LocalGetSupportingVertexWithoutMargin(ref Vector3 vec0)
        {
	        Vector3 supVec = Vector3.Zero;
	        float newDot,maxDot = float.MinValue;

	        Vector3 vec = vec0;
	        float lenSqr = vec.LengthSquared();
	        if (lenSqr < 0.0001f)
	        {
		        vec = Vector3.Right;
	        } else
	        {
                float rlen = (1.0f) / (float)Math.Sqrt(lenSqr);
                vec *= rlen;

                vec.Normalize();
            }

			if (BulletGlobals.g_streamWriter != null && debugConvexHull)
			{
				BulletGlobals.g_streamWriter.WriteLine("localGetSupportingVertexWithoutMargin");
				MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "vec", vec);
				for (int i = 0; i < m_unscaledPoints.Count; ++i)
				{
					MathUtil.PrintVector3(BulletGlobals.g_streamWriter, m_unscaledPoints[i]);
				}
			}


	        for (int i=0;i<m_unscaledPoints.Count;i++)
	        {
		        Vector3 vtx = m_unscaledPoints[i] * m_localScaling;

		        newDot = Vector3.Dot(vec,vtx);
		        if (newDot > maxDot)
		        {
			        maxDot = newDot;
			        supVec = vtx;
		        }
	        }
	        return supVec;
        }
        
        public override Vector3 LocalGetSupportingVertex(ref Vector3 vec)
        {
	        Vector3 supVertex = LocalGetSupportingVertexWithoutMargin(ref vec);

	        if ( GetMargin()!=0f)
	        {
		        Vector3 vecnorm = vec;
                if (vecnorm.LengthSquared() < (MathUtil.SIMD_EPSILON * MathUtil.SIMD_EPSILON))
		        {
			        vecnorm = new Vector3(-1f);
		        } 
		        vecnorm.Normalize();
		        supVertex += GetMargin() * vecnorm;
	        }
	        return supVertex;
        }

        public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IList<Vector3> vectors, IList<Vector4> supportVerticesOut, int numVectors)
        {
	        float newDot;
	        //use 'w' component of supportVerticesOut?
	        {
		        for (int i=0;i<numVectors;i++)
		        {
                    Vector4 temp = supportVerticesOut[i];
					temp.W = -MathUtil.BT_LARGE_FLOAT;
			        supportVerticesOut[i] = temp;
		        }
	        }
	        for (int i=0;i<m_unscaledPoints.Count;i++)
	        {
		        Vector3 vtx = GetScaledPoint(i);

		        for (int j=0;j<numVectors;j++)
		        {
			        Vector3 vec = vectors[j];
        			
			        newDot = Vector3.Dot(vec,vtx);
			        if (newDot > supportVerticesOut[j].W)
			        {
				        //WARNING: don't swap next lines, the w component would get overwritten!
				        supportVerticesOut[j] = new Vector4(vtx,newDot);
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
        public override void GetEdge(int i, out Vector3 pa, out Vector3 pb)
        {
            int index0 = i % m_unscaledPoints.Count;
            int index1 = (i + 1) % m_unscaledPoints.Count;
            pa = GetScaledPoint(index0);
            pb = GetScaledPoint(index1);
        }

        public override void GetVertex(int i, out Vector3 vtx)
        {
            vtx = GetScaledPoint(i);
        }

        public override int GetNumPlanes()
        {
            return 0;
        }

        public override void GetPlane(out Vector3 planeNormal, out Vector3 planeSupport, int i)
        {
            Debug.Assert(false);
            planeNormal = Vector3.Zero;
            planeSupport = Vector3.Zero;
        }

        public override bool IsInside(ref Vector3 pt, float tolerance)
        {
            Debug.Assert(false);
            return false;
        }

	    ///in case we receive negative scaling
        public override void SetLocalScaling(ref Vector3 scaling)
        {
            m_localScaling = scaling;
            RecalcLocalAabb();
        }

        private IList<Vector3> m_unscaledPoints;
		public static bool debugConvexHull = true;
    }

}
