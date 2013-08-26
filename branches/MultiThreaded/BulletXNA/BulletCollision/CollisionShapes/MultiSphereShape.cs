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

using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
    public class MultiSphereShape : ConvexInternalAabbCachingShape
    {
	    public MultiSphereShape (IList<IndexedVector3> positions,IList<float> radi,int numSpheres)
        {
            m_shapeType = BroadphaseNativeTypes.MULTI_SPHERE_SHAPE_PROXYTYPE;
            //float startMargin = float.MaxValue;

            for (int i = 0; i < numSpheres; i++)
            {
                m_localPositionArray.Add(positions[i]);
                m_radiArray.Add(radi[i]);
            }
            RecalcLocalAabb();
        }

	    ///CollisionShape Interface
        public override void CalculateLocalInertia(float mass, out IndexedVector3 inertia)
        {
            IndexedVector3 localAabbMin;
            IndexedVector3 localAabbMax;
	        GetCachedLocalAabb(out localAabbMin,out localAabbMax);
	        IndexedVector3 halfExtents = (localAabbMax-localAabbMin)*0.5f;

	        float lx=2f*(halfExtents.X);
	        float ly=2f*(halfExtents.Y);
	        float lz=2f*(halfExtents.Z);

	        inertia = new IndexedVector3(mass/(12.0f) * (ly*ly + lz*lz),
					        mass/(12.0f) * (lx*lx + lz*lz),
					        mass/(12.0f) * (lx*lx + ly*ly));
        }

    	/// btConvexShape Interface
        public override IndexedVector3 LocalGetSupportingVertexWithoutMargin(ref IndexedVector3 vec0)
        {
	        int i;
	        IndexedVector3 supVec = IndexedVector3.Zero;

	        float maxDot = float.MinValue;

	        IndexedVector3 vec = vec0;
	        float lenSqr = vec.LengthSquared();
	        if (lenSqr < (MathUtil.SIMD_EPSILON*MathUtil.SIMD_EPSILON))
	        {
		        vec = new IndexedVector3(1,0,0);
	        } 
            else
	        {
                float rlen = (1.0f) / (float)Math.Sqrt(lenSqr);
                vec *= rlen;
                //vec.Normalize();
            }

	        IndexedVector3 vtx = IndexedVector3.Zero;
	        float newDot;

            int numSpheres = m_localPositionArray.Count;
	        for (i=0;i<numSpheres;i++)
	        {
                IndexedVector3 pos = m_localPositionArray[i];
                float rad = m_radiArray[i];
                vtx = (pos) + vec * m_localScaling * (rad) - vec * GetMargin();
		        newDot = IndexedVector3.Dot(vec,vtx);
		        if (newDot > maxDot)
		        {
			        maxDot = newDot;
			        supVec = vtx;
		        }
	        }
	        return supVec;
        }

        public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IndexedVector3[] vectors, IndexedVector4[] supportVerticesOut, int numVectors)
        {
	        for (int j=0;j<numVectors;j++)
	        {
	            float maxDot = float.MinValue;

		        IndexedVector3 vec = vectors[j];

	            IndexedVector3 vtx = IndexedVector3.Zero;
	            float newDot;
                int numSpheres = m_localPositionArray.Count;
                for (int i=0;i<numSpheres;i++)
                {
                    IndexedVector3 pos = m_localPositionArray[i];
                    float rad = m_radiArray[i];
                    vtx = (pos) + vec * m_localScaling * (rad) - vec * GetMargin();
	                newDot = IndexedVector3.Dot(vec,vtx);
	                if (newDot > maxDot)
	                {
		                maxDot = newDot;
		                supportVerticesOut[j] = new IndexedVector4(vtx,0);
	                }
                }
            }
        }
	
	    public int	GetSphereCount()
	    {
		    return m_localPositionArray.Count;
	    }

	    public IndexedVector3 GetSpherePosition(int index)
	    {
		    return m_localPositionArray[index];
	    }

	    public float GetSphereRadius(int index)
	    {
		    return m_radiArray[index];
	    }


	    public override String GetName()
	    {
		    return "MultiSphere";
	    }

        private IList<IndexedVector3> m_localPositionArray = new List<IndexedVector3>();
        private IList<float> m_radiArray = new List<float>();
        private IndexedVector3 m_inertiaHalfExtents;
    }

}
