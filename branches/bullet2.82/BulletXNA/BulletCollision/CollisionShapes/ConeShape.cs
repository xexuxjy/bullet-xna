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
    public class ConeShape : ConvexInternalShape
    {

	    public ConeShape (float radius,float height)
        {
            m_radius = radius;
            m_height = height;
            m_shapeType = BroadphaseNativeTypes.CONE_SHAPE_PROXYTYPE;
            SetConeUpIndex(1);
            m_sinAngle = (float)(m_radius / Math.Sqrt(m_radius * m_radius + m_height * m_height));

        }

        public override IndexedVector3 LocalGetSupportingVertex(ref IndexedVector3 vec)
        {
            return ConeLocalSupport(ref vec);
        }

        public override IndexedVector3 LocalGetSupportingVertexWithoutMargin(ref IndexedVector3 vec)
        {
            return ConeLocalSupport(ref vec);
        }
		public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IndexedVector3[] vectors, IndexedVector4[] supportVerticesOut, int numVectors)
        {
	        for (int i=0;i<numVectors;i++)
	        {
		        IndexedVector3 vec = vectors[i];
		        supportVerticesOut[i] = new IndexedVector4(ConeLocalSupport(ref vec),0);
	        }
        }

        public override void	SetLocalScaling(ref IndexedVector3 scaling)
        {
	        int axis = m_coneIndices[1];
	        int r1 = m_coneIndices[0];
	        int r2 = m_coneIndices[2];
            m_height *= scaling[axis] / m_localScaling[axis];
            m_radius *= (scaling[r1] / m_localScaling[r1] + scaling[r2] / m_localScaling[r2]) / 2;
            m_sinAngle = (m_radius / (float)Math.Sqrt(m_radius * m_radius + m_height * m_height));
	        base.SetLocalScaling(ref scaling);
        }


	    public float GetRadius()  { return m_radius;}
	    public float GetHeight()  { return m_height;}


	    public override void CalculateLocalInertia(float mass, out IndexedVector3 inertia) 
	    {
            IndexedMatrix identity = IndexedMatrix.Identity;
		    IndexedVector3 aabbMin;
            IndexedVector3 aabbMax;
		    GetAabb(ref identity,out aabbMin,out aabbMax);

		    IndexedVector3 halfExtents = (aabbMax-aabbMin)*0.5f;

		    float margin = GetMargin();

            float lx=2f*(halfExtents.X+margin);
            float ly=2f*(halfExtents.Y+margin);
            float lz=2f*(halfExtents.Z+margin);
            float x2 = lx*lx;
            float y2 = ly*ly;
            float z2 = lz*lz;
            float scaledmass = mass * 0.08333333f;

            inertia = scaledmass * (new IndexedVector3(y2+z2,x2+z2,x2+y2));
	    }


	    public override String GetName() 
	    {
		    return "Cone";
	    }
    		
	    ///choose upAxis index
        public void SetConeUpIndex(int upIndex)
        {
            switch (upIndex)
            {
                case 0:
                    {
                        m_coneIndices[0] = 1;
                        m_coneIndices[1] = 0;
                        m_coneIndices[2] = 2;
                        break;
                    }
                case 1:
                    {
                        m_coneIndices[0] = 0;
                        m_coneIndices[1] = 1;
                        m_coneIndices[2] = 2;
                        break;
                    }
                case 2:
                    {
                        m_coneIndices[0] = 0;
                        m_coneIndices[1] = 2;
                        m_coneIndices[2] = 1;
                        break;
                    }
                default:
                    Debug.Assert(false);
                    break;
            };

        }
    		
	    public int GetConeUpIndex() 
	    {
		    return m_coneIndices[1];
	    }

	    protected float m_sinAngle;
	    protected float m_radius;
	    protected float m_height;
	    protected int[] m_coneIndices = new int[3];

        protected IndexedVector3 ConeLocalSupport(ref IndexedVector3 v)
        {
	        float halfHeight = m_height * 0.5f;

         if (v[m_coneIndices[1]] > v.Length() * m_sinAngle)
         {
	        IndexedVector3 tmp = new IndexedVector3();

	        tmp[m_coneIndices[0]] = 0.0f;
	        tmp[m_coneIndices[1]] = halfHeight;
	        tmp[m_coneIndices[2]] = 0.0f;
	        return tmp;
         }
          else 
         {
            float s = (float)Math.Sqrt(v[m_coneIndices[0]] * v[m_coneIndices[0]] + v[m_coneIndices[2]] * v[m_coneIndices[2]]);
            if (s > MathUtil.SIMD_EPSILON) 
            {
              float d = m_radius / s;
	        IndexedVector3 tmp = new IndexedVector3();
	          tmp[m_coneIndices[0]] = v[m_coneIndices[0]] * d;
	          tmp[m_coneIndices[1]] = -halfHeight;
	          tmp[m_coneIndices[2]] = v[m_coneIndices[2]] * d;
	          return tmp;
            }
            else  
            {
	        IndexedVector3 tmp = new IndexedVector3();
		        tmp[m_coneIndices[0]] = 0.0f;
		        tmp[m_coneIndices[1]] = -halfHeight;
		        tmp[m_coneIndices[2]] = 0.0f;
		        return tmp;
	        }
          }

        }
    }

    ///btConeShape implements a Cone shape, around the X axis
    public class ConeShapeX : ConeShape
    {
        public ConeShapeX(float radius, float height)
            : base(radius, height)
        {
            SetConeUpIndex(0);
        }
    }

    ///btConeShapeZ implements a Cone shape, around the Z axis
    public class ConeShapeZ : ConeShape
    {
        public ConeShapeZ(float radius,float height)
            : base(radius, height)
        {
            SetConeUpIndex(2);
        }

    }
}
