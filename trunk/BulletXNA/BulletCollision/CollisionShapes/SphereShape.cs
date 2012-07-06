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

using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
    public class SphereShape : ConvexInternalShape
    {
        public SphereShape()
            : this(1f)
        {
        }

	    public SphereShape (float radius)
	    {
		    m_shapeType = BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE;
		    m_implicitShapeDimensions.X  = radius;
		    m_collisionMargin = radius;
	    }

        public void Initialize(float radius)
        {
            m_shapeType = BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE;
            m_implicitShapeDimensions.X = radius;
            m_collisionMargin = radius;
        }

	
	    public override IndexedVector3	LocalGetSupportingVertex(ref IndexedVector3 vec)
        {
	        IndexedVector3 supVertex;
	        supVertex = LocalGetSupportingVertexWithoutMargin(ref vec);

	        IndexedVector3 vecnorm = vec;
	        if (vecnorm.LengthSquared() < (MathUtil.SIMD_EPSILON*MathUtil.SIMD_EPSILON))
	        {
		        vecnorm = new IndexedVector3(-1f);
	        } 
	        vecnorm.Normalize();
	        supVertex+= GetMargin() * vecnorm;
	        return supVertex;
        }

        public override IndexedVector3 LocalGetSupportingVertexWithoutMargin(ref IndexedVector3 vec)
        {
            return IndexedVector3.Zero;
        }

        //notice that the vectors should be unit length
		public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IndexedVector3[] vectors, IndexedVector4[] supportVerticesOut, int numVectors) 
        {
	        for (int i=0;i<numVectors;i++)
	        {
		        supportVerticesOut[i] = IndexedVector4.Zero;
	        }
        }


        public override void CalculateLocalInertia(float mass, out IndexedVector3 inertia)
        {
            float elem = 0.4f * mass * GetMargin()*GetMargin();
            inertia = new IndexedVector3(elem);
        }

        public override void GetAabb(ref IndexedMatrix t, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
        {
	        IndexedVector3 center = t._origin;
            float margin = GetMargin();
	        IndexedVector3 extent = new IndexedVector3(margin);
	        aabbMin = center - extent;
	        aabbMax = center + extent;
        }

        public virtual float GetRadius() 
        { 
            return m_implicitShapeDimensions.X * m_localScaling.X;
        }

	    public void	SetUnscaledRadius(float	radius)
	    {
		    m_implicitShapeDimensions.X = radius;
		    SetMargin(radius);
	    }

	    //debugging
	    public override String GetName()
        {
            return "SPHERE";
        }


        public override float GetMargin()
	    {
		    //to improve gjk behaviour, use radius+margin as the full margin, so never get into the penetration case
		    //this means, non-uniform scaling is not supported anymore
		    return GetRadius();
	    }

        public override void Cleanup()
        {
            base.Cleanup();
        }

    }
}
