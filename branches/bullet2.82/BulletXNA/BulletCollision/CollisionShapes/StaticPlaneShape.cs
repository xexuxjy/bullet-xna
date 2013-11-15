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
    public class StaticPlaneShape : ConcaveShape
    {
        public StaticPlaneShape(IndexedVector3 planeNormal, float planeConstant) : this(ref planeNormal,planeConstant)
        {

        }

        public StaticPlaneShape(ref IndexedVector3 planeNormal,float planeConstant)
        {
            m_shapeType = BroadphaseNativeTypes.STATIC_PLANE_PROXYTYPE;
            m_planeNormal = planeNormal;
            m_planeConstant = planeConstant;
            m_localScaling = IndexedVector3.Zero;
        }

        public override void  Cleanup()
        {
 	         base.Cleanup();
        }

        public override void GetAabb(ref IndexedMatrix t, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
        {
            aabbMin = MathUtil.MIN_VECTOR;
            aabbMax = MathUtil.MAX_VECTOR;
        }

        public override void ProcessAllTriangles(ITriangleCallback callback, ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax)
        {
	        IndexedVector3 halfExtents = (aabbMax - aabbMin) * .5f;
	        float radius = halfExtents.Length();
	        IndexedVector3 center = (aabbMax + aabbMin) * 0.5f;
        	
	        //this is where the triangles are generated, given AABB and plane equation (normal/constant)

	        IndexedVector3 tangentDir0;
            IndexedVector3 tangentDir1;

	        //tangentDir0/tangentDir1 can be precalculated
	        TransformUtil.PlaneSpace1(ref m_planeNormal, out tangentDir0, out tangentDir1);

            IndexedVector3 supVertex0 = IndexedVector3.Zero;
            IndexedVector3 supVertex1 = IndexedVector3.Zero;

	        IndexedVector3 projectedCenter = center - (IndexedVector3.Dot(m_planeNormal,center) - m_planeConstant)*m_planeNormal;

            IndexedVector3[] triangle = new IndexedVector3[3];
	        triangle[0] = (projectedCenter + tangentDir0*radius + tangentDir1*radius);
            triangle[1] = (projectedCenter + tangentDir0 * radius - tangentDir1 * radius);
            triangle[2] = (projectedCenter - tangentDir0 * radius - tangentDir1 * radius);

	        callback.ProcessTriangle(triangle,0,0);

	        triangle[0] = projectedCenter - tangentDir0*radius - tangentDir1*radius;
	        triangle[1] = projectedCenter - tangentDir0*radius + tangentDir1*radius;
	        triangle[2] = projectedCenter + tangentDir0*radius + tangentDir1*radius;

	        callback.ProcessTriangle(triangle,0,1);

        }

        public override void CalculateLocalInertia(float mass, out IndexedVector3 inertia)
        {
            inertia = IndexedVector3.Zero;
        }

        public override void SetLocalScaling(ref IndexedVector3 scaling)
        {
            m_localScaling = scaling;
        }
	    
        public override IndexedVector3 GetLocalScaling()
        {
            return m_localScaling;
        }
	
	    public IndexedVector3	GetPlaneNormal()
	    {
		    return	m_planeNormal;
	    }

	    public float GetPlaneConstant()
	    {
		    return	m_planeConstant;
	    }

	    //debugging
	    public override String GetName()
        {
            return "STATICPLANE";
        }


        protected IndexedVector3 m_localAabbMin;
        protected IndexedVector3 m_localAabbMax;
        protected IndexedVector3 m_planeNormal;
        protected float m_planeConstant;
        protected IndexedVector3 m_localScaling;

    }
}
