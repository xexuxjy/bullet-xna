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
using BulletXNA.BulletCollision.BroadphaseCollision;
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletCollision.CollisionShapes
{
    public class UniformScalingShape : ConvexShape
    {
	    public UniformScalingShape(ConvexShape childConvexShape, float uniformScalingFactor)
        {
            m_childConvexShape = childConvexShape;
            m_uniformScalingFactor = uniformScalingFactor;
            m_shapeType = BroadphaseNativeTypes.UNIFORM_SCALING_SHAPE_PROXYTYPE;
        }

        public override void Cleanup()
        {
 	         base.Cleanup();
        }

        public override Vector3 LocalGetSupportingVertexWithoutMargin(ref Vector3 vec)
        {
	        Vector3 tmpVertex = m_childConvexShape.LocalGetSupportingVertexWithoutMargin(ref vec);
	        return tmpVertex*m_uniformScalingFactor;
        }

        public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IList<Vector3> vectors, IList<Vector4> supportVerticesOut, int numVectors)
        {
	        m_childConvexShape.BatchedUnitVectorGetSupportingVertexWithoutMargin(vectors,supportVerticesOut,numVectors);
	        for (int i=0;i<numVectors;i++)
	        {
                supportVerticesOut[i] = (supportVerticesOut[i] * m_uniformScalingFactor); ;
	        }
        }

        public override Vector3 LocalGetSupportingVertex(ref Vector3 vec)
        {
	        Vector3 tmpVertex = m_childConvexShape.LocalGetSupportingVertex(ref vec);
	        return tmpVertex*m_uniformScalingFactor;
        }

        public override void CalculateLocalInertia(float mass, out Vector3 inertia)
        {
	        ///this linear upscaling is not realistic, but we don't deal with large mass ratios...
	        Vector3 tmpInertia;
	        m_childConvexShape.CalculateLocalInertia(mass, out tmpInertia);
	        inertia = tmpInertia * m_uniformScalingFactor;
        }

	        ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
        public override void GetAabb(ref Matrix t, out Vector3 aabbMin, out Vector3 aabbMax)
        {
	        m_childConvexShape.GetAabb(ref t,out aabbMin,out aabbMax);
	        Vector3 aabbCenter = (aabbMax+aabbMin)*.5f;
	        Vector3 scaledAabbHalfExtends = (aabbMax-aabbMin)*0.5f*m_uniformScalingFactor;

	        aabbMin = aabbCenter - scaledAabbHalfExtends;
	        aabbMax = aabbCenter + scaledAabbHalfExtends;

        }

        public override void GetAabbSlow(ref Matrix t, out Vector3 aabbMin, out Vector3 aabbMax)
        {
	        m_childConvexShape.GetAabbSlow(ref t,out aabbMin,out aabbMax);
	        Vector3 aabbCenter = (aabbMax+aabbMin)* 0.5f;
	        Vector3 scaledAabbHalfExtends = (aabbMax-aabbMin)*0.5f*m_uniformScalingFactor;

	        aabbMin = aabbCenter - scaledAabbHalfExtends;
	        aabbMax = aabbCenter + scaledAabbHalfExtends;
        }

        public override void SetLocalScaling(ref Vector3 scaling) 
        {
	        m_childConvexShape.SetLocalScaling(ref scaling);
        }

        public override Vector3 GetLocalScaling()
        {
	        return m_childConvexShape.GetLocalScaling();
        }

        public override void SetMargin(float margin)
        {
	        m_childConvexShape.SetMargin(margin);
        }

        public override float GetMargin()
        {
	        return m_childConvexShape.GetMargin() * m_uniformScalingFactor;
        }

        public override int GetNumPreferredPenetrationDirections()
        {
	        return m_childConvexShape.GetNumPreferredPenetrationDirections();
        }
	
        public override void GetPreferredPenetrationDirection(int index, out Vector3 penetrationVector)
        {
            m_childConvexShape.GetPreferredPenetrationDirection(index, out penetrationVector);
        }

	    public float GetUniformScalingFactor()
	    {
		    return m_uniformScalingFactor;
	    }

	    public ConvexShape GetChildShape() 
	    {
		    return m_childConvexShape;
	    }

	    public override String GetName()
	    {
		    return "UniformScalingShape";
	    }

        private ConvexShape m_childConvexShape;
        private float m_uniformScalingFactor;
    }
}
