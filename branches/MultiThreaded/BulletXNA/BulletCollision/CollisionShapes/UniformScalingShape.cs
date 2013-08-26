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

        public override IndexedVector3 LocalGetSupportingVertexWithoutMargin(ref IndexedVector3 vec)
        {
	        IndexedVector3 tmpVertex = m_childConvexShape.LocalGetSupportingVertexWithoutMargin(ref vec);
	        return tmpVertex*m_uniformScalingFactor;
        }

        public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IndexedVector3[] vectors, IndexedVector4[] supportVerticesOut, int numVectors)
        {
	        m_childConvexShape.BatchedUnitVectorGetSupportingVertexWithoutMargin(vectors,supportVerticesOut,numVectors);
	        for (int i=0;i<numVectors;i++)
	        {
                supportVerticesOut[i] = (supportVerticesOut[i] * m_uniformScalingFactor);
	        }
        }

        public override IndexedVector3 LocalGetSupportingVertex(ref IndexedVector3 vec)
        {
	        IndexedVector3 tmpVertex = m_childConvexShape.LocalGetSupportingVertex(ref vec);
	        return tmpVertex*m_uniformScalingFactor;
        }

        public override void CalculateLocalInertia(float mass, out IndexedVector3 inertia)
        {
	        ///this linear upscaling is not realistic, but we don't deal with large mass ratios...
	        IndexedVector3 tmpInertia;
	        m_childConvexShape.CalculateLocalInertia(mass, out tmpInertia);
	        inertia = tmpInertia * m_uniformScalingFactor;
        }

	        ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
        public override void GetAabb(ref IndexedMatrix t, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
        {
            GetAabbSlow(ref t, out aabbMin, out aabbMax);
        }

        public override void GetAabbSlow(ref IndexedMatrix t, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
        {
#if true
	IndexedVector3[] _directions = new IndexedVector3[]
	{
		new IndexedVector3( 1.0f,  0.0f,  0.0f),
		new IndexedVector3( 0.0f,  1.0f,  0.0f),
		new IndexedVector3( 0.0f,  0.0f,  1.0f),
		new IndexedVector3( -1.0f, 0.0f,  0.0f),
		new IndexedVector3( 0.0f, -1.0f,  0.0f),
		new IndexedVector3( 0.0f,  0.0f, -1.0f)
	};
	
	IndexedVector4[] _supporting = new IndexedVector4[]
	{
		IndexedVector4.Zero,
		IndexedVector4.Zero,
		IndexedVector4.Zero,
		IndexedVector4.Zero,
		IndexedVector4.Zero,
		IndexedVector4.Zero
	};

    for (int i = 0; i < 6; i++)
    {
        _directions[i] = _directions[i] * t._basis;
    }
    
    ObjectArray<IndexedVector4> tempSupporting = new ObjectArray<IndexedVector4>(6);

	
	BatchedUnitVectorGetSupportingVertexWithoutMargin(_directions, _supporting, 6);
	
	IndexedVector3 aabbMin1 = new IndexedVector3(0,0,0),aabbMax1 = new IndexedVector3(0,0,0);

	for ( int i = 0; i < 3; ++i )
	{
		IndexedVector3 temp = new IndexedVector3(_supporting[i].X, _supporting[i].Y, _supporting[i].Z);
		aabbMax1[i] = (t *temp)[i];
		temp = new IndexedVector3(_supporting[i+3].X, _supporting[i+3].Y, _supporting[i+3].Z);
        aabbMin1[i] = (t * temp)[i];
	}

	IndexedVector3 marginVec = new IndexedVector3(GetMargin());
	aabbMin = aabbMin1-marginVec;
	aabbMax = aabbMax1+marginVec;
	
#else

	float margin = getMargin();
	for (int i=0;i<3;i++)
	{
		IndexedVector3 vec(float(0.),float(0.),float(0.));
		vec[i] = float(1.);
		IndexedVector3 sv = localGetSupportingVertex(vec*t.getBasis());
		IndexedVector3 tmp = t(sv);
		aabbMax[i] = tmp[i]+margin;
		vec[i] = float(-1.);
		sv = localGetSupportingVertex(vec*t.getBasis());
		tmp = t(sv);
		aabbMin[i] = tmp[i]-margin;
	}

#endif
        }

        public override void SetLocalScaling(ref IndexedVector3 scaling) 
        {
	        m_childConvexShape.SetLocalScaling(ref scaling);
        }

        public override IndexedVector3 GetLocalScaling()
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
	
        public override void GetPreferredPenetrationDirection(int index, out IndexedVector3 penetrationVector)
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
