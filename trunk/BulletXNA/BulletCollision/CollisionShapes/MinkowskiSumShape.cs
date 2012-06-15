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
    public class MinkowskiSumShape : ConvexInternalShape
    {

        public MinkowskiSumShape(ConvexShape shapeA, ConvexShape shapeB)
        {
            m_shapeA = shapeA;
            m_shapeB = shapeB;
            m_transA = IndexedMatrix.Identity;
            m_transB = IndexedMatrix.Identity;
            m_shapeType = BroadphaseNativeTypes.MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE;
        }

        public override IndexedVector3 LocalGetSupportingVertexWithoutMargin(ref IndexedVector3 vec)
        {
            IndexedVector3 temp = vec * m_transA._basis;
            IndexedVector3 supVertexA = m_transA * (m_shapeA.LocalGetSupportingVertexWithoutMargin(ref temp));
            temp = -vec * m_transB._basis;
            IndexedVector3 supVertexB = m_transB * (m_shapeB.LocalGetSupportingVertexWithoutMargin(ref temp));
            return supVertexA - supVertexB;
        }

		public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IndexedVector3[] vectors, IndexedVector4[] supportVerticesOut, int numVectors)
        {
            ///@todo: could make recursive use of batching. probably this shape is not used frequently.
            for (int i = 0; i < numVectors; i++)
            {
                IndexedVector3 temp = vectors[i];
                supportVerticesOut[i] = new IndexedVector4(LocalGetSupportingVertexWithoutMargin(ref temp),0f);
            }
        }
        public override void CalculateLocalInertia(float mass, out IndexedVector3 inertia)
        {
            Debug.Assert(false);
            inertia = IndexedVector3.Zero;
        }

	    public void	SetTransformA(ref IndexedMatrix transA) 
        { 
            m_transA = transA;
        }
	    
	    public void	SetTransformB(ref IndexedMatrix transB) 
        { 
            m_transB = transB;
        }

    
        public IndexedMatrix GetTransformA()
        { 
            return m_transA;
        }

        public IndexedMatrix GetTransformB()
        { 
            return m_transB;
        }


        public override float GetMargin()
        {
            return m_shapeA.GetMargin() + m_shapeB.GetMargin();
        }

	    public ConvexShape GetShapeA()
        { 
            return m_shapeA;
        }
	    public ConvexShape GetShapeB()
        { 
            return m_shapeB;
        }

	    public override String GetName()
	    {
		    return "MinkowskiSum";
	    }

        private IndexedMatrix m_transA;
        private IndexedMatrix m_transB;

        private ConvexShape m_shapeA;
        private ConvexShape m_shapeB;

    }
}
