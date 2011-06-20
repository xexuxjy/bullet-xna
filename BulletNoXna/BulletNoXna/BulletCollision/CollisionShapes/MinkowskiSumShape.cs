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
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletCollision
{
    public class MinkowskiSumShape : ConvexInternalShape
    {

        public MinkowskiSumShape(ConvexShape shapeA, ConvexShape shapeB)
        {
            m_shapeA = shapeA;
            m_shapeB = shapeB;
            m_transA = Matrix.Identity;
            m_transB = Matrix.Identity;
            m_shapeType = BroadphaseNativeType.MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE;
        }

        public override Vector3 LocalGetSupportingVertexWithoutMargin(ref Vector3 vec)
        {
			Vector3 ta = MathUtil.TransposeTransformNormal(vec, m_transA);
			Vector3 tb = MathUtil.TransposeTransformNormal(vec, m_transB);

            Vector3 supVertexA = Vector3.Transform((m_shapeA.LocalGetSupportingVertexWithoutMargin(ref ta)),m_transA);
            Vector3 supVertexB = Vector3.Transform((m_shapeB.LocalGetSupportingVertexWithoutMargin(ref tb)), m_transB);
            return supVertexA - supVertexB;
        }

		public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector4[] supportVerticesOut, int numVectors)
        {
            ///@todo: could make recursive use of batching. probably this shape is not used frequently.
            for (int i = 0; i < numVectors; i++)
            {
                Vector3 temp = vectors[i];
                supportVerticesOut[i] = new Vector4(LocalGetSupportingVertexWithoutMargin(ref temp),0f);
            }
        }
        public override void CalculateLocalInertia(float mass, out Vector3 inertia)
        {
            Debug.Assert(false);
            inertia = Vector3.Zero;
        }

	    public void	SetTransformA(ref Matrix transA) 
        { 
            m_transA = transA;
        }
	    
	    public void	SetTransformB(ref Matrix transB) 
        { 
            m_transB = transB;
        }

    
        public Matrix GetTransformA()
        { 
            return m_transA;
        }

        public Matrix GetTransformB()
        { 
            return m_transB;
        }


        public override float Margin
        {
            get
            {
                return m_shapeA.Margin + m_shapeB.Margin;
            }
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

        private Matrix m_transA;
        private Matrix m_transB;

        private ConvexShape m_shapeA;
        private ConvexShape m_shapeB;

    }
}
