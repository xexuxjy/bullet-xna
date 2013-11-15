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

	///The btConvex2dShape allows to use arbitrary convex shapes are 2d convex shapes, with the Z component assumed to be 0.
	///For 2d boxes, the btBox2dShape is recommended.
	public class Convex2dShape : ConvexShape
	{
		private ConvexShape	m_childConvexShape;

		
		public Convex2dShape(ConvexShape convexChildShape)
		{
			m_childConvexShape = convexChildShape;
			m_shapeType = BroadphaseNativeTypes.CONVEX_2D_SHAPE_PROXYTYPE;
		}
		
		public override void Cleanup()
		{

			base.Cleanup();
		}

	
		public override IndexedVector3	LocalGetSupportingVertexWithoutMargin(ref IndexedVector3 vec)
		{
			return m_childConvexShape.LocalGetSupportingVertexWithoutMargin(ref vec);
		}

		public override IndexedVector3	LocalGetSupportingVertex(ref IndexedVector3 vec)
		{
			return m_childConvexShape.LocalGetSupportingVertex(ref vec);
		}

		public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IndexedVector3[] vectors,IndexedVector4[] supportVerticesOut,int numVectors)
		{
			m_childConvexShape.BatchedUnitVectorGetSupportingVertexWithoutMargin(vectors,supportVerticesOut,numVectors);
		}

		public override void CalculateLocalInertia(float mass, out IndexedVector3 inertia)
		{
            m_childConvexShape.CalculateLocalInertia(mass, out inertia);
		}

		public ConvexShape GetChildShape() 
		{
			return m_childConvexShape;
		}


		public override String GetName()
		{
			return "Convex2dShape";
		}
		


		///////////////////////////


		///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
		public override void GetAabb(ref IndexedMatrix t,out IndexedVector3 aabbMin,out IndexedVector3 aabbMax)
		{
			m_childConvexShape.GetAabb(ref t,out aabbMin,out aabbMax);
		}

		public override void GetAabbSlow(ref IndexedMatrix t,out IndexedVector3 aabbMin,out IndexedVector3 aabbMax)
		{
			m_childConvexShape.GetAabbSlow(ref t,out aabbMin,out aabbMax);
		}

		public void SetLocalScaling(IndexedVector3 scaling)
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
			return m_childConvexShape.GetMargin();
		}

		public override int	GetNumPreferredPenetrationDirections()
		{
			return m_childConvexShape.GetNumPreferredPenetrationDirections();
		}
		
		public override void GetPreferredPenetrationDirection(int index, out IndexedVector3 penetrationVector)
		{
            m_childConvexShape.GetPreferredPenetrationDirection(index, out penetrationVector);
		}


	}

}
