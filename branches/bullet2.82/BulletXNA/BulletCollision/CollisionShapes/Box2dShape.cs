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
///The btBox2dShape is a box primitive around the origin, its sides axis aligned with length specified by half extents, in local shape coordinates. When used as part of a btCollisionObject or btRigidBody it will be an oriented box in world space.
	public class Box2dShape: PolyhedralConvexShape
	{

		//IndexedVector3	m_boxHalfExtents1; //use m_implicitShapeDimensions instead

		private IndexedVector3 m_centroid;
		private IndexedVector3[] m_vertices = new IndexedVector3[4];
		private IndexedVector3[] m_normals = new IndexedVector3[4];


		public virtual IndexedVector3 GetHalfExtentsWithMargin()
		{
			IndexedVector3 halfExtents = GetHalfExtentsWithoutMargin();
			IndexedVector3 margin = new IndexedVector3(GetMargin());
			halfExtents += margin;
			return halfExtents;
		}
		
		public virtual IndexedVector3 GetHalfExtentsWithoutMargin()
		{
			return m_implicitShapeDimensions;//changed in Bullet 2.63: assume the scaling and margin are included
		}
		

		public override IndexedVector3	LocalGetSupportingVertex(ref IndexedVector3 vec)
		{
			IndexedVector3 halfExtents = GetHalfExtentsWithoutMargin();
			IndexedVector3 margin= new IndexedVector3(GetMargin());
			halfExtents += margin;

			return new IndexedVector3(MathUtil.FSel(vec.X, halfExtents.X, -halfExtents.X),
				MathUtil.FSel(vec.Y, halfExtents.Y, -halfExtents.Y),
				MathUtil.FSel(vec.Z, halfExtents.Z, -halfExtents.Z));
		}

		public override IndexedVector3	LocalGetSupportingVertexWithoutMargin(ref IndexedVector3 vec)
		{
			IndexedVector3 halfExtents = GetHalfExtentsWithoutMargin();

			return new IndexedVector3(MathUtil.FSel(vec.X, halfExtents.X, -halfExtents.X),
				MathUtil.FSel(vec.Y, halfExtents.Y, -halfExtents.Y),
				MathUtil.FSel(vec.Z, halfExtents.Z, -halfExtents.Z));
		}

		public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IndexedVector3[] vectors, IndexedVector4[] supportVerticesOut, int numVectors)
		{
			IndexedVector3 halfExtents = GetHalfExtentsWithoutMargin();
		
			for (int i=0;i<numVectors;i++)
			{
				IndexedVector3 vec = vectors[i];
				supportVerticesOut[i] = new IndexedVector4(MathUtil.FSel(vec.X, halfExtents.X, -halfExtents.X),
					MathUtil.FSel(vec.Y, halfExtents.Y, -halfExtents.Y),
					MathUtil.FSel(vec.Z, halfExtents.Z, -halfExtents.Z),0); 
			}

		}


		public Box2dShape(ref IndexedVector3 boxHalfExtents) 
		{
			m_centroid = IndexedVector3.Zero;
			m_vertices[0] = new IndexedVector3(-boxHalfExtents.X,-boxHalfExtents.Y,0);
			m_vertices[1] = new IndexedVector3(boxHalfExtents.X,-boxHalfExtents.Y,0);
			m_vertices[2] = new IndexedVector3(boxHalfExtents.X,boxHalfExtents.Y,0);
			m_vertices[3] = new IndexedVector3(-boxHalfExtents.X,boxHalfExtents.Y,0);

			m_normals[0] = new IndexedVector3(0,-1,0);
			m_normals[1] = new IndexedVector3(1,0,0);
			m_normals[2] = new IndexedVector3(0,1,0);
			m_normals[3] = new IndexedVector3(-1,0,0);


            float minDimension = boxHalfExtents.X;
            if (minDimension > boxHalfExtents.Y)
            {
                minDimension = boxHalfExtents.Y;
            }
            SetSafeMargin(minDimension);

			m_shapeType = BroadphaseNativeTypes.BOX_2D_SHAPE_PROXYTYPE;
			IndexedVector3 margin = new IndexedVector3(GetMargin());
			m_implicitShapeDimensions = (boxHalfExtents * m_localScaling) - margin;
		}

		public override void SetMargin(float collisionMargin)
		{
			//correct the m_implicitShapeDimensions for the margin
			IndexedVector3 oldMargin= new IndexedVector3(GetMargin());
			IndexedVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
			
			base.SetMargin(collisionMargin);
			IndexedVector3 newMargin= new IndexedVector3(GetMargin());
			m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;

		}
		public override void SetLocalScaling(ref IndexedVector3 scaling)
		{
			IndexedVector3 oldMargin= new IndexedVector3(GetMargin());
			IndexedVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
			IndexedVector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

			base.SetLocalScaling(ref scaling);

			m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;
		}

		public override void GetAabb(ref IndexedMatrix t,out IndexedVector3 aabbMin,out IndexedVector3 aabbMax)
		{
			AabbUtil2.TransformAabb(GetHalfExtentsWithoutMargin(),GetMargin(),ref t, out aabbMin,out aabbMax);
		}

		

		public override void CalculateLocalInertia(float mass, out IndexedVector3 inertia)
		{

		//float margin = float(0.);
			IndexedVector3 halfExtents = GetHalfExtentsWithMargin();

			float lx=2.0f*halfExtents.X;
			float ly=2.0f*halfExtents.Y;
			float lz=2.0f*halfExtents.Z;

			inertia = new IndexedVector3(mass/(12.0f) * (ly*ly + lz*lz),
							mass/(12.0f) * (lx*lx + lz*lz),
							mass/(12.0f) * (lx*lx + ly*ly));
		}





		public virtual int	GetVertexCount()
		{
			return 4;
		}

		public override int GetNumVertices()
		{
			return 4;
		}

		public IndexedVector3[] GetVertices()
		{
			return m_vertices;
		}

		public IndexedVector3[] GetNormals()
		{
			return m_normals;
		}

        public override void GetPlane(out IndexedVector3 planeNormal, out IndexedVector3 planeSupport, int i)
		{
			//this plane might not be aligned...
			IndexedVector4 plane;
			GetPlaneEquation(out plane,i);
			planeNormal = new IndexedVector3(plane.X,plane.Y,plane.Z);
			planeSupport = LocalGetSupportingVertex(-planeNormal);
		}


		public IndexedVector3 GetCentroid()
		{
			return m_centroid;
		}
		
		public override int GetNumPlanes()
		{
			return 6;
		}	

		public override int GetNumEdges()
		{
			return 12;
		}


		public override void GetVertex(int i, out IndexedVector3 vtx)
		{
			IndexedVector3 halfExtents = GetHalfExtentsWithoutMargin();

			vtx = new IndexedVector3(
					halfExtents.X * (1-(i&1)) - halfExtents.X * (i&1),
					halfExtents.Y * (1-((i&2)>>1)) - halfExtents.Y * ((i&2)>>1),
					halfExtents.Z * (1-((i&4)>>2)) - halfExtents.Z * ((i&4)>>2));
		}
		

		public void GetPlaneEquation(out IndexedVector4 plane,int i)
		{
			IndexedVector3 halfExtents = GetHalfExtentsWithoutMargin();

			switch (i)
			{
			case 0:
				plane = new IndexedVector4(1,0,0,-halfExtents.X);
				break;
			case 1:
				plane = new IndexedVector4(-1,0,0,-halfExtents.X);
				break;
			case 2:
				plane = new IndexedVector4(0,1,0,-halfExtents.Y);
				break;
			case 3:
				plane = new IndexedVector4(0,-1,0,-halfExtents.Y);
				break;
			case 4:
				plane = new IndexedVector4(0,0,1,-halfExtents.Z);
				break;
			case 5:
				plane = new IndexedVector4(0,0,-1,-halfExtents.Z);
				break;
			default:
				Debug.Assert(false);
                plane = IndexedVector4.Zero;
				break;
			}
		}


        public override void GetEdge(int i, out IndexedVector3 pa, out IndexedVector3 pb)
		//virtual void getEdge(int i,Edge& edge) const
		{
			int edgeVert0 = 0;
			int edgeVert1 = 0;

			switch (i)
			{
			case 0:
					edgeVert0 = 0;
					edgeVert1 = 1;
				break;
			case 1:
					edgeVert0 = 0;
					edgeVert1 = 2;
				break;
			case 2:
				edgeVert0 = 1;
				edgeVert1 = 3;

				break;
			case 3:
				edgeVert0 = 2;
				edgeVert1 = 3;
				break;
			case 4:
				edgeVert0 = 0;
				edgeVert1 = 4;
				break;
			case 5:
				edgeVert0 = 1;
				edgeVert1 = 5;

				break;
			case 6:
				edgeVert0 = 2;
				edgeVert1 = 6;
				break;
			case 7:
				edgeVert0 = 3;
				edgeVert1 = 7;
				break;
			case 8:
				edgeVert0 = 4;
				edgeVert1 = 5;
				break;
			case 9:
				edgeVert0 = 4;
				edgeVert1 = 6;
				break;
			case 10:
				edgeVert0 = 5;
				edgeVert1 = 7;
				break;
			case 11:
				edgeVert0 = 6;
				edgeVert1 = 7;
				break;
			default:
				Debug.Assert(false);
				break;

			}
            GetVertex(edgeVert0, out pa);
            GetVertex(edgeVert1, out pb);
		}


		
		public override bool IsInside(ref IndexedVector3 pt,float tolerance)
		{
			IndexedVector3 halfExtents = GetHalfExtentsWithoutMargin();

			//float minDist = 2*tolerance;
			
			bool result =	(pt.X <= (halfExtents.X+tolerance)) &&
							(pt.X >= (-halfExtents.X-tolerance)) &&
							(pt.Y <= (halfExtents.Y+tolerance)) &&
							(pt.Y >= (-halfExtents.Y-tolerance)) &&
							(pt.Z <= (halfExtents.Z+tolerance)) &&
							(pt.Z >= (-halfExtents.Z-tolerance));
			
			return result;
		}


		//debugging
		public override String GetName()
		{
			return "Box2d";
		}

		public override int	GetNumPreferredPenetrationDirections()
		{
			return 6;
		}
		
		public override void GetPreferredPenetrationDirection(int index, out IndexedVector3 penetrationVector) 
		{
			switch (index)
			{
			case 0:
				penetrationVector = new IndexedVector3(1,0,0);
				break;
			case 1:
				penetrationVector = new IndexedVector3(-1,0,0);
				break;
			case 2:
				penetrationVector = new IndexedVector3(0,1,0);
				break;
			case 3:
				penetrationVector = new IndexedVector3(0,-1,0);
				break;
			case 4:
				penetrationVector = new IndexedVector3(0,0,1);
				break;
			case 5:
				penetrationVector = new IndexedVector3(0,0,-1);
				break;
			default:
				Debug.Assert(false);
                penetrationVector = IndexedVector3.Zero;
				break;
			}
		}

	}

}
