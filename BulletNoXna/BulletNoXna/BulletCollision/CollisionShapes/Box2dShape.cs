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
using System.Diagnostics;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
///The btBox2dShape is a box primitive around the origin, its sides axis aligned with length specified by half extents, in local shape coordinates. When used as part of a btCollisionObject or btRigidBody it will be an oriented box in world space.
	public class Box2dShape: PolyhedralConvexShape
	{

		//Vector3	m_boxHalfExtents1; //use m_implicitShapeDimensions instead

		private Vector3 m_centroid;
		private Vector3[] m_vertices = new Vector3[4];
		private Vector3[] m_normals = new Vector3[4];


		public virtual Vector3 GetHalfExtentsWithMargin()
		{
			Vector3 halfExtents = GetHalfExtentsWithoutMargin();
			Vector3 margin = new Vector3(Margin);
			halfExtents += margin;
			return halfExtents;
		}
		
		public virtual Vector3 GetHalfExtentsWithoutMargin()
		{
			return m_implicitShapeDimensions;//changed in Bullet 2.63: assume the scaling and margin are included
		}
		

		public override Vector3	LocalGetSupportingVertex(ref Vector3 vec)
		{
			Vector3 halfExtents = GetHalfExtentsWithoutMargin();
            Vector3 margin = new Vector3(Margin);
			halfExtents += margin;

			return new Vector3(MathUtil.FSel(vec.X, halfExtents.X, -halfExtents.X),
				MathUtil.FSel(vec.Y, halfExtents.Y, -halfExtents.Y),
				MathUtil.FSel(vec.Z, halfExtents.Z, -halfExtents.Z));
		}

		public override Vector3	LocalGetSupportingVertexWithoutMargin(ref Vector3 vec)
		{
			Vector3 halfExtents = GetHalfExtentsWithoutMargin();

			return new Vector3(MathUtil.FSel(vec.X, halfExtents.X, -halfExtents.X),
				MathUtil.FSel(vec.Y, halfExtents.Y, -halfExtents.Y),
				MathUtil.FSel(vec.Z, halfExtents.Z, -halfExtents.Z));
		}

		public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IList<Vector3> vectors, IList<Vector4> supportVerticesOut, int numVectors)
		{
			Vector3 halfExtents = GetHalfExtentsWithoutMargin();
		
			for (int i=0;i<numVectors;i++)
			{
				Vector3 vec = vectors[i];
				supportVerticesOut[i] = new Vector4(MathUtil.FSel(vec.X, halfExtents.X, -halfExtents.X),
					MathUtil.FSel(vec.Y, halfExtents.Y, -halfExtents.Y),
					MathUtil.FSel(vec.Z, halfExtents.Z, -halfExtents.Z),0); 
			}

		}


		public Box2dShape(ref Vector3 boxHalfExtents) 
		{
			m_centroid = Vector3.Zero;
			m_vertices[0] = new Vector3(-boxHalfExtents.X,-boxHalfExtents.Y,0);
			m_vertices[1] = new Vector3(boxHalfExtents.X,-boxHalfExtents.Y,0);
			m_vertices[2] = new Vector3(boxHalfExtents.X,boxHalfExtents.Y,0);
			m_vertices[3] = new Vector3(-boxHalfExtents.X,boxHalfExtents.Y,0);

			m_normals[0] = new Vector3(0,-1,0);
			m_normals[1] = new Vector3(1,0,0);
			m_normals[2] = new Vector3(0,1,0);
			m_normals[3] = new Vector3(-1,0,0);

			m_shapeType = BroadphaseNativeType.Box2DShape;
            Vector3 margin = new Vector3(Margin);
			m_implicitShapeDimensions = (boxHalfExtents * m_localScaling) - margin;
		}

		public override float Margin
		{
            set
            {
                //correct the m_implicitShapeDimensions for the margin
                Vector3 oldMargin = new Vector3(Margin);
                Vector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions + oldMargin;

                base.Margin = value;
                Vector3 newMargin = new Vector3(Margin);
                m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;
            }
		}
		public override void SetLocalScaling(ref Vector3 scaling)
		{
			Vector3 oldMargin= new Vector3(Margin);
			Vector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
			Vector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

			base.SetLocalScaling(ref scaling);

			m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;
		}

		public override void GetAabb(ref Matrix t,out Vector3 aabbMin,out Vector3 aabbMax)
		{
			AabbUtil2.TransformAabb(GetHalfExtentsWithoutMargin(),Margin,ref t, out aabbMin,out aabbMax);
		}

		

		public override void CalculateLocalInertia(float mass, out Vector3 inertia)
		{

		//float margin = float(0.);
			Vector3 halfExtents = GetHalfExtentsWithMargin();

			float lx=2.0f*halfExtents.X;
			float ly=2.0f*halfExtents.Y;
			float lz=2.0f*halfExtents.Z;

			inertia = new Vector3(mass/(12.0f) * (ly*ly + lz*lz),
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

		public Vector3[] GetVertices()
		{
			return m_vertices;
		}

		public Vector3[] GetNormals()
		{
			return m_normals;
		}

        public override void GetPlane(out Vector3 planeNormal, out Vector3 planeSupport, int i)
		{
			//this plane might not be aligned...
			Vector4 plane;
			GetPlaneEquation(out plane,i);
			planeNormal = new Vector3(plane.X,plane.Y,plane.Z);
			planeSupport = LocalGetSupportingVertex(-planeNormal);
		}


		public Vector3 GetCentroid()
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


		public override void GetVertex(int i, out Vector3 vtx)
		{
			Vector3 halfExtents = GetHalfExtentsWithoutMargin();

			vtx = new Vector3(
					halfExtents.X * (1-(i&1)) - halfExtents.X * (i&1),
					halfExtents.Y * (1-((i&2)>>1)) - halfExtents.Y * ((i&2)>>1),
					halfExtents.Z * (1-((i&4)>>2)) - halfExtents.Z * ((i&4)>>2));
		}
		

		public void GetPlaneEquation(out Vector4 plane,int i)
		{
			Vector3 halfExtents = GetHalfExtentsWithoutMargin();

			switch (i)
			{
			case 0:
				plane = new Vector4(1,0,0,-halfExtents.X);
				break;
			case 1:
				plane = new Vector4(-1,0,0,-halfExtents.X);
				break;
			case 2:
				plane = new Vector4(0,1,0,-halfExtents.Y);
				break;
			case 3:
				plane = new Vector4(0,-1,0,-halfExtents.Y);
				break;
			case 4:
				plane = new Vector4(0,0,1,-halfExtents.Z);
				break;
			case 5:
				plane = new Vector4(0,0,-1,-halfExtents.Z);
				break;
			default:
				Debug.Assert(false);
                plane = Vector4.Zero;
				break;
			}
		}


        public override void GetEdge(int i, out Vector3 pa, out Vector3 pb)
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


		
		public override bool IsInside(ref Vector3 pt,float tolerance)
		{
			Vector3 halfExtents = GetHalfExtentsWithoutMargin();

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
		
		public override void GetPreferredPenetrationDirection(int index, out Vector3 penetrationVector) 
		{
			switch (index)
			{
			case 0:
				penetrationVector = new Vector3(1,0,0);
				break;
			case 1:
				penetrationVector = new Vector3(-1,0,0);
				break;
			case 2:
				penetrationVector = new Vector3(0,1,0);
				break;
			case 3:
				penetrationVector = new Vector3(0,-1,0);
				break;
			case 4:
				penetrationVector = new Vector3(0,0,1);
				break;
			case 5:
				penetrationVector = new Vector3(0,0,-1);
				break;
			default:
				Debug.Assert(false);
                penetrationVector = Vector3.Zero;
				break;
			}
		}

	}

}
