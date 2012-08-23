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

using System.Diagnostics;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
	public class Box2dBox2dCollisionAlgorithm : ActivatingCollisionAlgorithm
	{
		public Box2dBox2dCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci)
		: base(ci) 
        {
            
        }

		public Box2dBox2dCollisionAlgorithm(PersistentManifold mf,CollisionAlgorithmConstructionInfo ci,CollisionObject body0,CollisionObject body1) :
			base(ci,body0,body1)
		{
            m_ownManifold = false;
            m_manifoldPtr = mf;
			if (m_manifoldPtr == null && m_dispatcher.NeedsCollision(body0, body1))
			{
				m_manifoldPtr = m_dispatcher.GetNewManifold(body0, body1);
				m_ownManifold = true;
			}
		}

		public override void Cleanup()
		{
			if (m_ownManifold)
			{
				if (m_manifoldPtr != null)
				{
					m_dispatcher.ReleaseManifold(m_manifoldPtr);
				}
			}
			base.Cleanup();
		}

		public override void ProcessCollision(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
		{
			if (m_manifoldPtr == null)
			{
				return;
			}

			CollisionObject col0 = body0;
			CollisionObject col1 = body1;
			Box2dShape box0 = (Box2dShape)col0.GetCollisionShape();
			Box2dShape box1 = (Box2dShape)col1.GetCollisionShape();

			resultOut.SetPersistentManifold(m_manifoldPtr);

			B2CollidePolygons(ref resultOut, box0, col0.GetWorldTransform(), box1, col1.GetWorldTransform());

			//  refreshContactPoints is only necessary when using persistent contact points. otherwise all points are newly added
			if (m_ownManifold)
			{
				resultOut.RefreshContactPoints();
			}

		}

		public override float CalculateTimeOfImpact(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
		{
			//not yet
			return 1.0f;
		}

        public override void GetAllContactManifolds(PersistentManifoldArray manifoldArray)
		{
			if (m_manifoldPtr != null && m_ownManifold)
			{
				manifoldArray.Add(m_manifoldPtr);
			}
		}


		// Find edge normal of max separation on A - return if separating axis is found
		// Find edge normal of max separation on B - return if separation axis is found
		// Choose reference edge as min(minA, minB)
		// Find incident edge
		// Clip

		// The normal points from 1 to 2

		void B2CollidePolygons(ref ManifoldResult manifold,
							  Box2dShape polyA, IndexedMatrix xfA,
							  Box2dShape polyB, IndexedMatrix xfB)
		{
			B2CollidePolygons(ref manifold, polyA, ref xfA, polyB, ref xfB);
		}


		void B2CollidePolygons(ref ManifoldResult manifold,
							  Box2dShape polyA, ref IndexedMatrix xfA,
							  Box2dShape polyB, ref IndexedMatrix xfB)
		{

			int edgeA = 0;
			float separationA = FindMaxSeparation(ref edgeA, polyA, ref xfA, polyB, ref xfB);
			if (separationA > 0.0f)
			{
				return;
			}

			int edgeB = 0;
			float separationB = FindMaxSeparation(ref edgeB, polyB, ref xfB, polyA, ref xfA);
			if (separationB > 0.0f)
			{
				return;
			}

			Box2dShape poly1;	// reference poly
			Box2dShape poly2;	// incident poly
			IndexedMatrix xf1, xf2;
			int edge1;		// reference edge
			bool flip;
			const float k_relativeTol = 0.98f;
			const float k_absoluteTol = 0.001f;

			// TODO_ERIN use "radius" of poly for absolute tolerance.
			if (separationB > k_relativeTol * separationA + k_absoluteTol)
			{
				poly1 = polyB;
				poly2 = polyA;
				xf1 = xfB;
				xf2 = xfA;
				edge1 = edgeB;
				flip = true;
			}
			else
			{
				poly1 = polyA;
				poly2 = polyB;
				xf1 = xfA;
				xf2 = xfB;
				edge1 = edgeA;
				flip = false;
			}

			ClipVertex[] incidentEdge = new ClipVertex[2];
			FindIncidentEdge(incidentEdge, poly1, ref xf1, edge1, poly2, ref xf2);

			int count1 = poly1.GetVertexCount();
			IndexedVector3[] vertices1 = poly1.GetVertices();

			IndexedVector3 v11 = vertices1[edge1];
			IndexedVector3 v12 = edge1 + 1 < count1 ? vertices1[edge1+1] : vertices1[0];

			IndexedVector3 dv = v12 - v11;
			IndexedVector3 sideNormal = xf1._basis * (v12 - v11);
			sideNormal.Normalize();
			IndexedVector3 frontNormal = CrossS(ref sideNormal, 1.0f);

            v11 = xf1 * v11;
            v12 = xf1 * v12;

			float frontOffset = frontNormal.Dot(ref v11);
			float sideOffset1 = -(sideNormal.Dot(ref v11));
			float sideOffset2 = sideNormal.Dot(ref v12);

			// Clip incident edge against extruded edge1 side edges.
			ClipVertex[] clipPoints1 = new ClipVertex[2];
			clipPoints1[0].v = IndexedVector3.Zero;
			clipPoints1[1].v = IndexedVector3.Zero;

			ClipVertex[] clipPoints2 = new ClipVertex[2];
			clipPoints2[0].v = IndexedVector3.Zero;
			clipPoints2[1].v = IndexedVector3.Zero;


			int np;

			// Clip to box side 1
			np = ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, sideOffset1);

			if (np < 2)
			{
				return;
			}

			// Clip to negative box side 1
			np = ClipSegmentToLine(clipPoints2, clipPoints1,  sideNormal, sideOffset2);

			if (np < 2)
			{
				return;
			}

			// Now clipPoints2 contains the clipped points.
			IndexedVector3 manifoldNormal = flip ? -frontNormal : frontNormal;

			int pointCount = 0;
			for (int i = 0; i < b2_maxManifoldPoints; ++i)
			{
				float separation = frontNormal.Dot(clipPoints2[i].v) - frontOffset;

				if (separation <= 0.0f)
				{
					
					//b2ManifoldPoint* cp = manifold.points + pointCount;
					//float separation = separation;
					//cp.localPoint1 = b2MulT(xfA, clipPoints2[i].v);
					//cp.localPoint2 = b2MulT(xfB, clipPoints2[i].v);

					manifold.AddContactPoint(-manifoldNormal,clipPoints2[i].v,separation);

		//			cp.id = clipPoints2[i].id;
		//			cp.id.features.flip = flip;
					++pointCount;
				}
			}

		//	manifold.pointCount = pointCount;}
		}

		public static int ClipSegmentToLine(ClipVertex[] vOut, ClipVertex[] vIn,
							  IndexedVector3 normal, float offset)
		{
			// Start with no output points
			int numOut = 0;

			// Calculate the distance of end points to the line
			float distance0 = normal.Dot(vIn[0].v) - offset;
			float distance1 = normal.Dot(vIn[1].v) - offset;

			// If the points are behind the plane
			if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
			if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

			// If the points are on different sides of the plane
			if (distance0 * distance1 < 0.0f)
			{
				// Find intersection point of edge and plane
				float interp = distance0 / (distance0 - distance1);
				vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
				if (distance0 > 0.0f)
				{
					vOut[numOut].id = vIn[0].id;
				}
				else
				{
					vOut[numOut].id = vIn[1].id;
				}
				++numOut;
			}

			return numOut;
		}

		// Find the separation between poly1 and poly2 for a give edge normal on poly1.
		static float EdgeSeparation(Box2dShape poly1, ref IndexedMatrix xf1, int edge1,
									  Box2dShape poly2, ref IndexedMatrix xf2)
		{
			IndexedVector3[] vertices1 = poly1.GetVertices();
			IndexedVector3[] normals1 = poly1.GetNormals();

			int count2 = poly2.GetVertexCount();
			IndexedVector3[] vertices2 = poly2.GetVertices();

			Debug.Assert(0 <= edge1 && edge1 < poly1.GetVertexCount());

			// Convert normal from poly1's frame into poly2's frame.
			IndexedVector3 normal1World = xf1._basis * normals1[edge1];
            IndexedVector3 normal1 = xf1._basis.Transpose() * normal1World;

			// Find support vertex on poly2 for -normal.
			int index = 0;
			float minDot = MathUtil.BT_LARGE_FLOAT;

			for (int i = 0; i < count2; ++i)
			{
				float dot = vertices2[i].Dot(normal1);
				if (dot < minDot)
				{
					minDot = dot;
					index = i;
				}
			}

            IndexedVector3 v1 = xf1 * vertices1[edge1];
			IndexedVector3 v2 = xf2 * vertices2[index];
			float separation = (v2 - v1).Dot(normal1World);
			return separation;
		}

		// Find the max separation between poly1 and poly2 using edge normals from poly1.
		static float FindMaxSeparation(ref int edgeIndex,
										 Box2dShape poly1, ref IndexedMatrix xf1,
										 Box2dShape poly2, ref IndexedMatrix xf2)
		{
			int count1 = poly1.GetVertexCount();
			IndexedVector3[] normals1 = poly1.GetNormals();

			// Vector pointing from the centroid of poly1 to the centroid of poly2.
			IndexedVector3 d = xf2 * poly2.GetCentroid() - xf1 * poly1.GetCentroid();
            IndexedVector3 dLocal1 = xf1._basis.Transpose() * d;


			// Find edge normal on poly1 that has the largest projection onto d.
			int edge = 0;
			float maxDot = -MathUtil.BT_LARGE_FLOAT;
			for (int i = 0; i < count1; ++i)
			{
				float dot = normals1[i].Dot(ref dLocal1);
				if (dot > maxDot)
				{
					maxDot = dot;
					edge = i;
				}
			}

			// Get the separation for the edge normal.
			float s = EdgeSeparation(poly1, ref xf1, edge, poly2, ref xf2);
			if (s > 0.0f)
			{
				return s;
			}

			// Check the separation for the previous edge normal.
			int prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
			float sPrev = EdgeSeparation(poly1, ref xf1, prevEdge, poly2, ref xf2);
			if (sPrev > 0.0f)
			{
				return sPrev;
			}

			// Check the separation for the next edge normal.
			int nextEdge = edge + 1 < count1 ? edge + 1 : 0;
			float sNext = EdgeSeparation(poly1, ref xf1, nextEdge, poly2, ref xf2);
			if (sNext > 0.0f)
			{
				return sNext;
			}

			// Find the best edge and the search direction.
			int bestEdge;
			float bestSeparation;
			int increment;
			if (sPrev > s && sPrev > sNext)
			{
				increment = -1;
				bestEdge = prevEdge;
				bestSeparation = sPrev;
			}
			else if (sNext > s)
			{
				increment = 1;
				bestEdge = nextEdge;
				bestSeparation = sNext;
			}
			else
			{
				edgeIndex = edge;
				return s;
			}

			// Perform a local search for the best edge normal.
			for ( ; ; )
			{
				if (increment == -1)
					edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
				else
					edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;

				s = EdgeSeparation(poly1, ref xf1, edge, poly2, ref xf2);
				if (s > 0.0f)
				{
					return s;
				}

				if (s > bestSeparation)
				{
					bestEdge = edge;
					bestSeparation = s;
				}
				else
				{
					break;
				}
			}

			edgeIndex = bestEdge;
			return bestSeparation;
		}

		static void FindIncidentEdge(ClipVertex[] c,
									 Box2dShape poly1, ref IndexedMatrix xf1, int edge1,
									 Box2dShape poly2, ref IndexedMatrix xf2)
		{
			IndexedVector3[] normals1 = poly1.GetNormals();

			int count2 = poly2.GetVertexCount();
			IndexedVector3[] vertices2 = poly2.GetVertices();
			IndexedVector3[] normals2 = poly2.GetNormals();

			Debug.Assert(0 <= edge1 && edge1 < poly1.GetVertexCount());

			// Get the normal of the reference edge in poly2's frame.
            IndexedVector3 normal1 = xf2._basis.Transpose() * (xf1._basis * normals1[edge1]);

			// Find the incident edge on poly2.
			int index = 0;
			float minDot = MathUtil.BT_LARGE_FLOAT;
			for (int i = 0; i < count2; ++i)
			{
				float dot = normal1.Dot(normals2[i]);
				if (dot < minDot)
				{
					minDot = dot;
					index = i;
				}
			}

			// Build the clip vertices for the incident edge.
			int i1 = index;
			int i2 = i1 + 1 < count2 ? i1 + 1 : 0;

			c[0].v = xf2 * vertices2[i1];
		//	c[0].id.features.referenceEdge = (unsigned char)edge1;
		//	c[0].id.features.incidentEdge = (unsigned char)i1;
		//	c[0].id.features.incidentVertex = 0;

			c[1].v = xf2 * vertices2[i2];
		//	c[1].id.features.referenceEdge = (unsigned char)edge1;
		//	c[1].id.features.incidentEdge = (unsigned char)i2;
		//	c[1].id.features.incidentVertex = 1;
		}


		//#define btCrossS(a,s) IndexedVector3(s * a.getY(), -s * a.getX(),0.f)

		public static IndexedVector3 CrossS(ref IndexedVector3 a, float s)
		{
			return new IndexedVector3(s*a.Y,-s * a.X,0);
		}

		private bool m_ownManifold;
		private PersistentManifold m_manifoldPtr;

		private const int b2_maxManifoldPoints = 2;
	}





	public struct ClipVertex
	{
		public IndexedVector3 v;
		public int id;
		//b2ContactID id;
		//b2ContactID id;
	}

	//#define b2Dot(a,b) (a).dot(b)
	//#define b2Mul(a,b) (a)*(b)
	//#define b2MulT(a,b) (a).transpose()*(b)
	//#define b2Cross(a,b) (a).cross(b)







	public class Box2dBox2dCreateFunc : CollisionAlgorithmCreateFunc
	{
		public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
		{
			return new Box2dBox2dCollisionAlgorithm(null, ci, body0, body1);
		}
	}
}
