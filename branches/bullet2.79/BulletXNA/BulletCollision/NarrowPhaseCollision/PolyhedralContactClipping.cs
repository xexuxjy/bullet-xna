using System.Diagnostics;
using BulletXNA.LinearMath;
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletCollision
{
    public static class PolyhedralContactClipping
    {
        public static int gActualSATPairTests = 0;

        public static int gExpectedNbTests = 0;
        public static int gActualNbTests = 0;
        public static bool gUseInternalObject = true;



#if TEST_INTERNAL_OBJECTS

public static void BoxSupport(IndexedVector3 extents, IndexedVector3 sv, IndexedVector3 p)
{
	// This version is ~11.000 cycles (4%) faster overall in one of the tests.
//	IR(p[0]) = IR(extents[0])|(IR(sv[0])&SIGN_BITMASK);
//	IR(p[1]) = IR(extents[1])|(IR(sv[1])&SIGN_BITMASK);
//	IR(p[2]) = IR(extents[2])|(IR(sv[2])&SIGN_BITMASK);
	p[0] = sv[0] < 0.0f ? -extents[0] : extents[0];
	p[1] = sv[1] < 0.0f ? -extents[1] : extents[1];
	p[2] = sv[2] < 0.0f ? -extents[2] : extents[2];
}

void InverseTransformPoint3x3(out IndexedVector3 outVec, ref IndexedVector3 in, ref IndexedMatrix tr)
{
	const btMatrix3x3& rot = tr.getBasis();
	const btVector3& r0 = rot[0];
	const btVector3& r1 = rot[1];
	const btVector3& r2 = rot[2];

	const btScalar x = r0.x()*in.x() + r1.x()*in.y() + r2.x()*in.z();
	const btScalar y = r0.y()*in.x() + r1.y()*in.y() + r2.y()*in.z();
	const btScalar z = r0.z()*in.x() + r1.z()*in.y() + r2.z()*in.z();

	out.setValue(x, y, z);
}

 bool TestInternalObjects( const btTransform& trans0, const btTransform& trans1, const btVector3& delta_c, const btVector3& axis, const btConvexPolyhedron& convex0, const btConvexPolyhedron& convex1, btScalar dmin)
{
	const btScalar dp = delta_c.dot(axis);

	btVector3 localAxis0;
	InverseTransformPoint3x3(localAxis0, axis,trans0);
	btVector3 localAxis1;
	InverseTransformPoint3x3(localAxis1, axis,trans1);

	btScalar p0[3];
	BoxSupport(convex0.m_extents, localAxis0, p0);
	btScalar p1[3];
	BoxSupport(convex1.m_extents, localAxis1, p1);

	const btScalar Radius0 = p0[0]*localAxis0.x() + p0[1]*localAxis0.y() + p0[2]*localAxis0.z();
	const btScalar Radius1 = p1[0]*localAxis1.x() + p1[1]*localAxis1.y() + p1[2]*localAxis1.z();

	const btScalar MinRadius = Radius0>convex0.m_radius ? Radius0 : convex0.m_radius;
	const btScalar MaxRadius = Radius1>convex1.m_radius ? Radius1 : convex1.m_radius;

	const btScalar MinMaxRadius = MaxRadius + MinRadius;
	const btScalar d0 = MinMaxRadius + dp;
	const btScalar d1 = MinMaxRadius - dp;

	const btScalar depth = d0<d1 ? d0:d1;
	if(depth>dmin)
		return false;
	return true;
}
#endif //TEST_INTERNAL_OBJECTS



        public static void ClipHullAgainstHull(IndexedVector3 separatingNormal, ConvexPolyhedron hullA, ConvexPolyhedron hullB, IndexedMatrix transA, IndexedMatrix transB, float minDist, float maxDist, IDiscreteCollisionDetectorInterfaceResult resultOut)
        {
            ClipHullAgainstHull(ref separatingNormal, hullA, hullB, ref transA, ref transB, minDist, maxDist, resultOut);
        }

        public static void ClipHullAgainstHull(ref IndexedVector3 separatingNormal, ConvexPolyhedron hullA, ConvexPolyhedron hullB, ref IndexedMatrix transA, ref IndexedMatrix transB, float minDist, float maxDist, IDiscreteCollisionDetectorInterfaceResult resultOut)
        {
            float curMaxDist = maxDist;
            int closestFaceB = -1;

            {
                float dmax = float.MinValue;
                for (int face = 0; face < hullB.m_faces.Count; face++)
                {
                    IndexedVector3 Normal = new IndexedVector3(hullB.m_faces[face].m_plane[0], hullB.m_faces[face].m_plane[1], hullB.m_faces[face].m_plane[2]);
                    IndexedVector3 WorldNormal = transB._basis * Normal;

                    float d = IndexedVector3.Dot(WorldNormal, separatingNormal);
                    if (d > dmax)
                    {
                        dmax = d;
                        closestFaceB = face;
                    }
                }
            }

            if (closestFaceB < 0)
            {
                return;
            }

            // setup initial clip face (minimizing face from hull B)
            ObjectArray<IndexedVector3> worldVertsB1 = new ObjectArray<IndexedVector3>();
            {
                Face polyB = hullB.m_faces[closestFaceB];
                int numVertices = polyB.m_indices.Count;
                for (int e0 = 0; e0 < numVertices; e0++)
                {
                    IndexedVector3 b = hullB.m_vertices[polyB.m_indices[e0]];
                    // check this to see if it is transposed version
                    worldVertsB1.Add(transB * b);
                }
            }

            ClipFaceAgainstHull(ref separatingNormal, hullA, ref transA, worldVertsB1, minDist, maxDist, resultOut);

        }

        public static void ClipFaceAgainstHull(IndexedVector3 separatingNormal, ConvexPolyhedron hullA, IndexedMatrix transA, ObjectArray<IndexedVector3> worldVertsB1, float minDist, float maxDist, IDiscreteCollisionDetectorInterfaceResult resultOut)
        {
            ClipFaceAgainstHull(ref separatingNormal, hullA, ref transA, worldVertsB1, minDist, maxDist, resultOut);
        }


        public static void ClipFaceAgainstHull(ref IndexedVector3 separatingNormal, ConvexPolyhedron hullA, ref IndexedMatrix transA, ObjectArray<IndexedVector3> worldVertsB1, float minDist, float maxDist, IDiscreteCollisionDetectorInterfaceResult resultOut)
        {
            ObjectArray<IndexedVector3> worldVertsB2 = new ObjectArray<IndexedVector3>();
            ObjectArray<IndexedVector3> pVtxIn = worldVertsB1;
            ObjectArray<IndexedVector3> pVtxOut = worldVertsB2;
            pVtxOut.Capacity = pVtxIn.Count;

            int closestFaceA = -1;
            {
                float dmin = float.MaxValue;
                for (int face = 0; face < hullA.m_faces.Count; face++)
                {
                    IndexedVector3 Normal = new IndexedVector3(hullA.m_faces[face].m_plane[0], hullA.m_faces[face].m_plane[1], hullA.m_faces[face].m_plane[2]);
                    IndexedVector3 faceANormalWS = transA._basis * Normal;

                    float d = IndexedVector3.Dot(faceANormalWS, separatingNormal);
                    if (d < dmin)
                    {
                        dmin = d;
                        closestFaceA = face;
                    }
                }
            }
            if (closestFaceA < 0)
                return;

            Face polyA = hullA.m_faces[closestFaceA];

            // clip polygon to back of planes of all faces of hull A that are adjacent to witness face
            int numContacts = pVtxIn.Count;
            int numVerticesA = polyA.m_indices.Count;
            for (int e0 = 0; e0 < numVerticesA; e0++)
            {
                /*const IndexedVector3& a = hullA.m_vertices[polyA.m_indices[e0]];
                const IndexedVector3& b = hullA.m_vertices[polyA.m_indices[(e0+1)%numVerticesA]];
                const IndexedVector3 edge0 = a - b;
                const IndexedVector3 WorldEdge0 = transA.getBasis() * edge0;
                */

                int otherFace = polyA.m_connectedFaces[e0];
                IndexedVector3 localPlaneNormal = new IndexedVector3(hullA.m_faces[otherFace].m_plane[0], hullA.m_faces[otherFace].m_plane[1], hullA.m_faces[otherFace].m_plane[2]);
                float localPlaneEq = hullA.m_faces[otherFace].m_plane[3];

                IndexedVector3 planeNormalWS = transA._basis * localPlaneNormal;
                float planeEqWS = localPlaneEq - IndexedVector3.Dot(planeNormalWS, transA._origin);
                //clip face

                ClipFace(pVtxIn, pVtxOut, ref planeNormalWS, planeEqWS);

                //btSwap(pVtxIn,pVtxOut);
                ObjectArray<IndexedVector3> temp = pVtxIn;
                pVtxIn = pVtxOut;
                pVtxOut = temp;

                pVtxOut.Clear();
            }



            //#define ONLY_REPORT_DEEPEST_POINT

            IndexedVector3 point;


            // only keep points that are behind the witness face
            {
                IndexedVector3 localPlaneNormal = new IndexedVector3(polyA.m_plane[0], polyA.m_plane[1], polyA.m_plane[2]);
                float localPlaneEq = polyA.m_plane[3];
                IndexedVector3 planeNormalWS = transA._basis * localPlaneNormal;
                float planeEqWS = localPlaneEq - IndexedVector3.Dot(planeNormalWS, transA._origin);
                for (int i = 0; i < pVtxIn.Count; i++)
                {

                    float depth = IndexedVector3.Dot(planeNormalWS, pVtxIn[i]) + planeEqWS;
                    if (depth <= maxDist && depth >= minDist)
                    {
                        IndexedVector3 point2 = pVtxIn[i];
#if ONLY_REPORT_DEEPEST_POINT
				curMaxDist = depth;
#else
#if false
				if (depth<-3)
				{
					printf("error in btPolyhedralContactClipping depth = %f\n", depth);
					printf("likely wrong separatingNormal passed in\n");
				} 
#endif
                        resultOut.AddContactPoint(ref separatingNormal, ref point2, depth);
#endif
                    }
                }
            }
#if ONLY_REPORT_DEEPEST_POINT
	if (curMaxDist<maxDist)
	{
		resultOut.AddContactPoint(ref separatingNormal,ref point,curMaxDist);
	}
#endif //ONLY_REPORT_DEEPEST_POINT


        }

        public static bool FindSeparatingAxis(ConvexPolyhedron hullA, ConvexPolyhedron hullB, IndexedMatrix transA, IndexedMatrix transB, out IndexedVector3 sep)
        {
            return FindSeparatingAxis(hullA, hullB, ref transA, ref transB, out sep);
        }

        public static bool FindSeparatingAxis(ConvexPolyhedron hullA, ConvexPolyhedron hullB, ref IndexedMatrix transA, ref IndexedMatrix transB, out IndexedVector3 sep)
        {
            gActualSATPairTests++;
            // dummy value to satisfy exit points.
            sep = new IndexedVector3(0, 1, 0);
#if TEST_INTERNAL_OBJECTS
	IndexedVector3 c0 = IndexedVector3.Transform(hullA.mLocalCenter,transA);
	IndexedVector3 c1 = IndexedVector3.Transform(hullB.mLocalCenter,transB);
	IndexedVector3 DeltaC2 = c0 - c1;
#endif

            float dmin = float.MaxValue;
            int curPlaneTests = 0;

            int numFacesA = hullA.m_faces.Count;
            // Test normals from hullA
            for (int i = 0; i < numFacesA; i++)
            {
                IndexedVector3 Normal = new IndexedVector3(hullA.m_faces[i].m_plane[0], hullA.m_faces[i].m_plane[1], hullA.m_faces[i].m_plane[2]);
                IndexedVector3 faceANormalWS = transA._basis * Normal;

                curPlaneTests++;
#if TEST_INTERNAL_OBJECTS
		gExpectedNbTests++;
		if(gUseInternalObject && !TestInternalObjects(ref transA,ref transB,DeltaC2, ref faceANormalWS, hullA, hullB, dmin))
			continue;
		gActualNbTests++;
#endif

                float d;
                if (!TestSepAxis(hullA, hullB, ref transA, ref transB, ref faceANormalWS, out d))
                    return false;

                if (d < dmin)
                {
                    dmin = d;
                    sep = faceANormalWS;
                }
            }

            int numFacesB = hullB.m_faces.Count;
            // Test normals from hullB
            for (int i = 0; i < numFacesB; i++)
            {
                IndexedVector3 Normal = new IndexedVector3(hullB.m_faces[i].m_plane[0], hullB.m_faces[i].m_plane[1], hullB.m_faces[i].m_plane[2]);
                IndexedVector3 WorldNormal = transB._basis * Normal;

                curPlaneTests++;
#if TEST_INTERNAL_OBJECTS
		gExpectedNbTests++;
		if(gUseInternalObject && !TestInternalObjects(ref transA,ref transB,DeltaC2, ref WorldNormal, hullA, hullB, dmin))
			continue;
		gActualNbTests++;
#endif

                float d;
                if (!TestSepAxis(hullA, hullB, ref transA, ref transB, ref WorldNormal, out d))
                    return false;

                if (d < dmin)
                {
                    dmin = d;
                    sep = WorldNormal;
                }
            }

            IndexedVector3 edgeAstart, edgeAend, edgeBstart, edgeBend;

            int curEdgeEdge = 0;
            // Test edges
            for (int e0 = 0; e0 < hullA.m_uniqueEdges.Count; e0++)
            {
                IndexedVector3 edge0 = hullA.m_uniqueEdges[e0];
                IndexedVector3 WorldEdge0 = transA._basis * edge0;
                for (int e1 = 0; e1 < hullB.m_uniqueEdges.Count; e1++)
                {
                    IndexedVector3 edge1 = hullB.m_uniqueEdges[e1];
                    IndexedVector3 WorldEdge1 = transB._basis * edge1;

                    IndexedVector3 Cross = IndexedVector3.Cross(WorldEdge0, WorldEdge1);
                    curEdgeEdge++;
                    if (!MathUtil.IsAlmostZero(ref Cross))
                    {
                        Cross.Normalize();

#if TEST_INTERNAL_OBJECTS
				gExpectedNbTests++;
				if(gUseInternalObject && !TestInternalObjects(transA,transB,DeltaC2, Cross, hullA, hullB, dmin))
					continue;
				gActualNbTests++;
#endif

                        float dist;
                        if (!TestSepAxis(hullA, hullB, ref transA, ref transB, ref Cross, out dist))
                        {
                            return false;
                        }

                        if (dist < dmin)
                        {
                            dmin = dist;
                            sep = Cross;
                        }
                    }
                }

            }

            IndexedVector3 deltaC = transB._origin - transA._origin;
            if ((IndexedVector3.Dot(deltaC, sep)) > 0.0f)
            {
                sep = -sep;
            }

            return true;

        }

        ///the clipFace method is used internally
        public static void ClipFace(ObjectArray<IndexedVector3> pVtxIn, ObjectArray<IndexedVector3> ppVtxOut, ref IndexedVector3 planeNormalWS, float planeEqWS)
        {
            int ve;
            float ds, de;
            int numVerts = pVtxIn.Count;
            if (numVerts < 2)
                return;

            IndexedVector3 firstVertex = pVtxIn[pVtxIn.Count - 1];
            IndexedVector3 endVertex = pVtxIn[0];

            IndexedVector3.Dot(ref planeNormalWS, ref firstVertex, out ds);
            ds += planeEqWS;

            for (ve = 0; ve < numVerts; ve++)
            {
                endVertex = pVtxIn[ve];

                IndexedVector3.Dot(ref planeNormalWS, ref endVertex, out de);
                de += planeEqWS;

                if (ds < 0)
                {
                    if (de < 0)
                    {
                        // Start < 0, end < 0, so output endVertex
                        ppVtxOut.Add(endVertex);
                    }
                    else
                    {
                        // Start < 0, end >= 0, so output intersection
                        ppVtxOut.Add(MathUtil.Vector3Lerp(ref firstVertex, ref endVertex, (float)(ds * 1.0f / (ds - de))));
                    }
                }
                else
                {
                    if (de < 0)
                    {
                        // Start >= 0, end < 0 so output intersection and end
                        ppVtxOut.Add(MathUtil.Vector3Lerp(ref firstVertex, ref endVertex, (float)(ds * 1.0f / (ds - de))));
                        ppVtxOut.Add(endVertex);
                    }
                }
                firstVertex = endVertex;
                ds = de;
            }

        }

        public static bool TestSepAxis(ConvexPolyhedron hullA, ConvexPolyhedron hullB, ref IndexedMatrix transA, ref IndexedMatrix transB, ref IndexedVector3 sep_axis, out float depth)
        {
            float Min0, Max0;
            float Min1, Max1;
            hullA.Project(ref transA, ref sep_axis, out Min0, out Max0);
            hullB.Project(ref transB, ref sep_axis, out Min1, out Max1);

            if (Max0 < Min1 || Max1 < Min0)
            {
                depth = 0;
                return false;
            }

            float d0 = Max0 - Min1;
            Debug.Assert(d0 >= 0.0f);
            float d1 = Max1 - Min0;
            Debug.Assert(d1 >= 0.0f);
            depth = d0 < d1 ? d0 : d1;
            return true;
        }


    }
}
