using System.Diagnostics;
using BulletXNA.LinearMath;
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletCollision
{
    public static class PolyhedralContactClipping
    {
        public static int gActualSATPairTests = 0;


		public static void ClipHullAgainstHull(Vector3 separatingNormal, ConvexPolyhedron hullA, ConvexPolyhedron hullB, Matrix transA, Matrix transB, float minDist, float maxDist, IDiscreteCollisionDetectorInterfaceResult resultOut)
		{
			ClipHullAgainstHull(ref separatingNormal, hullA, hullB, ref transA, ref transB, minDist, maxDist, resultOut);
		}
		
		public static void ClipHullAgainstHull(ref Vector3 separatingNormal, ConvexPolyhedron hullA, ConvexPolyhedron hullB, ref Matrix transA, ref Matrix transB, float minDist, float maxDist, IDiscreteCollisionDetectorInterfaceResult resultOut)
        {
	float curMaxDist=maxDist;
	int closestFaceB=-1;

	{
		float dmax = float.MinValue;
		for(int face=0;face<hullB.m_faces.Count;face++)
		{
			Vector3 Normal = new Vector3(hullB.m_faces[face].m_plane[0], hullB.m_faces[face].m_plane[1], hullB.m_faces[face].m_plane[2]);
			Vector3 WorldNormal = Vector3.TransformNormal(Normal,transB);

			float d = Vector3.Dot(WorldNormal,separatingNormal);
			if (d > dmax)
			{
				dmax = d;
				closestFaceB = face;
			}
		}
	}



	if (closestFaceB<0)
	{
		return;
	}



	// setup initial clip face (minimizing face from hull B)
	ObjectArray<Vector3> worldVertsB1 = new ObjectArray<Vector3>();
	{
		Face polyB = hullB.m_faces[closestFaceB];
		int numVertices = polyB.m_indices.Count;
		for(int e0=0;e0<numVertices;e0++)
		{
			Vector3 b = hullB.m_vertices[polyB.m_indices[e0]];
            // check this to see if it is transposed version
			worldVertsB1.Add(MathUtil.TransposeTransformNormal(ref b,ref transB));
		}
	}

	ClipFaceAgainstHull(ref separatingNormal, hullA, ref transA,worldVertsB1, minDist, maxDist,resultOut);

        }

		public static void ClipFaceAgainstHull(Vector3 separatingNormal, ConvexPolyhedron hullA, Matrix transA, ObjectArray<Vector3> worldVertsB1, float minDist, float maxDist, IDiscreteCollisionDetectorInterfaceResult resultOut)
		{
			ClipFaceAgainstHull(ref separatingNormal, hullA, ref transA, worldVertsB1, minDist, maxDist, resultOut);
		}

		
		public static void ClipFaceAgainstHull(ref Vector3 separatingNormal, ConvexPolyhedron hullA, ref Matrix transA, ObjectArray<Vector3> worldVertsB1, float minDist, float maxDist, IDiscreteCollisionDetectorInterfaceResult resultOut)
        {
	ObjectArray<Vector3> worldVertsB2 = new ObjectArray<Vector3>();
	ObjectArray<Vector3> pVtxIn = worldVertsB1;
	ObjectArray<Vector3> pVtxOut = worldVertsB2;
	pVtxOut.Capacity = pVtxIn.Count;

	int closestFaceA=-1;
	{
		float dmin = float.MaxValue;
		for(int face=0;face<hullA.m_faces.Count;face++)
		{
			Vector3 Normal = new Vector3(hullA.m_faces[face].m_plane[0], hullA.m_faces[face].m_plane[1], hullA.m_faces[face].m_plane[2]);
			Vector3 faceANormalWS = Vector3.TransformNormal(Normal,transA);
		
			float d = Vector3.Dot(faceANormalWS,separatingNormal);
			if (d < dmin)
			{
				dmin = d;
				closestFaceA = face;
			}
		}
	}
	if (closestFaceA<0)
		return;

	Face polyA = hullA.m_faces[closestFaceA];

		// clip polygon to back of planes of all faces of hull A that are adjacent to witness face
	int numContacts = pVtxIn.Count;
	int numVerticesA = polyA.m_indices.Count;
	for(int e0=0;e0<numVerticesA;e0++)
	{
		/*const Vector3& a = hullA.m_vertices[polyA.m_indices[e0]];
		const Vector3& b = hullA.m_vertices[polyA.m_indices[(e0+1)%numVerticesA]];
		const Vector3 edge0 = a - b;
		const Vector3 WorldEdge0 = transA.getBasis() * edge0;
		*/

		int otherFace = polyA.m_connectedFaces[e0];
		Vector3 localPlaneNormal = new Vector3(hullA.m_faces[otherFace].m_plane[0],hullA.m_faces[otherFace].m_plane[1],hullA.m_faces[otherFace].m_plane[2]);
		float localPlaneEq = hullA.m_faces[otherFace].m_plane[3];

		Vector3 planeNormalWS = Vector3.TransformNormal(localPlaneNormal,transA);
		float planeEqWS=localPlaneEq-Vector3.Dot(planeNormalWS,transA.Translation);
		//clip face

		ClipFace(pVtxIn, pVtxOut,ref planeNormalWS,planeEqWS);

        //btSwap(pVtxIn,pVtxOut);
        ObjectArray<Vector3> temp = pVtxIn;
        pVtxIn = pVtxOut;
        pVtxOut = temp;

		pVtxOut.Clear();
	}



//#define ONLY_REPORT_DEEPEST_POINT

	Vector3 point;
	

	// only keep points that are behind the witness face
	{
		Vector3 localPlaneNormal = new Vector3(polyA.m_plane[0],polyA.m_plane[1],polyA.m_plane[2]);
		float localPlaneEq = polyA.m_plane[3];
		Vector3 planeNormalWS = Vector3.TransformNormal(localPlaneNormal,transA);
		float planeEqWS=localPlaneEq-Vector3.Dot(planeNormalWS,transA.Translation);
		for (int i=0;i<pVtxIn.Count;i++)
		{
			
			float depth = Vector3.Dot(planeNormalWS,pVtxIn[i])+planeEqWS;
			if (depth <=maxDist && depth >=minDist)
			{
				Vector3 point2 = pVtxIn[i];
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
				resultOut.AddContactPoint(ref separatingNormal,ref point2,depth);
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

		public static bool FindSeparatingAxis(ConvexPolyhedron hullA, ConvexPolyhedron hullB, Matrix transA, Matrix transB, out Vector3 sep)
		{
			return FindSeparatingAxis(hullA,hullB,ref transA,ref transB,out sep);
		}

        public static bool FindSeparatingAxis(ConvexPolyhedron hullA, ConvexPolyhedron hullB, ref Matrix transA, ref Matrix transB, out Vector3 sep)
        {
            gActualSATPairTests++;
			// dummy value to satisfy exit points.
			sep = Vector3.Up;
#if TEST_INTERNAL_OBJECTS
	Vector3 c0 = Vector3.Transform(hullA.mLocalCenter,transA);
	Vector3 c1 = Vector3.Transform(hullB.mLocalCenter,transB);
	Vector3 DeltaC2 = c0 - c1;
#endif

            float dmin = float.MaxValue;
            int curPlaneTests = 0;

            int numFacesA = hullA.m_faces.Count;
            // Test normals from hullA
            for (int i = 0; i < numFacesA; i++)
            {
                Vector3 Normal = new Vector3(hullA.m_faces[i].m_plane[0], hullA.m_faces[i].m_plane[1], hullA.m_faces[i].m_plane[2]);
                Vector3 faceANormalWS = Vector3.TransformNormal(Normal, transA);

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
                Vector3 Normal = new Vector3(hullB.m_faces[i].m_plane[0], hullB.m_faces[i].m_plane[1], hullB.m_faces[i].m_plane[2]);
                Vector3 WorldNormal = Vector3.TransformNormal(Normal, transB);

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

            Vector3 edgeAstart, edgeAend, edgeBstart, edgeBend;

            int curEdgeEdge = 0;
            // Test edges
            for (int e0 = 0; e0 < hullA.m_uniqueEdges.Count; e0++)
            {
                Vector3 edge0 = hullA.m_uniqueEdges[e0];
                Vector3 WorldEdge0 = Vector3.TransformNormal(edge0, transA);
                for (int e1 = 0; e1 < hullB.m_uniqueEdges.Count; e1++)
                {
                    Vector3 edge1 = hullB.m_uniqueEdges[e1];
                    Vector3 WorldEdge1 = Vector3.TransformNormal(edge1, transB);

                    Vector3 Cross = Vector3.Cross(WorldEdge0, WorldEdge1);
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

            Vector3 deltaC = transB.Translation - transA.Translation;
            if ((Vector3.Dot(deltaC, sep)) > 0.0f)
            {
                sep = -sep;
            }

            return true;

        }

        ///the clipFace method is used internally
        public static void ClipFace(ObjectArray<Vector3> pVtxIn, ObjectArray<Vector3> ppVtxOut, ref Vector3 planeNormalWS, float planeEqWS)
        {
            int ve;
            float ds, de;
            int numVerts = pVtxIn.Count;
            if (numVerts < 2)
                return;

            Vector3 firstVertex = pVtxIn[pVtxIn.Count - 1];
            Vector3 endVertex = pVtxIn[0];

            Vector3.Dot(ref planeNormalWS, ref firstVertex, out ds);
            ds += planeEqWS;

            for (ve = 0; ve < numVerts; ve++)
            {
                endVertex = pVtxIn[ve];

                Vector3.Dot(ref planeNormalWS, ref endVertex, out de);
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

        public static bool TestSepAxis(ConvexPolyhedron hullA, ConvexPolyhedron hullB, ref Matrix transA, ref Matrix transB, ref Vector3 sep_axis, out float depth)
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
