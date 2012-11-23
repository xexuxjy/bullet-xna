#define BT_INTERNAL_EDGE_DEBUG_DRAW
#define DEBUG_INTERNAL_EDGE
using System;
using System.Diagnostics;
using BulletXNA.LinearMath;




namespace BulletXNA.BulletCollision.CollisionDispatch
{
    public class InternalEdgeUtility
    {

        ///Enable the BT_INTERNAL_EDGE_DEBUG_DRAW define and call btSetDebugDrawer, to get visual info to see if the internal edge utility works properly.
        ///If the utility doesn't work properly, you might have to adjust the threshold values in btTriangleInfoMap
        //#define BT_INTERNAL_EDGE_DEBUG_DRAW

        //#define DEBUG_INTERNAL_EDGE



#if BT_INTERNAL_EDGE_DEBUG_DRAW

        public static void DebugDrawLine(Vector3 from, Vector3 to, Vector3 color)
        {
            if (BulletGlobals.gDebugDraw != null)
            {
                BulletGlobals.gDebugDraw.DrawLine(ref from, ref to, ref color);
            }
        }

        public static void DebugDrawLine(ref Vector3 from, ref Vector3 to, ref Vector3 color)
        {
            if (BulletGlobals.gDebugDraw != null)
            {
                BulletGlobals.gDebugDraw.DrawLine(ref from, ref to, ref color);
            }
        }
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW


        public static int GetHash(int partId, int triangleIndex)
        {
            int hash = (partId << (31 - QuantizedBvh.MAX_NUM_PARTS_IN_BITS)) | triangleIndex;
            return hash;
        }

        public static float GetAngle(ref Vector3 edgeA, ref Vector3 normalA, ref Vector3 normalB)
        {
            Vector3 refAxis0 = edgeA;
            Vector3 refAxis1 = normalA;
            Vector3 swingAxis = normalB;
            float angle = (float)Math.Atan2(Vector3.Dot(ref swingAxis, ref refAxis0), Vector3.Dot(ref swingAxis, ref refAxis1));
            return angle;
        }


        public class ConnectivityProcessor : ITriangleCallback
        {
            public int m_partIdA;
            public int m_triangleIndexA;
            public Vector3[] m_triangleVerticesA;
            public TriangleInfoMap m_triangleInfoMap;

            public virtual void ProcessTriangle(Vector3[] triangle, int partId, int triangleIndex)
            {
                //skip self-collisions
                if ((m_partIdA == partId) && (m_triangleIndexA == triangleIndex))
                {
                    return;
                }

                //skip duplicates (disabled for now)
                //if ((m_partIdA <= partId) && (m_triangleIndexA <= triangleIndex))
                //	return;

                //search for shared vertices and edges
                int numshared = 0;
                int[] sharedVertsA = new int[] { -1, -1, -1 };
                int[] sharedVertsB = new int[] { -1, -1, -1 };

                ///skip degenerate triangles
                float crossBSqr = Vector3.Cross((triangle[1] - triangle[0]), (triangle[2] - triangle[0])).LengthSquared();
                if (crossBSqr < m_triangleInfoMap.m_equalVertexThreshold)
                {
                    return;
                }

                float crossASqr = Vector3.Cross((m_triangleVerticesA[1] - m_triangleVerticesA[0]), (m_triangleVerticesA[2] - m_triangleVerticesA[0])).LengthSquared();
                ///skip degenerate triangles
                if (crossASqr < m_triangleInfoMap.m_equalVertexThreshold)
                {
                    return;
                }

#if false
                printf("triangle A[0]	=	(%f,%f,%f)\ntriangle A[1]	=	(%f,%f,%f)\ntriangle A[2]	=	(%f,%f,%f)\n",
                    m_triangleVerticesA[0].GetX(),m_triangleVerticesA[0].GetY(),m_triangleVerticesA[0].GetZ(),
                    m_triangleVerticesA[1].GetX(),m_triangleVerticesA[1].GetY(),m_triangleVerticesA[1].GetZ(),
                    m_triangleVerticesA[2].GetX(),m_triangleVerticesA[2].GetY(),m_triangleVerticesA[2].GetZ());

                printf("partId=%d, triangleIndex=%d\n",partId,triangleIndex);
                printf("triangle B[0]	=	(%f,%f,%f)\ntriangle B[1]	=	(%f,%f,%f)\ntriangle B[2]	=	(%f,%f,%f)\n",
                    triangle[0].GetX(),triangle[0].GetY(),triangle[0].GetZ(),
                    triangle[1].GetX(),triangle[1].GetY(),triangle[1].GetZ(),
                    triangle[2].GetX(),triangle[2].GetY(),triangle[2].GetZ());
#endif

                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        if ((m_triangleVerticesA[i] - triangle[j]).LengthSquared() < m_triangleInfoMap.m_equalVertexThreshold)
                        {
                            sharedVertsA[numshared] = i;
                            sharedVertsB[numshared] = j;
                            numshared++;
                            ///degenerate case
                            if (numshared >= 3)
                            {
                                return;
                            }
                        }
                    }
                    ///degenerate case
                    if (numshared >= 3)
                    {
                        return;
                    }
                }
                switch (numshared)
                {
                    case 0:
                        {
                            break;
                        }
                    case 1:
                        {
                            //shared vertex
                            break;
                        }
                    case 2:
                        {
                            //shared edge
                            //we need to make sure the edge is in the order V2V0 and not V0V2 so that the signs are correct
                            if (sharedVertsA[0] == 0 && sharedVertsA[1] == 2)
                            {
                                sharedVertsA[0] = 2;
                                sharedVertsA[1] = 0;
                                int tmp = sharedVertsB[1];
                                sharedVertsB[1] = sharedVertsB[0];
                                sharedVertsB[0] = tmp;
                            }

                            int hash = GetHash(m_partIdA, m_triangleIndexA);


                            TriangleInfo info = null;
                            if (m_triangleInfoMap.ContainsKey(hash))
                            {
                                info = m_triangleInfoMap[hash];
                            }
                            else
                            {
                                info = new TriangleInfo();
                                m_triangleInfoMap[hash] = info;
                            }

                            int sumvertsA = sharedVertsA[0] + sharedVertsA[1];
                            int otherIndexA = 3 - sumvertsA;


                            Vector3 edge = new Vector3(m_triangleVerticesA[sharedVertsA[1]] - m_triangleVerticesA[sharedVertsA[0]]);

                            TriangleShape tA = new TriangleShape(m_triangleVerticesA[0], m_triangleVerticesA[1], m_triangleVerticesA[2]);
                            int otherIndexB = 3 - (sharedVertsB[0] + sharedVertsB[1]);

                            TriangleShape tB = new TriangleShape(triangle[sharedVertsB[1]], triangle[sharedVertsB[0]], triangle[otherIndexB]);
                            //btTriangleShape tB(triangle[0],triangle[1],triangle[2]);

                            Vector3 normalA;
                            Vector3 normalB;
                            tA.CalcNormal(out normalA);
                            tB.CalcNormal(out normalB);
                            edge.Normalize();
                            Vector3 edgeCrossA = Vector3.Normalize(Vector3.Cross(edge, normalA));

                            {
                                Vector3 tmp = m_triangleVerticesA[otherIndexA] - m_triangleVerticesA[sharedVertsA[0]];
                                if (Vector3.Dot(edgeCrossA, tmp) < 0)
                                {
                                    edgeCrossA *= -1;
                                }
                            }

                            Vector3 edgeCrossB = Vector3.Cross(edge, normalB).Normalized();

                            {
                                Vector3 tmp = triangle[otherIndexB] - triangle[sharedVertsB[0]];
                                if (Vector3.Dot(edgeCrossB, tmp) < 0)
                                {
                                    edgeCrossB *= -1;
                                }
                            }

                            float angle2 = 0;
                            float ang4 = 0.0f;

                            Vector3 calculatedEdge = Vector3.Cross(edgeCrossA, edgeCrossB);
                            float len2 = calculatedEdge.LengthSquared();

                            float correctedAngle = 0f;
                            Vector3 calculatedNormalB = normalA;
                            bool isConvex = false;

                            if (len2 < m_triangleInfoMap.m_planarEpsilon)
                            {
                                angle2 = 0.0f;
                                ang4 = 0.0f;
                            }
                            else
                            {
                                calculatedEdge.Normalize();
                                Vector3 calculatedNormalA = Vector3.Cross(calculatedEdge, edgeCrossA);
                                calculatedNormalA.Normalize();
                                angle2 = GetAngle(ref calculatedNormalA, ref edgeCrossA, ref edgeCrossB);
                                ang4 = MathUtil.SIMD_PI - angle2;
                                float dotA = Vector3.Dot(normalA, edgeCrossB);
                                ///@todo: check if we need some epsilon, due to floating point imprecision
                                isConvex = (dotA < 0f);

                                correctedAngle = isConvex ? ang4 : -ang4;
                                Quaternion orn2 = Quaternion.CreateFromAxisAngle(calculatedEdge, -correctedAngle);
                                Matrix rotateMatrix = Matrix.CreateFromQuaternion(orn2);
                                calculatedNormalB = new IndexedBasisMatrix(orn2) * normalA;
                            }


                            //alternatively use 
                            //Vector3 calculatedNormalB2 = quatRotate(orn,normalA);


                            switch (sumvertsA)
                            {
                                case 1:
                                    {
                                        Vector3 edge1 = m_triangleVerticesA[0] - m_triangleVerticesA[1];
                                        Quaternion orn = Quaternion.CreateFromAxisAngle(edge1, -correctedAngle);
                                        Vector3 computedNormalB = MathUtil.QuatRotate(orn, normalA);
                                        float bla = Vector3.Dot(computedNormalB, normalB);
                                        if (bla < 0)
                                        {
                                            computedNormalB *= -1;
                                            info.m_flags |= TriangleInfoMap.TRI_INFO_V0V1_SWAP_NORMALB;
                                        }
#if DEBUG_INTERNAL_EDGE
                                        if ((computedNormalB - normalB).Length() > 0.0001f)
                                        {
                                            System.Console.WriteLine("warning: normals not identical");
                                        }
#endif//DEBUG_INTERNAL_EDGE

                                        info.m_edgeV0V1Angle = -correctedAngle;

                                        if (isConvex)
                                        {
                                            info.m_flags |= TriangleInfoMap.TRI_INFO_V0V1_CONVEX;
                                        }
                                        break;
                                    }
                                case 2:
                                    {
                                        Vector3 edge1 = m_triangleVerticesA[2] - m_triangleVerticesA[0];
                                        Quaternion orn = Quaternion.CreateFromAxisAngle(edge1, -correctedAngle);
                                        Vector3 computedNormalB = MathUtil.QuatRotate(orn, normalA);
                                        if (Vector3.Dot(computedNormalB, normalB) < 0)
                                        {
                                            computedNormalB *= -1;
                                            info.m_flags |= TriangleInfoMap.TRI_INFO_V2V0_SWAP_NORMALB;
                                        }

#if DEBUG_INTERNAL_EDGE
                                        if ((computedNormalB - normalB).Length() > 0.0001)
                                        {
                                            System.Console.WriteLine("warning: normals not identical");
                                        }
#endif //DEBUG_INTERNAL_EDGE
                                        info.m_edgeV2V0Angle = -correctedAngle;
                                        if (isConvex)
                                            info.m_flags |= TriangleInfoMap.TRI_INFO_V2V0_CONVEX;
                                        break;
                                    }
                                case 3:
                                    {
                                        Vector3 edge1 = m_triangleVerticesA[1] - m_triangleVerticesA[2];
                                        Quaternion orn = Quaternion.CreateFromAxisAngle(edge1, -correctedAngle);
                                        Vector3 computedNormalB = MathUtil.QuatRotate(orn, normalA);
                                        if (Vector3.Dot(computedNormalB, normalB) < 0)
                                        {
                                            info.m_flags |= TriangleInfoMap.TRI_INFO_V1V2_SWAP_NORMALB;
                                            computedNormalB *= -1;
                                        }
#if DEBUG_INTERNAL_EDGE
                                        if ((computedNormalB - normalB).Length() > 0.0001)
                                        {
                                            System.Console.WriteLine("warning: normals not identical");
                                        }
#endif //DEBUG_INTERNAL_EDGE
                                        info.m_edgeV1V2Angle = -correctedAngle;

                                        if (isConvex)
                                        {
                                            info.m_flags |= TriangleInfoMap.TRI_INFO_V1V2_CONVEX;
                                        }
                                        break;
                                    }
                            }

                            break;
                        }
                    default:
                        {
                            //				printf("warning: duplicate triangle\n");
                            break;
                        }

                }
            }

            public virtual bool graphics()
            {
                return false;
            }

            public virtual void Cleanup()
            {
            }

        }
        /////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////

        public static void GenerateInternalEdgeInfo(BvhTriangleMeshShape trimeshShape, TriangleInfoMap triangleInfoMap)
        {
            //the user pointer shouldn't already be used for other purposes, we intend to store connectivity info there!
            if (trimeshShape.GetTriangleInfoMap() != null)
            {
                return;
            }

            trimeshShape.SetTriangleInfoMap(triangleInfoMap);

            StridingMeshInterface meshInterface = trimeshShape.MeshInterface;
            Vector3 meshScaling = meshInterface.GetScaling();

            for (int partId = 0; partId < meshInterface.GetNumSubParts(); partId++)
            {
                object vertexbase = null;
                int numverts = 0;
                PHY_ScalarType type = PHY_ScalarType.PHY_INTEGER;
                int stride = 0;
                object indexbase = null;
                int indexstride = 0;
                int numfaces = 0;
                PHY_ScalarType indicestype = PHY_ScalarType.PHY_INTEGER;
                //PHY_ScalarType indexType=0;

                Vector3[] triangleVerts = new Vector3[3];
                meshInterface.GetLockedReadOnlyVertexIndexBase(out vertexbase, out numverts, out type, out stride, out indexbase, out indexstride, out numfaces, out indicestype, partId);
                Vector3 aabbMin, aabbMax;

                switch (indicestype)
                {
                    case PHY_ScalarType.PHY_INTEGER:
                        {
                            int[] indexList = ((ObjectArray<int>)indexbase).GetRawArray();

                            if (vertexbase is ObjectArray<Vector3>)
                            {
                                Vector3[] vertexList = (vertexbase as ObjectArray<Vector3>).GetRawArray();
                                int indexCounter = 0;
                                for (int triangleIndex = 0; triangleIndex < numfaces; triangleIndex++)
                                {
                                    int index1 = indexList[triangleIndex];
                                    int index2 = indexList[triangleIndex + 1];
                                    int index3 = indexList[triangleIndex + 2];

                                    triangleVerts[0] = new Vector3(vertexList[index1]) * meshScaling;
                                    triangleVerts[1] = new Vector3(vertexList[index2]) * meshScaling;
                                    triangleVerts[2] = new Vector3(vertexList[index3]) * meshScaling;
                                    ProcessResult(triangleVerts, out aabbMin, out aabbMax, trimeshShape, partId, triangleIndex, triangleInfoMap);
                                }
                            }
                            else if (vertexbase is ObjectArray<float>)
                            {
                                float[] vertexList = (vertexbase as ObjectArray<float>).GetRawArray();
                                for (int triangleIndex = 0; triangleIndex < numfaces; triangleIndex++)
                                {
                                    triangleVerts[0] = new Vector3(vertexList[indexList[triangleIndex]], vertexList[indexList[triangleIndex] + 1], vertexList[indexList[triangleIndex] + 2]) * meshScaling;
                                    triangleVerts[1] = new Vector3(vertexList[indexList[triangleIndex + 1]], vertexList[indexList[triangleIndex + 1] + 1], vertexList[indexList[triangleIndex + 1] + 2]) * meshScaling;
                                    triangleVerts[2] = new Vector3(vertexList[indexList[triangleIndex + 2]], vertexList[indexList[triangleIndex + 2] + 1], vertexList[indexList[triangleIndex + 2] + 2]) * meshScaling;
                                    ProcessResult(triangleVerts, out aabbMin, out aabbMax, trimeshShape, partId, triangleIndex, triangleInfoMap);
                                }
                            }
                            break;
                        }
                    default:
                        {
                            Debug.Assert(indicestype == PHY_ScalarType.PHY_INTEGER);
                            break;

                        }
                }
            }

        }

        private static void ProcessResult(Vector3[] triangleVerts, out Vector3 aabbMin, out Vector3 aabbMax, BvhTriangleMeshShape trimeshShape, int partId, int triangleIndex, TriangleInfoMap triangleInfoMap)
        {
            aabbMin = MathUtil.MAX_VECTOR;
            aabbMax = MathUtil.MIN_VECTOR;
            aabbMin.SetMin(ref triangleVerts[0]);
            aabbMax.SetMax(ref triangleVerts[0]);
            aabbMin.SetMin(ref triangleVerts[1]);
            aabbMax.SetMax(ref triangleVerts[1]);
            aabbMin.SetMin(ref triangleVerts[2]);
            aabbMax.SetMax(ref triangleVerts[2]);

            ConnectivityProcessor connectivityProcessor = new ConnectivityProcessor();
            connectivityProcessor.m_partIdA = partId;
            connectivityProcessor.m_triangleIndexA = triangleIndex;
            connectivityProcessor.m_triangleVerticesA = triangleVerts;
            connectivityProcessor.m_triangleInfoMap = triangleInfoMap;

            trimeshShape.ProcessAllTriangles(connectivityProcessor, ref aabbMin, ref aabbMax);

        }




        // Given a point and a line segment (defined by two points), compute the closest point
        // in the line.  Cap the point at the endpoints of the line segment.
        public static void NearestPointInLineSegment(ref Vector3 point, ref Vector3 line0, ref Vector3 line1, out Vector3 nearestPoint)
        {
            Vector3 lineDelta = line1 - line0;

            // Handle degenerate lines
            if (MathUtil.FuzzyZero(lineDelta.LengthSquared()))
            {
                nearestPoint = line0;
            }
            else
            {
                float delta = Vector3.Dot((point - line0), (lineDelta)) / Vector3.Dot((lineDelta), (lineDelta));

                // Clamp the point to conform to the segment's endpoints
                if (delta < 0)
                {
                    delta = 0;
                }
                else if (delta > 1)
                {
                    delta = 1;
                }

                nearestPoint = line0 + lineDelta * delta;
            }
        }

        public static bool ClampNormal(Vector3 edge, Vector3 tri_normal_org, Vector3 localContactNormalOnB, float correctedEdgeAngle, out Vector3 clampedLocalNormal)
        {
            return ClampNormal(ref edge, ref tri_normal_org, ref localContactNormalOnB, correctedEdgeAngle, out clampedLocalNormal);
        }

        public static bool ClampNormal(ref Vector3 edge, ref Vector3 tri_normal_org, ref Vector3 localContactNormalOnB, float correctedEdgeAngle, out Vector3 clampedLocalNormal)
        {
            Vector3 tri_normal = tri_normal_org;
            //we only have a local triangle normal, not a local contact normal . only normal in world space...
            //either compute the current angle all in local space, or all in world space

            Vector3 edgeCross = Vector3.Cross(edge, tri_normal).Normalized();
            float curAngle = GetAngle(ref edgeCross, ref tri_normal, ref localContactNormalOnB);

            if (correctedEdgeAngle < 0)
            {
                if (curAngle < correctedEdgeAngle)
                {
                    float diffAngle = correctedEdgeAngle - curAngle;
                    Quaternion rotation = Quaternion.CreateFromAxisAngle(edge, diffAngle);
                    clampedLocalNormal = new IndexedBasisMatrix(rotation) * localContactNormalOnB;
                    return true;
                }
            }

            if (correctedEdgeAngle >= 0)
            {
                if (curAngle > correctedEdgeAngle)
                {
                    float diffAngle = correctedEdgeAngle - curAngle;
                    Quaternion rotation = Quaternion.CreateFromAxisAngle(edge, diffAngle);
                    clampedLocalNormal = new IndexedBasisMatrix(rotation) * localContactNormalOnB;
                    return true;
                }
            }
            clampedLocalNormal = Vector3.Zero;
            return false;
        }



        /// Changes a btManifoldPoint collision normal to the normal from the mesh.
        public static void AdjustInternalEdgeContacts(ManifoldPoint cp, CollisionObject colObj0, CollisionObject colObj1, int partId0, int index0, InternalEdgeAdjustFlags normalAdjustFlags)
        {
            //btAssert(colObj0.GetCollisionShape().GetShapeType() == TRIANGLE_SHAPE_PROXYTYPE);
            if (colObj0.CollisionShape.ShapeType != BroadphaseNativeType.TriangleShape)
                return;

            BvhTriangleMeshShape trimesh = null;

            if (colObj0.RootCollisionShape.ShapeType == BroadphaseNativeType.SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE)
            {
                //trimesh = ((ScaledBvhTriangleMeshShape)colObj0.GetRootCollisionShape()).GetChildShape();
            }
            else
            {
                trimesh = (BvhTriangleMeshShape)colObj0.RootCollisionShape;
            }

            TriangleInfoMap triangleInfoMapPtr = (TriangleInfoMap)trimesh.GetTriangleInfoMap();
            if (triangleInfoMapPtr == null)
            {
                return;
            }

            int hash = GetHash(partId0, index0);

            TriangleInfo info;
            if (!triangleInfoMapPtr.TryGetValue(hash, out info))
            {
                return;
            }


            float frontFacing = (normalAdjustFlags & InternalEdgeAdjustFlags.BT_TRIANGLE_CONVEX_BACKFACE_MODE) == 0 ? 1.0f : -1.0f;

            TriangleShape tri_shape = colObj0.CollisionShape as TriangleShape;
            Vector3 v0, v1, v2;
            tri_shape.GetVertex(0, out v0);
            tri_shape.GetVertex(1, out v1);
            tri_shape.GetVertex(2, out v2);

            Vector3 center = (v0 + v1 + v2) * (1.0f / 3.0f);

            Vector3 red = new Vector3(1, 0, 0), green = new Vector3(0, 1, 0), blue = new Vector3(0, 0, 1), white = new Vector3(1, 1, 1), black = new Vector3(0, 0, 0);
            Vector3 tri_normal;
            tri_shape.CalcNormal(out tri_normal);

            //float dot = tri_normal.dot(cp.m_normalWorldOnB);
            Vector3 nearest;
            NearestPointInLineSegment(ref cp.m_localPointB, ref v0, ref v1, out nearest);

            Vector3 contact = cp.m_localPointB;
#if BT_INTERNAL_EDGE_DEBUG_DRAW
            Matrix tr = colObj0.GetWorldTransform();
            DebugDrawLine(tr * nearest, tr * cp.m_localPointB, red);
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW



            bool isNearEdge = false;

            int numConcaveEdgeHits = 0;
            int numConvexEdgeHits = 0;

            Vector3 localContactNormalOnB = colObj0.GetWorldTransform()._basis.Transpose() * cp.m_normalWorldOnB;
            localContactNormalOnB.Normalize();//is this necessary?

            // Get closest edge
            int bestedge = -1;
            float disttobestedge = MathUtil.BT_LARGE_FLOAT;
            //
            // Edge 0 . 1
            if (Math.Abs(info.m_edgeV0V1Angle) < triangleInfoMapPtr.m_maxEdgeAngleThreshold)
            {
                //Vector3 nearest;
                NearestPointInLineSegment(ref cp.m_localPointB, ref v0, ref v1, out nearest);
                float len = (contact - nearest).Length();
                //
                if (len < disttobestedge)
                {
                    bestedge = 0;
                    disttobestedge = len;
                }
            }
            // Edge 1 . 2
            if (Math.Abs(info.m_edgeV1V2Angle) < triangleInfoMapPtr.m_maxEdgeAngleThreshold)
            {
                //Vector3 nearest;
                NearestPointInLineSegment(ref  cp.m_localPointB, ref v1, ref v2, out nearest);
                float len = (contact - nearest).Length();
                //
                if (len < disttobestedge)
                {
                    bestedge = 1;
                    disttobestedge = len;
                }
            }
            // Edge 2 . 0
            if (Math.Abs(info.m_edgeV2V0Angle) < triangleInfoMapPtr.m_maxEdgeAngleThreshold)
            {
                //Vector3 nearest;
                NearestPointInLineSegment(ref  cp.m_localPointB, ref v2, ref v0, out nearest);
                float len = (contact - nearest).Length();
                //
                if (len < disttobestedge)
                {
                    bestedge = 2;
                    disttobestedge = len;
                }
            }

#if BT_INTERNAL_EDGE_DEBUG_DRAW
            Vector3 upfix = tri_normal * new Vector3(0.1f, 0.1f, 0.1f);
            DebugDrawLine(tr * v0 + upfix, tr * v1 + upfix, red);
#endif
            if (Math.Abs(info.m_edgeV0V1Angle) < triangleInfoMapPtr.m_maxEdgeAngleThreshold)
            {
#if BT_INTERNAL_EDGE_DEBUG_DRAW
                DebugDrawLine(tr * contact, tr * (contact + cp.m_normalWorldOnB * 10), black);
#endif
                float len = (contact - nearest).Length();
                if (len < triangleInfoMapPtr.m_edgeDistanceThreshold)
                    if (bestedge == 0)
                    {
                        Vector3 edge = (v0 - v1);
                        isNearEdge = true;

                        if (info.m_edgeV0V1Angle == 0.0f)
                        {
                            numConcaveEdgeHits++;
                        }
                        else
                        {

                            bool isEdgeConvex = (info.m_flags & TriangleInfoMap.TRI_INFO_V0V1_CONVEX) != 0;
                            float swapFactor = isEdgeConvex ? 1.0f : -1.0f;
#if BT_INTERNAL_EDGE_DEBUG_DRAW
                            DebugDrawLine(tr * nearest, tr * (nearest + swapFactor * tri_normal * 10), white);
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW

                            Vector3 nA = swapFactor * tri_normal;

                            Quaternion orn = new Quaternion(edge, info.m_edgeV0V1Angle);
                            Vector3 computedNormalB = MathUtil.QuatRotate(ref orn, ref tri_normal);
                            if ((info.m_flags & TriangleInfoMap.TRI_INFO_V0V1_SWAP_NORMALB) != 0)
                            {
                                computedNormalB *= -1;
                            }
                            Vector3 nB = swapFactor * computedNormalB;

                            float NdotA = localContactNormalOnB.Dot(ref nA);
                            float NdotB = localContactNormalOnB.Dot(ref nB);
                            bool backFacingNormal = (NdotA < triangleInfoMapPtr.m_convexEpsilon) && (NdotB < triangleInfoMapPtr.m_convexEpsilon);

#if DEBUG_INTERNAL_EDGE
                            {

                                DebugDrawLine(cp.GetPositionWorldOnB(), cp.GetPositionWorldOnB() + tr._basis * (nB * 20), red);
                            }
#endif //DEBUG_INTERNAL_EDGE


                            if (backFacingNormal)
                            {
                                numConcaveEdgeHits++;
                            }
                            else
                            {
                                numConvexEdgeHits++;
                                Vector3 clampedLocalNormal;
                                bool isClamped = ClampNormal(edge, swapFactor * tri_normal, localContactNormalOnB, info.m_edgeV0V1Angle, out clampedLocalNormal);
                                if (isClamped)
                                {
                                    if (((normalAdjustFlags & InternalEdgeAdjustFlags.BT_TRIANGLE_CONVEX_DOUBLE_SIDED) != 0) || (clampedLocalNormal.Dot(frontFacing * tri_normal) > 0))
                                    {
                                        Vector3 newNormal = colObj0.GetWorldTransform()._basis * clampedLocalNormal;
                                        //					cp.m_distance1 = cp.m_distance1 * newNormal.dot(cp.m_normalWorldOnB);
                                        cp.m_normalWorldOnB = newNormal;
                                        // Reproject collision point along normal. (what about cp.m_distance1?)
                                        cp.m_positionWorldOnB = cp.m_positionWorldOnA - cp.m_normalWorldOnB * cp.m_distance1;
                                        cp.m_localPointB = colObj0.GetWorldTransform().InvXform(cp.m_positionWorldOnB);

                                    }
                                }
                            }
                        }
                    }
            }

            NearestPointInLineSegment(ref contact, ref v1, ref v2, out nearest);
#if BT_INTERNAL_EDGE_DEBUG_DRAW
            DebugDrawLine(tr * nearest, tr * cp.m_localPointB, green);
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW

#if BT_INTERNAL_EDGE_DEBUG_DRAW
            DebugDrawLine(tr * v1 + upfix, tr * v2 + upfix, green);
#endif

            if (Math.Abs(info.m_edgeV1V2Angle) < triangleInfoMapPtr.m_maxEdgeAngleThreshold)
            {
#if BT_INTERNAL_EDGE_DEBUG_DRAW
                DebugDrawLine(tr * contact, tr * (contact + cp.m_normalWorldOnB * 10), black);
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW



                float len = (contact - nearest).Length();
                if (len < triangleInfoMapPtr.m_edgeDistanceThreshold)
                    if (bestedge == 1)
                    {
                        isNearEdge = true;
#if BT_INTERNAL_EDGE_DEBUG_DRAW
                        DebugDrawLine(tr * nearest, tr * (nearest + tri_normal * 10), white);
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW

                        Vector3 edge = (v1 - v2);

                        isNearEdge = true;

                        if (info.m_edgeV1V2Angle == 0f)
                        {
                            numConcaveEdgeHits++;
                        }
                        else
                        {
                            bool isEdgeConvex = (info.m_flags & TriangleInfoMap.TRI_INFO_V1V2_CONVEX) != 0;
                            float swapFactor = isEdgeConvex ? 1.0f : -1.0f;
#if BT_INTERNAL_EDGE_DEBUG_DRAW
                            DebugDrawLine(tr * nearest, tr * (nearest + swapFactor * tri_normal * 10), white);
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW

                            Vector3 nA = swapFactor * tri_normal;

                            Quaternion orn = new Quaternion(edge, info.m_edgeV1V2Angle);
                            Vector3 computedNormalB = MathUtil.QuatRotate(ref orn, ref tri_normal);
                            if ((info.m_flags & TriangleInfoMap.TRI_INFO_V1V2_SWAP_NORMALB) != 0)
                            {
                                computedNormalB *= -1;
                            }
                            Vector3 nB = swapFactor * computedNormalB;

#if DEBUG_INTERNAL_EDGE
                            {
                                DebugDrawLine(cp.GetPositionWorldOnB(), cp.GetPositionWorldOnB() + tr._basis * (nB * 20), red);
                            }
#endif //DEBUG_INTERNAL_EDGE


                            float NdotA = localContactNormalOnB.Dot(ref nA);
                            float NdotB = localContactNormalOnB.Dot(ref nB);
                            bool backFacingNormal = (NdotA < triangleInfoMapPtr.m_convexEpsilon) && (NdotB < triangleInfoMapPtr.m_convexEpsilon);

                            if (backFacingNormal)
                            {
                                numConcaveEdgeHits++;
                            }
                            else
                            {
                                numConvexEdgeHits++;
                                Vector3 localContactNormalOnB2 = colObj0.GetWorldTransform()._basis.Transpose() * cp.m_normalWorldOnB;
                                Vector3 clampedLocalNormal;
                                bool isClamped = ClampNormal(edge, swapFactor * tri_normal, localContactNormalOnB2, info.m_edgeV1V2Angle, out clampedLocalNormal);
                                if (isClamped)
                                {
                                    if (((normalAdjustFlags & InternalEdgeAdjustFlags.BT_TRIANGLE_CONVEX_DOUBLE_SIDED) != 0) || (clampedLocalNormal.Dot(frontFacing * tri_normal) > 0))
                                    {
                                        Vector3 newNormal = colObj0.GetWorldTransform()._basis * clampedLocalNormal;
                                        //					cp.m_distance1 = cp.m_distance1 * newNormal.dot(cp.m_normalWorldOnB);
                                        cp.m_normalWorldOnB = newNormal;
                                        // Reproject collision point along normal.
                                        cp.m_positionWorldOnB = cp.m_positionWorldOnA - cp.m_normalWorldOnB * cp.m_distance1;
                                        cp.m_localPointB = colObj0.GetWorldTransform().InvXform(cp.m_positionWorldOnB);
                                    }
                                }
                            }
                        }
                    }
            }

            NearestPointInLineSegment(ref contact, ref v2, ref v0, out nearest);
#if BT_INTERNAL_EDGE_DEBUG_DRAW
            DebugDrawLine(tr * nearest, tr * cp.m_localPointB, blue);
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW
#if BT_INTERNAL_EDGE_DEBUG_DRAW
            DebugDrawLine(tr * v2 + upfix, tr * v0 + upfix, blue);
#endif

            if (Math.Abs(info.m_edgeV2V0Angle) < triangleInfoMapPtr.m_maxEdgeAngleThreshold)
            {

#if BT_INTERNAL_EDGE_DEBUG_DRAW
                DebugDrawLine(tr * contact, tr * (contact + cp.m_normalWorldOnB * 10), black);
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW

                float len = (contact - nearest).Length();
                if (len < triangleInfoMapPtr.m_edgeDistanceThreshold)
                    if (bestedge == 2)
                    {
                        isNearEdge = true;
#if BT_INTERNAL_EDGE_DEBUG_DRAW
                        DebugDrawLine(tr * nearest, tr * (nearest + tri_normal * 10), white);
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW

                        Vector3 edge = (v2 - v0);

                        if (info.m_edgeV2V0Angle == 0f)
                        {
                            numConcaveEdgeHits++;
                        }
                        else
                        {

                            bool isEdgeConvex = (info.m_flags & TriangleInfoMap.TRI_INFO_V2V0_CONVEX) != 0;
                            float swapFactor = isEdgeConvex ? 1.0f : -1.0f;
#if BT_INTERNAL_EDGE_DEBUG_DRAW
                            DebugDrawLine(tr * nearest, tr * (nearest + swapFactor * tri_normal * 10), white);
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW

                            Vector3 nA = swapFactor * tri_normal;
                            Quaternion orn = new Quaternion(edge, info.m_edgeV2V0Angle);
                            Vector3 computedNormalB = MathUtil.QuatRotate(ref orn, ref tri_normal);
                            if ((info.m_flags & TriangleInfoMap.TRI_INFO_V2V0_SWAP_NORMALB) != 0)
                            {
                                computedNormalB *= -1;
                            }
                            Vector3 nB = swapFactor * computedNormalB;

#if DEBUG_INTERNAL_EDGE
                            {
                                DebugDrawLine(cp.GetPositionWorldOnB(), cp.GetPositionWorldOnB() + tr._basis * (nB * 20), red);
                            }
#endif //DEBUG_INTERNAL_EDGE

                            float NdotA = localContactNormalOnB.Dot(ref nA);
                            float NdotB = localContactNormalOnB.Dot(ref nB);
                            bool backFacingNormal = (NdotA < triangleInfoMapPtr.m_convexEpsilon) && (NdotB < triangleInfoMapPtr.m_convexEpsilon);

                            if (backFacingNormal)
                            {
                                numConcaveEdgeHits++;
                            }
                            else
                            {
                                numConvexEdgeHits++;
                                //				printf("hitting convex edge\n");


                                Vector3 localContactNormalOnB2 = colObj0.GetWorldTransform()._basis.Transpose() * cp.m_normalWorldOnB;
                                Vector3 clampedLocalNormal;
                                bool isClamped = ClampNormal(edge, swapFactor * tri_normal, localContactNormalOnB2, info.m_edgeV2V0Angle, out clampedLocalNormal);
                                if (isClamped)
                                {
                                    if (((normalAdjustFlags & InternalEdgeAdjustFlags.BT_TRIANGLE_CONVEX_DOUBLE_SIDED) != 0) || (clampedLocalNormal.Dot(frontFacing * tri_normal) > 0))
                                    {
                                        Vector3 newNormal = colObj0.GetWorldTransform()._basis * clampedLocalNormal;
                                        //					cp.m_distance1 = cp.m_distance1 * newNormal.dot(cp.m_normalWorldOnB);
                                        cp.m_normalWorldOnB = newNormal;
                                        // Reproject collision point along normal.
                                        cp.m_positionWorldOnB = cp.m_positionWorldOnA - cp.m_normalWorldOnB * cp.m_distance1;
                                        cp.m_localPointB = colObj0.GetWorldTransform().InvXform(cp.m_positionWorldOnB);
                                    }
                                }
                            }
                        }


                    }
            }

#if DEBUG_INTERNAL_EDGE
            {
                Vector3 color = new Vector3(0, 1, 1);
                DebugDrawLine(cp.GetPositionWorldOnB(), cp.GetPositionWorldOnB() + cp.m_normalWorldOnB * 10, color);
            }
#endif //DEBUG_INTERNAL_EDGE

            if (isNearEdge)
            {

                if (numConcaveEdgeHits > 0)
                {
                    if ((normalAdjustFlags & InternalEdgeAdjustFlags.BT_TRIANGLE_CONCAVE_DOUBLE_SIDED) != 0)
                    {
                        //fix tri_normal so it pointing the same direction as the current local contact normal
                        if (tri_normal.Dot(ref localContactNormalOnB) < 0)
                        {
                            tri_normal *= -1;
                        }
                        cp.m_normalWorldOnB = colObj0.GetWorldTransform()._basis * tri_normal;
                    }
                    else
                    {
                        Vector3 newNormal = tri_normal * frontFacing;
                        //if the tri_normal is pointing opposite direction as the current local contact normal, skip it
                        float d = newNormal.Dot(ref localContactNormalOnB);
                        if (d < 0)
                        {
                            return;
                        }
                        //modify the normal to be the triangle normal (or backfacing normal)
                        cp.m_normalWorldOnB = colObj0.GetWorldTransform()._basis * newNormal;
                    }

                    // Reproject collision point along normal.
                    cp.m_positionWorldOnB = cp.m_positionWorldOnA - cp.m_normalWorldOnB * cp.m_distance1;
                    cp.m_localPointB = colObj0.GetWorldTransform().InvXform(cp.m_positionWorldOnB);
                }
            }
        }


        public enum InternalEdgeAdjustFlags
        {
            BT_TRIANGLE_CONVEX_BACKFACE_NONE = 0,
            BT_TRIANGLE_CONVEX_BACKFACE_MODE = 1,
            BT_TRIANGLE_CONCAVE_DOUBLE_SIDED = 2, //double sided options are experimental, single sided is recommended
            BT_TRIANGLE_CONVEX_DOUBLE_SIDED = 4
        }

    }
}
