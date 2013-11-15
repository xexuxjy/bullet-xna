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

        public static void DebugDrawLine(IndexedVector3 from, IndexedVector3 to, IndexedVector3 color)
        {
            if (BulletGlobals.gDebugDraw != null)
            {
                BulletGlobals.gDebugDraw.DrawLine(ref from, ref to, ref color);
            }
        }

        public static void DebugDrawLine(ref IndexedVector3 from, ref IndexedVector3 to, ref IndexedVector3 color)
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

        public static float GetAngle(ref IndexedVector3 edgeA, ref IndexedVector3 normalA, ref IndexedVector3 normalB)
        {
            IndexedVector3 refAxis0 = edgeA;
            IndexedVector3 refAxis1 = normalA;
            IndexedVector3 swingAxis = normalB;
            float angle = (float)Math.Atan2(IndexedVector3.Dot(ref swingAxis, ref refAxis0), IndexedVector3.Dot(ref swingAxis, ref refAxis1));
            return angle;
        }


        public class ConnectivityProcessor : ITriangleCallback
        {
            public int m_partIdA;
            public int m_triangleIndexA;
            public IndexedVector3[] m_triangleVerticesA;
            public TriangleInfoMap m_triangleInfoMap;

            public virtual void ProcessTriangle(IndexedVector3[] triangle, int partId, int triangleIndex)
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
                float crossBSqr = IndexedVector3.Cross((triangle[1] - triangle[0]), (triangle[2] - triangle[0])).LengthSquared();
                if (crossBSqr < m_triangleInfoMap.m_equalVertexThreshold)
                {
                    return;
                }

                float crossASqr = IndexedVector3.Cross((m_triangleVerticesA[1] - m_triangleVerticesA[0]), (m_triangleVerticesA[2] - m_triangleVerticesA[0])).LengthSquared();
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


                            IndexedVector3 edge = new IndexedVector3(m_triangleVerticesA[sharedVertsA[1]] - m_triangleVerticesA[sharedVertsA[0]]);

                            TriangleShape tA = new TriangleShape(m_triangleVerticesA[0], m_triangleVerticesA[1], m_triangleVerticesA[2]);
                            int otherIndexB = 3 - (sharedVertsB[0] + sharedVertsB[1]);

                            TriangleShape tB = new TriangleShape(triangle[sharedVertsB[1]], triangle[sharedVertsB[0]], triangle[otherIndexB]);
                            //btTriangleShape tB(triangle[0],triangle[1],triangle[2]);

                            IndexedVector3 normalA;
                            IndexedVector3 normalB;
                            tA.CalcNormal(out normalA);
                            tB.CalcNormal(out normalB);
                            edge.Normalize();
                            IndexedVector3 edgeCrossA = IndexedVector3.Normalize(IndexedVector3.Cross(edge, normalA));

                            {
                                IndexedVector3 tmp = m_triangleVerticesA[otherIndexA] - m_triangleVerticesA[sharedVertsA[0]];
                                if (IndexedVector3.Dot(edgeCrossA, tmp) < 0)
                                {
                                    edgeCrossA *= -1;
                                }
                            }

                            IndexedVector3 edgeCrossB = IndexedVector3.Cross(edge, normalB).Normalized();

                            {
                                IndexedVector3 tmp = triangle[otherIndexB] - triangle[sharedVertsB[0]];
                                if (IndexedVector3.Dot(edgeCrossB, tmp) < 0)
                                {
                                    edgeCrossB *= -1;
                                }
                            }

                            float angle2 = 0;
                            float ang4 = 0.0f;

                            IndexedVector3 calculatedEdge = IndexedVector3.Cross(edgeCrossA, edgeCrossB);
                            float len2 = calculatedEdge.LengthSquared();

                            float correctedAngle = 0f;
                            IndexedVector3 calculatedNormalB = normalA;
                            bool isConvex = false;

                            if (len2 < m_triangleInfoMap.m_planarEpsilon)
                            {
                                angle2 = 0.0f;
                                ang4 = 0.0f;
                            }
                            else
                            {
                                calculatedEdge.Normalize();
                                IndexedVector3 calculatedNormalA = IndexedVector3.Cross(calculatedEdge, edgeCrossA);
                                calculatedNormalA.Normalize();
                                angle2 = GetAngle(ref calculatedNormalA, ref edgeCrossA, ref edgeCrossB);
                                ang4 = MathUtil.SIMD_PI - angle2;
                                float dotA = IndexedVector3.Dot(normalA, edgeCrossB);
                                ///@todo: check if we need some epsilon, due to floating point imprecision
                                isConvex = (dotA < 0f);

                                correctedAngle = isConvex ? ang4 : -ang4;
                                IndexedQuaternion orn2 = new IndexedQuaternion(calculatedEdge, -correctedAngle);
                                IndexedMatrix rotateMatrix = IndexedMatrix.CreateFromQuaternion(orn2);
                                calculatedNormalB = new IndexedBasisMatrix(orn2) * normalA;
                            }


                            //alternatively use 
                            //IndexedVector3 calculatedNormalB2 = quatRotate(orn,normalA);


                            switch (sumvertsA)
                            {
                                case 1:
                                    {
                                        IndexedVector3 edge1 = m_triangleVerticesA[0] - m_triangleVerticesA[1];
                                        IndexedQuaternion orn = new IndexedQuaternion(edge1, -correctedAngle);
                                        IndexedVector3 computedNormalB = MathUtil.QuatRotate(orn, normalA);
                                        float bla = IndexedVector3.Dot(computedNormalB, normalB);
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
                                        IndexedVector3 edge1 = m_triangleVerticesA[2] - m_triangleVerticesA[0];
                                        IndexedQuaternion orn = new IndexedQuaternion(edge1, -correctedAngle);
                                        IndexedVector3 computedNormalB = MathUtil.QuatRotate(orn, normalA);
                                        if (IndexedVector3.Dot(computedNormalB, normalB) < 0)
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
                                        IndexedVector3 edge1 = m_triangleVerticesA[1] - m_triangleVerticesA[2];
                                        IndexedQuaternion orn = new IndexedQuaternion(edge1, -correctedAngle);
                                        IndexedVector3 computedNormalB = MathUtil.QuatRotate(orn, normalA);
                                        if (IndexedVector3.Dot(computedNormalB, normalB) < 0)
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

            StridingMeshInterface meshInterface = trimeshShape.GetMeshInterface();
            IndexedVector3 meshScaling = meshInterface.GetScaling();

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

                IndexedVector3[] triangleVerts = new IndexedVector3[3];
                meshInterface.GetLockedReadOnlyVertexIndexBase(out vertexbase, out numverts, out type, out stride, out indexbase, out indexstride, out numfaces, out indicestype, partId);
                IndexedVector3 aabbMin, aabbMax;

                switch (indicestype)
                {
                    case PHY_ScalarType.PHY_INTEGER:
                        {
                            int[] indexList = ((ObjectArray<int>)indexbase).GetRawArray();

                            if (vertexbase is ObjectArray<IndexedVector3>)
                            {
                                IndexedVector3[] vertexList = (vertexbase as ObjectArray<IndexedVector3>).GetRawArray();
                                int indexCounter = 0;
                                for (int triangleIndex = 0; triangleIndex < numfaces; triangleIndex++)
                                {
                                    int index1 = indexList[triangleIndex];
                                    int index2 = indexList[triangleIndex + 1];
                                    int index3 = indexList[triangleIndex + 2];

                                    triangleVerts[0] = new IndexedVector3(vertexList[index1]) * meshScaling;
                                    triangleVerts[1] = new IndexedVector3(vertexList[index2]) * meshScaling;
                                    triangleVerts[2] = new IndexedVector3(vertexList[index3]) * meshScaling;
                                    ProcessResult(triangleVerts, out aabbMin, out aabbMax, trimeshShape, partId, triangleIndex, triangleInfoMap);
                                }
                            }
#if XNA
                            else if (vertexbase is ObjectArray<Microsoft.Xna.Framework.Vector3>)
                            {
                                Microsoft.Xna.Framework.Vector3[] vertexList = (vertexbase as ObjectArray<Microsoft.Xna.Framework.Vector3>).GetRawArray();
                                int indexCounter = 0;
                                for (int triangleIndex = 0; triangleIndex < numfaces; triangleIndex++)
                                {
                                    int index1 = indexList[triangleIndex];
                                    int index2 = indexList[triangleIndex + 1];
                                    int index3 = indexList[triangleIndex + 2];

                                    triangleVerts[0] = new IndexedVector3(vertexList[index1]) * meshScaling;
                                    triangleVerts[1] = new IndexedVector3(vertexList[index2]) * meshScaling;
                                    triangleVerts[2] = new IndexedVector3(vertexList[index3]) * meshScaling;
                                    ProcessResult(triangleVerts, out aabbMin, out aabbMax, trimeshShape, partId, triangleIndex, triangleInfoMap);
                                }
                            }
#endif
                            else if (vertexbase is ObjectArray<float>)
                            {
                                float[] vertexList = (vertexbase as ObjectArray<float>).GetRawArray();
                                for (int triangleIndex = 0; triangleIndex < numfaces; triangleIndex++)
                                {
                                    triangleVerts[0] = new IndexedVector3(vertexList[indexList[triangleIndex]], vertexList[indexList[triangleIndex] + 1], vertexList[indexList[triangleIndex] + 2]) * meshScaling;
                                    triangleVerts[1] = new IndexedVector3(vertexList[indexList[triangleIndex + 1]], vertexList[indexList[triangleIndex + 1] + 1], vertexList[indexList[triangleIndex + 1] + 2]) * meshScaling;
                                    triangleVerts[2] = new IndexedVector3(vertexList[indexList[triangleIndex + 2]], vertexList[indexList[triangleIndex + 2] + 1], vertexList[indexList[triangleIndex + 2] + 2]) * meshScaling;
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

        private static void ProcessResult(IndexedVector3[] triangleVerts, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax, BvhTriangleMeshShape trimeshShape, int partId, int triangleIndex, TriangleInfoMap triangleInfoMap)
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
        public static void NearestPointInLineSegment(ref IndexedVector3 point, ref IndexedVector3 line0, ref IndexedVector3 line1, out IndexedVector3 nearestPoint)
        {
            IndexedVector3 lineDelta = line1 - line0;

            // Handle degenerate lines
            if (MathUtil.FuzzyZero(lineDelta.LengthSquared()))
            {
                nearestPoint = line0;
            }
            else
            {
                float delta = IndexedVector3.Dot((point - line0), (lineDelta)) / IndexedVector3.Dot((lineDelta), (lineDelta));

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

        public static bool ClampNormal(IndexedVector3 edge, IndexedVector3 tri_normal_org, IndexedVector3 localContactNormalOnB, float correctedEdgeAngle, out IndexedVector3 clampedLocalNormal)
        {
            return ClampNormal(ref edge, ref tri_normal_org, ref localContactNormalOnB, correctedEdgeAngle, out clampedLocalNormal);
        }

        public static bool ClampNormal(ref IndexedVector3 edge, ref IndexedVector3 tri_normal_org, ref IndexedVector3 localContactNormalOnB, float correctedEdgeAngle, out IndexedVector3 clampedLocalNormal)
        {
            IndexedVector3 tri_normal = tri_normal_org;
            //we only have a local triangle normal, not a local contact normal . only normal in world space...
            //either compute the current angle all in local space, or all in world space

            IndexedVector3 edgeCross = IndexedVector3.Cross(edge, tri_normal).Normalized();
            float curAngle = GetAngle(ref edgeCross, ref tri_normal, ref localContactNormalOnB);

            if (correctedEdgeAngle < 0)
            {
                if (curAngle < correctedEdgeAngle)
                {
                    float diffAngle = correctedEdgeAngle - curAngle;
                    IndexedQuaternion rotation = new IndexedQuaternion(edge, diffAngle);
                    clampedLocalNormal = new IndexedBasisMatrix(rotation) * localContactNormalOnB;
                    return true;
                }
            }

            if (correctedEdgeAngle >= 0)
            {
                if (curAngle > correctedEdgeAngle)
                {
                    float diffAngle = correctedEdgeAngle - curAngle;
                    IndexedQuaternion rotation = new IndexedQuaternion(edge, diffAngle);
                    clampedLocalNormal = new IndexedBasisMatrix(rotation) * localContactNormalOnB;
                    return true;
                }
            }
            clampedLocalNormal = IndexedVector3.Zero;
            return false;
        }



        /// Changes a btManifoldPoint collision normal to the normal from the mesh.
        public static void AdjustInternalEdgeContacts(ManifoldPoint cp, CollisionObject colObj0, CollisionObject colObj1, int partId0, int index0, InternalEdgeAdjustFlags normalAdjustFlags)
        {
            //btAssert(colObj0.GetCollisionShape().GetShapeType() == TRIANGLE_SHAPE_PROXYTYPE);
            if (colObj0.GetCollisionShape().GetShapeType() != BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE)
                return;

            BvhTriangleMeshShape trimesh = null;

            if (colObj0.GetRootCollisionShape().GetShapeType() == BroadphaseNativeTypes.SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE)
            {
                //trimesh = ((ScaledBvhTriangleMeshShape)colObj0.GetRootCollisionShape()).GetChildShape();
            }
            else
            {
                trimesh = (BvhTriangleMeshShape)colObj0.GetRootCollisionShape();
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

            TriangleShape tri_shape = colObj0.GetCollisionShape() as TriangleShape;
            IndexedVector3 v0, v1, v2;
            tri_shape.GetVertex(0, out v0);
            tri_shape.GetVertex(1, out v1);
            tri_shape.GetVertex(2, out v2);

            IndexedVector3 center = (v0 + v1 + v2) * (1.0f / 3.0f);

            IndexedVector3 red = new IndexedVector3(1, 0, 0), green = new IndexedVector3(0, 1, 0), blue = new IndexedVector3(0, 0, 1), white = new IndexedVector3(1, 1, 1), black = new IndexedVector3(0, 0, 0);
            IndexedVector3 tri_normal;
            tri_shape.CalcNormal(out tri_normal);

            //float dot = tri_normal.dot(cp.m_normalWorldOnB);
            IndexedVector3 nearest;
            NearestPointInLineSegment(ref cp.m_localPointB, ref v0, ref v1, out nearest);

            IndexedVector3 contact = cp.m_localPointB;
#if BT_INTERNAL_EDGE_DEBUG_DRAW
            IndexedMatrix tr = colObj0.GetWorldTransform();
            DebugDrawLine(tr * nearest, tr * cp.m_localPointB, red);
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW



            bool isNearEdge = false;

            int numConcaveEdgeHits = 0;
            int numConvexEdgeHits = 0;

            IndexedVector3 localContactNormalOnB = colObj0.GetWorldTransform()._basis.Transpose() * cp.m_normalWorldOnB;
            localContactNormalOnB.Normalize();//is this necessary?

            // Get closest edge
            int bestedge = -1;
            float disttobestedge = MathUtil.BT_LARGE_FLOAT;
            //
            // Edge 0 . 1
            if (Math.Abs(info.m_edgeV0V1Angle) < triangleInfoMapPtr.m_maxEdgeAngleThreshold)
            {
                //IndexedVector3 nearest;
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
                //IndexedVector3 nearest;
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
                //IndexedVector3 nearest;
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
            IndexedVector3 upfix = tri_normal * new IndexedVector3(0.1f, 0.1f, 0.1f);
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
                        IndexedVector3 edge = (v0 - v1);
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

                            IndexedVector3 nA = swapFactor * tri_normal;

                            IndexedQuaternion orn = new IndexedQuaternion(edge, info.m_edgeV0V1Angle);
                            IndexedVector3 computedNormalB = MathUtil.QuatRotate(ref orn, ref tri_normal);
                            if ((info.m_flags & TriangleInfoMap.TRI_INFO_V0V1_SWAP_NORMALB) != 0)
                            {
                                computedNormalB *= -1;
                            }
                            IndexedVector3 nB = swapFactor * computedNormalB;

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
                                IndexedVector3 clampedLocalNormal;
                                bool isClamped = ClampNormal(edge, swapFactor * tri_normal, localContactNormalOnB, info.m_edgeV0V1Angle, out clampedLocalNormal);
                                if (isClamped)
                                {
                                    if (((normalAdjustFlags & InternalEdgeAdjustFlags.BT_TRIANGLE_CONVEX_DOUBLE_SIDED) != 0) || (clampedLocalNormal.Dot(frontFacing * tri_normal) > 0))
                                    {
                                        IndexedVector3 newNormal = colObj0.GetWorldTransform()._basis * clampedLocalNormal;
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

                        IndexedVector3 edge = (v1 - v2);

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

                            IndexedVector3 nA = swapFactor * tri_normal;

                            IndexedQuaternion orn = new IndexedQuaternion(edge, info.m_edgeV1V2Angle);
                            IndexedVector3 computedNormalB = MathUtil.QuatRotate(ref orn, ref tri_normal);
                            if ((info.m_flags & TriangleInfoMap.TRI_INFO_V1V2_SWAP_NORMALB) != 0)
                            {
                                computedNormalB *= -1;
                            }
                            IndexedVector3 nB = swapFactor * computedNormalB;

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
                                IndexedVector3 localContactNormalOnB2 = colObj0.GetWorldTransform()._basis.Transpose() * cp.m_normalWorldOnB;
                                IndexedVector3 clampedLocalNormal;
                                bool isClamped = ClampNormal(edge, swapFactor * tri_normal, localContactNormalOnB2, info.m_edgeV1V2Angle, out clampedLocalNormal);
                                if (isClamped)
                                {
                                    if (((normalAdjustFlags & InternalEdgeAdjustFlags.BT_TRIANGLE_CONVEX_DOUBLE_SIDED) != 0) || (clampedLocalNormal.Dot(frontFacing * tri_normal) > 0))
                                    {
                                        IndexedVector3 newNormal = colObj0.GetWorldTransform()._basis * clampedLocalNormal;
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

                        IndexedVector3 edge = (v2 - v0);

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

                            IndexedVector3 nA = swapFactor * tri_normal;
                            IndexedQuaternion orn = new IndexedQuaternion(edge, info.m_edgeV2V0Angle);
                            IndexedVector3 computedNormalB = MathUtil.QuatRotate(ref orn, ref tri_normal);
                            if ((info.m_flags & TriangleInfoMap.TRI_INFO_V2V0_SWAP_NORMALB) != 0)
                            {
                                computedNormalB *= -1;
                            }
                            IndexedVector3 nB = swapFactor * computedNormalB;

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


                                IndexedVector3 localContactNormalOnB2 = colObj0.GetWorldTransform()._basis.Transpose() * cp.m_normalWorldOnB;
                                IndexedVector3 clampedLocalNormal;
                                bool isClamped = ClampNormal(edge, swapFactor * tri_normal, localContactNormalOnB2, info.m_edgeV2V0Angle, out clampedLocalNormal);
                                if (isClamped)
                                {
                                    if (((normalAdjustFlags & InternalEdgeAdjustFlags.BT_TRIANGLE_CONVEX_DOUBLE_SIDED) != 0) || (clampedLocalNormal.Dot(frontFacing * tri_normal) > 0))
                                    {
                                        IndexedVector3 newNormal = colObj0.GetWorldTransform()._basis * clampedLocalNormal;
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
                IndexedVector3 color = new IndexedVector3(0, 1, 1);
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
                        IndexedVector3 newNormal = tri_normal * frontFacing;
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
