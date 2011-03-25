//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using Microsoft.Xna.Framework;

//namespace BulletXNA.BulletCollision.CollisionDispatch
//{
//    public class InternalEdgeUtility
//    {
//        ///Call btGenerateInternalEdgeInfo to create triangle info, store in the shape 'userInfo'
//        void	btGenerateInternalEdgeInfo (btBvhTriangleMeshShape*trimeshShape, btTriangleInfoMap* triangleInfoMap);

//        ///Call the btFixMeshNormal to adjust the collision normal, using the triangle info map (generated using btGenerateInternalEdgeInfo)
//        ///If this info map is missing, or the triangle is not store in this map, nothing will be done
//        void	btAdjustInternalEdgeContacts(ManifoldPoint cp, CollisionObject trimeshColObj0,CollisionObject otherColObj1, int partId0, int index0, int normalAdjustFlags = 0);

//        ///Enable the BT_INTERNAL_EDGE_DEBUG_DRAW define and call btSetDebugDrawer, to get visual info to see if the internal edge utility works properly.
//        ///If the utility doesn't work properly, you might have to adjust the threshold values in btTriangleInfoMap
//        //#define BT_INTERNAL_EDGE_DEBUG_DRAW

//#if BT_INTERNAL_EDGE_DEBUG_DRAW
//    void	btSetDebugDrawer(btIDebugDraw* debugDrawer);
//#endif //BT_INTERNAL_EDGE_DEBUG_DRAW
////#define DEBUG_INTERNAL_EDGE



//#if BT_INTERNAL_EDGE_DEBUG_DRAW
//    public static IDebugDraw gDebugDrawer = null;

//    public static void setDebugDrawer(IDebugDraw debugDrawer)
//    {
//        gDebugDrawer = debugDrawer;
//    }

//    public static void debugDrawLine(ref Vector3 from,ref Vector3 to, ref Vector3 color)
//    {
//        if (gDebugDrawer != null)
//        {
//            gDebugDrawer.drawLine(ref from,ref to,ref color);
//        }
//    }
//#endif //BT_INTERNAL_EDGE_DEBUG_DRAW


//        public static int getHash(int partId, int triangleIndex)
//        {
//            int hash = (partId<<(31-MAX_NUM_PARTS_IN_BITS)) | triangleIndex;
//            return hash;
//        }

//        public static float getAngle(ref Vector3 edgeA, ref Vector3 normalA,ref Vector3 normalB)
//        {
//            Vector3 refAxis0  = edgeA;
//            Vector3 refAxis1  = normalA;
//            Vector3 swingAxis = normalB;
//            float angle = Math.Atan2(Vector3.Dot(swingAxis,refAxis0), Vector3.Dot(swingAxis,refAxis1));
//            return  angle;
//        }


//        public class ConnectivityProcessor : ITriangleCallback
//        {
//            public int m_partIdA;
//            public int	m_triangleIndexA;
//            public IList<Vector3>   m_triangleVerticesA;
//            public TriangleInfoMap	m_triangleInfoMap;

//            public virtual void processTriangle(IList<Vector3> triangle, int partId, int triangleIndex)
//            {
//                //skip self-collisions
//                if ((m_partIdA == partId) && (m_triangleIndexA == triangleIndex))
//                {
//                    return;
//                }

//                //skip duplicates (disabled for now)
//                //if ((m_partIdA <= partId) && (m_triangleIndexA <= triangleIndex))
//                //	return;

//                //search for shared vertices and edges
//                int numshared = 0;
//                int[] sharedVertsA = new int[]{-1,-1,-1};
//                int[] sharedVertsB = new int[]{-1,-1,-1};

//                ///skip degenerate triangles
//                float crossBSqr = (Vector3.Cross((triangle[1]-triangle[0]),(triangle[2]-triangle[0])).LengthSquared();
//                if (crossBSqr < m_triangleInfoMap.m_equalVertexThreshold)
//                {
//                    return;
//                }

//                float crossASqr = Vector3.Cross((m_triangleVerticesA[1]-m_triangleVerticesA[0]),(m_triangleVerticesA[2]-m_triangleVerticesA[0])).LengthSquared();
//                ///skip degenerate triangles
//                if (crossASqr< m_triangleInfoMap.m_equalVertexThreshold)
//                {
//                    return;
//                }

//        #if false
//                printf("triangle A[0]	=	(%f,%f,%f)\ntriangle A[1]	=	(%f,%f,%f)\ntriangle A[2]	=	(%f,%f,%f)\n",
//                    m_triangleVerticesA[0].getX(),m_triangleVerticesA[0].getY(),m_triangleVerticesA[0].getZ(),
//                    m_triangleVerticesA[1].getX(),m_triangleVerticesA[1].getY(),m_triangleVerticesA[1].getZ(),
//                    m_triangleVerticesA[2].getX(),m_triangleVerticesA[2].getY(),m_triangleVerticesA[2].getZ());

//                printf("partId=%d, triangleIndex=%d\n",partId,triangleIndex);
//                printf("triangle B[0]	=	(%f,%f,%f)\ntriangle B[1]	=	(%f,%f,%f)\ntriangle B[2]	=	(%f,%f,%f)\n",
//                    triangle[0].getX(),triangle[0].getY(),triangle[0].getZ(),
//                    triangle[1].getX(),triangle[1].getY(),triangle[1].getZ(),
//                    triangle[2].getX(),triangle[2].getY(),triangle[2].getZ());
//        #endif

//                for (int i=0;i<3;i++)
//                {
//                    for (int j=0;j<3;j++)
//                    {
//                        if ( (m_triangleVerticesA[i]-triangle[j]).LengthSquared() < m_triangleInfoMap.m_equalVertexThreshold)
//                        {
//                            sharedVertsA[numshared] = i;
//                            sharedVertsB[numshared] = j;
//                            numshared++;
//                            ///degenerate case
//                            if(numshared >= 3)
//                            {
//                                return;
//                            }
//                        }
//                    }
//                    ///degenerate case
//                    if(numshared >= 3)
//                    {
//                        return;
//                    }
//                }
//                switch (numshared)
//                {
//                case 0:
//                    {
//                        break;
//                    }
//                case 1:
//                    {
//                        //shared vertex
//                        break;
//                    }
//                case 2:
//                    {
//                        //shared edge
//                        //we need to make sure the edge is in the order V2V0 and not V0V2 so that the signs are correct
//                        if (sharedVertsA[0] == 0 && sharedVertsA[1] == 2)
//                        {
//                            sharedVertsA[0] = 2;
//                            sharedVertsA[1] = 0;
//                            int tmp = sharedVertsB[1];
//                            sharedVertsB[1] = sharedVertsB[0];
//                            sharedVertsB[0] = tmp;
//                        }

//                        int hash = getHash(m_partIdA,m_triangleIndexA);

//                        TriangleInfo info = m_triangleInfoMap.find(hash);
//                        if (info == null)
//                        {
//                            TriangleInfo tmp = new TriangleInfo();
//                            m_triangleInfoMap.insert(hash,tmp);
//                            info = m_triangleInfoMap.find(hash);
//                        }

//                        int sumvertsA = sharedVertsA[0]+sharedVertsA[1];
//                        int otherIndexA = 3-sumvertsA;

        				
//                        Vector3 edge = new Vector3(m_triangleVerticesA[sharedVertsA[1]]-m_triangleVerticesA[sharedVertsA[0]]);

//                        TriangleShape tA = new TriangleShape(m_triangleVerticesA[0],m_triangleVerticesA[1],m_triangleVerticesA[2]);
//                        int otherIndexB = 3-(sharedVertsB[0]+sharedVertsB[1]);

//                        TriangleShape tB = new TriangleShape(triangle[sharedVertsB[1]],triangle[sharedVertsB[0]],triangle[otherIndexB]);
//                        //btTriangleShape tB(triangle[0],triangle[1],triangle[2]);

//                        Vector3 normalA;
//                        Vector3 normalB;
//                        tA.calcNormal(normalA);
//                        tB.calcNormal(normalB);
//                        edge.Normalize();
//                        Vector3 edgeCrossA = Vector3.Normalize(Vector3.Cross(edge,normalA));

//                        {
//                            Vector3 tmp = m_triangleVerticesA[otherIndexA]-m_triangleVerticesA[sharedVertsA[0]];
//                            if (Vector3.Dot(edgeCrossA,tmp) < 0)
//                            {
//                                edgeCrossA*=-1;
//                            }
//                        }

//                        Vector3 edgeCrossB = Vector3.Normalise(Vector3.Cross(edge,normalB));

//                        {
//                            Vector3 tmp = triangle[otherIndexB]-triangle[sharedVertsB[0]];
//                            if (Vector3.Dot(edgeCrossB,tmp) < 0)
//                            {
//                                edgeCrossB*=-1;
//                            }
//                        }

//                        float	angle2 = 0;
//                        float	ang4 = 0.f;

//                        Vector3 calculatedEdge = Vector3.Cross(edgeCrossA,edgeCrossB);
//                        float len2 = calculatedEdge.LengthSquared();

//                        float correctedAngle = 0f;
//                        Vector3 calculatedNormalB = normalA;
//                        bool isConvex = false;

//                        if (len2<m_triangleInfoMap.m_planarEpsilon)
//                        {
//                            angle2 = 0.f;
//                            ang4 = 0.f;
//                        } 
//                        else
//                        {
//                            calculatedEdge.Normalize();
//                            Vector3 calculatedNormalA = Vector3.Cross(calculatedEdge,edgeCrossA);
//                            calculatedNormalA.Normalize();
//                            angle2 = getAngle(ref calculatedNormalA,ref edgeCrossA,ref edgeCrossB);
//                            ang4 = MathUtil.SIMD_PI-angle2;
//                            float dotA = Vector3.Dot(normalA,edgeCrossB);
//                            ///@todo: check if we need some epsilon, due to floating point imprecision
//                            isConvex = (dotA<0f);

//                            correctedAngle = isConvex ? ang4 : -ang4;
//                            Quaternion orn2 = Quaternion.CreateFromAxisAngle(calculatedEdge,-correctedAngle);
//                            Matrix rotateMatrix = Matrix.CreateFromQuaternion(orn2);
//                            calculatedNormalB = Vector3.TransformNormal(normalA,rotateMatrix);
//                        }

        							
//                        //alternatively use 
//                        //Vector3 calculatedNormalB2 = quatRotate(orn,normalA);


//                        switch (sumvertsA)
//                        {
//                        case 1:
//                            {
//                                Vector3 edge = m_triangleVerticesA[0]-m_triangleVerticesA[1];
//                                Quaternion orn = Quaternion.CreateFromAxisAngle(edge,-correctedAngle);
//                                Vector3 computedNormalB = MathUtil.quatRotate(orn,normalA);
//                                float bla = Vector3.Dot(computedNormalB,normalB);
//                                if (bla<0)
//                                {
//                                    computedNormalB*=-1;
//                                    info.m_flags |= TRI_INFO_V0V1_SWAP_NORMALB;
//                                }
//        #if DEBUG_INTERNAL_EDGE
//                                if ((computedNormalB-normalB).length()>0.0001)
//                                {
//                                    printf("warning: normals not identical\n");
//                                }
//        #endif//DEBUG_INTERNAL_EDGE

//                                info.m_edgeV0V1Angle = -correctedAngle;

//                                if (isConvex)
//                                {
//                                    info.m_flags |= TRI_INFO_V0V1_CONVEX;
//                                }
//                                break;
//                            }
//                        case 2:
//                            {
//                                Vector3 edge = m_triangleVerticesA[2]-m_triangleVerticesA[0];
//                                Quaternion orn = Quaternion.CreateFromAxisAngle(edge,-correctedAngle);
//                                Vector3 computedNormalB = MathUtil.quatRotate(orn,normalA);
//                                if (Vector3.Dot(computedNormalB,normalB)<0)
//                                {
//                                    computedNormalB*=-1;
//                                    info.m_flags |= TRI_INFO_V2V0_SWAP_NORMALB;
//                                }

//        #if DEBUG_INTERNAL_EDGE
//                                if ((computedNormalB-normalB).length()>0.0001)
//                                {
//                                    printf("warning: normals not identical\n");
//                                }
//        #endif //DEBUG_INTERNAL_EDGE
//                                info.m_edgeV2V0Angle = -correctedAngle;
//                                if (isConvex)
//                                    info.m_flags |= TRI_INFO_V2V0_CONVEX;
//                                break;	
//                            }
//                        case 3:
//                            {
//                                Vector3 edge = m_triangleVerticesA[1]-m_triangleVerticesA[2];
//                                Quaternion orn = Quaternion.CreateFromAxisAngle(edge,-correctedAngle);
//                                Vector3 computedNormalB = MathUtil.quatRotate(orn,normalA);
//                                if (Vector3.Dot(computedNormalB,normalB)<0)
//                                {
//                                    info.m_flags |= TRI_INFO_V1V2_SWAP_NORMALB;
//                                    computedNormalB*=-1;
//                                }
//        #if DEBUG_INTERNAL_EDGE
//                                if ((computedNormalB-normalB).length()>0.0001)
//                                {
//                                    printf("warning: normals not identical\n");
//                                }
//        #endif //DEBUG_INTERNAL_EDGE
//                                info.m_edgeV1V2Angle = -correctedAngle;

//                                if (isConvex)
//                                {
//                                    info.m_flags |= TRI_INFO_V1V2_CONVEX;
//                                }
//                                break;
//                            }
//                        }

//                        break;
//                    }
//                default:
//                    {
//                        //				printf("warning: duplicate triangle\n");
//                        break;
//                    }

//                }
//            }
//        };
//        /////////////////////////////////////////////////////////
//        /////////////////////////////////////////////////////////

//        public void generateInternalEdgeInfo (BvhTriangleMeshShape trimeshShape, TriangleInfoMap triangleInfoMap)
//        {
//            //the user pointer shouldn't already be used for other purposes, we intend to store connectivity info there!
//            if (trimeshShape.getTriangleInfoMap())
//            {
//                return;
//            }

//            trimeshShape.setTriangleInfoMap(triangleInfoMap);

//            StridingMeshInterface meshInterface = trimeshShape.getMeshInterface();
//            Vector3 meshScaling = meshInterface.getScaling();

//            for (int partId = 0; partId< meshInterface.getNumSubParts();partId++)
//            {
//                const unsigned char *vertexbase = 0;
//                int numverts = 0;
//                PHY_ScalarType type = PHY_INTEGER;
//                int stride = 0;
//                const unsigned char *indexbase = 0;
//                int indexstride = 0;
//                int numfaces = 0;
//                PHY_ScalarType indicestype = PHY_INTEGER;
//                //PHY_ScalarType indexType=0;

//                Vector3[] triangleVerts = new Vector3[3];
//                meshInterface.getLockedReadOnlyVertexIndexBase(vertexbase,numverts, type,stride,indexbase,indexstride,numfaces,indicestype,partId);
//                Vector3 aabbMin,aabbMax;

//                for (int triangleIndex = 0 ; triangleIndex < numfaces;triangleIndex++)
//                {
//                    unsigned int* gfxbase = (unsigned int*)(indexbase+triangleIndex*indexstride);

//                    for (int j=2;j>=0;j--)
//                    {

//                        int graphicsindex = indicestype==PHY_SHORT?((unsigned short*)gfxbase)[j]:gfxbase[j];
//                        if (type == PHY_FLOAT)
//                        {
//                            float* graphicsbase = (float*)(vertexbase+graphicsindex*stride);
//                            triangleVerts[j] = Vector3(
//                                graphicsbase[0]*meshScaling.getX(),
//                                graphicsbase[1]*meshScaling.getY(),
//                                graphicsbase[2]*meshScaling.getZ());
//                        }
//                        else
//                        {
//                            double* graphicsbase = (double*)(vertexbase+graphicsindex*stride);
//                            triangleVerts[j] = Vector3( float(graphicsbase[0]*meshScaling.getX()), float(graphicsbase[1]*meshScaling.getY()), float(graphicsbase[2]*meshScaling.getZ()));
//                        }
//                    }
//                    aabbMin.setValue(float(BT_LARGE_FLOAT),float(BT_LARGE_FLOAT),float(BT_LARGE_FLOAT));
//                    aabbMax.setValue(float(-BT_LARGE_FLOAT),float(-BT_LARGE_FLOAT),float(-BT_LARGE_FLOAT)); 
//                    aabbMin.setMin(triangleVerts[0]);
//                    aabbMax.setMax(triangleVerts[0]);
//                    aabbMin.setMin(triangleVerts[1]);
//                    aabbMax.setMax(triangleVerts[1]);
//                    aabbMin.setMin(triangleVerts[2]);
//                    aabbMax.setMax(triangleVerts[2]);

//                    btConnectivityProcessor connectivityProcessor;
//                    connectivityProcessor.m_partIdA = partId;
//                    connectivityProcessor.m_triangleIndexA = triangleIndex;
//                    connectivityProcessor.m_triangleVerticesA = &triangleVerts[0];
//                    connectivityProcessor.m_triangleInfoMap  = triangleInfoMap;

//                    trimeshShape.processAllTriangles(&connectivityProcessor,aabbMin,aabbMax);
//                }

//            }

//        }


//        // Given a point and a line segment (defined by two points), compute the closest point
//        // in the line.  Cap the point at the endpoints of the line segment.
//        public void NearestPointInLineSegment(ref Vector3 point, ref Vector3 line0, ref Vector3 line1, ref Vector3 nearestPoint)
//        {
//            Vector3 lineDelta = line1 - line0;

//            // Handle degenerate lines
//            if ( MathUtil.fuzzyZero(lineDelta.LengthSquared())
//            {
//                nearestPoint = line0;
//            }
//            else
//            {
//                float delta = Vector3.Dot((point-line0),(lineDelta)) / Vector3.Dot((lineDelta),(lineDelta));

//                // Clamp the point to conform to the segment's endpoints
//                if ( delta < 0 )
//                {
//                    delta = 0;
//                }
//                else if ( delta > 1 )
//                {
//                    delta = 1;
//                }

//                nearestPoint = line0 + lineDelta*delta;
//            }
//        }


//        public bool	ClampNormal(ref Vector3 edge,ref Vector3 tri_normal_org,ref Vector3 localContactNormalOnB, float correctedEdgeAngle, ref Vector3 clampedLocalNormal)
//        {
//            Vector3 tri_normal = tri_normal_org;
//            //we only have a local triangle normal, not a local contact normal . only normal in world space...
//            //either compute the current angle all in local space, or all in world space

//            Vector3 edgeCross = Vector3.Cross(edge,tri_normal).Normalize();
//            float curAngle = getAngle(ref edgeCross,ref tri_normal,ref localContactNormalOnB);

//            if (correctedEdgeAngle<0)
//            {
//                if (curAngle < correctedEdgeAngle)
//                {
//                    float diffAngle = correctedEdgeAngle-curAngle;
//                    Quaternion rotation = Quaternion.CreateFromAxisAngle(edge,diffAngle);
//                    Matrix rotMtx = Matrix.CreateFromQuaternion(rotation);
//                    clampedLocalNormal = Vector3.TransformNormal(localContactNormalOnB,rotMtx);
//                    return true;
//                }
//            }

//            if (correctedEdgeAngle>=0)
//            {
//                if (curAngle > correctedEdgeAngle)
//                {
//                    float diffAngle = correctedEdgeAngle-curAngle;
//                    Quaternion rotation = Quaternion.CreateFromAxisAngle(edge,diffAngle);
//                    Matrix rotMtx = Matrix.CreateFromQuaternion(rotation);

//                    clampedLocalNormal = Vector3.TransformNormal(localContactNormalOnB,rotation);
//                    return true;
//                }
//            }
//            return false;
//        }



//        /// Changes a btManifoldPoint collision normal to the normal from the mesh.
//        public void AdjustInternalEdgeContacts(ManifoldPoint cp, CollisionObject colObj0,CollisionObject colObj1, int partId0, int index0, int normalAdjustFlags)
//        {
//            //btAssert(colObj0.getCollisionShape().getShapeType() == TRIANGLE_SHAPE_PROXYTYPE);
//            if (colObj0.getCollisionShape().getShapeType() != TRIANGLE_SHAPE_PROXYTYPE)
//            {
//                return;
//            }

//            BvhTriangleMeshShape* trimesh = (btBvhTriangleMeshShape*)colObj0.getRootCollisionShape();
//            TriangleInfoMap* triangleInfoMapPtr = (btTriangleInfoMap*) trimesh.getTriangleInfoMap();
//            if (triangleInfoMapPtr == null)
//            {
//                return;
//            }

//            int hash = btGetHash(partId0,index0);


//            TriangleInfo info = triangleInfoMapPtr.find(hash);
//            if (info == null)
//            {
//                return;
//            }

//            float frontFacing = (normalAdjustFlags & BT_TRIANGLE_CONVEX_BACKFACE_MODE)==0? 1.f : -1.f;
        	
//            TriangleShape tri_shape = (TriangleShape)(colObj0.getCollisionShape());
//            Vector3 v0,v1,v2;
//            tri_shape.getVertex(0,ref v0);
//            tri_shape.getVertex(1,ref v1);
//            tri_shape.getVertex(2,ref v2);

//            Vector3 center = (v0+v1+v2)*(1f/3f);

//            Vector3 red = new Vector3(1,0,0), green = new Vector3(0,1,0),blue = new Vector3(0,0,1),white = new Vector3(1,1,1),black = new Vector3(0,0,0);
//            Vector3 tri_normal;
//            tri_shape.calcNormal(ref tri_normal);

//            //float dot = tri_normal.dot(cp.m_normalWorldOnB);
//            Vector3 nearest;
//            nearestPointInLineSegment(ref cp.m_localPointB,ref v0,ref v1,ref nearest);

//            Vector3 contact = cp.m_localPointB;
//        #if BT_INTERNAL_EDGE_DEBUG_DRAW
//            const btTransform& tr = colObj0.getWorldTransform();
//            btDebugDrawLine(tr*nearest,tr*cp.m_localPointB,red);
//        #endif //BT_INTERNAL_EDGE_DEBUG_DRAW

//            bool isNearEdge = false;

//            int numConcaveEdgeHits = 0;
//            int numConvexEdgeHits = 0;

//            Vector3 localContactNormalOnB = Vector3.TransformNormal(cp.m_normalWorldOnB,MathUtil.transposeBasis(colObj0.getWorldTransform());
//            localContactNormalOnB.Normalize();//is this necessary?

//            if ((info.m_edgeV0V1Angle)< MathUtil.SIMD_2_PI)
//            {
//        #if BT_INTERNAL_EDGE_DEBUG_DRAW
//                btDebugDrawLine(tr*contact,tr*(contact+cp.m_normalWorldOnB*10),black);
//        #endif
//                float len = (contact-nearest).Length();
//                if(len<triangleInfoMapPtr.m_edgeDistanceThreshold)
//                {
//                    Vector3 edge = new Vector3(v0-v1);
//                    isNearEdge = true;

//                    if (info.m_edgeV0V1Angle==0f)
//                    {
//                        numConcaveEdgeHits++;
//                    } 
//                    else
//                    {
//                        bool isEdgeConvex = (info.m_flags & TRI_INFO_V0V1_CONVEX);
//                        float swapFactor = isEdgeConvex ? float(1) : float(-1);
//            #if BT_INTERNAL_EDGE_DEBUG_DRAW
//                        btDebugDrawLine(tr*nearest,tr*(nearest+swapFactor*tri_normal*10),white);
//            #endif //BT_INTERNAL_EDGE_DEBUG_DRAW

//                        Vector3 nA = swapFactor * tri_normal;

//                        Quaternion orn = Quaternion.CreateFromAxisAngle(edge,info.m_edgeV0V1Angle);
//                        Vector3 computedNormalB = MathUtil.quatRotate(orn,tri_normal);
//                        if (info.m_flags & TRI_INFO_V0V1_SWAP_NORMALB)
//                        {
//                            computedNormalB*=-1;
//                        }
//                        Vector3 nB = swapFactor*computedNormalB;

//                        float	NdotA = localContactNormalOnB.dot(nA);
//                        float	NdotB = localContactNormalOnB.dot(nB);
//                        bool backFacingNormal = (NdotA< triangleInfoMapPtr.m_convexEpsilon) && (NdotB<triangleInfoMapPtr.m_convexEpsilon);

//        #if DEBUG_INTERNAL_EDGE
//                        {
        					
//                            btDebugDrawLine(cp.getPositionWorldOnB(),cp.getPositionWorldOnB()+tr.getBasis()*(nB*20),red);
//                        }
//        #endif //DEBUG_INTERNAL_EDGE


//                        if (backFacingNormal)
//                        {
//                            numConcaveEdgeHits++;
//                        }
//                        else
//                        {
//                            numConvexEdgeHits++;
//                            Vector3 clampedLocalNormal;
//                            bool isClamped = btClampNormal(edge,swapFactor*tri_normal,localContactNormalOnB, info.m_edgeV0V1Angle,clampedLocalNormal);
//                            if (isClamped)
//                            {
//                                if (((normalAdjustFlags & BT_TRIANGLE_CONVEX_DOUBLE_SIDED)!=0) || (Vector3.Dot(clampedLocalNormal.dot(frontFacing*tri_normal)>0)))
//                                {
//                                    Vector3 newNormal = Vector3.TransformNormal(clampedLocalNormal,colObj0.getWorldTransform());
//                                    //					cp.m_distance1 = cp.m_distance1 * newNormal.dot(cp.m_normalWorldOnB);
//                                    cp.m_normalWorldOnB = newNormal;
//                                    // Reproject collision point along normal. (what about cp.m_distance1?)
//                                    cp.m_positionWorldOnB = cp.m_positionWorldOnA - cp.m_normalWorldOnB * cp.m_distance1;
//                                    cp.m_localPointB = colObj0.getWorldTransform().invXform(cp.m_positionWorldOnB);
        							
//                                }
//                            }
//                        }
//                    }
//                }
//            }

//            btNearestPointInLineSegment(contact,v1,v2,nearest);
//        #if BT_INTERNAL_EDGE_DEBUG_DRAW
//            btDebugDrawLine(tr*nearest,tr*cp.m_localPointB,green);
//        #endif //BT_INTERNAL_EDGE_DEBUG_DRAW

//            if ((info.m_edgeV1V2Angle)< MathUtil.SIMD_2_PI)
//            {
//        #if BT_INTERNAL_EDGE_DEBUG_DRAW
//                btDebugDrawLine(tr*contact,tr*(contact+cp.m_normalWorldOnB*10),black);
//        #endif //BT_INTERNAL_EDGE_DEBUG_DRAW

//                float len = (contact-nearest).length();
//                if(len<triangleInfoMapPtr.m_edgeDistanceThreshold)
//                {
//                    isNearEdge = true;
//        #if BT_INTERNAL_EDGE_DEBUG_DRAW
//                    btDebugDrawLine(tr*nearest,tr*(nearest+tri_normal*10),white);
//        #endif //BT_INTERNAL_EDGE_DEBUG_DRAW

//                    Vector3 edge = (v1-v2);

//                    isNearEdge = true;

//                    if (info.m_edgeV1V2Angle == 0f)
//                    {
//                        numConcaveEdgeHits++;
//                    } else
//                    {
//                        bool isEdgeConvex = (info.m_flags & TRI_INFO_V1V2_CONVEX)!=0;
//                        float swapFactor = isEdgeConvex ? 1f : -1f;
//            #if BT_INTERNAL_EDGE_DEBUG_DRAW
//                        btDebugDrawLine(tr*nearest,tr*(nearest+swapFactor*tri_normal*10),white);
//            #endif //BT_INTERNAL_EDGE_DEBUG_DRAW

//                        Vector3 nA = swapFactor * tri_normal;
        				
//                        Quaternion orn = Quaternion.CreateFromAxisAngle(edge,info.m_edgeV1V2Angle);
//                        Vector3 computedNormalB = quatRotate(orn,tri_normal);
//                        if (info.m_flags & TRI_INFO_V1V2_SWAP_NORMALB)
//                        {
//                            computedNormalB*=-1;
//                        }
//                        Vector3 nB = swapFactor*computedNormalB;

//        #if DEBUG_INTERNAL_EDGE
//                        {
//                            btDebugDrawLine(cp.getPositionWorldOnB(),cp.getPositionWorldOnB()+tr.getBasis()*(nB*20),red);
//                        }
//        #endif //DEBUG_INTERNAL_EDGE


//                        float	NdotA = localContactNormalOnB.dot(nA);
//                        float	NdotB = localContactNormalOnB.dot(nB);
//                        bool backFacingNormal = (NdotA< triangleInfoMapPtr.m_convexEpsilon) && (NdotB<triangleInfoMapPtr.m_convexEpsilon);

//                        if (backFacingNormal)
//                        {
//                            numConcaveEdgeHits++;
//                        }
//                        else
//                        {
//                            numConvexEdgeHits++;
//                            Vector3 localContactNormalOnB = Vector3.TransformNormal(cp.m_normalOnWOrldOnB,MathUtil.transposeBasis(colObj0.getWorldTransform());
//                            Vector3 clampedLocalNormal;
//                            bool isClamped = btClampNormal(edge,swapFactor*tri_normal,localContactNormalOnB, info.m_edgeV1V2Angle,clampedLocalNormal);
//                            if (isClamped)
//                            {
//                                if (((normalAdjustFlags & BT_TRIANGLE_CONVEX_DOUBLE_SIDED)!=0) || (clampedLocalNormal.dot(frontFacing*tri_normal)>0))
//                                {
//                                    Vector3 newNormal = Vector3.TransformNormal(clampedLocalNormal,colObj0.getWorldTransform());
//                                    //					cp.m_distance1 = cp.m_distance1 * newNormal.dot(cp.m_normalWorldOnB);
//                                    cp.m_normalWorldOnB = newNormal;
//                                    // Reproject collision point along normal.
//                                    cp.m_positionWorldOnB = cp.m_positionWorldOnA - cp.m_normalWorldOnB * cp.m_distance1;
//                                    cp.m_localPointB = colObj0.getWorldTransform().invXform(cp.m_positionWorldOnB);
//                                }
//                            }
//                        }
//                    }
//                }
//            }

//            btNearestPointInLineSegment(contact,v2,v0,nearest);
//        #if BT_INTERNAL_EDGE_DEBUG_DRAW
//            btDebugDrawLine(tr*nearest,tr*cp.m_localPointB,blue);
//        #endif //BT_INTERNAL_EDGE_DEBUG_DRAW

//            if ((info.m_edgeV2V0Angle)< MathUtil.SIMD_2_PI)
//            {

//        #if BT_INTERNAL_EDGE_DEBUG_DRAW
//                btDebugDrawLine(tr*contact,tr*(contact+cp.m_normalWorldOnB*10),black);
//        #endif //BT_INTERNAL_EDGE_DEBUG_DRAW

//                float len = (contact-nearest).Length();
//                if(len<triangleInfoMapPtr.m_edgeDistanceThreshold)
//                {
//                    isNearEdge = true;
//        #if BT_INTERNAL_EDGE_DEBUG_DRAW
//                    btDebugDrawLine(tr*nearest,tr*(nearest+tri_normal*10),white);
//        #endif //BT_INTERNAL_EDGE_DEBUG_DRAW

//                    Vector3 edge = (v2-v0);

//                    if (info.m_edgeV2V0Angle==0f)
//                    {
//                        numConcaveEdgeHits++;
//                    } 
//                    else
//                    {

//                        bool isEdgeConvex = (info.m_flags & TRI_INFO_V2V0_CONVEX)!=0;
//                        float swapFactor = isEdgeConvex ? 1 : -1f;
//            #if BT_INTERNAL_EDGE_DEBUG_DRAW
//                        btDebugDrawLine(tr*nearest,tr*(nearest+swapFactor*tri_normal*10),white);
//            #endif //BT_INTERNAL_EDGE_DEBUG_DRAW

//                        Vector3 nA = swapFactor * tri_normal;
//                        Quaternion orn = Quaternion.CreateFromAxisAngle(edge,info.m_edgeV2V0Angle);
//                        Vector3 computedNormalB = MathUtil.quatRotate(orn,tri_normal);
//                        if (info.m_flags & TRI_INFO_V2V0_SWAP_NORMALB)
//                        {
//                            computedNormalB*=-1;
//                        }
//                        Vector3 nB = swapFactor*computedNormalB;

//        #if DEBUG_INTERNAL_EDGE
//                        {
//                            btDebugDrawLine(cp.getPositionWorldOnB(),cp.getPositionWorldOnB()+tr.getBasis()*(nB*20),red);
//                        }
//        #endif //DEBUG_INTERNAL_EDGE

//                        float	NdotA = Vector3.Dot(localContactNormalOnB,nA);
//                        float	NdotB = Vector3.Dot(localContactNormalOnB,nB);
//                        bool backFacingNormal = (NdotA< triangleInfoMapPtr.m_convexEpsilon) && (NdotB<triangleInfoMapPtr.m_convexEpsilon);

//                        if (backFacingNormal)
//                        {
//                            numConcaveEdgeHits++;
//                        }
//                        else
//                        {
//                            numConvexEdgeHits++;
//                            //				printf("hitting convex edge\n");


//                            Vector3 localContactNormalOnB = Vector3.TransformNormal(cp.m_normalWorldOnB,MathUtil.transposeBasis(colObj0.getWorldTransform());
//                            Vector3 clampedLocalNormal;
//                            bool isClamped = btClampNormal(edge,swapFactor*tri_normal,localContactNormalOnB,info.m_edgeV2V0Angle,clampedLocalNormal);
//                            if (isClamped)
//                            {
//                                if (((normalAdjustFlags & BT_TRIANGLE_CONVEX_DOUBLE_SIDED)!=0) || (clampedLocalNormal.dot(frontFacing*tri_normal)>0))
//                                {
//                                    Vector3 newNormal = MathUtil.transformNormal(clampedLocalNormal,colObj0.getWorldTransform());
//                                    //					cp.m_distance1 = cp.m_distance1 * newNormal.dot(cp.m_normalWorldOnB);
//                                    cp.m_normalWorldOnB = newNormal;
//                                    // Reproject collision point along normal.
//                                    cp.m_positionWorldOnB = cp.m_positionWorldOnA - cp.m_normalWorldOnB * cp.m_distance1;
//                                    cp.m_localPointB = colObj0.getWorldTransform().invXform(cp.m_positionWorldOnB);
//                                }
//                            }
//                        } 
//                    }
//                }
//            }

//        #if DEBUG_INTERNAL_EDGE
//            {
//                Vector3 color(0,1,1);
//                btDebugDrawLine(cp.getPositionWorldOnB(),cp.getPositionWorldOnB()+cp.m_normalWorldOnB*10,color);
//            }
//        #endif //DEBUG_INTERNAL_EDGE

//            if (isNearEdge)
//            {

//                if (numConcaveEdgeHits>0)
//                {
//                    if ((normalAdjustFlags & BT_TRIANGLE_CONCAVE_DOUBLE_SIDED)!=0)
//                    {
//                        //fix tri_normal so it pointing the same direction as the current local contact normal
//                        if (Vector3.Dot(tri_normal,localContactNormalOnB) < 0)
//                        {
//                            tri_normal *= -1;
//                        }
//                        cp.m_normalWorldOnB = MathUtil.transformNormal(tri_normal,colObj0.getWorldTransform().getBasis());
//                    } 
//                    else
//                    {
//                        //modify the normal to be the triangle normal (or backfacing normal)
//                        cp.m_normalWorldOnB = MathUtil.transformNormal((tri_normal *frontFacing),colObj0.getWorldTransform());
//                    }
        			
//                    // Reproject collision point along normal.
//                    cp.m_positionWorldOnB = cp.m_positionWorldOnA - cp.m_normalWorldOnB * cp.m_distance1;
//                    cp.m_localPointB = colObj0.getWorldTransform().invXform(cp.m_positionWorldOnB);
//                }
//            }
//        }

//    }


//    public enum btInternalEdgeAdjustFlags
//    {
//        BT_TRIANGLE_CONVEX_BACKFACE_MODE = 1,
//        BT_TRIANGLE_CONCAVE_DOUBLE_SIDED = 2, //double sided options are experimental, single sided is recommended
//        BT_TRIANGLE_CONVEX_DOUBLE_SIDED = 4
//    }

//}
