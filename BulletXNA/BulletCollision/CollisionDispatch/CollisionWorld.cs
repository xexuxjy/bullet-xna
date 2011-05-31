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

//#define USE_BRUTEFORCE_RAYBROADPHASE

using System.Diagnostics;
using BulletXNA.BulletCollision.BroadphaseCollision;
using BulletXNA.BulletCollision.CollisionShapes;
using BulletXNA.BulletCollision.NarrowPhaseCollision;
using BulletXNA.LinearMath;
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletCollision.CollisionDispatch
{
    public class CollisionWorld
    {

        //this constructor doesn't own the dispatcher and paircache/broadphase
        public CollisionWorld(IDispatcher dispatcher, IBroadphaseInterface broadphasePairCache, ICollisionConfiguration collisionConfiguration)
        {
            m_dispatcher1 = dispatcher;
            m_broadphasePairCache = broadphasePairCache;
            m_collisionObjects = new ObjectArray<CollisionObject>();
            m_dispatchInfo = new DispatcherInfo();
            m_forceUpdateAllAabbs = true;
        }

        public virtual void Cleanup()
        {
            foreach (CollisionObject collisionObject in m_collisionObjects)
            {
                BroadphaseProxy bp = collisionObject.GetBroadphaseHandle();
                if (bp != null)
                {
                    //
                    // only clear the cached algorithms
                    //
                    if (GetBroadphase().GetOverlappingPairCache() != null)
                    {
                        GetBroadphase().GetOverlappingPairCache().CleanProxyFromPairs(bp, m_dispatcher1);
                    }
                    GetBroadphase().DestroyProxy(bp, m_dispatcher1);
                    collisionObject.SetBroadphaseHandle(null);
                }
            }
        }

        public void SetBroadphase(IBroadphaseInterface pairCache)
        {
            m_broadphasePairCache = pairCache;
        }

        public IBroadphaseInterface GetBroadphase()
        {
            return m_broadphasePairCache;
        }

        public IOverlappingPairCache GetPairCache()
        {
            return m_broadphasePairCache.GetOverlappingPairCache();
        }

        public IDispatcher GetDispatcher()
        {
            return m_dispatcher1;
        }

        public void UpdateSingleAabb(CollisionObject colObj)
        {
            Vector3 minAabb;
            Vector3 maxAabb;
            Matrix wt = colObj.GetWorldTransform();
            colObj.GetCollisionShape().GetAabb(ref wt, out minAabb, out maxAabb);
            //need to increase the aabb for contact thresholds
            Vector3 contactThreshold = new Vector3(BulletGlobals.gContactBreakingThreshold);
            minAabb -= contactThreshold;
            maxAabb += contactThreshold;

            if(GetDispatchInfo().m_useContinuous && colObj.GetInternalType()==CollisionObjectTypes.CO_RIGID_BODY)
	        {
		        Vector3 minAabb2,maxAabb2;
		        colObj.GetCollisionShape().GetAabb(colObj.GetInterpolationWorldTransform(),out minAabb2 ,out maxAabb2);
		        minAabb2 -= contactThreshold;
		        maxAabb2 += contactThreshold;
		        MathUtil.VectorMin(ref minAabb2,ref minAabb);
                MathUtil.VectorMax(ref maxAabb2, ref maxAabb);
            }

            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugCollisionWorld)
            {
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "updateSingleAabbMin", minAabb);
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "updateSingleAabbMax", maxAabb);
            }


            IBroadphaseInterface bp = m_broadphasePairCache as IBroadphaseInterface;

            //moving objects should be moderately sized, probably something wrong if not
            if (colObj.IsStaticObject() || ((maxAabb - minAabb).LengthSquared() < 1e12f))
            {
                bp.SetAabb(colObj.GetBroadphaseHandle(), ref minAabb, ref maxAabb, m_dispatcher1);
            }
            else
            {
                //something went wrong, investigate
                //this assert is unwanted in 3D modelers (danger of loosing work)
                colObj.SetActivationState(ActivationState.DISABLE_SIMULATION);

                //static bool reportMe = true;
                bool reportMe = true;
                if (reportMe && m_debugDrawer != null)
                {
                    reportMe = false;
                    m_debugDrawer.ReportErrorWarning("Overflow in AABB, object removed from simulation");
                    m_debugDrawer.ReportErrorWarning("If you can reproduce this, please email bugs@continuousphysics.com\n");
                    m_debugDrawer.ReportErrorWarning("Please include above information, your Platform, version of OS.\n");
                    m_debugDrawer.ReportErrorWarning("Thanks.\n");
                }
            }
        }

        public virtual void UpdateAabbs()
        {
            BulletGlobals.StartProfile("updateAabbs");

            //Matrix predictedTrans = new Matrix();
            for (int i = 0; i < m_collisionObjects.Count; i++)
            {
                CollisionObject colObj = m_collisionObjects[i];

                //only update aabb of active objects
                if (m_forceUpdateAllAabbs || colObj.IsActive())
                {
                    UpdateSingleAabb(colObj);
                }
            }
            BulletGlobals.StopProfile();
        }


        public virtual void SetDebugDrawer(IDebugDraw debugDrawer)
        {
            m_debugDrawer = debugDrawer;
            BulletGlobals.gDebugDraw = debugDrawer;
        }

        public virtual IDebugDraw GetDebugDrawer()
        {
            return m_debugDrawer;
        }

        public virtual void DebugDrawWorld()
        {
            if (GetDebugDrawer() != null && ((GetDebugDrawer().GetDebugMode() & DebugDrawModes.DBG_DrawContactPoints) != 0))
            {
                int numManifolds = GetDispatcher().GetNumManifolds();
                Vector3 color = Vector3.Zero;
                for (int i=0;i<numManifolds;i++)
                {
                    PersistentManifold contactManifold = GetDispatcher().GetManifoldByIndexInternal(i);
                    //btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold.getBody0());
                    //btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold.getBody1());

                    int numContacts = contactManifold.GetNumContacts();
                    for (int j=0;j<numContacts;j++)
                    {
                        ManifoldPoint cp = contactManifold.GetContactPoint(j);
                        GetDebugDrawer().DrawContactPoint(cp.GetPositionWorldOnB(),cp.GetNormalWorldOnB(),cp.GetDistance(),cp.GetLifeTime(),color);
                    }
                }
            }

            if (GetDebugDrawer() != null)
            {
                DebugDrawModes debugMode = GetDebugDrawer().GetDebugMode();
                bool wireFrame = (debugMode & DebugDrawModes.DBG_DrawWireframe) != 0;
                bool aabb = (debugMode & DebugDrawModes.DBG_DrawAabb) != 0;

                if(wireFrame || aabb)
                {
					int length = m_collisionObjects.Count;
					for (int i = 0; i < length;++i )
					{
	                    CollisionObject colObj = m_collisionObjects[i];

                        if (wireFrame)
                        {
                            Vector3 color = new Vector3(255, 255, 255);
                            switch (colObj.GetActivationState())
                            {
                                case ActivationState.ACTIVE_TAG:
                                    {
                                        color = new Vector3(255, 255, 255);
                                        break;
                                    }
                                case ActivationState.ISLAND_SLEEPING:
                                    {
                                        color = new Vector3(0, 255, 0);
                                        break;
                                    }
                                case ActivationState.WANTS_DEACTIVATION:
                                    {
                                        color = new Vector3(0, 255, 255);
                                        break;
                                    }
                                case ActivationState.DISABLE_DEACTIVATION:
                                    {
                                        color = new Vector3(255, 0, 0);
                                        break;
                                    }
                                case ActivationState.DISABLE_SIMULATION:
                                    {
                                        color = new Vector3(255, 255, 0);
                                        break;
                                    }

                                default:
                                    {
                                        color = new Vector3(255, 0, 0);
                                        break;
                                    }
                            };
                            Matrix transform = colObj.GetWorldTransform();
                            //DrawHelper.debugDrawObject(ref transform, colObj.getCollisionShape(), ref color, getDebugDrawer());
                        }
                        if (aabb)
                        {
                            Vector3 minAabb;
                            Vector3 maxAabb;
                            Vector3 colorvec = new Vector3(1, 0, 0);
                            colObj.GetCollisionShape().GetAabb(colObj.GetWorldTransform(), out minAabb, out maxAabb);
                	        Vector3 contactThreshold = new Vector3(BulletGlobals.gContactBreakingThreshold);
					        minAabb -= contactThreshold;
					        maxAabb += contactThreshold;

					        Vector3 minAabb2,maxAabb2;

					        if(colObj.GetInternalType()==CollisionObjectTypes.CO_RIGID_BODY)
					        {
						        colObj.GetCollisionShape().GetAabb(colObj.GetInterpolationWorldTransform(),out minAabb2,out maxAabb2);
						        minAabb2 -= contactThreshold;
						        maxAabb2 += contactThreshold;
						        MathUtil.VectorMin(ref minAabb2,ref minAabb);
						        MathUtil.VectorMax(ref maxAabb2,ref maxAabb);
					        }

                            m_debugDrawer.DrawAabb(ref minAabb, ref maxAabb, ref colorvec);
                        }

                    }
        	
                }
            }

        }

        public virtual void DebugDrawObject(ref Matrix worldTransform, CollisionShape shape, ref Vector3 color)
        {
        }


        public int GetNumCollisionObjects()
        {
            return m_collisionObjects.Count;
        }

        /// rayTest performs a raycast on all objects in the btCollisionWorld, and calls the resultCallback
        /// This allows for several queries: first hit, all hits, any hit, dependent on the value returned by the callback.
        public virtual void RayTest(ref Vector3 rayFromWorld, ref Vector3 rayToWorld, RayResultCallback resultCallback)
        {
            BulletGlobals.StartProfile("rayTest");
            /// use the broadphase to accelerate the search for objects, based on their aabb
            /// and for each object with ray-aabb overlap, perform an exact ray test
            SingleRayCallback rayCB = new SingleRayCallback(ref rayFromWorld, ref rayToWorld, this, resultCallback);

#if !USE_BRUTEFORCE_RAYBROADPHASE
            m_broadphasePairCache.RayTest(ref rayFromWorld, ref rayToWorld, rayCB);
            rayCB.Cleanup();

#else
	        for (int i=0;i<GetNumCollisionObjects();i++)
	        {
		        rayCB.Process(m_collisionObjects[i].GetBroadphaseHandle());
	        }	
#endif //USE_BRUTEFORCE_RAYBROADPHASE
            BulletGlobals.StopProfile();
        }



        //fixed me == checked Debug draw world


        ///contactTest performs a discrete collision test between colObj against all objects in the btCollisionWorld, and calls the resultCallback.
        ///it reports one or more contact points for every overlapping object (including the one with deepest penetration)
        public void ContactTest(CollisionObject colObj, ContactResultCallback resultCallback)
        {
        }

        ///contactTest performs a discrete collision test between two collision objects and calls the resultCallback if overlap if detected.
        ///it reports one or more contact points (including the one with deepest penetration)
        public void ContactPairTest(CollisionObject colObjA, CollisionObject colObjB, ContactResultCallback resultCallback)
        {
        }


        // convexTest performs a swept convex cast on all objects in the btCollisionWorld, and calls the resultCallback
        // This allows for several queries: first hit, all hits, any hit, dependent on the value return by the callback.
        public virtual void ConvexSweepTest(ConvexShape castShape, Matrix convexFromWorld, Matrix convexToWorld, ConvexResultCallback resultCallback, float allowedCcdPenetration)
        {
            ConvexSweepTest(castShape, ref convexFromWorld, ref convexToWorld, resultCallback, allowedCcdPenetration);
        }

        public virtual void ConvexSweepTest(ConvexShape castShape, ref Matrix convexFromWorld, ref Matrix convexToWorld, ConvexResultCallback resultCallback, float allowedCcdPenetration)
        {
            BulletGlobals.StartProfile("convexSweepTest");
            /// use the broadphase to accelerate the search for objects, based on their aabb
            /// and for each object with ray-aabb overlap, perform an exact ray test
            /// unfortunately the implementation for rayTest and convexSweepTest duplicated, albeit practically identical

            Matrix convexFromTrans = new Matrix();
            Matrix convexToTrans = new Matrix();
            convexFromTrans = convexFromWorld;
            convexToTrans = convexToWorld;
            Vector3 castShapeAabbMin;
            Vector3 castShapeAabbMax;
            /* Compute AABB that encompasses angular movement */
            {
                Vector3 linVel = new Vector3();
                Vector3 angVel = new Vector3();
                TransformUtil.CalculateVelocity(ref convexFromTrans, ref convexToTrans, 1.0f, ref linVel, ref angVel);
                Vector3 zeroLinVel = new Vector3();
                Matrix R = MathUtil.BasisMatrix(ref convexFromTrans);
                castShape.CalculateTemporalAabb(ref R, ref zeroLinVel, ref angVel, 1.0f, out castShapeAabbMin, out castShapeAabbMax);
            }

#if !USE_BRUTEFORCE_RAYBROADPHASE
            SingleSweepCallback convexCB = new SingleSweepCallback(castShape, ref convexFromWorld, ref convexToWorld, this, resultCallback, allowedCcdPenetration);
            Vector3 tempFrom = convexFromTrans.Translation;
            Vector3 tempTo = convexToTrans.Translation;
            m_broadphasePairCache.RayTest(ref tempFrom, ref tempTo, convexCB, ref castShapeAabbMin, ref castShapeAabbMax);
            convexCB.Cleanup();
#else
	        /// go over all objects, and if the ray intersects their aabb + cast shape aabb,
	        // do a ray-shape query using convexCaster (CCD)
	        int i;
	        for (i=0;i<m_collisionObjects.Count;i++)
	        {
		        CollisionObject	collisionObject= m_collisionObjects[i];
		        //only perform raycast if filterMask matches
		        if(resultCallback.NeedsCollision(collisionObject.GetBroadphaseHandle())) 
                {
			        //RigidcollisionObject* collisionObject = ctrl.GetRigidcollisionObject();
			        Vector3 collisionObjectAabbMin = new Vector3();
                    Vector3 collisionObjectAabbMax = new Vector3();
			        collisionObject.GetCollisionShape().GetAabb(collisionObject.GetWorldTransform(),ref collisionObjectAabbMin,ref collisionObjectAabbMax);
			        AabbUtil2.AabbExpand(ref collisionObjectAabbMin, ref collisionObjectAabbMax, ref castShapeAabbMin, ref castShapeAabbMax);
			        float hitLambda = 1f; //could use resultCallback.m_closestHitFraction, but needs testing
			        Vector3 hitNormal = new Vector3();
                    Vector3 fromOrigin = convexFromWorld.Translation;
                    Vector3 toOrigin = convexToWorld.Translation;
                    if (AabbUtil2.RayAabb(ref fromOrigin, ref toOrigin, ref collisionObjectAabbMin, ref collisionObjectAabbMax, ref hitLambda, ref hitNormal))
			        {
                        Matrix trans = collisionObject.GetWorldTransform();
				        ObjectQuerySingle(castShape, ref convexFromTrans,ref convexToTrans,
					        collisionObject,
						        collisionObject.GetCollisionShape(),
						        ref trans,
						        resultCallback,
						        allowedCcdPenetration);
			        }
		        }
	        }
#endif //USE_BRUTEFORCE_RAYBROADPHASE
            BulletGlobals.StopProfile();
        }

        public virtual void AddCollisionObject(CollisionObject collisionObject)
        {
            AddCollisionObject(collisionObject, CollisionFilterGroups.DefaultFilter, CollisionFilterGroups.AllFilter);
        }

        public virtual void AddCollisionObject(CollisionObject collisionObject, CollisionFilterGroups collisionFilterGroup, CollisionFilterGroups collisionFilterMask)
        {
            //check that the object isn't already added
            //btAssert( m_collisionObjects.findLinearSearch(collisionObject)  == m_collisionObjects.size());

            Debug.Assert(collisionObject != null);
            m_collisionObjects.Add(collisionObject);

            //calculate new AABB
            Matrix trans = collisionObject.GetWorldTransform();
            Vector3 minAabb;
            Vector3 maxAabb;

            collisionObject.GetCollisionShape().GetAabb(ref trans, out minAabb, out maxAabb);

            BroadphaseNativeTypes type = collisionObject.GetCollisionShape().GetShapeType();
            collisionObject.SetBroadphaseHandle(GetBroadphase().CreateProxy(
                ref minAabb,
                ref maxAabb,
                type,
                collisionObject,
                collisionFilterGroup,
                collisionFilterMask,
                m_dispatcher1, 0
                ));
        }

        public ObjectArray<CollisionObject> GetCollisionObjectArray()
        {
            return m_collisionObjects;
        }

        public DispatcherInfo GetDispatchInfo()
        {
            return m_dispatchInfo;
        }

        public virtual void PerformDiscreteCollisionDetection()
        {
            BulletGlobals.StartProfile("performDiscreteCollisionDetection");

            DispatcherInfo dispatchInfo = GetDispatchInfo();

            UpdateAabbs();

            {
                BulletGlobals.StartProfile("calculateOverlappingPairs");
                m_broadphasePairCache.CalculateOverlappingPairs(m_dispatcher1);
                BulletGlobals.StopProfile();
            }


            IDispatcher dispatcher = GetDispatcher();
            {
                BulletGlobals.StartProfile("dispatchAllCollisionPairs");
                if (dispatcher != null)
                {
                    dispatcher.DispatchAllCollisionPairs(m_broadphasePairCache.GetOverlappingPairCache(), dispatchInfo, m_dispatcher1);
                }
                BulletGlobals.StopProfile();
            }
            BulletGlobals.StopProfile();
        }


        public virtual void RemoveCollisionObject(CollisionObject collisionObject)
        {
            //bool removeFromBroadphase = false;
            {
                BroadphaseProxy bp = collisionObject.GetBroadphaseHandle();
                if (bp != null)
                {
                    //
                    // only clear the cached algorithms
                    //
                    GetBroadphase().GetOverlappingPairCache().CleanProxyFromPairs(bp, m_dispatcher1);
                    GetBroadphase().DestroyProxy(bp, m_dispatcher1);
                    collisionObject.SetBroadphaseHandle(null);
                }
            }
            //swapremove
            m_collisionObjects.Remove(collisionObject);
        }

        public bool GetForceUpdateAllAabbs()
        {
            return m_forceUpdateAllAabbs;
        }

        public void SetForceUpdateAllAabbs(bool forceUpdateAllAabbs)
        {
            m_forceUpdateAllAabbs = forceUpdateAllAabbs;
        }


        public static void RayTestSingle(ref Matrix rayFromTrans, ref Matrix rayToTrans,
                          CollisionObject collisionObject,
                          CollisionShape collisionShape,
                          ref Matrix colObjWorldTransform,
                          RayResultCallback resultCallback)
        {
            SphereShape pointShape = new SphereShape(0.0f);
            pointShape.SetMargin(0f);
            ConvexShape castShape = pointShape;

            if (collisionShape.IsConvex())
            {
                BulletGlobals.StartProfile("rayTestConvex");
                CastResult castResult = new CastResult();
                castResult.m_fraction = resultCallback.m_closestHitFraction;

                ConvexShape convexShape = collisionShape as ConvexShape;
                VoronoiSimplexSolver simplexSolver = new VoronoiSimplexSolver();
                //#define USE_SUBSIMPLEX_CONVEX_CAST 1
                //#ifdef USE_SUBSIMPLEX_CONVEX_CAST

                // FIXME - MAN - convexcat here seems to make big difference to forklift.
                SubSimplexConvexCast convexCaster = new SubSimplexConvexCast(castShape, convexShape, simplexSolver);

                //GjkConvexCast convexCaster = new GjkConvexCast(castShape, convexShape, simplexSolver);


                //#else
                //btGjkConvexCast	convexCaster(castShape,convexShape,&simplexSolver);
                //btContinuousConvexCollision convexCaster(castShape,convexShape,&simplexSolver,0);
                //#endif //#USE_SUBSIMPLEX_CONVEX_CAST

                if (convexCaster.CalcTimeOfImpact(ref rayFromTrans, ref rayToTrans, ref colObjWorldTransform, ref colObjWorldTransform, castResult))
                {
                    //add hit
                    if (castResult.m_normal.LengthSquared() > 0.0001f)
                    {
                        if (castResult.m_fraction < resultCallback.m_closestHitFraction)
                        {

                            //if (resultCallback.m_closestHitFraction != 1f)
                            //{
                            //    int ibreak = 0;
                            //    convexCaster.calcTimeOfImpact(ref rayFromTrans, ref rayToTrans, ref colObjWorldTransform, ref colObjWorldTransform, castResult);
                            //}

                            //#ifdef USE_SUBSIMPLEX_CONVEX_CAST
                            //rotate normal into worldspace
                            castResult.m_normal = Vector3.TransformNormal(castResult.m_normal, rayFromTrans);
                            //#endif //USE_SUBSIMPLEX_CONVEX_CAST

                            castResult.m_normal.Normalize();
                            LocalRayResult localRayResult = new LocalRayResult(
                                    collisionObject,
                                    null,
                                    ref castResult.m_normal,
                                    castResult.m_fraction
                                );

                            bool normalInWorldSpace = true;
                            resultCallback.AddSingleResult(localRayResult, normalInWorldSpace);

                        }
                    }
                }
                castResult.Cleanup();
                BulletGlobals.StopProfile();
            }
            else
            {
                if (collisionShape.IsConcave())
                {
                    BulletGlobals.StartProfile("rayTestConcave");
                    if (collisionShape.GetShapeType() == BroadphaseNativeTypes.TRIANGLE_MESH_SHAPE_PROXYTYPE && collisionShape is BvhTriangleMeshShape)
                    {
                        ///optimized version for btBvhTriangleMeshShape
                        BvhTriangleMeshShape triangleMesh = (BvhTriangleMeshShape)collisionShape;
                        Matrix worldTocollisionObject = Matrix.Invert(colObjWorldTransform);
                        Vector3 rayFromLocal = Vector3.Transform(rayFromTrans.Translation, worldTocollisionObject);
                        Vector3 rayToLocal = Vector3.Transform(rayToTrans.Translation, worldTocollisionObject);

                        Matrix transform = Matrix.Identity;
                        BridgeTriangleRaycastCallback rcb = new BridgeTriangleRaycastCallback(ref rayFromLocal, ref rayToLocal, resultCallback, collisionObject, triangleMesh, ref transform);
                        rcb.m_hitFraction = resultCallback.m_closestHitFraction;
                        triangleMesh.PerformRaycast(rcb, ref rayFromLocal, ref rayToLocal);
                        rcb.Cleanup();
                    }
                    else
                    {
                        //generic (slower) case
                        ConcaveShape concaveShape = (ConcaveShape)collisionShape;

                        Matrix worldTocollisionObject = Matrix.Invert(colObjWorldTransform);

                        Vector3 rayFromLocal = Vector3.Transform(rayFromTrans.Translation, worldTocollisionObject);
                        Vector3 rayToLocal = Vector3.Transform(rayToTrans.Translation, worldTocollisionObject);

                        //ConvexCast::CastResult
                        Matrix transform = Matrix.Identity;
                        BridgeTriangleConcaveRaycastCallback rcb = new BridgeTriangleConcaveRaycastCallback(ref rayFromLocal, ref rayToLocal, resultCallback, collisionObject, concaveShape, ref transform);
                        rcb.m_hitFraction = resultCallback.m_closestHitFraction;

                        Vector3 rayAabbMinLocal = rayFromLocal;
                        MathUtil.VectorMin(ref rayToLocal, ref rayAabbMinLocal);
                        Vector3 rayAabbMaxLocal = rayFromLocal;
                        MathUtil.VectorMax(ref rayToLocal, ref rayAabbMaxLocal);

                        concaveShape.ProcessAllTriangles(rcb, ref rayAabbMinLocal, ref rayAabbMaxLocal);
                        rcb.Cleanup();
                    }
                    BulletGlobals.StopProfile();
                }
                else
                {
                    BulletGlobals.StartProfile("rayTestCompound");
                    ///@todo: use AABB tree or other BVH acceleration structure, see btDbvt
                    if (collisionShape.IsCompound())
                    {

                        CompoundShape compoundShape = collisionShape as CompoundShape;
                        Dbvt dbvt = compoundShape.GetDynamicAabbTree();


                        RayTester rayCB = new RayTester(
                            collisionObject,
                            compoundShape,
                            ref colObjWorldTransform,
                            ref rayFromTrans,
                            ref rayToTrans,
                            resultCallback);
#if !DISABLE_DBVT_COMPOUNDSHAPE_RAYCAST_ACCELERATION
                        if (dbvt != null)
                        {
                            Vector3 localRayFrom = MathUtil.InverseTimes(ref colObjWorldTransform, ref rayFromTrans).Translation;
                            Vector3 localRayTo = MathUtil.InverseTimes(ref colObjWorldTransform, ref rayToTrans).Translation;
                            Dbvt.RayTest(dbvt.m_root, ref localRayFrom, ref localRayTo, rayCB);
                        }
                        else
#endif //DISABLE_DBVT_COMPOUNDSHAPE_RAYCAST_ACCELERATION
                        {
                            for (int i = 0, n = compoundShape.GetNumChildShapes(); i < n; ++i)
                            {
                                rayCB.Process(i);
                            }
                        }
                        rayCB.Cleanup();
                        BulletGlobals.StopProfile();
                    }
                }
            }
        }

        /// objectQuerySingle performs a collision detection query and calls the resultCallback. It is used internally by rayTest.
        public static void ObjectQuerySingle(ConvexShape castShape, ref Matrix convexFromTrans, ref Matrix convexToTrans,
                          CollisionObject collisionObject, CollisionShape collisionShape,
                          ref Matrix colObjWorldTransform,
                          ConvexResultCallback resultCallback, float allowedPenetration)
        {
            if (collisionShape.IsConvex())
            {

                BulletGlobals.StartProfile("convexSweepConvex");
                CastResult castResult = new CastResult();
                castResult.m_allowedPenetration = allowedPenetration;
                castResult.m_fraction = resultCallback.m_closestHitFraction;//float(1.);//??

                ConvexShape convexShape = collisionShape as ConvexShape;
                VoronoiSimplexSolver simplexSolver = new VoronoiSimplexSolver();
                GjkEpaPenetrationDepthSolver gjkEpaPenetrationSolver = new GjkEpaPenetrationDepthSolver();

                ContinuousConvexCollision convexCaster1 = new ContinuousConvexCollision(castShape, convexShape, simplexSolver, gjkEpaPenetrationSolver);
                //btGjkConvexCast convexCaster2(castShape,convexShape,&simplexSolver);
                //btSubsimplexConvexCast convexCaster3(castShape,convexShape,&simplexSolver);

                IConvexCast castPtr = convexCaster1;

                if (castPtr.CalcTimeOfImpact(ref convexFromTrans, ref convexToTrans, ref colObjWorldTransform, ref colObjWorldTransform, castResult))
                {
                    //add hit
                    if (castResult.m_normal.LengthSquared() > 0.0001f)
                    {
                        if (castResult.m_fraction < resultCallback.m_closestHitFraction)
                        {
                            castResult.m_normal.Normalize();
                            LocalConvexResult localConvexResult = new LocalConvexResult
                                        (
                                            collisionObject,
                                            null,
                                            ref castResult.m_normal,
                                            ref castResult.m_hitPoint,
                                            castResult.m_fraction
                                        );

                            bool normalInWorldSpace = true;
                            resultCallback.AddSingleResult(localConvexResult, normalInWorldSpace);

                        }
                    }
                }
                BulletGlobals.StopProfile();
            }
            else
            {
				if (collisionShape.IsConcave())
				{
					if (collisionShape.GetShapeType() == BroadphaseNativeTypes.TRIANGLE_MESH_SHAPE_PROXYTYPE)
					{
						BulletGlobals.StartProfile("convexSweepbtBvhTriangleMesh");
						BvhTriangleMeshShape triangleMesh = (BvhTriangleMeshShape)collisionShape;
						Matrix worldTocollisionObject = Matrix.Invert(colObjWorldTransform);
						Vector3 convexFromLocal = Vector3.Transform(convexFromTrans.Translation, worldTocollisionObject);
						Vector3 convexToLocal = Vector3.Transform(convexToTrans.Translation, worldTocollisionObject);
						// rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation

						Matrix rotationXform = MathUtil.BasisMatrix(ref worldTocollisionObject) * MathUtil.BasisMatrix(ref convexToTrans);

						BridgeTriangleConvexcastCallback tccb = new BridgeTriangleConvexcastCallback(castShape, ref convexFromTrans, ref convexToTrans, resultCallback, collisionObject, triangleMesh, ref colObjWorldTransform);
						tccb.m_hitFraction = resultCallback.m_closestHitFraction;
						tccb.m_allowedPenetration = allowedPenetration;

						Vector3 boxMinLocal;
						Vector3 boxMaxLocal;
						castShape.GetAabb(ref rotationXform, out boxMinLocal, out boxMaxLocal);
						triangleMesh.PerformConvexCast(tccb, ref convexFromLocal, ref convexToLocal, ref boxMinLocal, ref boxMaxLocal);
						BulletGlobals.StopProfile();
					}
					else
					{
						if (collisionShape.GetShapeType() == BroadphaseNativeTypes.STATIC_PLANE_PROXYTYPE)
						{
							CastResult castResult = new CastResult();
							castResult.m_allowedPenetration = allowedPenetration;
							castResult.m_fraction = resultCallback.m_closestHitFraction;
							StaticPlaneShape planeShape = collisionShape as StaticPlaneShape;
							ContinuousConvexCollision convexCaster1 = new ContinuousConvexCollision(castShape, planeShape);

							if (convexCaster1.CalcTimeOfImpact(ref convexFromTrans, ref convexToTrans, ref colObjWorldTransform, ref colObjWorldTransform, castResult))
							{
								//add hit
								if (castResult.m_normal.LengthSquared() > 0.0001f)
								{
									if (castResult.m_fraction < resultCallback.m_closestHitFraction)
									{
										castResult.m_normal.Normalize();
										LocalConvexResult localConvexResult = new LocalConvexResult
											(
											collisionObject,
											null,
											ref castResult.m_normal,
											ref castResult.m_hitPoint,
											castResult.m_fraction
											);

										bool normalInWorldSpace = true;
										resultCallback.AddSingleResult(localConvexResult, normalInWorldSpace);
									}
								}
							}

						}
						else
						{
							BulletGlobals.StartProfile("convexSweepConcave");
							ConcaveShape concaveShape = (ConcaveShape)collisionShape;
							Matrix worldTocollisionObject = Matrix.Invert(colObjWorldTransform);
							Vector3 convexFromLocal = Vector3.Transform(convexFromTrans.Translation, worldTocollisionObject);
							Vector3 convexToLocal = Vector3.Transform(convexToTrans.Translation, worldTocollisionObject);
							// rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
							Matrix rotationXform = MathUtil.BasisMatrix(ref worldTocollisionObject) * MathUtil.BasisMatrix(ref convexToTrans);

							BridgeTriangleConvexcastCallback2 tccb = new BridgeTriangleConvexcastCallback2(castShape, ref convexFromTrans, ref convexToTrans, resultCallback, collisionObject, concaveShape, ref colObjWorldTransform);
							tccb.m_hitFraction = resultCallback.m_closestHitFraction;
							tccb.m_allowedPenetration = allowedPenetration;
							Vector3 boxMinLocal;
							Vector3 boxMaxLocal;
							castShape.GetAabb(ref rotationXform, out boxMinLocal, out boxMaxLocal);

							Vector3 rayAabbMinLocal = convexFromLocal;
							MathUtil.VectorMin(ref convexToLocal, ref rayAabbMinLocal);
							//rayAabbMinLocal.setMin(convexToLocal);
							Vector3 rayAabbMaxLocal = convexFromLocal;
							//rayAabbMaxLocal.setMax(convexToLocal);
							MathUtil.VectorMax(ref convexToLocal, ref rayAabbMaxLocal);

							rayAabbMinLocal += boxMinLocal;
							rayAabbMaxLocal += boxMaxLocal;
							concaveShape.ProcessAllTriangles(tccb, ref rayAabbMinLocal, ref rayAabbMaxLocal);
							BulletGlobals.StopProfile();
						}
					}
				}
				else
				{
					///@todo : use AABB tree or other BVH acceleration structure!
					if (collisionShape.IsCompound())
					{
						BulletGlobals.StartProfile("convexSweepCompound");
						CompoundShape compoundShape = (CompoundShape)collisionShape;
						for (int i = 0; i < compoundShape.GetNumChildShapes(); i++)
						{
							Matrix childTrans = compoundShape.GetChildTransform(i);
							CollisionShape childCollisionShape = compoundShape.GetChildShape(i);
							Matrix childWorldTrans = MathUtil.BulletMatrixMultiply(colObjWorldTransform, childTrans);
							// replace collision shape so that callback can determine the triangle
							CollisionShape saveCollisionShape = collisionObject.GetCollisionShape();
							collisionObject.InternalSetTemporaryCollisionShape(childCollisionShape);

							LocalInfoAdder my_cb = new LocalInfoAdder(i, resultCallback);
							my_cb.m_closestHitFraction = resultCallback.m_closestHitFraction;


							ObjectQuerySingle(castShape, ref convexFromTrans, ref convexToTrans,
								collisionObject,
								childCollisionShape,
								ref childWorldTrans,
								my_cb, allowedPenetration);
							// restore
							collisionObject.InternalSetTemporaryCollisionShape(saveCollisionShape);
						}
						BulletGlobals.StopProfile();
					}
				}
            }
        }
        protected ObjectArray<CollisionObject> m_collisionObjects;
        protected IDispatcher m_dispatcher1;
        protected DispatcherInfo m_dispatchInfo;

        protected IBroadphaseInterface m_broadphasePairCache;
        protected IDebugDraw m_debugDrawer;

        ///m_forceUpdateAllAabbs can be set to false as an optimization to only update active object AABBs
        ///it is true by default, because it is error-prone (setting the position of static objects wouldn't update their AABB)
        protected bool m_forceUpdateAllAabbs;

	       protected IProfileManager m_profileManager;
    }



    ///LocalShapeInfo gives extra information for complex shapes
    ///Currently, only btTriangleMeshShape is available, so it just contains triangleIndex and subpart
    public class LocalShapeInfo
    {
        public int m_shapePart;
        public int m_triangleIndex;

        //const btCollisionShape*	m_shapeTemp;
        //const btTransform*	m_shapeLocalTransform;
    };



    public class LocalRayResult
    {
        public LocalRayResult(CollisionObject collisionObject,
            LocalShapeInfo localShapeInfo,
            ref Vector3 hitNormalLocal,
            float hitFraction)
        {
            m_collisionObject = collisionObject;
            m_localShapeInfo = localShapeInfo;
            m_hitNormalLocal = hitNormalLocal;
            m_hitFraction = hitFraction;
        }

        public CollisionObject m_collisionObject;
        public LocalShapeInfo m_localShapeInfo;
        public Vector3 m_hitNormalLocal;
        public float m_hitFraction;

    };



    ///RayResultCallback is used to report new raycast results
    public abstract class RayResultCallback
    {

        public bool HasHit()
        {
            return (m_collisionObject != null);
        }

        public RayResultCallback()
        {
            m_closestHitFraction = 1f;
            m_collisionObject = null;
            m_collisionFilterGroup = CollisionFilterGroups.DefaultFilter;
            m_collisionFilterMask = CollisionFilterGroups.AllFilter;
            //@BP Mod
            m_flags = 0;
        }

        public virtual bool NeedsCollision(BroadphaseProxy proxy0)
        {
            bool collides = (proxy0.m_collisionFilterGroup & m_collisionFilterMask) != 0;
            collides = collides && ((m_collisionFilterGroup & proxy0.m_collisionFilterMask) != 0);
            return collides;
        }

        public abstract float AddSingleResult(LocalRayResult rayResult, bool normalInWorldSpace);
        public virtual void Cleanup()
        {
        }

        public float m_closestHitFraction;
        public CollisionObject m_collisionObject;
        public CollisionFilterGroups m_collisionFilterGroup;
        public CollisionFilterGroups m_collisionFilterMask;
        //@BP Mod - Custom flags, currently used to enable backface culling on tri-meshes, see btRaycastCallback
        public EFlags m_flags;
    }

    public class ClosestRayResultCallback : RayResultCallback
    {

        public ClosestRayResultCallback(Vector3 rayFromWorld, Vector3 rayToWorld)
        {
            m_rayFromWorld = rayFromWorld;
            m_rayToWorld = rayToWorld;
        }


        public ClosestRayResultCallback(ref Vector3 rayFromWorld, ref Vector3 rayToWorld)
        {
            m_rayFromWorld = rayFromWorld;
            m_rayToWorld = rayToWorld;
        }

        public Vector3 m_rayFromWorld;//used to calculate hitPointWorld from hitFraction
        public Vector3 m_rayToWorld;

        public Vector3 m_hitNormalWorld;
        public Vector3 m_hitPointWorld;

        public override float AddSingleResult(LocalRayResult rayResult, bool normalInWorldSpace)
        {
            //caller already does the filter on the m_closestHitFraction
            //btAssert(rayResult.m_hitFraction <= m_closestHitFraction);

            m_closestHitFraction = rayResult.m_hitFraction;
            m_collisionObject = rayResult.m_collisionObject;
            if (normalInWorldSpace)
            {
                m_hitNormalWorld = rayResult.m_hitNormalLocal;
            }
            else
            {
                ///need to transform normal into worldspace
                m_hitNormalWorld = Vector3.TransformNormal(rayResult.m_hitNormalLocal, m_collisionObject.GetWorldTransform());
            }
            m_hitPointWorld = MathUtil.Interpolate3(ref m_rayFromWorld, ref m_rayToWorld, rayResult.m_hitFraction);
            return rayResult.m_hitFraction;
        }

    }



    public class LocalConvexResult
    {
        public LocalConvexResult(CollisionObject hitCollisionObject,
            LocalShapeInfo localShapeInfo,
            ref Vector3 hitNormalLocal,
            ref Vector3 hitPointLocal,
            float hitFraction
            )
        {
            m_hitCollisionObject = hitCollisionObject;
            m_localShapeInfo = localShapeInfo;
            m_hitNormalLocal = hitNormalLocal;
            m_hitPointLocal = hitPointLocal;
            m_hitFraction = hitFraction;
        }

        public CollisionObject m_hitCollisionObject;
        public LocalShapeInfo m_localShapeInfo;
        public Vector3 m_hitNormalLocal;
        public Vector3 m_hitPointLocal;
        public float m_hitFraction;
    };

	public class AllHitsRayResultCallback : RayResultCallback
	{
		public AllHitsRayResultCallback(ref Vector3	rayFromWorld,ref Vector3 rayToWorld)
		{
		    m_rayFromWorld = rayFromWorld;
		    m_rayToWorld = rayToWorld;
		}

		public ObjectArray<CollisionObject>	m_collisionObjects = new ObjectArray<CollisionObject>();

		Vector3	m_rayFromWorld;//used to calculate hitPointWorld from hitFraction
		Vector3	m_rayToWorld;

		public ObjectArray<Vector3>	m_hitNormalWorld = new ObjectArray<Vector3>();
		public ObjectArray<Vector3>	m_hitPointWorld= new ObjectArray<Vector3>();
		public ObjectArray<float> m_hitFractions = new ObjectArray<float>();
			
		public override float AddSingleResult(LocalRayResult rayResult,bool normalInWorldSpace)
		{
			m_collisionObject = rayResult.m_collisionObject;
			m_collisionObjects.Add(rayResult.m_collisionObject);
			Vector3 hitNormalWorld;
			if (normalInWorldSpace)
			{
				hitNormalWorld = rayResult.m_hitNormalLocal;
			} else
			{
				///need to transform normal into worldspace
				hitNormalWorld = Vector3.TransformNormal(rayResult.m_hitNormalLocal,m_collisionObject.GetWorldTransform());
			}
			m_hitNormalWorld.Add(hitNormalWorld);
			Vector3 hitPointWorld = MathUtil.Interpolate3(ref m_rayFromWorld,ref m_rayToWorld,rayResult.m_hitFraction);
			m_hitPointWorld.Add(hitPointWorld);
			m_hitFractions.Add(rayResult.m_hitFraction);
			return m_closestHitFraction;
		}
	}




    ///RayResultCallback is used to report new raycast results
    public abstract class ConvexResultCallback
    {
        public ConvexResultCallback()
        {
            m_closestHitFraction = 1f;
            m_collisionFilterGroup = CollisionFilterGroups.DefaultFilter;
            m_collisionFilterMask = CollisionFilterGroups.AllFilter;
        }

        public bool HasHit()
        {
            return (m_closestHitFraction < 1f);
        }

        public virtual bool NeedsCollision(BroadphaseProxy proxy0)
        {
            bool collides = (proxy0.m_collisionFilterGroup & m_collisionFilterMask) != 0;
            collides = collides && ((m_collisionFilterGroup & proxy0.m_collisionFilterMask) != 0);
            return collides;
        }

        public abstract float AddSingleResult(LocalConvexResult convexResult, bool normalInWorldSpace);

        public float m_closestHitFraction;
        public CollisionFilterGroups m_collisionFilterGroup;
        public CollisionFilterGroups m_collisionFilterMask;

    };



    public class ClosestConvexResultCallback : ConvexResultCallback
    {
        public ClosestConvexResultCallback(Vector3 convexFromWorld, Vector3 convexToWorld)
            : this(ref convexFromWorld, ref convexToWorld)
        {
        }

        public ClosestConvexResultCallback(ref Vector3 convexFromWorld, ref Vector3 convexToWorld)
        {
            m_convexFromWorld = convexFromWorld;
            m_convexToWorld = convexToWorld;
            m_hitCollisionObject = null;
        }

        public override float AddSingleResult(LocalConvexResult convexResult, bool normalInWorldSpace)
        {
            //caller already does the filter on the m_closestHitFraction
            //btAssert(convexResult.m_hitFraction <= m_closestHitFraction);

            m_closestHitFraction = convexResult.m_hitFraction;
            m_hitCollisionObject = convexResult.m_hitCollisionObject;
            if (normalInWorldSpace)
            {
                m_hitNormalWorld = convexResult.m_hitNormalLocal;
            }
            else
            {
                ///need to transform normal into worldspace
                m_hitNormalWorld = Vector3.TransformNormal(convexResult.m_hitNormalLocal, m_hitCollisionObject.GetWorldTransform());
            }
            m_hitPointWorld = convexResult.m_hitPointLocal;
            return convexResult.m_hitFraction;
        }

        public Vector3 m_convexFromWorld;//used to calculate hitPointWorld from hitFraction
        public Vector3 m_convexToWorld;

        public Vector3 m_hitNormalWorld;
        public Vector3 m_hitPointWorld;
        public CollisionObject m_hitCollisionObject;
    }
    //ConvexCast::CastResult
    public class BridgeTriangleRaycastCallback : TriangleRaycastCallback
    {
        public RayResultCallback m_resultCallback;
        public CollisionObject m_collisionObject;
        public TriangleMeshShape m_triangleMesh;
        public Matrix m_colObjWorldTransform;

        public BridgeTriangleRaycastCallback(ref Vector3 from, ref Vector3 to,
            RayResultCallback resultCallback, CollisionObject collisionObject, TriangleMeshShape triangleMesh, ref Matrix colObjWorldTransform) :
            base(ref from, ref to, resultCallback.m_flags)
        {
            //@BP Mod
            m_resultCallback = resultCallback;
            m_collisionObject = collisionObject;
            m_triangleMesh = triangleMesh;
            m_colObjWorldTransform = colObjWorldTransform;
        }


        public override float ReportHit(ref Vector3 hitNormalLocal, float hitFraction, int partId, int triangleIndex)
        {
            LocalShapeInfo shapeInfo = new LocalShapeInfo();
            shapeInfo.m_shapePart = partId;
            shapeInfo.m_triangleIndex = triangleIndex;

            Vector3 hitNormalWorld = Vector3.TransformNormal(hitNormalLocal, m_colObjWorldTransform);

            LocalRayResult rayResult = new LocalRayResult
            (m_collisionObject,
                shapeInfo,
                ref hitNormalWorld,
                hitFraction);

            bool normalInWorldSpace = true;
            return m_resultCallback.AddSingleResult(rayResult, normalInWorldSpace);
        }

        public override void Cleanup()
        {
            base.Cleanup();
        }
    }


    public class BridgeTriangleConcaveRaycastCallback : TriangleRaycastCallback
    {

        public BridgeTriangleConcaveRaycastCallback(ref Vector3 from, ref Vector3 to,
            RayResultCallback resultCallback, CollisionObject collisionObject, ConcaveShape triangleMesh, ref Matrix colObjWorldTransform) :
            //@BP Mod
            base(ref from, ref to, resultCallback.m_flags)
        {
            m_resultCallback = resultCallback;
            m_collisionObject = collisionObject;
            m_triangleMesh = triangleMesh;
            m_colObjWorldTransform = colObjWorldTransform;
        }


        public override float ReportHit(ref Vector3 hitNormalLocal, float hitFraction, int partId, int triangleIndex)
        {
            LocalShapeInfo shapeInfo = new LocalShapeInfo();
            shapeInfo.m_shapePart = partId;
            shapeInfo.m_triangleIndex = triangleIndex;

            Vector3 hitNormalWorld = Vector3.TransformNormal(hitNormalLocal, m_colObjWorldTransform);

            LocalRayResult rayResult = new LocalRayResult
            (m_collisionObject,
                shapeInfo,
                ref hitNormalWorld,
                hitFraction);

            bool normalInWorldSpace = true;
            return m_resultCallback.AddSingleResult(rayResult, normalInWorldSpace);
        }


        public Matrix m_colObjWorldTransform;
        RayResultCallback m_resultCallback;
        CollisionObject m_collisionObject;
        ConcaveShape m_triangleMesh;
    }



    //ConvexCast::CastResult
    public class BridgeTriangleConvexcastCallback : TriangleConvexcastCallback
    {
        public BridgeTriangleConvexcastCallback(ConvexShape castShape, ref Matrix from, ref Matrix to,
            ConvexResultCallback resultCallback, CollisionObject collisionObject, TriangleMeshShape triangleMesh, ref Matrix triangleToWorld) :
            base(castShape, ref from, ref to, ref triangleToWorld, triangleMesh.GetMargin())
        {
            m_resultCallback = resultCallback;
            m_collisionObject = collisionObject;
            m_triangleMesh = triangleMesh;
        }

        public override float ReportHit(ref Vector3 hitNormalLocal, ref Vector3 hitPointLocal, float hitFraction, int partId, int triangleIndex)
        {
            LocalShapeInfo shapeInfo = new LocalShapeInfo();
            shapeInfo.m_shapePart = partId;
            shapeInfo.m_triangleIndex = triangleIndex;
            if (hitFraction <= m_resultCallback.m_closestHitFraction)
            {
                LocalConvexResult convexResult = new LocalConvexResult
                (m_collisionObject,
                    shapeInfo,
                    ref hitNormalLocal,
                    ref hitPointLocal,
                    hitFraction);

                bool normalInWorldSpace = true;
                return m_resultCallback.AddSingleResult(convexResult, normalInWorldSpace);
            }
            return hitFraction;
        }
        ConvexResultCallback m_resultCallback;
        CollisionObject m_collisionObject;
        TriangleMeshShape m_triangleMesh;
    };


    //ConvexCast::CastResult
    public class BridgeTriangleConvexcastCallback2 : TriangleConvexcastCallback
    {

        public BridgeTriangleConvexcastCallback2(ConvexShape castShape, ref Matrix from, ref Matrix to,
            ConvexResultCallback resultCallback, CollisionObject collisionObject, ConcaveShape triangleMesh, ref Matrix triangleToWorld) :
            base(castShape, ref from, ref to, ref triangleToWorld, triangleMesh.GetMargin())
        {
            m_resultCallback = resultCallback;
            m_collisionObject = collisionObject;
            m_triangleMesh = triangleMesh;
        }


        public override float ReportHit(ref Vector3 hitNormalLocal, ref Vector3 hitPointLocal, float hitFraction, int partId, int triangleIndex)
        {
            LocalShapeInfo shapeInfo = new LocalShapeInfo();
            shapeInfo.m_shapePart = partId;
            shapeInfo.m_triangleIndex = triangleIndex;
            if (hitFraction <= m_resultCallback.m_closestHitFraction)
            {
                LocalConvexResult convexResult = new LocalConvexResult
                (m_collisionObject,
                    shapeInfo,
                    ref hitNormalLocal,
                    ref hitPointLocal,
                    hitFraction);

                bool normalInWorldSpace = false;

                return m_resultCallback.AddSingleResult(convexResult, normalInWorldSpace);
            }
            return hitFraction;
        }
        ConvexResultCallback m_resultCallback;
        CollisionObject m_collisionObject;
        ConcaveShape m_triangleMesh;

    }



    public class SingleRayCallback : BroadphaseRayCallback
    {
        public SingleRayCallback(ref Vector3 rayFromWorld, ref Vector3 rayToWorld, CollisionWorld world, RayResultCallback resultCallback)
        {
            m_rayFromWorld = rayFromWorld;
            m_rayToWorld = rayToWorld;
            m_world = world;
            m_resultCallback = resultCallback;
            m_rayFromTrans = Matrix.Identity;
            m_rayFromTrans.Translation = m_rayFromWorld;
            m_rayToTrans = Matrix.Identity;
            m_rayToTrans.Translation = m_rayToWorld;

            Vector3 rayDir = (rayToWorld - rayFromWorld);

            rayDir.Normalize();
            ///what about division by zero? -. just set rayDirection[i] to INF/1e30
            m_rayDirectionInverse.X = MathUtil.FuzzyZero(rayDir.X) ? float.MaxValue : 1f / rayDir.X;
            m_rayDirectionInverse.Y = MathUtil.FuzzyZero(rayDir.Y) ? float.MaxValue : 1f / rayDir.Y;
            m_rayDirectionInverse.Z = MathUtil.FuzzyZero(rayDir.Z) ? float.MaxValue : 1f / rayDir.Z;
            m_signs[0] = m_rayDirectionInverse.X < 0.0f;
            m_signs[1] = m_rayDirectionInverse.Y < 0.0f;
            m_signs[2] = m_rayDirectionInverse.Z < 0.0f;

            m_lambda_max = Vector3.Dot(rayDir, (m_rayToWorld - m_rayFromWorld));
        }

        public override bool Process(BroadphaseProxy proxy)
        {
            ///terminate further ray tests, once the closestHitFraction reached zero
            if (MathUtil.FuzzyZero(m_resultCallback.m_closestHitFraction))
            {
                return false;
            }
            CollisionObject collisionObject = proxy.m_clientObject as CollisionObject;

            //only perform raycast if filterMask matches
            if (m_resultCallback.NeedsCollision(collisionObject.GetBroadphaseHandle()))
            {
                //RigidcollisionObject* collisionObject = ctrl.GetRigidcollisionObject();
                //Vector3 collisionObjectAabbMin,collisionObjectAabbMax;
                //#if 0
                //#ifdef RECALCULATE_AABB
                //            Vector3 collisionObjectAabbMin,collisionObjectAabbMax;
                //            collisionObject.getCollisionShape().getAabb(collisionObject.getWorldTransform(),collisionObjectAabbMin,collisionObjectAabbMax);
                //#else
                //getBroadphase().getAabb(collisionObject.getBroadphaseHandle(),collisionObjectAabbMin,collisionObjectAabbMax);
                Vector3 collisionObjectAabbMin = collisionObject.GetBroadphaseHandle().m_aabbMin;
                Vector3 collisionObjectAabbMax = collisionObject.GetBroadphaseHandle().m_aabbMax;
                //#endif
                //#endif
                //float hitLambda = m_resultCallback.m_closestHitFraction;
                //culling already done by broadphase
                //if (btRayAabb(m_rayFromWorld,m_rayToWorld,collisionObjectAabbMin,collisionObjectAabbMax,hitLambda,m_hitNormal))
                {
                    Matrix trans = collisionObject.GetWorldTransform();
                    CollisionWorld.RayTestSingle(ref m_rayFromTrans, ref m_rayToTrans,
                        collisionObject,
                            collisionObject.GetCollisionShape(),
                            ref trans,
                            m_resultCallback);
                }
            }
            return true;
        }
        Vector3 m_rayFromWorld;
        Vector3 m_rayToWorld;
        Matrix m_rayFromTrans;
        Matrix m_rayToTrans;
        Vector3 m_hitNormal;
        CollisionWorld m_world;
        RayResultCallback m_resultCallback;

    }




    public class SingleSweepCallback : BroadphaseRayCallback
    {

        Matrix m_convexFromTrans;
        Matrix m_convexToTrans;
        Vector3 m_hitNormal;
        CollisionWorld m_world;
        ConvexResultCallback m_resultCallback;
        float m_allowedCcdPenetration;
        ConvexShape m_castShape;


        public SingleSweepCallback(ConvexShape castShape, ref Matrix convexFromTrans, ref Matrix convexToTrans, CollisionWorld world, ConvexResultCallback resultCallback, float allowedPenetration)
        {
            m_convexFromTrans = convexFromTrans;
            m_convexToTrans = convexToTrans;
            m_world = world;
            m_resultCallback = resultCallback;
            m_allowedCcdPenetration = allowedPenetration;
            m_castShape = castShape;
            Vector3 unnormalizedRayDir = (m_convexToTrans.Translation - m_convexFromTrans.Translation);
            Vector3 rayDir = unnormalizedRayDir;
            rayDir.Normalize();
            ///what about division by zero? -. just set rayDirection[i] to INF/1e30
            m_rayDirectionInverse.X = MathUtil.CompareFloat(rayDir.X, 0.0f) ? float.MaxValue : 1f / rayDir.X;
            m_rayDirectionInverse.Y = MathUtil.CompareFloat(rayDir.Y, 0.0f) ? float.MaxValue : 1f / rayDir.Y;
            m_rayDirectionInverse.Z = MathUtil.CompareFloat(rayDir.Z, 0.0f) ? float.MaxValue : 1f / rayDir.Z;

            m_signs[0] = m_rayDirectionInverse.X < 0.0;
            m_signs[1] = m_rayDirectionInverse.Y < 0.0;
            m_signs[2] = m_rayDirectionInverse.Z < 0.0;

            m_lambda_max = Vector3.Dot(rayDir, unnormalizedRayDir);

        }

        public override bool Process(BroadphaseProxy proxy)
        {
            ///terminate further convex sweep tests, once the closestHitFraction reached zero
            if (MathUtil.FuzzyZero(m_resultCallback.m_closestHitFraction))
            {
                return false;
            }
            CollisionObject collisionObject = proxy.m_clientObject as CollisionObject;

            //only perform raycast if filterMask matches
            if (m_resultCallback.NeedsCollision(collisionObject.GetBroadphaseHandle()))
            {
                //RigidcollisionObject* collisionObject = ctrl.GetRigidcollisionObject();
                Matrix temp = collisionObject.GetWorldTransform();
                CollisionWorld.ObjectQuerySingle(m_castShape, ref m_convexFromTrans, ref m_convexToTrans,
                        collisionObject,
                            collisionObject.GetCollisionShape(),
                            ref temp,
                            m_resultCallback,
                            m_allowedCcdPenetration);
            }

            return true;
        }
    }


    ///ContactResultCallback is used to report contact points
    public abstract class ContactResultCallback
    {
        public CollisionFilterGroups m_collisionFilterGroup;
        public CollisionFilterGroups m_collisionFilterMask;

        public ContactResultCallback()
        {
            m_collisionFilterGroup = CollisionFilterGroups.DefaultFilter;
            m_collisionFilterMask = CollisionFilterGroups.AllFilter;
        }

        public virtual bool NeedsCollision(BroadphaseProxy proxy0)
        {
            bool collides = (proxy0.m_collisionFilterGroup & m_collisionFilterMask) != 0;
            collides = collides && ((m_collisionFilterGroup & proxy0.m_collisionFilterMask) != 0);
            return collides;
        }

        public abstract float AddSingleResult(ref ManifoldPoint cp, CollisionObject colObj0, int partId0, int index0, CollisionObject colObj1, int partId1, int index1);
    }


    public class LocalInfoAdder : ConvexResultCallback
    {
        public ConvexResultCallback m_userCallback;
        public int m_i;

        public LocalInfoAdder(int i, ConvexResultCallback user)
        {
            m_userCallback = user;
            m_i = i;
            m_closestHitFraction = user.m_closestHitFraction;
        }

        public override bool  NeedsCollision(BroadphaseProxy proxy0)
{
 	 return m_userCallback.NeedsCollision(proxy0);
}

        public override float AddSingleResult(LocalConvexResult r, bool b)
        {
            LocalShapeInfo shapeInfo = new LocalShapeInfo();
            shapeInfo.m_shapePart = -1;
            shapeInfo.m_triangleIndex = m_i;
            if (r.m_localShapeInfo == null)
            {
                r.m_localShapeInfo = shapeInfo;
            }
            float result = m_userCallback.AddSingleResult(r, b);
            m_closestHitFraction = m_userCallback.m_closestHitFraction;
            return result;
        }
    }



    public class LocalInfoAdder2 : RayResultCallback
    {
        public int m_i;
        public RayResultCallback m_userCallback;
        public LocalInfoAdder2(int i, RayResultCallback user)
        {
            m_i = i;
            m_userCallback = user;
            m_closestHitFraction = user.m_closestHitFraction;
        }

        public override float AddSingleResult(LocalRayResult r, bool b)
        {
            LocalShapeInfo shapeInfo = new LocalShapeInfo();
            shapeInfo.m_shapePart = -1;
            shapeInfo.m_triangleIndex = m_i;
            if (r.m_localShapeInfo == null)
            {
                r.m_localShapeInfo = shapeInfo;
            }
            float result = m_userCallback.AddSingleResult(r, b);
            m_closestHitFraction = m_userCallback.m_closestHitFraction;

            return result;
        }

        public override bool NeedsCollision(BroadphaseProxy proxy0)
        {
            return m_userCallback.NeedsCollision(proxy0);
        }

        public virtual void cleanup()
        {
        }

    };


    public class BridgedManifoldResult : ManifoldResult
    {
        ContactResultCallback m_resultCallback;

        public BridgedManifoldResult(CollisionObject obj0, CollisionObject obj1, ContactResultCallback resultCallback)
            : base(obj0, obj1)
        {
            m_resultCallback = resultCallback;
        }

        public override void AddContactPoint(ref Vector3 normalOnBInWorld, ref Vector3 pointInWorld, float depth)
        {
            bool isSwapped = m_manifoldPtr.GetBody0() != m_body0;
            Vector3 pointA = pointInWorld + normalOnBInWorld * depth;
            Vector3 localA;
            Vector3 localB;
            if (isSwapped)
            {
                localA = MathUtil.InverseTransform(ref m_rootTransB, ref pointA);
                localB = MathUtil.InverseTransform(ref m_rootTransA, ref pointInWorld);
            }
            else
            {
                localA = MathUtil.InverseTransform(ref m_rootTransA, ref pointA);
                localB = MathUtil.InverseTransform(ref m_rootTransB, ref pointInWorld);
            }

            ManifoldPoint newPt = new ManifoldPoint(ref localA, ref localB, ref normalOnBInWorld, depth);
            newPt.m_positionWorldOnA = pointA;
            newPt.m_positionWorldOnB = pointInWorld;

            //BP mod, store contact triangles.
            if (isSwapped)
            {
                newPt.m_partId0 = m_partId1;
                newPt.m_partId1 = m_partId0;
                newPt.m_index0 = m_index1;
                newPt.m_index1 = m_index0;
            }
            else
            {
                newPt.m_partId0 = m_partId0;
                newPt.m_partId1 = m_partId1;
                newPt.m_index0 = m_index0;
                newPt.m_index1 = m_index1;
            }

            //experimental feature info, for per-triangle material etc.
            CollisionObject obj0 = isSwapped ? m_body1 : m_body0;
            CollisionObject obj1 = isSwapped ? m_body0 : m_body1;
            m_resultCallback.AddSingleResult(ref newPt, obj0, newPt.m_partId0, newPt.m_index0, obj1, newPt.m_partId1, newPt.m_index1);
        }
    }



    public class SingleContactCallback : IBroadphaseAabbCallback
    {
        CollisionObject m_collisionObject;
        CollisionWorld m_world;
        ContactResultCallback m_resultCallback;

        public SingleContactCallback(CollisionObject collisionObject, CollisionWorld world, ContactResultCallback resultCallback)
        {
            m_collisionObject = collisionObject;
            m_world = world;
            m_resultCallback = resultCallback;
        }

        public virtual void Cleanup()
        {
        }

        public virtual bool Process(BroadphaseProxy proxy)
        {
            if (proxy.m_clientObject == m_collisionObject)
            {
                return true;
            }
            CollisionObject collisionObject = proxy.m_clientObject as CollisionObject;

            //only perform raycast if filterMask matches
            if (m_resultCallback.NeedsCollision(collisionObject.GetBroadphaseHandle()))
            {
                CollisionAlgorithm algorithm = m_world.GetDispatcher().FindAlgorithm(m_collisionObject, collisionObject);
                if (algorithm != null)
                {
                    BridgedManifoldResult contactPointResult = new BridgedManifoldResult(m_collisionObject, collisionObject, m_resultCallback);
                    //discrete collision detection query
                    algorithm.ProcessCollision(m_collisionObject, collisionObject, m_world.GetDispatchInfo(), contactPointResult);

                    algorithm.Cleanup();
                    m_world.GetDispatcher().FreeCollisionAlgorithm(algorithm);
                }
            }
            return true;
        }
    }



    public class DebugDrawcallback : ITriangleCallback, IInternalTriangleIndexCallback
    {
        public IDebugDraw m_debugDrawer;
        public Vector3 m_color;
        public Matrix m_worldTrans;

        public DebugDrawcallback(IDebugDraw debugDrawer, ref Matrix worldTrans, ref Vector3 color)
        {
            m_debugDrawer = debugDrawer;
            m_color = color;
            m_worldTrans = worldTrans;
        }

        public virtual bool graphics()
        {
            return true;
        }

        public virtual void InternalProcessTriangleIndex(Vector3[] triangle, int partId, int triangleIndex)
        {
            ProcessTriangle(triangle, partId, triangleIndex);
        }

        public virtual void ProcessTriangle(Vector3[] triangle, int partId, int triangleIndex)
        {
            //(voidpartId;
            //(void)triangleIndex;

            Vector3 wv0, wv1, wv2;
            Vector3.Transform(ref triangle[0], ref m_worldTrans, out wv0);
            Vector3.Transform(ref triangle[1], ref m_worldTrans, out wv1);
            Vector3.Transform(ref triangle[2], ref m_worldTrans, out wv2);
            Vector3 center = (wv0 + wv1 + wv2) * (1f / 3f);

            Vector3 normal = Vector3.Cross((wv1 - wv0), (wv2 - wv0));
            normal.Normalize();
            Vector3 normalColor = new Vector3(1, 1, 0);
            m_debugDrawer.DrawLine(center, center + normal, normalColor);

            m_debugDrawer.DrawLine(ref wv0, ref wv1, ref m_color);
            m_debugDrawer.DrawLine(ref wv1, ref wv2, ref m_color);
            m_debugDrawer.DrawLine(ref wv2, ref wv0, ref m_color);
        }
        public void Cleanup()
        {
        }
    }


    public class RayTester : Collide
    {
        public CollisionObject m_collisionObject;
        public CompoundShape m_compoundShape;
        public Matrix m_colObjWorldTransform;
        public Matrix m_rayFromTrans;
        public Matrix m_rayToTrans;
        public RayResultCallback m_resultCallback;

        public RayTester(CollisionObject collisionObject,
                CompoundShape compoundShape,
                ref Matrix colObjWorldTransform,
                ref Matrix rayFromTrans,
                ref Matrix rayToTrans,
                RayResultCallback resultCallback)
        {
            m_collisionObject = collisionObject;
            m_compoundShape = compoundShape;
            m_colObjWorldTransform = colObjWorldTransform;
            m_rayFromTrans = rayFromTrans;
            m_rayToTrans = rayToTrans;
            m_resultCallback = resultCallback;
        }

        public void Process(int i)
        {
            CollisionShape childCollisionShape = m_compoundShape.GetChildShape(i);
            Matrix childTrans = m_compoundShape.GetChildTransform(i);
            Matrix childWorldTrans = MathUtil.BulletMatrixMultiply(ref m_colObjWorldTransform, ref childTrans);

            // replace collision shape so that callback can determine the triangle
            CollisionShape saveCollisionShape = m_collisionObject.GetCollisionShape();
            m_collisionObject.InternalSetTemporaryCollisionShape(childCollisionShape);

            LocalInfoAdder2 my_cb = new LocalInfoAdder2(i, m_resultCallback);

            CollisionWorld.RayTestSingle(
                ref m_rayFromTrans,
                ref m_rayToTrans,
                m_collisionObject,
                childCollisionShape,
                ref childWorldTrans,
                my_cb);

            // restore
            m_collisionObject.InternalSetTemporaryCollisionShape(saveCollisionShape);
        }

        public override void Process(DbvtNode leaf)
        {
            Process(leaf.dataAsInt);
        }

        public virtual void Cleanup()
        {

        }
    }



}