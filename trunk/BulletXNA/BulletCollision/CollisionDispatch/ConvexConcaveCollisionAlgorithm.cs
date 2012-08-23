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

using BulletXNA.LinearMath;
using System;

namespace BulletXNA.BulletCollision
{
    /// btConvexConcaveCollisionAlgorithm  supports collision between convex shapes and (concave) trianges meshes.

    public class ConvexConcaveCollisionAlgorithm : ActivatingCollisionAlgorithm
    {
        public ConvexConcaveCollisionAlgorithm() { } // for pool
        public ConvexConcaveCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1, bool isSwapped)
            : base(ci, body0, body1)
        {
            m_isSwapped = isSwapped;
            m_convexTriangleCallback = new ConvexTriangleCallback(m_dispatcher, body0, body1, isSwapped);
        }

        public void Inititialize(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1, bool isSwapped)
        {
            base.Initialize(ci, body0, body1);
            m_isSwapped = isSwapped;
            if (m_convexTriangleCallback == null)
            {
                m_convexTriangleCallback = new ConvexTriangleCallback(m_dispatcher, body0, body1, isSwapped);
            }
            else
            {
                m_convexTriangleCallback.Initialize(m_dispatcher, body0, body1, isSwapped);
            }

        }
        public override void Cleanup()
        {
            // empty on purpose...
            base.Cleanup();
            if (m_convexTriangleCallback != null)
            {
                m_convexTriangleCallback.Cleanup();
                // pool means we don't clear this
                //m_convexTriangleCallback = null;
            }
            BulletGlobals.ConvexConcaveCollisionAlgorithmPool.Free(this);
        }

        //public override void processCollision (CollisionObject body0,CollisionObject body1,DispatcherInfo dispatchInfo,ManifoldResult resultOut)
        //{
        //    CollisionObject convexBody = m_isSwapped ? body1 : body0;
        //    CollisionObject triBody = m_isSwapped ? body0 : body1;

        //    if (triBody.getCollisionShape().isConcave())
        //    {
        //        CollisionObject triOb = triBody;
        //        ConcaveShape concaveShape = (ConcaveShape)(triOb.getCollisionShape());

        //        if (convexBody.getCollisionShape().isConvex())
        //        {
        //            float collisionMarginTriangle = concaveShape.getMargin();

        //            resultOut.setPersistentManifold(m_convexTriangleCallback.m_manifoldPtr);
        //            m_convexTriangleCallback.setTimeStepAndCounters(collisionMarginTriangle, dispatchInfo, resultOut);

        //            //Disable persistency. previously, some older algorithm calculated all contacts in one go, so you can clear it here.
        //            //m_dispatcher.clearManifold(m_convexTriangleCallback.m_manifoldPtr);

        //            m_convexTriangleCallback.m_manifoldPtr.setBodies(convexBody, triBody);

        //            IndexedVector3 min = m_convexTriangleCallback.getAabbMin();
        //            IndexedVector3 max = m_convexTriangleCallback.getAabbMax();

        //            concaveShape.processAllTriangles(m_convexTriangleCallback, ref min,ref max );

        //            resultOut.refreshContactPoints();

        //        }
        //    }
        //}


        //public override float calculateTimeOfImpact(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        //{
        //    CollisionObject convexbody = m_isSwapped ? body1 : body0;
        //    CollisionObject triBody = m_isSwapped ? body0 : body1;

        //    //quick approximation using raycast, todo: hook up to the continuous collision detection (one of the btConvexCast)

        //    //only perform CCD above a certain threshold, this prevents blocking on the long run
        //    //because object in a blocked ccd state (hitfraction<1) get their linear velocity halved each frame...
        //    float squareMot0 = (convexbody.getInterpolationWorldTransform()._origin - convexbody.getWorldTransform()._origin).LengthSquared();
        //    if (squareMot0 < convexbody.getCcdSquareMotionThreshold())
        //    {
        //        return 1f;
        //    }

        //    //const IndexedVector3& from = convexbody.m_worldTransform._origin;
        //    //IndexedVector3 to = convexbody.m_interpolationWorldTransform._origin;
        //    //todo: only do if the motion exceeds the 'radius'

        //    //IndexedMatrix triInv = IndexedMatrix.Invert(triBody.getWorldTransform());
        //    //IndexedMatrix convexFromLocal = MathUtil.bulletMatrixMultiply(triInv , convexbody.getWorldTransform());
        //    //IndexedMatrix convexToLocal = MathUtil.bulletMatrixMultiply(triInv , convexbody.getInterpolationWorldTransform());

        //    IndexedMatrix triInv = IndexedMatrix.Invert(triBody.getWorldTransform());
        //    IndexedMatrix convexFromLocal = MathUtil.inverseTimes(triBody.getWorldTransform(), convexbody.getWorldTransform());
        //    IndexedMatrix convexToLocal = MathUtil.inverseTimes(triBody.getWorldTransform(), convexbody.getInterpolationWorldTransform());

        //    if (triBody.getCollisionShape().isConcave())
        //    {
        //        IndexedVector3 rayAabbMin = convexFromLocal._origin;
        //        MathUtil.vectorMin(convexToLocal._origin, ref rayAabbMin);
        //        IndexedVector3 rayAabbMax = convexFromLocal._origin;
        //        MathUtil.vectorMax(convexToLocal._origin,ref rayAabbMax);
        //        float ccdRadius0 = convexbody.getCcdSweptSphereRadius();
        //        rayAabbMin -= new IndexedVector3(ccdRadius0,ccdRadius0,ccdRadius0);
        //        rayAabbMax += new IndexedVector3(ccdRadius0,ccdRadius0,ccdRadius0);

        //        float curHitFraction = 1.0f; //is this available?
        //        LocalTriangleSphereCastCallback raycastCallback = new LocalTriangleSphereCastCallback(ref convexFromLocal, ref convexToLocal,
        //            convexbody.getCcdSweptSphereRadius(),curHitFraction);

        //        raycastCallback.m_hitFraction = convexbody.getHitFraction();

        //        CollisionObject concavebody = triBody;

        //        ConcaveShape triangleMesh = (ConcaveShape) concavebody.getCollisionShape();

        //        if (triangleMesh != null)
        //        {
        //            triangleMesh.processAllTriangles(raycastCallback,ref rayAabbMin,ref rayAabbMax);
        //        }

        //        if (raycastCallback.m_hitFraction < convexbody.getHitFraction())
        //        {
        //            convexbody.setHitFraction( raycastCallback.m_hitFraction);
        //            float result = raycastCallback.m_hitFraction;
        //            raycastCallback.cleanup();
        //            return result;
        //        }

        //        raycastCallback.cleanup();
        //    }
        //    return 1f;
        //}

        public override void ProcessCollision(CollisionObject bodyA, CollisionObject bodyB, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {


            //fixme

            CollisionObject convexBody = m_isSwapped ? bodyB : bodyA;
            CollisionObject triBody = m_isSwapped ? bodyA : bodyB;

            if (triBody.GetCollisionShape().IsConcave())
            {
                CollisionObject triOb = triBody;
                ConcaveShape concaveShape = triOb.GetCollisionShape() as ConcaveShape;

                if (convexBody.GetCollisionShape().IsConvex())
                {
                    float collisionMarginTriangle = concaveShape.GetMargin();

                    resultOut.SetPersistentManifold(m_convexTriangleCallback.m_manifoldPtr);
                    m_convexTriangleCallback.SetTimeStepAndCounters(collisionMarginTriangle, dispatchInfo, resultOut);

                    //Disable persistency. previously, some older algorithm calculated all contacts in one go, so you can clear it here.
                    //m_dispatcher->clearManifold(m_btConvexTriangleCallback.m_manifoldPtr);

                    m_convexTriangleCallback.m_manifoldPtr.SetBodies(convexBody, triBody);
                    IndexedVector3 min = m_convexTriangleCallback.GetAabbMin();
                    IndexedVector3 max = m_convexTriangleCallback.GetAabbMax();

                    concaveShape.ProcessAllTriangles(m_convexTriangleCallback, ref min, ref max);
                    resultOut.RefreshContactPoints();
                }
            }
        }

        public override float CalculateTimeOfImpact(CollisionObject bodyA, CollisionObject bodyB, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            CollisionObject convexbody = m_isSwapped ? bodyB : bodyA;
            CollisionObject triBody = m_isSwapped ? bodyA : bodyB;


            //quick approximation using raycast, todo: hook up to the continuous collision detection (one of the btConvexCast)

            //only perform CCD above a certain threshold, this prevents blocking on the long run
            //because object in a blocked ccd state (hitfraction<1) get their linear velocity halved each frame...
            float squareMot0 = (convexbody.GetInterpolationWorldTransform()._origin - convexbody.GetWorldTransform()._origin).LengthSquared();
            if (squareMot0 < convexbody.GetCcdSquareMotionThreshold())
            {
                return 1;
            }

            //IndexedMatrix triInv = MathHelper.InvertMatrix(triBody.getWorldTransform());
            IndexedMatrix triInv = triBody.GetWorldTransform().Inverse();

            IndexedMatrix convexFromLocal = triInv * convexbody.GetWorldTransform();
            IndexedMatrix convexToLocal = triInv * convexbody.GetInterpolationWorldTransform();

            if (triBody.GetCollisionShape().IsConcave())
            {
                IndexedVector3 rayAabbMin = convexFromLocal._origin;
                MathUtil.VectorMin(convexToLocal._origin, ref rayAabbMin);
                IndexedVector3 rayAabbMax = convexFromLocal._origin;
                MathUtil.VectorMax(convexToLocal._origin, ref rayAabbMax);
                IndexedVector3 ccdRadius0 = new IndexedVector3(convexbody.GetCcdSweptSphereRadius());
                rayAabbMin -= ccdRadius0;
                rayAabbMax += ccdRadius0;

                float curHitFraction = 1f; //is this available?
                using (LocalTriangleSphereCastCallback raycastCallback = BulletGlobals.LocalTriangleSphereCastCallbackPool.Get())
                {
                    raycastCallback.Initialize(ref convexFromLocal, ref convexToLocal,
                    convexbody.GetCcdSweptSphereRadius(), curHitFraction);

                    raycastCallback.m_hitFraction = convexbody.GetHitFraction();

                    CollisionObject concavebody = triBody;

                    ConcaveShape triangleMesh = concavebody.GetCollisionShape() as ConcaveShape;

                    if (triangleMesh != null)
                    {
                        triangleMesh.ProcessAllTriangles(raycastCallback, ref rayAabbMin, ref rayAabbMax);
                    }

                    if (raycastCallback.m_hitFraction < convexbody.GetHitFraction())
                    {
                        convexbody.SetHitFraction(raycastCallback.m_hitFraction);
                        return raycastCallback.m_hitFraction;
                    }
                }
            }
            return 1;
        }



        public override void GetAllContactManifolds(PersistentManifoldArray manifoldArray)
        {
            if (m_convexTriangleCallback.m_manifoldPtr != null)
            {
                manifoldArray.Add(m_convexTriangleCallback.m_manifoldPtr);
            }
        }

        public void ClearCache()
        {
            m_convexTriangleCallback.ClearCache();
        }

        private bool m_isSwapped;
        private ConvexTriangleCallback m_convexTriangleCallback;
    }


    public class LocalTriangleSphereCastCallback : ITriangleCallback,IDisposable
    {
        public IndexedMatrix m_ccdSphereFromTrans;
        public IndexedMatrix m_ccdSphereToTrans;
        public IndexedMatrix m_meshTransform;

        public float m_ccdSphereRadius;
        public float m_hitFraction;

        public virtual bool graphics()
        {
            return false;
        }
        public LocalTriangleSphereCastCallback() { } // for pool

        public LocalTriangleSphereCastCallback(ref IndexedMatrix from, ref IndexedMatrix to, float ccdSphereRadius, float hitFraction)
        {
            m_ccdSphereFromTrans = from;
            m_ccdSphereToTrans = to;
            m_ccdSphereRadius = ccdSphereRadius;
            m_hitFraction = hitFraction;
        }

        public void Initialize(ref IndexedMatrix from, ref IndexedMatrix to, float ccdSphereRadius, float hitFraction)
        {
            m_ccdSphereFromTrans = from;
            m_ccdSphereToTrans = to;
            m_ccdSphereRadius = ccdSphereRadius;
            m_hitFraction = hitFraction;
        }

        public virtual void Cleanup()
        {
            
        }

        public void ProcessTriangle(IndexedVector3[] triangle, int partId, int triangleIndex)
        {
            //do a swept sphere for now
            IndexedMatrix ident = IndexedMatrix.Identity;
            CastResult castResult = BulletGlobals.CastResultPool.Get();
            castResult.m_fraction = m_hitFraction;
            SphereShape pointShape = BulletGlobals.SphereShapePool.Get();
            pointShape.Initialize(m_ccdSphereRadius);
            using (TriangleShape triShape = BulletGlobals.TriangleShapePool.Get())
            {
                triShape.Initialize(ref triangle[0], ref triangle[1], ref triangle[2]);
                VoronoiSimplexSolver simplexSolver = BulletGlobals.VoronoiSimplexSolverPool.Get();
                SubSimplexConvexCast convexCaster = BulletGlobals.SubSimplexConvexCastPool.Get();
                convexCaster.Initialize(pointShape, triShape, simplexSolver);
                //GjkConvexCast	convexCaster(&pointShape,convexShape,&simplexSolver);
                //ContinuousConvexCollision convexCaster(&pointShape,convexShape,&simplexSolver,0);
                //local space?

                if (convexCaster.CalcTimeOfImpact(ref m_ccdSphereFromTrans, ref m_ccdSphereToTrans,
                    ref ident, ref ident, castResult))
                {
                    if (m_hitFraction > castResult.m_fraction)
                    {
                        m_hitFraction = castResult.m_fraction;
                    }
                }
                BulletGlobals.SubSimplexConvexCastPool.Free(convexCaster);
                BulletGlobals.VoronoiSimplexSolverPool.Free(simplexSolver);
                BulletGlobals.SphereShapePool.Free(pointShape);
                castResult.Cleanup();
            }
        }


        #region IDisposable Members

        public void Dispose()
        {
            BulletGlobals.LocalTriangleSphereCastCallbackPool.Free(this);
        }

        #endregion
    };

    public class ConvexTriangleCallback : ITriangleCallback
    {
        private CollisionObject m_convexBody;
        private CollisionObject m_triBody;

        private IndexedVector3 m_aabbMin;
        private IndexedVector3 m_aabbMax;

        private ManifoldResult m_resultOut;
        private IDispatcher m_dispatcher;
        private DispatcherInfo m_dispatchInfoPtr;
        private float m_collisionMarginTriangle;

        public int m_triangleCount;

        public PersistentManifold m_manifoldPtr;


        public virtual bool graphics()
        {
            return false;
        }

        public ConvexTriangleCallback(IDispatcher dispatcher, CollisionObject body0, CollisionObject body1, bool isSwapped)
        {
            m_dispatcher = dispatcher;
            m_convexBody = isSwapped ? body1 : body0;
            m_triBody = isSwapped ? body0 : body1;
            m_manifoldPtr = m_dispatcher.GetNewManifold(m_convexBody, m_triBody);
            ClearCache();
        }

        public void Initialize(IDispatcher dispatcher, CollisionObject body0, CollisionObject body1, bool isSwapped)
        {
            m_dispatcher = dispatcher;
            m_convexBody = isSwapped ? body1 : body0;
            m_triBody = isSwapped ? body0 : body1;
            m_manifoldPtr = m_dispatcher.GetNewManifold(m_convexBody, m_triBody);
            ClearCache();
        }


        public virtual void Cleanup()
        {
            ClearCache();
            m_dispatcher.ReleaseManifold(m_manifoldPtr);
        }


        public void SetTimeStepAndCounters(float collisionMarginTriangle, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            m_dispatchInfoPtr = dispatchInfo;
            m_collisionMarginTriangle = collisionMarginTriangle;
            m_resultOut = resultOut;

            //recalc aabbs
            //IndexedMatrix convexInTriangleSpace = MathUtil.bulletMatrixMultiply(IndexedMatrix.Invert(m_triBody.getWorldTransform()) , m_convexBody.getWorldTransform());
            IndexedMatrix convexInTriangleSpace = m_triBody.GetWorldTransform().Inverse() * m_convexBody.GetWorldTransform();
            CollisionShape convexShape = m_convexBody.GetCollisionShape();
            convexShape.GetAabb(ref convexInTriangleSpace, out m_aabbMin, out m_aabbMax);
            float extraMargin = collisionMarginTriangle;
            IndexedVector3 extra = new IndexedVector3(extraMargin);

            m_aabbMax += extra;
            m_aabbMin -= extra;
        }

        public virtual void ProcessTriangle(IndexedVector3[] triangle, int partId, int triangleIndex)
        {
            //aabb filter is already applied!	
            CollisionAlgorithmConstructionInfo ci = new CollisionAlgorithmConstructionInfo();
            ci.SetDispatcher(m_dispatcher);

            CollisionObject ob = m_triBody as CollisionObject;

            ///debug drawing of the overlapping triangles
            ///
#if false
            if (m_dispatchInfoPtr != null && m_dispatchInfoPtr.getDebugDraw() != null && ((m_dispatchInfoPtr.getDebugDraw().GetDebugMode() & DebugDrawModes.DBG_DrawWireframe) > 0))
            {
                IndexedVector3 color = new IndexedVector3(1, 1, 0);
                IndexedMatrix tr = ob.GetWorldTransform();

                IndexedVector3[] transformedTriangles = new IndexedVector3[3];
                IndexedVector3.Transform(triangle, ref tr, transformedTriangles);

                m_dispatchInfoPtr.getDebugDraw().DrawLine(ref transformedTriangles[0], ref transformedTriangles[1], ref color);
                m_dispatchInfoPtr.getDebugDraw().DrawLine(ref transformedTriangles[1], ref transformedTriangles[2], ref color);
                m_dispatchInfoPtr.getDebugDraw().DrawLine(ref transformedTriangles[2], ref transformedTriangles[0], ref color);

            }
#endif
            if (m_convexBody.GetCollisionShape().IsConvex())
            {
                using (TriangleShape tm = BulletGlobals.TriangleShapePool.Get())
                {
                    tm.Initialize(ref triangle[0], ref triangle[1], ref triangle[2]);
                    tm.SetMargin(m_collisionMarginTriangle);

                    CollisionShape tmpShape = ob.GetCollisionShape();
                    ob.InternalSetTemporaryCollisionShape(tm);

                    CollisionAlgorithm colAlgo = ci.GetDispatcher().FindAlgorithm(m_convexBody, m_triBody, m_manifoldPtr);
                    ///this should use the btDispatcher, so the actual registered algorithm is used
                    //		btConvexConvexAlgorithm cvxcvxalgo(m_manifoldPtr,ci,m_convexBody,m_triBody);

                    if (m_resultOut.GetBody0Internal() == m_triBody)
                    {
                        m_resultOut.SetShapeIdentifiersA(partId, triangleIndex);
                    }
                    else
                    {
                        m_resultOut.SetShapeIdentifiersB(partId, triangleIndex);
                    }

                    colAlgo.ProcessCollision(m_convexBody, m_triBody, m_dispatchInfoPtr, m_resultOut);
                    ci.GetDispatcher().FreeCollisionAlgorithm(colAlgo);
                    colAlgo = null;

                    ob.InternalSetTemporaryCollisionShape(tmpShape);
                }
            }
        }

        public void ClearCache()
        {
            m_dispatcher.ClearManifold(m_manifoldPtr);
        }

        public IndexedVector3 GetAabbMin()
        {
            return m_aabbMin;
        }
        public IndexedVector3 GetAabbMax()
        {
            return m_aabbMax;
        }

    }


    public class ConvexConcaveCreateFunc : CollisionAlgorithmCreateFunc
    {
        public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
        {
            ConvexConcaveCollisionAlgorithm algo = BulletGlobals.ConvexConcaveCollisionAlgorithmPool.Get();
            algo.Inititialize(ci, body0, body1, false);
            return algo;
        }
    }

    public class SwappedConvexConcaveCreateFunc : CollisionAlgorithmCreateFunc
    {
        public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
        {
            ConvexConcaveCollisionAlgorithm algo = BulletGlobals.ConvexConcaveCollisionAlgorithmPool.Get();
            algo.Inititialize(ci, body0, body1, true);
            return algo;
        }
    }

}
