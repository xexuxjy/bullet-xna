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

using System.Collections.Generic;
using BulletXNA.BulletCollision.BroadphaseCollision;
using BulletXNA.BulletCollision.CollisionShapes;
using BulletXNA.BulletCollision.NarrowPhaseCollision;
using BulletXNA.LinearMath;
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletCollision.CollisionDispatch
{
    /// btConvexConcaveCollisionAlgorithm  supports collision between convex shapes and (concave) trianges meshes.

    public class ConvexConcaveCollisionAlgorithm : ActivatingCollisionAlgorithm
    {
	    public ConvexConcaveCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci,CollisionObject body0,CollisionObject body1,bool isSwapped) : base(ci,body0,body1)
        {
            m_isSwapped = isSwapped;
            m_convexTriangleCallback = new ConvexTriangleCallback(m_dispatcher, body0, body1, isSwapped);
        }

        public override void Cleanup()
        {
            // empty on purpose...
            base.Cleanup();
            if (m_convexTriangleCallback != null)
            {
                m_convexTriangleCallback.Cleanup();
                m_convexTriangleCallback = null;
            }
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

        //            Vector3 min = m_convexTriangleCallback.getAabbMin();
        //            Vector3 max = m_convexTriangleCallback.getAabbMax();

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
        //    float squareMot0 = (convexbody.getInterpolationWorldTransform().Translation - convexbody.getWorldTransform().Translation).LengthSquared();
        //    if (squareMot0 < convexbody.getCcdSquareMotionThreshold())
        //    {
        //        return 1f;
        //    }

        //    //const Vector3& from = convexbody.m_worldTransform.Translation;
        //    //Vector3 to = convexbody.m_interpolationWorldTransform.Translation;
        //    //todo: only do if the motion exceeds the 'radius'

        //    //Matrix triInv = Matrix.Invert(triBody.getWorldTransform());
        //    //Matrix convexFromLocal = MathUtil.bulletMatrixMultiply(triInv , convexbody.getWorldTransform());
        //    //Matrix convexToLocal = MathUtil.bulletMatrixMultiply(triInv , convexbody.getInterpolationWorldTransform());

        //    Matrix triInv = Matrix.Invert(triBody.getWorldTransform());
        //    Matrix convexFromLocal = MathUtil.inverseTimes(triBody.getWorldTransform(), convexbody.getWorldTransform());
        //    Matrix convexToLocal = MathUtil.inverseTimes(triBody.getWorldTransform(), convexbody.getInterpolationWorldTransform());

        //    if (triBody.getCollisionShape().isConcave())
        //    {
        //        Vector3 rayAabbMin = convexFromLocal.Translation;
        //        MathUtil.vectorMin(convexToLocal.Translation, ref rayAabbMin);
        //        Vector3 rayAabbMax = convexFromLocal.Translation;
        //        MathUtil.vectorMax(convexToLocal.Translation,ref rayAabbMax);
        //        float ccdRadius0 = convexbody.getCcdSweptSphereRadius();
        //        rayAabbMin -= new Vector3(ccdRadius0,ccdRadius0,ccdRadius0);
        //        rayAabbMax += new Vector3(ccdRadius0,ccdRadius0,ccdRadius0);

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
                    Vector3 min = m_convexTriangleCallback.GetAabbMin();
                    Vector3 max = m_convexTriangleCallback.GetAabbMax();

                    concaveShape.ProcessAllTriangles(m_convexTriangleCallback, ref min,ref max);
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
            float squareMot0 = (convexbody.GetInterpolationWorldTransform().Translation - convexbody.GetWorldTransform().Translation).LengthSquared();
            if (squareMot0 < convexbody.GetCcdSquareMotionThreshold())
            {
                return 1;
            }

            //Matrix triInv = MathHelper.InvertMatrix(triBody.getWorldTransform());
            Matrix triInv = Matrix.Invert(triBody.GetWorldTransform());

            Matrix convexFromLocal = triInv * convexbody.GetWorldTransform();
            Matrix convexToLocal = triInv * convexbody.GetInterpolationWorldTransform();

            if (triBody.GetCollisionShape().IsConcave())
            {
                Vector3 rayAabbMin = convexFromLocal.Translation;
                MathUtil.VectorMin(convexToLocal.Translation ,ref rayAabbMin);
                Vector3 rayAabbMax = convexFromLocal.Translation;
                MathUtil.VectorMax(convexToLocal.Translation,ref rayAabbMax);
                float ccdRadius0 = convexbody.GetCcdSweptSphereRadius();
                rayAabbMin -= new Vector3(ccdRadius0, ccdRadius0, ccdRadius0);
                rayAabbMax += new Vector3(ccdRadius0, ccdRadius0, ccdRadius0);

                float curHitFraction = 1f; //is this available?
                LocalTriangleSphereCastCallback raycastCallback = new LocalTriangleSphereCastCallback(ref convexFromLocal, ref convexToLocal,
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

            return 1;
        }



        public override void GetAllContactManifolds(IList<PersistentManifold> manifoldArray)
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


    	public class LocalTriangleSphereCastCallback : ITriangleCallback
	    {
		    public Matrix m_ccdSphereFromTrans;
            public Matrix m_ccdSphereToTrans;
            public Matrix m_meshTransform;

            public float m_ccdSphereRadius;
            public float m_hitFraction;

            public virtual bool graphics()
            {
                return false;
            }

		    public LocalTriangleSphereCastCallback(ref Matrix from,ref Matrix to,float ccdSphereRadius,float hitFraction)
		    {
		        m_ccdSphereFromTrans = from;
                m_ccdSphereToTrans = to;
                m_ccdSphereRadius = ccdSphereRadius;
                m_hitFraction = hitFraction;
		    }

            public virtual void Cleanup()
            {
            }

            public void ProcessTriangle(Vector3[] triangle, int partId, int triangleIndex)
		    {
			    //do a swept sphere for now
			    Matrix ident = Matrix.Identity;
			    CastResult castResult = new CastResult();
			    castResult.m_fraction = m_hitFraction;
			    SphereShape	pointShape = new SphereShape(m_ccdSphereRadius);
			    TriangleShape triShape = new TriangleShape(ref triangle[0],ref triangle[1],ref triangle[2]);
			    VoronoiSimplexSolver	simplexSolver = new VoronoiSimplexSolver();
			    SubSimplexConvexCast convexCaster = new SubSimplexConvexCast(pointShape,triShape,simplexSolver);
			    //GjkConvexCast	convexCaster(&pointShape,convexShape,&simplexSolver);
			    //ContinuousConvexCollision convexCaster(&pointShape,convexShape,&simplexSolver,0);
			    //local space?

			    if (convexCaster.CalcTimeOfImpact(ref m_ccdSphereFromTrans,ref m_ccdSphereToTrans,
				    ref ident,ref ident,castResult))
			    {
                    if (m_hitFraction > castResult.m_fraction)
                    {
                        m_hitFraction = castResult.m_fraction;
                    }
			    }

		    }

	    };

    public class ConvexTriangleCallback : ITriangleCallback
    {
	    private CollisionObject m_convexBody;
	    private CollisionObject m_triBody;

	    private Vector3	m_aabbMin;
	    private Vector3	m_aabbMax ;

	    private ManifoldResult m_resultOut;
	    private IDispatcher	m_dispatcher;
	    private DispatcherInfo m_dispatchInfoPtr;
	    private float m_collisionMarginTriangle;
	
        public int	m_triangleCount;
	
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
            //Matrix convexInTriangleSpace = MathUtil.bulletMatrixMultiply(Matrix.Invert(m_triBody.getWorldTransform()) , m_convexBody.getWorldTransform());
            Matrix convexInTriangleSpace = MathUtil.InverseTimes(m_triBody.GetWorldTransform(), m_convexBody.GetWorldTransform());
            CollisionShape convexShape = m_convexBody.GetCollisionShape();
	        convexShape.GetAabb(ref convexInTriangleSpace,out m_aabbMin,out m_aabbMax);
	        float extraMargin = collisionMarginTriangle;
	        Vector3 extra = new Vector3(extraMargin,extraMargin,extraMargin);

	        m_aabbMax += extra;
	        m_aabbMin -= extra;
        }

        public virtual void ProcessTriangle(Vector3[] triangle, int partId, int triangleIndex)
        {
	        //aabb filter is already applied!	
	        CollisionAlgorithmConstructionInfo ci = new CollisionAlgorithmConstructionInfo();
	        ci.SetDispatcher(m_dispatcher);

	        CollisionObject ob = (CollisionObject)m_triBody;
        	
	        ///debug drawing of the overlapping triangles
            ///

	        if (m_dispatchInfoPtr != null && m_dispatchInfoPtr.getDebugDraw() != null&& ((m_dispatchInfoPtr.getDebugDraw().GetDebugMode() & DebugDrawModes.DBG_DrawWireframe) > 0))
	        {
		        Vector3 color = new Vector3(1,1,0);
		        Matrix tr = ob.GetWorldTransform();

                Vector3[] transformedTriangles = new Vector3[3];
                Vector3.Transform(triangle, ref tr, transformedTriangles);

                m_dispatchInfoPtr.getDebugDraw().DrawLine(ref transformedTriangles[0], ref transformedTriangles[1], ref color);
                m_dispatchInfoPtr.getDebugDraw().DrawLine(ref transformedTriangles[1], ref transformedTriangles[2], ref color);
                m_dispatchInfoPtr.getDebugDraw().DrawLine(ref transformedTriangles[2], ref transformedTriangles[0], ref color);

	        }

	        if (m_convexBody.GetCollisionShape().IsConvex())
	        {
		        TriangleShape tm = new TriangleShape(triangle[0],triangle[1],triangle[2]);	
		        tm.SetMargin(m_collisionMarginTriangle);
        		
		        CollisionShape tmpShape = ob.GetCollisionShape();
		        ob.InternalSetTemporaryCollisionShape(tm);
        		
		        CollisionAlgorithm colAlgo = ci.GetDispatcher().FindAlgorithm(m_convexBody,m_triBody,m_manifoldPtr);
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

		        colAlgo.ProcessCollision(m_convexBody,m_triBody,m_dispatchInfoPtr, m_resultOut);
                colAlgo.Cleanup();
		        ci.GetDispatcher().FreeCollisionAlgorithm(colAlgo);
                colAlgo = null;

                ob.InternalSetTemporaryCollisionShape( tmpShape);
	        }
        }

        public void ClearCache()
        {
            m_dispatcher.ClearManifold(m_manifoldPtr);
        }

	    public Vector3 GetAabbMin()
	    {
		    return m_aabbMin;
	    }
	    public Vector3 GetAabbMax()
	    {
		    return m_aabbMax;
	    }

    }


    public class ConvexConcaveCreateFunc : CollisionAlgorithmCreateFunc
    {
        public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
        {
            return new ConvexConcaveCollisionAlgorithm(ci, body0, body1,false);
        }
    }

    public class SwappedConvexConcaveCreateFunc : CollisionAlgorithmCreateFunc
    {
        public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
        {
            return new ConvexConcaveCollisionAlgorithm(ci, body0, body1, true);
        }
    }

}
