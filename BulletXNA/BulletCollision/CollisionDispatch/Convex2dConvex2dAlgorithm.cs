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

namespace BulletXNA.BulletCollision
{
    public class Convex2dConvex2dAlgorithm : ActivatingCollisionAlgorithm
    {
        public Convex2dConvex2dAlgorithm(PersistentManifold mf, CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1, ISimplexSolverInterface simplexSolver, IConvexPenetrationDepthSolver pdSolver, int numPerturbationIterations, int minimumPointsPerturbationThreshold)
            : base(ci, body0, body1)
        {
            m_simplexSolver = simplexSolver;
            m_pdSolver = pdSolver;
            m_ownManifold = false;
            m_manifoldPtr = mf;
            m_lowLevelOfDetail = false;
            m_numPerturbationIterations = numPerturbationIterations;
            m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;
        }

        public override void Cleanup()
        {
            if (m_ownManifold)
            {
                if (m_manifoldPtr != null)
                {
                    m_dispatcher.ReleaseManifold(m_manifoldPtr);
                    m_manifoldPtr = null;
                }
                m_ownManifold = false;
            }
        }

        public override void ProcessCollision(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            if (m_manifoldPtr == null)
            {
                //swapped?
                m_manifoldPtr = m_dispatcher.GetNewManifold(body0, body1);
                m_ownManifold = true;
            }
            resultOut.SetPersistentManifold(m_manifoldPtr);

            //comment-out next line to test multi-contact generation
            //resultOut.getPersistentManifold().clearManifold();


            ConvexShape min0 = body0.GetCollisionShape() as ConvexShape;
            ConvexShape min1 = body1.GetCollisionShape() as ConvexShape;

            IndexedVector3 normalOnB = IndexedVector3.Zero;
            IndexedVector3 pointOnBWorld = IndexedVector3.Zero;

            {
                ClosestPointInput input = ClosestPointInput.Default();

                using (GjkPairDetector gjkPairDetector = BulletGlobals.GjkPairDetectorPool.Get())
                {
                    gjkPairDetector.Initialize(min0, min1, m_simplexSolver, m_pdSolver);
                    //TODO: if (dispatchInfo.m_useContinuous)
                    gjkPairDetector.SetMinkowskiA(min0);
                    gjkPairDetector.SetMinkowskiB(min1);

                    {
                        input.m_maximumDistanceSquared = min0.GetMargin() + min1.GetMargin() + m_manifoldPtr.GetContactBreakingThreshold();
                        input.m_maximumDistanceSquared *= input.m_maximumDistanceSquared;
                    }

                    input.m_transformA = body0.GetWorldTransform();
                    input.m_transformB = body1.GetWorldTransform();

                    gjkPairDetector.GetClosestPoints(ref input, resultOut, dispatchInfo.getDebugDraw(), false);
#if DEBUG
                    if (BulletGlobals.g_streamWriter != null)
                    {
                        BulletGlobals.g_streamWriter.WriteLine("c2dc2d processCollision");
                        MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "transformA", input.m_transformA);
                        MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "transformB", input.m_transformB);
                    }
#endif                    
                }
                //BulletGlobals.GjkPairDetectorPool.Free(gjkPairDetector);
                //btVector3 v0,v1;
                //btVector3 sepNormalWorldSpace;

            }

            if (m_ownManifold)
            {
                resultOut.RefreshContactPoints();
            }

        }

        public override float CalculateTimeOfImpact(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            ///Rather then checking ALL pairs, only calculate TOI when motion exceeds threshold

            ///Linear motion for one of objects needs to exceed m_ccdSquareMotionThreshold
            ///body0.m_worldTransform,
            float resultFraction = 1.0f;


            float squareMot0 = (body0.GetInterpolationWorldTransform()._origin - body0.GetWorldTransform()._origin).LengthSquared();
            float squareMot1 = (body1.GetInterpolationWorldTransform()._origin - body1.GetWorldTransform()._origin).LengthSquared();

            if (squareMot0 < body0.GetCcdSquareMotionThreshold() &&
                squareMot1 < body1.GetCcdSquareMotionThreshold())
            {
                return resultFraction;
            }

            //An adhoc way of testing the Continuous Collision Detection algorithms
            //One object is approximated as a sphere, to simplify things
            //Starting in penetration should report no time of impact
            //For proper CCD, better accuracy and handling of 'allowed' penetration should be added
            //also the mainloop of the physics should have a kind of toi queue (something like Brian Mirtich's application of Timewarp for Rigidbodies)


            /// Convex0 against sphere for Convex1
            {
                ConvexShape convex0 = body0.GetCollisionShape() as ConvexShape;

                SphereShape sphere1 = BulletGlobals.SphereShapePool.Get();
                sphere1.Initialize(body1.GetCcdSweptSphereRadius()); //todo: allow non-zero sphere sizes, for better approximation
                CastResult result = BulletGlobals.CastResultPool.Get();
                VoronoiSimplexSolver voronoiSimplex = BulletGlobals.VoronoiSimplexSolverPool.Get();
                //SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
                ///Simplification, one object is simplified as a sphere
                using(GjkConvexCast ccd1 = BulletGlobals.GjkConvexCastPool.Get())
                {
                    ccd1.Initialize(convex0, sphere1, voronoiSimplex);
                //ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
                if (ccd1.CalcTimeOfImpact(body0.GetWorldTransform(), body0.GetInterpolationWorldTransform(),
                    body1.GetWorldTransform(), body1.GetInterpolationWorldTransform(), result))
                {

                    //store result.m_fraction in both bodies

                    if (body0.GetHitFraction() > result.m_fraction)
                    {
                        body0.SetHitFraction(result.m_fraction);
                    }

                    if (body1.GetHitFraction() > result.m_fraction)
                    {
                        body1.SetHitFraction(result.m_fraction);
                    }

                    if (resultFraction > result.m_fraction)
                    {
                        resultFraction = result.m_fraction;
                    }

                }
                BulletGlobals.VoronoiSimplexSolverPool.Free(voronoiSimplex);
                BulletGlobals.SphereShapePool.Free(sphere1);
                result.Cleanup();
                }
            }

            /// Sphere (for convex0) against Convex1
            {
                ConvexShape convex1 = body1.GetCollisionShape() as ConvexShape;

                SphereShape sphere0 = BulletGlobals.SphereShapePool.Get();
                sphere0.Initialize(body0.GetCcdSweptSphereRadius()); //todo: allow non-zero sphere sizes, for better approximation
                CastResult result = BulletGlobals.CastResultPool.Get();
                VoronoiSimplexSolver voronoiSimplex = BulletGlobals.VoronoiSimplexSolverPool.Get();
                //SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
                ///Simplification, one object is simplified as a sphere
                using (GjkConvexCast ccd1 = BulletGlobals.GjkConvexCastPool.Get())
                {
                    ccd1.Initialize(sphere0, convex1, voronoiSimplex);
                    //ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
                    if (ccd1.CalcTimeOfImpact(body0.GetWorldTransform(), body0.GetInterpolationWorldTransform(),
                        body1.GetWorldTransform(), body1.GetInterpolationWorldTransform(), result))
                    {

                        //store result.m_fraction in both bodies

                        if (body0.GetHitFraction() > result.m_fraction)
                        {
                            body0.SetHitFraction(result.m_fraction);
                        }

                        if (body1.GetHitFraction() > result.m_fraction)
                        {
                            body1.SetHitFraction(result.m_fraction);
                        }

                        if (resultFraction > result.m_fraction)
                        {
                            resultFraction = result.m_fraction;
                        }

                    }
                    BulletGlobals.VoronoiSimplexSolverPool.Free(voronoiSimplex);
                    BulletGlobals.SphereShapePool.Free(sphere0);
                    result.Cleanup();
                }
            }
            
            return resultFraction;
        }

        public override void GetAllContactManifolds(PersistentManifoldArray manifoldArray)
        {
            ///should we use m_ownManifold to avoid adding duplicates?
            if (m_manifoldPtr != null && m_ownManifold)
            {
                manifoldArray.Add(m_manifoldPtr);
            }
        }


        public void SetLowLevelOfDetail(bool useLowLevel)
        {
            m_lowLevelOfDetail = useLowLevel;
        }


        public PersistentManifold GetManifold()
        {
            return m_manifoldPtr;
        }



        private ISimplexSolverInterface m_simplexSolver;
        private IConvexPenetrationDepthSolver m_pdSolver;

        private bool m_ownManifold;
        private PersistentManifold m_manifoldPtr;
        private bool m_lowLevelOfDetail;

        private int m_numPerturbationIterations;
        private int m_minimumPointsPerturbationThreshold;

    }

    public class Convex2dConvex2dCreateFunc : CollisionAlgorithmCreateFunc
    {

        public Convex2dConvex2dCreateFunc(ISimplexSolverInterface simplexSolver, IConvexPenetrationDepthSolver pdSolver)
        {
            m_numPerturbationIterations = 0;
            m_minimumPointsPerturbationThreshold = 3;
            m_simplexSolver = simplexSolver;
            m_pdSolver = pdSolver;
        }

        public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
        {
            return new Convex2dConvex2dAlgorithm(ci.GetManifold(), ci, body0, body1, m_simplexSolver, m_pdSolver, m_numPerturbationIterations, m_minimumPointsPerturbationThreshold);
        }
        IConvexPenetrationDepthSolver m_pdSolver;
        ISimplexSolverInterface m_simplexSolver;
        int m_numPerturbationIterations;
        int m_minimumPointsPerturbationThreshold;
    }

}
