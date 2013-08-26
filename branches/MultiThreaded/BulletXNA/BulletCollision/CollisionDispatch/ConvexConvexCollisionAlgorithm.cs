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
#define DEBUG_CONTACTS
//#define BT_DISABLE_CAPSULE_CAPSULE_COLLIDER

using System.Diagnostics;
using BulletXNA.LinearMath;


namespace BulletXNA.BulletCollision
{
    public class ConvexConvexAlgorithm : ActivatingCollisionAlgorithm
    {


#if USE_SEPDISTANCE_UTIL2
	    ConvexSeparatingDistanceUtil	m_sepDistance;
#endif
        ISimplexSolverInterface m_simplexSolver;
        IConvexPenetrationDepthSolver m_pdSolver;

        bool m_ownManifold;
        PersistentManifold m_manifoldPtr;
        bool m_lowLevelOfDetail;

        int m_numPerturbationIterations;
        int m_minimumPointsPerturbationThreshold;

        ///cache separating vector to speedup collision detection

        bool disableCcd = false;
        ObjectArray<IndexedVector3> m_vertices = new ObjectArray<IndexedVector3>();


        public ConvexConvexAlgorithm() { } // for pool

        public ConvexConvexAlgorithm(PersistentManifold mf, CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1, ISimplexSolverInterface simplexSolver, IConvexPenetrationDepthSolver pdSolver, int numPerturbationIterations, int minimumPointsPerturbationThreshold)
            : base(ci, body0, body1)
        {
            m_simplexSolver = simplexSolver;
            m_pdSolver = pdSolver;
            m_ownManifold = false;
            m_manifoldPtr = mf;
            m_lowLevelOfDetail = false;
#if USE_SEPDISTANCE_UTIL2
            m_sepDistance ((static_cast<btConvexShape*>(body0.getCollisionShape())).getAngularMotionDisc(),
			  (static_cast<btConvexShape*>(body1.getCollisionShape())).getAngularMotionDisc()),
#endif
            m_numPerturbationIterations = numPerturbationIterations;
            m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;
        }

        public void Initialize(PersistentManifold mf, CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1, ISimplexSolverInterface simplexSolver, IConvexPenetrationDepthSolver pdSolver, int numPerturbationIterations, int minimumPointsPerturbationThreshold)
        {
            base.Initialize(ci, body0, body1);
            m_simplexSolver = simplexSolver;
            m_pdSolver = pdSolver;
            m_ownManifold = false;
            m_manifoldPtr = mf;
            m_lowLevelOfDetail = false;
#if USE_SEPDISTANCE_UTIL2
            m_sepDistance ((static_cast<btConvexShape*>(body0.getCollisionShape())).getAngularMotionDisc(),
			  (static_cast<btConvexShape*>(body1.getCollisionShape())).getAngularMotionDisc()),
#endif
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
                }
                m_ownManifold = false;
            }
            m_manifoldPtr = null;
            BulletGlobals.ConvexConvexAlgorithmPool.Free(this);
        }

        public override void ProcessCollision(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            if (m_manifoldPtr == null)
            {
                //swapped?
                m_manifoldPtr = m_dispatcher.GetNewManifold(body0, body1);
                m_ownManifold = true;
            }
            //resultOut = new ManifoldResult();
            resultOut.SetPersistentManifold(m_manifoldPtr);

            //comment-out next line to test multi-contact generation
            //resultOut.GetPersistentManifold().ClearManifold();


            ConvexShape min0 = body0.GetCollisionShape() as ConvexShape;
            ConvexShape min1 = body1.GetCollisionShape() as ConvexShape;
            IndexedVector3 normalOnB;
            IndexedVector3 pointOnBWorld;
#if !BT_DISABLE_CAPSULE_CAPSULE_COLLIDER
            if ((min0.GetShapeType() == BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE) && (min1.GetShapeType() == BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE))
            {
                CapsuleShape capsuleA = min0 as CapsuleShape;
                CapsuleShape capsuleB = min1 as CapsuleShape;
                //IndexedVector3 localScalingA = capsuleA.GetLocalScaling();
                //IndexedVector3 localScalingB = capsuleB.GetLocalScaling();

                float threshold = m_manifoldPtr.GetContactBreakingThreshold();

                float dist = CapsuleCapsuleDistance(out normalOnB, out pointOnBWorld, capsuleA.GetHalfHeight(), capsuleA.GetRadius(),
                    capsuleB.GetHalfHeight(), capsuleB.GetRadius(), capsuleA.GetUpAxis(), capsuleB.GetUpAxis(),
                    body0.GetWorldTransform(), body1.GetWorldTransform(), threshold);

                if (dist < threshold)
                {
                    Debug.Assert(normalOnB.LengthSquared() >= (MathUtil.SIMD_EPSILON * MathUtil.SIMD_EPSILON));
                    resultOut.AddContactPoint(ref normalOnB, ref pointOnBWorld, dist);
                }
                resultOut.RefreshContactPoints();
                return;
            }
#endif //BT_DISABLE_CAPSULE_CAPSULE_COLLIDER



#if USE_SEPDISTANCE_UTIL2
        	if (dispatchInfo.m_useConvexConservativeDistanceUtil)
            {
                m_sepDistance.updateSeparatingDistance(body0.getWorldTransform(),body1.getWorldTransform());
            }

	        if (!dispatchInfo.m_useConvexConservativeDistanceUtil || m_sepDistance.getConservativeSeparatingDistance()<=0.f)
#endif //USE_SEPDISTANCE_UTIL2

            {


                ClosestPointInput input = ClosestPointInput.Default();

                using (GjkPairDetector gjkPairDetector = BulletGlobals.GjkPairDetectorPool.Get())
                {
                    gjkPairDetector.Initialize(min0, min1, m_simplexSolver, m_pdSolver);
                    //TODO: if (dispatchInfo.m_useContinuous)
                    gjkPairDetector.SetMinkowskiA(min0);
                    gjkPairDetector.SetMinkowskiB(min1);

#if USE_SEPDISTANCE_UTIL2
	        if (dispatchInfo.m_useConvexConservativeDistanceUtil)
	        {
		        input.m_maximumDistanceSquared = float.MaxValue;
	        } 
            else
#endif //USE_SEPDISTANCE_UTIL2
                    {
                        input.m_maximumDistanceSquared = min0.GetMargin() + min1.GetMargin() + m_manifoldPtr.GetContactBreakingThreshold();
                        input.m_maximumDistanceSquared *= input.m_maximumDistanceSquared;
                    }

                    //input.m_stackAlloc = dispatchInfo.m_stackAllocator;
                    input.m_transformA = body0.GetWorldTransform();
                    input.m_transformB = body1.GetWorldTransform();


                    if (min0.IsPolyhedral() && min1.IsPolyhedral())
                    {


                        DummyResult dummy = new DummyResult();


                        PolyhedralConvexShape polyhedronA = min0 as PolyhedralConvexShape;
                        PolyhedralConvexShape polyhedronB = min1 as PolyhedralConvexShape;
                        if (polyhedronA.GetConvexPolyhedron() != null && polyhedronB.GetConvexPolyhedron() != null)
                        {
                            float threshold = m_manifoldPtr.GetContactBreakingThreshold();

                            float minDist = float.MinValue;
                            IndexedVector3 sepNormalWorldSpace = new IndexedVector3(0, 1, 0);
                            bool foundSepAxis = true;

                            if (dispatchInfo.m_enableSatConvex)
                            {
                                foundSepAxis = PolyhedralContactClipping.FindSeparatingAxis(
                                    polyhedronA.GetConvexPolyhedron(), polyhedronB.GetConvexPolyhedron(),
                                    body0.GetWorldTransform(),
                                    body1.GetWorldTransform(),
                                    out sepNormalWorldSpace);
                            }
                            else
                            {

#if ZERO_MARGIN
                gjkPairDetector.SetIgnoreMargin(true);
                gjkPairDetector.GetClosestPoints(input,resultOut,dispatchInfo.m_debugDraw);
#else

                                gjkPairDetector.GetClosestPoints(ref input, dummy, dispatchInfo.m_debugDraw);
#endif

                                float l2 = gjkPairDetector.GetCachedSeparatingAxis().LengthSquared();
                                if (l2 > MathUtil.SIMD_EPSILON)
                                {
                                    sepNormalWorldSpace = gjkPairDetector.GetCachedSeparatingAxis() * (1.0f / l2);
                                    //minDist = -1e30f;//gjkPairDetector.getCachedSeparatingDistance();
                                    minDist = gjkPairDetector.GetCachedSeparatingDistance() - min0.GetMargin() - min1.GetMargin();

#if ZERO_MARGIN
					foundSepAxis = true;//gjkPairDetector.getCachedSeparatingDistance()<0.f;
#else
                                    foundSepAxis = gjkPairDetector.GetCachedSeparatingDistance() < (min0.GetMargin() + min1.GetMargin());
#endif
                                }
                            }
                            if (foundSepAxis)
                            {
                                //				printf("sepNormalWorldSpace=%f,%f,%f\n",sepNormalWorldSpace.getX(),sepNormalWorldSpace.getY(),sepNormalWorldSpace.getZ());

                                PolyhedralContactClipping.ClipHullAgainstHull(sepNormalWorldSpace, polyhedronA.GetConvexPolyhedron(), polyhedronB.GetConvexPolyhedron(),
                                    body0.GetWorldTransform(),
                                    body1.GetWorldTransform(), minDist - threshold, threshold, resultOut);

                            }
                            if (m_ownManifold)
                            {
                                resultOut.RefreshContactPoints();
                            }

                            return;

                        }
                        else
                        {

                            //we can also deal with convex versus triangle (without connectivity data)
                            if (polyhedronA.GetConvexPolyhedron() != null && polyhedronB.GetShapeType() == BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE)
                            {
                                m_vertices.Clear();
                                TriangleShape tri = polyhedronB as TriangleShape;
                                m_vertices.Add(body1.GetWorldTransform() * tri.m_vertices1[0]);
                                m_vertices.Add(body1.GetWorldTransform() * tri.m_vertices1[1]);
                                m_vertices.Add(body1.GetWorldTransform() * tri.m_vertices1[2]);

                                float threshold = m_manifoldPtr.GetContactBreakingThreshold();
                                IndexedVector3 sepNormalWorldSpace = new IndexedVector3(0, 1, 0); ;
                                float minDist = float.MinValue;
                                float maxDist = threshold;

                                bool foundSepAxis = false;
                                if (false)
                                {
                                    polyhedronB.InitializePolyhedralFeatures();
                                    foundSepAxis = PolyhedralContactClipping.FindSeparatingAxis(
                                    polyhedronA.GetConvexPolyhedron(), polyhedronB.GetConvexPolyhedron(),
                                    body0.GetWorldTransform(),
                                    body1.GetWorldTransform(),
                                    out sepNormalWorldSpace);
                                    //	 printf("sepNormalWorldSpace=%f,%f,%f\n",sepNormalWorldSpace.getX(),sepNormalWorldSpace.getY(),sepNormalWorldSpace.getZ());

                                }
                                else
                                {
#if ZERO_MARGIN
					gjkPairDetector.SetIgnoreMargin(true);
					gjkPairDetector.GetClosestPoints(input,resultOut,dispatchInfo.m_debugDraw);
#else
                                    gjkPairDetector.GetClosestPoints(ref input, dummy, dispatchInfo.m_debugDraw);
#endif//ZERO_MARGIN

                                    float l2 = gjkPairDetector.GetCachedSeparatingAxis().LengthSquared();
                                    if (l2 > MathUtil.SIMD_EPSILON)
                                    {
                                        sepNormalWorldSpace = gjkPairDetector.GetCachedSeparatingAxis() * (1.0f / l2);
                                        //minDist = gjkPairDetector.getCachedSeparatingDistance();
                                        //maxDist = threshold;
                                        minDist = gjkPairDetector.GetCachedSeparatingDistance() - min0.GetMargin() - min1.GetMargin();
                                        foundSepAxis = true;
                                    }
                                }


                                if (foundSepAxis)
                                {
                                    PolyhedralContactClipping.ClipFaceAgainstHull(sepNormalWorldSpace, polyhedronA.GetConvexPolyhedron(),
                                        body0.GetWorldTransform(), m_vertices, minDist - threshold, maxDist, resultOut);
                                }

                                if (m_ownManifold)
                                {
                                    resultOut.RefreshContactPoints();
                                }
                                return;
                            }

                        }


                    }


                    gjkPairDetector.GetClosestPoints(ref input, resultOut, dispatchInfo.getDebugDraw(), false);
#if USE_SEPDISTANCE_UTIL2
	float sepDist = 0.f;
	if (dispatchInfo.m_useConvexConservativeDistanceUtil)
	{
		sepDist = gjkPairDetector.getCachedSeparatingDistance();
		if (sepDist>MathUtil.SIMD_EPSILON)
		{
			sepDist += dispatchInfo.m_convexConservativeDistanceThreshold;
			//now perturbe directions to get multiple contact points
		}
	}
#endif //USE_SEPDISTANCE_UTIL2

                    //now perform 'm_numPerturbationIterations' collision queries with the perturbated collision objects

                    //perform perturbation when more then 'm_minimumPointsPerturbationThreshold' points
                    if (m_numPerturbationIterations > 0 && resultOut.GetPersistentManifold().GetNumContacts() < m_minimumPointsPerturbationThreshold)
                    {
                        IndexedVector3 v0, v1;

                        IndexedVector3 sepNormalWorldSpace = gjkPairDetector.GetCachedSeparatingAxis();
                        sepNormalWorldSpace.Normalize();
                        TransformUtil.PlaneSpace1(ref sepNormalWorldSpace, out v0, out v1);

                        bool perturbeA = true;
                        const float angleLimit = 0.125f * MathUtil.SIMD_PI;
                        float perturbeAngle;
                        float radiusA = min0.GetAngularMotionDisc();
                        float radiusB = min1.GetAngularMotionDisc();
                        if (radiusA < radiusB)
                        {
                            perturbeAngle = BulletGlobals.gContactBreakingThreshold / radiusA;
                            perturbeA = true;
                        }
                        else
                        {
                            perturbeAngle = BulletGlobals.gContactBreakingThreshold / radiusB;
                            perturbeA = false;
                        }
                        if (perturbeAngle > angleLimit)
                        {
                            perturbeAngle = angleLimit;
                        }

                        IndexedMatrix unPerturbedTransform;
                        if (perturbeA)
                        {
                            unPerturbedTransform = input.m_transformA;
                        }
                        else
                        {
                            unPerturbedTransform = input.m_transformB;
                        }

                        for (int i = 0; i < m_numPerturbationIterations; i++)
                        {
                            if (v0.LengthSquared() > MathUtil.SIMD_EPSILON)
                            {

                                IndexedQuaternion perturbeRot = new IndexedQuaternion(v0, perturbeAngle);
                                float iterationAngle = i * (MathUtil.SIMD_2_PI / (float)m_numPerturbationIterations);
                                IndexedQuaternion rotq = new IndexedQuaternion(sepNormalWorldSpace, iterationAngle);

                                if (perturbeA)
                                {
                                    input.m_transformA._basis = (new IndexedBasisMatrix(MathUtil.QuaternionInverse(rotq) * perturbeRot * rotq) * body0.GetWorldTransform()._basis);
                                    input.m_transformB = body1.GetWorldTransform();

                                    input.m_transformB = body1.GetWorldTransform();
#if DEBUG_CONTACTS
                                    dispatchInfo.m_debugDraw.DrawTransform(ref input.m_transformA, 10.0f);
#endif //DEBUG_CONTACTS
                                }
                                else
                                {
                                    input.m_transformA = body0.GetWorldTransform();
                                    input.m_transformB._basis = (new IndexedBasisMatrix(MathUtil.QuaternionInverse(rotq) * perturbeRot * rotq) * body1.GetWorldTransform()._basis);
#if DEBUG_CONTACTS
                                    dispatchInfo.m_debugDraw.DrawTransform(ref input.m_transformB, 10.0f);
#endif
                                }

                                PerturbedContactResult perturbedResultOut = new PerturbedContactResult(resultOut, ref input.m_transformA, ref input.m_transformB, ref unPerturbedTransform, perturbeA, dispatchInfo.getDebugDraw());
                                gjkPairDetector.GetClosestPoints(ref input, perturbedResultOut, dispatchInfo.getDebugDraw(), false);

                            }


                        }
                    }



#if USE_SEPDISTANCE_UTIL2
	        if (dispatchInfo.m_useConvexConservativeDistanceUtil && (sepDist > MathUtil.SIMD_EPSILON))
	        {
		        m_sepDistance.initSeparatingDistance(gjkPairDetector.getCachedSeparatingAxis(),sepDist,body0.getWorldTransform(),body1.getWorldTransform());
	        }
#endif //USE_SEPDISTANCE_UTIL2

                }
            }

            if (m_ownManifold)
            {
                resultOut.RefreshContactPoints();
            }
        }

        public override float CalculateTimeOfImpact(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            //(void)resultOut;
            //(void)dispatchInfo;
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
            if (disableCcd)
            {
                return 1f;
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
                using (GjkConvexCast ccd1 = BulletGlobals.GjkConvexCastPool.Get())
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

        public static void SegmentsClosestPoints(
            out IndexedVector3 ptsVector,
            out IndexedVector3 offsetA,
            out IndexedVector3 offsetB,
            out float tA, out float tB,
            ref IndexedVector3 translation,
            ref IndexedVector3 dirA, float hlenA,
            ref IndexedVector3 dirB, float hlenB)
        {
            // compute the parameters of the closest points on each line segment

            float dirA_dot_dirB = IndexedVector3.Dot(ref dirA, ref dirB);
            float dirA_dot_trans = IndexedVector3.Dot(ref dirA, ref translation);
            float dirB_dot_trans = IndexedVector3.Dot(ref dirB, ref translation);

            float denom = 1.0f - dirA_dot_dirB * dirA_dot_dirB;

            if (MathUtil.FuzzyZero(denom))
            {
                tA = 0.0f;
            }
            else
            {
                tA = (dirA_dot_trans - dirB_dot_trans * dirA_dot_dirB) / denom;
                if (tA < -hlenA)
                {
                    tA = -hlenA;
                }
                else if (tA > hlenA)
                {
                    tA = hlenA;
                }
            }

            tB = tA * dirA_dot_dirB - dirB_dot_trans;

            if (tB < -hlenB)
            {
                tB = -hlenB;
                tA = tB * dirA_dot_dirB + dirA_dot_trans;

                if (tA < -hlenA)
                {
                    tA = -hlenA;
                }
                else if (tA > hlenA)
                {
                    tA = hlenA;
                }
            }
            else if (tB > hlenB)
            {
                tB = hlenB;
                tA = tB * dirA_dot_dirB + dirA_dot_trans;

                if (tA < -hlenA)
                {
                    tA = -hlenA;
                }
                else if (tA > hlenA)
                {
                    tA = hlenA;
                }
            }

            // compute the closest points relative to segment centers.

            offsetA = dirA * tA;
            offsetB = dirB * tB;

            ptsVector = translation - offsetA + offsetB;
        }

        public static float CapsuleCapsuleDistance(
            out IndexedVector3 normalOnB,
            out IndexedVector3 pointOnB,
            float capsuleLengthA,
            float capsuleRadiusA,
            float capsuleLengthB,
            float capsuleRadiusB,
            int capsuleAxisA,
            int capsuleAxisB,
            IndexedMatrix transformA,
            IndexedMatrix transformB,
            float distanceThreshold)
        {
            return CapsuleCapsuleDistance(out normalOnB, out pointOnB, capsuleLengthA,
                capsuleRadiusA, capsuleLengthB, capsuleRadiusB, capsuleAxisA, capsuleAxisB,
                ref transformA, ref transformB, distanceThreshold);
        }

        public static float CapsuleCapsuleDistance(
            out IndexedVector3 normalOnB,
            out IndexedVector3 pointOnB,
            float capsuleLengthA,
            float capsuleRadiusA,
            float capsuleLengthB,
            float capsuleRadiusB,
            int capsuleAxisA,
            int capsuleAxisB,
            ref IndexedMatrix transformA,
            ref IndexedMatrix transformB,
            float distanceThreshold)
        {
            IndexedVector3 directionA = transformA._basis.GetColumn(capsuleAxisA);
            IndexedVector3 translationA = transformA._origin;
            IndexedVector3 directionB = transformB._basis.GetColumn(capsuleAxisB);
            IndexedVector3 translationB = transformB._origin;

            // translation between centers

            IndexedVector3 translation = translationB - translationA;

            // compute the closest points of the capsule line segments

            IndexedVector3 ptsVector;           // the vector between the closest points

            IndexedVector3 offsetA, offsetB;    // offsets from segment centers to their closest points
            float tA, tB;              // parameters on line segment

            SegmentsClosestPoints(out ptsVector, out offsetA, out offsetB, out tA, out tB, ref translation,
                                   ref directionA, capsuleLengthA, ref directionB, capsuleLengthB);

            float distance = ptsVector.Length() - capsuleRadiusA - capsuleRadiusB;

            if (distance > distanceThreshold)
            {
                normalOnB = new IndexedVector3(0, 1, 0);
                pointOnB = IndexedVector3.Zero;
                return distance;
            }

            float lenSqr = ptsVector.LengthSquared();
            if (lenSqr <= (MathUtil.SIMD_EPSILON * MathUtil.SIMD_EPSILON))
            {
                //degenerate case where 2 capsules are likely at the same location: take a vector tangential to 'directionA'
                IndexedVector3 q;
                TransformUtil.PlaneSpace1(ref directionA, out normalOnB, out q);
            }
            else
            {
                // compute the contact normal
                normalOnB = ptsVector * -MathUtil.RecipSqrt(lenSqr);
            }
            pointOnB = transformB._origin + offsetB + normalOnB * capsuleRadiusB;

            return distance;
        }
    }



    public class ConvexConvexCreateFunc : CollisionAlgorithmCreateFunc
    {
        public ConvexConvexCreateFunc(ISimplexSolverInterface simplexSolver, IConvexPenetrationDepthSolver depthSolver)
        {
            m_numPerturbationIterations = 0;
            m_minimumPointsPerturbationThreshold = 3;
            m_simplexSolver = simplexSolver;
            m_pdSolver = depthSolver;
        }

        public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
        {
            ConvexConvexAlgorithm cca = BulletGlobals.ConvexConvexAlgorithmPool.Get();
            cca.Initialize(ci.GetManifold(), ci, body0, body1, m_simplexSolver, m_pdSolver, m_numPerturbationIterations, m_minimumPointsPerturbationThreshold);
            return cca;
        }

        public IConvexPenetrationDepthSolver m_pdSolver;
        public ISimplexSolverInterface m_simplexSolver;
        public int m_numPerturbationIterations;
        public int m_minimumPointsPerturbationThreshold;
    }



    public class PerturbedContactResult : ManifoldResult
    {
        public ManifoldResult m_originalManifoldResult;
        public IndexedMatrix m_transformA;
        public IndexedMatrix m_transformB;
        public IndexedMatrix m_unPerturbedTransform;
        public bool m_perturbA;
        public IDebugDraw m_debugDrawer;


        public PerturbedContactResult(ManifoldResult originalResult, ref IndexedMatrix transformA, ref IndexedMatrix transformB, ref IndexedMatrix unPerturbedTransform, bool perturbA, IDebugDraw debugDrawer)
        {
            m_originalManifoldResult = originalResult;
            m_transformA = transformA;
            m_transformB = transformB;
            m_perturbA = perturbA;
            m_unPerturbedTransform = unPerturbedTransform;
            m_debugDrawer = debugDrawer;
        }

        public override void AddContactPoint(ref IndexedVector3 normalOnBInWorld, ref IndexedVector3 pointInWorld, float orgDepth)
        {
            IndexedVector3 endPt, startPt;
            float newDepth;
            IndexedVector3 newNormal = new IndexedVector3(0, 1, 0);

            if (m_perturbA)
            {
                IndexedVector3 endPtOrg = pointInWorld + normalOnBInWorld * orgDepth;
                endPt = (m_unPerturbedTransform * m_transformA.Inverse()) * (endPtOrg);
                newDepth = IndexedVector3.Dot((endPt - pointInWorld), normalOnBInWorld);
                startPt = endPt + normalOnBInWorld * newDepth;
            }
            else
            {
                endPt = pointInWorld + normalOnBInWorld * orgDepth;
                startPt = (m_unPerturbedTransform * m_transformB.Inverse()) * (pointInWorld);
                newDepth = IndexedVector3.Dot((endPt - startPt), normalOnBInWorld);
            }

            //#define DEBUG_CONTACTS 1
#if DEBUG_CONTACTS
            m_debugDrawer.DrawLine(startPt, endPt, new IndexedVector3(1, 0, 0));
            m_debugDrawer.DrawSphere(startPt, 0.5f, new IndexedVector3(0, 1, 0));
            m_debugDrawer.DrawSphere(endPt, 0.5f, new IndexedVector3(0, 0, 1));
#endif //DEBUG_CONTACTS


            m_originalManifoldResult.AddContactPoint(ref normalOnBInWorld, ref startPt, newDepth);
        }

    }

    public struct DummyResult : IDiscreteCollisionDetectorInterfaceResult
    {
        public void SetShapeIdentifiersA(int partId0, int index0) { }
        public void SetShapeIdentifiersB(int partId1, int index1) { }

        public void AddContactPoint(IndexedVector3 normalOnBInWorld, IndexedVector3 pointInWorld, float depth)
        {
        }

        public void AddContactPoint(ref IndexedVector3 normalOnBInWorld, ref IndexedVector3 pointInWorld, float depth)
        {
        }
    }



}
