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

using System.Collections.Generic;
using System.Diagnostics;
using BulletXNA.BulletCollision.BroadphaseCollision;
using BulletXNA.BulletCollision.CollisionShapes;
using BulletXNA.BulletCollision.NarrowPhaseCollision;
using BulletXNA.LinearMath;
using Microsoft.Xna.Framework;


namespace BulletXNA.BulletCollision.CollisionDispatch
{
    public class ConvexConvexAlgorithm : ActivatingCollisionAlgorithm
    {


    #if USE_SEPDISTANCE_UTIL2
	    ConvexSeparatingDistanceUtil	m_sepDistance;
    #endif
	    ISimplexSolverInterface		m_simplexSolver;
	    IConvexPenetrationDepthSolver m_pdSolver;
    	
	    bool m_ownManifold;
	    PersistentManifold	m_manifoldPtr;
	    bool m_lowLevelOfDetail;
    	
	    int m_numPerturbationIterations;
	    int m_minimumPointsPerturbationThreshold;

	    ///cache separating vector to speedup collision detection

        bool disableCcd = false;

    	public ConvexConvexAlgorithm(PersistentManifold mf,CollisionAlgorithmConstructionInfo ci,CollisionObject body0,CollisionObject body1, ISimplexSolverInterface simplexSolver, IConvexPenetrationDepthSolver pdSolver, int numPerturbationIterations, int minimumPointsPerturbationThreshold) : base(ci,body0,body1)
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
        }

        public override void ProcessCollision(CollisionObject body0,CollisionObject body1,DispatcherInfo dispatchInfo,ManifoldResult resultOut)
        {
	        if (m_manifoldPtr == null)
	        {
		        //swapped?
		        m_manifoldPtr = m_dispatcher.GetNewManifold(body0,body1);
		        m_ownManifold = true;
	        }
            //resultOut = new ManifoldResult();
	        resultOut.SetPersistentManifold(m_manifoldPtr);

	        //comment-out next line to test multi-contact generation
	        //resultOut.getPersistentManifold().clearManifold();
        	

	        ConvexShape min0 = (ConvexShape)(body0.GetCollisionShape());
	        ConvexShape min1 = (ConvexShape)(body1.GetCollisionShape());
        	Vector3  normalOnB = Vector3.Up;
            Vector3  pointOnBWorld = Vector3.Zero;
#if !BT_DISABLE_CAPSULE_CAPSULE_COLLIDER
            if ((min0.GetShapeType() == BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE) && (min1.GetShapeType() == BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE))
	        {
		        CapsuleShape capsuleA = (CapsuleShape) min0;
		        CapsuleShape capsuleB = (CapsuleShape) min1;
		        Vector3 localScalingA = capsuleA.GetLocalScaling();
		        Vector3 localScalingB = capsuleB.GetLocalScaling();
        		
		        float threshold = m_manifoldPtr.GetContactBreakingThreshold();

		        float dist = CapsuleCapsuleDistance(ref normalOnB,ref pointOnBWorld,capsuleA.getHalfHeight(),capsuleA.getRadius(),
			        capsuleB.getHalfHeight(),capsuleB.getRadius(),capsuleA.GetUpAxis(),capsuleB.GetUpAxis(),
			        body0.GetWorldTransform(),body1.GetWorldTransform(),threshold);

		        if (dist<threshold)
		        {
                    Debug.Assert(normalOnB.LengthSquared() >= (MathUtil.SIMD_EPSILON * MathUtil.SIMD_EPSILON));
			        resultOut.AddContactPoint(ref normalOnB,ref pointOnBWorld,dist);	
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

        	
	        ClosestPointInput input = new ClosestPointInput();

	        GjkPairDetector	gjkPairDetector = new GjkPairDetector(min0,min1,m_simplexSolver,m_pdSolver);
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
		        input.m_maximumDistanceSquared*= input.m_maximumDistanceSquared;
	        }

            //input.m_stackAlloc = dispatchInfo.m_stackAllocator;
	        input.m_transformA = body0.GetWorldTransform();
	        input.m_transformB = body1.GetWorldTransform();

	        gjkPairDetector.GetClosestPoints(input,resultOut,dispatchInfo.getDebugDraw(),false);
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
	        if (m_numPerturbationIterations > 0 &&  resultOut.GetPersistentManifold().GetNumContacts() < m_minimumPointsPerturbationThreshold)
	        {
                Vector3 v0, v1;

                Vector3 sepNormalWorldSpace = gjkPairDetector.GetCachedSeparatingAxis();
                sepNormalWorldSpace.Normalize();
                TransformUtil.PlaneSpace1(ref sepNormalWorldSpace, out v0, out v1);

		        bool perturbeA = true;
		        const float angleLimit = 0.125f * MathUtil.SIMD_PI;
		        float perturbeAngle;
		        float radiusA = min0.GetAngularMotionDisc();
		        float radiusB = min1.GetAngularMotionDisc();
		        if (radiusA < radiusB)
		        {
			        perturbeAngle = BulletGlobals.gContactBreakingThreshold /radiusA;
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

		        Matrix unPerturbedTransform = Matrix.Identity;
		        if (perturbeA)
		        {
			        unPerturbedTransform = input.m_transformA;
		        } 
                else
		        {
			        unPerturbedTransform = input.m_transformB;
		        }
        		
		        for (int i=0;i<m_numPerturbationIterations;i++)
		        {
                    if (v0.LengthSquared() > MathUtil.SIMD_EPSILON)
                    {

                        Quaternion perturbeRot = Quaternion.CreateFromAxisAngle(v0, perturbeAngle);
                        float iterationAngle = i * (MathUtil.SIMD_2_PI / (float)m_numPerturbationIterations);
                        Quaternion rotq = Quaternion.CreateFromAxisAngle(sepNormalWorldSpace, iterationAngle);

                        if (perturbeA)
                        {
                            //input.m_transformA.setBasis(  btMatrix3x3(rotq.inverse()*perturbeRot*rotq)*body0.getWorldTransform().getBasis());
                            Quaternion temp = MathUtil.QuaternionMultiply(MathUtil.QuaternionInverse(ref rotq),MathUtil.QuaternionMultiply(perturbeRot,rotq));
                            input.m_transformA = MathUtil.BulletMatrixMultiplyBasis(Matrix.CreateFromQuaternion(temp),body0.GetWorldTransform());
                            input.m_transformB = body1.GetWorldTransform();
#if DEBUG_CONTACTS
				        dispatchInfo.m_debugDraw.DrawTransform(ref input.m_transformA,10.0f);
#endif //DEBUG_CONTACTS
                        }
                        else
                        {
                            input.m_transformA = body0.GetWorldTransform();
                            //input.m_transformB.setBasis( btMatrix3x3(rotq.inverse()*perturbeRot*rotq)*body1.getWorldTransform().getBasis());
                            Quaternion temp = MathUtil.QuaternionMultiply(MathUtil.QuaternionInverse(ref rotq),MathUtil.QuaternionMultiply(perturbeRot,rotq));
                            input.m_transformB = MathUtil.BulletMatrixMultiplyBasis(Matrix.CreateFromQuaternion(temp),body1.GetWorldTransform());
#if DEBUG_CONTACTS
				        dispatchInfo.m_debugDraw.DrawTransform(ref input.m_transformB,10.0f);
#endif
                        }

                        PerturbedContactResult perturbedResultOut = new PerturbedContactResult(resultOut, ref input.m_transformA, ref input.m_transformB, ref unPerturbedTransform, perturbeA, dispatchInfo.getDebugDraw());
                        gjkPairDetector.GetClosestPoints(input, perturbedResultOut, dispatchInfo.getDebugDraw(), false);
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

	        float squareMot0 = (body0.GetInterpolationWorldTransform().Translation - body0.GetWorldTransform().Translation).LengthSquared();
	        float squareMot1 = (body1.GetInterpolationWorldTransform().Translation - body1.GetWorldTransform().Translation).LengthSquared();
            
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
		        ConvexShape convex0 = (ConvexShape)(body0.GetCollisionShape());

		        SphereShape	sphere1 = new SphereShape(body1.GetCcdSweptSphereRadius()); //todo: allow non-zero sphere sizes, for better approximation
		        CastResult result = new CastResult();
		        VoronoiSimplexSolver voronoiSimplex = new VoronoiSimplexSolver();
		        //SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
		        ///Simplification, one object is simplified as a sphere
		        GjkConvexCast ccd1 = new GjkConvexCast( convex0 ,sphere1,voronoiSimplex);
		        //ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
		        if (ccd1.CalcTimeOfImpact(body0.GetWorldTransform(),body0.GetInterpolationWorldTransform(),
			        body1.GetWorldTransform(),body1.GetInterpolationWorldTransform(),result))
		        {
        		
			        //store result.m_fraction in both bodies
        		
			        if (body0.GetHitFraction()> result.m_fraction)
                    {
				        body0.SetHitFraction( result.m_fraction );
                    }
			        if (body1.GetHitFraction() > result.m_fraction)
                    {
				        body1.SetHitFraction( result.m_fraction);
                    }
			        if (resultFraction > result.m_fraction)
                    {
				        resultFraction = result.m_fraction;
                    }
		        }
	        }

	        /// Sphere (for convex0) against Convex1
	        {
		        ConvexShape convex1 = (ConvexShape)(body1.GetCollisionShape());

		        SphereShape	sphere0 = new SphereShape(body0.GetCcdSweptSphereRadius()); //todo: allow non-zero sphere sizes, for better approximation
		        CastResult result = new CastResult();
		        VoronoiSimplexSolver voronoiSimplex = new VoronoiSimplexSolver();
		        //SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
		        ///Simplification, one object is simplified as a sphere
		        GjkConvexCast ccd1 = new GjkConvexCast(sphere0,convex1,voronoiSimplex);
		        //ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
		        if (ccd1.CalcTimeOfImpact(body0.GetWorldTransform(),body0.GetInterpolationWorldTransform(),
			        body1.GetWorldTransform(),body1.GetInterpolationWorldTransform(),result))
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
	        }
        	
	        return resultFraction;
        }

	    public override void GetAllContactManifolds(IList<PersistentManifold> manifoldArray)
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
	        out Vector3 ptsVector,
	        out Vector3 offsetA,
	        out Vector3 offsetB,
	        out float tA, out float tB,
	        ref Vector3 translation,
	        ref Vector3 dirA, float hlenA,
	        ref Vector3 dirB, float hlenB )
        {
	        // compute the parameters of the closest points on each line segment

	        float dirA_dot_dirB = Vector3.Dot(dirA,dirB);
	        float dirA_dot_trans = Vector3.Dot(dirA,translation);
	        float dirB_dot_trans = Vector3.Dot(dirB,translation);

	        float denom = 1.0f - dirA_dot_dirB * dirA_dot_dirB;

	        if (MathUtil.FuzzyZero(denom)) 
            {
		        tA = 0.0f;
	        } 
            else 
            {
		        tA = ( dirA_dot_trans - dirB_dot_trans * dirA_dot_dirB ) / denom;
		        if ( tA < -hlenA )
                {
			        tA = -hlenA;
                }
		        else if ( tA > hlenA )
                {
			        tA = hlenA;
                }
	        }

	        tB = tA * dirA_dot_dirB - dirB_dot_trans;

	        if ( tB < -hlenB ) 
            {
		        tB = -hlenB;
		        tA = tB * dirA_dot_dirB + dirA_dot_trans;

		        if ( tA < -hlenA )
                {
			        tA = -hlenA;
                }
		        else if ( tA > hlenA )
                {
			        tA = hlenA;
                }
	        } 
            else if ( tB > hlenB ) 
            {
		        tB = hlenB;
		        tA = tB * dirA_dot_dirB + dirA_dot_trans;

		        if ( tA < -hlenA )
                {
			        tA = -hlenA;
                }
		        else if ( tA > hlenA )
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
			ref Vector3 normalOnB,
			ref Vector3 pointOnB,
			float capsuleLengthA,
			float capsuleRadiusA,
			float capsuleLengthB,
			float capsuleRadiusB,
			int capsuleAxisA,
			int capsuleAxisB,
			Matrix transformA,
			Matrix transformB,
			float distanceThreshold)
		{
			return CapsuleCapsuleDistance(ref normalOnB, ref pointOnB, capsuleLengthA,
				capsuleRadiusA, capsuleLengthB, capsuleRadiusB, capsuleAxisA, capsuleAxisB,
				ref transformA, ref transformB, distanceThreshold);
		}

        public static float CapsuleCapsuleDistance(
	        ref Vector3 normalOnB,
	        ref Vector3 pointOnB,
	        float capsuleLengthA,
	        float	capsuleRadiusA,
	        float capsuleLengthB,
	        float	capsuleRadiusB,
	        int capsuleAxisA,
	        int capsuleAxisB,
	        ref Matrix transformA,
	        ref Matrix transformB,
	        float distanceThreshold )
        {
	        Vector3 directionA = MathUtil.MatrixColumn(ref transformA,capsuleAxisA);
            Vector3 translationA = transformA.Translation;
	        Vector3 directionB = MathUtil.MatrixColumn(ref transformB,capsuleAxisB);
	        Vector3 translationB = transformB.Translation;

	        // translation between centers

	        Vector3 translation = translationB - translationA;

	        // compute the closest points of the capsule line segments

	        Vector3 ptsVector;           // the vector between the closest points

            Vector3 offsetA, offsetB;    // offsets from segment centers to their closest points
	        float tA, tB;              // parameters on line segment

            SegmentsClosestPoints(out ptsVector, out offsetA, out offsetB, out tA, out tB, ref translation,
						           ref directionA, capsuleLengthA, ref directionB, capsuleLengthB );

	        float distance = ptsVector.Length() - capsuleRadiusA - capsuleRadiusB;

            if (distance > distanceThreshold)
            {
                return distance;
            }

	        float lenSqr = ptsVector.LengthSquared();
            if (lenSqr <= (MathUtil.SIMD_EPSILON * MathUtil.SIMD_EPSILON))
	        {
		        //degenerate case where 2 capsules are likely at the same location: take a vector tangential to 'directionA'
		        Vector3 q;
		        TransformUtil.PlaneSpace1(ref directionA, out normalOnB, out q);
	        } 
            else
	        {
		        // compute the contact normal
		        normalOnB = ptsVector*-MathUtil.RecipSqrt(lenSqr);
	        }
	        pointOnB = transformB.Translation+offsetB + normalOnB * capsuleRadiusB;

	        return distance;
        }
    }



    public class ConvexConvexCreateFunc : CollisionAlgorithmCreateFunc
    {
        public ConvexConvexCreateFunc(ISimplexSolverInterface simplexSolver,IConvexPenetrationDepthSolver depthSolver)
        {
            m_numPerturbationIterations = 0;
            m_minimumPointsPerturbationThreshold =3;
            m_simplexSolver = simplexSolver;
            m_pdSolver = depthSolver;
        }

        public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
        {
            return new ConvexConvexAlgorithm(ci.GetManifold(), ci, body0, body1, m_simplexSolver, m_pdSolver, m_numPerturbationIterations, m_minimumPointsPerturbationThreshold);
        }
    	
        public IConvexPenetrationDepthSolver m_pdSolver;
		public ISimplexSolverInterface m_simplexSolver;
		public int m_numPerturbationIterations;
		public int m_minimumPointsPerturbationThreshold;
    }



    public class PerturbedContactResult : ManifoldResult
    {
	    public ManifoldResult m_originalManifoldResult;
	    public Matrix m_transformA;
	    public Matrix m_transformB;
	    public Matrix	m_unPerturbedTransform;
	    public bool	m_perturbA;
	    public IDebugDraw	m_debugDrawer;


	    public PerturbedContactResult(ManifoldResult originalResult,ref Matrix transformA,ref Matrix transformB,ref Matrix unPerturbedTransform,bool perturbA,IDebugDraw debugDrawer)
        {
		    m_originalManifoldResult = originalResult;
		    m_transformA = transformA;
		    m_transformB = transformB;
		    m_perturbA = perturbA;
		    m_unPerturbedTransform = unPerturbedTransform;
		    m_debugDrawer = debugDrawer;
	    }

	    public override void AddContactPoint(ref Vector3 normalOnBInWorld,ref Vector3 pointInWorld,float orgDepth)
	    {
		    Vector3 endPt,startPt;
		    float newDepth;
            Vector3 newNormal = Vector3.Up;

		    if (m_perturbA)
		    {
			    Vector3 endPtOrg = pointInWorld + normalOnBInWorld*orgDepth;
			    endPt = Vector3.Transform(endPtOrg,(MathUtil.BulletMatrixMultiply(m_unPerturbedTransform,Matrix.Invert(m_transformA))));
			    newDepth = Vector3.Dot((endPt -  pointInWorld),normalOnBInWorld);
			    startPt = endPt+normalOnBInWorld*newDepth;
		    } else
		    {
			    endPt = pointInWorld + normalOnBInWorld*orgDepth;
                startPt = Vector3.Transform(pointInWorld,(MathUtil.BulletMatrixMultiply(m_unPerturbedTransform,Matrix.Invert(m_transformB))));
			    newDepth = Vector3.Dot((endPt -  startPt),normalOnBInWorld);
		    }

    //#define DEBUG_CONTACTS 1
    #if DEBUG_CONTACTS
		    m_debugDrawer.DrawLine(startPt,endPt,new Vector3(1,0,0));
            m_debugDrawer.DrawSphere(startPt, 0.5f, new Vector3(0, 1, 0));
            m_debugDrawer.DrawSphere(endPt, 0.5f, new Vector3(0, 0, 1));
    #endif //DEBUG_CONTACTS

    		
		    m_originalManifoldResult.AddContactPoint(ref normalOnBInWorld,ref startPt,newDepth);
	    }

    }
}
