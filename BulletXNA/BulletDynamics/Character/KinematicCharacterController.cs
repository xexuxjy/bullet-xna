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
using BulletXNA.BulletCollision.CollisionDispatch;
using BulletXNA.BulletCollision.CollisionShapes;
using BulletXNA.BulletCollision.NarrowPhaseCollision;
using BulletXNA.LinearMath;
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletDynamics.Character
{
    public class KinematicCharacterController : ICharacterControllerInterface
    {
        protected Vector3 ComputeReflectionDirection(ref Vector3 direction, ref Vector3 normal)
        {
            return Vector3.Reflect(direction, normal);
        }

        protected Vector3 ParallelComponent(ref Vector3 direction, ref Vector3 normal)
        {
            float magnitude = Vector3.Dot(direction,normal);
            return normal * magnitude;
        }
        protected Vector3 PerpindicularComponent(ref Vector3 direction, ref Vector3 normal)
        {
            float magnitude = 1f-Vector3.Dot(direction, normal);
            return normal * magnitude;
        }

        protected bool RecoverFromPenetration(CollisionWorld collisionWorld)
        {
	        bool penetration = false;

	        collisionWorld.GetDispatcher().DispatchAllCollisionPairs(m_ghostObject.GetOverlappingPairCache(), collisionWorld.GetDispatchInfo(), collisionWorld.GetDispatcher());

	        m_currentPosition = m_ghostObject.GetWorldTransform().Translation;
        	
	        float maxPen = 0f;
	        for (int i = 0; i < m_ghostObject.GetOverlappingPairCache().GetNumOverlappingPairs(); i++)
	        {
		        m_manifoldArray.Clear();

		        BroadphasePair collisionPair = m_ghostObject.GetOverlappingPairCache().GetOverlappingPairArray()[i];
        		
		        if (collisionPair.m_algorithm != null)
                {
			        collisionPair.m_algorithm.GetAllContactManifolds(m_manifoldArray);
                }
        		
		        for (int j=0;j<m_manifoldArray.Count;j++)
		        {
			        PersistentManifold manifold = m_manifoldArray[j];
			        float directionSign = manifold.GetBody0() == m_ghostObject ? -1f : 1f;
			        for (int p=0;p<manifold.GetNumContacts();p++)
			        {
				        ManifoldPoint pt = manifold.GetContactPoint(p);

				        if (pt.GetDistance() < 0.0f)
				        {
					        if (pt.GetDistance() < maxPen)
					        {
						        maxPen = pt.GetDistance();
						        m_touchingNormal = pt.GetNormalWorldOnB() * directionSign;//??

					        }
                            m_currentPosition += pt.GetNormalWorldOnB() * directionSign * pt.GetDistance() * 0.2f;
					        penetration = true;
				        } else 
                        {
					        //printf("touching %f\n", pt.getDistance());
				        }
			        }
        			
			        //manifold->clearManifold();
		        }
	        }
	        Matrix newTrans = m_ghostObject.GetWorldTransform();
	        newTrans.Translation  = m_currentPosition;
	        m_ghostObject.SetWorldTransform(ref newTrans);
        //	printf("m_touchingNormal = %f,%f,%f\n",m_touchingNormal[0],m_touchingNormal[1],m_touchingNormal[2]);
	        return penetration;
        }

        protected void StepUp(CollisionWorld collisionWorld)
        {
	        // phase 1: up
	        Matrix start = Matrix.Identity, end = Matrix.Identity;
	        m_targetPosition = m_currentPosition + upAxisDirection[m_upAxis] * m_stepHeight;


	        /* FIXME: Handle penetration properly */
	        start.Translation = (m_currentPosition + upAxisDirection[m_upAxis] * 0.1f);
	        end.Translation = m_targetPosition;

	        KinematicClosestNotMeConvexResultCallback callback = new KinematicClosestNotMeConvexResultCallback(m_ghostObject);
	        callback.m_collisionFilterGroup = GetGhostObject().GetBroadphaseHandle().m_collisionFilterGroup;
	        callback.m_collisionFilterMask = GetGhostObject().GetBroadphaseHandle().m_collisionFilterMask;
        	
	        if (m_useGhostObjectSweepTest)
	        {
                m_ghostObject.ConvexSweepTest(m_convexShape, ref start, ref end, callback, collisionWorld.GetDispatchInfo().GetAllowedCcdPenetration());
	        }
	        else
	        {
                collisionWorld.ConvexSweepTest(m_convexShape, ref start, ref end, callback,0f);
	        }
        	
	        if (callback.hasHit())
	        {
		        // we moved up only a fraction of the step height
		        m_currentStepOffset = m_stepHeight * callback.m_closestHitFraction;
                m_currentPosition = MathUtil.Interpolate3(ref m_currentPosition, ref m_targetPosition, callback.m_closestHitFraction);
	        } 
            else 
            {
		        m_currentStepOffset = m_stepHeight;
		        m_currentPosition = m_targetPosition;
	        }

        }
        protected void UpdateTargetPositionBasedOnCollision(ref Vector3 hitNormal, float tangentMag, float normalMag)
        {
            Vector3 movementDirection = m_targetPosition - m_currentPosition;
            float movementLength = movementDirection.Length();
            if (movementLength > MathUtil.SIMD_EPSILON)
            {
                movementDirection.Normalize();

                Vector3 reflectDir = ComputeReflectionDirection(ref movementDirection, ref hitNormal);
                reflectDir.Normalize();

                Vector3 parallelDir, perpindicularDir;

                parallelDir = ParallelComponent(ref reflectDir, ref hitNormal);
                perpindicularDir = PerpindicularComponent(ref reflectDir, ref hitNormal);

                m_targetPosition = m_currentPosition;
                if (false)//tangentMag != 0.0)
                {
                    Vector3 parComponent = parallelDir * (tangentMag * movementLength);
                    //			printf("parComponent=%f,%f,%f\n",parComponent[0],parComponent[1],parComponent[2]);
                    m_targetPosition += parComponent;
                }

                if (normalMag != 0.0f)
                {
                    Vector3 perpComponent = perpindicularDir * (normalMag * movementLength);
                    //			printf("perpComponent=%f,%f,%f\n",perpComponent[0],perpComponent[1],perpComponent[2]);
                    m_targetPosition += perpComponent;
                }
            }
            else
            {
                //		printf("movementLength don't normalize a zero vector\n");
            }
        }

        protected void StepForwardAndStrafe(CollisionWorld collisionWorld, ref Vector3 walkMove)
        {
        //	printf("originalDir=%f,%f,%f\n",originalDir[0],originalDir[1],originalDir[2]);
	        // phase 2: forward and strafe
	        Matrix start = Matrix.Identity, end = Matrix.Identity;
	        m_targetPosition = m_currentPosition + walkMove;
        	
	        float fraction = 1.0f;
	        float distance2 = (m_currentPosition-m_targetPosition).LengthSquared();
        //	printf("distance2=%f\n",distance2);

	        if (m_touchingContact)
	        {
		        if (Vector3.Dot(m_normalizedDirection,m_touchingNormal) > 0.0f)
                {
			        UpdateTargetPositionBasedOnCollision (ref m_touchingNormal,0.0f,1.0f);
                }
	        }

	        int maxIter = 10;

	        while (fraction > 0.01f && maxIter-- > 0)
	        {
		        start.Translation = (m_currentPosition);
		        end.Translation = (m_targetPosition);

		        KinematicClosestNotMeConvexResultCallback callback = new KinematicClosestNotMeConvexResultCallback(m_ghostObject);
		        callback.m_collisionFilterGroup = GetGhostObject().GetBroadphaseHandle().m_collisionFilterGroup;
		        callback.m_collisionFilterMask = GetGhostObject().GetBroadphaseHandle().m_collisionFilterMask;


		        float margin = m_convexShape.GetMargin();
		        m_convexShape.SetMargin(margin + m_addedMargin);


		        if (m_useGhostObjectSweepTest)
		        {
			        m_ghostObject.ConvexSweepTest (m_convexShape, ref start, ref end, callback, collisionWorld.GetDispatchInfo().GetAllowedCcdPenetration());
		        } else
		        {
			        collisionWorld.ConvexSweepTest (m_convexShape, ref start, ref end, callback, collisionWorld.GetDispatchInfo().GetAllowedCcdPenetration());
		        }
        		
		        m_convexShape.SetMargin(margin);

        		
		        fraction -= callback.m_closestHitFraction;

		        if (callback.hasHit())
		        {	
			        // we moved only a fraction
			        float hitDistance = (callback.m_hitPointWorld - m_currentPosition).Length();
			        if (hitDistance<0.0f)
			        {
        //				printf("neg dist?\n");
			        }

			        /* If the distance is farther than the collision margin, move */
			        if (hitDistance > m_addedMargin)
			        {
        //				printf("callback.m_closestHitFraction=%f\n",callback.m_closestHitFraction);
				        m_currentPosition = MathUtil.Interpolate3(ref m_currentPosition, ref m_targetPosition, callback.m_closestHitFraction);
			        }
                    
			        UpdateTargetPositionBasedOnCollision (ref callback.m_hitNormalWorld,0f,1f);
			        Vector3 currentDir = m_targetPosition - m_currentPosition;
			        distance2 = currentDir.LengthSquared();
			        if (distance2 > MathUtil.SIMD_EPSILON)
			        {
				        currentDir.Normalize();
				        /* See Quake2: "If velocity is against original velocity, stop ead to avoid tiny oscilations in sloping corners." */
                        if (Vector3.Dot(currentDir, m_normalizedDirection) <= 0.0f)
				        {
					        break;
				        }
			        } 
                    else
			        {
        //				printf("currentDir: don't normalize a zero vector\n");
				        break;
			        }
		        } 
                else 
                {
			        // we moved whole way
			        m_currentPosition = m_targetPosition;
		        }

	        //	if (callback.m_closestHitFraction == 0.f)
	        //		break;

	        }

        }
        protected void StepDown(CollisionWorld collisionWorld, float dt)
        {
	        Matrix start= Matrix.Identity, end = Matrix.Identity;

	        // phase 3: down
	        Vector3 step_drop = upAxisDirection[m_upAxis] * m_currentStepOffset;
	        Vector3 gravity_drop = upAxisDirection[m_upAxis] * m_stepHeight; 
	        m_targetPosition -= (step_drop + gravity_drop);


	        start.Translation = m_currentPosition;
	        end.Translation = m_targetPosition;

	        KinematicClosestNotMeConvexResultCallback callback = new KinematicClosestNotMeConvexResultCallback(m_ghostObject);
	        callback.m_collisionFilterGroup = GetGhostObject().GetBroadphaseHandle().m_collisionFilterGroup;
	        callback.m_collisionFilterMask = GetGhostObject().GetBroadphaseHandle().m_collisionFilterMask;
        	
	        if (m_useGhostObjectSweepTest)
	        {
		        m_ghostObject.ConvexSweepTest (m_convexShape, ref start, ref end, callback, collisionWorld.GetDispatchInfo().GetAllowedCcdPenetration());
	        } 
            else
	        {
                collisionWorld.ConvexSweepTest(m_convexShape, start, end, callback, collisionWorld.GetDispatchInfo().GetAllowedCcdPenetration());
	        }

	        if (callback.hasHit())
	        {
		        // we dropped a fraction of the height -> hit floor
		        m_currentPosition = MathUtil.Interpolate3(ref m_currentPosition, ref m_targetPosition, callback.m_closestHitFraction);
	        } 
            else 
            {
		        // we dropped the full height
		        m_currentPosition = m_targetPosition;
	        }
        }

        public KinematicCharacterController(PairCachingGhostObject ghostObject, ConvexShape convexShape, float stepHeight, int upAxis)
        {
            m_upAxis = upAxis;
            m_addedMargin = 0.02f;
            m_walkDirection = Vector3.Zero;
            m_useGhostObjectSweepTest = true;
            m_ghostObject = ghostObject;
            m_stepHeight = stepHeight;
            m_turnAngle = 0f;
            m_convexShape = convexShape;
            m_useWalkDirection = true;	// use walk direction by default, legacy behavior
            m_velocityTimeInterval = 0.0f;
        }
	
        ///btActionInterface interface
	    public virtual void UpdateAction( CollisionWorld collisionWorld,float deltaTime)
	    {
		    PreStep ( collisionWorld);
		    PlayerStep (collisionWorld, deltaTime);
	    }
	
	    ///btActionInterface interface
        public void DebugDraw(IDebugDraw debugDrawer)
        {
        }

	    public void SetUpAxis(int axis)
	    {
		    if (axis < 0)
			    axis = 0;
		    if (axis > 2)
			    axis = 2;
		    m_upAxis = axis;
	    }

        public virtual void SetWalkDirection(ref Vector3 walkDirection)
	    {
            m_useWalkDirection = true;
            m_walkDirection = walkDirection;
            m_normalizedDirection = GetNormalizedVector(ref m_walkDirection);
        }

        public void SetVelocityForTimeInterval(ref Vector3 velocity,float timeInterval)
        {
        //	printf("setVelocity!\n");
        //	printf("  interval: %f\n", timeInterval);
        //	printf("  velocity: (%f, %f, %f)\n",
        //	    velocity.x(), velocity.y(), velocity.z());

	        m_useWalkDirection = false;
	        m_walkDirection = velocity;
	        m_normalizedDirection = GetNormalizedVector(ref m_walkDirection);
	        m_velocityTimeInterval = timeInterval;
        }



        public void Reset()
        {
        }

        public void Warp(ref Vector3 origin)
        {
            Matrix m = Matrix.CreateTranslation(origin);
            m_ghostObject.SetWorldTransform(ref m);
        }

        public void PreStep(CollisionWorld collisionWorld)
        {
            int numPenetrationLoops = 0;
            m_touchingContact = false;
            while (RecoverFromPenetration(collisionWorld))
            {
                numPenetrationLoops++;
                m_touchingContact = true;
                if (numPenetrationLoops > 4)
                {
                    //			printf("character could not recover from penetration = %d\n", numPenetrationLoops);
                    break;
                }
            }

            m_currentPosition = m_ghostObject.GetWorldTransform().Translation;
            m_targetPosition = m_currentPosition;

        }

        public void PlayerStep(CollisionWorld collisionWorld, float dt)
        {
            // quick check...
            if (!m_useWalkDirection && m_velocityTimeInterval <= 0.0)
            {
                //		printf("\n");
                return;		// no motion
            }

            Matrix xform = m_ghostObject.GetWorldTransform();

            //	printf("walkDirection(%f,%f,%f)\n",walkDirection[0],walkDirection[1],walkDirection[2]);
            //	printf("walkSpeed=%f\n",walkSpeed);

            StepUp(collisionWorld);
            if (m_useWalkDirection)
            {
                StepForwardAndStrafe(collisionWorld, ref m_walkDirection);
            }
            else
            {
                //printf("  time: %f", m_velocityTimeInterval);
                // still have some time left for moving!
                float dtMoving =
                   (dt < m_velocityTimeInterval) ? dt : m_velocityTimeInterval;
                m_velocityTimeInterval -= dt;

                // how far will we move while we are moving?
                Vector3 move = m_walkDirection * dtMoving;

                // printf("  dtMoving: %f", dtMoving);

                // okay, step
                StepForwardAndStrafe(collisionWorld, ref move);
            }
            StepDown(collisionWorld, dt);

            xform.Translation = m_currentPosition;
            m_ghostObject.SetWorldTransform(ref xform);
        }

        public void SetFallSpeed(float fallSpeed)
        {
            m_fallSpeed = fallSpeed;
        }

        public void SetJumpSpeed(float jumpSpeed)
        {
            m_jumpSpeed = jumpSpeed;
        }

        public void SetMaxJumpHeight(float maxJumpHeight)
        {
            m_maxJumpHeight = maxJumpHeight;
        }

        public bool CanJump()
        {
            return OnGround();
        }
        
        public void Jump()
        {
            if (CanJump())
            {
                //currently no jumping.
                //Matrix xform;
                //m_rigidBody.getMotionState().getWorldTransform (out xform);
                //Vector3 up = xform.Up;
                //up.Normalize ();
                //float magnitude = (1.0f/m_rigidBody.getInvMass()) * 8.0f;
                //m_rigidBody.applyCentralImpulse (up * magnitude);
            }

        }

        public PairCachingGhostObject GetGhostObject()
        {
            return m_ghostObject;
        }

        public void SetUseGhostSweepTest(bool useGhostObjectSweepTest)
	    {
		    m_useGhostObjectSweepTest = useGhostObjectSweepTest;
	    }

        public bool OnGround()
        {
            return true;
        }

        protected float m_halfHeight;

        protected PairCachingGhostObject m_ghostObject;
        protected ConvexShape m_convexShape;//is also in m_ghostObject, but it needs to be convex, so we store it here to avoid upcast

        protected float m_fallSpeed;
        protected float m_jumpSpeed;
        protected float m_maxJumpHeight;

        protected float m_turnAngle;

        protected float m_stepHeight;

        protected float m_addedMargin;//@todo: remove this and fix the code

        ///this is the desired walk direction, set by the user
        protected Vector3 m_walkDirection;
        protected Vector3 m_normalizedDirection;

        //some internal variables
        protected Vector3 m_currentPosition;
        float m_currentStepOffset;
        protected Vector3 m_targetPosition;

        ///keep track of the contact manifolds
        protected IList<PersistentManifold> m_manifoldArray;

        protected bool m_touchingContact;
        protected Vector3 m_touchingNormal;

        protected bool m_useGhostObjectSweepTest;

        protected int m_upAxis;
        protected bool m_useWalkDirection;
        protected float m_velocityTimeInterval;

        public static Vector3 GetNormalizedVector(ref Vector3 v)
        {
	        Vector3 n = Vector3.Normalize(v);
	        if (n.Length() < MathUtil.SIMD_EPSILON) 
            {
                n = Vector3.Zero;
	        }
	        return n;
        }

        protected static Vector3[] upAxisDirection = { Vector3.Right, Vector3.Up, Vector3.Backward };

    }


    ///@todo Interact with dynamic objects,
    ///Ride kinematicly animated platforms properly
    ///More realistic (or maybe just a config option) falling
    /// -> Should integrate falling velocity manually and use that in stepDown()
    ///Support jumping
    ///Support ducking
    public class KinematicClosestNotMeRayResultCallback : ClosestRayResultCallback
    {
        
	    public KinematicClosestNotMeRayResultCallback (CollisionObject me) : base(Vector3.Zero,Vector3.Zero)
	    {
		    m_me = me;
	    }

	    public override float AddSingleResult(LocalRayResult rayResult,bool normalInWorldSpace)
	    {
		    if (rayResult.m_collisionObject == m_me)
			    return 1.0f;

		    return base.AddSingleResult (rayResult, normalInWorldSpace);
	    }
    
	    protected CollisionObject m_me;
    }

    public class KinematicClosestNotMeConvexResultCallback : ClosestConvexResultCallback
    {
    
	    public KinematicClosestNotMeConvexResultCallback (CollisionObject me) : base(Vector3.Zero,Vector3.Zero)
	    {
		    m_me = me;
	    }

	    public override float AddSingleResult(LocalConvexResult convexResult,bool normalInWorldSpace)
	    {
		    if (convexResult.m_hitCollisionObject == m_me)
			    return 1.0f;

		    return base.AddSingleResult (convexResult, normalInWorldSpace);
	    }
    
	    protected CollisionObject m_me;
    }


}
