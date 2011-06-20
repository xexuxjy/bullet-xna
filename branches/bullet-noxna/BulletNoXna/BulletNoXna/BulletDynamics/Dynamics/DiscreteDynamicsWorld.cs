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

using System;
using System.Collections.Generic;
using System.Diagnostics;
using BulletXNA.BulletCollision;
using BulletXNA.LinearMath;
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletDynamics
{
    public class DiscreteDynamicsWorld : DynamicsWorld
    {
	    ///this btDiscreteDynamicsWorld constructor gets created objects from the user, and will not delete those
	    public DiscreteDynamicsWorld(IDispatcher dispatcher,IBroadphaseInterface pairCache,IConstraintSolver constraintSolver,ICollisionConfiguration collisionConfiguration) : base(dispatcher,pairCache,collisionConfiguration)
        {
            m_ownsIslandManager = true;
            m_constraints = new ObjectArray<TypedConstraint>();
            m_actions = new List<IActionInterface>();
            m_nonStaticRigidBodies = new ObjectArray<RigidBody>();
            m_islandManager = new SimulationIslandManager();
            m_constraintSolver = constraintSolver;

            Vector3 gravity = new Vector3(0, -10, 0);
            SetGravity(ref gravity);
            m_localTime = 0f;
            m_profileTimings = 0;
            m_synchronizeAllMotionStates = false;

	        if (m_constraintSolver == null)
	        {
		        m_constraintSolver = new SequentialImpulseConstraintSolver();
		        m_ownsConstraintSolver = true;
	        } 
            else
	        {
		        m_ownsConstraintSolver = false;
	        }
        }

        public override void Cleanup()
        {
            base.Cleanup();
	        //only delete it when we created it
	        if (m_ownsIslandManager)
	        {
                m_islandManager.Cleanup();
                m_islandManager = null;
                m_ownsIslandManager = false;
	        }
	        if (m_ownsConstraintSolver)
	        {
                m_constraintSolver.Cleanup();
                m_constraintSolver = null;
                m_ownsConstraintSolver = false;
	        }
        }

	    ///if maxSubSteps > 0, it will interpolate motion between fixedTimeStep's
        public override int StepSimulation(float timeStep, int maxSubSteps)
        {
            return StepSimulation(timeStep, maxSubSteps, s_fixedTimeStep);
        }

        public override int StepSimulation(float timeStep, int maxSubSteps, float fixedTimeStep)
        {
	        StartProfiling(timeStep);

            BulletGlobals.StartProfile("stepSimulation");

	        int numSimulationSubSteps = 0;

	        if (maxSubSteps != 0)
	        {
		        //fixed timestep with interpolation
		        m_localTime += timeStep;
		        if (m_localTime >= fixedTimeStep)
		        {
			        numSimulationSubSteps = (int)( m_localTime / fixedTimeStep);
			        m_localTime -= numSimulationSubSteps * fixedTimeStep;
		        }
	        } else
	        {
		        //variable timestep
		        fixedTimeStep = timeStep;
		        m_localTime = timeStep;
		        if (MathUtil.FuzzyZero(timeStep))
		        {
			        numSimulationSubSteps = 0;
			        maxSubSteps = 0;
		        } else
		        {
			        numSimulationSubSteps = 1;
			        maxSubSteps = 1;
		        }
	        }

	        //process some debugging flags
	        if (GetDebugDrawer() != null)
	        {
		        IDebugDraw debugDrawer = GetDebugDrawer ();
		        BulletGlobals.gDisableDeactivation = ((debugDrawer.GetDebugMode() & DebugDrawModes.DBG_NoDeactivation) != 0);
	        }
	        if (numSimulationSubSteps != 0)
	        {

		        //clamp the number of substeps, to prevent simulation grinding spiralling down to a halt
		        int clampedSimulationSteps = (numSimulationSubSteps > maxSubSteps)? maxSubSteps : numSimulationSubSteps;

                if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugDiscreteDynamicsWorld)
                {
                    BulletGlobals.g_streamWriter.WriteLine(String.Format("Stepsimulation numClamped[{0}] timestep[{1:0.00000}]", clampedSimulationSteps, fixedTimeStep));
                }

		        SaveKinematicState(fixedTimeStep*clampedSimulationSteps);

		        ApplyGravity();

		        for (int i=0;i<clampedSimulationSteps;i++)
		        {
			        InternalSingleStepSimulation(fixedTimeStep);
			        SynchronizeMotionStates();
		        }

	        } 
            else
            {
                SynchronizeMotionStates();
            }
	        ClearForces();

            if (m_profileManager != null)
            {
                m_profileManager.Increment_Frame_Counter();
            }
        	
	        return numSimulationSubSteps;
        }


	    public override void SynchronizeMotionStates()
        {
            if (m_synchronizeAllMotionStates)
            {
                //iterate  over all collision objects
				int length = m_collisionObjects.Count;
				for (int i = 0; i < length;++i )
				{
                    RigidBody body = RigidBody.Upcast(m_collisionObjects[i]);
                    if (body != null)
                    {
                        SynchronizeSingleMotionState(body);
                    }
                }
            }
            else
            {
		        //iterate over all active rigid bodies
				int length = m_nonStaticRigidBodies.Count;
		        for(int i=0;i<length;++i)
		        {
					RigidBody body =  m_nonStaticRigidBodies[i];
			        if (body.IsActive())
                    {
				        SynchronizeSingleMotionState(body);
                    }
		        }
            }
        }

	    ///this can be useful to synchronize a single rigid body . graphics object
	    public void	SynchronizeSingleMotionState(RigidBody body)
        {
	        Debug.Assert(body != null);

	        if (body.MotionState != null && !body.IsStaticOrKinematicObject)
	        {
		        //we need to call the update at least once, even for sleeping objects
		        //otherwise the 'graphics' transform never updates properly
		        ///@todo: add 'dirty' flag
		        //if (body.ActivationState != IslandSleeping)
		        {
			        Matrix interpolatedTransform;
			        TransformUtil.IntegrateTransform(body.GetInterpolationWorldTransform(),
				        body.SetInterpolationLinearVelocity(),body.GetInterpolationAngularVelocity(),
                        m_localTime*body.GetHitFraction(), out interpolatedTransform);
			        body.MotionState.SetWorldTransform(ref interpolatedTransform);
		        }
	        }
        }

	    public override void AddConstraint(TypedConstraint constraint, bool disableCollisionsBetweenLinkedBodies)
        {
            //if (constraint is ConeTwistConstraint)
			//if (constraint is HingeConstraint)
			//{
			//    return;
			//}
	        m_constraints.Add(constraint);
	        if (disableCollisionsBetweenLinkedBodies)
	        {
		        constraint.GetRigidBodyA().AddConstraintRef(constraint);
		        constraint.GetRigidBodyB().AddConstraintRef(constraint);
	        }
        }

	    public override void RemoveConstraint(TypedConstraint constraint)
        {
	        m_constraints.Remove(constraint);
	        constraint.GetRigidBodyA().RemoveConstraintRef(constraint);
	        constraint.GetRigidBodyB().RemoveConstraintRef(constraint);
        }

	    public override void AddAction(IActionInterface action)
        {
            m_actions.Add(action);
        }

	    public override void RemoveAction(IActionInterface action)
        {
            m_actions.Remove(action);
        }
    	
	    public SimulationIslandManager	GetSimulationIslandManager()
	    {
		    return m_islandManager;
	    }

	    public CollisionWorld GetCollisionWorld()
	    {
		    return this;
	    }

	    public override void SetGravity(ref Vector3 gravity)
        {
            m_gravity = gravity;
			int length = m_nonStaticRigidBodies.Count;
            for(int i=0;i<length;++i)
            {
				RigidBody body = m_nonStaticRigidBodies[i];
                if(body.IsActive() && ((body.GetFlags() & RigidBodyFlags.BT_DISABLE_WORLD_GRAVITY) != 0))
		        {
			        body.SetGravity(ref gravity);
		        }
	        }
        }

	    public override Vector3 GetGravity ()
        {
            return m_gravity;
        }
        public override void AddCollisionObject(CollisionObject collisionObject)
        {
            AddCollisionObject(collisionObject, CollisionFilterGroups.StaticFilter, CollisionFilterGroups.AllFilter ^ CollisionFilterGroups.StaticFilter);
        }
        public override void AddCollisionObject(CollisionObject collisionObject,CollisionFilterGroups collisionFilterGroup,CollisionFilterGroups collisionFilterMask)
        {
            base.AddCollisionObject(collisionObject,collisionFilterGroup,collisionFilterMask);
        }

        ///removeCollisionObject will first check if it is a rigid body, if so call removeRigidBody otherwise call btCollisionWorld::removeCollisionObject
	    public override void RemoveCollisionObject(CollisionObject collisionObject)
        {
	        RigidBody body = RigidBody.Upcast(collisionObject);
	        if (body != null)
            {
		        RemoveRigidBody(body);
            }
	        else
            {
		        base.RemoveCollisionObject(collisionObject);
            }
        }

        public void	SetSynchronizeAllMotionStates(bool synchronizeAll)
	    {
		    m_synchronizeAllMotionStates = synchronizeAll;
	    }

	    public bool GetSynchronizeAllMotionStates()
	    {
		    return m_synchronizeAllMotionStates;
	    }


	    public override void AddRigidBody(RigidBody body)
        {
            if (!body.IsStaticOrKinematicObject && 0 == (body.GetFlags() & RigidBodyFlags.BT_DISABLE_WORLD_GRAVITY))
	        {
		        body.SetGravity(ref m_gravity);
	        }

	        if (body.CollisionShape != null)
	        {
                if (!body.IsStaticObject)
		        {
			        m_nonStaticRigidBodies.Add(body);
		        } 
                else
		        {
			        body.ActivationState = ActivationState.IslandSleeping;
		        }

		        bool isDynamic = !(body.IsStaticObject || body.IsKinematicObject);
		        CollisionFilterGroups collisionFilterGroup = isDynamic? CollisionFilterGroups.DefaultFilter : CollisionFilterGroups.StaticFilter;
		        CollisionFilterGroups collisionFilterMask = isDynamic? 	CollisionFilterGroups.AllFilter : 	(CollisionFilterGroups.AllFilter ^ CollisionFilterGroups.StaticFilter);

		        AddCollisionObject(body,collisionFilterGroup,collisionFilterMask);
	        }
        }

        public override void AddRigidBody(RigidBody body, CollisionFilterGroups group, CollisionFilterGroups mask)
        {
	        if (!body.IsStaticOrKinematicObject  && 0 == (body.GetFlags() & RigidBodyFlags.BT_DISABLE_WORLD_GRAVITY))
	        {
		        body.SetGravity(ref m_gravity);
	        }

	        if (body.CollisionShape != null)
	        {
		        if (!body.IsStaticObject)
		        {
			        m_nonStaticRigidBodies.Add(body);
		        }
		         else
		        {
			        body.ActivationState = ActivationState.IslandSleeping;
		        }

		        AddCollisionObject(body,group,mask);
	        }
        }

	    public override void RemoveRigidBody(RigidBody body)
        {
	        m_nonStaticRigidBodies.Remove(body);
	        base.RemoveCollisionObject(body);
        }

	    public override void DebugDrawWorld()
        {
            BulletGlobals.StartProfile("debugDrawWorld");
            
            //if (getDebugDrawer() != null && ((getDebugDrawer().getDebugMode() & DebugDrawModes.DBG_DrawContactPoints) != 0))
            //{
            //    int numManifolds = getDispatcher().getNumManifolds();
            //    Vector3 color = Vector3.Zero;
            //    for (int i=0;i<numManifolds;i++)
            //    {
            //        PersistentManifold contactManifold = getDispatcher().getManifoldByIndexInternal(i);
            //        //btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold.getBody0());
            //        //btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold.getBody1());

            //        int numContacts = contactManifold.getNumContacts();
            //        for (int j=0;j<numContacts;j++)
            //        {
            //            ManifoldPoint cp = contactManifold.getContactPoint(j);
            //            getDebugDrawer().drawContactPoint(cp.getPositionWorldOnB(),cp.getNormalWorldOnB(),cp.getDistance(),cp.getLifeTime(),color);
            //        }
            //    }
            //}
	        bool drawConstraints = false;
	        if (GetDebugDrawer() != null)
	        {
                DebugDrawModes mode = GetDebugDrawer().GetDebugMode();
		        if((mode  & (DebugDrawModes.DBG_DrawConstraints | DebugDrawModes.DBG_DrawConstraintLimits)) != 0)
		        {
			        drawConstraints = true;
		        }
	        }

            if(drawConstraints)
	        {
		        for(int i = NumConstraints-1; i>=0 ;i--)
		        {
			        TypedConstraint constraint = GetConstraint(i);
			        //DrawHelper.DebugDrawConstraint(constraint,GetDebugDrawer());
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
                            switch (colObj.ActivationState)
                            {
                                case ActivationState.ActiveTag:
                                    {
                                        color = new Vector3(255, 255, 255);
                                        break;
                                    }
                                case ActivationState.IslandSleeping:
                                    {
                                        color = new Vector3(0, 255, 0);
                                        break;
                                    }
                                case ActivationState.WantsDeactivation:
                                    {
                                        color = new Vector3(0, 255, 255);
                                        break;
                                    }
                                case ActivationState.DisableDeactivation:
                                    {
                                        color = new Vector3(255, 0, 0);
                                        break;
                                    }
                                case ActivationState.DisableSimulation:
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
                            //DrawHelper.debugDrawObject(ref transform, colObj.CollisionShape, ref color, getDebugDrawer());
                        }
                        if (aabb)
                        {
                            Vector3 minAabb;
                            Vector3 maxAabb;
                            Vector3 colorvec = new Vector3(1, 0, 0);
                            colObj.CollisionShape.GetAabb(colObj.GetWorldTransform(), out minAabb, out maxAabb);
                            m_debugDrawer.DrawAabb(ref minAabb, ref maxAabb, ref colorvec);
                        }

                    }
        	
		            if (debugMode != 0)
		            {
						int LengthSquared = m_actions.Count;
			            for(int i=0;i<LengthSquared;++i)
			            {
				            m_actions[i].DebugDraw(m_debugDrawer);
			            }
		            }
                }
                BulletGlobals.StopProfile();
            }
        }

	    public override void SetConstraintSolver(IConstraintSolver solver)
        {
            //if (m_ownsConstraintSolver)
            //{
            //    btAlignedFree( m_constraintSolver);
            //}
	        m_ownsConstraintSolver = false;
	        m_constraintSolver = solver;
        }

	    public override IConstraintSolver GetConstraintSolver()
        {
            return m_constraintSolver;
        }
    	
	    public override int	NumConstraints
        {
            get
            {
                return m_constraints.Count;
            }
        }

	    public override TypedConstraint GetConstraint(int index)
        {
            return m_constraints[index];
        }
    	
	    public override DynamicsWorldType GetWorldType()
	    {
		    return DynamicsWorldType.BT_DISCRETE_DYNAMICS_WORLD;
	    }
    	
	    ///the forces on each rigidbody is accumulating together with gravity. clear this after each timestep.
	    public override void ClearForces()
        {
			int length = m_nonStaticRigidBodies.Count;
            for(int i=0;i<length;++i)
            {
		        m_nonStaticRigidBodies[i].ClearForces();
	        }
        }

	    ///apply gravity, call this once per timestep
	    public virtual void ApplyGravity()
        {
			int length = m_nonStaticRigidBodies.Count;
			for (int i = 0; i < length; ++i)
			{
				RigidBody body = m_nonStaticRigidBodies[i];
		        if (body != null && body.IsActive())
		        {
			        body.ApplyGravity();
		        }
	        }
        }

	    public virtual void SetNumTasks(int numTasks)
	    {
            //(void) numTasks;
	    }

	    ///obsolete, use updateActions instead
	    public virtual void UpdateVehicles(float timeStep)
	    {
		    UpdateActions(timeStep);
	    }

	    ///obsolete, use addAction instead
        public override void AddVehicle(IActionInterface vehicle)
        {
            AddAction(vehicle);
        }
	    ///obsolete, use removeAction instead
        public override void RemoveVehicle(IActionInterface vehicle)
        {
            RemoveAction(vehicle);
        }

	    ///obsolete, use addAction instead
        public override void AddCharacter(IActionInterface character)
        {
            AddAction(character);
        }
	    ///obsolete, use removeAction instead
        public override void RemoveCharacter(IActionInterface character)
        {
            RemoveAction(character);
        }

	    protected virtual void PredictUnconstraintMotion(float timeStep)
        {
            BulletGlobals.StartProfile("predictUnconstraintMotion");
			int length = m_nonStaticRigidBodies.Count;
            for (int i = 0; i < length; ++i)
            {
                RigidBody body = m_nonStaticRigidBodies[i];
                if (!body.IsStaticOrKinematicObject)
                {
                    body.IntegrateVelocities(timeStep);
                    //damping
                    body.ApplyDamping(timeStep);
                    Matrix temp;
                    body.PredictIntegratedTransform(timeStep, out temp);
                    body.SetInterpolationWorldTransform(ref temp);
                }
            }
            BulletGlobals.StopProfile();
        }

		protected virtual void IntegrateTransforms(float timeStep)
		{
			BulletGlobals.StartProfile("integrateTransforms");
			if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugDiscreteDynamicsWorld)
			{
				BulletGlobals.g_streamWriter.WriteLine("IntegrateTransforms");
			}

			Matrix predictedTrans;
			int length = m_nonStaticRigidBodies.Count;
			for (int i = 0; i < length; ++i)
			{
				RigidBody body = m_nonStaticRigidBodies[i];
				if (body != null)
				{
					body.SetHitFraction(1f);

					if (body.IsActive() && (!body.IsStaticOrKinematicObject))
					{
						body.PredictIntegratedTransform(timeStep, out predictedTrans);
						float squareMotion = (predictedTrans.Translation - body.GetWorldTransform().Translation).LengthSquared();

						//if (body.GetCcdSquareMotionThreshold() != 0 && body.GetCcdSquareMotionThreshold() < squareMotion)
						if (GetDispatchInfo().m_useContinuous && body.GetCcdSquareMotionThreshold() != 0.0f && body.GetCcdSquareMotionThreshold() < squareMotion)
						{
							BulletGlobals.StartProfile("CCD motion clamping");

							if (body.CollisionShape.IsConvex())
							{
								gNumClampedCcdMotions++;
#if USE_STATIC_ONLY
					class StaticOnlyCallback : public btClosestNotMeConvexResultCallback
					{
					public:

						StaticOnlyCallback (btCollisionObject* me,const btVector3& fromA,const btVector3& toA,btOverlappingPairCache* pairCache,btDispatcher* dispatcher) : 
						  btClosestNotMeConvexResultCallback(me,fromA,toA,pairCache,dispatcher)
						{
						}

					  	virtual bool needsCollision(btBroadphaseProxy* proxy0) const
						{
							btCollisionObject* otherObj = (btCollisionObject*) proxy0.m_clientObject;
							if (!otherObj.isStaticOrKinematicObject())
								return false;
							return btClosestNotMeConvexResultCallback::needsCollision(proxy0);
						}
					};

					StaticOnlyCallback sweepResults(body,body.GetWorldTransform().Translation,predictedTrans.Translation,getBroadphase().getOverlappingPairCache(),getDispatcher());
#else
								ClosestNotMeConvexResultCallback sweepResults = new ClosestNotMeConvexResultCallback(body, body.GetWorldTransform().Translation, predictedTrans.Translation, GetBroadphase().GetOverlappingPairCache(), GetDispatcher());
#endif
								//btConvexShape* convexShape = static_cast<btConvexShape*>(body.GetCollisionShape());
								SphereShape tmpSphere = new SphereShape(body.CcdSweptSphereRadius);//btConvexShape* convexShape = static_cast<btConvexShape*>(body.GetCollisionShape());
								sweepResults.m_allowedPenetration = GetDispatchInfo().GetAllowedCcdPenetration();

								sweepResults.m_collisionFilterGroup = body.GetBroadphaseProxy().m_collisionFilterGroup;
								sweepResults.m_collisionFilterMask = body.GetBroadphaseProxy().m_collisionFilterMask;
								Matrix modifiedPredictedTrans = MathUtil.BasisMatrix(body.GetWorldTransform());
								modifiedPredictedTrans.Translation = predictedTrans.Translation;

								ConvexSweepTest(tmpSphere, body.GetWorldTransform(), modifiedPredictedTrans, sweepResults, 0f);
								if (sweepResults.HasHit && (sweepResults.m_closestHitFraction < 1.0f))
								{

									//printf("clamped integration to hit fraction = %f\n",fraction);
									body.SetHitFraction(sweepResults.m_closestHitFraction);
									body.PredictIntegratedTransform(timeStep * body.GetHitFraction(), out predictedTrans);
									body.SetHitFraction(0.0f);
									body.ProceedToTransform(ref predictedTrans);

#if false
						btVector3 linVel = body.getLinearVelocity();

						float maxSpeed = body.getCcdMotionThreshold()/getSolverInfo().m_timeStep;
						float maxSpeedSqr = maxSpeed*maxSpeed;
						if (linVel.LengthSquared()>maxSpeedSqr)
						{
							linVel.normalize();
							linVel*= maxSpeed;
							body.setLinearVelocity(linVel);
							float ms2 = body.getLinearVelocity().LengthSquared();
							body.predictIntegratedTransform(timeStep, predictedTrans);

							float sm2 = (predictedTrans.Translation-body.GetWorldTransform().Translation).LengthSquared();
							float smt = body.getCcdSquareMotionThreshold();
							printf("sm2=%f\n",sm2);
						}
#else
									//response  between two dynamic objects without friction, assuming 0 penetration depth
									float appliedImpulse = 0.0f;
									float depth = 0.0f;
									appliedImpulse = ContactConstraint.ResolveSingleCollision(body, sweepResults.m_hitCollisionObject, ref sweepResults.m_hitPointWorld, ref sweepResults.m_hitNormalWorld, GetSolverInfo(), depth);


#endif

									continue;
								}
							}
							BulletGlobals.StopProfile();

						}


						body.ProceedToTransform(ref predictedTrans);
					}
				}
			}
		}
		
		protected virtual void AddSpeculativeContacts(float timeStep)
		{
			BulletGlobals.StartProfile("AddSpeculativeContacts");
	Matrix predictedTrans;
	for ( int i=0;i<m_nonStaticRigidBodies.Count;i++)
	{
		RigidBody body = m_nonStaticRigidBodies[i];
		body.SetHitFraction(1.0f);

		if (body.IsActive() && (!body.IsStaticOrKinematicObject))
		{
			body.PredictIntegratedTransform(timeStep, out predictedTrans);
			float squareMotion = (predictedTrans.Translation-body.GetWorldTransform().Translation).LengthSquared();

			if (body.GetCcdSquareMotionThreshold() != 0.0f && body.GetCcdSquareMotionThreshold() < squareMotion)
			{
				BulletGlobals.StartProfile("search speculative contacts");
				if (body.CollisionShape.IsConvex())
				{
					gNumClampedCcdMotions++;
					
					ClosestNotMeConvexResultCallback sweepResults = new ClosestNotMeConvexResultCallback(body,body.GetWorldTransform().Translation,predictedTrans.Translation,GetBroadphase().GetOverlappingPairCache(),GetDispatcher());
					//btConvexShape* convexShape = static_cast<btConvexShape*>(body.GetCollisionShape());
					SphereShape tmpSphere = new SphereShape(body.CcdSweptSphereRadius);//btConvexShape* convexShape = static_cast<btConvexShape*>(body.GetCollisionShape());

					sweepResults.m_collisionFilterGroup = body.GetBroadphaseProxy().m_collisionFilterGroup;
					sweepResults.m_collisionFilterMask  = body.GetBroadphaseProxy().m_collisionFilterMask;
					Matrix modifiedPredictedTrans = MathUtil.BasisMatrix(body.GetWorldTransform());
					modifiedPredictedTrans.Translation = predictedTrans.Translation;


					ConvexSweepTest(tmpSphere,body.GetWorldTransform(),modifiedPredictedTrans,sweepResults,0f);
					if (sweepResults.HasHit && (sweepResults.m_closestHitFraction < 1.0f))
					{
						BroadphaseProxy proxy0 = body.GetBroadphaseHandle();
						BroadphaseProxy proxy1 = sweepResults.m_hitCollisionObject.GetBroadphaseHandle();
						BroadphasePair pair = sweepResults.m_pairCache.FindPair(proxy0,proxy1);
						if (pair != null)
						{
							if (pair.m_algorithm != null)
							{
								ObjectArray<PersistentManifold> contacts = new ObjectArray<PersistentManifold>();
								pair.m_algorithm.GetAllContactManifolds(contacts);
								if (contacts.Count > 0)
								{
									ManifoldResult result = new ManifoldResult(body,sweepResults.m_hitCollisionObject);
									result.SetPersistentManifold(contacts[0]);

									Vector3 vec = (modifiedPredictedTrans.Translation-body.GetWorldTransform().Translation);
									vec*=sweepResults.m_closestHitFraction;
									
									float lenSqr = vec.LengthSquared();
									float depth = 0.0f;
									Vector3 pointWorld = sweepResults.m_hitPointWorld;
									if (lenSqr>MathUtil.SIMD_EPSILON)
									{
										depth = (float)Math.Sqrt(lenSqr);
										pointWorld -= vec;
										vec /= depth;
									}

									if (contacts[0].GetBody0()==body)
									{
										result.AddContactPoint(sweepResults.m_hitNormalWorld,pointWorld,depth);
#if false
										debugContacts.Add(sweepResults.m_hitPointWorld);//sweepResults.m_hitPointWorld);
										debugNormals.Add(sweepResults.m_hitNormalWorld);
#endif
									} else
									{
										//swapped
										result.AddContactPoint(-sweepResults.m_hitNormalWorld,pointWorld,depth);
										//sweepResults.m_hitPointWorld,depth);
										
#if false
										if (1)//firstHit==1)
										{
											firstHit=0;
											debugNormals.push_back(sweepResults.m_hitNormalWorld);
											debugContacts.push_back(pointWorld);//sweepResults.m_hitPointWorld);
											debugNormals.push_back(sweepResults.m_hitNormalWorld);
											debugContacts.push_back(sweepResults.m_hitPointWorld);
										}
										firstHit--;
#endif
									}
								}

							} else
							{
								//no algorithm, use dispatcher to create one

							}


						} else
						{
							//add an overlapping pair
							//printf("pair missing\n");

						}
					}
				}
			}
			
		}
	}
		}


	    protected virtual void	CalculateSimulationIslands()
        {

            BulletGlobals.StartProfile("calculateSimulationIslands");

	        GetSimulationIslandManager().UpdateActivationState(GetCollisionWorld(),GetCollisionWorld().GetDispatcher());
	        {
		        int length = m_constraints.Count;
				for(int i=0;i<length;++i)
				{
					TypedConstraint constraint = m_constraints[i];
			        RigidBody colObj0 = constraint.GetRigidBodyA();
			        RigidBody colObj1 = constraint.GetRigidBodyB();

			        if (((colObj0 != null) && (!colObj0.IsStaticOrKinematicObject)) &&
				        ((colObj1 != null) && (!colObj1.IsStaticOrKinematicObject)))
			        {
				        if (colObj0.IsActive() || colObj1.IsActive())
				        {
					        GetSimulationIslandManager().GetUnionFind().Unite((colObj0).GetIslandTag(),
						        (colObj1).GetIslandTag());
				        }
			        }
		        }
	        }

	        //Store the island id in each body
	        GetSimulationIslandManager().StoreIslandActivationState(GetCollisionWorld());
            BulletGlobals.StopProfile();
        }

	    protected virtual void SolveConstraints(ContactSolverInfo solverInfo)
        {
	        //sorted version of all btTypedConstraint, based on islandId
            ObjectArray<TypedConstraint> sortedConstraints;
            if (NumConstraints > 0)
            {
                sortedConstraints = new ObjectArray<TypedConstraint>(m_constraints);

                //sortedConstraints.quickSort(btSortConstraintOnIslandPredicate());
                //sortedConstraints.Sort(new SortConstraintOnIslandPredicate());
            }
            else
            {
                sortedConstraints = null;
            }

			if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugDiscreteDynamicsWorld)
            {
                BulletGlobals.g_streamWriter.WriteLine("solveConstraints");
            }


        //	btAssert(0);

            InplaceSolverIslandCallback solverCallback = new InplaceSolverIslandCallback(solverInfo, m_constraintSolver, sortedConstraints, NumConstraints, m_debugDrawer, m_dispatcher1);

			if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugDiscreteDynamicsWorld)
            {
                BulletGlobals.g_streamWriter.WriteLine("prepareSolve");
            }

	        m_constraintSolver.PrepareSolve(GetCollisionWorld().NumCollisionObjects, GetCollisionWorld().GetDispatcher().GetNumManifolds());

			if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugDiscreteDynamicsWorld)
            {
                BulletGlobals.g_streamWriter.WriteLine("buildAndProcessIslands");
            }

	        /// solve all the constraints for this island
	        m_islandManager.BuildAndProcessIslands(GetCollisionWorld().GetDispatcher(),GetCollisionWorld(),solverCallback);

            solverCallback.ProcessConstraints();

	        m_constraintSolver.AllSolved(solverInfo, m_debugDrawer);
        }
    	
	    protected void	UpdateActivationState(float timeStep)
        {
            BulletGlobals.StartProfile("updateActivationState");

			int length = m_nonStaticRigidBodies.Count;
			for (int i = 0; i < length; ++i)
			{
				RigidBody body = m_nonStaticRigidBodies[i];
				if (body != null)
		        {
			        body.UpdateDeactivation(timeStep);
                     
			        if (body.WantsSleeping())
			        {
				        if (body.IsStaticOrKinematicObject)
				        {
					        body.ActivationState = ActivationState.IslandSleeping;
				        } 
                        else
				        {
					        if (body.ActivationState == ActivationState.ActiveTag)
						        body.ActivationState = ActivationState.WantsDeactivation;
					        if (body.ActivationState == ActivationState.IslandSleeping) 
					        {
                                Vector3 zero = Vector3.Zero;
						        body.SetAngularVelocity(ref zero);
						        body.SetLinearVelocity(ref zero);
					        }
				        }
			        } 
                    else
			        {
				        if (body.ActivationState != ActivationState.DisableDeactivation)
					        body.ActivationState = ActivationState.ActiveTag;
			        }
		        }
	        }
            BulletGlobals.StopProfile();
        }

	    protected void	UpdateActions(float timeStep)
        {
            BulletGlobals.StartProfile("updateActions");
			int length = m_actions.Count;
			for (int i = 0; i < length;++i )
			{
				m_actions[i].UpdateAction(this, timeStep);
			}
            BulletGlobals.StopProfile();
        }

        protected void StartProfiling(float timeStep)
        {
            BulletGlobals.ResetProfile();

        }

	    protected virtual void InternalSingleStepSimulation( float timeStep)
        {
            BulletGlobals.StartProfile("internalSingleStepSimulation");

			if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugDiscreteDynamicsWorld)
            {
                BulletGlobals.g_streamWriter.WriteLine("internalSingleStepSimulation");
            }

            if(null != m_internalPreTickCallback) 
            {
		        m_internalPreTickCallback.InternalTickCallback(this, timeStep);
	        }	

	        ///apply gravity, predict motion
	        PredictUnconstraintMotion(timeStep);

	        DispatcherInfo dispatchInfo = GetDispatchInfo();

	        dispatchInfo.SetTimeStep(timeStep);
	        dispatchInfo.SetStepCount(0);
	        dispatchInfo.SetDebugDraw(GetDebugDrawer());

	        ///perform collision detection
	        PerformDiscreteCollisionDetection();

			if (GetDispatchInfo().m_useContinuous)
			{
				AddSpeculativeContacts(timeStep);
			}


	        CalculateSimulationIslands();
        	
	        GetSolverInfo().m_timeStep = timeStep;

	        ///solve contact and other joint constraints
	        SolveConstraints(GetSolverInfo());
        	
	        ///CallbackTriggers();

	        ///integrate transforms
	        IntegrateTransforms(timeStep);

	        ///update vehicle simulation
	        UpdateActions(timeStep);
        	
	        UpdateActivationState( timeStep );

	        if(m_internalTickCallback != null) 
            {
		        m_internalTickCallback.InternalTickCallback(this, timeStep);
	        }
            BulletGlobals.StopProfile();
        }


	    protected virtual void	SaveKinematicState(float timeStep)
        {
            ///would like to iterate over m_nonStaticRigidBodies, but unfortunately old API allows
            ///to switch status _after_ adding kinematic objects to the world
            ///fix it for Bullet 3.x release

			int length = m_collisionObjects.Count;
			for (int i = 0; i < length;++i )
			{
				RigidBody body = RigidBody.Upcast(m_collisionObjects[i]);
				if (body != null && body.ActivationState != ActivationState.IslandSleeping)
				{
					if (body.IsKinematicObject)
					{
						//to calculate velocities next frame
						body.SaveKinematicState(timeStep);
					}
				}
			}
        }

        public static int GetConstraintIslandId(TypedConstraint lhs)
        {
	        int islandId;
	        CollisionObject rcolObj0 = lhs.GetRigidBodyA();
	        CollisionObject rcolObj1 = lhs.GetRigidBodyB();
	        islandId= rcolObj0.GetIslandTag()>=0?rcolObj0.GetIslandTag():rcolObj1.GetIslandTag();
	        return islandId;
        }

        protected IConstraintSolver	m_constraintSolver;

	    protected SimulationIslandManager m_islandManager;

	    protected ObjectArray<TypedConstraint> m_constraints;

        protected ObjectArray<RigidBody> m_nonStaticRigidBodies;


	    protected Vector3 m_gravity;

	    //for variable timesteps
	    protected float m_localTime;
	    //for variable timesteps

	    protected bool m_ownsIslandManager;
	    protected bool m_ownsConstraintSolver;
        protected bool m_synchronizeAllMotionStates;

	    protected IList<IActionInterface> m_actions;
    	
	    protected int m_profileTimings;
        protected int gNumClampedCcdMotions;
        protected const float s_fixedTimeStep = 1f / 60f;
        //public const float s_fixedTimeStep = 0.016666668f;
        //protected float s_fixedTimeStep2 = 1 / 60;
        //protected float s_fixedTimeStep3 = (float)(1.0 / 60.0);
        //protected float s_fixedTimeStep4 = 0.016666668f;

        
    }

    public class InplaceSolverIslandCallback : IIslandCallback
	{
		public ContactSolverInfo m_solverInfo;
		public IConstraintSolver	m_solver;
		public ObjectArray<TypedConstraint> m_sortedConstraints;
		public int	m_numConstraints;
		public IDebugDraw m_debugDrawer;
		public IDispatcher m_dispatcher;
        public ObjectArray<CollisionObject> m_bodies;
		public ObjectArray<PersistentManifold> m_manifolds;
		public ObjectArray<TypedConstraint> m_constraints;

		public InplaceSolverIslandCallback(
			ContactSolverInfo solverInfo,
			IConstraintSolver	solver,
			ObjectArray<TypedConstraint> sortedConstraints,
			int	numConstraints,
			IDebugDraw	debugDrawer,
			IDispatcher dispatcher)
        {
			m_solverInfo = solverInfo;
			m_solver = solver;
			m_sortedConstraints = sortedConstraints;
			m_numConstraints = numConstraints;
			m_debugDrawer = debugDrawer;
			m_dispatcher = dispatcher;
            m_bodies = new ObjectArray<CollisionObject>();
            m_manifolds = new ObjectArray<PersistentManifold>();
            m_constraints = new ObjectArray<TypedConstraint>();
		}

        //InplaceSolverIslandCallback operator=(InplaceSolverIslandCallback& other)
        //{
        //    Debug.Assert(false);
        //    //(void)other;
        //    return *this;
        //}


        public virtual void ProcessIsland(ObjectArray<CollisionObject> bodies, int numBodies, ObjectArray<PersistentManifold> manifolds, int numManifolds, int islandId)
		{
			if (islandId<0)
			{
				if (numManifolds + m_numConstraints > 0)
				{
					///we don't split islands, so all constraints/contact manifolds/bodies are passed into the solver regardless the island id
					m_solver.SolveGroup( bodies,numBodies,manifolds,numManifolds,m_sortedConstraints,0,m_numConstraints,m_solverInfo,m_debugDrawer,m_dispatcher);
				}
			} 
            else
			{
				//also add all non-contact constraints/joints for this island
                int startConstraint = 0;
				int numCurConstraints = 0;
                int i = 0;
				
				//find the first constraint for this island
				for (i=0;i<m_numConstraints;i++)
				{
					if (DiscreteDynamicsWorld.GetConstraintIslandId(m_sortedConstraints[i]) == islandId)
					{
                        startConstraint = i;
                        break;
					}
				}
				//count the number of constraints in this island
				for (;i<m_numConstraints;i++)
				{
					if (DiscreteDynamicsWorld.GetConstraintIslandId(m_sortedConstraints[i]) == islandId)
					{
						numCurConstraints++;
					}
				}

                if (m_solverInfo.m_minimumSolverBatchSize<=1)
				{
					///only call solveGroup if there is some work: avoid virtual function call, its overhead can be excessive
					if (numManifolds + numCurConstraints != 0)
					{
                        m_solver.SolveGroup(bodies, numBodies, manifolds, numManifolds, m_sortedConstraints, startConstraint, numCurConstraints, m_solverInfo, m_debugDrawer, m_dispatcher);
					}
				} 
                else
                {					
					for (i=0;i<numBodies;i++)
                    {
						m_bodies.Add(bodies[i]);
                    }
					for (i=0;i<numManifolds;i++)
                    {
						m_manifolds.Add(manifolds[i]);
                    }
                    int lastConstraint = startConstraint + numCurConstraints;
                    for (i = startConstraint; i < lastConstraint; i++)
                    {
                        m_constraints.Add(m_sortedConstraints[i]);
                    }
					if ((m_constraints.Count+m_manifolds.Count)>m_solverInfo.m_minimumSolverBatchSize)
					{
						ProcessConstraints();
					} 
                    else
					{
						//printf("deferred\n");
					}
				}

                
//                ///only call solveGroup if there is some work: avoid virtual function call, its overhead can be excessive
//                if (numManifolds + numCurConstraints > 0)
//                {
////solveGroup(ObjectArray<CollisionObject> bodies,int numBodies,ObjectArray<PersistentManifold> manifold,int numManifolds,ObjectArray<TypedConstraint> constraints,int numConstraints, ContactSolverInfo info,IDebugDraw debugDrawer, IDispatcher dispatcher);
//                    m_solver.solveGroup(bodies,numBodies,manifolds, numManifolds,startConstraint,numCurConstraints,m_solverInfo,m_debugDrawer,m_dispatcher);
//                }
		
			}
		}

        public void	ProcessConstraints()
		{
			if (m_manifolds.Count + m_constraints.Count>0)
			{
				m_solver.SolveGroup( m_bodies,m_bodies.Count, m_manifolds, m_manifolds.Count, m_constraints, 0, m_constraints.Count ,m_solverInfo,m_debugDrawer,m_dispatcher);
			}
			m_bodies.Clear();
			m_manifolds.Clear();
			m_constraints.Clear();

		}


	}


    public class ClosestNotMeConvexResultCallback : ClosestConvexResultCallback
    {
	    public CollisionObject m_me;
	    public float m_allowedPenetration;
	    public IOverlappingPairCache m_pairCache;
	    public IDispatcher m_dispatcher;

	    public ClosestNotMeConvexResultCallback (CollisionObject me,Vector3 fromA,Vector3 toA,IOverlappingPairCache pairCache,IDispatcher dispatcher) : this(me, ref fromA, ref toA,pairCache,dispatcher) 
        {
        }
	    public ClosestNotMeConvexResultCallback (CollisionObject me,ref Vector3 fromA,ref Vector3 toA,IOverlappingPairCache pairCache,IDispatcher dispatcher) : 
	      base(ref fromA,ref toA)
        {
		    m_allowedPenetration = 0.0f;
		    m_me = me;
		    m_pairCache = pairCache;
		    m_dispatcher = dispatcher;
	    }

	    public override float AddSingleResult(LocalConvexResult convexResult,bool normalInWorldSpace)
	    {
		    if (convexResult.m_hitCollisionObject == m_me)
			    return 1.0f;

		    //ignore result if there is no contact response
		    if(!convexResult.m_hitCollisionObject.HasContactResponse)
			    return 1.0f;

		    Vector3 linVelA,linVelB;
		    linVelA = m_convexToWorld-m_convexFromWorld;
		    linVelB = Vector3.Zero;//toB.Translation-fromB.Translation;

		    Vector3 relativeVelocity = (linVelA-linVelB);
		    //don't report time of impact for motion away from the contact normal (or causes minor penetration)
		    if (Vector3.Dot(convexResult.m_hitNormalLocal,relativeVelocity)>=-m_allowedPenetration)
			    return 1f;

		    return base.AddSingleResult (convexResult, normalInWorldSpace);
	    }

	    public override bool NeedsCollision(BroadphaseProxy proxy0)
	    {
		    //don't collide with itself
		    if (proxy0.m_clientObject == m_me)
			    return false;

		    ///don't do CCD when the collision filters are not matching
		    if (!base.NeedsCollision(proxy0))
			    return false;

            CollisionObject otherObj = proxy0.m_clientObject as CollisionObject;

		    //call needsResponse, see http://code.google.com/p/bullet/issues/detail?id=179
		    if (m_dispatcher.NeedsResponse(m_me,otherObj))
		    {
#if false
			    ///don't do CCD when there are already contact points (touching contact/penetration)
                ObjectArray<PersistentManifold> manifoldArray = new ObjectArray<PersistentManifold>();
			    BroadphasePair collisionPair = m_pairCache.FindPair(m_me.GetBroadphaseHandle(),proxy0);
			    if (collisionPair != null)
			    {
				    if (collisionPair.m_algorithm != null)
				    {
					    collisionPair.m_algorithm.GetAllContactManifolds(manifoldArray);
						int length = manifoldArray.Count;
						for(int i=0;i<length;++i)
					    {
							if (manifoldArray[i].GetNumContacts() > 0)
							{
								return false;
							}
					    }
				    }
			    }
#endif
				return true;
		    }
		    return false;
	    }
    }

    public class SortConstraintOnIslandPredicate : IComparer<TypedConstraint>
    {
        #region IComparer<TypedConstraint> Members

        public int Compare(TypedConstraint x, TypedConstraint y)
        {
            int id1 = DiscreteDynamicsWorld.GetConstraintIslandId(x);
            int id2 = DiscreteDynamicsWorld.GetConstraintIslandId(y);
            return id1 - id2;
        }

        #endregion
    }


}
