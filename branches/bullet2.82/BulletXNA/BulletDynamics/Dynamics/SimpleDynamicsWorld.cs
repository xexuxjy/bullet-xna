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

using BulletXNA.BulletCollision;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletDynamics
{
	public abstract class SimpleDynamicsWorld : DynamicsWorld
	{
		///this btSimpleDynamicsWorld constructor creates dispatcher, broadphase pairCache and constraintSolver
		public SimpleDynamicsWorld(IDispatcher dispatcher, IBroadphaseInterface pairCache, IConstraintSolver constraintSolver, ICollisionConfiguration collisionConfiguration)
			: base(dispatcher, pairCache, collisionConfiguration)
		{
			m_constraintSolver = constraintSolver;
			m_ownsConstraintSolver = false;
			IndexedVector3 gravity = new IndexedVector3(0, 0, -10f);
			SetGravity(ref gravity);

		}

		public override void Cleanup()
		{
			base.Cleanup();
			if (m_ownsConstraintSolver)
			{
				m_constraintSolver.Cleanup();
				m_constraintSolver = null;
			}
		}

		///maxSubSteps/fixedTimeStep for interpolation is currently ignored for btSimpleDynamicsWorld, use btDiscreteDynamicsWorld instead
		public override int StepSimulation(float timeStep, int maxSubSteps, float fixedTimeStep)
		{
			///apply gravity, predict motion
			PredictUnconstraintMotion(timeStep);

			DispatcherInfo dispatchInfo = GetDispatchInfo();
			dispatchInfo.SetTimeStep(timeStep);
			dispatchInfo.SetStepCount(0);
			dispatchInfo.SetDebugDraw(GetDebugDrawer());

			///perform collision detection
			PerformDiscreteCollisionDetection();

			///solve contact constraints
			int numManifolds = m_dispatcher1.GetNumManifolds();
			if (numManifolds != 0)
			{
                PersistentManifoldArray manifoldPtr = (m_dispatcher1 as CollisionDispatcher).GetInternalManifoldPointer();

				ContactSolverInfo infoGlobal = new ContactSolverInfo();
				infoGlobal.m_timeStep = timeStep;
				m_constraintSolver.PrepareSolve(0, numManifolds);
				m_constraintSolver.SolveGroup(null, 0, manifoldPtr, 0, numManifolds, null, 0, 0, infoGlobal, m_debugDrawer, m_dispatcher1);
				m_constraintSolver.AllSolved(infoGlobal, m_debugDrawer);
			}

			///integrate transforms
			IntegrateTransforms(timeStep);

			UpdateAabbs();

			SynchronizeMotionStates();

			ClearForces();

			return 1;

		}

		public override void SetGravity(ref IndexedVector3 gravity)
		{
			m_gravity = gravity;
			foreach (CollisionObject colObj in m_collisionObjects)
			{
				RigidBody body = RigidBody.Upcast(colObj);
				if (body != null)
				{
					body.SetGravity(ref gravity);
				}
			}
		}

		public override IndexedVector3 GetGravity()
		{
			return m_gravity;
		}

		public override void AddRigidBody(RigidBody body)
		{
			body.SetGravity(ref m_gravity);

			if (body.GetCollisionShape() != null)
			{
				AddCollisionObject(body);
			}

		}


		public override void AddRigidBody(RigidBody body, CollisionFilterGroups group, CollisionFilterGroups mask)
		{
			body.SetGravity(ref m_gravity);

			if (body.GetCollisionShape() != null)
			{
				AddCollisionObject(body, group, mask);
			}
		}


		public override void DebugDrawWorld()
		{

		}

		public override void AddAction(IActionInterface action)
		{

		}

		public override void RemoveAction(IActionInterface action)
		{

		}


		public override void RemoveRigidBody(RigidBody body)
		{
			base.RemoveCollisionObject(body);
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

		public override void UpdateAabbs()
		{
			//IndexedMatrix predictedTrans = IndexedMatrix.Identity;
			foreach (CollisionObject colObj in m_collisionObjects)
			{
				RigidBody body = RigidBody.Upcast(colObj);
				if (body != null)
				{
					if (body.IsActive() && (!body.IsStaticObject()))
					{
						IndexedVector3 minAabb;
						IndexedVector3 maxAabb;
						colObj.GetCollisionShape().GetAabb(colObj.GetWorldTransform(), out minAabb, out maxAabb);
						IBroadphaseInterface bp = GetBroadphase();
						bp.SetAabb(body.GetBroadphaseHandle(), ref minAabb, ref maxAabb, m_dispatcher1);
					}
				}
			}
		}


		public override void SynchronizeMotionStates()
		{
			///@todo: iterate over awake simulation islands!
			foreach (CollisionObject colObj in m_collisionObjects)
			{
				RigidBody body = RigidBody.Upcast(colObj);
				if (body != null && body.GetMotionState() != null)
				{
					if (body.GetActivationState() != ActivationState.ISLAND_SLEEPING)
					{
						body.GetMotionState().SetWorldTransform(body.GetWorldTransform());
					}
				}
			}

		}

		public override void SetConstraintSolver(IConstraintSolver solver)
		{
			m_ownsConstraintSolver = false;
			m_constraintSolver = solver;

		}

		public override IConstraintSolver GetConstraintSolver()
		{
			return m_constraintSolver;
		}

		public override DynamicsWorldType GetWorldType()
		{
			return DynamicsWorldType.BT_SIMPLE_DYNAMICS_WORLD;
		}

		public override void ClearForces()
		{
			///@todo: iterate over awake simulation islands!
			for (int i = 0; i < m_collisionObjects.Count; i++)
			{
				CollisionObject colObj = m_collisionObjects[i];

				RigidBody body = RigidBody.Upcast(colObj);
				if (body != null)
				{
					body.ClearForces();
				}
			}

		}


		protected void PredictUnconstraintMotion(float timeStep)
		{
			foreach (CollisionObject colObj in m_collisionObjects)
			{
				RigidBody body = RigidBody.Upcast(colObj);
				if (body != null)
				{
					if (!body.IsStaticObject())
					{
						if (body.IsActive())
						{
							body.ApplyGravity();
							body.IntegrateVelocities(timeStep);
							body.ApplyDamping(timeStep);
							IndexedMatrix temp = body.GetInterpolationWorldTransform();
							body.PredictIntegratedTransform(timeStep, out temp);
							body.SetInterpolationWorldTransform(ref temp);
						}
					}
				}
			}
		}

		protected void IntegrateTransforms(float timeStep)
		{
			IndexedMatrix predictedTrans;
			foreach (CollisionObject colObj in m_collisionObjects)
			{
				RigidBody body = RigidBody.Upcast(colObj);
				if (body != null)
				{
					if (body.IsActive() && (!body.IsStaticObject()))
					{
						body.PredictIntegratedTransform(timeStep, out predictedTrans);
						body.ProceedToTransform(ref predictedTrans);
					}
				}
			}
		}

		protected IConstraintSolver m_constraintSolver;
		protected bool m_ownsConstraintSolver;
		protected IndexedVector3 m_gravity;

	}
}
