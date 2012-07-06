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
using BulletXNA.BulletCollision;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletDynamics
{

    public enum DynamicsWorldType
    {
        BT_SIMPLE_DYNAMICS_WORLD = 1,
        BT_DISCRETE_DYNAMICS_WORLD = 2,
        BT_CONTINUOUS_DYNAMICS_WORLD = 3,
		BT_SOFT_RIGID_DYNAMICS_WORLD = 4

    }

    /// Type for the callback for each tick
    public interface IInternalTickCallback
    {
        void InternalTickCallback(DynamicsWorld world, float timeStep);
    }


    public abstract class DynamicsWorld : CollisionWorld
    {

		public DynamicsWorld(IDispatcher dispatcher,IBroadphaseInterface broadphase,ICollisionConfiguration collisionConfiguration)
		:base(dispatcher,broadphase,collisionConfiguration)
		{
            m_internalTickCallback = null;
            m_worldUserInfo = null;
            m_solverInfo = new ContactSolverInfo();
		}

	
		///stepSimulation proceeds the simulation over 'timeStep', units in preferably in seconds.
		///By default, Bullet will subdivide the timestep in constant substeps of each 'fixedTimeStep'.
		///in order to keep the simulation real-time, the maximum number of substeps can be clamped to 'maxSubSteps'.
		///You can disable subdividing the timestep/substepping by passing maxSubSteps=0 as second argument to stepSimulation, but in that case you have to keep the timeStep constant.
        ///public override int	stepSimulation(float timeStep,int maxSubSteps=1, float fixedTimeStep=float(1.)/float(60.))=0;
        public abstract int StepSimulation(float timeStep, int maxSubSteps);
        public abstract int StepSimulation(float timeStep, int maxSubSteps, float fixedTimeStep);

        public override void DebugDrawWorld()
        {
            base.DebugDrawWorld();
        }

        public virtual void AddConstraint(TypedConstraint constraint)
        {
            AddConstraint(constraint, false);
        }
				
		public virtual void	AddConstraint(TypedConstraint constraint, bool disableCollisionsBetweenLinkedBodies) 
		{ 
            //(void)constraint; 
            //(void)disableCollisionsBetweenLinkedBodies;
		}

		public virtual void	RemoveConstraint(TypedConstraint constraint)
        {
            //(void)constraint;
        }

        public abstract void AddAction(IActionInterface action);

        public abstract void RemoveAction(IActionInterface action);

		//once a rigidbody is added to the dynamics world, it will get this gravity assigned
		//existing rigidbodies in the world get gravity assigned too, during this method
		public abstract void SetGravity(ref IndexedVector3 gravity);
        public abstract void SetGravity(IndexedVector3 gravity);

        public abstract IndexedVector3 GetGravity();

        public abstract void SynchronizeMotionStates();

        //public abstract  void addRigidBody(RigidBody body,short mask1,short mask2);
        public abstract  void AddRigidBody(RigidBody body);

        public abstract void AddRigidBody(RigidBody body, CollisionFilterGroups group, CollisionFilterGroups mask);

		public abstract  void	RemoveRigidBody(RigidBody body);

		public abstract  void	SetConstraintSolver(IConstraintSolver solver);

        public abstract  IConstraintSolver GetConstraintSolver();

        public virtual int GetNumConstraints()
        {
            return 0;
        }

        public virtual TypedConstraint GetConstraint(int index)
        {
            return null;
        }

		public abstract DynamicsWorldType GetWorldType();

		public abstract void ClearForces();

		/// Set the callback for when an internal tick (simulation substep) happens, optional user info
		public void SetInternalTickCallback(IInternalTickCallback cb, Object worldUserInfo,bool isPreTick) 
		{ 
            if (isPreTick)
            {
                m_internalPreTickCallback = cb;
            }
            else
            {
                m_internalTickCallback = cb;
            }
            m_worldUserInfo = worldUserInfo;
        }

		public void	SetWorldUserInfo(Object worldUserInfo)
		{
			m_worldUserInfo = worldUserInfo;
		}

		public Object GetWorldUserInfo()
		{
			return m_worldUserInfo;
		}

		public ContactSolverInfo GetSolverInfo()
		{
			return m_solverInfo;
		}

		///obsolete, use addAction instead.
		public virtual void	AddVehicle(IActionInterface vehicle)
        {
            
        }
		///obsolete, use removeAction instead
		public virtual void	RemoveVehicle(IActionInterface vehicle) 
        {
            
        }
		///obsolete, use addAction instead.
		public virtual void	AddCharacter(IActionInterface character) 
        {

        }
		///obsolete, use removeAction instead
		public virtual void RemoveCharacter(IActionInterface character) 
        {

        }
    
		protected IInternalTickCallback m_internalTickCallback;
        protected IInternalTickCallback m_internalPreTickCallback;
        protected Object m_worldUserInfo;
		protected ContactSolverInfo	m_solverInfo;

    }
}
