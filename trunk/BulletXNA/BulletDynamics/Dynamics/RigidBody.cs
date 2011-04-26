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
using BulletXNA.BulletCollision.BroadphaseCollision;
using BulletXNA.BulletCollision.CollisionDispatch;
using BulletXNA.BulletCollision.CollisionShapes;
using BulletXNA.BulletDynamics.ConstraintSolver;
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletDynamics.Dynamics
{
    public class RigidBody : CollisionObject
    {
        public static bool debugRigidBody = false;
        private const float MAX_ANGVEL = MathUtil.SIMD_HALF_PI;
        private static int uniqueId = 0;
        private static int s_counter = 0;

        private Matrix	m_invInertiaTensorWorld= Matrix.Identity;
	    private Vector3		m_linearVelocity;
	    private Vector3		m_angularVelocity;
	    private float		m_inverseMass;
        private Vector3     m_linearFactor;


	    private Vector3		m_gravity;	
	    private Vector3		m_gravity_acceleration;
	    private Vector3		m_invInertiaLocal;
	    private Vector3		m_totalForce;
	    private Vector3		m_totalTorque;
    	
	    private float		m_linearDamping;
	    private float		m_angularDamping;

	    private bool		m_additionalDamping;
	    private float		m_additionalDampingFactor;
	    private float		m_additionalLinearDampingThresholdSqr;
	    private float		m_additionalAngularDampingThresholdSqr;
	    private float		m_additionalAngularDampingFactor;

	    private float		m_linearSleepingThreshold;
	    private float		m_angularSleepingThreshold;

	    //m_optionalMotionState allows to automatic synchronize the world transform for active objects
	    private IMotionState	m_optionalMotionState;

	    //keep track of typed constraints referencing this rigid body
	    private IList<TypedConstraint> m_constraintRefs;

        private RigidBodyFlags m_rigidbodyFlags;

        public int m_debugBodyId;

        protected Vector3 m_deltaLinearVelocity;
        protected Vector3 m_deltaAngularVelocity;
        protected Vector3 m_angularFactor;
        protected Vector3 m_invMass;
        protected Vector3 m_pushVelocity;
        protected Vector3 m_turnVelocity;

		//static RigidBody()
		//{
		//    String filename = @"C:\users\man\xna-rb-output.txt";
		//    s_filestream = File.Open(filename, FileMode.Create, FileAccess.Write, FileShare.None);
		//    s_streamWriter = new StreamWriter(s_filestream);
		//}

        public RigidBody()
        { }

	    ///btRigidBody constructor using construction info
	    public RigidBody(RigidBodyConstructionInfo constructionInfo)
        {
            SetupRigidBody(constructionInfo);
        }

	    ///btRigidBody constructor for backwards compatibility. 
	    ///To specify friction (etc) during rigid body construction, please use the other constructor (using btRigidBodyConstructionInfo)
	    public RigidBody(float mass, IMotionState motionState, CollisionShape collisionShape, Vector3 localInertia)
        {
            RigidBodyConstructionInfo cinfo = new RigidBodyConstructionInfo(mass,motionState,collisionShape,localInertia);
	        SetupRigidBody(cinfo);
        }

        public override void Cleanup()
        {
            base.Cleanup();
            //No constraints should point to this rigidbody
            //Remove constraints from the dynamics world before you delete the related rigidbodies. 
            Debug.Assert(m_constraintRefs.Count == 0);

        }

	    ///setupRigidBody is only used internally by the constructor
	    protected void	SetupRigidBody(RigidBodyConstructionInfo constructionInfo)
        {
	        m_internalType=CollisionObjectTypes.CO_RIGID_BODY;

	        m_linearVelocity = Vector3.Zero;
	        m_angularVelocity = Vector3.Zero;
            m_angularFactor = Vector3.One;
            m_linearFactor = Vector3.One;
	        m_gravity = Vector3.Zero;
	        m_gravity_acceleration = Vector3.Zero;
	        m_totalForce = Vector3.Zero;
	        m_totalTorque = Vector3.Zero;
            m_linearDamping = 0f;
            m_angularDamping = 0.5f;
	        m_linearSleepingThreshold = constructionInfo.m_linearSleepingThreshold;
	        m_angularSleepingThreshold = constructionInfo.m_angularSleepingThreshold;
	        m_optionalMotionState = constructionInfo.m_motionState;
	        m_contactSolverType = 0;
	        m_frictionSolverType = 0;
	        m_additionalDamping = constructionInfo.m_additionalDamping;
	        m_additionalDampingFactor = constructionInfo.m_additionalDampingFactor;
	        m_additionalLinearDampingThresholdSqr = constructionInfo.m_additionalLinearDampingThresholdSqr;
	        m_additionalAngularDampingThresholdSqr = constructionInfo.m_additionalAngularDampingThresholdSqr;
	        m_additionalAngularDampingFactor = constructionInfo.m_additionalAngularDampingFactor;

	        if (m_optionalMotionState != null)
	        {
		        m_optionalMotionState.GetWorldTransform(out m_worldTransform);
	        } 
            else
	        {
		        SetWorldTransform(ref constructionInfo.m_startWorldTransform);
	        }

	        m_interpolationWorldTransform = m_worldTransform;
            m_interpolationLinearVelocity = Vector3.Zero;
            m_interpolationAngularVelocity = Vector3.Zero;
        	
	        //moved to btCollisionObject
	        m_friction = constructionInfo.m_friction;
	        m_restitution = constructionInfo.m_restitution;

	        SetCollisionShape( constructionInfo.m_collisionShape );
	        m_debugBodyId = uniqueId++;
        	
	        SetMassProps(constructionInfo.m_mass, constructionInfo.m_localInertia);
            SetDamping(constructionInfo.m_linearDamping, constructionInfo.m_angularDamping);
	        UpdateInertiaTensor();
            m_rigidbodyFlags = RigidBodyFlags.BT_NONE;
            m_constraintRefs = new List<TypedConstraint>();

            m_deltaLinearVelocity = Vector3.Zero;
            m_deltaAngularVelocity = Vector3.Zero;
            m_invMass = m_inverseMass * m_linearFactor;
            m_pushVelocity = Vector3.Zero;
            m_turnVelocity = Vector3.Zero;

        }

        public void ProceedToTransform(ref Matrix newTrans)
        {
            SetCenterOfMassTransform(ref newTrans);
        }
	
	    ///to keep collision detection and dynamics separate we don't store a rigidbody pointer
	    ///but a rigidbody is derived from btCollisionObject, so we can safely perform an upcast
	    public static RigidBody	Upcast(CollisionObject colObj)
	    {
		    if (colObj.GetInternalType()==CollisionObjectTypes.CO_RIGID_BODY)
            {
			    return colObj as RigidBody;
            }
		    return null;
	    }

	    /// continuous collision detection needs prediction
	    public void	PredictIntegratedTransform(float timeStep, ref Matrix predictedTransform) 
        {
			if (BulletGlobals.g_streamWriter != null && debugRigidBody)
			{
				BulletGlobals.g_streamWriter.WriteLine("predictIntegratedTransform pre");
				MathUtil.PrintMatrix(BulletGlobals.g_streamWriter,m_worldTransform);
				MathUtil.PrintVector3(BulletGlobals.g_streamWriter,"LinearVel", m_linearVelocity);
				MathUtil.PrintVector3(BulletGlobals.g_streamWriter,"AngularVel",m_angularVelocity);

			}
            TransformUtil.IntegrateTransform(m_worldTransform,m_linearVelocity,m_angularVelocity,timeStep,ref predictedTransform);
            MathUtil.SanityCheckVector(m_worldTransform.Up);
			if (BulletGlobals.g_streamWriter != null && debugRigidBody)
			{
				BulletGlobals.g_streamWriter.WriteLine("predictIntegratedTransform post");
				MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, predictedTransform);
			}
        }

        public void SaveKinematicState(float timeStep)
        {
	        //todo: clamp to some (user definable) safe minimum timestep, to limit maximum angular/linear velocities
	        if (timeStep != 0f)
	        {
		        //if we use motionstate to synchronize world transforms, get the new kinematic/animated world transform
		        if (GetMotionState() != null)
                {
                    GetMotionState().GetWorldTransform(out m_worldTransform);
                }
                Vector3 linVel = Vector3.Zero, angVel = Vector3.Zero;

                // debug steps to track NaN's
                Matrix worldTransform = m_worldTransform;
		        TransformUtil.CalculateVelocity(ref m_interpolationWorldTransform,ref worldTransform,timeStep,ref m_linearVelocity,ref m_angularVelocity);
                SetWorldTransform(ref worldTransform);

		        m_interpolationLinearVelocity = m_linearVelocity;
		        m_interpolationAngularVelocity = m_angularVelocity;
		        SetInterpolationWorldTransform(ref m_worldTransform);
		        //printf("angular = %f %f %f\n",m_angularVelocity.getX(),m_angularVelocity.getY(),m_angularVelocity.getZ());
	        }

        }

        public void ApplyGravity()
        {
            if (IsStaticOrKinematicObject())
            {
                return;
            }

            ApplyCentralForce(ref m_gravity);
        }

        public void SetGravity(ref Vector3 acceleration)
        {
            if (m_inverseMass != 0f)
            {
                m_gravity = acceleration * (1f / m_inverseMass);
            }
            m_gravity_acceleration = acceleration;
        }

	    public Vector3	GetGravity()
	    {
		    return m_gravity_acceleration;
    	}

        public void SetDamping(float lin_damping, float ang_damping)
        {
            m_linearDamping = MathHelper.Clamp(lin_damping, 0f, 1f);
            m_angularDamping = MathHelper.Clamp(ang_damping, 0f, 1f);

        }

	    public float GetLinearDamping()
	    {
		    return m_linearDamping;
	    }

	    public float GetAngularDamping()
	    {
		    return m_angularDamping;
	    }

	    public float GetLinearSleepingThreshold()
	    {
		    return m_linearSleepingThreshold;
	    }

	    public float GetAngularSleepingThreshold() 
	    {
		    return m_angularSleepingThreshold;
	    }

	    public void	ApplyDamping(float timeStep)
        {
	        //On new damping: see discussion/issue report here: http://code.google.com/p/bullet/issues/detail?id=74
	        //todo: do some performance comparisons (but other parts of the engine are probably bottleneck anyway

        //#define USE_OLD_DAMPING_METHOD 1
        #if USE_OLD_DAMPING_METHOD
	        m_linearVelocity *= GEN_clamped((btScalar(1.) - timeStep * m_linearDamping), (btScalar)btScalar(0.0), (btScalar)btScalar(1.0));
	        m_angularVelocity *= GEN_clamped((btScalar(1.) - timeStep * m_angularDamping), (btScalar)btScalar(0.0), (btScalar)btScalar(1.0));
        #else
	        m_linearVelocity *= (float)Math.Pow((1f-m_linearDamping), timeStep);
            m_angularVelocity *= (float)Math.Pow((1f - m_angularDamping), timeStep);
            MathUtil.SanityCheckVector(ref m_linearVelocity);
            MathUtil.SanityCheckVector(ref m_angularVelocity);
#endif

	        if (m_additionalDamping)
	        {
		        //Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
		        //Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics system has improved, this should become obsolete
		        if ((m_angularVelocity.LengthSquared() < m_additionalAngularDampingThresholdSqr) &&
			        (m_linearVelocity.LengthSquared() < m_additionalLinearDampingThresholdSqr))
		        {
			        m_angularVelocity *= m_additionalDampingFactor;
			        m_linearVelocity *= m_additionalDampingFactor;
		        }


                MathUtil.SanityCheckVector(ref m_linearVelocity);
                MathUtil.SanityCheckVector(ref m_angularVelocity);
                
                float speed = m_linearVelocity.Length();
		        if (speed < m_linearDamping)
		        {
			        float dampVel = 0.005f;
			        if (speed > dampVel)
			        {
				        Vector3 dir = m_linearVelocity;
                        dir.Normalize();
				        m_linearVelocity -=  dir * dampVel;
			        } 
                    else
			        {
				        m_linearVelocity = Vector3.Zero;
			        }
		        }

		        float angSpeed = m_angularVelocity.Length();
		        if (angSpeed < m_angularDamping)
		        {
			        float angDampVel = 0.005f;
			        if (angSpeed > angDampVel)
			        {
				        Vector3 dir = m_angularVelocity;
                        dir.Normalize();
				        m_angularVelocity -=  dir * angDampVel;
			        } else
			        {
                        m_angularVelocity = Vector3.Zero;
			        }
		        }
	        }
            MathUtil.SanityCheckVector(ref m_linearVelocity);
            MathUtil.SanityCheckVector(ref m_angularVelocity);

        }

        public void SetMassProps(float mass, Vector3 inertia)
        {
            SetMassProps(mass, ref inertia);
        }

	    public void	SetMassProps(float mass, ref Vector3 inertia)
        {
	        if (MathUtil.FuzzyZero(mass))
	        {
		        m_collisionFlags |= CollisionFlags.CF_STATIC_OBJECT;
		        m_inverseMass = 0f;
	        } 
            else
	        {
		        m_collisionFlags &= (~CollisionFlags.CF_STATIC_OBJECT);
		        m_inverseMass = 1.0f / mass;
	        }

            m_invInertiaLocal = new Vector3(
                            (inertia.X != 0f) ? 1f / inertia.X : 0f,
                           (inertia.Y !=  0f) ? 1f / inertia.Y : 0f,
                           (inertia.Z !=  0f) ? 1f / inertia.Z : 0f);
            m_invMass = m_linearFactor * m_inverseMass;
        }
	
        public Vector3 GetLinearFactor()
	    {
		    return m_linearFactor;
	    }

        public void SetLinearFactor(Vector3 linearFactor)
	    {
		    m_linearFactor = linearFactor;
		    m_invMass = m_linearFactor*m_inverseMass;
	    }

	    public float GetInvMass() 
        { 
            return m_inverseMass; 
        }
	    
        public Matrix GetInvInertiaTensorWorld()
        { 
		    return m_invInertiaTensorWorld; 
	    }

        public void IntegrateVelocities(float step)
        {
	        if (IsStaticOrKinematicObject())
		        return;

			if (BulletGlobals.g_streamWriter != null && debugRigidBody)
			{
				MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "integrate LinVel pre", m_linearVelocity);
				MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "integrate AngVel pre", m_angularVelocity);
			}



	        m_linearVelocity += m_totalForce * (m_inverseMass * step);
            MathUtil.SanityCheckVector(ref m_linearVelocity);
            m_angularVelocity += Vector3.TransformNormal(m_totalTorque,m_invInertiaTensorWorld) * step;
            MathUtil.SanityCheckVector(ref m_angularVelocity);
        
	        /// clamp angular velocity. collision calculations will fail on higher angular velocities	
	        float angvel = m_angularVelocity.Length();
	        if (angvel*step > MAX_ANGVEL)
	        {
		        m_angularVelocity *= (MAX_ANGVEL/step) /angvel;
	        }
            MathUtil.SanityCheckVector(ref m_angularVelocity);

			if(BulletGlobals.g_streamWriter != null && debugRigidBody)
			{
				MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "integrate LinVel post", m_linearVelocity);
				MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "integrate AngVel post", m_angularVelocity);
			}
        }

        public void SetCenterOfMassTransform(ref Matrix xform)
        {
			if (BulletGlobals.g_streamWriter != null && debugRigidBody)
			{
				BulletGlobals.g_streamWriter.WriteLine("RigidBody setCenterOfMassTransform");
				MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, xform);
			}


            if (IsStaticOrKinematicObject())
            {
                SetInterpolationWorldTransform(ref m_worldTransform);
            }
            else
            {
                SetInterpolationWorldTransform(ref xform);
            }
            m_interpolationLinearVelocity = GetLinearVelocity();
            m_interpolationAngularVelocity = GetAngularVelocity();
            SetWorldTransform(ref xform);
            UpdateInertiaTensor();

			if (BulletGlobals.g_streamWriter != null && debugRigidBody)
			{
				BulletGlobals.g_streamWriter.WriteLine("RigidBody setCenterOfMassTransform after calcs");
				MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, m_worldTransform);
			}

        }

	    public void ApplyCentralForce(ref Vector3 force)
	    {
            m_totalForce += force * m_linearFactor;
        }

	    public Vector3 GetTotalForce()
	    {
		    return m_totalForce;
	    }

	    public Vector3 GetTotalTorque()
	    {
		    return m_totalTorque;
	    }
    
	    public Vector3 GetInvInertiaDiagLocal()
	    {
		    return m_invInertiaLocal;
	    }

	    public void SetInvInertiaDiagLocal(ref Vector3 diagInvInertia)
	    {
		    m_invInertiaLocal = diagInvInertia;
	    }

	    public void	SetSleepingThresholds(float linear,float angular)
	    {
		    m_linearSleepingThreshold = linear;
		    m_angularSleepingThreshold = angular;
	    }

        public void ApplyTorque(Vector3 torque)
        {
            ApplyTorque(ref torque);
        }

	    public void	ApplyTorque(ref Vector3 torque)
	    {
            m_totalTorque += torque * m_angularFactor;
        }
	
	    public void	ApplyForce(ref Vector3 force, ref Vector3 rel_pos) 
	    {
		    ApplyCentralForce(ref force);
            Vector3 tempTorque = Vector3.Cross(rel_pos,force);
            tempTorque *= m_angularFactor;
            ApplyTorque(Vector3.Cross(rel_pos,(force * m_linearFactor)));
        }
	
	    public void ApplyCentralImpulse(ref Vector3 impulse)
	    {
            m_linearVelocity += impulse * m_linearFactor * m_inverseMass;
            MathUtil.SanityCheckVector(ref m_linearVelocity);
	    }

        public void ApplyTorqueImpulse(Vector3 torque)
        {
            ApplyTorqueImpulse(ref torque);
        }

  	    public void ApplyTorqueImpulse(ref Vector3 torque)
	    {
            m_angularVelocity += Vector3.TransformNormal(torque,m_invInertiaTensorWorld) * m_angularFactor;
        }

        public void ApplyImpulse(Vector3 impulse, Vector3 rel_pos)
        {
            ApplyImpulse(ref impulse, ref rel_pos);
        }
	
	    public void ApplyImpulse(ref Vector3 impulse, ref Vector3 rel_pos) 
	    {
		    if (m_inverseMass != 0f)
		    {
			    ApplyCentralImpulse(ref impulse);
			    if (m_angularFactor.X != 0f)
			    {
				    ApplyTorqueImpulse(Vector3.Cross(rel_pos,(impulse*m_linearFactor)));
			    }
		    }
	    }

	    //Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position
        public void InternalApplyImpulse(Vector3 linearComponent, Vector3 angularComponent, float impulseMagnitude)
        {
            InternalApplyImpulse(ref linearComponent, ref angularComponent, impulseMagnitude);
        }
	
	    public void ClearForces() 
	    {
		    m_totalForce = Vector3.Zero;
		    m_totalTorque = Vector3.Zero;
	    }
	
	    public void UpdateInertiaTensor()
        {
            if (BulletGlobals.g_streamWriter != null && debugRigidBody)
            {
                BulletGlobals.g_streamWriter.WriteLine("RigidBody updateInertiaTensor");
		        MathUtil.PrintVector3(BulletGlobals.g_streamWriter,"invInertiaLocal",m_invInertiaLocal);
                MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, m_worldTransform);
                MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, MathUtil.ScaleBasis(ref m_worldTransform, ref m_invInertiaLocal));
                MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, MathUtil.TransposeBasis(ref m_worldTransform));

            }
            m_invInertiaTensorWorld = MathUtil.BulletMatrixMultiply(MathUtil.ScaleBasis(ref m_worldTransform, ref m_invInertiaLocal), MathUtil.TransposeBasis(ref m_worldTransform));

            if (BulletGlobals.g_streamWriter != null && debugRigidBody)
            {
                MathUtil.PrintMatrix(BulletGlobals.g_streamWriter,m_invInertiaTensorWorld);
            }

            ++s_counter;

        }
	
	    public Vector3 GetCenterOfMassPosition() 
        { 
		    return m_worldTransform.Translation; 
	    }
	    
        public Quaternion GetOrientation()
        {
            return Quaternion.CreateFromRotationMatrix(m_worldTransform);
        }
	
	    public Matrix GetCenterOfMassTransform() 
        { 
		    return m_worldTransform; 
	    }
	
        public Vector3 GetLinearVelocity()
        {
		    return m_linearVelocity; 
	    }

	    public Vector3 GetAngularVelocity() 
        { 
		    return m_angularVelocity; 
	    }

        public void SetLinearVelocity(Vector3 lin_vel)
        {
            SetLinearVelocity(ref lin_vel);
        }

	    public void SetLinearVelocity(ref Vector3 lin_vel)
	    { 
		    m_linearVelocity = lin_vel;
            MathUtil.SanityCheckVector(ref m_linearVelocity);
        }

        public void SetAngularVelocity(Vector3 ang_vel)
        {
            SetAngularVelocity(ref ang_vel);
        }

	    public void SetAngularVelocity(ref Vector3 ang_vel) 
	    { 
		    m_angularVelocity = ang_vel; 
	    }

	    public Vector3 GetVelocityInLocalPoint(ref Vector3 rel_pos)
	    {
		    //we also calculate lin/ang velocity for kinematic objects
		    return m_linearVelocity + Vector3.Cross(m_angularVelocity,rel_pos);

		    //for kinematic objects, we could also use use:
		    //		return 	(m_worldTransform(rel_pos) - m_interpolationWorldTransform(rel_pos)) / m_kinematicTimeStep;
	    }

	    public void Translate(ref Vector3 v) 
	    {
		    m_worldTransform.Translation += v; 
	    }

	
	    public void	GetAabb(out Vector3 aabbMin,out Vector3 aabbMax)
        {
            GetCollisionShape().GetAabb(m_worldTransform, out aabbMin, out aabbMax);
        }
	
	    public float ComputeImpulseDenominator(ref Vector3 pos, ref Vector3 normal)
	    {
		    Vector3 r0 = pos - GetCenterOfMassPosition();

		    Vector3 c0 = Vector3.Cross(r0,normal);

		    Vector3 vec = Vector3.Cross(MathUtil.TransposeTransformNormal(c0,GetInvInertiaTensorWorld()),r0);

		    return m_inverseMass + Vector3.Dot(normal,vec);

	    }

	    public float ComputeAngularImpulseDenominator(ref Vector3 axis)
	    {
			Vector3 vec = MathUtil.TransposeTransformNormal(axis, GetInvInertiaTensorWorld());
		    return Vector3.Dot(axis,vec);
	    }

	    public void	UpdateDeactivation(float timeStep)
	    {
            if ((GetActivationState() == ActivationState.ISLAND_SLEEPING) || (GetActivationState() == ActivationState.DISABLE_DEACTIVATION))
            {
			    return;
            }
		    
            if ((GetLinearVelocity().LengthSquared() < m_linearSleepingThreshold*m_linearSleepingThreshold) &&
			    (GetAngularVelocity().LengthSquared() < m_angularSleepingThreshold*m_angularSleepingThreshold))
		    {
			    m_deactivationTime += timeStep;
		    } 
            else
		    {
			    m_deactivationTime=0f;
			    SetActivationState(ActivationState.UNDEFINED);
		    }

	    }

	    public bool	WantsSleeping()
	    {

		    if (GetActivationState() == ActivationState.DISABLE_DEACTIVATION)
            {
			    return false;
            }

		    //disable deactivation
            if (BulletGlobals.gDisableDeactivation || BulletGlobals.gDeactivationTime == 0f)
            {
			    return false;
            }

            if ((GetActivationState() == ActivationState.ISLAND_SLEEPING) || (GetActivationState() == ActivationState.WANTS_DEACTIVATION))
            {
			    return true;
            }

            if (m_deactivationTime > BulletGlobals.gDeactivationTime)
		    {
			    return true;
		    }
		    return false;
	    }
	
	    public BroadphaseProxy	GetBroadphaseProxy() 
	    {
		    return m_broadphaseHandle;
	    }

	    public void	SetNewBroadphaseProxy(BroadphaseProxy broadphaseProxy)
	    {
		    m_broadphaseHandle = broadphaseProxy;
	    }

	    //btMotionState allows to automatic synchronize the world transform for active objects
	    public IMotionState	GetMotionState()
	    {
		    return m_optionalMotionState;
	    }

	    public void	SetMotionState(IMotionState motionState)
	    {
		    m_optionalMotionState = motionState;
		    if (m_optionalMotionState != null)
            {
                motionState.GetWorldTransform(out m_worldTransform);
            }
	    }

	    //for experimental overriding of friction/contact solver func
	    int	m_contactSolverType;
	    int	m_frictionSolverType;

	    public void	SetAngularFactor(float angFac)
	    {
		    m_angularFactor = new Vector3(angFac,angFac,angFac);
	    }

        public void SetAngularFactor(Vector3 angFac)
		{
			SetAngularFactor(ref angFac);
		}

        public void SetAngularFactor(ref Vector3 angFac)
	    {
		    m_angularFactor = angFac;
	    }

	    public Vector3 GetAngularFactor()
	    {
		    return m_angularFactor;
	    }

        public void	SetFlags(RigidBodyFlags flags)
	    {
		    m_rigidbodyFlags = flags;
	    }

	    public RigidBodyFlags GetFlags()
	    {
		    return m_rigidbodyFlags;
	    }

	    //is this rigidbody added to a btCollisionWorld/btDynamicsWorld/btBroadphase?
	    public bool IsInWorld()
	    {
		    return (GetBroadphaseProxy() != null);
	    }

        public override bool CheckCollideWithOverride(CollisionObject co)
        {
	        RigidBody otherRb = RigidBody.Upcast(co);
	        if (otherRb == null)
		        return true;

	        for (int i = 0; i < m_constraintRefs.Count; ++i)
	        {
		        TypedConstraint c = m_constraintRefs[i];
                if (c.GetRigidBodyA() == otherRb || c.GetRigidBodyB() == otherRb)
                {
                    return false;
                }
	        }

	        return true;

        }

        public void AddConstraintRef(TypedConstraint c)
        {
            if (!m_constraintRefs.Contains(c))
            {
                m_constraintRefs.Add(c);
            }

            m_checkCollideWith = true;
        }
        public void RemoveConstraintRef(TypedConstraint c)
        {
            m_constraintRefs.Remove(c);
            m_checkCollideWith = m_constraintRefs.Count > 0;

        }

	    public TypedConstraint GetConstraintRef(int index)
	    {
		    return m_constraintRefs[index];
	    }

	    public int GetNumConstraintRefs()
	    {
		    return m_constraintRefs.Count;
	    }

        	////////////////////////////////////////////////
	    ///some internal methods, don't use them
    		
	    public Vector3 InternalGetDeltaLinearVelocity()
	    {
		    return m_deltaLinearVelocity;
	    }

        public void InternalSetDeltaLinearVelocity(ref Vector3 v)
        {
            m_deltaLinearVelocity = v;
            MathUtil.SanityCheckVector(ref m_deltaLinearVelocity);
        }

	    public Vector3 InternalGetDeltaAngularVelocity()
	    {
		    return m_deltaAngularVelocity;
        }

        public void InternalSetDeltaAngularVelocity(ref Vector3 v)
        {
            m_deltaAngularVelocity = v;
            MathUtil.SanityCheckVector(ref m_deltaAngularVelocity);
        }

	    public Vector3 InternalGetAngularFactor()
	    {
		    return m_angularFactor;
	    }

	    public Vector3 InternalGetInvMass()
	    {
		    return m_invMass;
	    }
    	
	    public Vector3 InternalGetPushVelocity()
	    {
		    return m_pushVelocity;
	    }

	    public Vector3 InternalGetTurnVelocity()
	    {
		    return m_turnVelocity;
	    }

	    public void	InternalGetVelocityInLocalPointObsolete(ref Vector3 rel_pos, ref Vector3 velocity )
	    {
		    velocity = GetLinearVelocity()+m_deltaLinearVelocity + Vector3.Cross((GetAngularVelocity()+m_deltaAngularVelocity),rel_pos);
	    }

	    public void	InternalGetAngularVelocity(ref Vector3 angVel)
	    {
		    angVel = GetAngularVelocity()+m_deltaAngularVelocity;
	    }

	    //Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position
	    public void InternalApplyImpulse(ref Vector3 linearComponent, ref Vector3 angularComponent,float impulseMagnitude)
	    {
			if (BulletGlobals.g_streamWriter != null && debugRigidBody)
			{
				BulletGlobals.g_streamWriter.WriteLine("{0} internalApplyImpule", (String)m_userObjectPointer);
				MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "linComponenet", linearComponent);
				MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "angComponenet", angularComponent);
				BulletGlobals.g_streamWriter.WriteLine("magnitude [{0:0.00000000}]", impulseMagnitude);
			}

		    if (m_inverseMass != 0f)
		    {
			    m_deltaLinearVelocity += linearComponent*impulseMagnitude;
			    m_deltaAngularVelocity += angularComponent*(impulseMagnitude*m_angularFactor);
                MathUtil.SanityCheckVector(ref m_deltaLinearVelocity);
                MathUtil.SanityCheckVector(ref m_deltaAngularVelocity);
            }

	    }


        public void InternalApplyPushImpulse(Vector3 linearComponent, Vector3 angularComponent, float impulseMagnitude)
        {
            InternalApplyPushImpulse(ref linearComponent, ref angularComponent, impulseMagnitude);
        }

        public void InternalApplyPushImpulse(ref Vector3 linearComponent, ref Vector3 angularComponent,float impulseMagnitude)
	    {
		    if (m_inverseMass != 0f)
		    {
			    m_pushVelocity += linearComponent*impulseMagnitude;
			    m_turnVelocity += angularComponent*(impulseMagnitude*m_angularFactor);
		    }
	    }
    	
	    public void	InternalWritebackVelocity()
	    {
		    if (m_inverseMass != 0f)
		    {
			    SetLinearVelocity(GetLinearVelocity()+ m_deltaLinearVelocity);
			    SetAngularVelocity(GetAngularVelocity()+m_deltaAngularVelocity);
			    m_deltaLinearVelocity = Vector3.Zero;
                m_deltaAngularVelocity = Vector3.Zero;
			    //m_originalBody->setCompanionId(-1);
		    }
	    }


        

        public void InternalWritebackVelocity(float timeStep)
        {
			if(BulletGlobals.g_streamWriter != null && debugRigidBody)
			{
				BulletGlobals.g_streamWriter.WriteLine("internalWritebackVelocity ");
				MathUtil.PrintVector3(BulletGlobals.g_streamWriter,"LinearVelocity",GetLinearVelocity());
				MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "DeltaLinearVelocity", m_deltaLinearVelocity);
				MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "AngularVelocity", GetAngularVelocity());
				MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "DeltaAngularVelocity", m_deltaAngularVelocity);

			}

	        if (m_inverseMass != 0f)
	        {
		        SetLinearVelocity(GetLinearVelocity()+ m_deltaLinearVelocity);
		        SetAngularVelocity(GetAngularVelocity()+m_deltaAngularVelocity);
        		
		        //correct the position/orientation based on push/turn recovery
		        Matrix newTransform = Matrix.Identity;
		        TransformUtil.IntegrateTransform(GetWorldTransform(),m_pushVelocity,m_turnVelocity,timeStep,ref newTransform);
		        SetWorldTransform(ref newTransform);
		        //m_originalBody->setCompanionId(-1);
	        }

            if (BulletGlobals.g_streamWriter != null && debugRigidBody)
            {
                BulletGlobals.g_streamWriter.WriteLine("post integrate transform.");
                MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, GetWorldTransform());
            }

            m_deltaLinearVelocity = Vector3.Zero;
            m_deltaAngularVelocity = Vector3.Zero;
            m_pushVelocity = Vector3.Zero;
            m_turnVelocity = Vector3.Zero;
        }
    }


	///The btRigidBodyConstructionInfo structure provides information to create a rigid body. Setting mass to zero creates a fixed (non-dynamic) rigid body.
	///For dynamic objects, you can use the collision shape to approximate the local inertia tensor, otherwise use the zero vector (default argument)
	///You can use the motion state to synchronize the world transform between physics and graphics objects. 
	///And if the motion state is provided, the rigid body will initialize its initial world transform from the motion state,
	///m_startWorldTransform is only used when you don't provide a motion state.
	public class RigidBodyConstructionInfo
	{
		public float m_mass;

		///When a motionState is provided, the rigid body will initialize its world transform from the motion state
		///In this case, m_startWorldTransform is ignored.
		public IMotionState		m_motionState;
		public Matrix	m_startWorldTransform= Matrix.Identity;

		public CollisionShape	m_collisionShape;
		public Vector3			m_localInertia;
		public float			m_linearDamping;
		public float			m_angularDamping;

		///best simulation results when friction is non-zero
		public float			m_friction;
		///best simulation results using zero restitution.
		public float			m_restitution;

		public float			m_linearSleepingThreshold;
		public float			m_angularSleepingThreshold;

		//Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
		//Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics system has improved, this should become obsolete
		public bool				m_additionalDamping;
		public float			m_additionalDampingFactor;
		public float			m_additionalLinearDampingThresholdSqr;
		public float			m_additionalAngularDampingThresholdSqr;
		public float			m_additionalAngularDampingFactor;

        public RigidBodyConstructionInfo(float mass, IMotionState motionState, CollisionShape collisionShape): this(mass,motionState,collisionShape,new Vector3(0,0,0))
        {

        }
		public RigidBodyConstructionInfo(float mass, IMotionState motionState, CollisionShape collisionShape, Vector3 localInertia)
        {
    		m_mass = mass;
			m_motionState =motionState;
			m_collisionShape = collisionShape;
			m_localInertia = localInertia;
			m_linearDamping = 0f;
			m_angularDamping = 0f;
			m_friction = 0.5f;
			m_restitution = 0f;
			m_linearSleepingThreshold = 0.8f;
			m_angularSleepingThreshold = 1f;
			m_additionalDamping = false;
			m_additionalDampingFactor = 0.005f;
			m_additionalLinearDampingThresholdSqr = 0.01f;
			m_additionalAngularDampingThresholdSqr = 0.01f;
			m_additionalAngularDampingFactor = 0.01f;
            m_startWorldTransform = Matrix.Identity;
		}
	}

    [Flags]
    public enum RigidBodyFlags
    {
        BT_NONE = 0,
        BT_DISABLE_WORLD_GRAVITY = 1
    }



}
