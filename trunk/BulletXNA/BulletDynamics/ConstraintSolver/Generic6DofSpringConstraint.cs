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


/// Generic 6 DOF constraint that allows to set spring motors to any translational and rotational DOF

/// DOF index used in enableSpring() and setStiffness() means:
/// 0 : translation X
/// 1 : translation Y
/// 2 : translation Z
/// 3 : rotation X (3rd Euler rotational around new position of X axis, range [-PI+epsilon, PI-epsilon] )
/// 4 : rotation Y (2nd Euler rotational around new position of Y axis, range [-PI/2+epsilon, PI/2-epsilon] )
/// 5 : rotation Z (1st Euler rotational around Z axis, range [-PI+epsilon, PI-epsilon] )
using System;
using System.Diagnostics;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletDynamics
{
    public class Generic6DofSpringConstraint : Generic6DofConstraint
    {
        public Generic6DofSpringConstraint(RigidBody rbA, RigidBody rbB, IndexedMatrix frameInA, IndexedMatrix frameInB, bool useLinearReferenceFrameA)
            : this(rbA, rbB, ref frameInA, ref frameInB, useLinearReferenceFrameA)
        {
            Init();
        }

        
        public Generic6DofSpringConstraint(RigidBody rbA, RigidBody rbB, ref IndexedMatrix frameInA, ref IndexedMatrix frameInB ,bool useLinearReferenceFrameA) : base(rbA,rbB,ref frameInA,ref frameInB,useLinearReferenceFrameA)
        {
            Init();
        }

        protected void Init()
        {
            m_constraintType = TypedConstraintType.D6_SPRING_CONSTRAINT_TYPE;
            for (int i = 0; i < s_degreesOfFreedom; ++i)
            {
                m_springEnabled[i] = false;
                m_equilibriumPoint[i] = 0.0f;
                m_springStiffness[i] = 0.0f;
                m_springDamping[i] = 1.0f;
            }

        }


        protected void InternalUpdateSprings(ConstraintInfo2 info)
        {
            // it is assumed that calculateTransforms() have been called before this call
            IndexedVector3 relVel = m_rbB.GetLinearVelocity() - m_rbA.GetLinearVelocity();
            for (int i = 0; i < 3; i++)
            {
                if (m_springEnabled[i])
                {
                    // get current position of constraint
                    float currPos = m_calculatedLinearDiff[i];
                    // calculate difference
                    float delta = currPos - m_equilibriumPoint[i];
                    // spring force is (delta * m_stiffness) according to Hooke's Law
                    float force = delta * m_springStiffness[i];
                    float velFactor = info.fps * m_springDamping[i] / (float)info.m_numIterations;
                    m_linearLimits.m_targetVelocity[i] = velFactor * force;
                    m_linearLimits.m_maxMotorForce[i] = Math.Abs(force) / info.fps;
                }
            }
            for (int i = 0; i < 3; i++)
            {
                if (m_springEnabled[i + 3])
                {
                    // get current position of constraint
                    float currPos = m_calculatedAxisAngleDiff[i];
                    // calculate difference
                    float delta = currPos - m_equilibriumPoint[i + 3];
                    // spring force is (-delta * m_stiffness) according to Hooke's Law
                    float force = -delta * m_springStiffness[i + 3];
                    float velFactor = info.fps * m_springDamping[i + 3] / (float)info.m_numIterations;
                    m_angularLimits[i].m_targetVelocity = velFactor * force;
                    m_angularLimits[i].m_maxMotorForce = Math.Abs(force) / info.fps;
                }
            }
        }

        public void EnableSpring(int index, bool onOff)
        {
            Debug.Assert((index >= 0) && (index < s_degreesOfFreedom));
            m_springEnabled[index] = onOff;
            if (index < 3)
            {
                m_linearLimits.m_enableMotor[index] = onOff;
            }
            else
            {
                m_angularLimits[index - 3].m_enableMotor = onOff;
            }

        }
	    public void SetStiffness(int index, float stiffness)
        {
            Debug.Assert((index >= 0) && (index < s_degreesOfFreedom));
            m_springStiffness[index] = stiffness;

        }

	    public void SetDamping(int index, float damping)
        {
       	    Debug.Assert((index >= 0) && (index < s_degreesOfFreedom));
	        m_springDamping[index] = damping;
         
        }
	    
        public void SetEquilibriumPoint() // set the current constraint position/orientation as an equilibrium point for all DOF
        {
            CalculateTransforms();

            for (int i = 0; i < 3; i++)
            {
                m_equilibriumPoint[i] = m_calculatedLinearDiff[i];
            }
            for (int i = 0; i < 3; i++)
            {
                m_equilibriumPoint[i + 3] = m_calculatedAxisAngleDiff[i];
            }
        }
	    
        public void SetEquilibriumPoint(int index)  // set the current constraint position/orientation as an equilibrium point for given DOF
        {
            Debug.Assert((index >= 0) && (index < s_degreesOfFreedom));
            CalculateTransforms();
            if (index < 3)
            {
                m_equilibriumPoint[index] = m_calculatedLinearDiff[index];
            }
            else
            {
                m_equilibriumPoint[index] = m_calculatedAxisAngleDiff[index - 3];
            }
        }

		public void SetEquilibriumPoint(int index, float val)
		{
			Debug.Assert((index >= 0) && (index < 6));
			m_equilibriumPoint[index] = val;
		}



	    public override void GetInfo2 (ConstraintInfo2 info)
        {
            InternalUpdateSprings(info);
            base.GetInfo2(info);
        }

		public override void SetAxis(ref IndexedVector3 axis1, ref IndexedVector3 axis2)
		{
			IndexedVector3 zAxis = IndexedVector3.Normalize(axis1);
			IndexedVector3 yAxis = IndexedVector3.Normalize(axis2);
			IndexedVector3 xAxis = IndexedVector3.Cross(yAxis, zAxis); // we want right coordinate system

			IndexedMatrix frameInW = IndexedMatrix.Identity;
            frameInW._basis = new IndexedBasisMatrix(xAxis.X, yAxis.X, zAxis.X,
                                    xAxis.Y, yAxis.Y, zAxis.Y,
                                   xAxis.Z, yAxis.Z, zAxis.Z);

			// now get constraint frame in local coordinate systems
            m_frameInA = m_rbA.GetCenterOfMassTransform().Inverse() * frameInW;
            m_frameInB = m_rbB.GetCenterOfMassTransform().Inverse() * frameInW;

			CalculateTransforms();
		}



        protected bool[] m_springEnabled = new bool[s_degreesOfFreedom];
        protected float[] m_equilibriumPoint = new float[s_degreesOfFreedom];
        protected float[] m_springStiffness = new float[s_degreesOfFreedom];
        protected float[] m_springDamping = new float[s_degreesOfFreedom]; // between 0 and 1 (1 == no damping)
        private const int s_degreesOfFreedom = 6;
    }

}
