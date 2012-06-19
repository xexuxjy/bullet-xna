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
using System.IO;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletDynamics
{
	public enum TypedConstraintType
	{
		POINT2POINT_CONSTRAINT_TYPE = 3,
		HINGE_CONSTRAINT_TYPE,
		CONETWIST_CONSTRAINT_TYPE,
		D6_CONSTRAINT_TYPE,
		SLIDER_CONSTRAINT_TYPE,
		CONTACT_CONSTRAINT_TYPE,
		D6_SPRING_CONSTRAINT_TYPE,
		MAX_CONSTRAINT_TYPE
	}

	public enum ConstraintParams
	{
		BT_CONSTRAINT_ERP = 1,
		BT_CONSTRAINT_STOP_ERP,
		BT_CONSTRAINT_CFM,
		BT_CONSTRAINT_STOP_CFM
	}

	public class TypedConstraint : TypedObject
	{
		///TypedConstraint is the baseclass for Bullet constraints and vehicles

		private static RigidBody s_fixed;

		public const float DEFAULT_DEBUGDRAW_SIZE = 0.3f;

        public String m_debugName;

		private int m_userConstraintType;

		// warning - these are unioned in the c++ version
		private int m_userConstraintId;
		private Object m_userConstraintPtr;

		private float m_breakingImpulseThreshold;
		private bool m_isEnabled;


		private bool m_needsFeedback;
        private int m_overrideNumSolverIterations;

		protected TypedConstraintType m_constraintType;

		protected RigidBody m_rbA;
		protected RigidBody m_rbB;
		protected float m_appliedImpulse;
		protected float m_dbgDrawSize;

		//public TypedConstraint(TypedConstraintType type)
		//{
		//    m_userConstraintType = -1;
		//    m_userConstraintId = -1;
		//    m_constraintType = type;
		//    m_rbA = s_fixed;
		//    m_rbB = s_fixed;
		//    m_appliedImpulse = 0f;
		//    m_dbgDrawSize = DEFAULT_DEBUGDRAW_SIZE;
		//    {
		//        s_fixed.setMassProps(0f,IndexedVector3.Zero);
		//    }

		//}

		public TypedConstraint()
			: base(-1)
		{

		}

		public TypedConstraint(TypedConstraintType type, RigidBody rbA)
			: base((int)type)
		{
			m_userConstraintType = -1;
			m_userConstraintId = -1;
			m_constraintType = type;
			m_rbA = rbA;
			m_rbB = GetFixedBody();
			m_appliedImpulse = 0f;
			m_breakingImpulseThreshold = MathUtil.SIMD_INFINITY;
			m_isEnabled = true;
			m_dbgDrawSize = DEFAULT_DEBUGDRAW_SIZE;
			{
				s_fixed.SetMassProps(0f, IndexedVector3.Zero);
			}
		}

		public TypedConstraint(TypedConstraintType type, RigidBody rbA, RigidBody rbB)
			: base((int)type)
		{
			m_userConstraintType = -1;
			m_userConstraintId = -1;
			m_constraintType = type;
			m_rbA = rbA;
			m_rbB = rbB;
			m_appliedImpulse = 0f;
			m_breakingImpulseThreshold = MathUtil.SIMD_INFINITY;
			m_isEnabled = true;
			m_dbgDrawSize = DEFAULT_DEBUGDRAW_SIZE;
			{
				GetFixedBody().SetMassProps(0f, IndexedVector3.Zero);
			}

		}

		public virtual void SetupSolverConstraint(IList<SolverConstraint> ca, int solverBodyA, int solverBodyB, float timeStep)
		{
		}

		public virtual void GetInfo1(ConstraintInfo1 info)
		{


		}

		public virtual void GetInfo2(ConstraintInfo2 info)
		{

		}

		///internal method used by the constraint solver, don't use them directly
		public void InternalSetAppliedImpulse(float appliedImpulse)
		{
			m_appliedImpulse = appliedImpulse;
		}
		///internal method used by the constraint solver, don't use them directly
		public float InternalGetAppliedImpulse()
		{
			return m_appliedImpulse;
		}

	    public int	GetOverrideNumSolverIterations()
	    {
		    return m_overrideNumSolverIterations;
	    }

	    ///override the number of constraint solver iterations used to solve this constraint
	    ///-1 will use the default number of iterations, as specified in SolverInfo.m_numIterations
	    public void SetOverrideNumSolverIterations(int overideNumIterations)
	    {
		    m_overrideNumSolverIterations = overideNumIterations;
	    }


		protected float GetMotorFactor(float pos, float lowLim, float uppLim, float vel, float timeFact)
		{
			if (lowLim > uppLim)
			{
				return 1f;
			}
			else if (MathUtil.CompareFloat(lowLim, uppLim))
			{
				return 0f;
			}
			float lim_fact = 1.0f;
			float delta_max = vel / timeFact;
			if (delta_max < 0.0f)
			{
				if ((pos >= lowLim) && (pos < (lowLim - delta_max)))
				{
					lim_fact = (lowLim - pos) / delta_max;
				}
				else if (pos < lowLim)
				{
					lim_fact = 0.0f;
				}
				else
				{
					lim_fact = 1f;
				}
			}
			else if (delta_max > 0.0f)
			{
				if ((pos <= uppLim) && (pos > (uppLim - delta_max)))
				{
					lim_fact = (uppLim - pos) / delta_max;
				}
				else if (pos > uppLim)
				{
					lim_fact = 0.0f;
				}
				else
				{
					lim_fact = 1.0f;
				}
			}
			else
			{
				lim_fact = 0.0f;
			}
			return lim_fact;

		}

		public static RigidBody GetFixedBody()
		{
			//static btRigidBody s_fixed(0, 0,0);
			//s_fixed.setMassProps(float(0.),btVector3(float(0.),float(0.),float(0.)));
			if (s_fixed == null)
			{
				s_fixed = new RigidBody(0f, null, null, IndexedVector3.Zero);
			}
			IndexedVector3 inertia = IndexedVector3.Zero;
			s_fixed.SetMassProps(0f, ref inertia);
			return s_fixed;
		}

		public RigidBody GetRigidBodyA()
		{
			return m_rbA;
		}

		public RigidBody GetRigidBodyB()
		{
			return m_rbB;
		}

		public int GetUserConstraintType()
		{
			return m_userConstraintType;
		}

		public void SetUserConstraintType(int userConstraintType)
		{
			m_userConstraintType = userConstraintType;
		}

		public void SetUserConstraintId(int uid)
		{
			m_userConstraintId = uid;
		}

		public int GetUserConstraintId()
		{
			return m_userConstraintId;
		}

		public int GetUid()
		{
			return m_userConstraintId;
		}

		public bool NeedsFeedback()
		{
			return m_needsFeedback;
		}

		///enableFeedback will allow to read the applied linear and angular impulse
		///use getAppliedImpulse, getAppliedLinearImpulse and getAppliedAngularImpulse to read feedback information
		public void EnableFeedback(bool needsFeedback)
		{
			m_needsFeedback = needsFeedback;
		}

		public float GetAppliedImpulse()
		{
			return m_appliedImpulse;
		}

		public TypedConstraintType GetConstraintType()
		{
			return m_constraintType;
		}

		public void SetDbgDrawSize(float dbgDrawSize)
		{
			m_dbgDrawSize = dbgDrawSize;
		}
		public float GetDbgDrawSize()
		{
			return m_dbgDrawSize;
		}


		// returns angle in range [-SIMD_2_PI, SIMD_2_PI], closest to one of the limits 
		// all arguments should be normalized angles (i.e. in range [-SIMD_PI, SIMD_PI])
		public float AdjustAngleToLimits(float angleInRadians, float angleLowerLimitInRadians, float angleUpperLimitInRadians)
		{
			if (angleLowerLimitInRadians >= angleUpperLimitInRadians)
			{
				return angleInRadians;
			}
			else if (angleInRadians < angleLowerLimitInRadians)
			{
				float diffLo = Math.Abs(MathUtil.NormalizeAngle(angleLowerLimitInRadians - angleInRadians)); // this is positive
				float diffHi = Math.Abs(MathUtil.NormalizeAngle(angleUpperLimitInRadians - angleInRadians));
				return (diffLo < diffHi) ? angleInRadians : (angleInRadians + MathUtil.SIMD_2_PI);
			}
			else if (angleInRadians > angleUpperLimitInRadians)
			{
				float diffHi = Math.Abs(MathUtil.NormalizeAngle(angleInRadians - angleUpperLimitInRadians)); // this is positive
				float diffLo = Math.Abs(MathUtil.NormalizeAngle(angleInRadians - angleLowerLimitInRadians));
				return (diffLo < diffHi) ? (angleInRadians - MathUtil.SIMD_2_PI) : angleInRadians;
			}
			else
			{
				return angleInRadians;
			}
		}

		public virtual void Cleanup() { }


		public float GetBreakingImpulseThreshold()
		{
			return m_breakingImpulseThreshold;
		}

		public void SetBreakingImpulseThreshold(float threshold)
		{
			m_breakingImpulseThreshold = threshold;
		}

		public bool IsEnabled()
		{
			return m_isEnabled;
		}

		public void SetEnabled(bool enabled)
		{
			m_isEnabled = enabled;
		}

    	///internal method used by the constraint solver, don't use them directly
	    public virtual	void SolveConstraintObsolete(RigidBody bodyA,RigidBody bodyB,float timeStep) 
        {

        }

        public virtual void BuildJacobian()
        {


        }

		public static void PrintInfo1(StreamWriter writer, TypedConstraint constraint, ConstraintInfo1 info)
		{
			if (writer != null)
			{
				writer.WriteLine("getInfo1 [{0}] [{1}] [{2}] [{3}]", constraint.m_userConstraintId, constraint.GetObjectType(), (string)constraint.GetRigidBodyA().GetUserPointer(), (string)constraint.GetRigidBodyB().GetUserPointer());
				MathUtil.PrintMatrix(writer, "rBA cmot", constraint.GetRigidBodyA().GetCenterOfMassTransform());
				MathUtil.PrintMatrix(writer, "rBB cmot", constraint.GetRigidBodyB().GetCenterOfMassTransform());
				MathUtil.PrintMatrix(writer, "rBA inv tensor", constraint.GetRigidBodyA().GetInvInertiaTensorWorld());
				MathUtil.PrintMatrix(writer, "rBB inv tensor", constraint.GetRigidBodyB().GetInvInertiaTensorWorld());
				writer.WriteLine(String.Format("NumRows [{0}] Nub[{1}]", info.m_numConstraintRows, info.nub));
			}

		}

		public static void PrintInfo2(StreamWriter writer, TypedConstraint constraint, ConstraintInfo2 info2)
		{
			if (writer != null)
			{

				writer.WriteLine(String.Format("getInfo2 [{0}] [{1}] [{2}] [{3}]", constraint.m_userConstraintId, constraint.GetObjectType(), (string)constraint.GetRigidBodyA().GetUserPointer(), (string)constraint.GetRigidBodyB().GetUserPointer()));
				writer.WriteLine(String.Format("numRows [{0}] fps[{1:0.00000000}] erp[{2:0.00000000}] findex[{3}] numIter[{4}]", info2.m_numRows, info2.fps, info2.erp, info2.findex, info2.m_numIterations));
				for (int i = 0; i < info2.m_numRows; ++i)
				{
					writer.WriteLine(String.Format("TypedConstraint[{0}]", i));
					writer.WriteLine("ContactNormal");
					MathUtil.PrintVector3(writer, info2.m_solverConstraints[i].m_contactNormal);
					writer.WriteLine("rel1pos1CrossNormal");
					MathUtil.PrintVector3(writer, info2.m_solverConstraints[i].m_relpos1CrossNormal);
					writer.WriteLine("rel1pos2CrossNormal");
					MathUtil.PrintVector3(writer, info2.m_solverConstraints[i].m_relpos2CrossNormal);

				}
			}

		}

		public static void PrintSolverConstraint(StreamWriter writer, SolverConstraint constraint, int index)
		{
			if (writer != null)
			{
				writer.WriteLine("SolverConstraint[{0}][{1}][{2}]", index,(String)constraint.m_solverBodyA.GetUserPointer(),(String)constraint.m_solverBodyB.GetUserPointer());
                MathUtil.PrintVector3(writer, "relPos1CrossNormal", constraint.m_relpos1CrossNormal);
                MathUtil.PrintVector3(writer, "contactNormal", constraint.m_contactNormal);
                MathUtil.PrintVector3(writer, "m_angularComponentA", constraint.m_angularComponentA);
                MathUtil.PrintVector3(writer, "m_angularComponentB", constraint.m_angularComponentB);
                writer.WriteLine("Friction [{0:0.00000000}] jagDiag[{1:0.00000000}] rhs[{2:0.00000000}] cfm[{3:0.00000000}] lower[{4:0.00000000}] upper[{5:0.00000000}] rhsPen[{6:0.00000000}]", constraint.m_friction, constraint.m_jacDiagABInv,
                    constraint.m_rhs, constraint.m_cfm, constraint.m_lowerLimit, constraint.m_lowerLimit, constraint.m_rhsPenetration);
			}
		}




	}

	public class ConstraintInfo1
	{
		public int m_numConstraintRows;
		public int nub;

        ~ConstraintInfo1()
        {
            int ibreak = 0;
        }

	}

	public class ConstraintInfo2
	{
        // little dodgy...
        public const int maxConstraints = 6;
		// Workaround for the idea of having multiple solver constraints and row count.
		// This should be populated with a list of the SolverConstraint for a give Constraint. - MAN
        public SolverConstraint[] m_solverConstraints = new SolverConstraint[maxConstraints];
        public int m_numRows = 0;
		// integrator parameters: frames per second (1/stepsize), default error
		// reduction parameter (0..1).
		public float fps;
		public float erp;

		// findex vector for variables. see the LCP solver interface for a
		// description of what this does. this is set to -1 on entry.
		// note that the returned indexes are relative to the first index of
		// the constraint.
		public int findex;
		public int m_numIterations;
		//damping of the velocity
		public float m_damping;

        public void Reset()
        {

            for (int i = 0; i < maxConstraints; ++i)
            {
                m_solverConstraints[i] = null;
            }
            m_numRows = 0;
            fps = 0f;
            erp = 0;
            findex = 0;
            m_numIterations = 0;
            m_damping = 0f;
        }

        ~ConstraintInfo2()
        {
            int ibreak = 0;
        }
	}


	public class AngularLimit
	{
		private float m_center;
		private float m_halfRange;
		private float m_softness;
		private float m_biasFactor;
		private float m_relaxationFactor;
		private float m_correction;
		private float m_sign;

		private bool m_solveLimit;

		/// Default constructor initializes limit as inactive, allowing free constraint movement
		public AngularLimit()
		{
			m_center = 0.0f;
			m_halfRange = -1.0f;
			m_softness = 0.9f;
			m_biasFactor = 0.3f;
			m_relaxationFactor = 1.0f;
			m_correction = 0.0f;
			m_sign = 0.0f;
			m_solveLimit = false;
		}

		/// Sets all limit's parameters.
		/// When low > high limit becomes inactive.
		/// When high - low > 2PI limit is ineffective too becouse no angle can exceed the limit
		public void Set(float low, float high)
		{
			Set(low, high, 0.9f, 0.3f, 1.0f);
		}

		public void Set(float low, float high, float _softness, float _biasFactor, float _relaxationFactor)
		{
			m_halfRange = (high - low) / 2.0f;
			m_center = MathUtil.NormalizeAngle(low + m_halfRange);
			m_softness = _softness;
			m_biasFactor = _biasFactor;
			m_relaxationFactor = _relaxationFactor;
		}

		/// Checks conastaint angle against limit. If limit is active and the angle violates the limit
		/// correction is calculated.
		public void Test(float angle)
		{
			m_correction = 0.0f;
			m_sign = 0.0f;
			m_solveLimit = false;

			if (m_halfRange >= 0.0f)
			{
				float deviation = MathUtil.NormalizeAngle(angle - m_center);
				if (deviation < -m_halfRange)
				{
					m_solveLimit = true;
					m_correction = -(deviation + m_halfRange);
					m_sign = +1.0f;
				}
				else if (deviation > m_halfRange)
				{
					m_solveLimit = true;
					m_correction = m_halfRange - deviation;
					m_sign = -1.0f;
				}
			}
		}

		/// Returns limit's softness
		public float GetSoftness()
		{
			return m_softness;
		}

		/// Returns limit's bias factor
		public float GetBiasFactor()
		{
			return m_biasFactor;
		}

		/// Returns limit's relaxation factor
		public float GetRelaxationFactor()
		{
			return m_relaxationFactor;
		}

		/// Returns correction value evaluated when test() was invoked 
		public float GetCorrection()
		{
			return m_correction;
		}

		/// Returns sign value evaluated when test() was invoked 
		public float GetSign()
		{
			return m_sign;
		}

		/// Gives half of the distance between min and max limit angle
		public float GetHalfRange()
		{
			return m_halfRange;
		}

		/// Returns true when the last test() invocation recognized limit violation
		public bool IsLimit()
		{
			return m_solveLimit;
		}

		/// Checks given angle against limit. If limit is active and angle doesn't fit it, the angle
		/// returned is modified so it equals to the limit closest to given angle.
		public void Fit(ref float angle)
		{
			if (m_halfRange > 0.0f)
			{
				float relativeAngle = MathUtil.NormalizeAngle(angle - m_center);
				if (!MathUtil.CompareFloat(relativeAngle, m_halfRange))
				{
					if (relativeAngle > 0.0f)
					{
						angle = GetHigh();
					}
					else
					{
						angle = GetLow();
					}
				}
			}
		}

		/// Returns correction value multiplied by sign value
		public float GetError()
		{
			return m_correction * m_sign;
		}

		public float GetLow()
		{
			return MathUtil.NormalizeAngle(m_center - m_halfRange);
		}

		public float GetHigh()
		{
			return MathUtil.NormalizeAngle(m_center + m_halfRange);
		}

	}


}
