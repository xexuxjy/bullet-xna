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

#define _BT_USE_CENTER_LIMIT_

using System;
using BulletXNA.LinearMath;


namespace BulletXNA.BulletDynamics
{
	public class HingeConstraint : TypedConstraint
	{
		private const bool HINGE_USE_FRAME_OFFSET = true;

		private static IndexedVector3 vHinge = new IndexedVector3(0, 0, 1);

        private IndexedMatrix m_rbAFrame = IndexedMatrix.Identity; // constraint axii. Assumes z is hinge axis.
		private IndexedMatrix m_rbBFrame = IndexedMatrix.Identity;

		private float m_motorTargetVelocity;
		private float m_maxMotorImpulse;

#if	_BT_USE_CENTER_LIMIT_
		AngularLimit m_limit;
#else
	private float	m_lowerLimit;	
	private float	m_upperLimit;	
	private float	m_limitSign;
	private float	m_correction;

	private float	m_limitSoftness; 
	private float	m_biasFactor; 
	private float	m_relaxationFactor; 

	private bool		m_solveLimit;
#endif

		private float m_kHinge;


		private float m_accLimitImpulse;
		private float m_hingeAngle;
		private float m_referenceSign;

		private bool m_angularOnly;
		private bool m_enableAngularMotor;
		private bool m_useOffsetForConstraintFrame;
		private bool m_useReferenceFrameA;

		private float m_accMotorImpulse;

		private int m_flags;
		private float m_normalCFM;
		private float m_stopCFM;
		private float m_stopERP;
        
        public HingeConstraint(RigidBody rbA, RigidBody rbB, ref IndexedVector3 pivotInA, ref IndexedVector3 pivotInB, ref IndexedVector3 axisInA, ref IndexedVector3 axisInB)
			: this(rbA, rbB, ref pivotInA, ref pivotInB, ref axisInA, ref axisInB, false)
		{
		}


		public HingeConstraint(RigidBody rbA, RigidBody rbB, ref IndexedVector3 pivotInA, ref IndexedVector3 pivotInB, ref IndexedVector3 axisInA, ref IndexedVector3 axisInB, bool useReferenceFrameA)
			: base(TypedConstraintType.HINGE_CONSTRAINT_TYPE, rbA, rbB)
		{
			m_angularOnly = false;
			m_enableAngularMotor = false;
			m_useOffsetForConstraintFrame = HINGE_USE_FRAME_OFFSET;

			m_useReferenceFrameA = useReferenceFrameA;

			m_rbAFrame._origin = pivotInA;
#if	_BT_USE_CENTER_LIMIT_
			m_limit = new AngularLimit();
#endif


			m_flags = 0;

			// since no frame is given, assume this to be zero angle and just pick rb transform axis
            IndexedVector3 rbAxisA1 = rbA.GetCenterOfMassTransform()._basis.GetColumn(0);

			IndexedVector3 rbAxisA2 = IndexedVector3.Zero;
			float projection = IndexedVector3.Dot(axisInA, rbAxisA1);
			if (projection >= 1.0f - MathUtil.SIMD_EPSILON)
			{
                rbAxisA1 = -rbA.GetCenterOfMassTransform()._basis.GetColumn(2);
                rbAxisA2 = rbA.GetCenterOfMassTransform()._basis.GetColumn(1);
            }
			else if (projection <= -1.0f + MathUtil.SIMD_EPSILON)
			{
                rbAxisA1 = rbA.GetCenterOfMassTransform()._basis.GetColumn(2);
                rbAxisA2 = rbA.GetCenterOfMassTransform()._basis.GetColumn(1);
            }
			else
			{
				rbAxisA2 = IndexedVector3.Cross(axisInA, rbAxisA1);
				rbAxisA1 = IndexedVector3.Cross(rbAxisA2, axisInA);
			}

            
            //m_rbAFrame._basis = new IndexedBasisMatrix(ref rbAxisA1, ref rbAxisA2, ref axisInA);
            m_rbAFrame._basis = new IndexedBasisMatrix(rbAxisA1.X, rbAxisA2.X, axisInA.X,
                                    rbAxisA1.Y, rbAxisA2.Y, axisInA.Y,
                                    rbAxisA1.Z, rbAxisA2.Z, axisInA.Z);

			IndexedQuaternion rotationArc = MathUtil.ShortestArcQuat(ref axisInA, ref axisInB);
			IndexedVector3 rbAxisB1 = MathUtil.QuatRotate(ref rotationArc, ref rbAxisA1);
			IndexedVector3 rbAxisB2 = IndexedVector3.Cross(axisInB, rbAxisB1);

			m_rbBFrame._origin = pivotInB;
            //m_rbBFrame._basis = new IndexedBasisMatrix(ref rbAxisB1, ref rbAxisB2, ref axisInB);
            m_rbBFrame._basis = new IndexedBasisMatrix(rbAxisB1.X, rbAxisB2.X, axisInB.X,
                                    rbAxisB1.Y, rbAxisB2.Y, axisInB.Y,
                                    rbAxisB1.Z, rbAxisB2.Z, axisInB.Z);

#if!	_BT_USE_CENTER_LIMIT_
	//start with free
	m_lowerLimit = float(1.0f);
	m_upperLimit = float(-1.0f);
	m_biasFactor = 0.3f;
	m_relaxationFactor = 1.0f;
	m_limitSoftness = 0.9f;
	m_solveLimit = false;
#endif
			m_referenceSign = m_useReferenceFrameA ? -1f : 1f;
		}

		public HingeConstraint(RigidBody rbA, ref IndexedVector3 pivotInA, ref IndexedVector3 axisInA, bool useReferenceFrameA)
			: base(TypedConstraintType.HINGE_CONSTRAINT_TYPE, rbA)
		{
			m_angularOnly = false;
			m_enableAngularMotor = false;
			m_useReferenceFrameA = useReferenceFrameA;
			m_useOffsetForConstraintFrame = HINGE_USE_FRAME_OFFSET;
			m_flags = 0;

#if	_BT_USE_CENTER_LIMIT_
			m_limit = new AngularLimit();
#endif

			// since no frame is given, assume this to be zero angle and just pick rb transform axis
			// fixed axis in worldspace
			IndexedVector3 rbAxisA1, rbAxisA2;
			TransformUtil.PlaneSpace1(ref axisInA, out rbAxisA1, out rbAxisA2);

			m_rbAFrame._origin = pivotInA;
            //m_rbAFrame._basis = new IndexedBasisMatrix(ref rbAxisA1, ref rbAxisA2, ref axisInA);
            m_rbAFrame._basis = new IndexedBasisMatrix(rbAxisA1.X, rbAxisA2.X, axisInA.X,
                                                rbAxisA1.Y, rbAxisA2.Y, axisInA.Y,
                                                rbAxisA1.Z, rbAxisA2.Z, axisInA.Z);

            IndexedVector3 axisInB = rbA.GetCenterOfMassTransform()._basis * axisInA;

			IndexedQuaternion rotationArc = MathUtil.ShortestArcQuat(ref axisInA, ref axisInB);
			IndexedVector3 rbAxisB1 = MathUtil.QuatRotate(ref rotationArc, ref rbAxisA1);
			IndexedVector3 rbAxisB2 = IndexedVector3.Cross(axisInB, rbAxisB1);

            m_rbBFrame._origin = rbA.GetCenterOfMassTransform() * pivotInA;
            //m_rbBFrame._basis = new IndexedBasisMatrix(ref rbAxisB1, ref rbAxisB2, ref axisInB);
            m_rbBFrame._basis = new IndexedBasisMatrix(rbAxisB1.X, rbAxisB2.X, axisInB.X,
                        rbAxisB1.Y, rbAxisB2.Y, axisInB.Y,
                        rbAxisB1.Z, rbAxisB2.Z, axisInB.Z);

			//start with free
#if!	_BT_USE_CENTER_LIMIT_

            m_lowerLimit = 1f;
            m_upperLimit = -1f;
            m_biasFactor = 0.3f;
            m_relaxationFactor = 1.0f;
            m_limitSoftness = 0.9f;
            m_solveLimit = false;
#endif
			m_referenceSign = m_useReferenceFrameA ? -1.0f : 1.0f;
		}

		public HingeConstraint(RigidBody rbA, RigidBody rbB, ref IndexedMatrix rbAFrame, ref IndexedMatrix rbBFrame)
			: this(rbA, rbB, ref rbAFrame, ref rbBFrame, false)
		{
		}

		public HingeConstraint(RigidBody rbA, RigidBody rbB, ref IndexedMatrix rbAFrame, ref IndexedMatrix rbBFrame, bool useReferenceFrameA)
			: base(TypedConstraintType.HINGE_CONSTRAINT_TYPE, rbA, rbB)
		{
			m_rbAFrame = rbAFrame;
			m_rbBFrame = rbBFrame;
			m_angularOnly = false;
			m_enableAngularMotor = false;
			m_useOffsetForConstraintFrame = HINGE_USE_FRAME_OFFSET;
			m_useReferenceFrameA = useReferenceFrameA;
			m_flags = 0;

#if	_BT_USE_CENTER_LIMIT_
			m_limit = new AngularLimit();
#else

            //start with free
            m_lowerLimit = 1f;
            m_upperLimit = -1f;
            m_biasFactor = 0.3f;
            m_relaxationFactor = 1.0f;
            m_limitSoftness = 0.9f;
            m_solveLimit = false;
#endif
			m_referenceSign = m_useReferenceFrameA ? -1.0f : 1.0f;
		}

		public HingeConstraint(RigidBody rbA, ref IndexedMatrix rbAFrame)
			: this(rbA, ref rbAFrame, false)
		{
		}

		public HingeConstraint(RigidBody rbA, ref IndexedMatrix rbAFrame, bool useReferenceFrameA)
			: base(TypedConstraintType.HINGE_CONSTRAINT_TYPE, rbA)
		{
			m_rbAFrame = rbAFrame;
			m_rbBFrame = rbAFrame;
			m_angularOnly = false;
			m_enableAngularMotor = false;
			m_useOffsetForConstraintFrame = HINGE_USE_FRAME_OFFSET;

			m_useReferenceFrameA = useReferenceFrameA;
			m_flags = 0;

            m_rbBFrame._origin = m_rbA.GetCenterOfMassTransform() * (m_rbAFrame._origin);

#if	_BT_USE_CENTER_LIMIT_
			m_limit = new AngularLimit();
#else

            //start with free
            m_lowerLimit = 1f;
            m_upperLimit = -1f;
            m_biasFactor = 0.3f;
            m_relaxationFactor = 1.0f;
            m_limitSoftness = 0.9f;
            m_solveLimit = false;
#endif
			m_referenceSign = m_useReferenceFrameA ? -1.0f : 1.0f;

		}


		public IndexedMatrix GetFrameOffsetA()
		{
			return m_rbAFrame;
		}

		public IndexedMatrix GetFrameOffsetB()
		{
			return m_rbBFrame;
		}

		public void setFrames(ref IndexedMatrix frameA, ref IndexedMatrix frameB)
		{
            m_rbAFrame = frameA;
            m_rbBFrame = frameB;
		}


		public override void GetInfo1(ConstraintInfo1 info)
		{
            info.m_numConstraintRows = 5; // Fixed 3 linear + 2 angular
            info.nub = 1;
            //prepare constraint
            TestLimit(m_rbA.GetCenterOfMassTransform(), m_rbB.GetCenterOfMassTransform());
            if (GetSolveLimit() || GetEnableAngularMotor())
            {
                info.m_numConstraintRows++; // limit 3rd anguar as well
                info.nub--;
            }
#if DEBUG            
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugConstraints)
            {
                PrintInfo1(BulletGlobals.g_streamWriter, this, info);
            }
#endif            
		}

		public void GetInfo1NonVirtual(ConstraintInfo1 info)
		{
            //always add the 'limit' row, to avoid computation (data is not available yet)
            info.m_numConstraintRows = 6; // Fixed 3 linear + 2 angular
            info.nub = 0;
#if DEBUG            
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugConstraints)
            {
                PrintInfo1(BulletGlobals.g_streamWriter, this, info);
            }
#endif            
		}

		public override void GetInfo2(ConstraintInfo2 info)
		{
			if (m_useOffsetForConstraintFrame)
			{
				GetInfo2InternalUsingFrameOffset(info, m_rbA.GetCenterOfMassTransform(), m_rbB.GetCenterOfMassTransform(), m_rbA.GetAngularVelocity(), m_rbB.GetAngularVelocity());
			}
			else
			{
				GetInfo2Internal(info, m_rbA.GetCenterOfMassTransform(), m_rbB.GetCenterOfMassTransform(), m_rbA.GetAngularVelocity(), m_rbB.GetAngularVelocity());
			}
		}

		public void GetInfo2NonVirtual(ConstraintInfo2 info, IndexedMatrix transA, IndexedMatrix transB, IndexedVector3 angVelA, IndexedVector3 angVelB)
		{
			///the regular (virtual) implementation getInfo2 already performs 'testLimit' during getInfo1, so we need to do it now
			TestLimit(ref transA, ref transB);
			GetInfo2Internal(info, transA, transB, angVelA, angVelB);
		}

		public void GetInfo2Internal(ConstraintInfo2 info, IndexedMatrix transA, IndexedMatrix transB, IndexedVector3 angVelA, IndexedVector3 angVelB)
		{
			// transforms in world space
            IndexedMatrix trA = transA * m_rbAFrame;
            IndexedMatrix trB = transB * m_rbBFrame;
			// pivot point
			IndexedVector3 pivotAInW = trA._origin;
			IndexedVector3 pivotBInW = trB._origin;

			// linear (all fixed)
			//info.m_J1linearAxis[0] = 1;
			//info.m_J1linearAxis[s + 1] = 1;
			//info.m_J1linearAxis[2 * s + 2] = 1;
			if (!m_angularOnly)
			{
				info.m_solverConstraints[0].m_contactNormal.X = 1f;
				info.m_solverConstraints[1].m_contactNormal.Y = 1f;
				info.m_solverConstraints[2].m_contactNormal.Z = 1f;
			}

			IndexedVector3 a1 = pivotAInW - transA._origin;
			{
				IndexedVector3 a1neg = -a1;
				MathUtil.GetSkewSymmetricMatrix(ref a1neg,
					out info.m_solverConstraints[0].m_relpos1CrossNormal,
					out info.m_solverConstraints[1].m_relpos1CrossNormal,
					out info.m_solverConstraints[2].m_relpos1CrossNormal);
				//if (info.m_solverConstraints[0].m_relpos1CrossNormal.X == 0.15)
				//{
				//    int ibreak = 0;
				//}
				int ibreak = 0;

			}
			IndexedVector3 a2 = pivotBInW - transB._origin;
			{
				MathUtil.GetSkewSymmetricMatrix(ref a2,
					out info.m_solverConstraints[0].m_relpos2CrossNormal,
					out info.m_solverConstraints[1].m_relpos2CrossNormal,
					out info.m_solverConstraints[2].m_relpos2CrossNormal);
			}
			// linear RHS
			float k = info.fps * info.erp;
			if (!m_angularOnly)
			{
				for (int i = 0; i < 3; i++)
				{
					float val = k * (pivotBInW[i] - pivotAInW[i]);
					info.m_solverConstraints[i].m_rhs = val;
				}
			}
			// make rotations around X and Y equal
			// the hinge axis should be the only unconstrained
			// rotational axis, the angular velocity of the two bodies perpendicular to
			// the hinge axis should be equal. thus the constraint equations are
			//    p*w1 - p*w2 = 0
			//    q*w1 - q*w2 = 0
			// where p and q are unit vectors normal to the hinge axis, and w1 and w2
			// are the angular velocity vectors of the two bodies.
			// get hinge axis (Z)
            IndexedVector3 ax1 = trA._basis.GetColumn(2);
			// get 2 orthos to hinge axis (X, Y)
            IndexedVector3 p = trA._basis.GetColumn(0);
            IndexedVector3 q = trA._basis.GetColumn(1);
			// set the two hinge angular rows 


			MathUtil.SanityCheckVector(ax1);
			MathUtil.SanityCheckVector(p);
			MathUtil.SanityCheckVector(q);

			int s3 = 3;
			int s4 = 4;

			info.m_solverConstraints[s3].m_relpos1CrossNormal = p;
			info.m_solverConstraints[s4].m_relpos1CrossNormal = q;

			info.m_solverConstraints[s3].m_relpos2CrossNormal = -p;
			info.m_solverConstraints[s4].m_relpos2CrossNormal = -q;

			// compute the right hand side of the constraint equation. set relative
			// body velocities along p and q to bring the hinge back into alignment.
			// if ax1,ax2 are the unit length hinge axes as computed from body1 and
			// body2, we need to rotate both bodies along the axis u = (ax1 x ax2).
			// if `theta' is the angle between ax1 and ax2, we need an angular velocity
			// along u to cover angle erp*theta in one step :
			//   |angular_velocity| = angle/time = erp*theta / stepsize
			//                      = (erp*fps) * theta
			//    angular_velocity  = |angular_velocity| * (ax1 x ax2) / |ax1 x ax2|
			//                      = (erp*fps) * theta * (ax1 x ax2) / sin(theta)
			// ...as ax1 and ax2 are unit length. if theta is smallish,
			// theta ~= sin(theta), so
			//    angular_velocity  = (erp*fps) * (ax1 x ax2)
			// ax1 x ax2 is in the plane space of ax1, so we project the angular
			// velocity to p and q to find the right hand side.
            IndexedVector3 ax2 = trB._basis.GetColumn(2); 
			IndexedVector3 u = IndexedVector3.Cross(ax1, ax2);

			info.m_solverConstraints[s3].m_rhs = k * IndexedVector3.Dot(u, p);
			info.m_solverConstraints[s4].m_rhs = k * IndexedVector3.Dot(u, q);

			// check angular limits
			int nrow = 4; // last filled row

			float limit_err = 0.0f;
			int limit = 0;
			if (GetSolveLimit())
			{
#if	_BT_USE_CENTER_LIMIT_
				limit_err = m_limit.GetCorrection() * m_referenceSign;
#else
	limit_err = m_correction * m_referenceSign;
#endif
			}
			// if the hinge has joint limits or motor, add in the extra row
			bool powered = false;
			if (GetEnableAngularMotor())
			{
				powered = true;
			}
			if (limit != 0 || powered)
			{
				nrow++;
				info.m_solverConstraints[nrow].m_relpos1CrossNormal = ax1;
				info.m_solverConstraints[nrow].m_relpos2CrossNormal = -ax1;

				float lostop = GetLowerLimit();
				float histop = GetUpperLimit();
				if (limit != 0 && (MathUtil.CompareFloat(lostop, histop)))
				{  // the joint motor is ineffective
					powered = false;
				}
				info.m_solverConstraints[nrow].m_rhs = 0.0f;
				float currERP = ((m_flags & (int)HingeFlags.BT_HINGE_FLAGS_ERP_STOP) != 0) ? m_stopERP : info.erp;

				if (powered)
				{
					if ((m_flags & (int)HingeFlags.BT_HINGE_FLAGS_CFM_NORM) != 0)
					{
						info.m_solverConstraints[nrow].m_cfm = m_normalCFM;
					}

					float mot_fact = GetMotorFactor(m_hingeAngle, lostop, histop, m_motorTargetVelocity, info.fps * currERP);
					info.m_solverConstraints[nrow].m_rhs += mot_fact * m_motorTargetVelocity * m_referenceSign;
					info.m_solverConstraints[nrow].m_lowerLimit = -m_maxMotorImpulse;
					info.m_solverConstraints[nrow].m_upperLimit = m_maxMotorImpulse;
				}
				if (limit != 0)
				{
					k = info.fps * currERP;
					info.m_solverConstraints[nrow].m_rhs += k * limit_err;

					if ((m_flags & (int)HingeFlags.BT_HINGE_FLAGS_CFM_STOP) != 0)
					{
						info.m_solverConstraints[nrow].m_cfm = m_stopCFM;
					}

					if (MathUtil.CompareFloat(lostop, histop))
					{
						// limited low and high simultaneously
						info.m_solverConstraints[nrow].m_lowerLimit = -MathUtil.SIMD_INFINITY;
						info.m_solverConstraints[nrow].m_upperLimit = MathUtil.SIMD_INFINITY;
					}
					else if (limit == 1)
					{ // low limit
						info.m_solverConstraints[nrow].m_lowerLimit = 0f;
						info.m_solverConstraints[nrow].m_upperLimit = MathUtil.SIMD_INFINITY;
					}
					else
					{ // high limit
						info.m_solverConstraints[nrow].m_lowerLimit = -MathUtil.SIMD_INFINITY;
						info.m_solverConstraints[nrow].m_upperLimit = 0f;
					}
					// bounce (we'll use slider parameter abs(1.0 - m_dampingLimAng) for that)
#if	_BT_USE_CENTER_LIMIT_
					float bounce = m_limit.GetRelaxationFactor();
#else
			float bounce = m_relaxationFactor;
#endif

					if (bounce > 0f)
					{
						float vel = IndexedVector3.Dot(angVelA, ax1);
						vel -= IndexedVector3.Dot(angVelB, ax1);
						// only apply bounce if the velocity is incoming, and if the
						// resulting c[] exceeds what we already have.
						if (limit == 1)
						{	// low limit
							if (vel < 0)
							{
								float newc = -bounce * vel;

								if (newc > info.m_solverConstraints[nrow].m_rhs)
								{
									info.m_solverConstraints[nrow].m_rhs = newc;
								}
							}
						}
						else
						{	// high limit - all those computations are reversed
							if (vel > 0)
							{
								float newc = -bounce * vel;
								if (newc < info.m_solverConstraints[nrow].m_rhs)
								{
									info.m_solverConstraints[nrow].m_rhs = newc;
								}
							}
						}
					}
#if	_BT_USE_CENTER_LIMIT_
					info.m_solverConstraints[nrow].m_rhs *= m_limit.GetBiasFactor();

#else
                    info.m_solverConstraints[nrow].m_rhs *= m_biasFactor;
#endif

				} // if(limit)
			} // if angular limit or powered
#if DEBUG			
			if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugConstraints)
			{
				PrintInfo2(BulletGlobals.g_streamWriter, this, info);
			}
#endif
		}

		public void SetFrames(ref IndexedMatrix frameA, ref IndexedMatrix frameB)
		{
			m_rbAFrame = frameA;
			m_rbBFrame = frameB;

		}


		public void GetInfo2InternalUsingFrameOffset(ConstraintInfo2 info, IndexedMatrix transA, IndexedMatrix transB, IndexedVector3 angVelA, IndexedVector3 angVelB)
		{
			GetInfo2InternalUsingFrameOffset(info, ref transA, ref transB, ref angVelA, ref angVelB);
		}

		public void GetInfo2InternalUsingFrameOffset(ConstraintInfo2 info, ref IndexedMatrix transA, ref IndexedMatrix transB, ref IndexedVector3 angVelA, ref IndexedVector3 angVelB)
		{
			// transforms in world space
#if DEBUG			
			if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugConstraints)
			{
				MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "rbAFrame", m_rbAFrame);
				MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "rbBFrame", m_rbBFrame);
				MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "transA", transA);
				MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "transB", transB);
			}
#endif
            IndexedMatrix trA = transA * m_rbAFrame;
            IndexedMatrix trB = transB * m_rbBFrame;

#if DEBUG
			if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugConstraints)
			{
				MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "trA", trA);
				MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "trB", trB);
			}
#endif			
			// pivot point
			IndexedVector3 pivotAInW = trA._origin;
			IndexedVector3 pivotBInW = trB._origin;
#if true
			// difference between frames in WCS
			IndexedVector3 ofs = trB._origin - trA._origin;
			// now get weight factors depending on masses
			float miA = GetRigidBodyA().GetInvMass();
			float miB = GetRigidBodyB().GetInvMass();
			bool hasStaticBody = (miA < MathUtil.SIMD_EPSILON) || (miB < MathUtil.SIMD_EPSILON);
			float miS = miA + miB;
			float factA, factB;
			if (miS > 0.0f)
			{
				factA = miB / miS;
			}
			else
			{
				factA = 0.5f;
			}
			factB = 1.0f - factA;
			// get the desired direction of hinge axis
			// as weighted sum of Z-orthos of frameA and frameB in WCS
            IndexedVector3 ax1A = trA._basis.GetColumn(2);
            IndexedVector3 ax1B = trB._basis.GetColumn(2);

			IndexedVector3 ax1 = ax1A * factA + ax1B * factB;
			ax1.Normalize();
			// fill first 3 rows 
			// we want: velA + wA x relA == velB + wB x relB
			IndexedMatrix bodyA_trans = transA;
			IndexedMatrix bodyB_trans = transB;
			int s0 = 0;
			int s1 = 1;
			int s2 = 2;
			int nrow = 2; // last filled row
			IndexedVector3 tmpA, tmpB, relA, relB, p, q;
			// get vector from bodyB to frameB in WCS
			relB = trB._origin - bodyB_trans._origin;
			// get its projection to hinge axis
			IndexedVector3 projB = ax1 * IndexedVector3.Dot(relB, ax1);
			// get vector directed from bodyB to hinge axis (and orthogonal to it)
			IndexedVector3 orthoB = relB - projB;
			// same for bodyA
			relA = trA._origin - bodyA_trans._origin;
			IndexedVector3 projA = ax1 * IndexedVector3.Dot(relA, ax1);
			IndexedVector3 orthoA = relA - projA;
			IndexedVector3 totalDist = projA - projB;
			// get offset vectors relA and relB
			relA = orthoA + totalDist * factA;
			relB = orthoB - totalDist * factB;
			// now choose average ortho to hinge axis
			p = orthoB * factA + orthoA * factB;
			float len2 = p.LengthSquared();
			if (len2 > MathUtil.SIMD_EPSILON)
			{
				p.Normalize();
			}
			else
			{
                p = trA._basis.GetColumn(1);
			}
			// make one more ortho
			q = IndexedVector3.Cross(ax1, p);
			// fill three rows
			tmpA = IndexedVector3.Cross(relA, p);
			tmpB = IndexedVector3.Cross(relB, p);


			info.m_solverConstraints[s0].m_relpos1CrossNormal = tmpA;
			info.m_solverConstraints[s0].m_relpos2CrossNormal = -tmpB;

            tmpA = IndexedVector3.Cross(ref relA, ref q);
            tmpB = IndexedVector3.Cross(ref relB, ref q);
			if (hasStaticBody && GetSolveLimit())
			{ // to make constraint between static and dynamic objects more rigid
				// remove wA (or wB) from equation if angular limit is hit
				tmpB *= factB;
				tmpA *= factA;
			}

			info.m_solverConstraints[s1].m_relpos1CrossNormal = tmpA;
			info.m_solverConstraints[s1].m_relpos2CrossNormal = -tmpB;

            tmpA = IndexedVector3.Cross(ref relA, ref ax1);
            tmpB = IndexedVector3.Cross(ref relB, ref ax1);
			if (hasStaticBody)
			{ // to make constraint between static and dynamic objects more rigid
				// remove wA (or wB) from equation
				tmpB *= factB;
				tmpA *= factA;
			}
			info.m_solverConstraints[s2].m_relpos1CrossNormal = tmpA;
			info.m_solverConstraints[s2].m_relpos2CrossNormal = -tmpB;

			float k = info.fps * info.erp;

			if (!m_angularOnly)
			{
				info.m_solverConstraints[s0].m_contactNormal = p;
				info.m_solverConstraints[s1].m_contactNormal = q;
				info.m_solverConstraints[s2].m_contactNormal = ax1;

				// compute three elements of right hand side
                float rhs = k * IndexedVector3.Dot(ref p, ref ofs);
				info.m_solverConstraints[s0].m_rhs = rhs;
                rhs = k * IndexedVector3.Dot(ref q, ref ofs);
				info.m_solverConstraints[s1].m_rhs = rhs;
                rhs = k * IndexedVector3.Dot(ref ax1, ref ofs);
				info.m_solverConstraints[s2].m_rhs = rhs;
			}

			// the hinge axis should be the only unconstrained
			// rotational axis, the angular velocity of the two bodies perpendicular to
			// the hinge axis should be equal. thus the constraint equations are
			//    p*w1 - p*w2 = 0
			//    q*w1 - q*w2 = 0
			// where p and q are unit vectors normal to the hinge axis, and w1 and w2
			// are the angular velocity vectors of the two bodies.
			int s3 = 3;
			int s4 = 4;
			info.m_solverConstraints[s3].m_relpos1CrossNormal = p;
			info.m_solverConstraints[s4].m_relpos1CrossNormal = q;

			info.m_solverConstraints[s3].m_relpos2CrossNormal = -p;
			info.m_solverConstraints[s4].m_relpos2CrossNormal = -q;

			// compute the right hand side of the constraint equation. set relative
			// body velocities along p and q to bring the hinge back into alignment.
			// if ax1A,ax1B are the unit length hinge axes as computed from bodyA and
			// bodyB, we need to rotate both bodies along the axis u = (ax1 x ax2).
			// if "theta" is the angle between ax1 and ax2, we need an angular velocity
			// along u to cover angle erp*theta in one step :
			//   |angular_velocity| = angle/time = erp*theta / stepsize
			//                      = (erp*fps) * theta
			//    angular_velocity  = |angular_velocity| * (ax1 x ax2) / |ax1 x ax2|
			//                      = (erp*fps) * theta * (ax1 x ax2) / sin(theta)
			// ...as ax1 and ax2 are unit length. if theta is smallish,
			// theta ~= sin(theta), so
			//    angular_velocity  = (erp*fps) * (ax1 x ax2)
			// ax1 x ax2 is in the plane space of ax1, so we project the angular
			// velocity to p and q to find the right hand side.
			k = info.fps * info.erp;
            IndexedVector3 u = IndexedVector3.Cross(ref ax1A, ref ax1B);
			info.m_solverConstraints[s3].m_rhs = k * IndexedVector3.Dot(u, p);
			info.m_solverConstraints[s4].m_rhs = k * IndexedVector3.Dot(u, q);
#endif
			// check angular limits
			nrow = 4; // last filled row
			int srow;
			float limit_err = 0f;
			int limit = 0;
			if (GetSolveLimit())
			{
#if	_BT_USE_CENTER_LIMIT_
				limit_err = m_limit.GetCorrection() * m_referenceSign;
#else
	limit_err = m_correction * m_referenceSign;
#endif

				limit = (limit_err > 0f) ? 1 : 2;
			}
			// if the hinge has joint limits or motor, add in the extra row
			bool powered = false;
			if (GetEnableAngularMotor())
			{
				powered = true;
			}
			if (limit != 0 || powered)
			{
				nrow++;
				srow = nrow;
				info.m_solverConstraints[srow].m_relpos1CrossNormal = ax1;
				info.m_solverConstraints[srow].m_relpos2CrossNormal = -ax1;

				float lostop = GetLowerLimit();
				float histop = GetUpperLimit();
				if (limit != 0 && (MathUtil.CompareFloat(lostop, histop)))
				{  // the joint motor is ineffective
					powered = false;
				}
				info.m_solverConstraints[srow].m_rhs = 0f;
				float currERP = ((m_flags & (int)HingeFlags.BT_HINGE_FLAGS_ERP_STOP) != 0) ? m_stopERP : info.erp;
				if (powered)
				{
					if ((m_flags & (int)HingeFlags.BT_HINGE_FLAGS_CFM_NORM) != 0)
					{
						info.m_solverConstraints[srow].m_cfm = m_normalCFM;
					}
					float mot_fact = GetMotorFactor(m_hingeAngle, lostop, histop, m_motorTargetVelocity, info.fps * currERP);
					info.m_solverConstraints[srow].m_rhs += mot_fact * m_motorTargetVelocity * m_referenceSign;
					info.m_solverConstraints[srow].m_lowerLimit = -m_maxMotorImpulse;
					info.m_solverConstraints[srow].m_upperLimit = m_maxMotorImpulse;
				}
				if (limit != 0)
				{
					k = info.fps * currERP;
					info.m_solverConstraints[srow].m_rhs += k * limit_err;
					if ((m_flags & (int)HingeFlags.BT_HINGE_FLAGS_CFM_STOP) != 0)
					{
						info.m_solverConstraints[srow].m_cfm = m_stopCFM;
					}
					if (MathUtil.CompareFloat(lostop, histop))
					{
						// limited low and high simultaneously
						info.m_solverConstraints[srow].m_lowerLimit = -MathUtil.SIMD_INFINITY;
						info.m_solverConstraints[srow].m_upperLimit = MathUtil.SIMD_INFINITY;
					}
					else if (limit == 1)
					{ // low limit
						info.m_solverConstraints[srow].m_lowerLimit = 0;
						info.m_solverConstraints[srow].m_upperLimit = MathUtil.SIMD_INFINITY;
					}
					else
					{ // high limit
						info.m_solverConstraints[srow].m_lowerLimit = -MathUtil.SIMD_INFINITY;
						info.m_solverConstraints[srow].m_upperLimit = 0;
					}
					// bounce (we'll use slider parameter abs(1.0 - m_dampingLimAng) for that)
#if	_BT_USE_CENTER_LIMIT_
					float bounce = m_limit.GetRelaxationFactor();
#else
			float bounce = m_relaxationFactor;
#endif

					if (bounce > 0f)
					{
                        float vel = IndexedVector3.Dot(ref angVelA, ref ax1);
                        vel -= IndexedVector3.Dot(ref angVelB, ref ax1);
						// only apply bounce if the velocity is incoming, and if the
						// resulting c[] exceeds what we already have.
						if (limit == 1)
						{	// low limit
							if (vel < 0)
							{
								float newc = -bounce * vel;
								if (newc > info.m_solverConstraints[srow].m_rhs)
								{
									info.m_solverConstraints[srow].m_rhs = newc;
								}
							}
						}
						else
						{	// high limit - all those computations are reversed
							if (vel > 0)
							{
								float newc = -bounce * vel;
								if (newc < info.m_solverConstraints[srow].m_rhs)
								{
									info.m_solverConstraints[srow].m_rhs = newc;
								}
							}
						}
					}
#if	_BT_USE_CENTER_LIMIT_
					info.m_solverConstraints[srow].m_rhs *= m_limit.GetBiasFactor();
#else
                    info.m_solverConstraints[srow].m_rhs *= m_biasFactor;
#endif

				} // if(limit)
			} // if angular limit or powered
#if DEBUG			
			if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugConstraints)
			{
				PrintInfo2(BulletGlobals.g_streamWriter, this, info);
			}
#endif
		}

		public void UpdateRHS(float timeStep)
		{
		}


		public void SetAngularOnly(bool angularOnly)
		{
			m_angularOnly = angularOnly;
		}

		public void EnableAngularMotor(bool enableMotor, float targetVelocity, float maxMotorImpulse)
		{
			m_enableAngularMotor = enableMotor;
			m_motorTargetVelocity = targetVelocity;
			m_maxMotorImpulse = maxMotorImpulse;
		}

		// extra motor API, including ability to set a target rotation (as opposed to angular velocity)
		// note: setMotorTarget sets angular velocity under the hood, so you must call it every tick to
		//       maintain a given angular target.
		public void EnableMotor(bool enableMotor)
		{
			m_enableAngularMotor = enableMotor;
		}

		public void SetMaxMotorImpulse(float maxMotorImpulse)
		{
			m_maxMotorImpulse = maxMotorImpulse;
		}

		public void SetMotorTarget(ref IndexedQuaternion qAinB, float dt) // qAinB is rotation of body A wrt body B.
		{
			// convert target from body to constraint space
            IndexedQuaternion qConstraint = MathUtil.QuaternionInverse(m_rbBFrame.GetRotation())* qAinB * m_rbAFrame.GetRotation();

            qConstraint.Normalize();

			// extract "pure" hinge component
			IndexedVector3 vNoHinge = MathUtil.QuatRotate(ref qConstraint, ref vHinge);
			vNoHinge.Normalize();
			IndexedQuaternion qNoHinge = MathUtil.ShortestArcQuat(ref vHinge, ref vNoHinge);
			IndexedQuaternion qHinge = MathUtil.QuaternionInverse(ref qNoHinge) * qConstraint;
			qHinge.Normalize();

			// compute angular target, clamped to limits
			float targetAngle = MathUtil.QuatAngle(ref qHinge);

			if (targetAngle > MathUtil.SIMD_PI) // long way around. flip quat and recalculate.
			{
				qHinge = -qHinge;
				targetAngle = MathUtil.QuatAngle(ref qHinge);
			}
			if (qHinge.Z < 0)
			{
				targetAngle = -targetAngle;
			}

			SetMotorTarget(targetAngle, dt);

		}

		public void SetMotorTarget(float targetAngle, float dt)
		{
#if	_BT_USE_CENTER_LIMIT_
			m_limit.Fit(ref targetAngle);
#else

            if (m_lowerLimit < m_upperLimit)
            {
                if (targetAngle < m_lowerLimit)
                {
                    targetAngle = m_lowerLimit;
                }
                else if (targetAngle > m_upperLimit)
                {
                    targetAngle = m_upperLimit;
                }
            }
#endif
			// compute angular velocity
			float curAngle = GetHingeAngle(m_rbA.GetCenterOfMassTransform(), m_rbB.GetCenterOfMassTransform());
			float dAngle = targetAngle - curAngle;
			m_motorTargetVelocity = dAngle / dt;

		}

		public void SetLimit(float low, float high)
		{
			SetLimit(low, high, .9f, .3f, 1.0f);
		}

		public void SetLimit(float low, float high, float _softness, float _biasFactor, float _relaxationFactor)
		{

#if	_BT_USE_CENTER_LIMIT_
			m_limit.Set(low, high, _softness, _biasFactor, _relaxationFactor);
#else
		    m_lowerLimit = MathUtil.NormalizeAngle(low);
            m_upperLimit = MathUtil.NormalizeAngle(high);

		    m_limitSoftness =  _softness;
		    m_biasFactor = _biasFactor;
		    m_relaxationFactor = _relaxationFactor;
#endif
		}

		public void SetAxis(ref IndexedVector3 axisInA)
		{
			IndexedVector3 rbAxisA1, rbAxisA2;
			TransformUtil.PlaneSpace1(ref axisInA, out rbAxisA1, out rbAxisA2);
			IndexedVector3 pivotInA = m_rbAFrame._origin;
			//		m_rbAFrame._origin = pivotInA;

			//MathUtil.setBasis(ref m_rbAFrame,ref axisInA,ref rbAxisA1,ref rbAxisA2);
			m_rbAFrame._basis = new IndexedBasisMatrix(rbAxisA1.X, rbAxisA2.X, axisInA.X,
                                    rbAxisA1.Y, rbAxisA2.Y, axisInA.Y,
                                    rbAxisA1.Z, rbAxisA2.Z, axisInA.Z);

            IndexedVector3 axisInB = m_rbA.GetCenterOfMassTransform()._basis * axisInA;

			IndexedQuaternion rotationArc = MathUtil.ShortestArcQuat(ref axisInA, ref axisInB);
			IndexedVector3 rbAxisB1 = MathUtil.QuatRotate(ref rotationArc, ref rbAxisA1);
            IndexedVector3 rbAxisB2 = IndexedVector3.Cross(ref axisInB, ref rbAxisB1);

            m_rbBFrame._origin = m_rbB.GetCenterOfMassTransform().Inverse() * (m_rbA.GetCenterOfMassTransform() * (pivotInA));

            m_rbBFrame._basis = new IndexedBasisMatrix(rbAxisB1.X, rbAxisB2.X, axisInB.X,
                                    rbAxisB1.Y, rbAxisB2.Y, axisInB.Y,
                                    rbAxisB1.Z, rbAxisB2.Z, axisInB.Z);
		}

		public float GetLowerLimit()
		{
#if	_BT_USE_CENTER_LIMIT_
			return m_limit.GetLow();
#else
	return m_lowerLimit;
#endif
		}

		public float GetUpperLimit()
		{
#if	_BT_USE_CENTER_LIMIT_
			return m_limit.GetHigh();
#else		
	return m_upperLimit;
#endif
		}


		public float GetHingeAngle()
		{
			return GetHingeAngle(m_rbA.GetCenterOfMassTransform(), m_rbB.GetCenterOfMassTransform());
		}

		public float GetHingeAngle(IndexedMatrix transA, IndexedMatrix transB)
		{
			return GetHingeAngle(ref transA, ref transB);
		}

		public float GetHingeAngle(ref IndexedMatrix transA, ref IndexedMatrix transB)
		{
            IndexedVector3 refAxis0 = transA._basis * m_rbAFrame._basis.GetColumn(0);
            IndexedVector3 refAxis1 = transA._basis * m_rbAFrame._basis.GetColumn(1);
            IndexedVector3 swingAxis = transB._basis * m_rbBFrame._basis.GetColumn(1);

            refAxis0.Normalize();
            refAxis1.Normalize();
            swingAxis.Normalize();

            float a = IndexedVector3.Dot(ref swingAxis, ref refAxis0);
            float b = IndexedVector3.Dot(ref swingAxis, ref refAxis1);

            float angle = (float)Math.Atan2(a,b );

            float result = m_referenceSign * angle;
#if DEBUG            
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugConstraints)
            {
                BulletGlobals.g_streamWriter.WriteLine("GetHingeAngle [{0:0.00000000}][{1:0.00000000}][{2:0.00000000}][{3:0.00000000}]", a,b,m_referenceSign,result);
            }
#endif            
            return result;
		}

		public void TestLimit(IndexedMatrix transA, IndexedMatrix transB)
		{
			TestLimit(ref transA, ref transB);
		}

		public void TestLimit(ref IndexedMatrix transA, ref IndexedMatrix transB)
		{
			// Compute limit information
			m_hingeAngle = GetHingeAngle(ref transA, ref transB);
#if	_BT_USE_CENTER_LIMIT_
			m_limit.Test(m_hingeAngle);
#else

	        m_correction = 0f;
	        m_limitSign = 0f;
	        m_solveLimit = false;
	        if (m_lowerLimit <= m_upperLimit)
	        {
                m_hingeAngle = AdjustAngleToLimits(m_hingeAngle, m_lowerLimit, m_upperLimit);
		        if (m_hingeAngle <= m_lowerLimit)
		        {
			        m_correction = (m_lowerLimit - m_hingeAngle);
			        m_limitSign = 1.0f;
			        m_solveLimit = true;
		        } 
		        else if (m_hingeAngle >= m_upperLimit)
		        {
			        m_correction = m_upperLimit - m_hingeAngle;
			        m_limitSign = -1.0f;
			        m_solveLimit = true;
		        }
	        }
#endif
			return;
		}


		public IndexedMatrix GetAFrame() { return m_rbAFrame; }
		public IndexedMatrix GetBFrame() { return m_rbBFrame; }

		public bool GetSolveLimit()
		{
#if	_BT_USE_CENTER_LIMIT_
			return m_limit.IsLimit();
#else
	return m_solveLimit;
#endif
		}

		public float GetLimitSign()
		{
#if	_BT_USE_CENTER_LIMIT_
			return m_limit.GetSign();
#else
		return m_limitSign;
#endif
		}

		public bool GetAngularOnly()
		{
			return m_angularOnly;
		}

		public bool GetEnableAngularMotor()
		{
			return m_enableAngularMotor;
		}

		public float GetMotorTargetVelocity()
		{
			return m_motorTargetVelocity;
		}

		public float GetMaxMotorImpulse()
		{
			return m_maxMotorImpulse;
		}

		// access for UseFrameOffset
		public bool GetUseFrameOffset()
		{
			return m_useOffsetForConstraintFrame;
		}

		public void SetUseFrameOffset(bool frameOffsetOnOff)
		{
			m_useOffsetForConstraintFrame = frameOffsetOnOff;
		}


	}

	public enum HingeFlags
	{
		BT_HINGE_FLAGS_CFM_STOP = 1,
		BT_HINGE_FLAGS_ERP_STOP = 2,
		BT_HINGE_FLAGS_CFM_NORM = 4
	};

}
