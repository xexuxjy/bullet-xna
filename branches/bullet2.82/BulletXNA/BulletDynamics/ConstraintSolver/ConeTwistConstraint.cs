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
using System.Diagnostics;

using BulletXNA.LinearMath;

namespace BulletXNA.BulletDynamics
{
	public class ConeTwistConstraint : TypedConstraint
	{
		public const float CONETWIST_DEF_FIX_THRESH = .05f;
		public static IndexedVector3 vTwist = new IndexedVector3(1, 0, 0); // twist axis in constraint's space

		public JacobianEntry[] m_jac = new JacobianEntry[3]; //3 orthogonal linear constraints

		public IndexedMatrix m_rbAFrame = IndexedMatrix.Identity;
		public IndexedMatrix m_rbBFrame = IndexedMatrix.Identity;

		public float m_limitSoftness;
		public float m_biasFactor;
		public float m_relaxationFactor;

		public float m_damping;

		public float m_swingSpan1;
		public float m_swingSpan2;
		public float m_twistSpan;

		public float m_fixThresh;

		public IndexedVector3 m_swingAxis;
		public IndexedVector3 m_twistAxis;

		public float m_kSwing;
		public float m_kTwist;

		public float m_twistLimitSign;
		public float m_swingCorrection;
		public float m_twistCorrection;

		public float m_twistAngle;

		public float m_accSwingLimitImpulse;
		public float m_accTwistLimitImpulse;

		public bool m_angularOnly;
		public bool m_solveTwistLimit;
		public bool m_solveSwingLimit;

		// not yet used...
		public float m_swingLimitRatio;
		public float m_twistLimitRatio;
		public IndexedVector3 m_twistAxisA;

		// motor
		public bool m_bMotorEnabled;
		public bool m_bNormalizedMotorStrength;
		public IndexedQuaternion m_qTarget = IndexedQuaternion.Identity;
		public float m_maxMotorImpulse;
		public IndexedVector3 m_accMotorImpulse;

		// parameters
		public int m_flags;
		public float m_linCFM;
		public float m_linERP;
		public float m_angCFM;

        private bool m_useSolveConstraintObsolete = false;
        static bool bDoTorque = true;



		public ConeTwistConstraint(RigidBody rbA, RigidBody rbB, ref IndexedMatrix rbAFrame, ref IndexedMatrix rbBFrame) :
			base(TypedConstraintType.CONETWIST_CONSTRAINT_TYPE, rbA, rbB)
		{
			m_angularOnly = false;
			m_rbAFrame = rbAFrame;
			m_rbBFrame = rbBFrame;
			Init();
		}

		public ConeTwistConstraint(RigidBody rbA, ref IndexedMatrix rbAFrame)
			: base(TypedConstraintType.CONETWIST_CONSTRAINT_TYPE, rbA)
		{
			m_rbAFrame = rbAFrame;
			m_rbBFrame = rbAFrame;
			m_angularOnly = false;
			Init();
		}

		protected void Init()
		{
			m_angularOnly = false;
			m_solveTwistLimit = false;
			m_solveSwingLimit = false;
			m_bMotorEnabled = false;
			m_maxMotorImpulse = -1f;

			SetLimit(MathUtil.BT_LARGE_FLOAT, MathUtil.BT_LARGE_FLOAT, MathUtil.BT_LARGE_FLOAT);
			m_damping = 0.01f;
			m_fixThresh = CONETWIST_DEF_FIX_THRESH;
			m_flags = 0;
			m_linCFM = 0f;
			m_linERP = 0.7f;
			m_angCFM = 0.0f;
		}



        public static float ComputeAngularImpulseDenominator(ref IndexedVector3 axis, ref IndexedBasisMatrix invInertiaWorld)
		{
            //IndexedVector3 vec = MathUtil.TransposeTransformNormal(axis, invInertiaWorld);
            IndexedVector3 vec = axis* invInertiaWorld;
			return axis.Dot(ref vec);
		}


		public void GetInfo1NonVirtual(ConstraintInfo1 info)
		{
            //always reserve 6 rows: object transform is not available on SPU
            info.m_numConstraintRows = 6;
            info.nub = 0;
		}

		public override void GetInfo1(ConstraintInfo1 info)
		{
            if (m_useSolveConstraintObsolete)
            {
                info.m_numConstraintRows = 0;
                info.nub = 0;
            }
            else
            {

                info.m_numConstraintRows = 3;
                info.nub = 3;

                CalcAngleInfo2(m_rbA.GetCenterOfMassTransform(), m_rbB.GetCenterOfMassTransform(), m_rbA.GetInvInertiaTensorWorld(), m_rbB.GetInvInertiaTensorWorld());
                if (m_solveSwingLimit)
                {
                    info.m_numConstraintRows++;
                    info.nub--;
                    if ((m_swingSpan1 < m_fixThresh) && (m_swingSpan2 < m_fixThresh))
                    {
                        info.m_numConstraintRows++;
                        info.nub--;
                    }
                }
                if (m_solveTwistLimit)
                {
                    info.m_numConstraintRows++;
                    info.nub--;
                }
#if DEBUG                
                if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugConstraints)
                {
                    PrintInfo1(BulletGlobals.g_streamWriter, this, info);
                }
#endif                
            }
        }

		public override void GetInfo2(ConstraintInfo2 info)
		{
			GetInfo2NonVirtual(info, m_rbA.GetCenterOfMassTransform(),
								m_rbB.GetCenterOfMassTransform(),
								m_rbA.GetInvInertiaTensorWorld(),
								m_rbB.GetInvInertiaTensorWorld());
		}

        public void GetInfo2NonVirtual(ConstraintInfo2 info, IndexedMatrix transA, IndexedMatrix transB, IndexedBasisMatrix invInertiaWorldA, IndexedBasisMatrix invInertiaWorldB)
		{
			CalcAngleInfo2(ref transA, ref transB, ref invInertiaWorldA, ref invInertiaWorldB);

            Debug.Assert(!m_useSolveConstraintObsolete);


			// set jacobian
			info.m_solverConstraints[0].m_contactNormal.X = 1f;
			info.m_solverConstraints[1].m_contactNormal.Y = 1f;
			info.m_solverConstraints[2].m_contactNormal.Z = 1f;

			IndexedVector3 a1 = transA._basis * m_rbAFrame._origin;
			{
				IndexedVector3 a1neg = -a1;
				MathUtil.GetSkewSymmetricMatrix(ref a1neg,
					out info.m_solverConstraints[0].m_relpos1CrossNormal,
					out info.m_solverConstraints[1].m_relpos1CrossNormal,
					out info.m_solverConstraints[2].m_relpos1CrossNormal);
			}

            IndexedVector3 a2 = transB._basis * m_rbBFrame._origin;
			{
				MathUtil.GetSkewSymmetricMatrix(ref a2,
					out info.m_solverConstraints[0].m_relpos2CrossNormal,
					out info.m_solverConstraints[1].m_relpos2CrossNormal,
					out info.m_solverConstraints[2].m_relpos2CrossNormal);
			}

			// set right hand side
			float linERP = ((m_flags & (int)ConeTwistFlags.BT_CONETWIST_FLAGS_LIN_ERP) != 0) ? m_linERP : info.erp;
			float k = info.fps * linERP;

			for (int j = 0; j < 3; j++)
			{
				info.m_solverConstraints[j].m_rhs = k * (a2[j] + transB._origin[j] - a1[j] - transA._origin[j]);
				info.m_solverConstraints[j].m_lowerLimit = -MathUtil.SIMD_INFINITY;
				info.m_solverConstraints[j].m_upperLimit = MathUtil.SIMD_INFINITY;
				if ((m_flags & (int)ConeTwistFlags.BT_CONETWIST_FLAGS_LIN_CFM) != 0)
				{
					info.m_solverConstraints[j].m_cfm = m_linCFM;
				}
			}
			int row = 3;

			IndexedVector3 ax1;
			// angular limits
			if (m_solveSwingLimit)
			{
				if ((m_swingSpan1 < m_fixThresh) && (m_swingSpan2 < m_fixThresh))
				{
					IndexedMatrix trA = transA *  m_rbAFrame;

					IndexedVector3 p = trA._basis.GetColumn(1);
					IndexedVector3 q = trA._basis.GetColumn(2);
					info.m_solverConstraints[row].m_relpos1CrossNormal = p;
					info.m_solverConstraints[row + 1].m_relpos1CrossNormal = q;
					info.m_solverConstraints[row].m_relpos2CrossNormal = -p;
					info.m_solverConstraints[row + 1].m_relpos2CrossNormal = -q;

					float fact = info.fps * m_relaxationFactor;
					info.m_solverConstraints[row].m_rhs = fact * m_swingAxis.Dot(ref p);
					info.m_solverConstraints[row + 1].m_rhs = fact * m_swingAxis.Dot(ref q);
					info.m_solverConstraints[row].m_lowerLimit = -MathUtil.SIMD_INFINITY;
					info.m_solverConstraints[row].m_upperLimit = MathUtil.SIMD_INFINITY;
					info.m_solverConstraints[row + 1].m_lowerLimit = -MathUtil.SIMD_INFINITY;
					info.m_solverConstraints[row + 1].m_upperLimit = MathUtil.SIMD_INFINITY;
					row += 2;
				}
				else
				{
					ax1 = m_swingAxis * m_relaxationFactor * m_relaxationFactor;
					info.m_solverConstraints[row].m_relpos1CrossNormal = ax1;
					info.m_solverConstraints[row].m_relpos2CrossNormal = -ax1;

					float k1 = info.fps * m_biasFactor;

					info.m_solverConstraints[row].m_rhs = k1 * m_swingCorrection;
					if ((m_flags & (int)ConeTwistFlags.BT_CONETWIST_FLAGS_ANG_CFM) != 0)
					{
						info.m_solverConstraints[row].m_cfm = m_angCFM;
					}
					// m_swingCorrection is always positive or 0
					info.m_solverConstraints[row].m_lowerLimit = 0;
					info.m_solverConstraints[row].m_upperLimit = MathUtil.SIMD_INFINITY;
					++row;
				}
			}
			if (m_solveTwistLimit)
			{
				ax1 = m_twistAxis * m_relaxationFactor * m_relaxationFactor;
				info.m_solverConstraints[row].m_relpos1CrossNormal = ax1;
				info.m_solverConstraints[row].m_relpos2CrossNormal = -ax1;
				float k1 = info.fps * m_biasFactor;
				info.m_solverConstraints[row].m_rhs = k1 * m_twistCorrection;
				if ((m_flags & (int)ConeTwistFlags.BT_CONETWIST_FLAGS_ANG_CFM) != 0)
				{
					info.m_solverConstraints[row].m_cfm = m_angCFM;
				}

				if (m_twistSpan > 0.0f)
				{
					if (m_twistCorrection > 0.0f)
					{
						info.m_solverConstraints[row].m_lowerLimit = 0;
						info.m_solverConstraints[row].m_upperLimit = MathUtil.SIMD_INFINITY;
					}
					else
					{
						info.m_solverConstraints[row].m_lowerLimit = -MathUtil.SIMD_INFINITY;
						info.m_solverConstraints[row].m_upperLimit = 0;
					}
				}
				else
				{
					info.m_solverConstraints[row].m_lowerLimit = -MathUtil.SIMD_INFINITY;
					info.m_solverConstraints[row].m_upperLimit = MathUtil.SIMD_INFINITY;
				}
				++row;
			}
#if DEBUG
			if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugConstraints)
			{
                //PrintInfo2(BulletGlobals.g_streamWriter, this, info);
			}
#endif			

		}

		public void UpdateRHS(float timeStep)
		{
		}

		void SetAngularOnly(bool angularOnly)
		{
			m_angularOnly = angularOnly;
		}

		void SetLimit(int limitIndex, float limitValue)
		{
			switch (limitIndex)
			{
				case 3:
					{
						m_twistSpan = limitValue;
						break;
					}
				case 4:
					{
						m_swingSpan2 = limitValue;
						break;
					}
				case 5:
					{
						m_swingSpan1 = limitValue;
						break;
					}
				default:
					{
						break;
					}
			}
		}
		public void SetLimit(float _swingSpan1, float _swingSpan2, float _twistSpan)
		{
			SetLimit(_swingSpan1, _swingSpan2, _twistSpan, 1f, .3f, 1f);
		}

        public void SetLimit(float _swingSpan1, float _swingSpan2, float _twistSpan, float _softness)
        {
            SetLimit(_swingSpan1, _swingSpan2, _twistSpan, _softness, .3f, 1f);
        }

		public void SetLimit(float _swingSpan1, float _swingSpan2, float _twistSpan, float _softness, float _biasFactor, float _relaxationFactor)
		{
			m_swingSpan1 = _swingSpan1;
			m_swingSpan2 = _swingSpan2;
			m_twistSpan = _twistSpan;

			m_limitSoftness = _softness;
			m_biasFactor = _biasFactor;
			m_relaxationFactor = _relaxationFactor;
		}

		public IndexedMatrix GetAFrame() { return m_rbAFrame; }
		public IndexedMatrix GetBFrame() { return m_rbBFrame; }

		public bool GetSolveTwistLimit()
		{
			return m_solveTwistLimit;
		}

		public bool GetSolveSwingLimit()
		{
			return m_solveTwistLimit;
		}

		public float GetTwistLimitSign()
		{
			return m_twistLimitSign;
		}

		public void CalcAngleInfo()
		{
			m_swingCorrection = 0f;
			m_twistLimitSign = 0f;
			m_solveTwistLimit = false;
			m_solveSwingLimit = false;

			IndexedVector3 b1Axis1 = IndexedVector3.Zero, b1Axis2 = IndexedVector3.Zero, b1Axis3 = IndexedVector3.Zero;
			IndexedVector3 b2Axis1 = IndexedVector3.Zero, b2Axis2 = IndexedVector3.Zero;

			IndexedMatrix transA = GetRigidBodyA().GetCenterOfMassTransform();
			IndexedMatrix transB = GetRigidBodyB().GetCenterOfMassTransform();

            b1Axis1 = GetRigidBodyA().GetCenterOfMassTransform()._basis * m_rbAFrame._basis.GetColumn(0);
            b2Axis1 = GetRigidBodyB().GetCenterOfMassTransform()._basis * m_rbBFrame._basis.GetColumn(0);

			float swing1 = 0f, swing2 = 0f;

			float swx = 0f, swy = 0f;
			float thresh = 10f;
			float fact;

			// Get Frame into world space
			if (m_swingSpan1 >= 0.05f)
			{
                b1Axis2 = GetRigidBodyA().GetCenterOfMassTransform()._basis * m_rbAFrame._basis.GetColumn(1);
				swx = b2Axis1.Dot(ref b1Axis1);
				swy = b2Axis1.Dot(ref b1Axis2);
				swing1 = (float)Math.Atan2(swy, swx);
				fact = (swy * swy + swx * swx) * thresh * thresh;
				fact = fact / (fact + 1f);
				swing1 *= fact;
			}

			if (m_swingSpan2 >= 0.05f)
			{
                b1Axis3 = GetRigidBodyA().GetCenterOfMassTransform()._basis * m_rbAFrame._basis.GetColumn(2);
				swx = b2Axis1.Dot(ref b1Axis1);
				swy = b2Axis1.Dot(ref b1Axis3);
				swing2 = (float)Math.Atan2(swy, swx);
				fact = (swy * swy + swx * swx) * thresh * thresh;
				fact = fact / (fact + 1f);
				swing2 *= fact;
			}

			float RMaxAngle1Sq = 1.0f / (m_swingSpan1 * m_swingSpan1);
			float RMaxAngle2Sq = 1.0f / (m_swingSpan2 * m_swingSpan2);
			float EllipseAngle = Math.Abs(swing1 * swing1) * RMaxAngle1Sq + Math.Abs(swing2 * swing2) * RMaxAngle2Sq;

			if (EllipseAngle > 1.0f)
			{
				m_swingCorrection = EllipseAngle - 1.0f;
				m_solveSwingLimit = true;
				// Calculate necessary axis & factors
				m_swingAxis = b2Axis1.Cross(b1Axis2 * b2Axis1.Dot(ref b1Axis2) + b1Axis3 * b2Axis1.Dot(ref b1Axis3));
				m_swingAxis.Normalize();
				float swingAxisSign = (b2Axis1.Dot(ref b1Axis1) >= 0.0f) ? 1.0f : -1.0f;
				m_swingAxis *= swingAxisSign;
			}

			// Twist limits
			if (m_twistSpan >= 0f)
			{
                IndexedVector3 b2Axis2a = GetRigidBodyB().GetCenterOfMassTransform()._basis * m_rbBFrame._basis.GetColumn(1); 
				IndexedQuaternion rotationArc = MathUtil.ShortestArcQuat(ref b2Axis1, ref b1Axis1);
				IndexedVector3 TwistRef = MathUtil.QuatRotate(ref rotationArc, ref b2Axis2a);
				float twist = (float)Math.Atan2(TwistRef.Dot(ref b1Axis3), TwistRef.Dot(ref b1Axis2));
				m_twistAngle = twist;

				//		float lockedFreeFactor = (m_twistSpan > float(0.05f)) ? m_limitSoftness : float(0.);
				float lockedFreeFactor = (m_twistSpan > 0.05f) ? 1.0f : 0f;
				if (twist <= -m_twistSpan * lockedFreeFactor)
				{
					m_twistCorrection = -(twist + m_twistSpan);
					m_solveTwistLimit = true;
					m_twistAxis = (b2Axis1 + b1Axis1) * 0.5f;
					m_twistAxis.Normalize();
					m_twistAxis *= -1.0f;
				}
				else if (twist > m_twistSpan * lockedFreeFactor)
				{
					m_twistCorrection = (twist - m_twistSpan);
					m_solveTwistLimit = true;
					m_twistAxis = (b2Axis1 + b1Axis1) * 0.5f;
					m_twistAxis.Normalize();
				}
			}

		}

        public void CalcAngleInfo2(IndexedMatrix transA, IndexedMatrix transB, IndexedBasisMatrix invInertiaWorldA, IndexedBasisMatrix invInertiaWorldB)
		{
			CalcAngleInfo2(ref transA, ref transB, ref invInertiaWorldA, ref invInertiaWorldB);
		}

        public void CalcAngleInfo2(ref IndexedMatrix transA, ref IndexedMatrix transB, ref IndexedBasisMatrix invInertiaWorldA, ref IndexedBasisMatrix invInertiaWorldB)
		{
			m_swingCorrection = 0;
			m_twistLimitSign = 0;
			m_solveTwistLimit = false;
			m_solveSwingLimit = false;

			// compute rotation of A wrt B (in constraint space)
            if (m_bMotorEnabled && (!m_useSolveConstraintObsolete))
			{	// it is assumed that setMotorTarget() was alredy called 
				// and motor target m_qTarget is within constraint limits
				// TODO : split rotation to pure swing and pure twist
				// compute desired transforms in world
				IndexedMatrix trPose = IndexedMatrix.CreateFromQuaternion(m_qTarget);
				IndexedMatrix trA = transA * m_rbAFrame;
				IndexedMatrix trB = transB * m_rbBFrame;
                IndexedMatrix trDeltaAB = trB * trPose * trA.Inverse();
				IndexedQuaternion qDeltaAB = trDeltaAB.GetRotation();
				IndexedVector3 swingAxis = new IndexedVector3(qDeltaAB.X, qDeltaAB.Y, qDeltaAB.Z);
                float swingAxisLen2 = swingAxis.LengthSquared();
                if (MathUtil.FuzzyZero(swingAxisLen2))
                {
                    return;
                }

				m_swingAxis = swingAxis;
				m_swingAxis.Normalize();
				m_swingCorrection = MathUtil.QuatAngle(ref qDeltaAB);
				if (!MathUtil.FuzzyZero(m_swingCorrection))
				{
					m_solveSwingLimit = true;
				}
				return;
			}


			{

				// compute rotation of A wrt B (in constraint space)
				// Not sure if these need order swapping as well?
                IndexedQuaternion qA = transA.GetRotation() * m_rbAFrame.GetRotation();
                IndexedQuaternion qB = transB.GetRotation() * m_rbBFrame.GetRotation();
                
                IndexedQuaternion qAB = MathUtil.QuaternionInverse(qB) * qA;

				// split rotation into cone and twist
				// (all this is done from B's perspective. Maybe I should be averaging axes...)
				IndexedVector3 vConeNoTwist = MathUtil.QuatRotate(ref qAB, ref vTwist);
				vConeNoTwist.Normalize();
				IndexedQuaternion qABCone = MathUtil.ShortestArcQuat(ref vTwist, ref vConeNoTwist);
				qABCone.Normalize();
				IndexedQuaternion qABTwist = MathUtil.QuaternionInverse(qABCone) * qAB;
				qABTwist.Normalize();

				if (m_swingSpan1 >= m_fixThresh && m_swingSpan2 >= m_fixThresh)
				{
					float swingAngle = 0f, swingLimit = 0f;
					IndexedVector3 swingAxis = IndexedVector3.Zero;
					ComputeConeLimitInfo(ref qABCone, ref swingAngle, ref swingAxis, ref swingLimit);

					if (swingAngle > swingLimit * m_limitSoftness)
					{
						m_solveSwingLimit = true;

						// compute limit ratio: 0->1, where
						// 0 == beginning of soft limit
						// 1 == hard/real limit
						m_swingLimitRatio = 1f;
						if (swingAngle < swingLimit && m_limitSoftness < 1f - MathUtil.SIMD_EPSILON)
						{
							m_swingLimitRatio = (swingAngle - swingLimit * m_limitSoftness) /
												(swingLimit - (swingLimit * m_limitSoftness));
						}

						// swing correction tries to get back to soft limit
						m_swingCorrection = swingAngle - (swingLimit * m_limitSoftness);

						// adjustment of swing axis (based on ellipse normal)
						AdjustSwingAxisToUseEllipseNormal(ref swingAxis);

						// Calculate necessary axis & factors		
						m_swingAxis = MathUtil.QuatRotate(qB, -swingAxis);

						m_twistAxisA = IndexedVector3.Zero;

						m_kSwing = 1f /
							(ComputeAngularImpulseDenominator(ref m_swingAxis, ref invInertiaWorldA) +
							 ComputeAngularImpulseDenominator(ref m_swingAxis, ref invInertiaWorldB));
					}
				}
				else
				{
					// you haven't set any limits;
					// or you're trying to set at least one of the swing limits too small. (if so, do you really want a conetwist constraint?)
					// anyway, we have either hinge or fixed joint

                    IndexedVector3 ivA = transA._basis * m_rbAFrame._basis.GetColumn(0);
                    IndexedVector3 jvA = transA._basis * m_rbAFrame._basis.GetColumn(1);
                    IndexedVector3 kvA = transA._basis * m_rbAFrame._basis.GetColumn(2);
                    IndexedVector3 ivB = transB._basis * m_rbBFrame._basis.GetColumn(0);
                    
                    IndexedVector3 target = IndexedVector3.Zero;
					float x = ivB.Dot(ref ivA);
					float y = ivB.Dot(ref jvA);
					float z = ivB.Dot(ref kvA);
					if ((m_swingSpan1 < m_fixThresh) && (m_swingSpan2 < m_fixThresh))
					{ // fixed. We'll need to add one more row to constraint
						if ((!MathUtil.FuzzyZero(y)) || (!(MathUtil.FuzzyZero(z))))
						{
							m_solveSwingLimit = true;
							m_swingAxis = -ivB.Cross(ref ivA);
						}
					}
					else
					{
						if (m_swingSpan1 < m_fixThresh)
						{ // hinge around Y axis
							if (!(MathUtil.FuzzyZero(y)))
							{
								m_solveSwingLimit = true;
								if (m_swingSpan2 >= m_fixThresh)
								{
									y = 0;
									float span2 = (float)Math.Atan2(z, x);
									if (span2 > m_swingSpan2)
									{
										x = (float)Math.Cos(m_swingSpan2);
										z = (float)Math.Sin(m_swingSpan2);
									}
									else if (span2 < -m_swingSpan2)
									{
										x = (float)Math.Cos(m_swingSpan2);
										z = -(float)Math.Sin(m_swingSpan2);
									}
								}
							}
						}
						else
						{ // hinge around Z axis
							if (!MathUtil.FuzzyZero(z))
							{
								m_solveSwingLimit = true;
								if (m_swingSpan1 >= m_fixThresh)
								{
									z = 0f;
									float span1 = (float)Math.Atan2(y, x);
									if (span1 > m_swingSpan1)
									{
										x = (float)Math.Cos(m_swingSpan1);
										y = (float)Math.Sin(m_swingSpan1);
									}
									else if (span1 < -m_swingSpan1)
									{
										x = (float)Math.Cos(m_swingSpan1);
										y = -(float)Math.Sin(m_swingSpan1);
									}
								}
							}
						}
						target.X = x * ivA.X + y * jvA.X + z * kvA.X;
						target.Y = x * ivA.Y + y * jvA.Y + z * kvA.Y;
						target.Z = x * ivA.Z + y * jvA.Z + z * kvA.Z;
						target.Normalize();
						m_swingAxis = -(ivB.Cross(ref target));
						m_swingCorrection = m_swingAxis.Length();
						m_swingAxis.Normalize();
					}
				}

				if (m_twistSpan >= 0f)
				{
					IndexedVector3 twistAxis;
					ComputeTwistLimitInfo(ref qABTwist, out m_twistAngle, out twistAxis);

					if (m_twistAngle > m_twistSpan * m_limitSoftness)
					{
						m_solveTwistLimit = true;

						m_twistLimitRatio = 1f;
						if (m_twistAngle < m_twistSpan && m_limitSoftness < 1f - MathUtil.SIMD_EPSILON)
						{
							m_twistLimitRatio = (m_twistAngle - m_twistSpan * m_limitSoftness) /
												(m_twistSpan - m_twistSpan * m_limitSoftness);
						}

						// twist correction tries to get back to soft limit
						m_twistCorrection = m_twistAngle - (m_twistSpan * m_limitSoftness);

						m_twistAxis = MathUtil.QuatRotate(qB, -twistAxis);

						m_kTwist = 1f /
							(ComputeAngularImpulseDenominator(ref m_twistAxis, ref invInertiaWorldA) +
							 ComputeAngularImpulseDenominator(ref m_twistAxis, ref invInertiaWorldB));
					}

					if (m_solveSwingLimit)
					{
						m_twistAxisA = MathUtil.QuatRotate(qA, -twistAxis);
					}
				}
				else
				{
					m_twistAngle = 0f;
				}
			}
		}

		public float GetSwingSpan1()
		{
			return m_swingSpan1;
		}
		public float GetSwingSpan2()
		{
			return m_swingSpan2;
		}
		public float GetTwistSpan()
		{
			return m_twistSpan;
		}
		public float GetTwistAngle()
		{
			return m_twistAngle;
		}
		public bool IsPastSwingLimit()
		{
			return m_solveSwingLimit;
		}

		public void SetDamping(float damping)
		{
			m_damping = damping;
		}

		public void EnableMotor(bool b)
		{
			m_bMotorEnabled = b;
		}
		public void SetMaxMotorImpulse(float maxMotorImpulse)
		{
			m_maxMotorImpulse = maxMotorImpulse;
			m_bNormalizedMotorStrength = false;
		}
		public void SetMaxMotorImpulseNormalized(float maxMotorImpulse)
		{
			m_maxMotorImpulse = maxMotorImpulse;
			m_bNormalizedMotorStrength = true;
		}

		public float GetFixThresh()
		{
			return m_fixThresh;
		}

		public void SetFixThresh(float fixThresh)
		{
			m_fixThresh = fixThresh;
		}

		// setMotorTarget:
		// q: the desired rotation of bodyA wrt bodyB.
		// note: if q violates the joint limits, the internal target is clamped to avoid conflicting impulses (very bad for stability)
		// note: don't forget to enableMotor()
		public void SetMotorTarget(ref IndexedQuaternion q)
		{
			IndexedMatrix trACur = m_rbA.GetCenterOfMassTransform();
			IndexedMatrix trBCur = m_rbB.GetCenterOfMassTransform();
			IndexedMatrix trABCur = trBCur.Inverse() * trACur;
			IndexedQuaternion qABCur = trABCur.GetRotation();
            IndexedMatrix trConstraintCur = (trBCur * m_rbBFrame).Inverse() * (trACur * m_rbAFrame);
                
			IndexedQuaternion qConstraintCur = trConstraintCur.GetRotation();

            IndexedQuaternion qConstraint = MathUtil.QuaternionInverse(m_rbBFrame.GetRotation()) * q * m_rbAFrame.GetRotation();
			SetMotorTargetInConstraintSpace(ref qConstraint);
		}

		// same as above, but q is the desired rotation of frameA wrt frameB in constraint space
		public void SetMotorTargetInConstraintSpace(ref IndexedQuaternion q)
		{
			m_qTarget = q;

			// clamp motor target to within limits
			{
				float softness = 1f;//m_limitSoftness;

				// split into twist and cone
				IndexedVector3 vTwisted = MathUtil.QuatRotate(ref m_qTarget, ref vTwist);
				IndexedQuaternion qTargetCone = MathUtil.ShortestArcQuat(ref vTwist, ref vTwisted);
				qTargetCone.Normalize();
				IndexedQuaternion qTargetTwist = MathUtil.QuaternionMultiply(MathUtil.QuaternionInverse(qTargetCone), m_qTarget);
				qTargetTwist.Normalize();

				// clamp cone
				if (m_swingSpan1 >= 0.05f && m_swingSpan2 >= 0.05f)
				{
					float swingAngle = 0f, swingLimit = 0f; IndexedVector3 swingAxis = IndexedVector3.Zero;
					ComputeConeLimitInfo(ref qTargetCone, ref swingAngle, ref swingAxis, ref swingLimit);

					if (Math.Abs(swingAngle) > MathUtil.SIMD_EPSILON)
					{
						if (swingAngle > swingLimit * softness)
						{
							swingAngle = swingLimit * softness;
						}
						else if (swingAngle < -swingLimit * softness)
						{
							swingAngle = -swingLimit * softness;
						}
						qTargetCone = new IndexedQuaternion(swingAxis, swingAngle);
					}
				}

				// clamp twist
				if (m_twistSpan >= 0.05f)
				{
					float twistAngle; IndexedVector3 twistAxis;
					ComputeTwistLimitInfo(ref qTargetTwist, out twistAngle, out twistAxis);

					if (Math.Abs(twistAngle) > MathUtil.SIMD_EPSILON)
					{
						// eddy todo: limitSoftness used here???
						if (twistAngle > m_twistSpan * softness)
						{
							twistAngle = m_twistSpan * softness;
						}
						else if (twistAngle < -m_twistSpan * softness)
						{
							twistAngle = -m_twistSpan * softness;
						}
						qTargetTwist = new IndexedQuaternion(twistAxis, twistAngle);
					}
				}

				m_qTarget = qTargetCone * qTargetTwist;
			}


		}

		public IndexedVector3 GetPointForAngle(float fAngleInRadians, float fLength)
		{
			// compute x/y in ellipse using cone angle (0 -> 2*PI along surface of cone)
			float xEllipse = (float)Math.Cos(fAngleInRadians);
			float yEllipse = (float)Math.Sin(fAngleInRadians);

			// Use the slope of the vector (using x/yEllipse) and find the length
			// of the line that intersects the ellipse:
			//  x^2   y^2
			//  --- + --- = 1, where a and b are semi-major axes 2 and 1 respectively (ie. the limits)
			//  a^2   b^2
			// Do the math and it should be clear.

			float swingLimit = m_swingSpan1; // if xEllipse == 0, just use axis b (1)
			if (Math.Abs(xEllipse) > MathUtil.SIMD_EPSILON)
			{
				float surfaceSlope2 = (yEllipse * yEllipse) / (xEllipse * xEllipse);
				float norm = 1 / (m_swingSpan2 * m_swingSpan2);
				norm += surfaceSlope2 / (m_swingSpan1 * m_swingSpan1);
				float swingLimit2 = (1 + surfaceSlope2) / norm;
				swingLimit = (float)Math.Sqrt(swingLimit2);
			}

			// convert into point in constraint space:
			// note: twist is x-axis, swing 1 and 2 are along the z and y axes respectively
			IndexedVector3 vSwingAxis = new IndexedVector3(0, xEllipse, -yEllipse);
			IndexedQuaternion qSwing = new IndexedQuaternion(vSwingAxis, swingLimit);
			IndexedVector3 vPointInConstraintSpace = new IndexedVector3(fLength, 0, 0);
			return MathUtil.QuatRotate(ref qSwing, ref vPointInConstraintSpace);
		}

		protected void ComputeConeLimitInfo(ref IndexedQuaternion qCone, // in
			ref float swingAngle, ref IndexedVector3 vSwingAxis, ref float swingLimit) // all outs
		{
			swingAngle = MathUtil.QuatAngle(ref qCone);
			if (swingAngle > MathUtil.SIMD_EPSILON)
			{
				vSwingAxis = new IndexedVector3(qCone.X, qCone.Y, qCone.Z);
				vSwingAxis.Normalize();
				if (Math.Abs(vSwingAxis.X) > MathUtil.SIMD_EPSILON)
				{
					// non-zero twist?! this should never happen.
					Debug.Assert(false);
				}

				// Compute limit for given swing. tricky:
				// Given a swing axis, we're looking for the intersection with the bounding cone ellipse.
				// (Since we're dealing with angles, this ellipse is embedded on the surface of a sphere.)

				// For starters, compute the direction from center to surface of ellipse.
				// This is just the perpendicular (ie. rotate 2D vector by PI/2) of the swing axis.
				// (vSwingAxis is the cone rotation (in z,y); change vars and rotate to (x,y) coords.)
				float xEllipse = vSwingAxis.Y;
				float yEllipse = -vSwingAxis.Z;

				// Now, we use the slope of the vector (using x/yEllipse) and find the length
				// of the line that intersects the ellipse:
				//  x^2   y^2
				//  --- + --- = 1, where a and b are semi-major axes 2 and 1 respectively (ie. the limits)
				//  a^2   b^2
				// Do the math and it should be clear.

				swingLimit = m_swingSpan1; // if xEllipse == 0, we have a pure vSwingAxis.z rotation: just use swingspan1
				if (Math.Abs(xEllipse) > MathUtil.SIMD_EPSILON)
				{
					float surfaceSlope2 = (yEllipse * yEllipse) / (xEllipse * xEllipse);
					float norm = 1f / (m_swingSpan2 * m_swingSpan2);
					norm += surfaceSlope2 / (m_swingSpan1 * m_swingSpan1);
					float swingLimit2 = (1 + surfaceSlope2) / norm;
					swingLimit = (float)Math.Sqrt(swingLimit2);
				}

				// test!
				/*swingLimit = m_swingSpan2;
				if (fabs(vSwingAxis.z()) > SIMD_EPSILON)
				{
				float mag_2 = m_swingSpan1*m_swingSpan1 + m_swingSpan2*m_swingSpan2;
				float sinphi = m_swingSpan2 / sqrt(mag_2);
				float phi = asin(sinphi);
				float theta = atan2(fabs(vSwingAxis.y()),fabs(vSwingAxis.z()));
				float alpha = 3.14159f - theta - phi;
				float sinalpha = sin(alpha);
				swingLimit = m_swingSpan1 * sinphi/sinalpha;
				}*/
			}
			else if (swingAngle < 0)
			{
				// this should never happen!
				Debug.Assert(false);
			}
		}

		protected void ComputeTwistLimitInfo(ref IndexedQuaternion qTwist, // in
			out float twistAngle, out IndexedVector3 vTwistAxis) // all outs
		{
			IndexedQuaternion qMinTwist = qTwist;
			twistAngle = MathUtil.QuatAngle(ref qTwist);

			if (twistAngle > MathUtil.SIMD_PI) // long way around. flip quat and recalculate.
			{
				qMinTwist = -(qTwist);
				twistAngle = MathUtil.QuatAngle(ref qTwist);
			}
			if (twistAngle < 0f)
			{
				// this should never happen
				Debug.Assert(false);
			}

			vTwistAxis = new IndexedVector3(qMinTwist.X, qMinTwist.Y, qMinTwist.Z);
			if (twistAngle > MathUtil.SIMD_EPSILON)
			{
				vTwistAxis.Normalize();
			}
		}

		protected void AdjustSwingAxisToUseEllipseNormal(ref IndexedVector3 vSwingAxis)
		{
			// the swing axis is computed as the "twist-free" cone rotation,
			// but the cone limit is not circular, but elliptical (if swingspan1 != swingspan2).
			// so, if we're outside the limits, the closest way back inside the cone isn't 
			// along the vector back to the center. better (and more stable) to use the ellipse normal.

			// convert swing axis to direction from center to surface of ellipse
			// (ie. rotate 2D vector by PI/2)
			float y = -vSwingAxis.Z;
			float z = vSwingAxis.Y;

			// do the math...
			if (Math.Abs(z) > MathUtil.SIMD_EPSILON) // avoid division by 0. and we don't need an update if z == 0.
			{
				// compute gradient/normal of ellipse surface at current "point"
				float grad = y / z;
				grad *= m_swingSpan2 / m_swingSpan1;

				// adjust y/z to represent normal at point (instead of vector to point)
				if (y > 0)
				{
					y = Math.Abs(grad * z);
				}
				else
				{
					y = -Math.Abs(grad * z);
				}

				// convert ellipse direction back to swing axis
				vSwingAxis.Z = -y;
				vSwingAxis.Y = z;
				vSwingAxis.Normalize();
			}
		}


		public IndexedMatrix GetFrameOffsetA()
		{
			return m_rbAFrame;
		}

		public IndexedMatrix GetFrameOffsetB()
		{
			return m_rbBFrame;
		}


		public void SetFrames(ref IndexedMatrix frameA, ref IndexedMatrix frameB)
		{
			m_rbAFrame = frameA;
			m_rbBFrame = frameB;
			// obsolete
			BuildJacobian();
            //CalculateTransforms();
		}


        public override void BuildJacobian()
        {
            if (m_useSolveConstraintObsolete)
            {
                m_appliedImpulse = 0.0f;
                m_accTwistLimitImpulse = 0.0f;
                m_accSwingLimitImpulse = 0.0f;
                m_accMotorImpulse = IndexedVector3.Zero;

                if (!m_angularOnly)
                {
                    IndexedVector3 pivotAInW = m_rbA.GetCenterOfMassTransform() * m_rbAFrame._origin;
                    IndexedVector3 pivotBInW = m_rbB.GetCenterOfMassTransform() * m_rbBFrame._origin;
                    IndexedVector3 relPos = pivotBInW - pivotAInW;

                    IndexedVector3[] normal = new IndexedVector3[3];
                    if (relPos.LengthSquared() > MathUtil.SIMD_EPSILON)
                    {
                        normal[0] = relPos.Normalized();
                    }
                    else
                    {
                        normal[0] = new IndexedVector3(1.0f, 0, 0);
                    }

                    TransformUtil.PlaneSpace1(ref normal[0], out normal[1], out normal[2]);

                    for (int i = 0; i < 3; i++)
                    {
                        m_jac[i] = new JacobianEntry(
                        m_rbA.GetCenterOfMassTransform()._basis.Transpose(),
                        m_rbB.GetCenterOfMassTransform()._basis.Transpose(),
                        pivotAInW - m_rbA.GetCenterOfMassPosition(),
                        pivotBInW - m_rbB.GetCenterOfMassPosition(),
                        normal[i],
                        m_rbA.GetInvInertiaDiagLocal(),
                        m_rbA.GetInvMass(),
                        m_rbB.GetInvInertiaDiagLocal(),
                        m_rbB.GetInvMass());
                    }
                }

                CalcAngleInfo2(m_rbA.GetCenterOfMassTransform(), m_rbB.GetCenterOfMassTransform(), m_rbA.GetInvInertiaTensorWorld(), m_rbB.GetInvInertiaTensorWorld());
            }
        }



        public void solveConstraintObsolete(RigidBody bodyA, RigidBody bodyB, float timeStep)
{
	if (m_useSolveConstraintObsolete)
	{
		IndexedVector3 pivotAInW = m_rbA.GetCenterOfMassTransform()*m_rbAFrame._origin;
		IndexedVector3 pivotBInW = m_rbB.GetCenterOfMassTransform()*m_rbBFrame._origin;

		float tau = 0.3f;

		//linear part
		if (!m_angularOnly)
		{
			IndexedVector3 rel_pos1 = pivotAInW - m_rbA.GetCenterOfMassPosition(); 
			IndexedVector3 rel_pos2 = pivotBInW - m_rbB.GetCenterOfMassPosition();

            IndexedVector3 vel1 = IndexedVector3.Zero;
			bodyA.InternalGetVelocityInLocalPointObsolete(ref rel_pos1,ref vel1);
            IndexedVector3 vel2 = IndexedVector3.Zero;
			bodyB.InternalGetVelocityInLocalPointObsolete(ref rel_pos2,ref vel2);
			IndexedVector3 vel = vel1 - vel2;

			for (int i=0;i<3;i++)
			{		
				IndexedVector3 normal = m_jac[i].m_linearJointAxis;
				float jacDiagABInv = 1.0f / m_jac[i].GetDiagonal();

				float rel_vel = normal.Dot(ref vel);
				//positional error (zeroth order error)
				float depth = -(pivotAInW - pivotBInW).Dot(ref normal); //this is the error projected on the normal
				float impulse = depth*tau/timeStep  * jacDiagABInv -  rel_vel * jacDiagABInv;
				m_appliedImpulse += impulse;
				
				IndexedVector3 ftorqueAxis1 = rel_pos1.Cross(ref normal);
				IndexedVector3 ftorqueAxis2 = rel_pos2.Cross(ref normal);
				bodyA.InternalApplyImpulse(normal*m_rbA.GetInvMass(), m_rbA.GetInvInertiaTensorWorld()*ftorqueAxis1,impulse,null);
				bodyB.InternalApplyImpulse(normal*m_rbB.GetInvMass(), m_rbB.GetInvInertiaTensorWorld()*ftorqueAxis2,-impulse,null);
		
			}
		}

		// apply motor
		if (m_bMotorEnabled)
		{
			// compute current and predicted transforms
			IndexedMatrix trACur = m_rbA.GetCenterOfMassTransform();
			IndexedMatrix trBCur = m_rbB.GetCenterOfMassTransform();
			IndexedVector3 omegaA = IndexedVector3.Zero; bodyA.InternalGetAngularVelocity(ref omegaA);
            IndexedVector3 omegaB = IndexedVector3.Zero; bodyB.InternalGetAngularVelocity(ref omegaB);
			IndexedMatrix trAPred;
			IndexedVector3 zerovec = new IndexedVector3(0,0,0);
			TransformUtil.IntegrateTransform(ref trACur, ref zerovec, ref omegaA, timeStep, out trAPred);
			IndexedMatrix trBPred;
			TransformUtil.IntegrateTransform(ref trBCur, ref zerovec, ref omegaB, timeStep, out trBPred);

			// compute desired transforms in world
			IndexedMatrix trPose = IndexedMatrix.CreateFromQuaternion(m_qTarget);
			IndexedMatrix trABDes = m_rbBFrame * trPose * m_rbAFrame.Inverse();
			IndexedMatrix trADes = trBPred * trABDes;
			IndexedMatrix trBDes = trAPred * trABDes.Inverse();

			// compute desired omegas in world
			IndexedVector3 omegaADes, omegaBDes;
			
			TransformUtil.CalculateVelocity(ref trACur, ref trADes, timeStep, out zerovec, out omegaADes);
			TransformUtil.CalculateVelocity(ref trBCur, ref trBDes, timeStep, out zerovec, out omegaBDes);

			// compute delta omegas
			IndexedVector3 dOmegaA = omegaADes - omegaA;
			IndexedVector3 dOmegaB = omegaBDes - omegaB;

			// compute weighted avg axis of dOmega (weighting based on inertias)
            IndexedVector3 axisA = IndexedVector3.Zero, axisB = IndexedVector3.Zero;
			float kAxisAInv = 0, kAxisBInv = 0;

			if (dOmegaA.LengthSquared() > MathUtil.SIMD_EPSILON)
			{
				axisA = dOmegaA.Normalized();
				kAxisAInv = GetRigidBodyA().ComputeAngularImpulseDenominator(ref axisA);
			}

			if (dOmegaB.LengthSquared() > MathUtil.SIMD_EPSILON)
			{
				axisB = dOmegaB.Normalized();
				kAxisBInv = GetRigidBodyB().ComputeAngularImpulseDenominator(ref axisB);
			}

			IndexedVector3 avgAxis = kAxisAInv * axisA + kAxisBInv * axisB;

			if (bDoTorque && avgAxis.LengthSquared() > MathUtil.SIMD_EPSILON)
			{
				avgAxis.Normalize();
				kAxisAInv = GetRigidBodyA().ComputeAngularImpulseDenominator(ref avgAxis);
				kAxisBInv = GetRigidBodyB().ComputeAngularImpulseDenominator(ref avgAxis);
				float kInvCombined = kAxisAInv + kAxisBInv;

				IndexedVector3 impulse = (kAxisAInv * dOmegaA - kAxisBInv * dOmegaB) /
									(kInvCombined * kInvCombined);

				if (m_maxMotorImpulse >= 0)
				{
					float fMaxImpulse = m_maxMotorImpulse;
					if (m_bNormalizedMotorStrength)
						fMaxImpulse = fMaxImpulse/kAxisAInv;

					IndexedVector3 newUnclampedAccImpulse = m_accMotorImpulse + impulse;
					float  newUnclampedMag = newUnclampedAccImpulse.Length();
					if (newUnclampedMag > fMaxImpulse)
					{
						newUnclampedAccImpulse.Normalize();
						newUnclampedAccImpulse *= fMaxImpulse;
						impulse = newUnclampedAccImpulse - m_accMotorImpulse;
					}
					m_accMotorImpulse += impulse;
				}

				float  impulseMag  = impulse.Length();
				IndexedVector3 impulseAxis =  impulse / impulseMag;

				bodyA.InternalApplyImpulse(new IndexedVector3(0,0,0), m_rbA.GetInvInertiaTensorWorld()*impulseAxis, impulseMag,null);
				bodyB.InternalApplyImpulse(new IndexedVector3(0,0,0), m_rbB.GetInvInertiaTensorWorld()*impulseAxis, -impulseMag,null);

			}
		}
		else if (m_damping > MathUtil.SIMD_EPSILON) // no motor: do a little damping
		{
			IndexedVector3 angVelA = IndexedVector3.Zero; bodyA.InternalGetAngularVelocity(ref angVelA);
			IndexedVector3 angVelB= IndexedVector3.Zero; bodyB.InternalGetAngularVelocity(ref angVelB);
			IndexedVector3 relVel = angVelB - angVelA;
			if (relVel.LengthSquared() > MathUtil.SIMD_EPSILON)
			{
				IndexedVector3 relVelAxis = relVel.Normalized();
				float m_kDamping =  1.0f /
					(GetRigidBodyA().ComputeAngularImpulseDenominator(ref relVelAxis) +
					 GetRigidBodyB().ComputeAngularImpulseDenominator(ref relVelAxis));
				IndexedVector3 impulse = m_damping * m_kDamping * relVel;

				float  impulseMag  = impulse.Length();
				IndexedVector3 impulseAxis = impulse / impulseMag;
				bodyA.InternalApplyImpulse(new IndexedVector3(0,0,0), m_rbA.GetInvInertiaTensorWorld()*impulseAxis, impulseMag,null);
				bodyB.InternalApplyImpulse(new IndexedVector3(0,0,0), m_rbB.GetInvInertiaTensorWorld()*impulseAxis, -impulseMag,null);
			}
		}

		// joint limits
		{
			///solve angular part
			IndexedVector3 angVelA = IndexedVector3.Zero;
			bodyA.InternalGetAngularVelocity(ref angVelA);
			IndexedVector3 angVelB= IndexedVector3.Zero;
			bodyB.InternalGetAngularVelocity(ref angVelB);

			// solve swing limit
			if (m_solveSwingLimit)
			{
				float amplitude = m_swingLimitRatio * m_swingCorrection*m_biasFactor/timeStep;
				float relSwingVel = (angVelB - angVelA).Dot(ref m_swingAxis);
				if (relSwingVel > 0)
					amplitude += m_swingLimitRatio * relSwingVel * m_relaxationFactor;
				float impulseMag = amplitude * m_kSwing;

				// Clamp the accumulated impulse
				float temp = m_accSwingLimitImpulse;
				m_accSwingLimitImpulse = Math.Max(m_accSwingLimitImpulse + impulseMag, 0.0f);
				impulseMag = m_accSwingLimitImpulse - temp;

				IndexedVector3 impulse = m_swingAxis * impulseMag;

				// don't let cone response affect twist
				// (this can happen since body A's twist doesn't match body B's AND we use an elliptical cone limit)
				{
					IndexedVector3 impulseTwistCouple = impulse.Dot(ref m_twistAxisA) * m_twistAxisA;
					IndexedVector3 impulseNoTwistCouple = impulse - impulseTwistCouple;
					impulse = impulseNoTwistCouple;
				}

				impulseMag = impulse.Length();
				IndexedVector3 noTwistSwingAxis = impulse / impulseMag;

				bodyA.InternalApplyImpulse(new IndexedVector3(0,0,0), m_rbA.GetInvInertiaTensorWorld()*noTwistSwingAxis, impulseMag,null);
				bodyB.InternalApplyImpulse(new IndexedVector3(0,0,0), m_rbB.GetInvInertiaTensorWorld()*noTwistSwingAxis, -impulseMag,null);
			}


			// solve twist limit
			if (m_solveTwistLimit)
			{
				float amplitude = m_twistLimitRatio * m_twistCorrection*m_biasFactor/timeStep;
				float relTwistVel = (angVelB - angVelA).Dot( ref m_twistAxis );
				if (relTwistVel > 0) // only damp when moving towards limit (m_twistAxis flipping is important)
					amplitude += m_twistLimitRatio * relTwistVel * m_relaxationFactor;
				float impulseMag = amplitude * m_kTwist;

				// Clamp the accumulated impulse
				float temp = m_accTwistLimitImpulse;
				m_accTwistLimitImpulse = Math.Max(m_accTwistLimitImpulse + impulseMag, 0.0f );
				impulseMag = m_accTwistLimitImpulse - temp;

				IndexedVector3 impulse = m_twistAxis * impulseMag;

				bodyA.InternalApplyImpulse(new IndexedVector3(0,0,0), m_rbA.GetInvInertiaTensorWorld()*m_twistAxis,impulseMag,null);
				bodyB.InternalApplyImpulse(new IndexedVector3(0,0,0), m_rbB.GetInvInertiaTensorWorld()*m_twistAxis,-impulseMag,null);
			}		
		}
	}

}



	}

	[Flags]
	public enum ConeTwistFlags
	{
		BT_CONETWIST_FLAGS_LIN_CFM = 1,
		BT_CONETWIST_FLAGS_LIN_ERP = 2,
		BT_CONETWIST_FLAGS_ANG_CFM = 4
	}

}
