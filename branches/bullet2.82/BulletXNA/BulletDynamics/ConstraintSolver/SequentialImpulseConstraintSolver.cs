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
using BulletXNA.BulletCollision;
using BulletXNA.LinearMath;
using System.Collections.Generic;

namespace BulletXNA.BulletDynamics
{
	public class SequentialImpulseConstraintSolver : IConstraintSolver
	{
		private static RigidBody s_fixed;
		protected ObjectArray<SolverConstraint> m_tmpSolverContactConstraintPool;
		protected ObjectArray<SolverConstraint> m_tmpSolverNonContactConstraintPool;
		protected ObjectArray<SolverConstraint> m_tmpSolverContactFrictionConstraintPool;
		protected ObjectArray<int> m_orderTmpConstraintPool;
        protected ObjectArray<int> m_orderNonContactConstraintPool;
		protected ObjectArray<int> m_orderFrictionConstraintPool;
		protected ObjectArray<ConstraintInfo1> m_tmpConstraintSizesPool;
        protected ObjectArray<ConstraintInfo2> m_tmpConstraintInfo2Pool;

        protected int m_maxOverrideNumSolverIterations;


		// FIXME - MAN - decide where this lives.
		static int gNumSplitImpulseRecoveries = 0;

		///m_btSeed2 is used for re-arranging the constraint rows. improves convergence/quality of friction
		protected ulong m_btSeed2;

		protected int m_counter;

        protected int m_lowerLimitCount = 0;
        protected int m_genericCount = 0;
        protected int m_setupCount = 0;
        protected int m_iterCount = 0;
        protected int m_finishCount = 0;


		public SequentialImpulseConstraintSolver()
		{

			//FIXME RHS SEEMS TO BE OUT

			m_tmpSolverContactConstraintPool = new ObjectArray<SolverConstraint>();
			m_tmpSolverNonContactConstraintPool = new ObjectArray<SolverConstraint>();
			m_tmpSolverContactFrictionConstraintPool = new ObjectArray<SolverConstraint>();
			m_tmpConstraintSizesPool = new ObjectArray<ConstraintInfo1>();
			m_orderTmpConstraintPool = new ObjectArray<int>();
			m_orderFrictionConstraintPool = new ObjectArray<int>();
            m_orderNonContactConstraintPool = new ObjectArray<int>();
            m_tmpConstraintInfo2Pool = new ObjectArray<ConstraintInfo2>();
		}

		public virtual void Cleanup()
		{

		}

		public void SetupFrictionConstraint(ref SolverConstraint solverConstraint, ref IndexedVector3 normalAxis, RigidBody solverBodyA, RigidBody solverBodyB,
						ManifoldPoint cp, ref IndexedVector3 rel_pos1, ref IndexedVector3 rel_pos2,
						CollisionObject colObj0, CollisionObject colObj1, float relaxation)
		{
			SetupFrictionConstraint(ref solverConstraint, ref normalAxis, solverBodyA, solverBodyB, cp, ref rel_pos1, ref rel_pos2, colObj0, colObj1, relaxation, 0f, 0f);
		}

		public void SetupFrictionConstraint(ref SolverConstraint solverConstraint, ref IndexedVector3 normalAxis, RigidBody solverBodyA, RigidBody solverBodyB,
								ManifoldPoint cp, ref IndexedVector3 rel_pos1, ref IndexedVector3 rel_pos2,
								CollisionObject colObj0, CollisionObject colObj1, float relaxation,
								float desiredVelocity, float cfmSlip)
		{
			RigidBody body0 = RigidBody.Upcast(colObj0);
			RigidBody body1 = RigidBody.Upcast(colObj1);

			solverConstraint.m_contactNormal = normalAxis;

			solverConstraint.m_solverBodyA = body0 != null ? body0 : GetFixedBody();
			solverConstraint.m_solverBodyB = body1 != null ? body1 : GetFixedBody();

			solverConstraint.m_friction = cp.GetCombinedFriction();
#if DEBUG
            if (BulletGlobals.g_streamWriter != null && (body0 != null  || body1 != null) && BulletGlobals.debugSolver)
            {
                BulletGlobals.g_streamWriter.WriteLine("SetupFrictionConstraint[{0}][{1}]", (String)solverConstraint.m_solverBodyA.GetUserPointer(), (String)solverConstraint.m_solverBodyB.GetUserPointer());
                MathUtil.PrintContactPoint(BulletGlobals.g_streamWriter, cp);
            }
#endif

			solverConstraint.m_originalContactPoint = null;
            //solverConstraint.m_originalContactPointConstraint = null;
			solverConstraint.m_appliedImpulse = 0f;
			solverConstraint.m_appliedPushImpulse = 0f;




			{
				IndexedVector3 ftorqueAxis1 = IndexedVector3.Cross(rel_pos1, solverConstraint.m_contactNormal);
				solverConstraint.m_relpos1CrossNormal = ftorqueAxis1;
                solverConstraint.m_angularComponentA = body0 != null ? body0.GetInvInertiaTensorWorld() * ftorqueAxis1 * body0.GetAngularFactor() : IndexedVector3.Zero;
            }
			{
				IndexedVector3 ftorqueAxis1 = IndexedVector3.Cross(rel_pos2, -solverConstraint.m_contactNormal);
				solverConstraint.m_relpos2CrossNormal = ftorqueAxis1;
                solverConstraint.m_angularComponentB = body1 != null ? body1.GetInvInertiaTensorWorld() * ftorqueAxis1 * body1.GetAngularFactor() : IndexedVector3.Zero;
            }

#if COMPUTE_IMPULSE_DENOM
	        float denom0 = rb0.computeImpulseDenominator(pos1,solverConstraint.m_contactNormal);
	        float denom1 = rb1.computeImpulseDenominator(pos2,solverConstraint.m_contactNormal);
#else
			IndexedVector3 vec;
			float denom0 = 0f;
			float denom1 = 0f;
			if (body0 != null)
			{
				vec = IndexedVector3.Cross(solverConstraint.m_angularComponentA, rel_pos1);
				denom0 = body0.GetInvMass() + IndexedVector3.Dot(normalAxis, vec);
			}
			if (body1 != null)
			{
				vec = IndexedVector3.Cross(-solverConstraint.m_angularComponentB, rel_pos2);
				denom1 = body1.GetInvMass() + IndexedVector3.Dot(normalAxis, vec);
			}


#endif //COMPUTE_IMPULSE_DENOM
			float denom = relaxation / (denom0 + denom1);
			solverConstraint.m_jacDiagABInv = denom;
			MathUtil.SanityCheckFloat(solverConstraint.m_jacDiagABInv);
#if _USE_JACOBIAN
	        solverConstraint.m_jac =  new JacobianEntry (
		        ref rel_pos1,ref rel_pos2,ref solverConstraint.m_contactNormal,
		        body0.getInvInertiaDiagLocal(),
		        body0.getInvMass(),
		        body1.getInvInertiaDiagLocal(),
		        body1.getInvMass());
#endif //_USE_JACOBIAN


			{
				float rel_vel;
				float vel1Dotn = IndexedVector3.Dot(solverConstraint.m_contactNormal, body0 != null ? body0.GetLinearVelocity() : IndexedVector3.Zero)
					+ IndexedVector3.Dot(solverConstraint.m_relpos1CrossNormal, body0 != null ? body0.GetAngularVelocity() : IndexedVector3.Zero);
				float vel2Dotn = -IndexedVector3.Dot(solverConstraint.m_contactNormal, body1 != null ? body1.GetLinearVelocity() : IndexedVector3.Zero)
					+ IndexedVector3.Dot(solverConstraint.m_relpos2CrossNormal, body1 != null ? body1.GetAngularVelocity() : IndexedVector3.Zero);

				rel_vel = vel1Dotn + vel2Dotn;

				//float positionalError = 0f;

				float velocityError = desiredVelocity - rel_vel;
				float damper = 1f;
				float velocityImpulse = (velocityError * solverConstraint.m_jacDiagABInv) * damper;
				solverConstraint.m_rhs = velocityImpulse;
				solverConstraint.m_cfm = cfmSlip;
				solverConstraint.m_lowerLimit = 0;
				solverConstraint.m_upperLimit = 1e10f;
			}
		}

		public SolverConstraint AddFrictionConstraint(ref IndexedVector3 normalAxis, RigidBody solverBodyA, RigidBody solverBodyB, int frictionIndex, ManifoldPoint cp, ref IndexedVector3 rel_pos1, ref IndexedVector3 rel_pos2, CollisionObject colObj0, CollisionObject colObj1, float relaxation, float desiredVelocity, float cfmSlip)
		{
            // will create the object
            SolverConstraint solverConstraint = m_tmpSolverContactFrictionConstraintPool[m_tmpSolverContactFrictionConstraintPool.Count];
            solverConstraint.Reset();
			solverConstraint.m_frictionIndex = frictionIndex;
			SetupFrictionConstraint(ref solverConstraint, ref normalAxis, solverBodyA, solverBodyB, cp, ref rel_pos1, ref rel_pos2,
									colObj0, colObj1, relaxation, desiredVelocity, cfmSlip);
			return solverConstraint;
		}

		protected void SetupContactConstraint(ref SolverConstraint solverConstraint, CollisionObject colObj0, CollisionObject colObj1, ManifoldPoint cp,
								ContactSolverInfo infoGlobal, ref IndexedVector3 vel, ref float rel_vel, ref float relaxation,
								out IndexedVector3 rel_pos1, out IndexedVector3 rel_pos2)
		{
            RigidBody rb0 = colObj0 as RigidBody;// RigidBody.Upcast(colObj0);
            RigidBody rb1 = colObj1 as RigidBody;//RigidBody.Upcast(colObj1);

			IndexedVector3 pos1 = cp.GetPositionWorldOnA();
			IndexedVector3 pos2 = cp.GetPositionWorldOnB();

			rel_pos1 = pos1 - colObj0.m_worldTransform._origin;
			rel_pos2 = pos2 - colObj1.m_worldTransform._origin;

			relaxation = 1f;

            // cross

            IndexedVector3 torqueAxis0 = new IndexedVector3(rel_pos1.Y *cp.m_normalWorldOnB.Z - rel_pos1.Z * cp.m_normalWorldOnB.Y,
                rel_pos1.Z *cp.m_normalWorldOnB.X - rel_pos1.X * cp.m_normalWorldOnB.Z,
                rel_pos1.X *cp.m_normalWorldOnB.Y - rel_pos1.Y * cp.m_normalWorldOnB.X);

            IndexedVector3 torqueAxis1 = new IndexedVector3(rel_pos2.Y *cp.m_normalWorldOnB.Z - rel_pos2.Z * cp.m_normalWorldOnB.Y,
                rel_pos2.Z *cp.m_normalWorldOnB.X - rel_pos2.X * cp.m_normalWorldOnB.Z,
                rel_pos2.X *cp.m_normalWorldOnB.Y - rel_pos2.Y * cp.m_normalWorldOnB.X);


            solverConstraint.m_angularComponentA = rb0 != null ? rb0.GetInvInertiaTensorWorld() * torqueAxis0 * rb0.GetAngularFactor() : IndexedVector3.Zero;
            //IndexedVector3 torqueAxis1 = IndexedVector3.Cross(ref rel_pos2, ref cp.m_normalWorldOnB);
            solverConstraint.m_angularComponentB = rb1 != null ? rb1.GetInvInertiaTensorWorld() * -torqueAxis1 * rb1.GetAngularFactor() : IndexedVector3.Zero;


            //IndexedVector3 torqueAxis0 = IndexedVector3.Cross(ref rel_pos1, ref cp.m_normalWorldOnB);
            //solverConstraint.m_angularComponentA = rb0 != null ? rb0.GetInvInertiaTensorWorld() * torqueAxis0 * rb0.GetAngularFactor() : IndexedVector3.Zero;
            //IndexedVector3 torqueAxis1 = IndexedVector3.Cross(ref rel_pos2, ref cp.m_normalWorldOnB);
            //solverConstraint.m_angularComponentB = rb1 != null ? rb1.GetInvInertiaTensorWorld() * -torqueAxis1 * rb1.GetAngularFactor() : IndexedVector3.Zero;

			{
#if COMPUTE_IMPULSE_DENOM
		        float denom0 = rb0.computeImpulseDenominator(pos1,cp.m_normalWorldOnB);
		        float denom1 = rb1.computeImpulseDenominator(pos2,cp.m_normalWorldOnB);
#else
				IndexedVector3 vec;
				float denom0 = 0f;
				float denom1 = 0f;
				if (rb0 != null)
				{
					vec = IndexedVector3.Cross(ref solverConstraint.m_angularComponentA, ref rel_pos1);
                    denom0 = rb0.GetInvMass() + IndexedVector3.Dot(cp.m_normalWorldOnB, vec);
				}
				if (rb1 != null)
				{
					vec = IndexedVector3.Cross((-solverConstraint.m_angularComponentB), rel_pos2);
                    denom1 = rb1.GetInvMass() + IndexedVector3.Dot(cp.m_normalWorldOnB, vec);
				}
#endif //COMPUTE_IMPULSE_DENOM

				float denom = relaxation / (denom0 + denom1);
				MathUtil.SanityCheckFloat(denom);
				solverConstraint.m_jacDiagABInv = denom;
			}

			solverConstraint.m_contactNormal = cp.m_normalWorldOnB;
			solverConstraint.m_relpos1CrossNormal = IndexedVector3.Cross(rel_pos1, cp.m_normalWorldOnB);
			solverConstraint.m_relpos2CrossNormal = IndexedVector3.Cross(rel_pos2, -cp.m_normalWorldOnB);



			IndexedVector3 vel1 = rb0 != null ? rb0.GetVelocityInLocalPoint(ref rel_pos1) : IndexedVector3.Zero;
			IndexedVector3 vel2 = rb1 != null ? rb1.GetVelocityInLocalPoint(ref rel_pos2) : IndexedVector3.Zero;

			vel = vel1 - vel2;

            rel_vel = IndexedVector3.Dot(cp.m_normalWorldOnB, vel);

			float penetration = cp.GetDistance() + infoGlobal.m_linearSlop;


			solverConstraint.m_friction = cp.GetCombinedFriction();

			float restitution = 0f;

			if (cp.GetLifeTime() > infoGlobal.m_restingContactRestitutionThreshold)
			{
				restitution = 0f;
			}
			else
			{
				restitution = RestitutionCurve(rel_vel, cp.GetCombinedResitution());
				if (restitution <= 0f)
				{
					restitution = 0f;
				}
			}

			///warm starting (or zero if disabled)
			if (TestSolverMode(infoGlobal.m_solverMode, SolverMode.SOLVER_USE_WARMSTARTING))
			{
				solverConstraint.m_appliedImpulse = cp.GetAppliedImpulse() * infoGlobal.m_warmstartingFactor;
				if (rb0 != null)
				{
					IndexedVector3 contactNormalTemp = solverConstraint.m_contactNormal;
                    rb0.InternalApplyImpulse(solverConstraint.m_contactNormal * rb0.GetInvMass() * rb0.GetLinearFactor(), solverConstraint.m_angularComponentA, solverConstraint.m_appliedImpulse, "SetupContactConstraint-rb0");
                }
				if (rb1 != null)
				{
                    rb1.InternalApplyImpulse(solverConstraint.m_contactNormal * rb1.GetInvMass() * rb1.GetLinearFactor(), -solverConstraint.m_angularComponentB, -solverConstraint.m_appliedImpulse,"SetupContactConstraint-rb1");
                }
			}
			else
			{
				solverConstraint.m_appliedImpulse = 0f;
			}
			solverConstraint.m_appliedPushImpulse = 0f;
			{
				float rel_vel2 = 0f;
				float vel1Dotn = IndexedVector3.Dot(solverConstraint.m_contactNormal, (rb0 != null ? rb0.GetLinearVelocity() : IndexedVector3.Zero))
					+ IndexedVector3.Dot(solverConstraint.m_relpos1CrossNormal, (rb0 != null ? rb0.GetAngularVelocity() : IndexedVector3.Zero));
				float vel2Dotn = -IndexedVector3.Dot(solverConstraint.m_contactNormal, (rb1 != null ? rb1.GetLinearVelocity() : IndexedVector3.Zero))
					+ IndexedVector3.Dot(solverConstraint.m_relpos2CrossNormal, (rb1 != null ? rb1.GetAngularVelocity() : IndexedVector3.Zero));

				rel_vel2 = vel1Dotn + vel2Dotn;

				float positionalError = 0f;

                if (rel_vel2 > 20)
                {
                    int ibreak = 0;
                }

				float velocityError = restitution - rel_vel2;// * damping;

				if (penetration > 0f)
				{
					positionalError = 0f;
					velocityError -= penetration / infoGlobal.m_timeStep;
				}
				else
				{
					positionalError = -penetration * infoGlobal.m_erp / infoGlobal.m_timeStep;
				}
				
				float penetrationImpulse = positionalError * solverConstraint.m_jacDiagABInv;
				float velocityImpulse = velocityError * solverConstraint.m_jacDiagABInv;
				if (!infoGlobal.m_splitImpulse || (penetration > infoGlobal.m_splitImpulsePenetrationThreshold))
				{
					//combine position and velocity into rhs
					solverConstraint.m_rhs = penetrationImpulse + velocityImpulse;
					solverConstraint.m_rhsPenetration = 0f;
				}
				else
				{
					//split position and velocity into rhs and m_rhsPenetration
					solverConstraint.m_rhs = velocityImpulse;
					solverConstraint.m_rhsPenetration = penetrationImpulse;
				}
				solverConstraint.m_cfm = 0f;
				solverConstraint.m_lowerLimit = 0;
				solverConstraint.m_upperLimit = 1e10f;
			}
		}

		protected void SetFrictionConstraintImpulse(ref SolverConstraint solverConstraint, RigidBody rb0, RigidBody rb1,
										 ManifoldPoint cp, ContactSolverInfo infoGlobal)
		{
			if (TestSolverMode(infoGlobal.m_solverMode, SolverMode.SOLVER_USE_FRICTION_WARMSTARTING))
			{
				{
					SolverConstraint frictionConstraint1 = m_tmpSolverContactFrictionConstraintPool[solverConstraint.m_frictionIndex];
					if (TestSolverMode(infoGlobal.m_solverMode, SolverMode.SOLVER_USE_WARMSTARTING))
					{
						frictionConstraint1.m_appliedImpulse = cp.m_appliedImpulseLateral1 * infoGlobal.m_warmstartingFactor;
						if (rb0 != null)
						{
                            rb0.InternalApplyImpulse(frictionConstraint1.m_contactNormal * rb0.GetInvMass(), frictionConstraint1.m_angularComponentA, frictionConstraint1.m_appliedImpulse,"SetupFriction-rb0");
						}
						if (rb1 != null)
						{
                            rb1.InternalApplyImpulse(frictionConstraint1.m_contactNormal * rb1.GetInvMass(), -frictionConstraint1.m_angularComponentB, -frictionConstraint1.m_appliedImpulse, "SetupFriction-rb1");
						}
					}
					else
					{
						frictionConstraint1.m_appliedImpulse = 0f;
					}
					m_tmpSolverContactFrictionConstraintPool[solverConstraint.m_frictionIndex] = frictionConstraint1;

				}

				if (TestSolverMode(infoGlobal.m_solverMode, SolverMode.SOLVER_USE_2_FRICTION_DIRECTIONS))
				{
					SolverConstraint frictionConstraint2 = m_tmpSolverContactFrictionConstraintPool[solverConstraint.m_frictionIndex + 1];
					if (TestSolverMode(infoGlobal.m_solverMode, SolverMode.SOLVER_USE_WARMSTARTING))
					{
						frictionConstraint2.m_appliedImpulse = cp.m_appliedImpulseLateral2 * infoGlobal.m_warmstartingFactor;
						if (rb0 != null)
						{
							rb0.InternalApplyImpulse(frictionConstraint2.m_contactNormal * rb0.GetInvMass(), frictionConstraint2.m_angularComponentA, frictionConstraint2.m_appliedImpulse,"SetFriction-rb0");
						}
						if (rb1 != null)
						{
							rb1.InternalApplyImpulse(frictionConstraint2.m_contactNormal * rb1.GetInvMass(), -frictionConstraint2.m_angularComponentB, -frictionConstraint2.m_appliedImpulse,"SetFriction-rb1");
						}
					}
					else
					{
						frictionConstraint2.m_appliedImpulse = 0f;
					}
					m_tmpSolverContactFrictionConstraintPool[solverConstraint.m_frictionIndex + 1] = frictionConstraint2;
				}
			}
			else
			{
				SolverConstraint frictionConstraint1 = m_tmpSolverContactFrictionConstraintPool[solverConstraint.m_frictionIndex];
				frictionConstraint1.m_appliedImpulse = 0f;
				if (TestSolverMode(infoGlobal.m_solverMode, SolverMode.SOLVER_USE_2_FRICTION_DIRECTIONS))
				{
					SolverConstraint frictionConstraint2 = m_tmpSolverContactFrictionConstraintPool[solverConstraint.m_frictionIndex + 1];
					frictionConstraint2.m_appliedImpulse = 0f;
					m_tmpSolverContactFrictionConstraintPool[solverConstraint.m_frictionIndex + 1] = frictionConstraint2;
				}
				m_tmpSolverContactFrictionConstraintPool[solverConstraint.m_frictionIndex] = frictionConstraint1;

			}
		}

		//protected void initSolverBody(SolverBody solverBody, CollisionObject collisionObject)
		//{
		//    RigidBody rb = collisionObject != null ? RigidBody.upcast(collisionObject) : null;

		//    solverBody.setDeltaLinearVelocity(IndexedVector3.Zero);
		//    solverBody.setDeltaLinearVelocity(IndexedVector3.Zero);

		//    if (rb != null)
		//    {
		//        solverBody.m_invMass = rb.getInvMass();
		//        solverBody.m_originalBody = rb;
		//        solverBody.m_angularFactor = rb.getAngularFactor();
		//    } else
		//    {
		//        solverBody.m_invMass = 0f;
		//        solverBody.m_originalBody = null;
		//        solverBody.m_angularFactor = 1f;
		//    }
		//}

		protected float RestitutionCurve(float rel_vel, float restitution)
		{
            // base on rest2 ?
			float rest = (restitution * restitution * -rel_vel);
			return rest;
		}

		protected void ConvertContact(PersistentManifold manifold, ContactSolverInfo infoGlobal)
		{
			CollisionObject colObj0 = null, colObj1 = null;


			colObj0 = manifold.GetBody0() as CollisionObject;
			colObj1 = manifold.GetBody1() as CollisionObject;

			RigidBody solverBodyA = colObj0 as RigidBody;//RigidBody.Upcast(colObj0);
            RigidBody solverBodyB = colObj1 as RigidBody;//RigidBody.Upcast(colObj1);

			///avoid collision response between two static objects
			if ((solverBodyA == null || solverBodyA.GetInvMass() == 0f) && (solverBodyB == null || solverBodyB.GetInvMass() == 0f))
			{
				return;
			}

			for (int j = 0; j < manifold.GetNumContacts(); j++)
			{
				ManifoldPoint cp = manifold.GetContactPoint(j);
#if DEBUG				
                if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugSolver && false)
                {
                    String nameA = solverBodyA != null ? (String)solverBodyA.GetUserPointer() : "Null";
                    String nameB = solverBodyA != null ? (String)solverBodyB.GetUserPointer() : "Null";

                    BulletGlobals.g_streamWriter.WriteLine("ConvertContact [{0}][{1}][{2}][{3}][{4}]", j, nameA,nameB, cp.GetDistance() ,manifold.GetContactProcessingThreshold());
                    MathUtil.PrintContactPoint(BulletGlobals.g_streamWriter, cp);
                }
#endif

				if (cp.GetDistance() <= manifold.GetContactProcessingThreshold())
				{
					//                    IndexedVector3 pos1 = cp.getPositionWorldOnA();
					//                    IndexedVector3 pos2 = cp.getPositionWorldOnB();

					IndexedVector3 rel_pos1;
					IndexedVector3 rel_pos2;
					//;

					float relaxation = 1f;
					float rel_vel = 0f;
					IndexedVector3 vel = IndexedVector3.Zero;

					int frictionIndex = m_tmpSolverContactConstraintPool.Count;

                    // will create if needed.
                    SolverConstraint solverConstraint = m_tmpSolverContactConstraintPool[m_tmpSolverContactConstraintPool.Count];
                    solverConstraint.Reset();

                    RigidBody rb0 = solverBodyA;//RigidBody.Upcast(colObj0);
					RigidBody rb1 = solverBodyB;//RigidBody.Upcast(colObj1);
					solverConstraint.m_solverBodyA = rb0 != null ? rb0 : GetFixedBody();
					solverConstraint.m_solverBodyB = rb1 != null ? rb1 : GetFixedBody();

					solverConstraint.m_originalContactPoint = cp;

#if DEBUG
                    if (BulletGlobals.g_streamWriter != null && rb0 != null && BulletGlobals.debugSolver)
                    {
                        BulletGlobals.g_streamWriter.WriteLine("ConvertContact [{0}][{1}]", (String)solverConstraint.m_solverBodyA.GetUserPointer(), (String)solverConstraint.m_solverBodyB.GetUserPointer());
                        MathUtil.PrintContactPoint(BulletGlobals.g_streamWriter, cp);
                    }
#endif


                    SetupContactConstraint(ref solverConstraint, colObj0, colObj1, cp, infoGlobal, ref vel, ref rel_vel, ref relaxation, out rel_pos1, out rel_pos2);

#if DEBUG
					if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugSolver)
					{
						TypedConstraint.PrintSolverConstraint(BulletGlobals.g_streamWriter, solverConstraint, 99);
					}
#endif
					/////setup the friction constraints

					solverConstraint.m_frictionIndex = m_tmpSolverContactFrictionConstraintPool.Count;

					if (!(TestSolverMode(infoGlobal.m_solverMode, SolverMode.SOLVER_ENABLE_FRICTION_DIRECTION_CACHING)) || !cp.GetLateralFrictionInitialized())
					{
						cp.m_lateralFrictionDir1 = vel - cp.m_normalWorldOnB * rel_vel;
						float lat_rel_vel = cp.m_lateralFrictionDir1.LengthSquared();
						if (!TestSolverMode(infoGlobal.m_solverMode, SolverMode.SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION) && lat_rel_vel > MathUtil.SIMD_EPSILON)
						{
							cp.m_lateralFrictionDir1 /= (float)Math.Sqrt(lat_rel_vel);

							if (TestSolverMode(infoGlobal.m_solverMode, SolverMode.SOLVER_USE_2_FRICTION_DIRECTIONS))
							{
								cp.m_lateralFrictionDir2 = IndexedVector3.Cross(cp.m_lateralFrictionDir1, cp.m_normalWorldOnB);
								cp.m_lateralFrictionDir2.Normalize();//??
								ApplyAnisotropicFriction(colObj0, ref cp.m_lateralFrictionDir2);
								ApplyAnisotropicFriction(colObj1, ref cp.m_lateralFrictionDir2);
								AddFrictionConstraint(ref cp.m_lateralFrictionDir2, solverBodyA, solverBodyB, frictionIndex, cp, ref rel_pos1, ref rel_pos2, colObj0, colObj1, relaxation, 0f, 0f);
							}
							ApplyAnisotropicFriction(colObj0, ref cp.m_lateralFrictionDir1);
							ApplyAnisotropicFriction(colObj1, ref cp.m_lateralFrictionDir1);
							AddFrictionConstraint(ref cp.m_lateralFrictionDir1, solverBodyA, solverBodyB, frictionIndex, cp, ref rel_pos1, ref rel_pos2, colObj0, colObj1, relaxation, 0f, 0f);

							cp.m_lateralFrictionInitialized = true;
						}
						else
						{
							//re-calculate friction direction every frame, todo: check if this is really needed
                            IndexedVector3 temp = cp.m_normalWorldOnB;
							TransformUtil.PlaneSpace1(ref temp, out cp.m_lateralFrictionDir1, out cp.m_lateralFrictionDir2);
							if (TestSolverMode(infoGlobal.m_solverMode, SolverMode.SOLVER_USE_2_FRICTION_DIRECTIONS))
							{
								ApplyAnisotropicFriction(colObj0, ref cp.m_lateralFrictionDir2);
								ApplyAnisotropicFriction(colObj1, ref cp.m_lateralFrictionDir2);
								AddFrictionConstraint(ref cp.m_lateralFrictionDir2, solverBodyA, solverBodyB, frictionIndex, cp, ref rel_pos1, ref rel_pos2, colObj0, colObj1, relaxation, 0f, 0f);
							}
							ApplyAnisotropicFriction(colObj0, ref cp.m_lateralFrictionDir1);
							ApplyAnisotropicFriction(colObj1, ref cp.m_lateralFrictionDir1);
							AddFrictionConstraint(ref cp.m_lateralFrictionDir1, solverBodyA, solverBodyB, frictionIndex, cp, ref rel_pos1, ref rel_pos2, colObj0, colObj1, relaxation, 0f, 0f);

							cp.m_lateralFrictionInitialized = true;
						}

					}
					else
					{
						AddFrictionConstraint(ref cp.m_lateralFrictionDir1, solverBodyA, solverBodyB, frictionIndex, cp, ref rel_pos1, ref rel_pos2, colObj0, colObj1, relaxation, cp.m_contactMotion1, cp.m_contactCFM1);

						if (TestSolverMode(infoGlobal.m_solverMode, SolverMode.SOLVER_USE_2_FRICTION_DIRECTIONS))
						{
							AddFrictionConstraint(ref cp.m_lateralFrictionDir2, solverBodyA, solverBodyB, frictionIndex, cp, ref rel_pos1, ref rel_pos2, colObj0, colObj1, relaxation, cp.m_contactMotion2, cp.m_contactCFM2);
						}
					}
					SetFrictionConstraintImpulse(ref solverConstraint, rb0, rb1, cp, infoGlobal);
                    
				}
			}
		}


		//internal method
		protected int GetOrInitSolverBody(CollisionObject body)
		{
			//int solverBodyA = -1;

			//if (body.getCompanionId() >= 0)
			//{
			//    //body has already been converted
			//    solverBodyA = body.getCompanionId();
			//} 
			//else
			//{
			//    RigidBody rb = RigidBody.upcast(body);
			//    if (rb != null && !MathUtil.0fuzzyZero(rb.getInvMass()))
			//    {
			//        solverBodyA = m_tmpSolverBodyPool.Count;
			//        SolverBody solverBody = new SolverBody();
			//        m_tmpSolverBodyPool.Add(solverBody);
			//        initSolverBody(solverBody, body);
			//        body.setCompanionId(solverBodyA);

			//    } 
			//    else
			//    {
			//        return 0;//assume first one is a fixed solver body
			//    }
			//}
			//return solverBodyA;
			return 0;
		}

		protected void ResolveSingleConstraintRowGeneric(RigidBody body1, RigidBody body2, ref SolverConstraint c)
		{
            m_genericCount++;
                float deltaImpulse = c.m_rhs - c.m_appliedImpulse * c.m_cfm;

                float deltaVel1Dotn = (c.m_contactNormal.X * body1.m_deltaLinearVelocity.X) + (c.m_contactNormal.Y * body1.m_deltaLinearVelocity.Y) + (c.m_contactNormal.Z * body1.m_deltaLinearVelocity.Z) +
                (c.m_relpos1CrossNormal.X * body1.m_deltaAngularVelocity.X) + (c.m_relpos1CrossNormal.Y * body1.m_deltaAngularVelocity.Y) + (c.m_relpos1CrossNormal.Z * body1.m_deltaAngularVelocity.Z);

                float deltaVel2Dotn = -((c.m_contactNormal.X * body2.m_deltaLinearVelocity.X) + (c.m_contactNormal.Y * body2.m_deltaLinearVelocity.Y) + (c.m_contactNormal.Z * body2.m_deltaLinearVelocity.Z)) +
                (c.m_relpos2CrossNormal.X * body2.m_deltaAngularVelocity.X) + (c.m_relpos2CrossNormal.Y * body2.m_deltaAngularVelocity.Y) + (c.m_relpos2CrossNormal.Z * body2.m_deltaAngularVelocity.Z);


                float originalDeltaImpulse = deltaImpulse;

                deltaImpulse -= deltaVel1Dotn * c.m_jacDiagABInv;
                deltaImpulse -= deltaVel2Dotn * c.m_jacDiagABInv;

#if DEBUG
                if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugSolver && false)
                {
                    BulletGlobals.g_streamWriter.WriteLine("ResolveSingleConstraintRowGeneric start [{0}][{1}][{2}][{3}].", originalDeltaImpulse, deltaVel1Dotn, deltaVel2Dotn, c.m_jacDiagABInv);
                }
#endif
                float sum = c.m_appliedImpulse + deltaImpulse;
                if (sum < c.m_lowerLimit)
                {
                    deltaImpulse = c.m_lowerLimit - c.m_appliedImpulse;
                    c.m_appliedImpulse = c.m_lowerLimit;
                }
                else if (sum > c.m_upperLimit)
                {
                    deltaImpulse = c.m_upperLimit - c.m_appliedImpulse;
                    c.m_appliedImpulse = c.m_upperLimit;
                }
                else
                {
                    c.m_appliedImpulse = sum;
                }

                IndexedVector3 temp = new IndexedVector3(c.m_contactNormal.X * body1.m_invMass.X, c.m_contactNormal.Y * body1.m_invMass.Y, c.m_contactNormal.Z * body1.m_invMass.Z);

                body1.InternalApplyImpulse(ref temp, ref c.m_angularComponentA, deltaImpulse, "ResolveSingleConstraintGeneric-body1");

                temp = new IndexedVector3(-c.m_contactNormal.X * body2.m_invMass.X, -c.m_contactNormal.Y * body2.m_invMass.Y, -c.m_contactNormal.Z * body2.m_invMass.Z);

                body2.InternalApplyImpulse(ref temp, ref c.m_angularComponentB, deltaImpulse, "ResolveSingleConstraintGeneric-body2");
		}

		protected void ResolveSingleConstraintRowLowerLimit(RigidBody body1, RigidBody body2, ref SolverConstraint c)
		{
            m_lowerLimitCount++;
			//check magniture of applied impulse from SolverConstraint 
			float deltaImpulse = c.m_rhs - c.m_appliedImpulse * c.m_cfm;

            if (deltaImpulse > 200)
            {
                int ibreak = 0;
            }

            float deltaVel1Dotn = (c.m_contactNormal.X * body1.m_deltaLinearVelocity.X) + (c.m_contactNormal.Y * body1.m_deltaLinearVelocity.Y) + (c.m_contactNormal.Z * body1.m_deltaLinearVelocity.Z) +
            (c.m_relpos1CrossNormal.X * body1.m_deltaAngularVelocity.X) + (c.m_relpos1CrossNormal.Y * body1.m_deltaAngularVelocity.Y) + (c.m_relpos1CrossNormal.Z * body1.m_deltaAngularVelocity.Z);

            float deltaVel2Dotn = -((c.m_contactNormal.X * body2.m_deltaLinearVelocity.X) + (c.m_contactNormal.Y * body2.m_deltaLinearVelocity.Y) + (c.m_contactNormal.Z * body2.m_deltaLinearVelocity.Z)) +
            (c.m_relpos2CrossNormal.X * body2.m_deltaAngularVelocity.X) + (c.m_relpos2CrossNormal.Y * body2.m_deltaAngularVelocity.Y) + (c.m_relpos2CrossNormal.Z * body2.m_deltaAngularVelocity.Z);

			deltaImpulse -= deltaVel1Dotn * c.m_jacDiagABInv;
			deltaImpulse -= deltaVel2Dotn * c.m_jacDiagABInv;

			float sum = c.m_appliedImpulse + deltaImpulse;

			if (sum < c.m_lowerLimit)
			{
				deltaImpulse = c.m_lowerLimit - c.m_appliedImpulse;
				c.m_appliedImpulse = c.m_lowerLimit;
			}
			else
			{
				c.m_appliedImpulse = sum;
			}

            IndexedVector3 temp = new IndexedVector3(c.m_contactNormal.X * body1.m_invMass.X,c.m_contactNormal.Y * body1.m_invMass.Y,c.m_contactNormal.Z * body1.m_invMass.Z);
            body1.InternalApplyImpulse(ref temp, ref c.m_angularComponentA, deltaImpulse, "ResolveSingleConstraintRowLowerLimit-body1");

            temp = new IndexedVector3(-c.m_contactNormal.X * body2.m_invMass.X, -c.m_contactNormal.Y * body2.m_invMass.Y, -c.m_contactNormal.Z * body2.m_invMass.Z);
            body2.InternalApplyImpulse(ref temp, ref c.m_angularComponentB, deltaImpulse, "ResolveSingleConstraintRowLowerLimit-body2");

		}

		protected void ResolveSplitPenetrationImpulseCacheFriendly(
			RigidBody body1,
			RigidBody body2,
			ref SolverConstraint c)
		{
			if (c.m_rhsPenetration != 0f)
			{
				gNumSplitImpulseRecoveries++;
				float deltaImpulse = c.m_rhsPenetration - (c.m_appliedPushImpulse * c.m_cfm);
				float deltaVel1Dotn = IndexedVector3.Dot(c.m_contactNormal, body1.InternalGetPushVelocity()) + IndexedVector3.Dot(c.m_relpos1CrossNormal, body1.InternalGetTurnVelocity());
				float deltaVel2Dotn = -IndexedVector3.Dot(c.m_contactNormal, body2.InternalGetPushVelocity()) + IndexedVector3.Dot(c.m_relpos2CrossNormal, body2.InternalGetTurnVelocity());

				deltaImpulse -= deltaVel1Dotn * c.m_jacDiagABInv;
				deltaImpulse -= deltaVel2Dotn * c.m_jacDiagABInv;
				float sum = c.m_appliedPushImpulse + deltaImpulse;
				if (sum < c.m_lowerLimit)
				{
					deltaImpulse = c.m_lowerLimit - c.m_appliedPushImpulse;
					c.m_appliedPushImpulse = c.m_lowerLimit;
				}
				else
				{
					c.m_appliedPushImpulse = sum;
				}
				body1.InternalApplyPushImpulse(c.m_contactNormal * body1.InternalGetInvMass(), c.m_angularComponentA, deltaImpulse);
				body2.InternalApplyPushImpulse(-c.m_contactNormal * body2.InternalGetInvMass(), c.m_angularComponentB, deltaImpulse);
			}
		}


		public virtual float SolveGroup(ObjectArray<CollisionObject> bodies, int numBodies, PersistentManifoldArray manifoldPtr, int startManifold, int numManifolds, ObjectArray<TypedConstraint> constraints, int startConstraint, int numConstraints, ContactSolverInfo infoGlobal, IDebugDraw debugDrawer, IDispatcher dispatcher)
		{
			BulletGlobals.StartProfile("solveGroup");
			//you need to provide at least some bodies
			Debug.Assert(bodies.Count > 0);

            SolveGroupCacheFriendlySetup(bodies, numBodies, manifoldPtr, startManifold, numManifolds, constraints, startConstraint, numConstraints, infoGlobal, debugDrawer, dispatcher);

            SolveGroupCacheFriendlyIterations(bodies, numBodies, manifoldPtr, startManifold, numManifolds, constraints, startConstraint, numConstraints, infoGlobal, debugDrawer, dispatcher);

            SolveGroupCacheFriendlyFinish(bodies, numBodies, manifoldPtr, startManifold, numManifolds, constraints, startConstraint, numConstraints, infoGlobal, debugDrawer);

            BulletGlobals.StopProfile();
			return 0.0f;

		}

		public virtual void PrepareSolve(int numBodies, int numManifolds)
		{
		}


		public virtual void AllSolved(ContactSolverInfo info, IDebugDraw debugDrawer)
		{
		}


		public static RigidBody GetFixedBody()
		{
			if (s_fixed == null)
			{
				s_fixed = new RigidBody(0f, null, null, IndexedVector3.Zero);
                s_fixed.SetUserPointer("SICS:Fixed");
			}
			s_fixed.SetMassProps(0f, IndexedVector3.Zero);
			return s_fixed;
		}

        protected virtual void SolveGroupCacheFriendlySplitImpulseIterations(ObjectArray<CollisionObject> bodies, int numBodies, PersistentManifoldArray manifold, int startManifold, int numManifolds, ObjectArray<TypedConstraint> constraints, int startConstraint, int numConstraints, ContactSolverInfo infoGlobal, IDebugDraw debugDrawer)
		{
			if (infoGlobal.m_splitImpulse)
			{
				for (int iteration = 0; iteration < infoGlobal.m_numIterations; iteration++)
				{
					{
						int numPoolConstraints = m_tmpSolverContactConstraintPool.Count;
						for (int j = 0; j < numPoolConstraints; j++)
						{
							SolverConstraint solveManifold = m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[j]];
							ResolveSplitPenetrationImpulseCacheFriendly(solveManifold.m_solverBodyA, solveManifold.m_solverBodyB, ref solveManifold);
							m_tmpSolverContactConstraintPool[m_orderTmpConstraintPool[j]] = solveManifold;
						}
					}
				}
			}
		}

		protected virtual float SolveGroupCacheFriendlyFinish(ObjectArray<CollisionObject> bodies, int numBodies, PersistentManifoldArray manifold, int startManifold, int numManifolds, ObjectArray<TypedConstraint> constraints, int startConstraint, int numConstraints, ContactSolverInfo infoGlobal, IDebugDraw debugDrawer)
		{
            m_finishCount++;
			int numPoolConstraints = m_tmpSolverContactConstraintPool.Count;

#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugSolver)
            {
                BulletGlobals.g_streamWriter.WriteLine("SolveGroupCacheFriendlyFinish start [{0}].",numPoolConstraints);
            }
#endif

			for (int j = 0; j < numPoolConstraints; j++)
			{

				SolverConstraint solveManifold = m_tmpSolverContactConstraintPool[j];
                ((ManifoldPoint)solveManifold.m_originalContactPoint).SetAppliedImpulse(solveManifold.m_appliedImpulse);
				if ((infoGlobal.m_solverMode & SolverMode.SOLVER_USE_FRICTION_WARMSTARTING) != 0)
				{
                    ((ManifoldPoint)solveManifold.m_originalContactPoint).SetAppliedImpulseLateral1(m_tmpSolverContactFrictionConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse);
                    ((ManifoldPoint)solveManifold.m_originalContactPoint).SetAppliedImpulseLateral2(m_tmpSolverContactFrictionConstraintPool[solveManifold.m_frictionIndex + 1].m_appliedImpulse);
				}

				//do a callback here?
			}

			numPoolConstraints = m_tmpSolverNonContactConstraintPool.Count;
			for (int j = 0; j < numPoolConstraints; j++)
			{
				SolverConstraint solverConstr = m_tmpSolverNonContactConstraintPool[j];
				TypedConstraint constr = solverConstr.m_originalContactPoint as TypedConstraint;
				constr.InternalSetAppliedImpulse(solverConstr.m_appliedImpulse);
				if (Math.Abs(solverConstr.m_appliedImpulse) >= constr.GetBreakingImpulseThreshold())
				{
					constr.SetEnabled(false);
				}
				m_tmpSolverNonContactConstraintPool[j] = solverConstr;

			}

			if (infoGlobal.m_splitImpulse)
			{
				for (int i = 0; i < numBodies; i++)
				{
					RigidBody rb = RigidBody.Upcast(bodies[i]);
					if (rb != null)
					{
						rb.InternalWritebackVelocity(infoGlobal.m_timeStep);
					}
				}
			}
			else
			{
				for (int i = 0; i < numBodies; i++)
				{
					RigidBody rb = RigidBody.Upcast(bodies[i]);
					if (rb != null)
					{
						rb.InternalWritebackVelocity();
					}
				}
			}
			m_tmpSolverContactConstraintPool.Resize(0);
            m_tmpSolverNonContactConstraintPool.Resize(0);
            m_tmpSolverContactFrictionConstraintPool.Resize(0);
            m_tmpConstraintInfo2Pool.Resize(0);
#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugSolver)
            {
                BulletGlobals.g_streamWriter.WriteLine("SolveGroupCacheFriendlyFinish stop.");
            }
#endif
			return 0f;
		}

        protected virtual float SolveGroupCacheFriendlySetup(ObjectArray<CollisionObject> bodies, int numBodies, PersistentManifoldArray manifold, int startManifold, int numManifolds, ObjectArray<TypedConstraint> constraints, int startConstraint, int numConstraints, ContactSolverInfo infoGlobal, IDebugDraw debugDrawer, IDispatcher dispatcher)
		{
            m_setupCount++;
			BulletGlobals.StartProfile("solveGroupCacheFriendlySetup");

            m_maxOverrideNumSolverIterations = 0;
            
            m_counter++;

#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugSolver)
            {
                BulletGlobals.g_streamWriter.WriteLine("SolveGroupCacheFriendlySetup start [{0}].", m_counter);
            }
#endif

			if ((numConstraints + numManifolds) == 0)
			{
				//		printf("empty\n");
                BulletGlobals.StopProfile();
                return 0f;
			}

			IndexedVector3 zero = IndexedVector3.Zero;

			if (infoGlobal.m_splitImpulse)
			{
				for (int i = 0; i < numBodies; i++)
				{
                    //RigidBody body = RigidBody.Upcast(bodies[i]);
                    RigidBody body = bodies[i] as RigidBody;
					if (body != null)
					{	
						body.InternalSetDeltaLinearVelocity(ref zero);
						body.InternalSetDeltaAngularVelocity(ref zero);
						body.InternalSetPushVelocity(ref zero);
						body.InternalSetTurnVelocity(ref zero);
					}
				}
			}
			else
			{
				for (int i = 0; i < numBodies; i++)
				{
                    RigidBody body = bodies[i] as RigidBody;
                    if (body != null)
					{	
						body.InternalSetDeltaLinearVelocity(ref zero);
						body.InternalSetDeltaAngularVelocity(ref zero);
					}
				}
			}

            if (true)
            {
                int j;
                int lastConstraint = startConstraint + numConstraints;
                for (j = startConstraint; j < lastConstraint; j++)
                {
                    TypedConstraint constraint = constraints[j];
                    //constraint.BuildJacobian();
                    constraint.InternalSetAppliedImpulse(0.0f);
                }
            }

			//if (1)
			{
				{

					int totalNumRows = 0;
					//calculate the total number of contraint rows
                    m_tmpConstraintSizesPool.Resize(numConstraints);
                    for (int i = 0; i < numConstraints; i++)
					{
                        ConstraintInfo1 info1 = m_tmpConstraintSizesPool[i];
						if (constraints[startConstraint + i].IsEnabled())
						{
							constraints[startConstraint + i].GetInfo1(info1);
						}
						else
						{
							info1.m_numConstraintRows = 0;
							info1.nub = 0;
						}

						totalNumRows += info1.m_numConstraintRows;
					}

                    m_tmpSolverNonContactConstraintPool.Resize(totalNumRows);

					///setup the btSolverConstraints
					int currentRow = 0;

                    for (int i = 0; i < numConstraints; i++)
					{
						ConstraintInfo1 info1a = m_tmpConstraintSizesPool[i];

						if (info1a.m_numConstraintRows != 0)
                        {
                            Debug.Assert(currentRow < totalNumRows);

                            TypedConstraint constraint = constraints[startConstraint + i];

                            RigidBody rbA = constraint.GetRigidBodyA();
                            RigidBody rbB = constraint.GetRigidBodyB();

                            int overrideNumSolverIterations = constraint.GetOverrideNumSolverIterations() > 0 ? constraint.GetOverrideNumSolverIterations() : infoGlobal.m_numIterations;
                            {

                                if (overrideNumSolverIterations > m_maxOverrideNumSolverIterations)
                                {
                                    m_maxOverrideNumSolverIterations = overrideNumSolverIterations;
                                }
                            }

                            for (int j = 0; j < info1a.m_numConstraintRows; j++)
                            {
                                int index = currentRow + j;
                                SolverConstraint solverConstraint = m_tmpSolverNonContactConstraintPool[index];
                                solverConstraint.Reset();
                                solverConstraint.m_lowerLimit = -MathUtil.SIMD_INFINITY;
                                solverConstraint.m_upperLimit = MathUtil.SIMD_INFINITY;
                                solverConstraint.m_appliedImpulse = 0f;
                                solverConstraint.m_appliedPushImpulse = 0f;
                                solverConstraint.m_solverBodyA = rbA;
                                solverConstraint.m_solverBodyB = rbB;
                                solverConstraint.m_overrideNumSolverIterations = overrideNumSolverIterations;
							}

                            rbA.InternalSetDeltaLinearVelocity(ref zero);
                            rbA.InternalSetDeltaAngularVelocity(ref zero);
                            rbB.InternalSetDeltaLinearVelocity(ref zero);
                            rbB.InternalSetDeltaAngularVelocity(ref zero);
                            ConstraintInfo2 info2 = m_tmpConstraintInfo2Pool[m_tmpConstraintInfo2Pool.Count];


                            Debug.Assert(info1a.m_numConstraintRows <= ConstraintInfo2.maxConstraints);
                            info2.m_numRows = info1a.m_numConstraintRows;
							// MAN - copy the data into the info block for passing to the constraints
							for (int j = 0; j < info1a.m_numConstraintRows; ++j)
							{
								info2.m_solverConstraints[j] = m_tmpSolverNonContactConstraintPool[currentRow + j];
							}

                            info2.fps = 1f / infoGlobal.m_timeStep;
                            info2.erp = infoGlobal.m_erp;
                            info2.m_numIterations = infoGlobal.m_numIterations;
                            info2.m_damping = infoGlobal.m_damping;
                            constraint.GetInfo2(info2);


                            //FIXME - log the output of the solverconstraints for comparison.

                            for (int j = 0; j < (info1a.m_numConstraintRows); j++)
                            {
                                SolverConstraint solverConstraint = m_tmpSolverNonContactConstraintPool[currentRow + j];


                                if (solverConstraint.m_upperLimit >= constraint.GetBreakingImpulseThreshold())
                                {
                                    solverConstraint.m_upperLimit = constraint.GetBreakingImpulseThreshold();
                                }

                                if (solverConstraint.m_lowerLimit <= -constraint.GetBreakingImpulseThreshold())
                                {
                                    solverConstraint.m_lowerLimit = -constraint.GetBreakingImpulseThreshold();
                                }



                                solverConstraint.m_originalContactPoint = constraint;

                                {
                                    IndexedVector3 ftorqueAxis1 = solverConstraint.m_relpos1CrossNormal;
                                    solverConstraint.m_angularComponentA = constraint.GetRigidBodyA().GetInvInertiaTensorWorld() * ftorqueAxis1 * constraint.GetRigidBodyA().GetAngularFactor();
                                }
                                {
                                    IndexedVector3 ftorqueAxis2 = solverConstraint.m_relpos2CrossNormal;
                                    solverConstraint.m_angularComponentB = constraint.GetRigidBodyB().GetInvInertiaTensorWorld() * ftorqueAxis2 * constraint.GetRigidBodyB().GetAngularFactor();
                                }

                                {
                                    IndexedVector3 iMJlA = solverConstraint.m_contactNormal * rbA.GetInvMass();
                                    IndexedVector3 iMJaA = rbA.GetInvInertiaTensorWorld() * solverConstraint.m_relpos1CrossNormal;
                                    IndexedVector3 iMJlB = solverConstraint.m_contactNormal * rbB.GetInvMass();//sign of normal?
                                    IndexedVector3 iMJaB = rbB.GetInvInertiaTensorWorld() * solverConstraint.m_relpos2CrossNormal;

                                    float sum = IndexedVector3.Dot(ref iMJlA, ref solverConstraint.m_contactNormal);
                                    float a = IndexedVector3.Dot(ref iMJaA, ref solverConstraint.m_relpos1CrossNormal);
                                    float b = IndexedVector3.Dot(ref iMJlB, ref solverConstraint.m_contactNormal);
                                    float c = IndexedVector3.Dot(ref iMJaB, ref solverConstraint.m_relpos2CrossNormal);
									sum += a;
									sum += b;
									sum += c;

                                    solverConstraint.m_jacDiagABInv = 1f / sum;

                                    MathUtil.SanityCheckFloat(solverConstraint.m_jacDiagABInv);
                                }


                                ///fix rhs
                                ///todo: add force/torque accelerators
                                {
                                    float rel_vel;
                                    float vel1Dotn = IndexedVector3.Dot(solverConstraint.m_contactNormal, rbA.GetLinearVelocity()) + IndexedVector3.Dot(solverConstraint.m_relpos1CrossNormal, rbA.GetAngularVelocity());
                                    float vel2Dotn = -IndexedVector3.Dot(solverConstraint.m_contactNormal, rbB.GetLinearVelocity()) + IndexedVector3.Dot(solverConstraint.m_relpos2CrossNormal, rbB.GetAngularVelocity());

                                    rel_vel = vel1Dotn + vel2Dotn;

                                    float restitution = 0f;
                                    float positionalError = solverConstraint.m_rhs;//already filled in by getConstraintInfo2
                                    float velocityError = restitution - rel_vel * info2.m_damping;
                                    float penetrationImpulse = positionalError * solverConstraint.m_jacDiagABInv;
                                    float velocityImpulse = velocityError * solverConstraint.m_jacDiagABInv;
                                    solverConstraint.m_rhs = penetrationImpulse + velocityImpulse;
                                    solverConstraint.m_appliedImpulse = 0f;

                                }
#if DEBUG                                
                                if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugSolver)
                                {
                                    TypedConstraint.PrintSolverConstraint(BulletGlobals.g_streamWriter, solverConstraint, j);
                                }
#endif
                                //m_tmpSolverNonContactConstraintPool[currentRow + j] = solverConstraint;
							}


                            //if(BulletGlobals.g_streamWriter != null && BulletGlobals.debugSolver)
                            //{
                            //    TypedConstraint.PrintInfo2(BulletGlobals.g_streamWriter,constraint,info2);
                            //}

                        }
						currentRow += m_tmpConstraintSizesPool[i].m_numConstraintRows;
					}
				}

				{
                    int lastManifold = startManifold + numManifolds;
                    for (int i = startManifold; i < lastManifold; i++)
					{
                        ConvertContact(manifold[i], infoGlobal);
					}
				}
			}

			ContactSolverInfo info = infoGlobal;

            int numNonContactPool = m_tmpSolverNonContactConstraintPool.Count;
			int numConstraintPool = m_tmpSolverContactConstraintPool.Count;
			int numFrictionPool = m_tmpSolverContactFrictionConstraintPool.Count;

			///@todo: use stack allocator for such temporarily memory, same for solver bodies/constraints
            m_orderNonContactConstraintPool.EnsureCapacity(numNonContactPool);///
            m_orderTmpConstraintPool.EnsureCapacity(numConstraintPool);
            m_orderFrictionConstraintPool.EnsureCapacity(numFrictionPool);

			{
                for (int i = 0; i < numNonContactPool; i++)
                {
                    m_orderNonContactConstraintPool[i] = i;
                }
				for (int i = 0; i < numConstraintPool; i++)
				{
					m_orderTmpConstraintPool[i] = i;
				}
				for (int i = 0; i < numFrictionPool; i++)
				{
                    m_orderFrictionConstraintPool[i] = i;
				}
			}

            BulletGlobals.StopProfile();

#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugSolver)
            {
                BulletGlobals.g_streamWriter.WriteLine("SolveGroupCacheFriendlySetup stop.");
            }
#endif
			return 0f;


		}

		//protected virtual float solveGroupCacheFriendlyIterations()
		protected float SolveGroupCacheFriendlyIterations(ObjectArray<CollisionObject> bodies, int numBodies, PersistentManifoldArray manifoldPtr, int startManifold, int numManifolds, ObjectArray<TypedConstraint> constraints, int startConstraint, int numConstraints, ContactSolverInfo infoGlobal, IDebugDraw debugDrawer, IDispatcher dispatcher)
		{
            m_iterCount++;
            BulletGlobals.StartProfile("solveGroupCacheFriendlyIterations");
#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugSolver)
            {
                BulletGlobals.g_streamWriter.WriteLine("SolveGroupCacheFriendlyIterations start.");
            }
#endif

            {
		        ///this is a special step to resolve penetrations (just for contacts)
		        SolveGroupCacheFriendlySplitImpulseIterations(bodies ,numBodies,manifoldPtr,startManifold,numManifolds,constraints,startConstraint,numConstraints,infoGlobal,debugDrawer);

		        int maxIterations = m_maxOverrideNumSolverIterations > infoGlobal.m_numIterations? m_maxOverrideNumSolverIterations : infoGlobal.m_numIterations;

		        //for ( int iteration = maxIterations-1  ; iteration >= 0;iteration--)
                for (int iteration = 0; iteration < maxIterations; iteration++)
                {
			        SolveSingleIteration(iteration, bodies, numBodies, manifoldPtr,startManifold,numManifolds, constraints, startConstraint, numConstraints, infoGlobal, debugDrawer);
		        }
		
	        }
#if DEBUG	        
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugSolver)
            {
                BulletGlobals.g_streamWriter.WriteLine("SolveGroupCacheFriendlyIterations stop.");
            }
#endif
            BulletGlobals.StopProfile();
			return 0.0f;
		}

        protected float SolveSingleIteration(int iteration, ObjectArray<CollisionObject> bodies, int numBodies, PersistentManifoldArray manifold, int startManifold, int numManifolds, ObjectArray<TypedConstraint> constraints, int startConstraint, int numConstraints, ContactSolverInfo infoGlobal, IDebugDraw debugDrawer)
		{
            int numNonContactPool = m_tmpSolverNonContactConstraintPool.Count;
			int numConstraintPool = m_tmpSolverContactConstraintPool.Count;
			int numFrictionPool = m_tmpSolverContactFrictionConstraintPool.Count;

#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugSolver)
            {
                BulletGlobals.g_streamWriter.WriteLine( "solveSingleIter [{0}][{1}][{2}].", numNonContactPool, numConstraintPool, numFrictionPool);
	        }
#endif

			//should traverse the contacts random order...
			if (TestSolverMode(infoGlobal.m_solverMode, SolverMode.SOLVER_RANDMIZE_ORDER))
			{
                if ((iteration & 7) == 0)
                {
                    for (int j = 0; j < numNonContactPool; ++j)
                    {
                        int tmp = m_orderNonContactConstraintPool[j];
                        int swapi = RandInt2(j + 1);
                        m_orderNonContactConstraintPool[j] = m_orderNonContactConstraintPool[swapi];
                        m_orderNonContactConstraintPool[swapi] = tmp;
                    }

                    //contact/friction constraints are not solved more than 
                    if (iteration < infoGlobal.m_numIterations)
                    {
                        for (int j = 0; j < numConstraintPool; ++j)
                        {
                            int tmp = m_orderTmpConstraintPool[j];
                            int swapi = RandInt2(j + 1);
                            m_orderTmpConstraintPool[j] = m_orderTmpConstraintPool[swapi];
                            m_orderTmpConstraintPool[swapi] = tmp;
                        }

                        for (int j = 0; j < numFrictionPool; ++j)
                        {
                            int tmp = m_orderFrictionConstraintPool[j];
                            int swapi = RandInt2(j + 1);
                            m_orderFrictionConstraintPool[j] = m_orderFrictionConstraintPool[swapi];
                            m_orderFrictionConstraintPool[swapi] = tmp;
                        }
                    }
                }
			}

			SolverConstraint[] rawTmpSolverNonContactConstraintPool = m_tmpSolverNonContactConstraintPool.GetRawArray();
            int[] rawOrderNonContactConstraintPool = m_orderNonContactConstraintPool.GetRawArray();
			///solve all joint constraints
		    for (int j=0;j<m_tmpSolverNonContactConstraintPool.Count;j++)
		    {
                SolverConstraint constraint = rawTmpSolverNonContactConstraintPool[rawOrderNonContactConstraintPool[j]];
                if (iteration < constraint.m_overrideNumSolverIterations)
                {
				    ResolveSingleConstraintRowGeneric(constraint.m_solverBodyA,constraint.m_solverBodyB,ref constraint);
                }
		    }

		    if (iteration< infoGlobal.m_numIterations)
		    {
                SolverConstraint[] rawTmpSolverContactConstraintPool = m_tmpSolverContactConstraintPool.GetRawArray();
                int[] rawOrderTmpConstraintPool = m_orderTmpConstraintPool.GetRawArray();

			    ///solve all contact constraints
			    int numPoolConstraints = m_tmpSolverContactConstraintPool.Count;
			    for (int j=0;j<numPoolConstraints;j++)
			    {
                    SolverConstraint solveManifold = rawTmpSolverContactConstraintPool[rawOrderTmpConstraintPool[j]];
				    ResolveSingleConstraintRowLowerLimit(solveManifold.m_solverBodyA,solveManifold.m_solverBodyB,ref solveManifold);
			    }
			    ///solve all friction constraints
			    int numFrictionPoolConstraints = m_tmpSolverContactFrictionConstraintPool.Count;
                SolverConstraint[] rawTmpSolverContactFrictionConstraintPool = m_tmpSolverContactFrictionConstraintPool.GetRawArray();
                int[] rawOrderFrictionConstraintPool = m_orderFrictionConstraintPool.GetRawArray();
                for (int j = 0; j < numFrictionPoolConstraints; j++)
			    {
				    SolverConstraint solveManifold = rawTmpSolverContactFrictionConstraintPool[rawOrderFrictionConstraintPool[j]];
				    float totalImpulse = m_tmpSolverContactConstraintPool[solveManifold.m_frictionIndex].m_appliedImpulse;

				    if (totalImpulse>0f)
				    {
					    solveManifold.m_lowerLimit = -(solveManifold.m_friction*totalImpulse);
					    solveManifold.m_upperLimit = solveManifold.m_friction*totalImpulse;

					    ResolveSingleConstraintRowGeneric(solveManifold.m_solverBodyA,solveManifold.m_solverBodyB,ref solveManifold);
				    }
			    }
            }
			return 0f;
		}

		///clear internal cached data and reset random seed
		public virtual void Reset()
		{
			m_btSeed2 = 0;
		}

		public ulong Rand2()
		{
			m_btSeed2 = (1664525L * m_btSeed2 + 1013904223L) & 0xffffffff;
			return m_btSeed2;
		}

		//See ODE: adam's all-int straightforward(?) dRandInt (0..n-1)
		public int RandInt2(int n)
		{
			// seems good; xor-fold and modulus
			ulong un = (ulong)(n);
			ulong r = Rand2();

			// note: probably more aggressive than it needs to be -- might be
			//       able to get away without one or two of the innermost branches.
			if (un <= 0x00010000UL)
			{
				r ^= (r >> 16);
				if (un <= 0x00000100UL)
				{
					r ^= (r >> 8);
					if (un <= 0x00000010UL)
					{
						r ^= (r >> 4);
						if (un <= 0x00000004UL)
						{
							r ^= (r >> 2);
							if (un <= 0x00000002UL)
							{
								r ^= (r >> 1);
							}
						}
					}
				}
			}

            int result = (int)(r % un);
#if DEBUG            
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugSolver)
            {
                BulletGlobals.g_streamWriter.WriteLine("random[{0}][{1}].",n,result);
	        }
#endif
            return result;
		}

		public void SetRandSeed(ulong seed)
		{
			m_btSeed2 = seed;
		}

		public ulong GetRandSeed()
		{
			return m_btSeed2;
		}

		private static void ApplyAnisotropicFriction(CollisionObject colObj, ref IndexedVector3 frictionDirection)
		{
			if (colObj != null && colObj.HasAnisotropicFriction())
			{
				// transform to local coordinates
                IndexedVector3 loc_lateral = frictionDirection * colObj.GetWorldTransform()._basis;
				IndexedVector3 friction_scaling = colObj.GetAnisotropicFriction();
				//apply anisotropic friction
				loc_lateral *= friction_scaling;
				// ... and transform it back to global coordinates
                frictionDirection = colObj.GetWorldTransform()._basis * loc_lateral;
			}
		}

		public static bool TestSolverMode(SolverMode val1, SolverMode val2)
		{
			return (val1 & val2) == val2;
		}
	}
}
