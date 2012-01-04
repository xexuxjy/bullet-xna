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

namespace BulletXNA.BulletDynamics
{

	public class ContactConstraint : TypedConstraint
	{

		public ContactConstraint(PersistentManifold contactManifold, RigidBody rbA, RigidBody rbB)
			: base(TypedConstraintType.CONTACT_CONSTRAINT_TYPE, rbA, rbB)
		{
			m_contactManifold = contactManifold;
		}


		public PersistentManifold GetContactManifold()
		{
			return m_contactManifold;
		}

		public void SetContactManifold(PersistentManifold contactManifold)
		{
			m_contactManifold = contactManifold;
		}

		public override void GetInfo1(ConstraintInfo1 info)
		{
		}

		public override void GetInfo2(ConstraintInfo2 info)
		{
		}


		//response  between two dynamic objects without friction, assuming 0 penetration depth
		public static float ResolveSingleCollision(
				RigidBody body1,
				CollisionObject colObj2,
				ref IndexedVector3 contactPositionWorld,
				ref IndexedVector3 contactNormalOnB,
				ContactSolverInfo solverInfo,
				float distance)
		{
			RigidBody body2 = RigidBody.Upcast(colObj2);


			IndexedVector3 normal = contactNormalOnB;

			IndexedVector3 rel_pos1 = contactPositionWorld - body1.GetWorldTransform()._origin;
			IndexedVector3 rel_pos2 = contactPositionWorld - colObj2.GetWorldTransform()._origin;

			IndexedVector3 vel1 = body1.GetVelocityInLocalPoint(ref rel_pos1);
			IndexedVector3 vel2 = body2 != null ? body2.GetVelocityInLocalPoint(ref rel_pos2) : IndexedVector3.Zero;
			IndexedVector3 vel = vel1 - vel2;
			float rel_vel = normal.Dot(ref vel);

			float combinedRestitution = body1.GetRestitution() * colObj2.GetRestitution();
			float restitution = combinedRestitution * -rel_vel;

			float positionalError = solverInfo.m_erp * -distance / solverInfo.m_timeStep;
			float velocityError = -(1.0f + restitution) * rel_vel;// * damping;
			float denom0 = body1.ComputeImpulseDenominator(ref contactPositionWorld, ref normal);
			float denom1 = body2 != null ? body2.ComputeImpulseDenominator(ref contactPositionWorld, ref normal) : 0.0f;
			float relaxation = 1.0f;
			float jacDiagABInv = relaxation / (denom0 + denom1);

			float penetrationImpulse = positionalError * jacDiagABInv;
			float velocityImpulse = velocityError * jacDiagABInv;

			float normalImpulse = penetrationImpulse + velocityImpulse;
			normalImpulse = 0.0f > normalImpulse ? 0.0f : normalImpulse;

			body1.ApplyImpulse(normal * (normalImpulse), rel_pos1);
			if (body2 != null)
			{
				body2.ApplyImpulse(-normal * (normalImpulse), rel_pos2);
			}

			return normalImpulse;
		}



		///bilateral constraint between two dynamic objects
		///positive distance = separation, negative distance = penetration
		public static void ResolveSingleBilateral(RigidBody body1, ref IndexedVector3 pos1,
							  RigidBody body2, ref IndexedVector3 pos2,
							  float distance, ref IndexedVector3 normal, ref float impulse, float timeStep)
		{
			float normalLenSqr = normal.LengthSquared();
			Debug.Assert(Math.Abs(normalLenSqr) < 1.1f);
			if (normalLenSqr > 1.1f)
			{
				impulse = 0f;
				return;
			}
			IndexedVector3 rel_pos1 = pos1 - body1.GetCenterOfMassPosition();
			IndexedVector3 rel_pos2 = pos2 - body2.GetCenterOfMassPosition();
			//this jacobian entry could be re-used for all iterations

			IndexedVector3 vel1 = body1.GetVelocityInLocalPoint(ref rel_pos1);
			IndexedVector3 vel2 = body2.GetVelocityInLocalPoint(ref rel_pos2);
			IndexedVector3 vel = vel1 - vel2;

            IndexedBasisMatrix m1 = body1.GetCenterOfMassTransform()._basis.Transpose();
            IndexedBasisMatrix m2 = body2.GetCenterOfMassTransform()._basis.Transpose();


			JacobianEntry jac = new JacobianEntry(m1, m2, rel_pos1, rel_pos2, normal,
				body1.GetInvInertiaDiagLocal(), body1.GetInvMass(),
				body2.GetInvInertiaDiagLocal(), body2.GetInvMass());

			float jacDiagAB = jac.GetDiagonal();
			float jacDiagABInv = 1f / jacDiagAB;


			float rel_vel = jac.GetRelativeVelocity(
				body1.GetLinearVelocity(),
                body1.GetCenterOfMassTransform()._basis.Transpose() * body1.GetAngularVelocity(),
                body2.GetLinearVelocity(),
                body2.GetCenterOfMassTransform()._basis.Transpose() * body2.GetAngularVelocity());
            float a = jacDiagABInv;

			rel_vel = normal.Dot(ref vel);

			//todo: move this into proper structure
			float contactDamping = 0.2f;

			if (ONLY_USE_LINEAR_MASS)
			{
				float massTerm = 1f / (body1.GetInvMass() + body2.GetInvMass());
				impulse = -contactDamping * rel_vel * massTerm;
			}
			else
			{
				float velocityImpulse = -contactDamping * rel_vel * jacDiagABInv;
				impulse = velocityImpulse;
			}
		}



		protected PersistentManifold m_contactManifold;
		public const bool ONLY_USE_LINEAR_MASS = false;
		public const bool USE_INTERNAL_APPLY_IMPULSE = true;

	}


	public interface IContactSolverFunc
	{
		float ContactSolverFunc(RigidBody body1, RigidBody body2, ManifoldPoint contactPoint,
									 ContactSolverInfo info);
	}
}
