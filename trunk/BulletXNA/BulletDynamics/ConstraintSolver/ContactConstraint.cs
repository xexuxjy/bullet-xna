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

using Microsoft.Xna.Framework;
using System.Diagnostics;
using BulletXNA.BullettDynamics.Dynamics;
using BulletXNA.BulletCollision.NarrowPhaseCollision;

namespace BulletXNA.BulletDynamics.ConstraintSolver
{

    public class ContactConstraint : TypedConstraint
    {

        public ContactConstraint(PersistentManifold contactManifold, RigidBody rbA, RigidBody rbB) : base(TypedConstraintType.CONTACT_CONSTRAINT_TYPE,rbA,rbB)
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



        ///bilateral constraint between two dynamic objects
        ///positive distance = separation, negative distance = penetration
        public static void ResolveSingleBilateral(RigidBody body1, ref Vector3 pos1,
                              RigidBody body2, ref Vector3 pos2,
                              float distance, ref Vector3 normal, ref float impulse, float timeStep)
        {
	        float normalLenSqr = normal.LengthSquared();
	        Debug.Assert(Math.Abs(normalLenSqr) < 1.1f);
	        if (normalLenSqr > 1.1f)
	        {
		        impulse = 0f;
		        return;
	        }
	        Vector3 rel_pos1 = pos1 - body1.GetCenterOfMassPosition(); 
	        Vector3 rel_pos2 = pos2 - body2.GetCenterOfMassPosition();
	        //this jacobian entry could be re-used for all iterations
        	
	        Vector3 vel1 = body1.GetVelocityInLocalPoint(ref rel_pos1);
	        Vector3 vel2 = body2.GetVelocityInLocalPoint(ref rel_pos2);
	        Vector3 vel = vel1 - vel2;

            Matrix m1 = MathUtil.TransposeBasis(body1.GetCenterOfMassTransform());
            Matrix m2 = MathUtil.TransposeBasis(body2.GetCenterOfMassTransform());


            JacobianEntry jac = new JacobianEntry(m1,m2,rel_pos1,rel_pos2,normal,
                body1.GetInvInertiaDiagLocal(),body1.GetInvMass(),
		        body2.GetInvInertiaDiagLocal(),body2.GetInvMass());

	        float jacDiagAB = jac.GetDiagonal();
	        float jacDiagABInv = 1f / jacDiagAB;

            
            float rel_vel = jac.GetRelativeVelocity(
                body1.GetLinearVelocity(),Vector3.TransformNormal(body1.GetAngularVelocity(),m1),
                body2.GetLinearVelocity(),Vector3.TransformNormal(body2.GetAngularVelocity(),m2));
	        float a = jacDiagABInv;

            rel_vel = Vector3.Dot(normal,vel);
        	
	        //todo: move this into proper structure
	        float contactDamping = 0.2f;

            if(ONLY_USE_LINEAR_MASS)
            {
	            float massTerm = 1f / (body1.GetInvMass() + body2.GetInvMass());
	            impulse = - contactDamping * rel_vel * massTerm;
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
        float ContactSolverFunc(RigidBody body1,RigidBody body2,ManifoldPoint contactPoint,
									 ContactSolverInfo info);
    }
}
