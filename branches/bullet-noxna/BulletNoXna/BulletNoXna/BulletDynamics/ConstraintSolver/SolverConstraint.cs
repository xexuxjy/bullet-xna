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
using BulletXNA.LinearMath;

namespace BulletXNA.BulletDynamics
{

    public struct SolverConstraint
    {
	    public Vector3	m_relpos1CrossNormal;
        public Vector3 m_contactNormal;
        public Vector3 m_relpos2CrossNormal;
        public Vector3 m_angularComponentA;
        public Vector3 m_angularComponentB;
        public float m_appliedPushImpulse;
        public float m_appliedImpulse;
        
        public float m_friction;
        public float m_jacDiagABInv;
        public int m_numConsecutiveRowsPerKernel;
        public int m_frictionIndex;
        public RigidBody m_solverBodyA;
        public RigidBody m_solverBodyB;
        public Object m_originalContactPoint;

        public float m_rhs;
        public float m_cfm;
        public float m_lowerLimit;
        public float m_upperLimit;
        public float m_rhsPenetration;
    }

    public enum SolverConstraintType
    {
        BT_SOLVER_CONTACT_1D = 0,
        BT_SOLVER_FRICTION_1D
    }

}
