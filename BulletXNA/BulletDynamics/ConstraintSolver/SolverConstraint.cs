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

using System.IO;
using BulletXNA.BulletCollision;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletDynamics
{

    public class SolverConstraint
    {
	    public IndexedVector3	m_relpos1CrossNormal;
        public IndexedVector3 m_contactNormal;
        public IndexedVector3 m_relpos2CrossNormal;
        public IndexedVector3 m_angularComponentA;
        public IndexedVector3 m_angularComponentB;
        public float m_appliedPushImpulse;
        public float m_appliedImpulse;
        
        public float m_friction;
        public float m_jacDiagABInv;
        public int m_numConsecutiveRowsPerKernel;
        public int m_frictionIndex;
		// warning - these are unioned in the c++ version
        public RigidBody m_solverBodyA;
		public int m_companionIdA;
		// warning - these are unioned in the c++ version
		public RigidBody m_solverBodyB;
		public int m_companionIdB;
		
        //public TypedConstraint m_originalContactPointConstraint;
        //public ManifoldPoint m_originalContactPoint;
        public object m_originalContactPoint;
        public float m_rhs;
        public float m_cfm;
        public float m_lowerLimit;
        public float m_upperLimit;
        public float m_rhsPenetration;
        public int m_overrideNumSolverIterations = -1;

        public SolverConstraint()
        {
            int i = 0;
        }


        public void Reset()
        {
            m_relpos1CrossNormal = IndexedVector3.Zero;
            m_contactNormal = IndexedVector3.Zero;
            m_relpos2CrossNormal = IndexedVector3.Zero;
            m_angularComponentA = IndexedVector3.Zero;
            m_angularComponentB = IndexedVector3.Zero;
            m_appliedPushImpulse = 0f;
            m_appliedImpulse = 0f;
            m_friction = 0f;
            m_jacDiagABInv = 0f;
            m_numConsecutiveRowsPerKernel = 0;
            m_frictionIndex = 0;
            m_solverBodyA = null;
            m_companionIdA = 0;
            m_solverBodyB = null;
            m_companionIdB = 0;
            m_originalContactPoint = null;
            //m_originalContactPointConstraint = null;
            m_rhs = 0f;
            m_cfm = 0;
            m_lowerLimit = 0f;
            m_upperLimit = 0f;
            m_rhsPenetration = 0f;
            m_overrideNumSolverIterations = -1;

        }


        public void PrintSolverConstraint(TextWriter tw)
        {
            if (tw != null)
            {
                MathUtil.PrintVector3(tw,"SC:RP1CN",m_relpos1CrossNormal);
                MathUtil.PrintVector3(tw,"SC:RP2CN",m_relpos2CrossNormal);
                MathUtil.PrintVector3(tw,"SC:CN",m_contactNormal);
                MathUtil.PrintVector3(tw,"SC:ANGA",m_angularComponentA);
                MathUtil.PrintVector3(tw,"SC:ANGB",m_angularComponentB);
                tw.WriteLine("SC:AppPush[{0}] AppImp[{1}] Fric[{2}] Jac[{3}] comA[{4}] comB[{5}]",m_appliedPushImpulse,m_appliedImpulse,m_friction,m_jacDiagABInv,m_companionIdA,m_companionIdB);
            }
        }


    }

    public enum SolverConstraintType
    {
        BT_SOLVER_CONTACT_1D = 0,
        BT_SOLVER_FRICTION_1D
    }

}
