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

using BulletXNA.LinearMath;

namespace BulletXNA.BulletDynamics
{
    public class Point2PointConstraint : TypedConstraint
    {
	    public JacobianEntry[] m_jac = new JacobianEntry[3]; //3 orthogonal linear constraints
	    public IndexedVector3 m_pivotInA;
	    public IndexedVector3 m_pivotInB;
        public Point2PointFlags m_flags = 0;
        public float m_erp;
        public float m_cfm;

	    public ConstraintSetting m_setting = new ConstraintSetting();


        public Point2PointConstraint(RigidBody rbA, RigidBody rbB, ref IndexedVector3 pivotInA, ref IndexedVector3 pivotInB)
            : base(TypedConstraintType.POINT2POINT_CONSTRAINT_TYPE,rbA,rbB)
        {
            m_pivotInA = pivotInA;
            m_pivotInB = pivotInB;
        }
	    public Point2PointConstraint(RigidBody rbA,ref IndexedVector3 pivotInA) : base(TypedConstraintType.POINT2POINT_CONSTRAINT_TYPE,rbA)
        {
            m_pivotInA = pivotInA;
            m_pivotInB = rbA.GetCenterOfMassTransform() * pivotInA;
        }

        public override void GetInfo1(ConstraintInfo1 info)
        {
            GetInfo1NonVirtual(info);
        }
        public void GetInfo1NonVirtual(ConstraintInfo1 info)
        {
            info.m_numConstraintRows = 3;
            info.nub = 3;
        }

        public override void GetInfo2(ConstraintInfo2 info)
        {
            GetInfo2NonVirtual(info, m_rbA.GetCenterOfMassTransform(), m_rbB.GetCenterOfMassTransform());
        }

        public void GetInfo2NonVirtual(ConstraintInfo2 info,IndexedMatrix body0_trans,IndexedMatrix body1_trans)
        {
            // anchor points in global coordinates with respect to body PORs.

            // set jacobian
            info.m_solverConstraints[0].m_contactNormal.X = 1;
            info.m_solverConstraints[1].m_contactNormal.Y = 1;
            info.m_solverConstraints[2].m_contactNormal.Z = 1;

            IndexedVector3 a1 = body0_trans._basis * GetPivotInA();
            {
                IndexedVector3 a1neg = -a1;

                MathUtil.GetSkewSymmetricMatrix(ref a1neg,
                    out info.m_solverConstraints[0].m_relpos1CrossNormal,
                    out info.m_solverConstraints[1].m_relpos1CrossNormal,
                    out info.m_solverConstraints[2].m_relpos1CrossNormal);
            }

            /*info->m_J2linearAxis[0] = -1;
            info->m_J2linearAxis[s+1] = -1;
            info->m_J2linearAxis[2*s+2] = -1;
            */

            IndexedVector3 a2 = body1_trans._basis * GetPivotInB();

            {
                IndexedVector3 a2n = -a2;

                MathUtil.GetSkewSymmetricMatrix(ref a2,
                    out info.m_solverConstraints[0].m_relpos2CrossNormal,
                    out info.m_solverConstraints[1].m_relpos2CrossNormal, 
                    out info.m_solverConstraints[2].m_relpos2CrossNormal);
            }

            // set right hand side
            float currERP = ((m_flags & Point2PointFlags.BT_P2P_FLAGS_ERP) != 0) ? m_erp : info.erp;
            float k = info.fps * currERP;
            int j;
            IndexedVector3 body0Origin = body0_trans._origin;
            IndexedVector3 body1Origin = body1_trans._origin;

            for (j = 0; j < 3; j++)
            {
                info.m_solverConstraints[j].m_rhs = k * (a2[j] + body1Origin[j] - a1[j] - body0Origin[j]);
                //printf("info->m_constraintError[%d]=%f\n",j,info->m_constraintError[j]);
            }

            if ((m_flags & Point2PointFlags.BT_P2P_FLAGS_CFM) != 0)
            {
                for (j = 0; j < 3; j++)
                {
                    info.m_solverConstraints[j].m_cfm = m_cfm;
                }
            }


            float impulseClamp = m_setting.m_impulseClamp;//
            for (j = 0; j < 3; j++)
            {
                if (m_setting.m_impulseClamp > 0)
                {
                    info.m_solverConstraints[j].m_lowerLimit = -impulseClamp;
                    info.m_solverConstraints[j].m_upperLimit = impulseClamp;
                }
            }
			info.m_damping = m_setting.m_damping;
        }


        public void UpdateRHS(float timeStep)
        {
        }

	    public void SetPivotA(ref IndexedVector3 pivotA)
	    {
		    m_pivotInA = pivotA;
	    }

	    public void SetPivotB(ref IndexedVector3 pivotB)
	    {
		    m_pivotInB = pivotB;
	    }

	    public IndexedVector3 GetPivotInA()
	    {
		    return m_pivotInA;
	    }

	    public IndexedVector3 GetPivotInB() 
	    {
		    return m_pivotInB;
	    }



    }


    public class ConstraintSetting
    {
	    public ConstraintSetting()
	    {
            m_tau = 0.3f;
            m_damping = 1f;
            m_impulseClamp = 0f;
	    }
	    public float m_tau;
	    public float m_damping;
	    public float m_impulseClamp;
    }

    public enum Point2PointFlags
    {
        BT_P2P_FLAGS_ERP = 1,
        BT_P2P_FLAGS_CFM = 2
    };


}
