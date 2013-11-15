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

using System.Diagnostics;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletDynamics
{
    public class JacobianEntry
    {

	public JacobianEntry() {}
	//constraint between two different rigidbodies
    public JacobianEntry(IndexedBasisMatrix world2A,
        IndexedBasisMatrix world2B,
        IndexedVector3 rel_pos1,
        IndexedVector3 rel_pos2,
        IndexedVector3 jointAxis,
        IndexedVector3 inertiaInvA,
        float massInvA,
        IndexedVector3 inertiaInvB,
        float massInvB) : this(ref world2A,ref world2B,ref rel_pos1,ref rel_pos2,
                    ref jointAxis, ref inertiaInvA,massInvA,ref inertiaInvB,massInvB)
    {
  
    }
        
        public JacobianEntry(
        ref IndexedBasisMatrix world2A,
        ref IndexedBasisMatrix world2B,
		ref IndexedVector3 rel_pos1,ref IndexedVector3 rel_pos2,
		ref IndexedVector3 jointAxis,
		ref IndexedVector3 inertiaInvA, 
		float massInvA,
		ref IndexedVector3 inertiaInvB,
		float massInvB)
	    {
            m_linearJointAxis = jointAxis;
            m_aJ = world2A * (rel_pos1.Cross(ref m_linearJointAxis));
            m_bJ = world2B * (rel_pos2.Cross(-m_linearJointAxis));
            m_0MinvJt = inertiaInvA * m_aJ;
		    m_1MinvJt = inertiaInvB * m_bJ;
		    m_Adiag = massInvA + IndexedVector3.Dot(m_0MinvJt,m_aJ) + massInvB + IndexedVector3.Dot(m_1MinvJt,m_bJ);

		    Debug.Assert(m_Adiag > 0.0f);
	    }

	//angular constraint between two different rigidbodies

        public JacobianEntry(IndexedVector3 jointAxis,
        IndexedBasisMatrix world2A,
        IndexedBasisMatrix world2B,
        IndexedVector3 inertiaInvA,
        IndexedVector3 inertiaInvB) : this(ref jointAxis,ref world2A,ref world2B,ref inertiaInvA,ref inertiaInvB)
        {

        }
        public JacobianEntry(ref IndexedVector3 jointAxis,
        ref IndexedBasisMatrix world2A,
        ref IndexedBasisMatrix world2B,
		ref IndexedVector3 inertiaInvA,
		ref IndexedVector3 inertiaInvB)
	    {
            m_linearJointAxis = IndexedVector3.Zero;
            m_aJ = world2A * jointAxis;
            m_bJ = world2B * -jointAxis;
            m_0MinvJt = inertiaInvA * m_aJ;
		    m_1MinvJt = inertiaInvB * m_bJ;
		    m_Adiag =  IndexedVector3.Dot(m_0MinvJt,m_aJ) + IndexedVector3.Dot(m_1MinvJt,m_bJ);

		    Debug.Assert(m_Adiag > 0.0f);
	    }

        public JacobianEntry(IndexedVector3 axisInA,
            IndexedVector3 axisInB,
            IndexedVector3 inertiaInvA,
            IndexedVector3 inertiaInvB) : this(ref axisInA,ref axisInB,ref inertiaInvA,ref inertiaInvB)
        {

        }
	    //angular constraint between two different rigidbodies
	    public JacobianEntry(ref IndexedVector3 axisInA,
		    ref IndexedVector3 axisInB,
		    ref IndexedVector3 inertiaInvA,
		    ref IndexedVector3 inertiaInvB)
	    {
            m_linearJointAxis = IndexedVector3.Zero;
            m_aJ = axisInA;
            m_bJ = -axisInB;
		    m_0MinvJt	= inertiaInvA * m_aJ;
		    m_1MinvJt = inertiaInvB * m_bJ;
		    m_Adiag =  IndexedVector3.Dot(m_0MinvJt,m_aJ) + IndexedVector3.Dot(m_1MinvJt,m_bJ);

		    Debug.Assert(m_Adiag > 0.0f);
	    }

	//constraint on one rigidbody
        public JacobianEntry(
            IndexedBasisMatrix world2A,
            IndexedVector3 rel_pos1, IndexedVector3 rel_pos2,
            IndexedVector3 jointAxis,
            IndexedVector3 inertiaInvA,
            float massInvA) : this(ref world2A,ref rel_pos1,ref rel_pos2,ref jointAxis,ref inertiaInvA,massInvA)
        {
        }

        public JacobianEntry(
        ref IndexedBasisMatrix world2A,
		ref IndexedVector3 rel_pos1,ref IndexedVector3 rel_pos2,
		ref IndexedVector3 jointAxis,
		ref IndexedVector3 inertiaInvA, 
		float massInvA)
	{
        m_linearJointAxis = jointAxis;
        m_aJ = world2A * (rel_pos1.Cross(ref jointAxis));
        m_bJ = world2A * (rel_pos2.Cross(-jointAxis));
        m_0MinvJt = inertiaInvA * m_aJ;
		m_1MinvJt = IndexedVector3.Zero;
		m_Adiag = massInvA + IndexedVector3.Dot(m_0MinvJt,m_aJ);

		Debug.Assert(m_Adiag > 0.0f);
	}

	public float GetDiagonal() 
    { 
        return m_Adiag; 
    }

	// for two constraints on the same rigidbody (for example vehicle friction)
	public float GetNonDiagonal(JacobianEntry jacB, float massInvA)
	{
		JacobianEntry jacA = this;
		float lin = massInvA * IndexedVector3.Dot(jacA.m_linearJointAxis,jacB.m_linearJointAxis);
		float ang = IndexedVector3.Dot(jacA.m_0MinvJt,jacB.m_aJ);
		return lin + ang;
	}

	// for two constraints on sharing two same rigidbodies (for example two contact points between two rigidbodies)
	public float GetNonDiagonal(JacobianEntry jacB,float massInvA,float massInvB)
	{
		JacobianEntry jacA = this;
		IndexedVector3 lin = jacA.m_linearJointAxis * jacB.m_linearJointAxis;
		IndexedVector3 ang0 = jacA.m_0MinvJt * jacB.m_aJ;
		IndexedVector3 ang1 = jacA.m_1MinvJt * jacB.m_bJ;
		IndexedVector3 lin0 = massInvA * lin ;
		IndexedVector3 lin1 = massInvB * lin;
		IndexedVector3 sum = ang0+ang1+lin0+lin1;
		return sum.X+sum.Y+sum.Z;
	}

    public float GetRelativeVelocity(IndexedVector3 linvelA, IndexedVector3 angvelA, IndexedVector3 linvelB, IndexedVector3 angvelB)
    {
        return GetRelativeVelocity(ref linvelA, ref angvelA, ref linvelB, ref angvelB);
    }
	public float GetRelativeVelocity(ref IndexedVector3 linvelA,ref IndexedVector3 angvelA,ref IndexedVector3 linvelB,ref IndexedVector3 angvelB)
	{
		IndexedVector3 linrel = linvelA - linvelB;
		IndexedVector3 angvela  = angvelA * m_aJ;
		IndexedVector3 angvelb  = angvelB * m_bJ;
		linrel *= m_linearJointAxis;
		angvela += angvelb;
		angvela += linrel;
		float rel_vel2 = angvela.X+angvela.Y+angvela.Z;
		return rel_vel2 + MathUtil.SIMD_EPSILON;
	}

	    public IndexedVector3	m_linearJointAxis;
        public IndexedVector3 m_aJ;
        public IndexedVector3 m_bJ;
        public IndexedVector3 m_0MinvJt;
        public IndexedVector3 m_1MinvJt;
	    //Optimization: can be stored in the w/last component of one of the vectors
        public float m_Adiag;
    }
}
