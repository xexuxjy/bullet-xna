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
    public JacobianEntry(Matrix world2A,
        Matrix world2B,
        Vector3 rel_pos1,
        Vector3 rel_pos2,
        Vector3 jointAxis,
        Vector3 inertiaInvA,
        float massInvA,
        Vector3 inertiaInvB,
        float massInvB) : this(ref world2A,ref world2B,ref rel_pos1,ref rel_pos2,
                    ref jointAxis, ref inertiaInvA,massInvA,ref inertiaInvB,massInvB)
    {
  
    }
        
        public JacobianEntry(
		ref Matrix world2A,
		ref Matrix world2B,
		ref Vector3 rel_pos1,ref Vector3 rel_pos2,
		ref Vector3 jointAxis,
		ref Vector3 inertiaInvA, 
		float massInvA,
		ref Vector3 inertiaInvB,
		float massInvB)
	    {
            m_linearJointAxis = jointAxis;
		    m_aJ = Vector3.TransformNormal(Vector3.Cross(rel_pos1,m_linearJointAxis),world2A);
            m_bJ = Vector3.TransformNormal(Vector3.Cross(rel_pos2,-m_linearJointAxis),world2B);
		    m_0MinvJt	= inertiaInvA * m_aJ;
		    m_1MinvJt = inertiaInvB * m_bJ;
		    m_Adiag = massInvA + Vector3.Dot(m_0MinvJt,m_aJ) + massInvB + Vector3.Dot(m_1MinvJt,m_bJ);

		    Debug.Assert(m_Adiag > 0.0f);
	    }

	//angular constraint between two different rigidbodies

        public JacobianEntry(Vector3 jointAxis,
        Matrix world2A,
        Matrix world2B,
        Vector3 inertiaInvA,
        Vector3 inertiaInvB) : this(ref jointAxis,ref world2A,ref world2B,ref inertiaInvA,ref inertiaInvB)
        {

        }
        public JacobianEntry(ref Vector3 jointAxis,
		ref Matrix world2A,
		ref Matrix world2B,
		ref Vector3 inertiaInvA,
		ref Vector3 inertiaInvB)
	    {
            m_linearJointAxis = Vector3.Zero;
		    m_aJ= Vector3.TransformNormal(jointAxis,world2A);
		    m_bJ = Vector3.TransformNormal(-jointAxis,world2B);
		    m_0MinvJt	= inertiaInvA * m_aJ;
		    m_1MinvJt = inertiaInvB * m_bJ;
		    m_Adiag =  Vector3.Dot(m_0MinvJt,m_aJ) + Vector3.Dot(m_1MinvJt,m_bJ);

		    Debug.Assert(m_Adiag > 0.0f);
	    }

        public JacobianEntry(Vector3 axisInA,
            Vector3 axisInB,
            Vector3 inertiaInvA,
            Vector3 inertiaInvB) : this(ref axisInA,ref axisInB,ref inertiaInvA,ref inertiaInvB)
        {

        }
	    //angular constraint between two different rigidbodies
	    public JacobianEntry(ref Vector3 axisInA,
		    ref Vector3 axisInB,
		    ref Vector3 inertiaInvA,
		    ref Vector3 inertiaInvB)
	    {
            m_linearJointAxis = Vector3.Zero;
            m_aJ = axisInA;
            m_bJ = -axisInB;
		    m_0MinvJt	= inertiaInvA * m_aJ;
		    m_1MinvJt = inertiaInvB * m_bJ;
		    m_Adiag =  Vector3.Dot(m_0MinvJt,m_aJ) + Vector3.Dot(m_1MinvJt,m_bJ);

		    Debug.Assert(m_Adiag > 0.0f);
	    }

	//constraint on one rigidbody
        public JacobianEntry(
            Matrix world2A,
            Vector3 rel_pos1, Vector3 rel_pos2,
            Vector3 jointAxis,
            Vector3 inertiaInvA,
            float massInvA) : this(ref world2A,ref rel_pos1,ref rel_pos2,ref jointAxis,ref inertiaInvA,massInvA)
        {
        }

        public JacobianEntry(
		ref Matrix world2A,
		ref Vector3 rel_pos1,ref Vector3 rel_pos2,
		ref Vector3 jointAxis,
		ref Vector3 inertiaInvA, 
		float massInvA)
	{
        m_linearJointAxis = jointAxis;
        m_aJ = Vector3.TransformNormal(Vector3.Cross(rel_pos1,jointAxis), world2A);
        m_bJ = Vector3.TransformNormal(Vector3.Cross(rel_pos2,-jointAxis), world2A);
		m_0MinvJt	= inertiaInvA * m_aJ;
		m_1MinvJt = Vector3.Zero;
		m_Adiag = massInvA + Vector3.Dot(m_0MinvJt,m_aJ);

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
		float lin = massInvA * Vector3.Dot(jacA.m_linearJointAxis,jacB.m_linearJointAxis);
		float ang = Vector3.Dot(jacA.m_0MinvJt,jacB.m_aJ);
		return lin + ang;
	}

	// for two constraints on sharing two same rigidbodies (for example two contact points between two rigidbodies)
	public float GetNonDiagonal(JacobianEntry jacB,float massInvA,float massInvB)
	{
		JacobianEntry jacA = this;
		Vector3 lin = jacA.m_linearJointAxis * jacB.m_linearJointAxis;
		Vector3 ang0 = jacA.m_0MinvJt * jacB.m_aJ;
		Vector3 ang1 = jacA.m_1MinvJt * jacB.m_bJ;
		Vector3 lin0 = massInvA * lin ;
		Vector3 lin1 = massInvB * lin;
		Vector3 sum = ang0+ang1+lin0+lin1;
		return sum.X+sum.Y+sum.Z;
	}

    public float GetRelativeVelocity(Vector3 linvelA, Vector3 angvelA, Vector3 linvelB, Vector3 angvelB)
    {
        return GetRelativeVelocity(ref linvelA, ref angvelA, ref linvelB, ref angvelB);
    }
	public float GetRelativeVelocity(ref Vector3 linvelA,ref Vector3 angvelA,ref Vector3 linvelB,ref Vector3 angvelB)
	{
		Vector3 linrel = linvelA - linvelB;
		Vector3 angvela  = angvelA * m_aJ;
		Vector3 angvelb  = angvelB * m_bJ;
		linrel *= m_linearJointAxis;
		angvela += angvelb;
		angvela += linrel;
		float rel_vel2 = angvela.X+angvela.Y+angvela.Z;
		return rel_vel2 + MathUtil.SIMD_EPSILON;
	}

	    public Vector3	m_linearJointAxis;
        public Vector3 m_aJ;
        public Vector3 m_bJ;
        public Vector3 m_0MinvJt;
        public Vector3 m_1MinvJt;
	    //Optimization: can be stored in the w/last component of one of the vectors
        public float m_Adiag;
    }
}
