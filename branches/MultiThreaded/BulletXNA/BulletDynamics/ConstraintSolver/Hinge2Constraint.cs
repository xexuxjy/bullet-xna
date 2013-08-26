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
    // Constraint similar to ODE Hinge2 Joint
    // has 3 degrees of frredom:
    // 2 rotational degrees of freedom, similar to Euler rotations around Z (axis 1) and X (axis 2)
    // 1 translational (along axis Z) with suspension spring
    public class Hinge2Constraint : Generic6DofSpringConstraint
    {
        // constructor
    	// anchor, axis1 and axis2 are in world coordinate system
	    // axis1 must be orthogonal to axis2
        public Hinge2Constraint(RigidBody rbA, RigidBody rbB, ref IndexedVector3 anchor, ref IndexedVector3 axis1, ref IndexedVector3 axis2) : base(rbA,rbB,IndexedMatrix.Identity,IndexedMatrix.Identity,true)
        {
            m_anchor = anchor;
            m_axis1 = axis1;
            m_axis2 = axis2;
            // build frame basis
            // 6DOF constraint uses Euler angles and to define limits
            // it is assumed that rotational order is :
            // Z - first, allowed limits are (-PI,PI);
            // new position of Y - second (allowed limits are (-PI/2 + epsilon, PI/2 - epsilon), where epsilon is a small positive number 
            // used to prevent constraint from instability on poles;
            // new position of X, allowed limits are (-PI,PI);
            // So to simulate ODE Universal joint we should use parent axis as Z, child axis as Y and limit all other DOFs
            // Build the frame in world coordinate system first
            IndexedVector3 zAxis = IndexedVector3.Normalize(axis1);
            IndexedVector3 xAxis = IndexedVector3.Normalize(axis2);
            IndexedVector3 yAxis = IndexedVector3.Cross(zAxis,xAxis); // we want right coordinate system

            IndexedMatrix frameInW = IndexedMatrix.Identity;
            frameInW._basis = new IndexedBasisMatrix(xAxis.X, yAxis.X, zAxis.X,
                                    xAxis.Y, yAxis.Y, zAxis.Y,
                                   xAxis.Z, yAxis.Z, zAxis.Z);
            frameInW._origin = anchor;

            // now get constraint frame in local coordinate systems
            m_frameInA = rbA.GetCenterOfMassTransform().Inverse() * frameInW;
            m_frameInB = rbB.GetCenterOfMassTransform().Inverse() * frameInW;
            // sei limits
            SetLinearLowerLimit(new IndexedVector3(0.0f, 0.0f, -1.0f));
            SetLinearUpperLimit(new IndexedVector3(0.0f, 0.0f, 1.0f));
            // like front wheels of a car
            SetAngularLowerLimit(new IndexedVector3(1.0f, 0.0f, -MathUtil.SIMD_HALF_PI * 0.5f));
            SetAngularUpperLimit(new IndexedVector3(-1.0f, 0.0f, MathUtil.SIMD_HALF_PI * 0.5f));
            // enable suspension
            EnableSpring(2, true);
            SetStiffness(2, MathUtil.SIMD_PI * MathUtil.SIMD_PI * 4.0f); // period 1 sec for 1 kilogramm weel :-)
            SetDamping(2, 0.01f);
            SetEquilibriumPoint();

        }
	    // access
	    public IndexedVector3 GetAnchor() { return m_calculatedTransformA._origin; }
	    public IndexedVector3 GetAnchor2() { return m_calculatedTransformB._origin; }
	    public IndexedVector3 GetAxis1() { return m_axis1; }
	    public IndexedVector3 GetAxis2() { return m_axis2; }
	    public float GetAngle1() { return GetAngle(2); }
        public float GetAngle2() { return GetAngle(0); }
	    // limits
        public void SetUpperLimit(float ang1max) { SetAngularUpperLimit(new IndexedVector3(-1.0f, 0.0f, ang1max)); }
        public void SetLowerLimit(float ang1min) { SetAngularLowerLimit(new IndexedVector3(1.0f, 0.0f, ang1min)); }
    
        protected IndexedVector3	m_anchor;
	    protected IndexedVector3	m_axis1;
	    protected IndexedVector3	m_axis2;

    }
}
