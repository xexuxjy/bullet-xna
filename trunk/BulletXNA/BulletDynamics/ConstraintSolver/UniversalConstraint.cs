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

using BulletXNA.BulletDynamics.Dynamics;
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletDynamics.ConstraintSolver
{
    public class UniversalConstraint : Generic6DofConstraint
    {
    

        /// Constraint similar to ODE Universal Joint
        /// has 2 rotatioonal degrees of freedom, similar to Euler rotations around Z (axis 1)
        /// and Y (axis 2)
        /// Description from ODE manual : 
        /// "Given axis 1 on body 1, and axis 2 on body 2 that is perpendicular to axis 1, it keeps them perpendicular. 
        /// In other words, rotation of the two bodies about the direction perpendicular to the two axes will be equal."

        private const float UNIV_EPS = 0.01f;

	    protected Vector3 m_anchor;
	    protected Vector3 m_axis1;
	    protected Vector3 m_axis2;

        // constructor
	    // anchor, axis1 and axis2 are in world coordinate system
	    // axis1 must be orthogonal to axis2
        public UniversalConstraint(RigidBody rbA, RigidBody rbB, ref Vector3 anchor, ref Vector3 axis1, ref Vector3 axis2)
            : base(rbA, rbB, ref BulletGlobals.IdentityMatrix, ref BulletGlobals.IdentityMatrix, true)
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
	        Vector3 zAxis = Vector3.Normalize(axis1);
	        Vector3 yAxis = Vector3.Normalize(axis2);
	        Vector3 xAxis = Vector3.Cross(yAxis,zAxis); // we want right coordinate system
	        Matrix frameInW = Matrix.Identity;
            MathUtil.SetBasis(ref frameInW,ref xAxis,ref yAxis,ref zAxis);
	        frameInW.Translation = anchor;
	        // now get constraint frame in local coordinate systems
			//m_frameInA = MathUtil.inverseTimes(rbA.getCenterOfMassTransform(),frameInW);
			//m_frameInB = MathUtil.inverseTimes(rbB.getCenterOfMassTransform(),frameInW);
			m_frameInA = MathUtil.BulletMatrixMultiply(Matrix.Invert(rbA.GetCenterOfMassTransform()), frameInW);
			m_frameInB = MathUtil.BulletMatrixMultiply(Matrix.Invert(rbB.GetCenterOfMassTransform()), frameInW);

	        // sei limits
	        SetLinearLowerLimit(Vector3.Zero);
	        SetLinearUpperLimit(Vector3.Zero);
	        SetAngularLowerLimit(new Vector3(0.0f, -MathUtil.SIMD_HALF_PI + UNIV_EPS, -MathUtil.SIMD_PI + UNIV_EPS));
	        SetAngularUpperLimit(new Vector3(0.0f,  MathUtil.SIMD_HALF_PI - UNIV_EPS,  MathUtil.SIMD_PI - UNIV_EPS));
        }

	    // access
	    public Vector3 GetAnchor() { return m_calculatedTransformA.Translation; }
	    public Vector3 GetAnchor2() { return m_calculatedTransformB.Translation; }
	    public Vector3 GetAxis1() { return m_axis1; }
	    public Vector3 GetAxis2() { return m_axis2; }
	    public float GetAngle1() { return GetAngle(2); }
	    public float GetAngle2() { return GetAngle(1); }
	    // limits
	    public void SetUpperLimit(float ang1max, float ang2max) { SetAngularUpperLimit(new Vector3(0.0f, ang1max, ang2max)); }
	    public void SetLowerLimit(float ang1min, float ang2min) { SetAngularLowerLimit(new Vector3(0.0f, ang1min, ang2min)); }

		public override void SetAxis(ref Vector3 axis1, ref Vector3 axis2)
		{
			Vector3 zAxis = Vector3.Normalize(axis1);
			Vector3 yAxis = Vector3.Normalize(axis2);
			Vector3 xAxis = Vector3.Cross(yAxis, zAxis); // we want right coordinate system

			Matrix frameInW = Matrix.Identity;
			MathUtil.SetBasis(ref frameInW, ref xAxis, ref yAxis, ref zAxis);

			// now get constraint frame in local coordinate systems
			m_frameInA = MathUtil.InverseTimes(m_rbA.GetCenterOfMassTransform(), frameInW);
			m_frameInB = MathUtil.InverseTimes(m_rbB.GetCenterOfMassTransform(), frameInW);

			CalculateTransforms();
		}


    }
}
