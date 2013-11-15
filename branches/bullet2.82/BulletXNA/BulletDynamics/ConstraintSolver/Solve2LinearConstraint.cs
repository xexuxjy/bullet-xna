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
using BulletXNA.LinearMath;

namespace BulletXNA.BulletDynamics
{
    /// raint class used for lateral tyre friction.
    public class Solve2LinearConstraint
    {
	    private float m_tau;
	    private float m_damping;

        public Solve2LinearConstraint(float tau, float damping)
	    {
		    m_tau = tau;
		    m_damping = damping;
	    }
	    //
	    // solve unilateral raint (equality, direct method)
	    //
        public void ResolveUnilateralPairConstraint(RigidBody body0, RigidBody body1, ref IndexedBasisMatrix world2A,
                            ref IndexedBasisMatrix world2B,
                            ref IndexedVector3 invInertiaADiag,
                            float invMassA,
                            ref IndexedVector3 linvelA, ref IndexedVector3 angvelA,
                            ref IndexedVector3 rel_posA1,
                            ref IndexedVector3 invInertiaBDiag,
                            float invMassB,
                            ref IndexedVector3 linvelB, ref IndexedVector3 angvelB,
                            ref IndexedVector3 rel_posA2,
                            float depthA, ref IndexedVector3 normalA,
                            ref IndexedVector3 rel_posB1, ref IndexedVector3 rel_posB2,
                            float depthB, ref IndexedVector3 normalB,
                            out float imp0, out float imp1)
        {
            //(void)linvelA;
            //(void)linvelB;
            //(void)angvelB;
            //(void)angvelA;

	        imp0 = 0f;
	        imp1 = 0f;

	        float len = Math.Abs(normalA.Length()) - 1f;
	        if (Math.Abs(len) >= MathUtil.SIMD_EPSILON)
		        return;

	        Debug.Assert(len < MathUtil.SIMD_EPSILON);

	        //this jacobian entry could be re-used for all iterations
	        JacobianEntry jacA = new JacobianEntry(ref world2A,ref world2B,ref rel_posA1,ref rel_posA2,ref normalA,ref invInertiaADiag,invMassA,
		        ref invInertiaBDiag,invMassB);
	        JacobianEntry jacB = new JacobianEntry(ref world2A,ref world2B,ref rel_posB1,ref rel_posB2,ref normalB,ref invInertiaADiag,invMassA,
		        ref invInertiaBDiag,invMassB);
        	
	        // float vel0 = jacA.getRelativeVelocity(linvelA,angvelA,linvelB,angvelB);
	        // float vel1 = jacB.getRelativeVelocity(linvelA,angvelA,linvelB,angvelB);

	        float vel0 = IndexedVector3.Dot(normalA,(body0.GetVelocityInLocalPoint(ref rel_posA1)-body1.GetVelocityInLocalPoint(ref rel_posA1)));
	        float vel1 = IndexedVector3.Dot(normalB,(body0.GetVelocityInLocalPoint(ref rel_posB1)-body1.GetVelocityInLocalPoint(ref rel_posB1)));

        //	float penetrationImpulse = (depth*contactTau*timeCorrection)  * massTerm;//jacDiagABInv
	        float massTerm = 1f / (invMassA + invMassB);


	        // calculate rhs (or error) terms
	        float dv0 = depthA  * m_tau * massTerm - vel0 * m_damping;
	        float dv1 = depthB  * m_tau * massTerm - vel1 * m_damping;


	        // dC/dv * dv = -C
        	
	        // jacobian * impulse = -error
	        //

	        //impulse = jacobianInverse * -error

	        // inverting 2x2 symmetric system (offdiagonal are equal!)
	        // 


	        float nonDiag = jacA.GetNonDiagonal(jacB,invMassA,invMassB);
	        float invDet = 1f / (jacA.GetDiagonal() * jacB.GetDiagonal() - nonDiag * nonDiag );
        	
	        //imp0 = dv0 * jacA.getDiagonal() * invDet + dv1 * -nonDiag * invDet;
	        //imp1 = dv1 * jacB.getDiagonal() * invDet + dv0 * - nonDiag * invDet;

	        imp0 = dv0 * jacA.GetDiagonal() * invDet + dv1 * -nonDiag * invDet;
	        imp1 = dv1 * jacB.GetDiagonal() * invDet + dv0 * - nonDiag * invDet;

	        //[a b]								  [d -c]
	        //[c d] inverse = (1 / determinant) * [-b a] where determinant is (ad - bc)

	        //[jA nD] * [imp0] = [dv0]
	        //[nD jB]   [imp1]   [dv1]
        }


	    //
	    // solving 2x2 lcp problem (inequality, direct solution )
	    //
        void ResolveBilateralPairraint(RigidBody body0, RigidBody body1, ref IndexedBasisMatrix world2A,
                            ref IndexedBasisMatrix world2B,
                            ref IndexedVector3 invInertiaADiag,
                            float invMassA,
                            ref IndexedVector3 linvelA, ref IndexedVector3 angvelA,
                            ref IndexedVector3 rel_posA1,
                            ref IndexedVector3 invInertiaBDiag,
                            float invMassB,
                            ref IndexedVector3 linvelB, ref IndexedVector3 angvelB,
                            ref IndexedVector3 rel_posA2,
                          float depthA, ref IndexedVector3 normalA,
                          ref IndexedVector3 rel_posB1, ref IndexedVector3 rel_posB2,
                          float depthB, ref IndexedVector3 normalB,
                          ref float imp0, ref float imp1)
        {
	        imp0 = 0f;
	        imp1 = 0f;

	        float len = Math.Abs(normalA.Length()) - 1f;
	        if (Math.Abs(len) >= MathUtil.SIMD_EPSILON)
		        return;

	        Debug.Assert(len < MathUtil.SIMD_EPSILON);


	        //this jacobian entry could be re-used for all iterations
	        JacobianEntry jacA = new JacobianEntry(ref world2A,ref world2B,ref rel_posA1,ref rel_posA2,ref normalA,ref invInertiaADiag,invMassA,
		        ref invInertiaBDiag,invMassB);
	        JacobianEntry jacB = new JacobianEntry(ref world2A,ref world2B,ref rel_posB1,ref rel_posB2,ref normalB,ref invInertiaADiag,invMassA,
		        ref invInertiaBDiag,invMassB);
        	
	        // float vel0 = jacA.getRelativeVelocity(linvelA,angvelA,linvelB,angvelB);
	        // float vel1 = jacB.getRelativeVelocity(linvelA,angvelA,linvelB,angvelB);

	        float vel0 = IndexedVector3.Dot(normalA,(body0.GetVelocityInLocalPoint(ref rel_posA1)-body1.GetVelocityInLocalPoint(ref rel_posA1)));
	        float vel1 = IndexedVector3.Dot(normalB,(body0.GetVelocityInLocalPoint(ref rel_posB1)-body1.GetVelocityInLocalPoint(ref rel_posB1)));

	        // calculate rhs (or error) terms
	         float dv0 = depthA  * m_tau - vel0 * m_damping;
	         float dv1 = depthB  * m_tau - vel1 * m_damping;

	        // dC/dv * dv = -C
        	
	        // jacobian * impulse = -error
	        //

	        //impulse = jacobianInverse * -error

	        // inverting 2x2 symmetric system (offdiagonal are equal!)
	        // 


	        float nonDiag = jacA.GetNonDiagonal(jacB,invMassA,invMassB);
	        float invDet = 1.0f / (jacA.GetDiagonal() * jacB.GetDiagonal() - nonDiag * nonDiag );
        	
	        //imp0 = dv0 * jacA.getDiagonal() * invDet + dv1 * -nonDiag * invDet;
	        //imp1 = dv1 * jacB.getDiagonal() * invDet + dv0 * - nonDiag * invDet;

	        imp0 = dv0 * jacA.GetDiagonal() * invDet + dv1 * -nonDiag * invDet;
	        imp1 = dv1 * jacB.GetDiagonal() * invDet + dv0 * - nonDiag * invDet;

	        //[a b]								  [d -c]
	        //[c d] inverse = (1 / determinant) * [-b a] where determinant is (ad - bc)

	        //[jA nD] * [imp0] = [dv0]
	        //[nD jB]   [imp1]   [dv1]

	        if ( imp0 > 0.0f)
	        {
		        if ( imp1 > 0.0f )
		        {
			        //both positive
		        }
		        else
		        {
			        imp1 = 0.0f;

			        // now imp0>0 imp1<0
			        imp0 = dv0 / jacA.GetDiagonal();
			        if ( imp0 > 0.0f)
			        {
			        } else
			        {
				        imp0 = 0f;
			        }
		        }
	        }
	        else
	        {
		        imp0 = 0f;

		        imp1 = dv1 / jacB.GetDiagonal();
		        if ( imp1 <= 0f )
		        {
			        imp1 = 0f;
			        // now imp0>0 imp1<0
			        imp0 = dv0 / jacA.GetDiagonal();
			        if ( imp0 > 0f)
			        {
			        } else
			        {
				        imp0 = 0f;
			        }
		        } else
		        {
		        }
	        }

        }

    /*
	    void resolveAngularraint(	ref IndexedMatrix invInertiaAWS,
						     float invMassA,
						    ref IndexedVector3 linvelA,ref IndexedVector3 angvelA,
						    ref IndexedVector3 rel_posA1,
						    ref IndexedMatrix invInertiaBWS,
						     float invMassB,
						    ref IndexedVector3 linvelB,ref IndexedVector3 angvelB,
						    ref IndexedVector3 rel_posA2,

					      float depthA, ref IndexedVector3 normalA, 
					      ref IndexedVector3 rel_posB1,ref IndexedVector3 rel_posB2,
					      float depthB, ref IndexedVector3 normalB, 
					      float& imp0,float& imp1);

    */

    };
}
