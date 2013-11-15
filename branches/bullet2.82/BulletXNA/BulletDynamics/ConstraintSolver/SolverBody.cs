///*
// * C# / XNA  port of Bullet (c) 2011 Mark Neale <xexuxjy@hotmail.com>
// *
// * Bullet Continuous Collision Detection and Physics Library
// * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
// *
// * This software is provided 'as-is', without any express or implied warranty.
// * In no event will the authors be held liable for any damages arising from
// * the use of this software.
// * 
// * Permission is granted to anyone to use this software for any purpose, 
// * including commercial applications, and to alter it and redistribute it
// * freely, subject to the following restrictions:
// * 
// * 1. The origin of this software must not be misrepresented; you must not
// *    claim that you wrote the original software. If you use this software
// *    in a product, an acknowledgment in the product documentation would be
// *    appreciated but is not required.
// * 2. Altered source versions must be plainly marked as such, and must not be
// *    misrepresented as being the original software.
// * 3. This notice may not be removed or altered from any source distribution.
// */

//using System;
//using System.Collections.Generic;
//
//using BulletXNA.BulletDynamics.Dynamics;

//namespace BulletXNA.BulletDynamics.ConstraintSolver
//{

/////Until we get other contributions, only use SIMD on Windows, when using Visual Studio 2008 or later, and not double precision
////#ifdef BT_USE_SSE
////#define USE_SIMD 1
////#endif //


////#ifdef USE_SIMD

////struct	btSimdScalar
////{
////    SIMD_FORCE_INLINE	btSimdScalar()
////    {

////    }

////    SIMD_FORCE_INLINE	btSimdScalar(float	fl)
////    :m_vec128 (_mm_set1_ps(fl))
////    {
////    }

////    SIMD_FORCE_INLINE	btSimdScalar(__m128 v128)
////        :m_vec128(v128)
////    {
////    }
////    union
////    {
////        __m128		m_vec128;
////        float		m_floats[4];
////        int			m_ints[4];
////        float	m_unusedPadding;
////    };
////    SIMD_FORCE_INLINE	__m128	get128()
////    {
////        return m_vec128;
////    }

////    SIMD_FORCE_INLINE	const __m128	get128() const
////    {
////        return m_vec128;
////    }

////    SIMD_FORCE_INLINE	void	set128(__m128 v128)
////    {
////        m_vec128 = v128;
////    }

////    SIMD_FORCE_INLINE	operator       __m128()       
////    { 
////        return m_vec128; 
////    }
////    SIMD_FORCE_INLINE	operator const __m128() const 
////    { 
////        return m_vec128; 
////    }
	
////    SIMD_FORCE_INLINE	operator float() const 
////    { 
////        return m_floats[0]; 
////    }

////};

///////@brief Return the elementwise product of two btSimdScalar
////SIMD_FORCE_INLINE btSimdScalar 
////operator*(const btSimdScalar& v1, const btSimdScalar& v2) 
////{
////    return btSimdScalar(_mm_mul_ps(v1.get128(),v2.get128()));
////}

///////@brief Return the elementwise product of two btSimdScalar
////SIMD_FORCE_INLINE btSimdScalar 
////operator+(const btSimdScalar& v1, const btSimdScalar& v2) 
////{
////    return btSimdScalar(_mm_add_ps(v1.get128(),v2.get128()));
////}


////#else
////#define btSimdScalar float
////#endif

/////The btSolverBody is an internal datastructure for the constraint solver. Only necessary data is packed to increase cache coherence/performance.
//    public class SolverBodyObsolete
//    {
//        public IndexedVector3 m_deltaLinearVelocity;
//        public IndexedVector3 m_deltaAngularVelocity;
//        public IndexedVector3 m_angularFactor;
//        public IndexedVector3 m_invMass;
//        public RigidBody	m_originalBody;
//        public IndexedVector3		m_pushVelocity;
//        public IndexedVector3		m_turnVelocity;	


//        public IndexedVector3 getDeltaLinearVelocity()
//        {
//            return m_deltaLinearVelocity;
//        }

//        public void setDeltaLinearVelocity(IndexedVector3 value)
//        {
//            m_deltaLinearVelocity = value;
//            MathUtil.sanityCheckVector(m_deltaLinearVelocity);
//        }

//        public void	getVelocityInLocalPointObsolete(ref IndexedVector3 rel_pos, ref IndexedVector3 velocity )
//        {
//            if (m_originalBody != null)
//            {
//                velocity = m_originalBody.getLinearVelocity()+m_deltaLinearVelocity + (IndexedVector3.Cross((m_originalBody.getAngularVelocity()+m_deltaAngularVelocity),rel_pos));
//            }
//            else
//            {
//                velocity = IndexedVector3.Zero;
//            }
//        }

//        public void	getAngularVelocity(ref IndexedVector3 angVel)
//        {
//            if (m_originalBody != null)
//            {
//                angVel = m_originalBody.getAngularVelocity()+m_deltaAngularVelocity;
//            }
//            else
//            {
//                angVel = IndexedVector3.Zero;
//            }
//        }

//        //Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position
//        public void applyImpulse(IndexedVector3 linearComponent, IndexedVector3 angularComponent, float impulseMagnitude)
//        {
//            applyImpulse(ref linearComponent, ref angularComponent, impulseMagnitude);
//        }

//        public void applyImpulse(ref IndexedVector3 linearComponent, ref IndexedVector3 angularComponent,float impulseMagnitude)
//        {
//            //if (m_invMass)
//            {
//                m_deltaLinearVelocity += linearComponent*impulseMagnitude;
//                m_deltaAngularVelocity += angularComponent*(impulseMagnitude*m_angularFactor);
//                MathUtil.sanityCheckVector(m_deltaLinearVelocity);
//                MathUtil.sanityCheckVector(m_deltaAngularVelocity);
//            }
//        }

//        public void internalApplyPushImpulse(ref IndexedVector3 linearComponent, ref IndexedVector3 angularComponent,float impulseMagnitude)
//        {
//            if (m_originalBody != null)
//            {
//                m_pushVelocity += linearComponent*impulseMagnitude;
//                m_turnVelocity += angularComponent*(impulseMagnitude*m_angularFactor);
//            }
//        }
	
//        public void	writebackVelocity()
//        {
//            if (m_originalBody != null)
//            {
//                m_originalBody.setLinearVelocity(m_originalBody.getLinearVelocity()+ m_deltaLinearVelocity);
//                m_originalBody.setAngularVelocity(m_originalBody.getAngularVelocity()+m_deltaAngularVelocity);
//                //m_originalBody.setCompanionId(-1);
//            }
//        }


//        public void	writebackVelocity(float timeStep)
//        {
//            if (m_originalBody != null)
//            {
//                m_originalBody.setLinearVelocity(m_originalBody.getLinearVelocity()+ m_deltaLinearVelocity);
//                m_originalBody.setAngularVelocity(m_originalBody.getAngularVelocity()+m_deltaAngularVelocity);
    			
//                //correct the position/orientation based on push/turn recovery
//                IndexedMatrix newTransform = IndexedMatrix.Identity;
//                IndexedMatrix orig = m_originalBody.GetWorldTransform();
//                TransformUtil.integrateTransform(ref orig,ref m_pushVelocity,ref m_turnVelocity,timeStep,ref newTransform);
//                m_originalBody.SetWorldTransform(ref newTransform);
    			
//                //m_originalBody.setCompanionId(-1);
//            }
//        }
//    }
//}
