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

namespace BulletXNA.BulletCollision
{
    /// ManifoldContactPoint collects and maintains persistent contactpoints.
    /// used to improve stability and performance of rigidbody dynamics response.
    public class ManifoldPoint
    {

        public ManifoldPoint()
        {

        }


        public void Initialise(ref IndexedVector3 pointA, ref IndexedVector3 pointB, ref IndexedVector3 normal, float distance)
        {
            /* Don't initialize default values twice in C# */
            m_lateralFrictionDir1 = IndexedVector3.Zero;
            m_lateralFrictionDir2 = IndexedVector3.Zero;
            m_lifeTime = 0;
            m_appliedImpulseLateral1 = 0f;
            m_appliedImpulseLateral2 = 0f;
            m_contactMotion1 = 0f;
            m_contactMotion2 = 0f;
            m_contactCFM1 = 0f;
            m_contactCFM2 = 0f;

            m_lateralFrictionInitialized = false;
            m_userPersistentData = null;
            m_appliedImpulse = 0f;
            m_partId0 = 0;
            m_partId1 = 0;
            m_index0 = 0;
            m_index1 = 0;
            m_combinedRestitution = 0f;
            m_combinedFriction = 0f;
            m_positionWorldOnA = IndexedVector3.Zero;
            m_positionWorldOnB = IndexedVector3.Zero;

            m_localPointA = pointA;
            m_localPointB = pointB;
            m_normalWorldOnB = normal;
            m_distance1 = distance;

            m_constraintRow[0].Reset();
            m_constraintRow[1].Reset();
            m_constraintRow[2].Reset();
        }

        public float GetDistance()
        {
            return m_distance1;
        }

        public void SetDistance(float value)
        {
            m_distance1 = value;
        }

        public int GetLifeTime()
        {
            return m_lifeTime;
        }

        public void SetLifeTime(int value)
        {
            m_lifeTime = value;
        }

        public void SetPositionWorldOnA(IndexedVector3 value)
        {
            m_positionWorldOnA = value;
        }

        public void SetPositionWorldOnA(ref IndexedVector3 value)
        {
            m_positionWorldOnA = value;
        }

        public IndexedVector3 GetPositionWorldOnA()
        {
            return m_positionWorldOnA;
            //				return m_positionWorldOnB + m_normalWorldOnB * m_distance1;
        }

        public void SetPositionWorldOnB(IndexedVector3 value)
        {
            m_positionWorldOnB = value;
        }

        public void SetPositionWorldOnB(ref IndexedVector3 value)
        {
            m_positionWorldOnB = value;
        }

        public IndexedVector3 GetPositionWorldOnB()
        {
            return m_positionWorldOnB;
        }

        ///this returns the most recent applied impulse, to satisfy contact constraints by the constraint solver
        public float GetAppliedImpulse()
        {
            return m_appliedImpulse;
        }

        public IndexedVector3 GetLocalPointA()
        {
            return m_localPointA;
        }

        public void SetLocalPointA(ref IndexedVector3 value)
        {
            m_localPointA = value;
        }

        public IndexedVector3 GetLocalPointB()
        {
            return m_localPointB;
        }

        public void SetLocalPointB(ref IndexedVector3 value)
        {
            m_localPointB = value;
        }

        public Object GetUserPersistentData()
        {
            return m_userPersistentData;
        }

        public void SetUserPersistentData(Object o)
        {
            m_userPersistentData = o;
        }

        public void SetAppliedImpulse(float value)
        {
            if (value > 30)
            {
                int ibreak = 0;
            }
            m_appliedImpulse = value;
        }

        public IndexedVector3 GetNormalWorldOnB()
        {
            return m_normalWorldOnB;
        }

        public void SetNormalWorldOnB(ref IndexedVector3 value)
        {
            m_normalWorldOnB = value;
        }

        public float GetCombinedFriction()
        {
            return m_combinedFriction;
        }

        public void SetCombinedFriction(float value)
        {
            m_combinedFriction = value;
        }

        public float GetCombinedResitution()
        {
            return m_combinedRestitution;
        }

        public void SetCombinedRestitution(float value)
        {
            m_combinedRestitution = value;
        }

        public bool GetLateralFrictionInitialized()
        {
            return m_lateralFrictionInitialized;
        }

        public void SetLateralFrictionInitalised(bool value)
        {
            m_lateralFrictionInitialized = value;
        }

        public float GetAppliedImpulseLateral1()
        {
            return m_appliedImpulseLateral1;
        }

        public void SetAppliedImpulseLateral1(float value)
        {
            m_appliedImpulseLateral1 = value;
        }

        public float GetAppliedImpulseLateral2()
        {
            return m_appliedImpulseLateral2;
        }

        public void SetAppliedImpulseLateral2(float value)
        {
            m_appliedImpulseLateral2 = value;
        }

        public IndexedVector3 GetLateralFrictionDir1()
        {
            return m_lateralFrictionDir1;
        }

        public void SetLateralFrictionDir1(ref IndexedVector3 value)
        {
            m_lateralFrictionDir1 = value;
        }

        public IndexedVector3 GetLateralFrictionDir2()
        {
            return m_lateralFrictionDir2;
        }

        public void SetLateralFrictionDir2(ref IndexedVector3 value)
        {
            m_lateralFrictionDir2 = value;
        }

        public int GetPartId0()
        {
            return m_partId0;
        }

        public int GetPartId1()
        {
            return m_partId1;
        }

        public int GetIndex0()
        {
            return m_index0;
        }

        public int GetIndex1()
        {
            return m_index1;
        }

        public void SetPartId0(int id)
        {
            m_partId0 = id;
        }

        public void SetPartId1(int id)
        {
            m_partId1 = id;
        }

        public void SetIndex0(int index)
        {
            m_index0 = index;
        }

        public void SetIndex1(int index)
        {
            m_index1 = index;
        }


        public IndexedVector3 m_localPointA;
        public IndexedVector3 m_localPointB;
        ///m_positionWorldOnA is redundant information, see getPositionWorldOnA(), but for clarity
        public IndexedVector3 m_positionWorldOnA;
        public IndexedVector3 m_positionWorldOnB;
        public IndexedVector3 m_normalWorldOnB;


        public float m_distance1;
        public float m_combinedFriction;
        public float m_combinedRestitution;

        //BP mod, store contact triangles.
        public int m_partId0;
        public int m_partId1;
        public int m_index0;
        public int m_index1;

        public Object m_userPersistentData;
        public float m_appliedImpulse;

        public bool m_lateralFrictionInitialized;
        public float m_appliedImpulseLateral1;
        public float m_appliedImpulseLateral2;
        public float m_contactMotion1;
        public float m_contactMotion2;
        public float m_contactCFM1;
        public float m_contactCFM2;


        public int m_lifeTime;//lifetime of the contactpoint in frames

        public IndexedVector3 m_lateralFrictionDir1;
        public IndexedVector3 m_lateralFrictionDir2;

        public ConstraintRow[] m_constraintRow = new ConstraintRow[3];

    }

    public struct ConstraintRow
    {
        public IndexedVector3 m_normal;
        public float m_rhs;
        public float m_jacDiagInv;
        public float m_lowerLimit;
        public float m_upperLimit;
        public float m_accumImpulse;

        public void Reset()
        {
            m_normal = new IndexedVector3();
            m_rhs = 0f;
            m_jacDiagInv = 0f;
            m_lowerLimit = 0f;
            m_upperLimit = 0f;
            m_accumImpulse = 0f;
        }

    }

}
