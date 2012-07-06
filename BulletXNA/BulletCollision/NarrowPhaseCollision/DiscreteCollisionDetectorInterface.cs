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

namespace BulletXNA.BulletCollision
{
    public interface IDiscreteCollisionDetectorInterface
    {
        void GetClosestPoints(ref ClosestPointInput input, IDiscreteCollisionDetectorInterfaceResult output, IDebugDraw debugDraw, bool swapResults);
    }

    public interface IDiscreteCollisionDetectorInterfaceResult
    {
        void SetShapeIdentifiersA(int partId0, int index0);
        void SetShapeIdentifiersB(int partId1, int index1);
        void AddContactPoint(IndexedVector3 normalOnBInWorld, IndexedVector3 pointInWorld, float depth);
        void AddContactPoint(ref IndexedVector3 normalOnBInWorld, ref IndexedVector3 pointInWorld, float depth);
    }

    // Temp change to class while i test maxdist squared
    public struct ClosestPointInput
    {
        public IndexedMatrix m_transformA;
        public IndexedMatrix m_transformB;
        public float m_maximumDistanceSquared;

        public ClosestPointInput(IndexedMatrix ma, IndexedMatrix mb, float dist2)
        {
            m_transformA = ma;
            m_transformB = mb;
            m_maximumDistanceSquared = dist2;
        }

        public static ClosestPointInput Default()
        {
            return _default;
        }

        private static ClosestPointInput _default = new ClosestPointInput(IndexedMatrix.Identity, IndexedMatrix.Identity, MathUtil.BT_LARGE_FLOAT);
    }

    public class StorageResult : IDiscreteCollisionDetectorInterfaceResult
    {
        public StorageResult()
        {
            m_distance = float.MaxValue;
        }

        public virtual void AddContactPoint(IndexedVector3 normalOnBInWorld, IndexedVector3 pointInWorld, float depth)
        {
            AddContactPoint(ref normalOnBInWorld, ref pointInWorld, depth);
        }

        public virtual void AddContactPoint(ref IndexedVector3 normalOnBInWorld, ref IndexedVector3 pointInWorld, float depth)
        {
            if (depth < m_distance)
            {
                m_normalOnSurfaceB = normalOnBInWorld;
                m_closestPointInB = pointInWorld;
                m_distance = depth;
            }
        }

        public virtual void SetShapeIdentifiersA(int partId0, int index0)
        {
        }

        public virtual void SetShapeIdentifiersB(int partId1, int index1)
        {
        }

        IndexedVector3 m_normalOnSurfaceB;
        IndexedVector3 m_closestPointInB;
        float m_distance = MathUtil.BT_LARGE_FLOAT; //negative means penetration !

    }
}
