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
    public interface IDispatcher
    {
        CollisionAlgorithm FindAlgorithm(CollisionObject body0, CollisionObject body1);

        CollisionAlgorithm FindAlgorithm(CollisionObject body0, CollisionObject body1, PersistentManifold sharedManifold);

        PersistentManifold GetNewManifold(CollisionObject body0, CollisionObject body1);

        void ReleaseManifold(PersistentManifold manifold);

        void ClearManifold(PersistentManifold manifold);

        bool NeedsCollision(CollisionObject body0, CollisionObject body1);

        bool NeedsResponse(CollisionObject body0, CollisionObject body1);

        void DispatchAllCollisionPairs(IOverlappingPairCache pairCache, DispatcherInfo dispatchInfo, IDispatcher dispatcher);

        int GetNumManifolds();

        PersistentManifold GetManifoldByIndexInternal(int index);

        PersistentManifoldArray GetInternalManifoldPointer();

        Object AllocateCollisionAlgorithm(int size);

        void FreeCollisionAlgorithm(CollisionAlgorithm collisionAlgorithm);
    }

    public enum DispatchFunc
    {
        DISPATCH_DISCRETE = 1,
        DISPATCH_CONTINUOUS
    }

    public class DispatcherInfo
    {
        public DispatcherInfo()
        {
            m_timeStep = 0f;
            m_stepCount = 0;
            m_dispatchFunc = DispatchFunc.DISPATCH_DISCRETE;
            m_timeOfImpact = 1f;
            m_useContinuous = true;
            m_debugDraw = null;
            m_enableSatConvex = false;
            m_enableSPU = true;
            m_useEpa = true;
            m_allowedCcdPenetration = 0.04f;
            m_useConvexConservativeDistanceUtil = false;
            m_convexConservativeDistanceThreshold = 0f;
        }

        public DispatchFunc GetDispatchFunc()
        {
            return m_dispatchFunc;
        }

        public void SetDispatchFunc(DispatchFunc func)
        {
            m_dispatchFunc = func;
        }

        public float GetTimeOfImpact()
        {
            return m_timeOfImpact;
        }

        public void SetTimeOfImpact(float toi)
        {
            m_timeOfImpact = toi;
        }

        public float GetTimeStep()
        {
            return m_timeStep;
        }

        public void SetTimeStep(float value)
        {
            m_timeStep = value;
        }

        public int GetStepCount()
        {
            return m_stepCount;
        }

        public void SetStepCount(int count)
        {
            m_stepCount = count;
        }

        public IDebugDraw getDebugDraw()
        {
            return m_debugDraw;
        }

        public void SetDebugDraw(IDebugDraw debugDraw)
        {
            m_debugDraw = debugDraw;
        }

        public float GetAllowedCcdPenetration()
        {
            return m_allowedCcdPenetration;
        }

        public void SetAllowedCcdPenetration(float ccd)
        {
            m_allowedCcdPenetration = ccd;
        }

        public float GetConvexConservativeDistanceThreshold()
        {
            return m_convexConservativeDistanceThreshold;
        }

        float m_timeStep;
        int m_stepCount;
        DispatchFunc m_dispatchFunc;
        float m_timeOfImpact;
        public bool m_useContinuous;
        public IDebugDraw m_debugDraw;
        public bool m_enableSatConvex;
        bool m_enableSPU;
        bool m_useEpa;
        float m_allowedCcdPenetration;
        bool m_useConvexConservativeDistanceUtil;
        float m_convexConservativeDistanceThreshold;
    }
}
