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
using System.Collections.Generic;

using BulletXNA.BulletCollision.CollisionDispatch;
using BulletXNA.BulletCollision.NarrowPhaseCollision;

namespace BulletXNA.BulletCollision.BroadphaseCollision
{
    public class CollisionAlgorithmConstructionInfo
    {
        public CollisionAlgorithmConstructionInfo()
        {
            m_dispatcher1 = null;
            m_manifold = null;
        }

        public CollisionAlgorithmConstructionInfo(IDispatcher dispatcher, int temp)
        {
            m_dispatcher1 = dispatcher;
            //(void)temp;
        }

        public int GetDispatcherId()
        {
            return 0;
        }

        public void SetManifold(PersistentManifold manifold)
        {
            m_manifold = manifold;
        }

        public PersistentManifold GetManifold()
        {
            return m_manifold;
        }

        public IDispatcher GetDispatcher()
        {
            return m_dispatcher1;
        }

        public void SetDispatcher(IDispatcher dispatcher)
        {
            m_dispatcher1 = dispatcher;
        }

        private IDispatcher m_dispatcher1;
        private PersistentManifold m_manifold;


    };


    ///btCollisionAlgorithm is an collision interface that is compatible with the Broadphase and btDispatcher.
    ///It is persistent over frames
    public abstract class CollisionAlgorithm
    {

        public CollisionAlgorithm()
        {
            int ibreak = 0;
        }

        public CollisionAlgorithm(CollisionAlgorithmConstructionInfo ci)
        {
            m_dispatcher = ci.GetDispatcher();
        }

        public abstract void ProcessCollision(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut);

        public abstract float CalculateTimeOfImpact(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut);

        public abstract void GetAllContactManifolds(IList<PersistentManifold> manifoldArray);

        public virtual void Cleanup()
        {
        }

        protected int GetDispatcherId()
        {
            return 1;
        }

        protected IDispatcher m_dispatcher;

    }
}