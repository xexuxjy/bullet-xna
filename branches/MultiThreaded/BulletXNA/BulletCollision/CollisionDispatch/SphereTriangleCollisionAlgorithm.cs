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
    public class SphereTriangleCollisionAlgorithm : ActivatingCollisionAlgorithm
    {
        public SphereTriangleCollisionAlgorithm() { } // for pool
        public SphereTriangleCollisionAlgorithm(PersistentManifold mf, CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1, bool swapped)
            : base(ci, body0, body1)
        {
            m_ownManifold = false;
            m_manifoldPtr = mf;
            m_swapped = swapped;
        }

        public virtual void Initialize(PersistentManifold mf, CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1, bool swapped)
        {
            base.Initialize(ci, body0, body1);
            m_ownManifold = false;
            m_manifoldPtr = mf;
            m_swapped = swapped;
        }

        public SphereTriangleCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci)
            : base(ci)
        {

        }

        public override void Initialize(CollisionAlgorithmConstructionInfo ci)
        {
            base.Initialize(ci);
        }

        public override void Cleanup()
        {
            base.Cleanup();
            if (m_ownManifold)
            {
                if (m_manifoldPtr != null)
                {
                    m_dispatcher.ReleaseManifold(m_manifoldPtr);
                    m_manifoldPtr = null;
                }
            }
            m_ownManifold = false;
            BulletGlobals.SphereTriangleCollisionAlgorithmPool.Free(this);
        }


        public override void ProcessCollision(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            //resultOut = new ManifoldResult();
            if (m_manifoldPtr == null)
            {
                return;
            }

            CollisionObject sphereObj = m_swapped ? body1 : body0;
            CollisionObject triObj = m_swapped ? body0 : body1;

            SphereShape sphere = sphereObj.GetCollisionShape() as SphereShape;
            TriangleShape triangle = triObj.GetCollisionShape() as TriangleShape;

            /// report a contact. internally this will be kept persistent, and contact reduction is done
            resultOut.SetPersistentManifold(m_manifoldPtr);
            using (SphereTriangleDetector detector = BulletGlobals.SphereTriangleDetectorPool.Get())
            {

                detector.Initialize(sphere, triangle, m_manifoldPtr.GetContactBreakingThreshold());
                ClosestPointInput input = ClosestPointInput.Default();
                input.m_maximumDistanceSquared = float.MaxValue;
                sphereObj.GetWorldTransform(out input.m_transformA);
                triObj.GetWorldTransform(out input.m_transformB);

                bool swapResults = m_swapped;

                detector.GetClosestPoints(ref input, resultOut, dispatchInfo.getDebugDraw(), swapResults);

                if (m_ownManifold)
                {
                    resultOut.RefreshContactPoints();
                }
            }
        }

        public override float CalculateTimeOfImpact(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            return 1f;
        }

        public override void GetAllContactManifolds(PersistentManifoldArray manifoldArray)
        {
            if (m_manifoldPtr != null && m_ownManifold)
            {
                manifoldArray.Add(m_manifoldPtr);
            }
        }

        private bool m_ownManifold;
        private PersistentManifold m_manifoldPtr;
        private bool m_swapped;
    }

    public class SphereTriangleCreateFunc : CollisionAlgorithmCreateFunc
    {
        public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
        {
            SphereTriangleCollisionAlgorithm algo = BulletGlobals.SphereTriangleCollisionAlgorithmPool.Get();
            algo.Initialize(ci.GetManifold(), ci, body0, body1, m_swapped);
            return algo;
        }
    }
}
