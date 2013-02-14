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

#define USE_POOLED_MANIFOLDRESULT

using System;
using System.Collections.Generic;
using System.Diagnostics;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
    public class CollisionDispatcher : IDispatcher
    {
        public CollisionDispatcher(ICollisionConfiguration collisionConfiguration)
        {
            m_collisionConfiguration = collisionConfiguration;
            m_dispatcherFlags = DispatcherFlags.CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD;
            SetNearCallback(new DefaultNearCallback());

            const int maxTypes = (int)BroadphaseNativeTypes.MAX_BROADPHASE_COLLISION_TYPES;
            m_doubleDispatch = new CollisionAlgorithmCreateFunc[maxTypes, maxTypes];
            for (int i = 0; i < maxTypes; i++)
            {
                for (int j = 0; j < maxTypes; j++)
                {
                    m_doubleDispatch[i, j] = m_collisionConfiguration.GetCollisionAlgorithmCreateFunc((BroadphaseNativeTypes)i, (BroadphaseNativeTypes)j);
                    Debug.Assert(m_doubleDispatch[i, j] != null);
                }
            }
        }

        public DispatcherFlags GetDispatcherFlags()
        {
            return m_dispatcherFlags;
        }

        public void SetDispatcherFlags(DispatcherFlags flags)
        {
            m_dispatcherFlags = flags;
        }


        public virtual void Cleanup()
        {

        }

        public virtual PersistentManifold GetNewManifold(CollisionObject b0, CollisionObject b1)
        {
            gNumManifold++;

            CollisionObject body0 = b0;
            CollisionObject body1 = b1;

            //optional relative contact breaking threshold, turned on by default (use setDispatcherFlags to switch off feature for improved performance)

            float contactBreakingThreshold = ((m_dispatcherFlags & DispatcherFlags.CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD) > 0) ?
                Math.Min(body0.GetCollisionShape().GetContactBreakingThreshold(BulletGlobals.gContactBreakingThreshold), body1.GetCollisionShape().GetContactBreakingThreshold(BulletGlobals.gContactBreakingThreshold))
                : BulletGlobals.gContactBreakingThreshold;

            float contactProcessingThreshold = Math.Min(body0.GetContactProcessingThreshold(), body1.GetContactProcessingThreshold());

            // nothing in our pool so create a new one and return it.
            // need a way to flush the pool ideally
            PersistentManifold manifold = BulletGlobals.PersistentManifoldPool.Get();
            manifold.Initialise(body0, body1, 0, contactBreakingThreshold, contactProcessingThreshold);

            manifold.m_index1a = m_manifoldsPtr.Count;


            m_manifoldsPtr.Add(manifold);
#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugDispatcher)
            {
                BulletGlobals.g_streamWriter.WriteLine("GetNewManifold[{0}][{1}]", manifold.m_index1a,m_manifoldsPtr.Count);
            }
#endif

            return manifold;
        }

        public virtual void ReleaseManifold(PersistentManifold manifold)
        {
            gNumManifold--;
            ClearManifold(manifold);
            int findIndex = manifold.m_index1a;
            //m_manifoldsPtr.Remove(manifold);
            m_manifoldsPtr.RemoveAtQuick(findIndex);
            m_manifoldsPtr[findIndex].m_index1a = findIndex;

            //PersistentManifold swapTemp = m_manifoldsPtr[findIndex];
            //m_manifoldsPtr[findIndex] = m_manifoldsPtr[m_manifoldsPtr.Count - 1];
            //m_manifoldsPtr[m_manifoldsPtr.Count - 1] = swapTemp;
            //m_manifoldsPtr[findIndex].m_index1a = findIndex;
            //m_manifoldsPtr.RemoveAt(m_manifoldsPtr.Count - 1);

#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugDispatcher)
            {
                BulletGlobals.g_streamWriter.WriteLine("ReleaseManifold[{0}][{1}]", manifold.m_index1a, m_manifoldsPtr.Count);
            }
#endif
            // and return it to free list.
            BulletGlobals.PersistentManifoldPool.Free(manifold);
        }

        public virtual void ClearManifold(PersistentManifold manifold)
        {
            manifold.ClearManifold();
        }

        public CollisionAlgorithm FindAlgorithm(CollisionObject body0, CollisionObject body1)
        {
            return FindAlgorithm(body0, body1, null);
        }

        public CollisionAlgorithm FindAlgorithm(CollisionObject body0, CollisionObject body1, PersistentManifold sharedManifold)
        {
            CollisionAlgorithmConstructionInfo ci = new CollisionAlgorithmConstructionInfo(this, -1);
            ci.SetManifold(sharedManifold);
            int index1 = (int)body0.GetCollisionShape().GetShapeType();
            int index2 = (int)body1.GetCollisionShape().GetShapeType();

            return m_doubleDispatch[index1, index2].CreateCollisionAlgorithm(ci, body0, body1);
        }

        public virtual bool NeedsCollision(CollisionObject body0, CollisionObject body1)
        {
            Debug.Assert(body0 != null);
            Debug.Assert(body1 != null);

            bool needsCollision = true;

#if BT_DEBUG
        	if ((m_dispatcherFlags & DispatcherFlags.CD_STATIC_STATIC_REPORTED == 0))
	        {
		        //broadphase filtering already deals with this
		        if ((body0.IsStaticOrKinematicObject()) && body1.isStaticOrKinematicObject())
		        {
                    m_dispatcherFlags |= DispatcherFlags.CD_STATIC_STATIC_REPORTED;
			        System.err.Writeline("warning CollisionDispatcher::needsCollision: static-static collision!\n");
		        }
	        }
#endif //BT_DEBUG

            if ((!body0.IsActive()) && (!body1.IsActive()))
            {
                needsCollision = false;
            }
            else if (!body0.CheckCollideWith(body1))
            {
                needsCollision = false;
            }
            return needsCollision;
        }

        public virtual bool NeedsResponse(CollisionObject body0, CollisionObject body1)
        {
            //here you can do filtering
            bool hasResponse = (body0.HasContactResponse() && body1.HasContactResponse());
            //no response between two static/kinematic bodies:
            hasResponse = hasResponse && ((!body0.IsStaticOrKinematicObject()) || (!body1.IsStaticOrKinematicObject()));
            return hasResponse;
        }

        public virtual void DispatchAllCollisionPairs(IOverlappingPairCache pairCache, DispatcherInfo dispatchInfo, IDispatcher dispatcher)
        {
            m_collisionCallback.Initialize(dispatchInfo, this);
            pairCache.ProcessAllOverlappingPairs(m_collisionCallback, dispatcher);
            m_collisionCallback.cleanup();
        }

        public void SetNearCallback(INearCallback nearCallback)
        {
            m_nearCallback = nearCallback;
        }

        public INearCallback GetNearCallback()
        {
            return m_nearCallback;
        }

        ///registerCollisionCreateFunc allows registration of custom/alternative collision create functions
        public void RegisterCollisionCreateFunc(int proxyType0, int proxyType1, CollisionAlgorithmCreateFunc createFunc)
        {
            m_doubleDispatch[proxyType0, proxyType1] = createFunc;
        }

        public int GetNumManifolds()
        {
            return m_manifoldsPtr.Count;
        }

        public PersistentManifold GetManifoldByIndexInternal(int index)
        {
            return m_manifoldsPtr[index];
        }

        //by default, Bullet will use this near callback
        //public static void  defaultNearCallback(BroadphasePair collisionPair, CollisionDispatcher dispatcher, DispatcherInfo dispatchInfo);

        public virtual Object AllocateCollisionAlgorithm(int size)
        {
            return null;
        }

        public virtual void FreeCollisionAlgorithm(CollisionAlgorithm collisionAlgorithm)
        {
            if (collisionAlgorithm != null)
            {
                collisionAlgorithm.Cleanup();
            }
        }

        public ICollisionConfiguration GetCollisionConfiguration()
        {
            return m_collisionConfiguration;
        }

        public void SetCollisionConfiguration(ICollisionConfiguration config)
        {
            m_collisionConfiguration = config;
        }

        public virtual PersistentManifoldArray GetInternalManifoldPointer()
        {
            return m_manifoldsPtr;
        }

        public ManifoldResult GetNewManifoldResult(CollisionObject o1, CollisionObject o2)
        {
            ManifoldResult manifoldResult = null;
#if USE_POOLED_MANIFOLDRESULT
            manifoldResult = BulletGlobals.ManifoldResultPool.Get();
#endif
            manifoldResult.Initialise(o1, o2);
            return manifoldResult;
        }

        public void FreeManifoldResult(ManifoldResult result)
        {
#if USE_POOLED_MANIFOLDRESULT
            BulletGlobals.ManifoldResultPool.Free(result);
#else

#endif
        }


        private PersistentManifoldArray m_manifoldsPtr = new PersistentManifoldArray();

        private DispatcherFlags m_dispatcherFlags;

        //private bool m_useIslands;
        //private bool m_staticWarningReported;
        private ManifoldResult m_defaultManifoldResult;
        private INearCallback m_nearCallback;
        private ICollisionConfiguration m_collisionConfiguration;

        private CollisionPairCallback m_collisionCallback = new CollisionPairCallback(null, null);

        public static int gNumManifold = 0;


        //btPoolAllocator*	m_collisionAlgorithmPoolAllocator;
        //btPoolAllocator*	m_persistentManifoldPoolAllocator;
        CollisionAlgorithmCreateFunc[,] m_doubleDispatch;
    }

    //-------------------------------------------------------------------------------------------------

    ///interface for iterating all overlapping collision pairs, no matter how those pairs are stored (array, set, map etc)
    ///this is useful for the collision dispatcher.
    public class CollisionPairCallback : IOverlapCallback
    {
        public CollisionPairCallback(DispatcherInfo dispatchInfo, CollisionDispatcher dispatcher)
        {
            m_dispatchInfo = dispatchInfo;
            m_dispatcher = dispatcher;
        }

        public void Initialize(DispatcherInfo dispatchInfo, CollisionDispatcher dispatcher)
        {
            m_dispatchInfo = dispatchInfo;
            m_dispatcher = dispatcher;
        }

        /*btCollisionPairCallback& operator=(btCollisionPairCallback& other)
        {
            m_dispatchInfo = other.m_dispatchInfo;
            m_dispatcher = other.m_dispatcher;
            return *this;
        }
        */

        public virtual void cleanup()
        {
        }

        public virtual bool ProcessOverlap(BroadphasePair pair)
        {
            m_dispatcher.GetNearCallback().NearCallback(pair, m_dispatcher, m_dispatchInfo);
            return false;
        }
        DispatcherInfo m_dispatchInfo;
        CollisionDispatcher m_dispatcher;
    }

    //-------------------------------------------------------------------------------------------------

    public interface INearCallback
    {
        void NearCallback(BroadphasePair collisionPair, CollisionDispatcher dispatcher, DispatcherInfo dispatchInfo);
    }

    //-------------------------------------------------------------------------------------------------

    public class DefaultNearCallback : INearCallback
    {
        public void NearCallback(BroadphasePair collisionPair, CollisionDispatcher dispatcher, DispatcherInfo dispatchInfo)
        {
            CollisionObject colObj0 = collisionPair.m_pProxy0.GetClientObject() as CollisionObject;
            CollisionObject colObj1 = collisionPair.m_pProxy1.GetClientObject() as CollisionObject;

            if (dispatcher.NeedsCollision(colObj0, colObj1))
            {
                //dispatcher will keep algorithms persistent in the collision pair
                if (collisionPair.m_algorithm == null)
                {
                    collisionPair.m_algorithm = dispatcher.FindAlgorithm(colObj0, colObj1, null);
                }

                if (collisionPair.m_algorithm != null)
                {
                    ManifoldResult contactPointResult = dispatcher.GetNewManifoldResult(colObj0, colObj1);

                    if (dispatchInfo.GetDispatchFunc() == DispatchFunc.DISPATCH_DISCRETE)
                    {
                        //discrete collision detection query
                        collisionPair.m_algorithm.ProcessCollision(colObj0, colObj1, dispatchInfo, contactPointResult);
                    }
                    else
                    {
                        //continuous collision detection query, time of impact (toi)
                        float toi = collisionPair.m_algorithm.CalculateTimeOfImpact(colObj0, colObj1, dispatchInfo, contactPointResult);
                        if (dispatchInfo.GetTimeOfImpact() > toi)
                        {
                            dispatchInfo.SetTimeOfImpact(toi);
                        }
                    }
#if DEBUG
                    if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugDispatcher)
                    {
                        BulletGlobals.g_streamWriter.WriteLine("NearCallback[{0}][{1}][{2}]", contactPointResult.GetBody0Internal().GetUserPointer(), contactPointResult.GetBody1Internal().GetUserPointer(),contactPointResult.GetPersistentManifold().GetNumContacts());
                    }
#endif
                    dispatcher.FreeManifoldResult(contactPointResult);
                }
            }
        }
    }

    public enum DispatcherFlags
    {
        CD_STATIC_STATIC_REPORTED = 1,
        CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD = 2,
        CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION = 4

    }

}
