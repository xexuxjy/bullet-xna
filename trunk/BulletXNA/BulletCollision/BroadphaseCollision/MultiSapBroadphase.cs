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
using System.Collections.Generic;
using System.Diagnostics;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
    public class MultiSapBroadphase : IBroadphaseInterface
    {
        public MultiSapBroadphase() : this(16384,null)
        {
        }

        public MultiSapBroadphase(int maxProxies, IOverlappingPairCache pairCache)
        {
            m_overlappingPairs = pairCache;
            m_optimizedAabbTree = null;
            m_ownsPairCache = false;
            m_invalidPair = 0;

            if (m_overlappingPairs == null)
            {
                m_ownsPairCache = true;
                m_overlappingPairs = new SortedOverlappingPairCache();
            }
            m_filterCallback = new MultiSapOverlapFilterCallback();

            m_overlappingPairs.SetOverlapFilterCallback(m_filterCallback);
            //	mem = btAlignedAlloc(sizeof(btSimpleBroadphase),16);
            //	m_simpleBroadphase = new (mem) btSimpleBroadphase(maxProxies,m_overlappingPairs);
            m_simpleBroadphase = new SimpleBroadphase(maxProxies, m_overlappingPairs);
        }


        public IList<IBroadphaseInterface> GetBroadphaseArray()
        {
            return m_sapBroadphases;
        }

        public virtual void Cleanup()
        {
            if (m_ownsPairCache)
            {
                m_overlappingPairs.Cleanup();
                m_overlappingPairs = null;
                m_ownsPairCache = false;
            }
        }

        public virtual BroadphaseProxy CreateProxy(IndexedVector3 aabbMin, IndexedVector3 aabbMax, BroadphaseNativeTypes shapeType, Object userPtr, CollisionFilterGroups collisionFilterGroup, CollisionFilterGroups collisionFilterMask, IDispatcher dispatcher, Object multiSapProxy)
        {
            return CreateProxy(ref aabbMin, ref aabbMax, shapeType, userPtr, collisionFilterGroup, collisionFilterMask, dispatcher, multiSapProxy);
        }

        public virtual BroadphaseProxy CreateProxy(ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax, BroadphaseNativeTypes shapeType, Object userPtr, CollisionFilterGroups collisionFilterGroup, CollisionFilterGroups collisionFilterMask, IDispatcher dispatcher, Object multiSapProxy)
        {
            //void* ignoreMe -> we could think of recursive multi-sap, if someone is interested

            MultiSapProxy proxy = new MultiSapProxy(ref aabbMin, ref aabbMax, shapeType, userPtr, collisionFilterGroup, collisionFilterMask);
            m_multiSapProxies.Add(proxy);

            ///this should deal with inserting/removal into child broadphases
            SetAabb(proxy, ref aabbMin, ref aabbMax, dispatcher);
            return proxy;
        }

        public virtual void DestroyProxy(BroadphaseProxy proxy, IDispatcher dispatcher)
        {
            ///not yet
            Debug.Assert(false);

        }

        public virtual void SetAabb(BroadphaseProxy proxy, ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax, IDispatcher dispatcher)
        {
            MultiSapProxy multiProxy = (MultiSapProxy)proxy;
            multiProxy.m_aabbMin = aabbMin;
            multiProxy.m_aabbMax = aabbMax;

            MyNodeOverlapCallback myNodeCallback = new MyNodeOverlapCallback(this, multiProxy, dispatcher);

            if (m_optimizedAabbTree != null)
            {
                m_optimizedAabbTree.ReportAabbOverlappingNodex(myNodeCallback, ref aabbMin, ref aabbMax);
            }

            for (int i = 0; i < multiProxy.m_bridgeProxies.Count; i++)
            {
                IndexedVector3 worldAabbMin;
                IndexedVector3 worldAabbMax;
                multiProxy.m_bridgeProxies[i].m_childBroadphase.GetBroadphaseAabb(out worldAabbMin, out worldAabbMax);
                bool overlapsBroadphase = AabbUtil2.TestAabbAgainstAabb2(ref worldAabbMin, ref worldAabbMax, ref multiProxy.m_aabbMin, ref multiProxy.m_aabbMax);
                if (!overlapsBroadphase)
                {
                    //remove it now
                    BridgeProxy bridgeProxy = multiProxy.m_bridgeProxies[i];

                    BroadphaseProxy childProxy = bridgeProxy.m_childProxy;
                    bridgeProxy.m_childBroadphase.DestroyProxy(childProxy, dispatcher);

                    multiProxy.m_bridgeProxies.RemoveAtQuick(i);
                }
            }


            /*

            if (1)
            {

                //find broadphase that contain this multiProxy
                int numChildBroadphases = getBroadphaseArray().size();
                for (int i=0;i<numChildBroadphases;i++)
                {
                    btBroadphaseInterface* childBroadphase = getBroadphaseArray()[i];
                    btVector3 worldAabbMin,worldAabbMax;
                    childBroadphase->getBroadphaseAabb(worldAabbMin,worldAabbMax);
                    bool overlapsBroadphase = TestAabbAgainstAabb2(worldAabbMin,worldAabbMax,multiProxy->m_aabbMin,multiProxy->m_aabbMax);
			
                //	fullyContained = fullyContained || boxIsContainedWithinBox(worldAabbMin,worldAabbMax,multiProxy->m_aabbMin,multiProxy->m_aabbMax);
                    int containingBroadphaseIndex = -1;
			
                    //if already contains this
			
                    for (int i=0;i<multiProxy->m_bridgeProxies.size();i++)
                    {
                        if (multiProxy->m_bridgeProxies[i]->m_childBroadphase == childBroadphase)
                        {
                            containingBroadphaseIndex = i;
                        }
                        alreadyInSimple = alreadyInSimple || (multiProxy->m_bridgeProxies[i]->m_childBroadphase == m_simpleBroadphase);
                    }

                    if (overlapsBroadphase)
                    {
                        if (containingBroadphaseIndex<0)
                        {
                            btBroadphaseProxy* childProxy = childBroadphase->createProxy(aabbMin,aabbMax,multiProxy->m_shapeType,multiProxy->m_clientObject,multiProxy->m_collisionFilterGroup,multiProxy->m_collisionFilterMask, dispatcher);
                            childProxy->m_multiSapParentProxy = multiProxy;
                            addToChildBroadphase(multiProxy,childProxy,childBroadphase);
                        }
                    } else
                    {
                        if (containingBroadphaseIndex>=0)
                        {
                            //remove
                            btBridgeProxy* bridgeProxy = multiProxy->m_bridgeProxies[containingBroadphaseIndex];

                            btBroadphaseProxy* childProxy = bridgeProxy->m_childProxy;
                            bridgeProxy->m_childBroadphase->destroyProxy(childProxy,dispatcher);
					
                            multiProxy->m_bridgeProxies.swap( containingBroadphaseIndex,multiProxy->m_bridgeProxies.size()-1);
                            multiProxy->m_bridgeProxies.pop_back();
                        }
                    }
                }


                ///If we are in no other child broadphase, stick the proxy in the global 'simple' broadphase (brute force)
                ///hopefully we don't end up with many entries here (can assert/provide feedback on stats)
                if (0)//!multiProxy->m_bridgeProxies.size())
                {
                    ///we don't pass the userPtr but our multisap proxy. We need to patch this, before processing an actual collision
                    ///this is needed to be able to calculate the aabb overlap
                    btBroadphaseProxy* childProxy = m_simpleBroadphase->createProxy(aabbMin,aabbMax,multiProxy->m_shapeType,multiProxy->m_clientObject,multiProxy->m_collisionFilterGroup,multiProxy->m_collisionFilterMask, dispatcher);
                    childProxy->m_multiSapParentProxy = multiProxy;
                    addToChildBroadphase(multiProxy,childProxy,m_simpleBroadphase);
                }
            }

            if (!multiProxy->m_bridgeProxies.size())
            {
                ///we don't pass the userPtr but our multisap proxy. We need to patch this, before processing an actual collision
                ///this is needed to be able to calculate the aabb overlap
                btBroadphaseProxy* childProxy = m_simpleBroadphase->createProxy(aabbMin,aabbMax,multiProxy->m_shapeType,multiProxy->m_clientObject,multiProxy->m_collisionFilterGroup,multiProxy->m_collisionFilterMask, dispatcher);
                childProxy->m_multiSapParentProxy = multiProxy;
                addToChildBroadphase(multiProxy,childProxy,m_simpleBroadphase);
            }
        */


            //update
            for (int i = 0; i < multiProxy.m_bridgeProxies.Count; i++)
            {
                BridgeProxy bridgeProxyRef = multiProxy.m_bridgeProxies[i];
                bridgeProxyRef.m_childBroadphase.SetAabb(bridgeProxyRef.m_childProxy, ref aabbMin, ref aabbMax, dispatcher);
            }
        }

        public virtual void GetAabb(BroadphaseProxy proxy, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
        {
            MultiSapProxy multiProxy = (MultiSapProxy)(proxy);
            aabbMin = multiProxy.m_aabbMin;
            aabbMax = multiProxy.m_aabbMax;
        }

        public virtual void RayTest(ref IndexedVector3 rayFrom, ref IndexedVector3 rayTo, BroadphaseRayCallback rayCallback)
        {
            IndexedVector3 min = MathUtil.MIN_VECTOR;
            IndexedVector3 max = MathUtil.MAX_VECTOR;
            RayTest(ref rayFrom, ref rayTo, rayCallback, ref min, ref max);
        }
        public virtual void RayTest(ref IndexedVector3 rayFrom, ref IndexedVector3 rayTo, BroadphaseRayCallback rayCallback, ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax)
        {
            for (int i = 0; i < m_multiSapProxies.Count; i++)
            {
                rayCallback.Process(m_multiSapProxies[i]);
            }
        }

        public virtual void AabbTest(ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax, IBroadphaseAabbCallback callback)
        {
            // not sure why this isn't implemented in the c++ version?
        }

        public void AddToChildBroadphase(MultiSapProxy parentMultiSapProxy, BroadphaseProxy childProxy, IBroadphaseInterface childBroadphase)
        {
            BridgeProxy bridgeProxyRef = new BridgeProxy();
            bridgeProxyRef.m_childProxy = childProxy;
            bridgeProxyRef.m_childBroadphase = childBroadphase;
            parentMultiSapProxy.m_bridgeProxies.Add(bridgeProxyRef);
        }

        ///calculateOverlappingPairs is optional: incremental algorithms (sweep and prune) might do it during the set aabb
        public void CalculateOverlappingPairs(IDispatcher dispatcher)
        {
            if (!m_stopUpdating && GetOverlappingPairCache().HasDeferredRemoval())
            {

                IList<BroadphasePair> overlappingPairArray = GetOverlappingPairCache().GetOverlappingPairArray();


                ((List<BroadphasePair>)overlappingPairArray).Sort();


                m_invalidPair = 0;

                int i;

                BroadphasePair previousPair = new BroadphasePair();
                previousPair.m_pProxy0 = null;
                previousPair.m_pProxy1 = null;
                previousPair.m_algorithm = null;


                for (i = 0; i < overlappingPairArray.Count; i++)
                {

                    BroadphasePair pair = overlappingPairArray[i];

                    MultiSapProxy aProxy0 = pair.m_pProxy0 != null ? (MultiSapProxy)pair.m_pProxy0.m_multiSapParentProxy : null;
                    MultiSapProxy aProxy1 = pair.m_pProxy1 != null ? (MultiSapProxy)pair.m_pProxy1.m_multiSapParentProxy : null;
                    MultiSapProxy bProxy0 = previousPair.m_pProxy0 != null ? (MultiSapProxy)previousPair.m_pProxy0.m_multiSapParentProxy : null;
                    MultiSapProxy bProxy1 = previousPair.m_pProxy1 != null ? (MultiSapProxy)previousPair.m_pProxy1.m_multiSapParentProxy : null;

                    bool isDuplicate = (aProxy0 == bProxy0) && (aProxy1 == bProxy1);

                    previousPair = pair;

                    bool needsRemoval = false;

                    if (!isDuplicate)
                    {
                        bool hasOverlap = TestAabbOverlap(pair.m_pProxy0, pair.m_pProxy1);

                        if (hasOverlap)
                        {
                            needsRemoval = false;//callback->processOverlap(pair);
                        }
                        else
                        {
                            needsRemoval = true;
                        }
                    }
                    else
                    {
                        //remove duplicate
                        needsRemoval = true;
                        //should have no algorithm
                        Debug.Assert(pair.m_algorithm == null);
                    }

                    if (needsRemoval)
                    {
                        GetOverlappingPairCache().CleanOverlappingPair(pair, dispatcher);

                        //		m_overlappingPairArray.swap(i,m_overlappingPairArray.size()-1);
                        //		m_overlappingPairArray.pop_back();
                        pair.m_pProxy0 = null;
                        pair.m_pProxy1 = null;
                        m_invalidPair++;
                        BulletGlobals.gOverlappingPairs--;
                    }

                }

                ///if you don't like to skip the invalid pairs in the array, execute following code:
                //#define CLEAN_INVALID_PAIRS 1
                //#ifdef CLEAN_INVALID_PAIRS

                //    //perform a sort, to sort 'invalid' pairs to the end
                //    //overlappingPairArray.heapSort(btMultiSapBroadphasePairSortPredicate());
                //    overlappingPairArray.quickSort(btMultiSapBroadphasePairSortPredicate());

                //    overlappingPairArray.resize(overlappingPairArray.size() - m_invalidPair);
                //    m_invalidPair = 0;
                //#endif//CLEAN_INVALID_PAIRS

                //printf("overlappingPairArray.size()=%d\n",overlappingPairArray.size());
            }
        }

        public bool TestAabbOverlap(BroadphaseProxy childProxy0, BroadphaseProxy childProxy1)
        {
            MultiSapProxy multiSapProxy0 = (MultiSapProxy)childProxy0.m_multiSapParentProxy;
            MultiSapProxy multiSapProxy1 = (MultiSapProxy)childProxy1.m_multiSapParentProxy;

            return AabbUtil2.TestAabbAgainstAabb2(ref multiSapProxy0.m_aabbMin, ref multiSapProxy0.m_aabbMax,
                ref multiSapProxy1.m_aabbMin, ref multiSapProxy1.m_aabbMax);
        }

        public virtual IOverlappingPairCache GetOverlappingPairCache()
        {
            return m_overlappingPairs;
        }

        ///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
        ///will add some transform later
        public virtual void GetBroadphaseAabb(out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
        {
            aabbMin = MathUtil.MIN_VECTOR;
            aabbMax = MathUtil.MAX_VECTOR;
        }

        public void BuildTree(ref IndexedVector3 bvhAabbMin, ref IndexedVector3 bvhAabbMax)
        {
            m_optimizedAabbTree = new QuantizedBvh();
            m_optimizedAabbTree.SetQuantizationValues(ref bvhAabbMin, ref bvhAabbMax);
            IList<QuantizedBvhNode> nodes = m_optimizedAabbTree.GetLeafNodeArray();
            for (int i = 0; i < m_sapBroadphases.Count; i++)
            {
                QuantizedBvhNode node = new QuantizedBvhNode();
                IndexedVector3 aabbMin;
                IndexedVector3 aabbMax;
                m_sapBroadphases[i].GetBroadphaseAabb(out aabbMin, out aabbMax);
                m_optimizedAabbTree.Quantize(out node.m_quantizedAabbMin, ref aabbMin, false);
                m_optimizedAabbTree.Quantize(out node.m_quantizedAabbMax, ref aabbMax, true);
                int partId = 0;
                node.m_escapeIndexOrTriangleIndex = (partId << (31 - QuantizedBvh.MAX_NUM_PARTS_IN_BITS)) | i;
                nodes.Add(node);
            }
            m_optimizedAabbTree.BuildInternal();
        }

        public virtual void PrintStats()
        {
        }

        //public void quicksort(IList<BroadphasePair> a, int lo, int hi);

        ///reset broadphase internal structures, to ensure determinism/reproducability
        public virtual void ResetPool(IDispatcher dispatcher)
        {
            // not yet
        }


        private IList<IBroadphaseInterface> m_sapBroadphases = new List<IBroadphaseInterface>();
        private SimpleBroadphase m_simpleBroadphase;

        private IOverlappingPairCache m_overlappingPairs;

        private QuantizedBvh m_optimizedAabbTree;

        private bool m_ownsPairCache;

        private IOverlapFilterCallback m_filterCallback;

        private int m_invalidPair;

        protected IList<MultiSapProxy> m_multiSapProxies;

        protected bool m_stopUpdating = false;


        class MyNodeOverlapCallback : INodeOverlapCallback
        {
            MultiSapBroadphase m_multiSap;
            MultiSapProxy m_multiProxy;
            IDispatcher m_dispatcher;

            public MyNodeOverlapCallback(MultiSapBroadphase multiSap, MultiSapProxy multiProxy, IDispatcher dispatcher)
            {
                m_multiSap = multiSap;
                m_multiProxy = multiProxy;
                m_dispatcher = dispatcher;

            }

            public virtual void ProcessNode(int nodeSubPart, int broadphaseIndex)
            {
                IBroadphaseInterface childBroadphase = m_multiSap.GetBroadphaseArray()[broadphaseIndex];

                int containingBroadphaseIndex = -1;
                //already found?
                for (int i = 0; i < m_multiProxy.m_bridgeProxies.Count; i++)
                {

                    if (m_multiProxy.m_bridgeProxies[i].m_childBroadphase == childBroadphase)
                    {
                        containingBroadphaseIndex = i;
                        break;
                    }
                }
                if (containingBroadphaseIndex < 0)
                {
                    //add it
                    BroadphaseProxy childProxy = childBroadphase.CreateProxy(ref m_multiProxy.m_aabbMin, ref m_multiProxy.m_aabbMax, m_multiProxy.m_shapeType, m_multiProxy.m_clientObject, m_multiProxy.m_collisionFilterGroup, m_multiProxy.m_collisionFilterMask, m_dispatcher, m_multiProxy);
                    m_multiSap.AddToChildBroadphase(m_multiProxy, childProxy, childBroadphase);
                }
            }

            public virtual void Cleanup()
            {
            }
        }
    }

    public class MultiSapProxy : BroadphaseProxy
    {
        ///array with all the entries that this proxy belongs to
        public ObjectArray<BridgeProxy> m_bridgeProxies;
        //public IndexedVector3	m_aabbMin;
        //public IndexedVector3	m_aabbMax;

        public BroadphaseNativeTypes m_shapeType;

        /*		void*	m_userPtr;
                short int	m_collisionFilterGroup;
                short int	m_collisionFilterMask;
        */
        public MultiSapProxy(ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax, BroadphaseNativeTypes shapeType,
            Object userPtr, CollisionFilterGroups collisionFilterGroup, CollisionFilterGroups collisionFilterMask)
            : base(ref aabbMin, ref aabbMax, userPtr, collisionFilterGroup, collisionFilterMask, null)
        {
            m_aabbMin = aabbMin;
            m_aabbMax = aabbMax;
            m_shapeType = shapeType;
            m_multiSapParentProxy = this;
        }
    }

    public class BridgeProxy
    {
        public BroadphaseProxy m_childProxy;
        public IBroadphaseInterface m_childBroadphase;
    }

    public class MultiSapOverlapFilterCallback : IOverlapFilterCallback
    {
        // return true when pairs need collision
        public virtual bool NeedBroadphaseCollision(BroadphaseProxy childProxy0, BroadphaseProxy childProxy1)
        {
            BroadphaseProxy multiProxy0 = (BroadphaseProxy)childProxy0.m_multiSapParentProxy;
            BroadphaseProxy multiProxy1 = (BroadphaseProxy)childProxy1.m_multiSapParentProxy;

            bool collides = (multiProxy0.m_collisionFilterGroup & multiProxy1.m_collisionFilterMask) != 0;
            collides = collides && ((multiProxy1.m_collisionFilterGroup & multiProxy0.m_collisionFilterMask) != 0);

            return collides;
        }
    }
}
