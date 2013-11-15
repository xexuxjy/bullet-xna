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

namespace BulletXNA.BulletCollision
{
    //-------------------------------------------------------------------------------------------------

    public static class OverlappingPairCacheGlobals
    {
        public static int gOverlappingPairs = 0;

        public static int gRemovePairs = 0;
        public static int gAddedPairs = 0;
        public static int gFindPairs = 0;
    }


    public interface IOverlapCallback
    {
        //return true for deletion of the pair
        bool ProcessOverlap(BroadphasePair pair);
    }

    //-------------------------------------------------------------------------------------------------

    public interface IOverlapFilterCallback
    {
        // return true when pairs need collision
        bool NeedBroadphaseCollision(BroadphaseProxy proxy0, BroadphaseProxy proxy1);
    }

    //-------------------------------------------------------------------------------------------------

    public interface IOverlappingPairCache : IOverlappingPairCallback
    {
        //BroadphasePair getOverlappingPairArrayPtr();
        ObjectArray<BroadphasePair> GetOverlappingPairArray();
        void CleanOverlappingPair(BroadphasePair pair, IDispatcher dispatcher);
        int GetNumOverlappingPairs();
        void CleanProxyFromPairs(BroadphaseProxy proxy, IDispatcher dispatcher);
        void SetOverlapFilterCallback(IOverlapFilterCallback callback);
        void ProcessAllOverlappingPairs(IOverlapCallback callback, IDispatcher dispatcher);
        BroadphasePair FindPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1);
        bool HasDeferredRemoval();
        void SetInternalGhostPairCallback(IOverlappingPairCallback ghostPairCallback);
        void SortOverlappingPairs(IDispatcher dispatcher);
        void Cleanup();

    }

    //-------------------------------------------------------------------------------------------------

    /// Hash-space based Pair Cache, thanks to Erin Catto, Box2D, http://www.box2d.org, and Pierre Terdiman, Codercorner, http://codercorner.com
    public class HashedOverlappingPairCache : IOverlappingPairCache
    {
        public HashedOverlappingPairCache()
        {
            m_overlapFilterCallback = null;
            m_blockedForChanges = false;
            m_ghostPairCallback = null;
            int initialAllocatedSize = 2;
            m_overlappingPairArray = new ObjectArray<BroadphasePair>(initialAllocatedSize);
            GrowTables();
        }


        public virtual void Cleanup()
        {

        }

        public virtual void RemoveOverlappingPairsContainingProxy(BroadphaseProxy proxy, IDispatcher dispatcher)
        {
            RemovePairCallback removeCallback = new RemovePairCallback(proxy);
            ProcessAllOverlappingPairs(removeCallback, dispatcher);
        }

        public virtual Object RemoveOverlappingPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1, IDispatcher dispatcher)
        {
            OverlappingPairCacheGlobals.gRemovePairs++;
            if (proxy0.m_uniqueId > proxy1.m_uniqueId)
            {
                BroadphaseProxy temp = proxy0;
                proxy0 = proxy1;
                proxy1 = temp;
            }
            int proxyId1 = proxy0.GetUid();
            int proxyId2 = proxy1.GetUid();
            int hash = (int)(GetHash((uint)(proxyId1), (uint)(proxyId2)) & (m_overlappingPairArray.Capacity - 1));

            BroadphasePair pair = InternalFindPair(proxy0, proxy1, hash);
            if (pair == null)
            {
                return null;
            }

            CleanOverlappingPair(pair, dispatcher);

            Object userData = pair.m_internalInfo1;

            Debug.Assert(pair.m_pProxy0.GetUid() == proxyId1);
            Debug.Assert(pair.m_pProxy1.GetUid() == proxyId2);

            //int pairIndex = m_overlappingPairArray.IndexOf(pair);
            // we've already found this.
            int pairIndex = pair.m_index;

            Debug.Assert(pairIndex < m_overlappingPairArray.Count);

            // Remove the pair from the hash table.
            int index = m_hashTable[hash];
            Debug.Assert(index != BT_NULL_PAIR);

            int previous = BT_NULL_PAIR;
            while (index != pairIndex)
            {
                previous = index;
                index = m_next[index];
            }

            if (previous != BT_NULL_PAIR)
            {
                Debug.Assert(m_next[previous] == pairIndex);
                m_next[previous] = m_next[pairIndex];
            }
            else
            {
                m_hashTable[hash] = m_next[pairIndex];
            }

            // We now move the last pair into spot of the
            // pair being removed. We need to fix the hash
            // table indices to support the move.

            int lastPairIndex = m_overlappingPairArray.Count - 1;

            if (m_ghostPairCallback != null)
            {
                m_ghostPairCallback.RemoveOverlappingPair(proxy0, proxy1, dispatcher);
            }
            // If the removed pair is the last pair, we are done.
            if (lastPairIndex == pairIndex)
            {
                m_overlappingPairArray.RemoveAt(lastPairIndex);
                return userData;
            }

            // Remove the last pair from the hash table.
            BroadphasePair last = m_overlappingPairArray[lastPairIndex];
            /* missing swap here too, Nat. */
            int lastHash = (int)(GetHash((uint)(last.m_pProxy0.GetUid()), (uint)(last.m_pProxy1.GetUid())) & (m_overlappingPairArray.Capacity - 1));

            index = m_hashTable[lastHash];
            Debug.Assert(index != BT_NULL_PAIR);

            previous = BT_NULL_PAIR;
            while (index != lastPairIndex)
            {
                previous = index;
                index = m_next[index];
            }

            if (previous != BT_NULL_PAIR)
            {
                Debug.Assert(m_next[previous] == lastPairIndex);
                m_next[previous] = m_next[lastPairIndex];
            }
            else
            {
                m_hashTable[lastHash] = m_next[lastPairIndex];
            }

            // Copy the last pair into the remove pair's spot.
            m_overlappingPairArray[pairIndex] = m_overlappingPairArray[lastPairIndex];

            // Insert the last pair into the hash table
            m_next[pairIndex] = m_hashTable[lastHash];
            m_hashTable[lastHash] = pairIndex;

            m_overlappingPairArray.RemoveAt(lastPairIndex);

#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugPairCache)
            {
                BulletGlobals.g_streamWriter.WriteLine("HPC:RemoveOverlappingPair endSize[{0}].", m_overlappingPairArray.Count);
            }
#endif
            return userData;
        }

        public bool NeedsBroadphaseCollision(BroadphaseProxy proxy0, BroadphaseProxy proxy1)
        {
            if (m_overlapFilterCallback != null)
            {
                return m_overlapFilterCallback.NeedBroadphaseCollision(proxy0, proxy1);
            }

            bool collides = (proxy0.m_collisionFilterGroup & proxy1.m_collisionFilterMask) != 0;
            collides = collides && ((proxy1.m_collisionFilterGroup & proxy0.m_collisionFilterMask) != 0);
#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugPairCache)
            {
                BulletGlobals.g_streamWriter.WriteLine("HPC:NeedsBroadphaseCollision collides[{0}].", collides);
            }
#endif

            return collides;
        }

        // Add a pair and return the new pair. If the pair already exists,
        // no new pair is created and the old one is returned.
        public virtual BroadphasePair AddOverlappingPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1)
        {
            OverlappingPairCacheGlobals.gAddedPairs++;

            if (!NeedsBroadphaseCollision(proxy0, proxy1))
            {
                return null;
            }



            return InternalAddPair(proxy0, proxy1);
        }

        public void CleanProxyFromPairs(BroadphaseProxy proxy, IDispatcher dispatcher)
        {
            CleanPairCallback cleanPairs = new CleanPairCallback(proxy, this, dispatcher);
            ProcessAllOverlappingPairs(cleanPairs, dispatcher);
        }


        public virtual void ProcessAllOverlappingPairs(IOverlapCallback callback, IDispatcher dispatcher)
        {
#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugPairCache)
            {
                BulletGlobals.g_streamWriter.WriteLine("HPC:ProcessAllOverlappingPairs overlap[{0}].", m_overlappingPairArray.Count);
            }
#endif
            for (int i = 0; i < m_overlappingPairArray.Count; )
            {
                BroadphasePair pair = m_overlappingPairArray[i];
                if (callback.ProcessOverlap(pair))
                {
                    RemoveOverlappingPair(pair.m_pProxy0, pair.m_pProxy1, dispatcher);

                    OverlappingPairCacheGlobals.gOverlappingPairs--;
                }
                else
                {
                    i++;
                }
            }
        }

        public ObjectArray<BroadphasePair> GetOverlappingPairArray()
        {
            return m_overlappingPairArray;
        }

        public void CleanOverlappingPair(BroadphasePair pair, IDispatcher dispatcher)
        {
            if (pair.m_algorithm != null)
            {
                dispatcher.FreeCollisionAlgorithm(pair.m_algorithm);
                pair.m_algorithm = null;
            }
        }

        public BroadphasePair FindPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1)
        {
            OverlappingPairCacheGlobals.gFindPairs++;
            if (proxy0.m_uniqueId > proxy1.m_uniqueId)
            {
                BroadphaseProxy temp;
                temp = proxy0;
                proxy0 = proxy1;
                proxy1 = temp;
            }
            int proxyId1 = proxy0.GetUid();
            int proxyId2 = proxy1.GetUid();

            /*if (proxyId1 > proxyId2) 
                btSwap(proxyId1, proxyId2);*/

            int hash = (int)(GetHash((uint)(proxyId1), (uint)(proxyId2)) & (m_overlappingPairArray.Capacity - 1));

            if (hash >= m_hashTable.Count)
            {
                return null;
            }

            int index = m_hashTable[hash];
            while (index != BT_NULL_PAIR && EqualsPair(m_overlappingPairArray[index], proxyId1, proxyId2) == false)
            {
                index = m_next[index];
            }

            if (index == BT_NULL_PAIR)
            {
                return null;
            }

            Debug.Assert(index < m_overlappingPairArray.Count);

            return m_overlappingPairArray[index];
        }

        public int GetCount()
        {
            return m_overlappingPairArray.Count;
        }
        //	btBroadphasePair* GetPairs() { return m_pairs; }

        public IOverlapFilterCallback GetOverlapFilterCallback()
        {
            return m_overlapFilterCallback;
        }

        public void SetOverlapFilterCallback(IOverlapFilterCallback callback)
        {
            m_overlapFilterCallback = callback;
        }

        public int GetNumOverlappingPairs()
        {
            return m_overlappingPairArray.Count;
        }

        private BroadphasePair InternalAddPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1)
        {
            if (proxy0.m_uniqueId > proxy1.m_uniqueId)
            {
                BroadphaseProxy temp = proxy0;
                proxy0 = proxy1;
                proxy1 = temp;
            }

            int proxyId1 = proxy0.GetUid();
            int proxyId2 = proxy1.GetUid();


            int hash = (int)(GetHash((uint)proxyId1, (uint)proxyId2) & (m_overlappingPairArray.Capacity - 1));	// New hash value with new mask

            BroadphasePair pair = InternalFindPair(proxy0, proxy1, hash);
            if (pair != null)
            {
                return pair;
            }
            else
            {
                /*for(int i=0;i<m_overlappingPairArray.size();++i)
                    {
                    if(	(m_overlappingPairArray[i].m_pProxy0==proxy0)&&
                        (m_overlappingPairArray[i].m_pProxy1==proxy1))
                        {
                        printf("Adding duplicated %u<>%u\r\n",proxyId1,proxyId2);
                        internalFindPair(proxy0, proxy1, hash);
                        }
                    }*/
                int count = m_overlappingPairArray.Count;
                int oldCapacity = m_overlappingPairArray.Capacity;

                // MAN - 2.76 - uses expand noninitializing....??
                //void* mem = &m_overlappingPairArray.expand();

                //this is where we add an actual pair, so also call the 'ghost'
                if (m_ghostPairCallback != null)
                {
                    m_ghostPairCallback.AddOverlappingPair(proxy0, proxy1);
                }
                pair = new BroadphasePair(proxy0, proxy1);
                m_overlappingPairArray.Add(pair);

                int newCapacity = m_overlappingPairArray.Capacity;

                if (oldCapacity < newCapacity)
                {
                    GrowTables();
                    //hash with new capacity
                    hash = (int)(GetHash((uint)(proxyId1), (uint)(proxyId2)) & (m_overlappingPairArray.Capacity - 1));
                }


                m_next[count] = m_hashTable[hash];
                m_hashTable[hash] = count;

                return pair;
            }
        }

        private void GrowTables()
        {
            int newCapacity = m_overlappingPairArray.Capacity;

            if (m_hashTable.Capacity < newCapacity)
            {
                int curHashTableSize = m_hashTable.Count;
                m_hashTable.Capacity = newCapacity;
                m_next.Capacity = newCapacity;

                for (int i = 0; i < newCapacity; ++i)
                {
                    m_hashTable[i] = BT_NULL_PAIR;
                }
                for (int i = 0; i < newCapacity; ++i)
                {
                    m_next[i] = BT_NULL_PAIR;
                }

                for (int i = 0; i < curHashTableSize; i++)
                {
                    BroadphasePair pair = m_overlappingPairArray[i];
                    int proxyId1 = pair.m_pProxy0.GetUid();
                    int proxyId2 = pair.m_pProxy1.GetUid();
                    /*if (proxyId1 > proxyId2) 
                        btSwap(proxyId1, proxyId2);*/
                    int hashValue = (int)(GetHash((uint)(proxyId1), (uint)(proxyId2)) & (m_overlappingPairArray.Capacity - 1));	// New hash value with new mask
                    m_next[i] = m_hashTable[hashValue];
                    m_hashTable[hashValue] = i;
                }
                int ibreak = 0;
            }
        }

        private bool EqualsPair(BroadphasePair pair, int proxyId1, int proxyId2)
        {
            return pair.m_pProxy0.m_uniqueId == proxyId1 && pair.m_pProxy1.m_uniqueId == proxyId2;
        }

        /*
        // Thomas Wang's hash, see: http://www.concentric.net/~Ttwang/tech/inthash.htm
        // This assumes proxyId1 and proxyId2 are 16-bit.
        SIMD_FORCE_INLINE int getHash(int proxyId1, int proxyId2)
        {
            int key = (proxyId2 << 16) | proxyId1;
            key = ~key + (key << 15);
            key = key ^ (key >> 12);
            key = key + (key << 2);
            key = key ^ (key >> 4);
            key = key * 2057;
            key = key ^ (key >> 16);
            return key;
        }
        */

        private uint GetHash(uint proxyId1, uint proxyId2)
        {
            int key = (int)(((uint)proxyId1) | ((uint)proxyId2) << 16);
            // Thomas Wang's hash

            key += ~(key << 15);
            key ^= (key >> 10);
            key += (key << 3);
            key ^= (key >> 6);
            key += ~(key << 11);
            key ^= (key >> 16);
            return (uint)(key);
        }

        public BroadphasePair InternalFindPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1, int hash)
        {
            BroadphasePair[] rawPairArray = m_overlappingPairArray.GetRawArray();
            int proxyId1 = proxy0.GetUid();
            int proxyId2 = proxy1.GetUid();
            //#if 0 // wrong, 'equalsPair' use unsorted uids, copy-past devil striked again. Nat.
            //if (proxyId1 > proxyId2) 
            //    btSwap(proxyId1, proxyId2);
            //#endif

            int index = m_hashTable[hash];

            while (index != BT_NULL_PAIR && EqualsPair(rawPairArray[index], proxyId1, proxyId2) == false)
            {
                index = m_next[index];
            }

            if (index == BT_NULL_PAIR)
            {
                return null;
            }

            //btAssert(index < m_overlappingPairArray.size());
            // if we know this then don't we don't need to look it up again..
            rawPairArray[index].m_index = index;
            return rawPairArray[index];
        }

        public virtual bool HasDeferredRemoval()
        {
            return false;
        }

        public virtual void SetInternalGhostPairCallback(IOverlappingPairCallback ghostPairCallback)
        {
            m_ghostPairCallback = ghostPairCallback;
        }

        public virtual void SortOverlappingPairs(IDispatcher dispatcher)
        {
            ObjectArray<BroadphasePair> tmpPairs = new ObjectArray<BroadphasePair>();
            tmpPairs.AddRange(m_overlappingPairArray);
            for (int i = 0; i < tmpPairs.Count; i++)
            {
                RemoveOverlappingPair(tmpPairs[i].m_pProxy0, tmpPairs[i].m_pProxy1, dispatcher);
            }

            for (int i = 0; i < m_next.Count; i++)
            {
                m_next[i] = BT_NULL_PAIR;
            }

            tmpPairs.Sort();
            //tmpPairs.quickSort(btBroadphasePairSortPredicate());

            for (int i = 0; i < tmpPairs.Count; i++)
            {
                AddOverlappingPair(tmpPairs[i].m_pProxy0, tmpPairs[i].m_pProxy1);
            }
        }


        //protected int[]	m_hashTable;
        //protected int[] m_next;

        protected ObjectArray<int> m_hashTable = new ObjectArray<int>();
        protected ObjectArray<int> m_next = new ObjectArray<int>();


        protected IOverlappingPairCallback m_ghostPairCallback;
        private ObjectArray<BroadphasePair> m_overlappingPairArray;
        private IOverlapFilterCallback m_overlapFilterCallback;
        private bool m_blockedForChanges;

        const int BT_NULL_PAIR = -1;
        private static Object NULL_PAIR = new Object();

    }

    //-------------------------------------------------------------------------------------------------

    ///btSortedOverlappingPairCache maintains the objects with overlapping AABB
    ///Typically managed by the Broadphase, Axis3Sweep or btSimpleBroadphase
    public class SortedOverlappingPairCache : IOverlappingPairCache
    {
        public SortedOverlappingPairCache()
        {
            m_blockedForChanges = false;
            m_hasDeferredRemoval = true;
            m_overlapFilterCallback = null;
            m_ghostPairCallback = null;
            m_overlappingPairArray = new ObjectArray<BroadphasePair>(2);
        }

        //virtual ~btSortedOverlappingPairCache();
        public virtual void Cleanup()
        {
        }

        public virtual void ProcessAllOverlappingPairs(IOverlapCallback callback, IDispatcher dispatcher)
        {
            for (int i = 0; i < m_overlappingPairArray.Count; )
            {
                BroadphasePair pair = m_overlappingPairArray[i];
                if (callback.ProcessOverlap(pair))
                {
                    CleanOverlappingPair(pair, dispatcher);
                    pair.m_pProxy0 = null;
                    pair.m_pProxy1 = null;
                    m_overlappingPairArray.RemoveAtQuick(i);
                    OverlappingPairCacheGlobals.gOverlappingPairs--;
                }
                else
                {
                    i++;
                }
            }
        }

        public Object RemoveOverlappingPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1, IDispatcher dispatcher)
        {
            if (!HasDeferredRemoval())
            {
                BroadphasePair findPair = new BroadphasePair(proxy0, proxy1);

                int findIndex = m_overlappingPairArray.IndexOf(findPair);
                if (findIndex >= 0 && findIndex < m_overlappingPairArray.Count)
                {
                    OverlappingPairCacheGlobals.gOverlappingPairs--;
                    BroadphasePair pair = m_overlappingPairArray[findIndex];
                    Object userData = pair.m_internalInfo1;
                    CleanOverlappingPair(pair, dispatcher);
                    if (m_ghostPairCallback != null)
                    {
                        m_ghostPairCallback.RemoveOverlappingPair(proxy0, proxy1, dispatcher);
                    }
                    //BroadphasePair temp = m_overlappingPairArray[findIndex];
                    //m_overlappingPairArray[findIndex] = m_overlappingPairArray[m_overlappingPairArray.Count-1];
                    //m_overlappingPairArray[m_overlappingPairArray.Count-1] = temp;
                    m_overlappingPairArray.RemoveAtQuick(findIndex);
                    return userData;
                }
            }

            return null;
        }

        public void CleanOverlappingPair(BroadphasePair pair, IDispatcher dispatcher)
        {
            if (pair.m_algorithm != null)
            {
                {
                    dispatcher.FreeCollisionAlgorithm(pair.m_algorithm);
                    pair.m_algorithm = null;
                    OverlappingPairCacheGlobals.gRemovePairs--;
                }
            }
        }

        public BroadphasePair AddOverlappingPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1)
        {
            //don't add overlap with own
            Debug.Assert(proxy0 != proxy1);

            if (!NeedsBroadphaseCollision(proxy0, proxy1))
            {
                return null;
            }
            
            BroadphasePair pair = new BroadphasePair(proxy0, proxy1);
            m_overlappingPairArray.Add(pair);

            OverlappingPairCacheGlobals.gOverlappingPairs++;
            OverlappingPairCacheGlobals.gAddedPairs++;

            if (m_ghostPairCallback != null)
            {
                m_ghostPairCallback.AddOverlappingPair(proxy0, proxy1);
            }
            return pair;
        }

        public BroadphasePair FindPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1)
        {
            if (!NeedsBroadphaseCollision(proxy0, proxy1))
            {
                return null;
            }

            BroadphasePair tmpPair = new BroadphasePair(proxy0, proxy1);
            int index = m_overlappingPairArray.IndexOf(tmpPair);
            if (index != -1)
            {
                return m_overlappingPairArray[index];
            }
            return null;
        }

        public void CleanProxyFromPairs(BroadphaseProxy proxy, IDispatcher dispatcher)
        {
            CleanPairCallback cleanPairs = new CleanPairCallback(proxy, this, dispatcher);
            ProcessAllOverlappingPairs(cleanPairs, dispatcher);
        }

        public void RemoveOverlappingPairsContainingProxy(BroadphaseProxy proxy, IDispatcher dispatcher)
        {
            RemovePairCallback removeCallback = new RemovePairCallback(proxy);
            ProcessAllOverlappingPairs(removeCallback, dispatcher);
        }

        public bool NeedsBroadphaseCollision(BroadphaseProxy proxy0, BroadphaseProxy proxy1)
        {
            if (m_overlapFilterCallback != null)
            {
                return m_overlapFilterCallback.NeedBroadphaseCollision(proxy0, proxy1);
            }
            bool collides = (proxy0.m_collisionFilterGroup & proxy1.m_collisionFilterMask) != 0;
            collides = collides && ((proxy1.m_collisionFilterGroup & proxy0.m_collisionFilterMask) != 0);
            return collides;
        }

        public ObjectArray<BroadphasePair> GetOverlappingPairArray()
        {
            return m_overlappingPairArray;
        }

        public int GetNumOverlappingPairs()
        {
            return m_overlappingPairArray.Count;
        }

        public IOverlapFilterCallback GetOverlapFilterCallback()
        {
            return m_overlapFilterCallback;
        }

        public void SetOverlapFilterCallback(IOverlapFilterCallback callback)
        {
            m_overlapFilterCallback = callback;
        }

        public virtual bool HasDeferredRemoval()
        {
            return m_hasDeferredRemoval;
        }

        public virtual void SetInternalGhostPairCallback(IOverlappingPairCallback ghostPairCallback)
        {
            m_ghostPairCallback = ghostPairCallback;
        }

        public virtual void SortOverlappingPairs(IDispatcher dispatcher)
        {
            //should already be sorted
        }
        //avoid brute-force finding all the time
        protected ObjectArray<BroadphasePair> m_overlappingPairArray;

        //during the dispatch, check that user doesn't destroy/create proxy
        protected bool m_blockedForChanges;

        ///by default, do the removal during the pair traversal
        protected bool m_hasDeferredRemoval;

        //if set, use the callback instead of the built in filter in needBroadphaseCollision
        protected IOverlapFilterCallback m_overlapFilterCallback;

        protected IOverlappingPairCallback m_ghostPairCallback;

    }
    //-------------------------------------------------------------------------------------------------
    ///btNullPairCache skips add/removal of overlapping pairs. Userful for benchmarking and unit testing.
    public class NullPairCache : IOverlappingPairCache
    {
        private ObjectArray<BroadphasePair> m_overlappingPairArray = new ObjectArray<BroadphasePair>();

        //public virtual BroadphasePair	getOverlappingPairArrayPtr()
        //{
        //    return &m_overlappingPairArray[0];
        //}

        public virtual void Cleanup()
        {
        }


        public ObjectArray<BroadphasePair> GetOverlappingPairArray()
        {
            return m_overlappingPairArray;
        }

        public virtual void CleanOverlappingPair(BroadphasePair pair, IDispatcher disaptcher)
        {

        }

        public virtual int GetNumOverlappingPairs()
        {
            return 0;
        }

        public virtual void CleanProxyFromPairs(BroadphaseProxy proxy, IDispatcher dispatcher)
        {

        }

        public virtual void SetOverlapFilterCallback(IOverlapFilterCallback callback)
        {
        }

        public virtual void ProcessAllOverlappingPairs(IOverlapCallback callback, IDispatcher dispatcher)
        {
        }

        public virtual BroadphasePair FindPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1)
        {
            return null;
        }

        public virtual bool HasDeferredRemoval()
        {
            return true;
        }

        public virtual void SetInternalGhostPairCallback(IOverlappingPairCallback ghostPairCallback)
        {

        }

        public virtual BroadphasePair AddOverlappingPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1)
        {
            return null;
        }

        public virtual Object RemoveOverlappingPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1, IDispatcher dispatcher)
        {
            return null;
        }

        public virtual void RemoveOverlappingPairsContainingProxy(BroadphaseProxy proxy0, IDispatcher dispatcher)
        {
        }

        public virtual void SortOverlappingPairs(IDispatcher dispatcher)
        {
        }
    }

    public class CleanPairCallback : IOverlapCallback
    {
        private BroadphaseProxy m_cleanProxy;
        private IOverlappingPairCache m_pairCache;
        private IDispatcher m_dispatcher;

        public CleanPairCallback(BroadphaseProxy cleanProxy, IOverlappingPairCache pairCache, IDispatcher dispatcher)
        {
            m_cleanProxy = cleanProxy;
            m_pairCache = pairCache;
            m_dispatcher = dispatcher;
        }
        public virtual bool ProcessOverlap(BroadphasePair pair)
        {
            if (pair != null && ((pair.m_pProxy0 == m_cleanProxy) ||
                (pair.m_pProxy1 == m_cleanProxy)))
            {
                m_pairCache.CleanOverlappingPair(pair, m_dispatcher);
            }
            return false;
        }
    }

    public class RemovePairCallback : IOverlapCallback
    {
        private BroadphaseProxy m_obsoleteProxy;

        public RemovePairCallback(BroadphaseProxy obsoleteProxy)
        {
            m_obsoleteProxy = obsoleteProxy;
        }
        public virtual bool ProcessOverlap(BroadphasePair pair)
        {
            return (pair != null && ((pair.m_pProxy0 == m_obsoleteProxy) ||
                (pair.m_pProxy1 == m_obsoleteProxy)));
        }
    }
}