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
using BulletXNA.BulletCollision.BroadphaseCollision;
using BulletXNA.BulletCollision.NarrowPhaseCollision;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision.CollisionDispatch
{
    ///SimulationIslandManager creates and handles simulation islands, using btUnionFind
    public class SimulationIslandManager
    {
        private UnionFind m_unionFind;
        private ObjectArray<PersistentManifold> m_islandmanifold;
        private ObjectArray<CollisionObject> m_islandBodies;

        private bool m_splitIslands;

        private static bool debugIslands = false;

        public SimulationIslandManager()
        {
            m_splitIslands = true;
            m_unionFind = new UnionFind();
            m_islandmanifold = new ObjectArray<PersistentManifold>();
            m_islandBodies = new ObjectArray<CollisionObject>();
        }

        public virtual void Cleanup()
        {
        }

        public void InitUnionFind(int n)
        {
            m_unionFind.Reset(n);
        }

        public UnionFind GetUnionFind()
        {
            return m_unionFind;
        }

        public virtual void UpdateActivationState(CollisionWorld collisionWorld, IDispatcher dispatcher)
        {
            InitUnionFind(collisionWorld.GetCollisionObjectArray().Count);

            // put the index into m_controllers into m_tag	
            {
                int index = 0;
                ObjectArray<CollisionObject> list = collisionWorld.GetCollisionObjectArray();
                int length = list.Count;
                CollisionObject[] rawList = list.GetRawArray();
                for (int i = 0; i < length; ++i)
                {
                    CollisionObject collisionObject = rawList[i];
                    collisionObject.SetIslandTag(index);
                    collisionObject.SetCompanionId(-1);
                    collisionObject.SetHitFraction(1f);
                    index++;
                }
            }
            // do the union find

            FindUnions(dispatcher, collisionWorld);

        }
        public virtual void StoreIslandActivationState(CollisionWorld collisionWorld)
        {
            int index = 0;
            ObjectArray<CollisionObject> list = collisionWorld.GetCollisionObjectArray();
            int length = list.Count;
            CollisionObject[] rawList = list.GetRawArray();
            for (int i = 0; i < length; ++i)
            {
                CollisionObject collisionObject = rawList[i];
                if (!collisionObject.IsStaticOrKinematicObject())
                {
                    collisionObject.SetIslandTag(m_unionFind.Find(index));
                    collisionObject.SetCompanionId(-1);
                }
                else
                {
                    collisionObject.SetIslandTag(-1);
                    collisionObject.SetCompanionId(-2);
                }
                index++;
            }

        }

        public void FindUnions(IDispatcher dispatcher, CollisionWorld collisionWorld)
        {
            ObjectArray<BroadphasePair> list = collisionWorld.GetPairCache().GetOverlappingPairArray();
            int length = list.Count;
            BroadphasePair[] rawList = list.GetRawArray();
            for (int i = 0; i < length; ++i)
            {
                BroadphasePair collisionPair = rawList[i];
                CollisionObject colObj0 = collisionPair.m_pProxy0.m_clientObject as CollisionObject;
                CollisionObject colObj1 = collisionPair.m_pProxy1.m_clientObject as CollisionObject;

                if (((colObj0 != null) && ((colObj0).MergesSimulationIslands())) &&
                    ((colObj1 != null) && ((colObj1).MergesSimulationIslands())))
                {

                    m_unionFind.Unite((colObj0).GetIslandTag(),
                        (colObj1).GetIslandTag());
                }
            }
        }

        static Comparison<PersistentManifold> sortPredicate = new Comparison<PersistentManifold>(PersistentManifoldSortPredicate);
        public void BuildAndProcessIslands(IDispatcher dispatcher, CollisionWorld collisionWorld, IIslandCallback callback)
        {
            ObjectArray<CollisionObject> collisionObjects = collisionWorld.GetCollisionObjectArray();

            BuildIslands(dispatcher, collisionWorld);

            int endIslandIndex = 1;
            int startIslandIndex;
            int numElem = GetUnionFind().GetNumElements();

            BulletGlobals.StartProfile("processIslands");

            if (!m_splitIslands)
            {
                ObjectArray<PersistentManifold> manifolds = dispatcher.GetInternalManifoldPointer();
                int maxNumManifolds = dispatcher.GetNumManifolds();
                callback.ProcessIsland(collisionObjects, collisionObjects.Count, manifolds, maxNumManifolds, -1);
            }
            else
            {
                // Sort manifolds, based on islands
                // Sort the vector using predicate and std::sort
                //std::sort(islandmanifold.begin(), islandmanifold.end(), btPersistentManifoldSortPredicate);

                int numManifolds = m_islandmanifold.Count;

                //we should do radix sort, it it much faster (O(n) instead of O (n log2(n))
                //m_islandmanifold.quickSort(btPersistentManifoldSortPredicate());
                //((ObjectArray<PersistentManifold>)m_islandmanifold).Sort();
                m_islandmanifold.Sort(sortPredicate);

                //now process all active islands (sets of manifolds for now)

                int startManifoldIndex = 0;
                int endManifoldIndex = 1;
                //traverse the simulation islands, and call the solver, unless all objects are sleeping/deactivated
                for (startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex)
                {
                    int islandId = GetUnionFind().GetElement(startIslandIndex).m_id;

                    if (BulletGlobals.g_streamWriter != null && debugIslands)
                    {
                        BulletGlobals.g_streamWriter.WriteLine(String.Format("buildAndProcessIslands start[{0}] end[{1}] id[{2}]", startIslandIndex, endIslandIndex, islandId));
                    }


                    bool islandSleeping = false;

                    for (endIslandIndex = startIslandIndex; (endIslandIndex < numElem) && (GetUnionFind().GetElement(endIslandIndex).m_id == islandId); endIslandIndex++)
                    {
                        int i = GetUnionFind().GetElement(endIslandIndex).m_sz;
                        CollisionObject colObj0 = collisionObjects[i];
                        m_islandBodies.Add(colObj0);
                        if (!colObj0.IsActive())
                        {
                            islandSleeping = true;
                            //islandSleeping = false;
                        }
                    }

                    //find the accompanying contact manifold for this islandId
                    int numIslandManifolds = 0;
                    PersistentManifold startManifold = null;

                    if (startManifoldIndex < numManifolds)
                    {
                        int curIslandId = GetIslandId(m_islandmanifold[startManifoldIndex]);
                        if (curIslandId == islandId)
                        {
                            startManifold = m_islandmanifold[startManifoldIndex];

                            for (endManifoldIndex = startManifoldIndex + 1; (endManifoldIndex < numManifolds) && (islandId == GetIslandId(m_islandmanifold[endManifoldIndex])); endManifoldIndex++)
                            {

                            }
                            /// Process the actual simulation, only if not sleeping/deactivated
                            numIslandManifolds = endManifoldIndex - startManifoldIndex;
                        }

                    }

                    if (!islandSleeping)
                    {
                        // pass shortedned list to callback.
                        ObjectArray<PersistentManifold> subList = new ObjectArray<PersistentManifold>();
                        for (int i = 0; i < numIslandManifolds; ++i)
                        {
                            subList.Add(m_islandmanifold[startManifoldIndex + i]);
                        }
                        callback.ProcessIsland(m_islandBodies, m_islandBodies.Count, subList, numIslandManifolds, islandId);
                        //			printf("Island callback of size:%d bodies, %d manifolds\n",islandBodies.size(),numIslandManifolds);
                    }

                    if (numIslandManifolds != 0)
                    {
                        startManifoldIndex = endManifoldIndex;
                    }

                    m_islandBodies.Clear();
                }
            } // else if(!splitIslands) 
            BulletGlobals.StopProfile();
        }

        public void BuildIslands(IDispatcher dispatcher, CollisionWorld collisionWorld)
        {
            BulletGlobals.StartProfile("islandUnionFindAndQuickSort");

            ObjectArray<CollisionObject> collisionObjects = collisionWorld.GetCollisionObjectArray();

            m_islandmanifold.Clear();

            //we are going to sort the unionfind array, and store the element id in the size
            //afterwards, we clean unionfind, to make sure no-one uses it anymore

            GetUnionFind().sortIslands();
            int numElem = GetUnionFind().GetNumElements();

            int endIslandIndex = 1;


            //update the sleeping state for bodies, if all are sleeping
            for (int startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex)
            {
                int islandId = GetUnionFind().GetElement(startIslandIndex).m_id;

                if (BulletGlobals.g_streamWriter != null && debugIslands)
                {
                    BulletGlobals.g_streamWriter.WriteLine(String.Format("buildIslands start[{0}] end[{1}] id[{2}]", startIslandIndex, endIslandIndex, islandId));
                }

                for (endIslandIndex = startIslandIndex + 1; (endIslandIndex < numElem) && (GetUnionFind().GetElement(endIslandIndex).m_id == islandId); endIslandIndex++)
                {
                }

                //int numSleeping = 0;

                bool allSleeping = true;

                for (int idx = startIslandIndex; idx < endIslandIndex; idx++)
                {
                    int i = GetUnionFind().GetElement(idx).m_sz;

                    CollisionObject colObj0 = collisionObjects[i];
                    if ((colObj0.GetIslandTag() != islandId) && (colObj0.GetIslandTag() != -1))
                    {
                        //				printf("error in island management\n");
                    }

                    Debug.Assert((colObj0.GetIslandTag() == islandId) || (colObj0.GetIslandTag() == -1));
                    if (colObj0.GetIslandTag() == islandId)
                    {
                        if (colObj0.GetActivationState() == ActivationState.ACTIVE_TAG)
                        {
                            allSleeping = false;
                        }
                        if (colObj0.GetActivationState() == ActivationState.DISABLE_DEACTIVATION)
                        {
                            allSleeping = false;
                        }
                    }
                }


                if (allSleeping)
                {
                    for (int idx = startIslandIndex; idx < endIslandIndex; idx++)
                    {
                        int i = GetUnionFind().GetElement(idx).m_sz;
                        CollisionObject colObj0 = collisionObjects[i];
                        if ((colObj0.GetIslandTag() != islandId) && (colObj0.GetIslandTag() != -1))
                        {
                            //					printf("error in island management\n");
                        }

                        Debug.Assert((colObj0.GetIslandTag() == islandId) || (colObj0.GetIslandTag() == -1));

                        if (colObj0.GetIslandTag() == islandId)
                        {
                            colObj0.SetActivationState(ActivationState.ISLAND_SLEEPING);
                        }
                    }
                }
                else
                {
                    for (int idx = startIslandIndex; idx < endIslandIndex; idx++)
                    {
                        int i = GetUnionFind().GetElement(idx).m_sz;

                        CollisionObject colObj0 = collisionObjects[i];
                        if ((colObj0.GetIslandTag() != islandId) && (colObj0.GetIslandTag() != -1))
                        {
                            //					printf("error in island management\n");
                        }

                        Debug.Assert((colObj0.GetIslandTag() == islandId) || (colObj0.GetIslandTag() == -1));

                        if (colObj0.GetIslandTag() == islandId)
                        {
                            if (colObj0.GetActivationState() == ActivationState.ISLAND_SLEEPING)
                            {
                                colObj0.SetActivationState(ActivationState.WANTS_DEACTIVATION);
                                colObj0.SetDeactivationTime(0f);
                            }
                        }
                    }
                }
            }

            int maxNumManifolds = dispatcher.GetNumManifolds();

            //#definef SPLIT_ISLANDS 1
            //#ifdef SPLIT_ISLANDS


            //#endif //SPLIT_ISLANDS


            for (int i = 0; i < maxNumManifolds; i++)
            {
                PersistentManifold manifold = dispatcher.GetManifoldByIndexInternal(i);

                CollisionObject colObj0 = manifold.GetBody0() as CollisionObject;
                CollisionObject colObj1 = manifold.GetBody1() as CollisionObject;

                ///@todo: check sleeping conditions!
                if (((colObj0 != null) && colObj0.GetActivationState() != ActivationState.ISLAND_SLEEPING) ||
                   ((colObj1 != null) && colObj1.GetActivationState() != ActivationState.ISLAND_SLEEPING))
                {

                    //kinematic objects don't merge islands, but wake up all connected objects
                    if (colObj0.IsKinematicObject() && colObj0.GetActivationState() != ActivationState.ISLAND_SLEEPING)
                    {
                        colObj1.Activate();
                    }
                    if (colObj1.IsKinematicObject() && colObj1.GetActivationState() != ActivationState.ISLAND_SLEEPING)
                    {
                        colObj0.Activate();
                    }
                    if (m_splitIslands)
                    {
                        //filtering for response
                        if (dispatcher.NeedsResponse(colObj0, colObj1))
                        {
                            m_islandmanifold.Add(manifold);
                        }
                    }
                }
            }
            BulletGlobals.StopProfile();
        }

        public bool GetSplitIslands()
        {
            return m_splitIslands;
        }

        public void SetSplitIslands(bool doSplitIslands)
        {
            m_splitIslands = doSplitIslands;
        }


        public static int GetIslandId(PersistentManifold lhs)
        {
            CollisionObject rcolObj0 = lhs.GetBody0() as CollisionObject;
            int islandId = rcolObj0.GetIslandTag();
            if (islandId >= 0)
                return islandId;

            CollisionObject rcolObj1 = lhs.GetBody1() as CollisionObject;
            return rcolObj1.GetIslandTag();
        }


        private static int PersistentManifoldSortPredicate(PersistentManifold lhs, PersistentManifold rhs)
        {
            int rIslandIdA, lIslandIdB;
            rIslandIdA = GetIslandId(rhs);
            lIslandIdB = GetIslandId(lhs);
            //return lIslandId0 < rIslandId0;
            if (lIslandIdB < rIslandIdA)
                return -1;
            //else if (lIslandIdB > rIslandIdA)
            //    return 1;
            return 1;
        }


    }

    public interface IIslandCallback
    {
        void ProcessIsland(ObjectArray<CollisionObject> bodies, int numBodies, ObjectArray<PersistentManifold> manifolds, int numManifolds, int islandId);
    }


}
