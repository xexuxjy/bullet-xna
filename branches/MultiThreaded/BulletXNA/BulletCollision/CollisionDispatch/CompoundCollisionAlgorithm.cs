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
using System.Diagnostics;
using BulletXNA.LinearMath;
using System;

namespace BulletXNA.BulletCollision
{
    public class CompoundCollisionAlgorithm : ActivatingCollisionAlgorithm
    {
        public CompoundCollisionAlgorithm() { } // for pool
        public CompoundCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1, bool isSwapped)
            : base(ci, body0, body1)
        {
            m_isSwapped = isSwapped;
            m_sharedManifold = ci.GetManifold();
            m_ownsManifold = false;
            CollisionObject colObj = m_isSwapped ? body1 : body0;
            Debug.Assert(colObj.GetCollisionShape().IsCompound());
            CompoundShape compoundShape = (CompoundShape)(colObj.GetCollisionShape());
            m_compoundShapeRevision = compoundShape.GetUpdateRevision();
            PreallocateChildAlgorithms(body0, body1);
        }

        public virtual void Initialize(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1, bool isSwapped)
        {
            base.Initialize(ci, body0, body1);
            m_isSwapped = isSwapped;
            m_sharedManifold = ci.GetManifold();
            m_ownsManifold = false;
            CollisionObject colObj = m_isSwapped ? body1 : body0;
            Debug.Assert(colObj.GetCollisionShape().IsCompound());
            CompoundShape compoundShape = (CompoundShape)(colObj.GetCollisionShape());
            m_compoundShapeRevision = compoundShape.GetUpdateRevision();
            PreallocateChildAlgorithms(body0, body1);
        }


        public override void Cleanup()
        {
            RemoveChildAlgorithms();
            m_compoundShapeRevision = 0;
            BulletGlobals.CompoundCollisionAlgorithmPool.Free(this);
        }

        private void RemoveChildAlgorithms()
        {
            int numChildren = m_childCollisionAlgorithms.Count;
            int i;
            for (i = 0; i < numChildren; i++)
            {
                if (m_childCollisionAlgorithms[i] != null)
                {
                    m_dispatcher.FreeCollisionAlgorithm(m_childCollisionAlgorithms[i]);
                    m_childCollisionAlgorithms[i] = null;
                }
            }
            m_childCollisionAlgorithms.Clear();
        }

        private void PreallocateChildAlgorithms(CollisionObject body0, CollisionObject body1)
        {
            CollisionObject colObj = m_isSwapped ? body1 : body0;
            CollisionObject otherObj = m_isSwapped ? body0 : body1;
            Debug.Assert(colObj.GetCollisionShape().IsCompound());

            CompoundShape compoundShape = (CompoundShape)(colObj.GetCollisionShape());

            int numChildren = compoundShape.GetNumChildShapes();
            int i;

            //m_childCollisionAlgorithms.resize(numChildren);
            m_childCollisionAlgorithms.Clear();
            for (i = 0; i < numChildren; i++)
            {
                if (compoundShape.GetDynamicAabbTree() != null)
                {
                    m_childCollisionAlgorithms.Add(null);
                }
                else
                {
                    CollisionShape tmpShape = colObj.GetCollisionShape();
                    CollisionShape childShape = compoundShape.GetChildShape(i);
                    colObj.InternalSetTemporaryCollisionShape(childShape);
                    CollisionAlgorithm ca = m_dispatcher.FindAlgorithm(colObj, otherObj, m_sharedManifold);
                    m_childCollisionAlgorithms.Add(ca);
                    colObj.InternalSetTemporaryCollisionShape(tmpShape);
                }
            }
        }


        public override void ProcessCollision(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            //resultOut = null;
            CollisionObject colObj = m_isSwapped ? body1 : body0;
            CollisionObject otherObj = m_isSwapped ? body0 : body1;

            Debug.Assert(colObj.GetCollisionShape().IsCompound());
            CompoundShape compoundShape = (CompoundShape)(colObj.GetCollisionShape());

            ///btCompoundShape might have changed:
            ////make sure the internal child collision algorithm caches are still valid
            if (compoundShape.GetUpdateRevision() != m_compoundShapeRevision)
            {
                ///clear and update all
                RemoveChildAlgorithms();
                PreallocateChildAlgorithms(body0, body1);
            }


            Dbvt tree = compoundShape.GetDynamicAabbTree();
            //use a dynamic aabb tree to cull potential child-overlaps
            using (CompoundLeafCallback callback = BulletGlobals.CompoundLeafCallbackPool.Get())
            {
                callback.Initialize(colObj, otherObj, m_dispatcher, dispatchInfo, resultOut, this, m_childCollisionAlgorithms, m_sharedManifold);

                ///we need to refresh all contact manifolds
                ///note that we should actually recursively traverse all children, btCompoundShape can nested more then 1 level deep
                ///so we should add a 'refreshManifolds' in the btCollisionAlgorithm
                {
                    m_manifoldArray.Clear();
                    for (int i = 0; i < m_childCollisionAlgorithms.Count; i++)
                    {
                        if (m_childCollisionAlgorithms[i] != null)
                        {
                            m_childCollisionAlgorithms[i].GetAllContactManifolds(m_manifoldArray);
                            for (int m = 0; m < m_manifoldArray.Count; m++)
                            {
                                if (m_manifoldArray[m].GetNumContacts() > 0)
                                {
                                    resultOut.SetPersistentManifold(m_manifoldArray[m]);
                                    resultOut.RefreshContactPoints();
                                    resultOut.SetPersistentManifold(null);//??necessary?
                                }
                            }
                            m_manifoldArray.Clear();
                        }
                    }
                }

                if (tree != null)
                {

                    IndexedVector3 localAabbMin;
                    IndexedVector3 localAabbMax;
                    IndexedMatrix otherInCompoundSpace;
                    //otherInCompoundSpace = MathUtil.BulletMatrixMultiply(colObj.GetWorldTransform(),otherObj.GetWorldTransform());
                    otherInCompoundSpace = colObj.GetWorldTransform().Inverse() * otherObj.GetWorldTransform();

                    otherObj.GetCollisionShape().GetAabb(ref otherInCompoundSpace, out localAabbMin, out localAabbMax);

                    DbvtAabbMm bounds = DbvtAabbMm.FromMM(ref localAabbMin, ref localAabbMax);
                    //process all children, that overlap with  the given AABB bounds
                    Dbvt.CollideTV(tree.m_root, ref bounds, callback);

                }
                else
                {
                    //iterate over all children, perform an AABB check inside ProcessChildShape
                    int numChildren = m_childCollisionAlgorithms.Count;
                    for (int i = 0; i < numChildren; i++)
                    {
                        callback.ProcessChildShape(compoundShape.GetChildShape(i), i);
                    }
                }

                {
                    //iterate over all children, perform an AABB check inside ProcessChildShape
                    int numChildren = m_childCollisionAlgorithms.Count;

                    m_manifoldArray.Clear();
                    CollisionShape childShape = null;
                    IndexedMatrix orgTrans;
                    IndexedMatrix orgInterpolationTrans;
                    IndexedMatrix newChildWorldTrans;


                    for (int i = 0; i < numChildren; i++)
                    {
                        if (m_childCollisionAlgorithms[i] != null)
                        {
                            childShape = compoundShape.GetChildShape(i);
                            //if not longer overlapping, remove the algorithm
                            orgTrans = colObj.GetWorldTransform();
                            orgInterpolationTrans = colObj.GetInterpolationWorldTransform();
                            IndexedMatrix childTrans = compoundShape.GetChildTransform(i);

                            newChildWorldTrans = orgTrans * childTrans;

                            //perform an AABB check first
                            IndexedVector3 aabbMin0;
                            IndexedVector3 aabbMax0;
                            IndexedVector3 aabbMin1;
                            IndexedVector3 aabbMax1;

                            childShape.GetAabb(ref newChildWorldTrans, out aabbMin0, out aabbMax0);
                            otherObj.GetCollisionShape().GetAabb(otherObj.GetWorldTransform(), out aabbMin1, out aabbMax1);

                            if (!AabbUtil2.TestAabbAgainstAabb2(ref aabbMin0, ref aabbMax0, ref aabbMin1, ref aabbMax1))
                            {
                                m_dispatcher.FreeCollisionAlgorithm(m_childCollisionAlgorithms[i]);
                                m_childCollisionAlgorithms[i] = null;
                            }
                        }
                    }
                }
            }
        }

        public override void GetAllContactManifolds(PersistentManifoldArray manifoldArray)
        {
            for (int i = 0; i < m_childCollisionAlgorithms.Count; i++)
            {
                if (m_childCollisionAlgorithms[i] != null)
                {
                    if (m_childCollisionAlgorithms[i] == this)
                    {
                        int ibreak = 0;
                    }
                    m_childCollisionAlgorithms[i].GetAllContactManifolds(manifoldArray);
                }
            }
        }

        public override float CalculateTimeOfImpact(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            resultOut = null;
            CollisionObject colObj = m_isSwapped ? body1 : body0;
            CollisionObject otherObj = m_isSwapped ? body0 : body1;

            Debug.Assert(colObj.GetCollisionShape().IsCompound());

            CompoundShape compoundShape = (CompoundShape)(colObj.GetCollisionShape());

            //We will use the OptimizedBVH, AABB tree to cull potential child-overlaps
            //If both proxies are Compound, we will deal with that directly, by performing sequential/parallel tree traversals
            //given Proxy0 and Proxy1, if both have a tree, Tree0 and Tree1, this means:
            //determine overlapping nodes of Proxy1 using Proxy0 AABB against Tree1
            //then use each overlapping node AABB against Tree0
            //and vise versa.

            float hitFraction = 1f;

            int numChildren = m_childCollisionAlgorithms.Count;
            IndexedMatrix orgTrans;
            float frac;

            for (int i = 0; i < numChildren; i++)
            {
                //temporarily exchange parent btCollisionShape with childShape, and recurse
                CollisionShape childShape = compoundShape.GetChildShape(i);

                //backup
                orgTrans = colObj.GetWorldTransform();

                IndexedMatrix childTrans = compoundShape.GetChildTransform(i);
                IndexedMatrix newChildWorldTrans = orgTrans * childTrans;

                colObj.SetWorldTransform(ref newChildWorldTrans);

                CollisionShape tmpShape = colObj.GetCollisionShape();
                colObj.InternalSetTemporaryCollisionShape(childShape);

                frac = m_childCollisionAlgorithms[i].CalculateTimeOfImpact(colObj, otherObj, dispatchInfo, resultOut);
                if (frac < hitFraction)
                {
                    hitFraction = frac;
                }
                //revert back
                colObj.InternalSetTemporaryCollisionShape(tmpShape);
                colObj.SetWorldTransform(ref orgTrans);
            }
            return hitFraction;

        }



        private IList<CollisionAlgorithm> m_childCollisionAlgorithms = new List<CollisionAlgorithm>();
        private PersistentManifoldArray m_manifoldArray = new PersistentManifoldArray();

        private bool m_isSwapped;

        private PersistentManifold m_sharedManifold;
        private bool m_ownsManifold;

        private int m_compoundShapeRevision;//to keep track of changes, so that childAlgorithm array can be updated
    }


    public class CompoundLeafCallback : DefaultCollide, IDisposable
    {
        public CollisionObject m_compoundColObj;
        public CollisionObject m_otherObj;
        public IDispatcher m_dispatcher;
        public DispatcherInfo m_dispatchInfo;
        public ManifoldResult m_resultOut;
        public IList<CollisionAlgorithm> m_childCollisionAlgorithms;
        public PersistentManifold m_sharedManifold;
        public CompoundCollisionAlgorithm m_parent;

        public CompoundLeafCallback() { } // for pool
        public CompoundLeafCallback(CollisionObject compoundObj, CollisionObject otherObj, IDispatcher dispatcher, DispatcherInfo dispatchInfo, ManifoldResult resultOut, CompoundCollisionAlgorithm parent,IList<CollisionAlgorithm> childCollisionAlgorithms, PersistentManifold sharedManifold)
        {
            m_compoundColObj = compoundObj;
            m_otherObj = otherObj;
            m_dispatcher = dispatcher;
            m_dispatchInfo = dispatchInfo;
            m_resultOut = resultOut;
            m_childCollisionAlgorithms = childCollisionAlgorithms;
            m_sharedManifold = sharedManifold;
            m_parent = parent;
        }

        public void Initialize(CollisionObject compoundObj, CollisionObject otherObj, IDispatcher dispatcher, DispatcherInfo dispatchInfo, ManifoldResult resultOut, CompoundCollisionAlgorithm parent, IList<CollisionAlgorithm> childCollisionAlgorithms, PersistentManifold sharedManifold)
        {
            m_compoundColObj = compoundObj;
            m_otherObj = otherObj;
            m_dispatcher = dispatcher;
            m_dispatchInfo = dispatchInfo;
            m_resultOut = resultOut;
            m_childCollisionAlgorithms = childCollisionAlgorithms;
            m_sharedManifold = sharedManifold;
            m_parent = parent;
        }


        public void ProcessChildShape(CollisionShape childShape, int index)
        {
            Debug.Assert(index >= 0);
            CompoundShape compoundShape = (CompoundShape)(m_compoundColObj.GetCollisionShape());
            Debug.Assert(index < compoundShape.GetNumChildShapes());

            //backup
            IndexedMatrix orgTrans = m_compoundColObj.GetWorldTransform();
            IndexedMatrix orgInterpolationTrans = m_compoundColObj.GetInterpolationWorldTransform();
            IndexedMatrix childTrans = compoundShape.GetChildTransform(index);
            IndexedMatrix newChildWorldTrans = orgTrans * childTrans;

            //perform an AABB check first
            IndexedVector3 aabbMin0;
            IndexedVector3 aabbMax0;
            IndexedVector3 aabbMin1;
            IndexedVector3 aabbMax1;

            childShape.GetAabb(ref newChildWorldTrans, out aabbMin0, out aabbMax0);
            m_otherObj.GetCollisionShape().GetAabb(m_otherObj.GetWorldTransform(), out aabbMin1, out aabbMax1);

            if (AabbUtil2.TestAabbAgainstAabb2(ref aabbMin0, ref aabbMax0, ref aabbMin1, ref aabbMax1))
            {
                m_compoundColObj.SetWorldTransform(ref newChildWorldTrans);
                m_compoundColObj.SetInterpolationWorldTransform(ref newChildWorldTrans);

                //the contactpoint is still projected back using the original inverted worldtrans
                CollisionShape tmpShape = m_compoundColObj.GetCollisionShape();
                m_compoundColObj.InternalSetTemporaryCollisionShape(childShape);

                if (m_childCollisionAlgorithms[index] == null)
                {
                    m_childCollisionAlgorithms[index] = m_dispatcher.FindAlgorithm(m_compoundColObj, m_otherObj, m_sharedManifold);
                    if (m_childCollisionAlgorithms[index] == m_parent)
                    {
                        int ibreak = 0;
                    }
                }

                ///detect swapping case
                if (m_resultOut.GetBody0Internal() == m_compoundColObj)
                {
                    m_resultOut.SetShapeIdentifiersA(-1, index);
                }
                else
                {
                    m_resultOut.SetShapeIdentifiersB(-1, index);
                }


                m_childCollisionAlgorithms[index].ProcessCollision(m_compoundColObj, m_otherObj, m_dispatchInfo, m_resultOut);
                if (m_dispatchInfo.getDebugDraw() != null && (((m_dispatchInfo.getDebugDraw().GetDebugMode() & DebugDrawModes.DBG_DrawAabb)) != 0))
                {
                    IndexedVector3 worldAabbMin = IndexedVector3.Zero, worldAabbMax = IndexedVector3.Zero;
                    m_dispatchInfo.getDebugDraw().DrawAabb(aabbMin0, aabbMax0, new IndexedVector3(1, 1, 1));
                    m_dispatchInfo.getDebugDraw().DrawAabb(aabbMin1, aabbMax1, new IndexedVector3(1, 1, 1));
                }

                //revert back transform 
                m_compoundColObj.InternalSetTemporaryCollisionShape(tmpShape);
                m_compoundColObj.SetWorldTransform(ref orgTrans);
                m_compoundColObj.SetInterpolationWorldTransform(ref orgInterpolationTrans);
            }
        }

        public override void Process(DbvtNode leaf)
        {
            int index = leaf.dataAsInt;

            CompoundShape compoundShape = (CompoundShape)(m_compoundColObj.GetCollisionShape());
            CollisionShape childShape = compoundShape.GetChildShape(index);
            if (m_dispatchInfo.getDebugDraw() != null && (((m_dispatchInfo.getDebugDraw().GetDebugMode() & DebugDrawModes.DBG_DrawAabb)) != 0))
            {
                IndexedVector3 worldAabbMin;
                IndexedVector3 worldAabbMax;
                IndexedMatrix orgTrans = m_compoundColObj.GetWorldTransform();

                AabbUtil2.TransformAabb(leaf.volume.Mins(), leaf.volume.Maxs(), 0f, ref orgTrans, out worldAabbMin, out worldAabbMax);
                m_dispatchInfo.getDebugDraw().DrawAabb(worldAabbMin, worldAabbMax, new IndexedVector3(1, 0, 0));
            }
            ProcessChildShape(childShape, index);
        }

        #region IDisposable Members

        public void Dispose()
        {
            BulletGlobals.CompoundLeafCallbackPool.Free(this);
        }

        #endregion
    }

    public class CompoundCreateFunc : CollisionAlgorithmCreateFunc
    {
        public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
        {
            CompoundCollisionAlgorithm algo = BulletGlobals.CompoundCollisionAlgorithmPool.Get();
            algo.Initialize(ci, body0, body1, false);
            return algo;
        }
    }

    public class SwappedCompoundCreateFunc : CollisionAlgorithmCreateFunc
    {
        public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
        {
            CompoundCollisionAlgorithm algo = BulletGlobals.CompoundCollisionAlgorithmPool.Get();
            algo.Initialize(ci, body0, body1, true);
            return algo;
        }
    }


}
