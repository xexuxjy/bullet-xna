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

using BulletXNA.BullettCollision.BroadphaseCollision;
using System.Diagnostics;
using BulletXNA.BullettCollision.CollisionDispatch;
using BulletXNA.BullettCollision.CollisionShapes;
using Microsoft.Xna.Framework;
using BulletXNA.BulletCollision.CollisionShapes;
using BulletXNA.BulletCollision.BroadphaseCollision;
using BulletXNA.BulletCollision.NarrowPhaseCollision;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision.CollisionDispatch
{
    public class CompoundCollisionAlgorithm : ActivatingCollisionAlgorithm
    {
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

        public override void Cleanup()
        {
            RemoveChildAlgorithms();
        }

        private void RemoveChildAlgorithms()
        {
            int numChildren = m_childCollisionAlgorithms.Count;
            int i;
            for (i = 0; i < numChildren; i++)
            {
                if (m_childCollisionAlgorithms[i] != null)
                {
                    m_childCollisionAlgorithms[i].Cleanup();
                    m_dispatcher.FreeCollisionAlgorithm(m_childCollisionAlgorithms[i]);
                    m_childCollisionAlgorithms[i] = null;
                }
            }
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
                    m_childCollisionAlgorithms.Add(m_dispatcher.FindAlgorithm(colObj, otherObj, m_sharedManifold));
                    colObj.InternalSetTemporaryCollisionShape(tmpShape);
                }
            }
        }


        public override void ProcessCollision(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            //resultOut = null;
	        CollisionObject colObj = m_isSwapped? body1 : body0;
	        CollisionObject otherObj = m_isSwapped? body0 : body1;

	        Debug.Assert (colObj.GetCollisionShape().IsCompound());
	        CompoundShape compoundShape = (CompoundShape)(colObj.GetCollisionShape());

	        ///btCompoundShape might have changed:
	        ////make sure the internal child collision algorithm caches are still valid
	        if (compoundShape.GetUpdateRevision() != m_compoundShapeRevision)
	        {
		        ///clear and update all
		        RemoveChildAlgorithms();
		        PreallocateChildAlgorithms(body0,body1);
	        }


	        Dbvt tree = compoundShape.GetDynamicAabbTree();
	        //use a dynamic aabb tree to cull potential child-overlaps
	        CompoundLeafCallback  callback = new CompoundLeafCallback(colObj,otherObj,m_dispatcher,dispatchInfo,resultOut,m_childCollisionAlgorithms,m_sharedManifold);

	        ///we need to refresh all contact manifolds
	        ///note that we should actually recursively traverse all children, btCompoundShape can nested more then 1 level deep
	        ///so we should add a 'refreshManifolds' in the btCollisionAlgorithm
	        {
                IList<PersistentManifold> manifoldArray = new List<PersistentManifold>();
		        for (int i=0;i<m_childCollisionAlgorithms.Count;i++)
		        {
			        if (m_childCollisionAlgorithms[i] != null)
			        {
				        m_childCollisionAlgorithms[i].GetAllContactManifolds(manifoldArray);
				        for (int m=0;m<manifoldArray.Count;m++)
				        {
					        if (manifoldArray[m].GetNumContacts() > 0)
					        {
						        resultOut.SetPersistentManifold(manifoldArray[m]);
						        resultOut.RefreshContactPoints();
						        resultOut.SetPersistentManifold(null);//??necessary?
					        }
				        }
				        manifoldArray.Clear();
			        }
		        }
	        }

	        if (tree != null)
	        {

		        Vector3 localAabbMin = new Vector3();
                Vector3 localAabbMax = new Vector3();
		        Matrix otherInCompoundSpace = Matrix.Identity;
				//otherInCompoundSpace = MathUtil.BulletMatrixMultiply(colObj.GetWorldTransform(),otherObj.GetWorldTransform());
				otherInCompoundSpace = MathUtil.InverseTimes(colObj.GetWorldTransform(), otherObj.GetWorldTransform());

		        otherObj.GetCollisionShape().GetAabb(ref otherInCompoundSpace,ref localAabbMin,ref localAabbMax);

                DbvtAabbMm bounds = DbvtAabbMm.FromMM(ref localAabbMin, ref localAabbMax);
		        //process all children, that overlap with  the given AABB bounds
		        Dbvt.CollideTV(tree.m_root,ref bounds,callback);

	        } 
            else
	        {
		        //iterate over all children, perform an AABB check inside ProcessChildShape
		        int numChildren = m_childCollisionAlgorithms.Count;
		        for (int i=0;i<numChildren;i++)
		        {
			        callback.ProcessChildShape(compoundShape.GetChildShape(i),i);
		        }
	        }

	        {
		        //iterate over all children, perform an AABB check inside ProcessChildShape
		        int numChildren = m_childCollisionAlgorithms.Count;

		        IList<PersistentManifold> manifoldArray = new List<PersistentManifold>();

		        for (int i=0;i<numChildren;i++)
		        {
			        if (m_childCollisionAlgorithms[i] != null)
			        {
				        CollisionShape childShape = compoundShape.GetChildShape(i);
			            //if not longer overlapping, remove the algorithm
				        Matrix orgTrans = colObj.GetWorldTransform();
				        Matrix orgInterpolationTrans = colObj.GetInterpolationWorldTransform();
				        Matrix childTrans = compoundShape.GetChildTransform(i);

						Matrix newChildWorldTrans = MathUtil.BulletMatrixMultiply(ref orgTrans, ref childTrans);

				        //perform an AABB check first
				        Vector3 aabbMin0 = new Vector3();
                        Vector3 aabbMax0 = new Vector3();
                        Vector3 aabbMin1 = new Vector3();
                        Vector3 aabbMax1 = new Vector3();
                            
				        childShape.GetAabb(ref newChildWorldTrans,ref aabbMin0,ref aabbMax0);
				        otherObj.GetCollisionShape().GetAabb(otherObj.GetWorldTransform(),ref aabbMin1,ref aabbMax1);

				        if (!AabbUtil2.TestAabbAgainstAabb2(ref aabbMin0,ref aabbMax0,ref aabbMin1,ref aabbMax1))
				        {
			                m_dispatcher.FreeCollisionAlgorithm(m_childCollisionAlgorithms[i]);
					        m_childCollisionAlgorithms[i] = null;
				        }
			        }
		        }
	        }
        }

        public override void GetAllContactManifolds(IList<PersistentManifold> manifoldArray)
        {
            for (int i = 0; i < m_childCollisionAlgorithms.Count; i++)
            {
                if (m_childCollisionAlgorithms[i] != null)
                {
                    m_childCollisionAlgorithms[i].GetAllContactManifolds(manifoldArray);
                }
            }
        }

        public override float CalculateTimeOfImpact(CollisionObject body0,CollisionObject body1,DispatcherInfo dispatchInfo,ManifoldResult resultOut)
        {
            resultOut = null;
	        CollisionObject colObj = m_isSwapped? body1 : body0;
	        CollisionObject otherObj = m_isSwapped? body0 : body1;

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
	        for (int i=0;i<numChildren;i++)
	        {
		        //temporarily exchange parent btCollisionShape with childShape, and recurse
		        CollisionShape childShape = compoundShape.GetChildShape(i);

		        //backup
		        Matrix orgTrans = colObj.GetWorldTransform();
        	
		        Matrix childTrans = compoundShape.GetChildTransform(i);
				Matrix newChildWorldTrans = MathUtil.BulletMatrixMultiply(ref orgTrans,ref childTrans);
					
                colObj.SetWorldTransform(ref newChildWorldTrans);

		        CollisionShape tmpShape = colObj.GetCollisionShape();
		        colObj.InternalSetTemporaryCollisionShape( childShape );
                
                float frac = m_childCollisionAlgorithms[i].CalculateTimeOfImpact(colObj,otherObj,dispatchInfo, resultOut);
		        if (frac<hitFraction)
		        {
			        hitFraction = frac;
		        }
		        //revert back
		        colObj.InternalSetTemporaryCollisionShape( tmpShape);
		        colObj.SetWorldTransform(ref orgTrans);
	        }
	        return hitFraction;

        }



        private IList<CollisionAlgorithm> m_childCollisionAlgorithms = new List<CollisionAlgorithm>();
        private bool m_isSwapped;

        private PersistentManifold m_sharedManifold;
        private bool m_ownsManifold;

        private int m_compoundShapeRevision;//to keep track of changes, so that childAlgorithm array can be updated
    }


    public class CompoundLeafCallback : Collide
    {
	    public CollisionObject m_compoundColObj;
	    public CollisionObject m_otherObj;
	    public IDispatcher m_dispatcher;
	    public DispatcherInfo m_dispatchInfo;
	    public ManifoldResult	m_resultOut;
	    public IList<CollisionAlgorithm>	m_childCollisionAlgorithms;
	    public PersistentManifold	m_sharedManifold;

	    public CompoundLeafCallback (CollisionObject compoundObj,CollisionObject otherObj,IDispatcher dispatcher,DispatcherInfo dispatchInfo,ManifoldResult resultOut,IList<CollisionAlgorithm> childCollisionAlgorithms,PersistentManifold sharedManifold)
	    {
            m_compoundColObj = compoundObj;
            m_otherObj = otherObj;
            m_dispatcher = dispatcher;
            m_dispatchInfo = dispatchInfo;
            m_resultOut = resultOut;
            m_childCollisionAlgorithms = childCollisionAlgorithms;
            m_sharedManifold = sharedManifold;
	    }

	    public void ProcessChildShape(CollisionShape childShape,int index)
	    {
		    CompoundShape compoundShape = (CompoundShape)(m_compoundColObj.GetCollisionShape());


		    //backup
		    Matrix orgTrans = m_compoundColObj.GetWorldTransform();
		    Matrix orgInterpolationTrans = m_compoundColObj.GetInterpolationWorldTransform();
		    Matrix childTrans = compoundShape.GetChildTransform(index);
			Matrix	newChildWorldTrans = MathUtil.BulletMatrixMultiply(ref orgTrans,ref childTrans);

		    //perform an AABB check first
		    Vector3 aabbMin0 = Vector3.Zero;
            Vector3 aabbMax0 = Vector3.Zero;
            Vector3 aabbMin1 = Vector3.Zero;
            Vector3 aabbMax1 = Vector3.Zero;

		    childShape.GetAabb(ref newChildWorldTrans,ref aabbMin0,ref aabbMax0);
		    m_otherObj.GetCollisionShape().GetAabb(m_otherObj.GetWorldTransform(),ref aabbMin1,ref aabbMax1);

		    if (AabbUtil2.TestAabbAgainstAabb2(ref aabbMin0,ref aabbMax0,ref aabbMin1,ref aabbMax1))
		    {
			    m_compoundColObj.SetWorldTransform(ref newChildWorldTrans);
			    m_compoundColObj.SetInterpolationWorldTransform(ref newChildWorldTrans);

			    //the contactpoint is still projected back using the original inverted worldtrans
			    CollisionShape tmpShape = m_compoundColObj.GetCollisionShape();
			    m_compoundColObj.InternalSetTemporaryCollisionShape( childShape );

			    if (m_childCollisionAlgorithms[index] == null)
                {
				    m_childCollisionAlgorithms[index] = m_dispatcher.FindAlgorithm(m_compoundColObj,m_otherObj,m_sharedManifold);
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


			    m_childCollisionAlgorithms[index].ProcessCollision(m_compoundColObj,m_otherObj,m_dispatchInfo, m_resultOut);
                if (m_dispatchInfo.getDebugDraw() != null && (((m_dispatchInfo.getDebugDraw().GetDebugMode() & DebugDrawModes.DBG_DrawAabb)) != 0))
			    {
                    Vector3 worldAabbMin = Vector3.Zero, worldAabbMax = Vector3.Zero;
                    m_dispatchInfo.getDebugDraw().DrawAabb(aabbMin0, aabbMax0, new Vector3(1, 1, 1));
                    m_dispatchInfo.getDebugDraw().DrawAabb(aabbMin1, aabbMax1, new Vector3(1, 1, 1));
			    }
    			
			    //revert back transform 
			    m_compoundColObj.InternalSetTemporaryCollisionShape( tmpShape);
			    m_compoundColObj.SetWorldTransform( ref orgTrans );
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
			    Vector3 worldAabbMin = Vector3.Zero;
                Vector3 worldAabbMax = Vector3.Zero;
			    Matrix orgTrans = m_compoundColObj.GetWorldTransform();
			    MathUtil.TransformAabb(leaf.volume.Mins(),leaf.volume.Maxs(),0f,orgTrans,ref worldAabbMin,ref worldAabbMax);
                m_dispatchInfo.getDebugDraw().DrawAabb(worldAabbMin, worldAabbMax, new Vector3(1, 0, 0));
		    }
		    ProcessChildShape(childShape,index);
	    }
    }

    public class CompoundCreateFunc : CollisionAlgorithmCreateFunc
    {
        public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
        {
            return new CompoundCollisionAlgorithm(ci, body0, body1,false);
        }
    }

    public class SwappedCompoundCreateFunc : CollisionAlgorithmCreateFunc
    {
        public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
        {
            return new CompoundCollisionAlgorithm(ci, body0, body1,true);
        }
    }


}
