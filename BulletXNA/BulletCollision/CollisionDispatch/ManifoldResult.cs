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
using BulletXNA.BulletCollision.NarrowPhaseCollision;
using Microsoft.Xna.Framework;
using BulletXNA.BullettCollision.CollisionDispatch;
using System.Diagnostics;

namespace BulletXNA.BulletCollision.CollisionDispatch
{
    public interface IContactAddedCallback
    {
        bool Callback(ManifoldPoint cp, CollisionObject colObj0, int partId0, int index0, CollisionObject colObj1, int partId1, int index1);
    }
    
    ///btManifoldResult is a helper class to manage  contact results.
    public class ManifoldResult : IDiscreteCollisionDetectorInterfaceResult
    {
        public ManifoldResult()
        {
            int ibreak = 0;
#if DEBUG_PART_INDEX
    m_partId0 = -1;
    m_partId1 = -1;
    m_index0 = -1;
    m_index1 = -1;
#endif //DEBUG_PART_INDEX

        }

        public ManifoldResult(CollisionObject body0, CollisionObject body1)
        {
            Initialise(body0, body1);
        }

        public void Initialise(CollisionObject body0, CollisionObject body1)
        {
            m_body0 = body0;
            m_body1 = body1;
	        m_rootTransA = body0.GetWorldTransform();
	        m_rootTransB = body1.GetWorldTransform();
            m_manifoldPtr = null;
        }

	    public void	SetPersistentManifold(PersistentManifold manifoldPtr)
	    {
		    m_manifoldPtr = manifoldPtr;
	    }

	    public PersistentManifold	GetPersistentManifold()
	    {
		    return m_manifoldPtr;
	    }

	    public virtual void SetShapeIdentifiersA(int partId0,int index0)
	    {
		    m_partId0=partId0;
		    m_index0=index0;
	    }

        public virtual void SetShapeIdentifiersB(int partId1, int index1)
        {
            m_partId1 = partId1;
            m_index1 = index1;
        }

        public CollisionObject GetBody0Internal()
	    {
		    return m_body0;
	    }

	    public CollisionObject GetBody1Internal()
	    {
		    return m_body1;
	    }

        public virtual void AddContactPoint(Vector3 normalOnBInWorld, Vector3 pointInWorld, float depth)
        {
            AddContactPoint(ref normalOnBInWorld, ref pointInWorld, depth);
        }

	    public virtual void AddContactPoint(ref Vector3 normalOnBInWorld,ref Vector3 pointInWorld,float depth)
        {
	        Debug.Assert(m_manifoldPtr != null);
	        //order in manifold needs to match
        	
	        if (depth > m_manifoldPtr.GetContactBreakingThreshold())
            {
		        return;
            }
	        
            bool isSwapped = m_manifoldPtr.GetBody0() != m_body0;

	        Vector3 pointA = pointInWorld + normalOnBInWorld * depth;

	        Vector3 localA;
	        Vector3 localB;
        	
	        if (isSwapped)
	        {
                localA = MathUtil.InverseTransform(ref m_rootTransB,ref pointA);
                localB = MathUtil.InverseTransform(ref m_rootTransA,ref pointInWorld);
	        } 
            else
	        {
                localA = MathUtil.InverseTransform(ref m_rootTransA,ref pointA);
                localB = MathUtil.InverseTransform(ref m_rootTransB,ref pointInWorld);
	        }

	        ManifoldPoint newPt = new ManifoldPoint(ref localA,ref localB,ref normalOnBInWorld,depth);
	        newPt.SetPositionWorldOnA(ref pointA);
	        newPt.SetPositionWorldOnB(ref pointInWorld);
        	
	        int insertIndex = m_manifoldPtr.GetCacheEntry(newPt);

	        newPt.SetCombinedFriction(CalculateCombinedFriction(m_body0,m_body1));
	        newPt.SetCombinedRestitution(CalculateCombinedRestitution(m_body0,m_body1));

            //BP mod, store contact triangles.
            if (isSwapped)
            {
                newPt.m_partId0 = m_partId1;
                newPt.m_partId1 = m_partId0;
                newPt.m_index0 = m_index1;
                newPt.m_index1 = m_index0;
            }
            else
            {
                newPt.m_partId0 = m_partId0;
                newPt.m_partId1 = m_partId1;
                newPt.m_index0 = m_index0;
                newPt.m_index1 = m_index1;
            }

            //printf("depth=%f\n",depth);
	        ///@todo, check this for any side effects
	        if (insertIndex >= 0)
	        {
		        //const btManifoldPoint& oldPoint = m_manifoldPtr->getContactPoint(insertIndex);
		        m_manifoldPtr.ReplaceContactPoint(newPt,insertIndex);
	        } else
	        {
		        insertIndex = m_manifoldPtr.AddManifoldPoint(newPt);
	        }
        	
	        //User can override friction and/or restitution
	        if (BulletGlobals.gContactAddedCallback != null &&
		        //and if either of the two bodies requires custom material
                 ((m_body0.GetCollisionFlags() & CollisionFlags.CF_CUSTOM_MATERIAL_CALLBACK) != 0 ||
                   (m_body1.GetCollisionFlags() & CollisionFlags.CF_CUSTOM_MATERIAL_CALLBACK)!=0))
	        {
		        //experimental feature info, for per-triangle material etc.
		        CollisionObject obj0 = isSwapped? m_body1 : m_body0;
		        CollisionObject obj1 = isSwapped? m_body0 : m_body1;
                //gContactAddedCallback.callback(m_manifoldPtr.getContactPoint(insertIndex),obj0,m_partId0,m_index0,obj1,m_partId1,m_index1);
				BulletGlobals.gContactAddedCallback.Callback(m_manifoldPtr.GetContactPoint(insertIndex), obj0, newPt.m_partId0, newPt.m_index0, obj1, newPt.m_partId1, newPt.m_index1);
	        }
        }

	    public void RefreshContactPoints()
	    {
		    Debug.Assert(m_manifoldPtr!=null);
		    if (m_manifoldPtr.GetNumContacts() == 0)
            {
			    return;
            }
		    
            bool isSwapped = m_manifoldPtr.GetBody0() != m_body0;

		    if (isSwapped)
		    {
			    m_manifoldPtr.RefreshContactPoints(ref m_rootTransB,ref m_rootTransA);
		    } 
            else
		    {
			    m_manifoldPtr.RefreshContactPoints(ref m_rootTransA,ref m_rootTransB);
		    }
	    }

        ///User can override this material combiner by implementing gContactAddedCallback and setting body0->m_collisionFlags |= btCollisionObject::customMaterialCallback;
        private float CalculateCombinedFriction(CollisionObject body0,CollisionObject body1)
        {
	        float friction = body0.GetFriction() * body1.GetFriction();

	        const float MAX_FRICTION  = 10f;
	        if (friction < -MAX_FRICTION)
		        friction = -MAX_FRICTION;
	        if (friction > MAX_FRICTION)
		        friction = MAX_FRICTION;
	        return friction;

        }

        private float CalculateCombinedRestitution(CollisionObject body0,CollisionObject body1)
        {
	        return body0.GetRestitution() * body1.GetRestitution();
        }
      
        protected PersistentManifold m_manifoldPtr;

	    //we need this for compounds
        protected Matrix m_rootTransA;
        protected Matrix m_rootTransB;

        protected CollisionObject m_body0;
        protected CollisionObject m_body1;
        protected int m_partId0;
        protected int m_partId1;
        protected int m_index0;
        protected int m_index1;
    }
}
