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
using BulletXNA.BulletCollision.BroadphaseCollision;
using BulletXNA.BulletCollision.CollisionShapes;
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletCollision.CollisionDispatch
{
public class GhostObject : CollisionObject
{
	public GhostObject()
    {
        SetInternalType(CollisionObjectTypes.CO_GHOST_OBJECT);
    }

    public override void Cleanup()
    {
        ///btGhostObject should have been removed from the world, so no overlapping objects
        Debug.Assert(m_overlappingObjects.Count == 0);
    }

	public void	ConvexSweepTest(ConvexShape castShape, ref Matrix convexFromWorld, ref Matrix convexToWorld, ConvexResultCallback resultCallback, float allowedCcdPenetration)
    {
	    Matrix	convexFromTrans = convexFromWorld;
        Matrix convexToTrans  = convexToWorld;

        Vector3 castShapeAabbMin;
        Vector3 castShapeAabbMax;
        /* Compute AABB that encompasses angular movement */
        Vector3 linVel = Vector3.Zero, angVel = Vector3.Zero;
	    TransformUtil.CalculateVelocity (ref convexFromTrans, ref convexToTrans, 1.0f, ref linVel, ref angVel);
	    Matrix R = MathUtil.BasisMatrix(ref convexFromTrans);
	    castShape.CalculateTemporalAabb (ref R, ref linVel, ref angVel, 1.0f, out castShapeAabbMin, out castShapeAabbMax);
	
	    /// go over all objects, and if the ray intersects their aabb + cast shape aabb,
	    // do a ray-shape query using convexCaster (CCD)
	    for (int i=0;i<m_overlappingObjects.Count;i++)
	    {
		    CollisionObject	collisionObject = m_overlappingObjects[i];
		    //only perform raycast if filterMask matches
		    if(resultCallback.NeedsCollision(collisionObject.GetBroadphaseHandle())) 
            {
			    //RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
                Vector3 collisionObjectAabbMin;
                Vector3 collisionObjectAabbMax;
                Matrix t = collisionObject.GetWorldTransform();
			    collisionObject.GetCollisionShape().GetAabb(ref t,out collisionObjectAabbMin,out collisionObjectAabbMax);
			    AabbUtil2.AabbExpand (ref collisionObjectAabbMin, ref collisionObjectAabbMax, ref castShapeAabbMin, ref castShapeAabbMax);
			    float hitLambda = 1f; //could use resultCallback.m_closestHitFraction, but needs testing
                Vector3 hitNormal;

                if (AabbUtil2.RayAabb(convexFromWorld.Translation, convexToWorld.Translation, ref collisionObjectAabbMin, ref collisionObjectAabbMax, ref hitLambda, out hitNormal))
			    {
                    Matrix wt = collisionObject.GetWorldTransform();
				    CollisionWorld.ObjectQuerySingle(castShape, ref convexFromTrans,ref convexToTrans,
					    collisionObject,
						    collisionObject.GetCollisionShape(),
						    ref wt,
						    resultCallback,
						    allowedCcdPenetration);
			    }
		    }
	    }

    }

	public void RayTest(ref Vector3 rayFromWorld, ref Vector3 rayToWorld, RayResultCallback resultCallback)
    {
    }

	///this method is mainly for expert/internal use only.
    public virtual void AddOverlappingObjectInternal(BroadphaseProxy otherProxy, BroadphaseProxy thisProxy)
    {
        CollisionObject otherObject = (CollisionObject)otherProxy.m_clientObject;
        Debug.Assert(otherObject != null);
        ///if this linearSearch becomes too slow (too many overlapping objects) we should add a more appropriate data structure
        if(!m_overlappingObjects.Contains(otherObject))
        {
            //not found
            m_overlappingObjects.Add(otherObject);
        }
    }
	///this method is mainly for expert/internal use only.
    public virtual void RemoveOverlappingObjectInternal(BroadphaseProxy otherProxy, IDispatcher dispatcher, BroadphaseProxy thisProxy)
    {
        CollisionObject otherObject = (CollisionObject)otherProxy.m_clientObject;
        Debug.Assert(otherObject != null);
        ///if this linearSearch becomes too slow (too many overlapping objects) we should add a more appropriate data structure
        if(!m_overlappingObjects.Contains(otherObject))
        {
            m_overlappingObjects.Remove(otherObject);
        }
    }

	public int GetNumOverlappingObjects()
	{
		return m_overlappingObjects.Count;
	}

	public CollisionObject GetOverlappingObject(int index)
	{
		return m_overlappingObjects[index];
	}

	public IList<CollisionObject> GetOverlappingPairs()
	{
		return m_overlappingObjects;
	}

	//
	// internal cast
	//

	public static GhostObject Upcast(CollisionObject colObj)
	{
		if (colObj.GetInternalType()==CollisionObjectTypes.CO_GHOST_OBJECT)
        {
			return (GhostObject)colObj;
        }
		return null;
	}

    protected IList<CollisionObject> m_overlappingObjects;

}

    public class PairCachingGhostObject : GhostObject
    {
	    public PairCachingGhostObject()
        {
            m_hashPairCache = new HashedOverlappingPairCache();
        }

	    public override void Cleanup()
        {
            m_hashPairCache.Cleanup();
	        m_hashPairCache = null;
        }

	    ///this method is mainly for expert/internal use only.
        public override void AddOverlappingObjectInternal(BroadphaseProxy otherProxy, BroadphaseProxy thisProxy)
        {
            BroadphaseProxy actualThisProxy = thisProxy != null ? thisProxy : GetBroadphaseHandle();
            Debug.Assert(actualThisProxy != null);

            CollisionObject otherObject = (CollisionObject)otherProxy.m_clientObject;
            Debug.Assert(otherObject != null);
            if(!m_overlappingObjects.Contains(otherObject))
            {
                m_overlappingObjects.Add(otherObject);
                m_hashPairCache.AddOverlappingPair(actualThisProxy, otherProxy);
            }
        }

        public override void RemoveOverlappingObjectInternal(BroadphaseProxy otherProxy, IDispatcher dispatcher, BroadphaseProxy thisProxy)
        {
            CollisionObject otherObject = (CollisionObject)otherProxy.m_clientObject;
            BroadphaseProxy actualThisProxy = thisProxy != null ? thisProxy : GetBroadphaseHandle();
            Debug.Assert(actualThisProxy != null);

            Debug.Assert(otherObject != null);
            if(m_overlappingObjects.Contains(otherObject))
            {
                m_overlappingObjects.Remove(otherObject);
                m_hashPairCache.RemoveOverlappingPair(actualThisProxy, otherProxy, dispatcher);
            }
        }

	    public HashedOverlappingPairCache GetOverlappingPairCache()
	    {
		    return m_hashPairCache;
	    }

        private HashedOverlappingPairCache	m_hashPairCache;

    }



    ///The btGhostPairCallback interfaces and forwards adding and removal of overlapping pairs from the btBroadphaseInterface to btGhostObject.
    public class GhostPairCallback : IOverlappingPairCallback
    {
	    public GhostPairCallback()
	    {
	    }

        public virtual void cleanup()
        {
        }

	    public virtual BroadphasePair AddOverlappingPair(BroadphaseProxy proxy0,BroadphaseProxy proxy1)
	    {
		    CollisionObject colObj0 = (CollisionObject) proxy0.m_clientObject;
		    CollisionObject colObj1 = (CollisionObject) proxy1.m_clientObject;
		    GhostObject ghost0 = GhostObject.Upcast(colObj0);
		    GhostObject ghost1 = GhostObject.Upcast(colObj1);
            if (ghost0 != null)
            {
                ghost0.AddOverlappingObjectInternal(proxy1, proxy0);
            }
            if (ghost1 != null)
            {
                ghost1.AddOverlappingObjectInternal(proxy0, proxy1);
            }
		    return null;
	    }

	    public virtual Object RemoveOverlappingPair(BroadphaseProxy proxy0,BroadphaseProxy proxy1,IDispatcher dispatcher)
	    {
		    CollisionObject colObj0 = (CollisionObject) proxy0.m_clientObject;
		    CollisionObject colObj1 = (CollisionObject) proxy1.m_clientObject;
		    GhostObject ghost0 = GhostObject.Upcast(colObj0);
		    GhostObject ghost1 = GhostObject.Upcast(colObj1);
            if (ghost0 != null)
            {
                ghost0.RemoveOverlappingObjectInternal(proxy1, dispatcher, proxy0);
            }
            if (ghost1 != null)
            {
                ghost1.RemoveOverlappingObjectInternal(proxy0, dispatcher, proxy1);
            }
		    return null;
	    }

	    public virtual void RemoveOverlappingPairsContainingProxy(BroadphaseProxy proxy0,IDispatcher dispatcher)
	    {
		    Debug.Assert(false);
		    //need to keep track of all ghost objects and call them here
		    //m_hashPairCache->removeOverlappingPairsContainingProxy(proxy0,dispatcher);
	    }
    }
}
