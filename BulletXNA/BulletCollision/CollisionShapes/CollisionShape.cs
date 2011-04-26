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

using BulletXNA.BulletCollision.BroadphaseCollision;
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletCollision.CollisionShapes
{
    public abstract class CollisionShape
    {
        public CollisionShape()
        {
            m_shapeType = BroadphaseNativeTypes.INVALID_SHAPE_PROXYTYPE;
            m_userPointer = null;
        }

	    public virtual void Cleanup()
	    {
	    }


	    ///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
        ///
        public virtual void GetAabb
            (Matrix t,out Vector3 aabbMin,out Vector3 aabbMax)
        {
            // t isn't assigned to as we're just getting the bounds.
            GetAabb(ref t,out aabbMin,out aabbMax);
        }

        public abstract void GetAabb(ref Matrix t,out Vector3 aabbMin,out Vector3 aabbMax);

	    public virtual void GetBoundingSphere(ref Vector3 center,ref float radius)
        {
	        Matrix tr = Matrix.Identity;
	        Vector3 aabbMin;
            Vector3 aabbMax;

	        GetAabb(ref tr,out aabbMin,out aabbMax);

	        radius = (aabbMax-aabbMin).Length()*0.5f;
	        center = (aabbMin+aabbMax)*0.5f;
        }

	    ///getAngularMotionDisc returns the maximus radius needed for Conservative Advancement to handle time-of-impact with rotations.
	    public virtual float GetAngularMotionDisc()
        {
	        Vector3	center = new Vector3();
	        float disc =0f;
	        GetBoundingSphere(ref center,ref disc);
	        disc += (center).Length();
	        return disc;
        }

	    public virtual float GetContactBreakingThreshold(float defaultContactThreshold)
        {
	        return GetAngularMotionDisc() * defaultContactThreshold;
        }


        ///calculateTemporalAabb calculates the enclosing aabb for the moving object over interval [0..timeStep)
	    ///result is conservative
	    public void CalculateTemporalAabb(ref Matrix curTrans,ref Vector3 linvel,ref Vector3 angvel,float timeStep, out Vector3 temporalAabbMin,out Vector3 temporalAabbMax)
        {
	        //start with static aabb
	        GetAabb(ref curTrans,out temporalAabbMin,out temporalAabbMax);

	        float temporalAabbMaxx = temporalAabbMax.X;
	        float temporalAabbMaxy = temporalAabbMax.Y;
	        float temporalAabbMaxz = temporalAabbMax.Z;
	        float temporalAabbMinx = temporalAabbMin.X;
	        float temporalAabbMiny = temporalAabbMin.Y;
	        float temporalAabbMinz = temporalAabbMin.Z;

	        // add linear motion
	        Vector3 linMotion = linvel*timeStep;
	        ///@todo: simd would have a vector max/min operation, instead of per-element access
	        if (linMotion.X > 0f)
		        temporalAabbMaxx += linMotion.X; 
	        else
		        temporalAabbMinx += linMotion.X;
	        if (linMotion.Y > 0f)
		        temporalAabbMaxy += linMotion.Y; 
	        else
		        temporalAabbMiny += linMotion.Y;
	        if (linMotion.Z > 0f)
		        temporalAabbMaxz += linMotion.Z; 
	        else
		        temporalAabbMinz += linMotion.Z;

	        //add conservative angular motion
	        float angularMotion = angvel.Length() * GetAngularMotionDisc() * timeStep;
	        Vector3 angularMotion3d = new Vector3(angularMotion,angularMotion,angularMotion);
	        temporalAabbMin = new Vector3(temporalAabbMinx,temporalAabbMiny,temporalAabbMinz);
	        temporalAabbMax = new Vector3(temporalAabbMaxx,temporalAabbMaxy,temporalAabbMaxz);

	        temporalAabbMin -= angularMotion3d;
	        temporalAabbMax += angularMotion3d;

        }
	
        public bool	IsPolyhedral()
	    {
		    return BroadphaseProxy.IsPolyhedral(GetShapeType());
	    }

	    public bool	IsConvex()
	    {
		    return BroadphaseProxy.IsConvex(GetShapeType());
	    }

        public bool	IsNonMoving()
	    {
		    return BroadphaseProxy.IsNonMoving(GetShapeType());
	    }

        public bool	IsConvex2d()
	    {
		    return BroadphaseProxy.IsConvex2d(GetShapeType());
	    }

        public bool	IsConcave() 
	    {
		    return BroadphaseProxy.IsConcave(GetShapeType());
	    }
	    public bool	IsCompound() 
	    {
		    return BroadphaseProxy.IsCompound(GetShapeType());
	    }

        public bool	IsSoftBody()
	    {
		    return BroadphaseProxy.IsSoftBody(GetShapeType());
	    }

	    ///isInfinite is used to catch simulation error (aabb check)
	    public bool IsInfinite()
	    {
		    return BroadphaseProxy.IsInfinite(GetShapeType());
	    }

        // defining these as virtual rather then abstract as the whole impementation agains _SPU_ seems odd
        public virtual void SetLocalScaling(ref Vector3 scaling)
        {
        }
        public virtual Vector3 GetLocalScaling()
        {
            return new Vector3(1, 1, 1);
        }
        public virtual void CalculateLocalInertia(float mass, ref Vector3 inertia)
        {
        }

        //debugging support
        public virtual String GetName()
        {
            return "Not-Defined";
        }

        public BroadphaseNativeTypes GetShapeType()
        {
            return m_shapeType;
        }

        public abstract void SetMargin(float margin);

        public abstract float GetMargin();
	
	    ///optional user data pointer
	    public void	SetUserPointer(Object  userPtr)
	    {
		    m_userPointer = userPtr;
	    }

	    public Object GetUserPointer()
	    {
		    return m_userPointer;
	    }

        protected BroadphaseNativeTypes m_shapeType;
        protected Object m_userPointer;

        public static float gContactThresholdFactor = 0.02f;

    }

        
}

