﻿/*
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
using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
    ///The btBroadphaseProxy is the main class that can be used with the Bullet broadphases. 
    ///It stores collision shape type information, collision filter information and a client object, typically a btCollisionObject or btRigidBody.
    public class BroadphaseProxy
    {
        ///optional filtering to cull potential collisions

        //Usually the client btCollisionObject or Rigidbody class
        public Object m_clientObject;
        public CollisionFilterGroups m_collisionFilterGroup;
        public CollisionFilterGroups m_collisionFilterMask;
        public Object m_multiSapParentProxy;
        public int m_uniqueId;//m_uniqueId is introduced for paircache. could get rid of this, by calculating the address offset etc.

        public Vector3 m_aabbMin;
        public Vector3 m_aabbMax;

        public int GetUid()
        {
            return m_uniqueId;
        }

        //used for memory pools
        public BroadphaseProxy()
        {
            m_clientObject = null;
            m_multiSapParentProxy = null;
        }

        public BroadphaseProxy(ref Vector3 aabbMin, ref Vector3 aabbMax, Object userPtr, CollisionFilterGroups collisionFilterGroup, CollisionFilterGroups collisionFilterMask, Object multiSapParentProxy)
        {
            m_clientObject = userPtr;
            m_collisionFilterGroup = collisionFilterGroup;
            m_collisionFilterMask = collisionFilterMask;
            m_aabbMin = aabbMin;
            m_aabbMax = aabbMax;
            m_multiSapParentProxy = multiSapParentProxy;
        }

        public static bool IsPolyhedral(BroadphaseNativeType proxyType)
        {
            return (proxyType < BroadphaseNativeType.IMPLICIT_CONVEX_SHAPES_START_HERE);
        }

        public static bool IsConvex(BroadphaseNativeType proxyType)
        {
            return (proxyType < BroadphaseNativeType.CONCAVE_SHAPES_START_HERE);
        }

        public static bool IsNonMoving(BroadphaseNativeType proxyType)
        {
            return (IsConcave(proxyType) && !(proxyType == BroadphaseNativeType.GIMPACT_SHAPE_PROXYTYPE));
        }


        public static bool IsConcave(BroadphaseNativeType proxyType)
        {
            return ((proxyType > BroadphaseNativeType.CONCAVE_SHAPES_START_HERE) &&
                (proxyType < BroadphaseNativeType.CONCAVE_SHAPES_END_HERE));
        }
        public static bool IsCompound(BroadphaseNativeType proxyType)
        {
            return (proxyType == BroadphaseNativeType.CompoundShape);
        }

        public static bool IsSoftBody(BroadphaseNativeType proxyType)
        {
            return (proxyType == BroadphaseNativeType.SoftBodyShape);
        }


        public static bool IsInfinite(BroadphaseNativeType proxyType)
        {
            return (proxyType == BroadphaseNativeType.StaticPlane);
        }

        public static bool IsConvex2d(BroadphaseNativeType proxyType)
        {
            return (proxyType == BroadphaseNativeType.Box2DShape) || (proxyType == BroadphaseNativeType.Convex2DShape);
        }


        public Vector3 GetMinAABB()
        {
            return m_aabbMin;
        }

        public Vector3 GetMaxAABB()
        {
            return m_aabbMax;
        }

        public void SetMinAABB(ref Vector3 min)
        {
            m_aabbMin = min;
        }

        public void SetMaxAABB(ref Vector3 max)
        {
            m_aabbMax = max;
        }

        public Object GetClientObject()
        {
            return m_clientObject;
        }

        public void SetClientObject(Object o)
        {
            m_clientObject = o;
        }

        public virtual void Cleanup()
        {
        }
    }

    public class BroadphasePair : IComparable
    {
        public BroadphasePair()
        {
            m_pProxy0 = null;
            m_pProxy1 = null;
            m_algorithm = null;
            m_internalInfo1 = null;
        }

        public BroadphasePair(ref BroadphasePair other)
        {
            m_pProxy0 = other.m_pProxy0;
            m_pProxy1 = other.m_pProxy1;
            m_algorithm = other.m_algorithm;
            m_internalInfo1 = other.m_internalInfo1;
        }

        public BroadphasePair(BroadphaseProxy proxy0, BroadphaseProxy proxy1)
        {
            //keep them sorted, so the std::set operations work
            if (proxy0.GetUid() < proxy1.GetUid())
            {
                m_pProxy0 = proxy0;
                m_pProxy1 = proxy1;
            }
            else
            {
                m_pProxy0 = proxy1;
                m_pProxy1 = proxy0;
            }

            m_algorithm = null;
            m_internalInfo1 = null;
        }


        public override bool Equals(object obj)
        {
            if (ReferenceEquals(this, obj))
            {
                return true;
            }
            else
            {
                if (obj is BroadphasePair)
                {
                    BroadphasePair that = (BroadphasePair)obj;
                    return ReferenceEquals(this.m_pProxy0, that.m_pProxy0) && ReferenceEquals(this.m_pProxy1, that.m_pProxy1);
                }
            }
            return false;
        }

        public static bool IsLessThen(BroadphasePair a, BroadphasePair b)
        {
            int uidA0 = a.m_pProxy0 != null ? a.m_pProxy0.GetUid() : -1;
            int uidB0 = b.m_pProxy0 != null ? b.m_pProxy0.GetUid() : -1;
            int uidA1 = a.m_pProxy1 != null ? a.m_pProxy1.GetUid() : -1;
            int uidB1 = b.m_pProxy1 != null ? b.m_pProxy1.GetUid() : -1;

            //return uidA0 > uidB0 || 
            //   (a.m_pProxy0 == b.m_pProxy0 && uidA1 > uidB1) ||
            //   (a.m_pProxy0 == b.m_pProxy0 && a.m_pProxy1 == b.m_pProxy1 && a.m_algorithm > b.m_algorithm); 
            return uidA0 > uidB0 ||
               (a.m_pProxy0 == b.m_pProxy0 && uidA1 > uidB1) ||
               (a.m_pProxy0 == b.m_pProxy0 && a.m_pProxy1 == b.m_pProxy1);
        }

        #region IComparable Members

        public int CompareTo(object obj)
        {
            return (IsLessThen(this, (BroadphasePair)obj) ? -1 : 1);
        }

        #endregion



        public BroadphaseProxy m_pProxy0;
        public BroadphaseProxy m_pProxy1;

        public CollisionAlgorithm m_algorithm;
        //don't use this data, it will be removed in future version.
        public Object m_internalInfo1;
        public int m_internalTmpValue;
    }

    public enum BroadphaseNativeType
    {
        // polyhedral convex shapes
        BoxShape,
        TriangleShape,
        TetrahedralShape,
        ConvexTriangleMeshShape,
        ConvexHullShape,
        CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE,
        CUSTOM_POLYHEDRAL_SHAPE_TYPE,
        //implicit convex shapes
        IMPLICIT_CONVEX_SHAPES_START_HERE,
        SphereShape,
        MultiSphereShape,
        CapsuleShape,
        ConeShape,
        ConvexShape,
        CylinderShape,
        UniformScalingShape,
        MINKOWSKI_SUM_SHAPE_PROXYTYPE,
        MinkowskiDifferenceShape,
        Box2DShape,
        Convex2DShape,
        CUSTOM_CONVEX_SHAPE_TYPE,
        //concave shapes
        CONCAVE_SHAPES_START_HERE,
        //keep all the convex shapetype below here, for the check IsConvexShape in broadphase proxy!
        TriangleMeshShape,
        SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE,
        ///used for demo integration FAST/Swift collision library and Bullet
        FAST_CONCAVE_MESH_PROXYTYPE,
        //terrain
        TERRAIN_SHAPE_PROXYTYPE,
        ///Used for GIMPACT Trimesh integration
        GIMPACT_SHAPE_PROXYTYPE,
        ///Multimaterial mesh
        MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE,

        EmptyShape,
        StaticPlane,
        CUSTOM_CONCAVE_SHAPE_TYPE,
        CONCAVE_SHAPES_END_HERE,

        CompoundShape,

        SoftBodyShape,
        HFFLUID_SHAPE_PROXYTYPE,
        HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE,
        INVALID_SHAPE_PROXYTYPE,

        MAX_BROADPHASE_COLLISION_TYPES

    };

    [Flags]
    public enum CollisionFilterGroups
    {
        DefaultFilter = 1,
        StaticFilter = 2,
        KinematicFilter = 4,
        DebrisFilter = 8,
        SensorTrigger = 16,
        CharacterFilter = 32,
        AllFilter = -1 //all bits sets: DefaultFilter | StaticFilter | KinematicFilter | DebrisFilter | SensorTrigger
    }

}