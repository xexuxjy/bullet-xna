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

using System.Diagnostics;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
    public abstract class ConvexInternalShape : ConvexShape
    {
        public ConvexInternalShape()
        {
            m_localScaling = new Vector3(1);
            m_collisionMargin = CollisionMargin.CONVEX_DISTANCE_MARGIN;
        }

        public override void Cleanup()
        {
            base.Cleanup();
        }

        public override Vector3 LocalGetSupportingVertex(ref Vector3 vec)
        {
	         Vector3 supVertex = LocalGetSupportingVertexWithoutMargin(ref vec);

	        if (Margin !=0f)
	        {
		        Vector3 vecnorm = vec;
		        if (vecnorm.LengthSquared() < (MathUtil.SIMD_EPSILON*MathUtil.SIMD_EPSILON))
		        {
			        vecnorm = new Vector3(-1);
		        } 
		        vecnorm.Normalize();
		        supVertex+= Margin * vecnorm;
	        }
	        return supVertex;
        }

        public Vector3 GetImplicitShapeDimensions()
        {
            return m_implicitShapeDimensions;
        }

	    ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	    public override void GetAabb(ref Matrix t,out Vector3 aabbMin,out Vector3 aabbMax)
	    {
		    GetAabbSlow(ref t,out aabbMin,out aabbMax);
	    }



        public override void GetAabbSlow(ref Matrix trans, out Vector3 aabbMin, out Vector3 aabbMax)
        {
	        float margin = Margin;
            aabbMin = Vector3.Zero;
            aabbMax = Vector3.Zero;
            for (int i = 0; i < 3; i++)
	        {
		        Vector3 vec = new Vector3();
		        MathUtil.VectorComponent(ref vec,i,1f);


				Vector3 temp = MathUtil.TransposeTransformNormal(vec, trans);
				Vector3 sv = LocalGetSupportingVertex(ref temp);
				Vector3 tmp = Vector3.Transform(sv, trans);
				MathUtil.VectorComponent(ref aabbMax, i, MathUtil.VectorComponent(ref tmp, i) + margin);
				MathUtil.VectorComponent(ref vec, i, -1f);
				temp = MathUtil.TransposeTransformNormal(vec, trans);
				sv = LocalGetSupportingVertex(ref temp);
				tmp = Vector3.Transform(sv, trans);
				MathUtil.VectorComponent(ref aabbMin, i, MathUtil.VectorComponent(ref tmp, i) - margin);
	        }
        }

        public override void SetLocalScaling(ref Vector3 scaling)
        {
            MathUtil.AbsoluteVector(ref scaling, out m_localScaling);
        }

        public override Vector3 GetLocalScaling()
	    {
		    return m_localScaling;
	    }

	    public Vector3 GetLocalScalingNV()
	    {
		    return m_localScaling;
	    }

        public override float Margin
	    {
            get
            {
                return m_collisionMargin;
            }
            set
            {
                m_collisionMargin = value;
            }
	    }

	    public float GetMarginNV()
	    {
		    return m_collisionMargin;
	    }

        public override int GetNumPreferredPenetrationDirections()
	    {
		    return 0;
	    }
	
	    public override void GetPreferredPenetrationDirection(int index, out Vector3 penetrationVector)
	    {
            Debug.Assert(false);
            penetrationVector = Vector3.Zero;
        }


        protected Vector3 m_localScaling;
        protected Vector3 m_implicitShapeDimensions;
        protected float m_collisionMargin;
        protected float m_padding;
    }

    public class ConvexInternalAabbCachingShape : ConvexInternalShape
    {
        protected Vector3 m_localAabbMin;
        protected Vector3 m_localAabbMax;
        protected bool m_isLocalAabbValid;


        protected void SetCachedLocalAabb(ref Vector3 aabbMin, ref Vector3 aabbMax)
        {
            m_isLocalAabbValid = true;
            m_localAabbMin = aabbMin;
            m_localAabbMax = aabbMax;
        }

        public void GetCachedLocalAabb(out Vector3 aabbMin, out Vector3 aabbMax)
        {
            Debug.Assert(m_isLocalAabbValid);
            aabbMin = m_localAabbMin;
            aabbMax = m_localAabbMax;
        }

        public void GetNonvirtualAabb(ref Matrix trans, out Vector3 aabbMin, out Vector3 aabbMax, float margin)
        {
            //lazy evaluation of local aabb
            Debug.Assert(m_isLocalAabbValid);
            AabbUtil2.TransformAabb(ref m_localAabbMin, ref m_localAabbMax, margin, ref trans, out aabbMin, out aabbMax);
        }

        public override Vector3 LocalGetSupportingVertexWithoutMargin(ref Vector3 vec)
        {
            return Vector3.Zero;
        }


        //public:

        //    virtual void	setLocalScaling(const btVector3& scaling);

        //    virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;

        //    void	recalcLocalAabb();


        public ConvexInternalAabbCachingShape()
        {
            m_localAabbMin = new Vector3(1);
            m_localAabbMax = new Vector3(-1);
            m_isLocalAabbValid = false;
        }

        public override void GetAabb(ref Matrix trans, out Vector3 aabbMin, out Vector3 aabbMax)
        {
            GetNonvirtualAabb(ref trans, out aabbMin, out aabbMax, Margin);
        }

        public override void SetLocalScaling(ref Vector3 scaling)
        {
            base.SetLocalScaling(ref scaling);
            RecalcLocalAabb();
        }

        public virtual void RecalcLocalAabb()
        {
	        m_isLocalAabbValid = true;
        	
	        #if true
            //fixme - make a static list.
            Vector4[] _supporting = new Vector4[6];
	        BatchedUnitVectorGetSupportingVertexWithoutMargin(_directions, _supporting, 6);
        	
	        for ( int i = 0; i < 3; ++i )
	        {
                Vector4 temp = _supporting[i];
		        MathUtil.VectorComponent(ref m_localAabbMax, i, (MathUtil.VectorComponent(ref temp,i) + m_collisionMargin));
                MathUtil.VectorComponent(ref m_localAabbMin, i, (MathUtil.VectorComponent(ref temp, i) - m_collisionMargin));
	        }
        	
	        #else

	        for (int i=0;i<3;i++)
	        {
		        btVector3 vec(btScalar(0.),btScalar(0.),btScalar(0.));
		        vec[i] = btScalar(1.);
		        btVector3 tmp = localGetSupportingVertex(vec);
		        m_localAabbMax[i] = tmp[i]+m_collisionMargin;
		        vec[i] = btScalar(-1.);
		        tmp = localGetSupportingVertex(vec);
		        m_localAabbMin[i] = tmp[i]-m_collisionMargin;
	        }
	        #endif
        }

        public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector4[] supportVerticesOut, int numVectors)
        {
        }


        public static Vector3[] _directions = new Vector3[]
	        {
		        new Vector3( 1.0f,  0.0f,  0.0f),
		        new Vector3( 0.0f,  1.0f,  0.0f),
		        new Vector3( 0.0f,  0.0f,  1.0f),
		        new Vector3( -1.0f, 0.0f,  0.0f),
		        new Vector3( 0.0f, -1.0f,  0.0f),
		        new Vector3( 0.0f,  0.0f, -1.0f)
	        };
		

    }
}