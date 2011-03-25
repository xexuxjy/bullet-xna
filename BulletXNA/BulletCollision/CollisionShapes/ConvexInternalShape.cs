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


using Microsoft.Xna.Framework;
using System.Diagnostics;

namespace BulletXNA.BulletCollision.CollisionShapes
{
    public abstract class ConvexInternalShape : ConvexShape
    {
        public ConvexInternalShape()
        {
            m_localScaling = new Vector3(1, 1, 1);
            m_collisionMargin = CollisionMargin.CONVEX_DISTANCE_MARGIN;
        }

        public override void Cleanup()
        {
            base.Cleanup();
        }

        public override Vector3 LocalGetSupportingVertex(ref Vector3 vec)
        {
	         Vector3 supVertex = LocalGetSupportingVertexWithoutMargin(ref vec);

	        if (GetMargin() !=0f)
	        {
		        Vector3 vecnorm = vec;
		        if (vecnorm.LengthSquared() < (MathUtil.SIMD_EPSILON*MathUtil.SIMD_EPSILON))
		        {
			        vecnorm = new Vector3(-1,-1,-1);
		        } 
		        vecnorm.Normalize();
		        supVertex+= GetMargin() * vecnorm;
	        }
	        return supVertex;
        }

        public Vector3 GetImplicitShapeDimensions()
        {
            return m_implicitShapeDimensions;
        }

	    ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	    public override void GetAabb(ref Matrix t,ref Vector3 aabbMin,ref Vector3 aabbMax)
	    {
		    GetAabbSlow(ref t,ref aabbMin,ref aabbMax);
	    }



        public override void GetAabbSlow(ref Matrix trans, ref Vector3 aabbMin, ref Vector3 aabbMax)
        {
	        float margin = GetMargin();
	        for (int i=0;i<3;i++)
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


		//btVector3 sv = localGetSupportingVertex(vec*trans.getBasis());

		//btVector3 tmp = trans(sv);
		//maxAabb[i] = tmp[i]+margin;
		//vec[i] = btScalar(-1.);
		//tmp = trans(localGetSupportingVertex(vec*trans.getBasis()));
		//minAabb[i] = tmp[i]-margin;

				//MathUtil.rotateVector(ref vec,ref trans,ref temp);
				//Vector3 sv = localGetSupportingVertex(ref temp);
				//Vector3 tmp = Vector3.Transform(sv,trans);
				//MathUtil.vectorComponent(ref aabbMax,i,MathUtil.vectorComponent(ref tmp,i)+margin);
				//MathUtil.vectorComponent(ref vec,i,-1);
				//MathUtil.rotateVector(ref vec, ref trans, ref temp);
				//tmp = Vector3.Transform(localGetSupportingVertex(ref temp),trans);
				//MathUtil.vectorComponent(ref aabbMin, i, MathUtil.vectorComponent(ref tmp, i) - margin);
	        }
        }

        public override void SetLocalScaling(ref Vector3 scaling)
        {
            MathUtil.AbsoluteVector(ref scaling, ref m_localScaling);
        }

        public override Vector3 GetLocalScaling()
	    {
		    return m_localScaling;
	    }

	    public Vector3 GetLocalScalingNV()
	    {
		    return m_localScaling;
	    }

        public override void SetMargin(float margin)
	    {
		    m_collisionMargin = margin;
	    }

        public override float GetMargin()
	    {
		    return m_collisionMargin;
	    }

	    public float GetMarginNV()
	    {
		    return m_collisionMargin;
	    }

        public override int GetNumPreferredPenetrationDirections()
	    {
		    return 0;
	    }
	
	    public override void GetPreferredPenetrationDirection(int index, ref Vector3 penetrationVector)
	    {
            //(void)penetrationVector;
            //(void)index;
            //btAssert(0);
            Debug.Assert(false);
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

        public void GetCachedLocalAabb(ref Vector3 aabbMin, ref Vector3 aabbMax)
        {
            Debug.Assert(m_isLocalAabbValid);
            aabbMin = m_localAabbMin;
            aabbMax = m_localAabbMax;
        }

        public void GetNonvirtualAabb(ref Matrix trans, ref Vector3 aabbMin, ref Vector3 aabbMax, float margin)
        {
            //lazy evaluation of local aabb
            Debug.Assert(m_isLocalAabbValid);
            AabbUtil2.TransformAabb(ref m_localAabbMin, ref m_localAabbMax, margin, ref trans, ref aabbMin, ref aabbMax);
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
            m_localAabbMin = new Vector3(1, 1, 1);
            m_localAabbMax = new Vector3(-1, -1, -1);
            m_isLocalAabbValid = false;
        }

        public override void GetAabb(ref Matrix trans, ref Vector3 aabbMin, ref Vector3 aabbMax)
        {
            GetNonvirtualAabb(ref trans, ref aabbMin, ref aabbMax, GetMargin());
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
            IList<Vector3> localDirections = new List<Vector3>();
            for (int i = 0; i < _directions.Length; ++i)
            {
                localDirections.Add(_directions[i]);
            }
            IList<Vector4> _supporting = new List<Vector4>();
        	
	        BatchedUnitVectorGetSupportingVertexWithoutMargin(localDirections, _supporting, 6);
        	
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

        public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IList<Vector3> vectors, IList<Vector4> supportVerticesOut, int numVectors)
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
