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

using System.Diagnostics;

using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
    ///The btConvexInternalShape uses a default collision margin set to CONVEX_DISTANCE_MARGIN.
    ///This collision margin used by Gjk and some other algorithms, see also btCollisionMargin.h
    ///Note that when creating small shapes (derived from btConvexInternalShape), 
    ///you need to make sure to set a smaller collision margin, using the 'setMargin' API
    ///There is a automatic mechanism 'setSafeMargin' used by btBoxShape and btCylinderShape

    public abstract class ConvexInternalShape : ConvexShape
    {
        public ConvexInternalShape()
        {
            m_localScaling = new IndexedVector3(1);
            m_collisionMargin = CollisionMargin.CONVEX_DISTANCE_MARGIN;
        }

        public override void Cleanup()
        {
            base.Cleanup();
        }

        public override IndexedVector3 LocalGetSupportingVertex(ref IndexedVector3 vec)
        {
	         IndexedVector3 supVertex = LocalGetSupportingVertexWithoutMargin(ref vec);

	        if (GetMargin() !=0f)
	        {
		        IndexedVector3 vecnorm = vec;
		        if (vecnorm.LengthSquared() < (MathUtil.SIMD_EPSILON*MathUtil.SIMD_EPSILON))
		        {
			        vecnorm = new IndexedVector3(-1);
		        } 
		        vecnorm.Normalize();
		        supVertex+= GetMargin() * vecnorm;
	        }
	        return supVertex;
        }

        public IndexedVector3 GetImplicitShapeDimensions()
        {
            return m_implicitShapeDimensions;
        }

        public void SetSafeMargin(float minDimension)
        {
            SetSafeMargin(minDimension, 0.1f);
        }

	    public void	SetSafeMargin(float minDimension, float defaultMarginMultiplier)
	    {
		    float safeMargin = defaultMarginMultiplier*minDimension;
		    if (safeMargin < GetMargin())
		    {
			    SetMargin(safeMargin);
		    }
	    }

	    public void SetSafeMargin(ref IndexedVector3 halfExtents)
        {
            SetSafeMargin(ref halfExtents,0.1f);
        }
	    
        public void SetSafeMargin(ref IndexedVector3 halfExtents, float defaultMarginMultiplier)
	    {
		    //see http://code.google.com/p/bullet/issues/detail?id=349
		    //this margin check could could be added to other collision shapes too,
		    //or add some assert/warning somewhere
		    float minDimension=halfExtents[halfExtents.MinAxis()]; 		
		    SetSafeMargin(minDimension, defaultMarginMultiplier);
	    }


	    ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	    public override void GetAabb(ref IndexedMatrix t,out IndexedVector3 aabbMin,out IndexedVector3 aabbMax)
	    {
		    GetAabbSlow(ref t,out aabbMin,out aabbMax);
	    }



        public override void GetAabbSlow(ref IndexedMatrix trans, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
        {
	        float margin = GetMargin();
            aabbMin = IndexedVector3.Zero;
            aabbMax = IndexedVector3.Zero;
            for (int i = 0; i < 3; i++)
	        {
		        IndexedVector3 vec = new IndexedVector3();
		        vec[i] = 1f;

                IndexedVector3 temp = vec * trans._basis;
				IndexedVector3 sv = LocalGetSupportingVertex(ref temp);
                IndexedVector3 tmp = trans * sv;
				aabbMax[i] = tmp[i] + margin;
				vec[i] = -1f;

                temp = vec * trans._basis;
				sv = LocalGetSupportingVertex(ref temp);
                tmp = trans * sv;
				aabbMin[i] = tmp[i] - margin;
	        }
        }

        public override void SetLocalScaling(ref IndexedVector3 scaling)
        {
            m_localScaling = scaling.Absolute();
        }

        public override IndexedVector3 GetLocalScaling()
	    {
		    return m_localScaling;
	    }

	    public IndexedVector3 GetLocalScalingNV()
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
	
	    public override void GetPreferredPenetrationDirection(int index, out IndexedVector3 penetrationVector)
	    {
            Debug.Assert(false);
            penetrationVector = IndexedVector3.Zero;
        }


        protected IndexedVector3 m_localScaling;
        protected IndexedVector3 m_implicitShapeDimensions;
        protected float m_collisionMargin;
        protected float m_padding;
    }

    public class ConvexInternalAabbCachingShape : ConvexInternalShape
    {
        protected IndexedVector3 m_localAabbMin;
        protected IndexedVector3 m_localAabbMax;
        protected bool m_isLocalAabbValid;


        protected void SetCachedLocalAabb(ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax)
        {
            m_isLocalAabbValid = true;
            m_localAabbMin = aabbMin;
            m_localAabbMax = aabbMax;
        }

        public void GetCachedLocalAabb(out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
        {
            Debug.Assert(m_isLocalAabbValid);
            aabbMin = m_localAabbMin;
            aabbMax = m_localAabbMax;
        }

        public void GetNonvirtualAabb(ref IndexedMatrix trans, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax, float margin)
        {
            //lazy evaluation of local aabb
            Debug.Assert(m_isLocalAabbValid);
            AabbUtil2.TransformAabb(ref m_localAabbMin, ref m_localAabbMax, margin, ref trans, out aabbMin, out aabbMax);
        }

        public override IndexedVector3 LocalGetSupportingVertexWithoutMargin(ref IndexedVector3 vec)
        {
            return IndexedVector3.Zero;
        }


        //public:

        //    virtual void	setLocalScaling(const btVector3& scaling);

        //    virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;

        //    void	recalcLocalAabb();


        public ConvexInternalAabbCachingShape()
        {
            m_localAabbMin = new IndexedVector3(1);
            m_localAabbMax = new IndexedVector3(-1);
            m_isLocalAabbValid = false;
        }

        public override void GetAabb(ref IndexedMatrix trans, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
        {
            GetNonvirtualAabb(ref trans, out aabbMin, out aabbMax, GetMargin());
        }

        public override void SetLocalScaling(ref IndexedVector3 scaling)
        {
            base.SetLocalScaling(ref scaling);
            RecalcLocalAabb();
        }

        public virtual void RecalcLocalAabb()
        {
	        m_isLocalAabbValid = true;
        	
	        #if true
            //fixme - make a static list.
            IndexedVector4[] _supporting = new IndexedVector4[6];
	        BatchedUnitVectorGetSupportingVertexWithoutMargin(_directions, _supporting, 6);
        	
	        for ( int i = 0; i < 3; ++i )
	        {
                IndexedVector3 s0 = new IndexedVector3(_supporting[i]);
                IndexedVector3 s1 = new IndexedVector3(_supporting[i+3]);

                m_localAabbMax[i] = s0[i] + m_collisionMargin;
                m_localAabbMin[i] = s1[i] - m_collisionMargin;
            }
        	
	        #else

	        for (int i=0;i<3;i++)
	        {
		        btVector3 vec(float(0.),float(0.),float(0.));
		        vec[i] = float(1.);
		        btVector3 tmp = localGetSupportingVertex(vec);
		        m_localAabbMax[i] = tmp[i]+m_collisionMargin;
		        vec[i] = float(-1.);
		        tmp = localGetSupportingVertex(vec);
		        m_localAabbMin[i] = tmp[i]-m_collisionMargin;
	        }
	        #endif
        }

        public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IndexedVector3[] vectors, IndexedVector4[] supportVerticesOut, int numVectors)
        {
        }


        public static IndexedVector3[] _directions = new IndexedVector3[]
	        {
		        new IndexedVector3( 1.0f,  0.0f,  0.0f),
		        new IndexedVector3( 0.0f,  1.0f,  0.0f),
		        new IndexedVector3( 0.0f,  0.0f,  1.0f),
		        new IndexedVector3( -1.0f, 0.0f,  0.0f),
		        new IndexedVector3( 0.0f, -1.0f,  0.0f),
		        new IndexedVector3( 0.0f,  0.0f, -1.0f)
	        };
		

    }
}
