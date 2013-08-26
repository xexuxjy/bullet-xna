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

using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
    public class CylinderShape : ConvexInternalShape
    {
		public CylinderShape(IndexedVector3 halfExtents)
			: this(ref halfExtents)
		{
		}

		
		public CylinderShape (ref IndexedVector3 halfExtents)
        {
            m_upAxis = 1;
            SetSafeMargin(ref halfExtents);
            m_shapeType = BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE;
	        IndexedVector3 margin = new IndexedVector3(GetMargin());
	        m_implicitShapeDimensions = (halfExtents * m_localScaling) - margin;
        }

	    public virtual IndexedVector3 GetHalfExtentsWithMargin()
	    {
		    IndexedVector3 halfExtents = GetHalfExtentsWithoutMargin();
		    IndexedVector3 margin = new IndexedVector3(GetMargin());
		    halfExtents += margin;
		    return halfExtents;
	    }
    	
        public override void CalculateLocalInertia(float mass, out IndexedVector3 inertia)
        {


//Until Bullet 2.77 a box approximation was used, so uncomment this if you need backwards compatibility
//#define USE_BOX_INERTIA_APPROXIMATION 1
#if  !USE_BOX_INERTIA_APPROXIMATION

	/*
	cylinder is defined as following:
	*
	* - principle axis aligned along y by default, radius in x, z-value not used
	* - for btCylinderShapeX: principle axis aligned along x, radius in y direction, z-value not used
	* - for btCylinderShapeZ: principle axis aligned along z, radius in x direction, y-value not used
	*
	*/

	float radius2;	// square of cylinder radius
	float height2;	// square of cylinder height
	IndexedVector3 halfExtents = GetHalfExtentsWithMargin();	// get cylinder dimension
	float div12 = mass / 12.0f;
	float div4 = mass / 4.0f;
	float div2 = mass / 2.0f;
	int idxRadius, idxHeight;

	switch (m_upAxis)	// get indices of radius and height of cylinder
	{
		case 0:		// cylinder is aligned along x
			idxRadius = 1;
			idxHeight = 0;
			break;
		case 2:		// cylinder is aligned along z
			idxRadius = 0;
			idxHeight = 2;
			break;
		default:	// cylinder is aligned along y
			idxRadius = 0;
			idxHeight = 1;
            break;
	}

	// calculate squares
	radius2 = halfExtents[idxRadius] * halfExtents[idxRadius];
	height2 = 4.0f * halfExtents[idxHeight] * halfExtents[idxHeight];


	// calculate tensor terms
	float t1 = div12 * height2 + div4 * radius2;
	float t2 = div2 * radius2;

	switch (m_upAxis)	// set diagonal elements of inertia tensor
	{
		case 0:		// cylinder is aligned along x
			inertia = new IndexedVector3(t2,t1,t1);
			break;
		case 2:		// cylinder is aligned along z
			inertia = new IndexedVector3(t1,t1,t2);
			break;
		default:	// cylinder is aligned along y
			inertia = new IndexedVector3(t1,t2,t1);
            break;
	}
#else
	        //approximation of box shape, todo: implement cylinder shape inertia before people notice ;-)
	        IndexedVector3 halfExtents = GetHalfExtentsWithMargin();

	        float lx=2.0f*(halfExtents.X);
	        float ly=2.0f*(halfExtents.Y);
	        float lz=2.0f*(halfExtents.Z);

	        inertia = new IndexedVector3(mass/12.0f * (ly*ly + lz*lz),
					        mass/12.0f * (lx*lx + lz*lz),
					        mass/12.0f * (lx*lx + ly*ly));
#endif //USE_BOX_INERTIA_APPROXIMATION
        }

	    public override void SetMargin(float collisionMargin)
	    {
		    //correct the m_implicitShapeDimensions for the margin
		    IndexedVector3 oldMargin = new IndexedVector3(GetMargin());
		    IndexedVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
    		
		    base.SetMargin(collisionMargin);
		    IndexedVector3 newMargin = new IndexedVector3(GetMargin());
		    m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;
	    }



	    public IndexedVector3 GetHalfExtentsWithoutMargin() 
	    {
		    return m_implicitShapeDimensions;//changed in Bullet 2.63: assume the scaling and margin are included
	    }
	    ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
        public override void GetAabb(ref IndexedMatrix trans, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
        {
            //skip the box 'getAabb'
            // FIXME - Don't think we can call on it directly as below
            //PolyhedralConvexShape.getAabb(ref trans,ref aabbMin,ref aabbMax);
            //base.getAabb(ref trans,ref aabbMin,ref aabbMax);
            AabbUtil2.TransformAabb(GetHalfExtentsWithoutMargin(), GetMargin(), ref trans, out aabbMin, out aabbMax);
        }

        public override IndexedVector3 LocalGetSupportingVertexWithoutMargin(ref IndexedVector3 vec)
        {
            return CylinderLocalSupportX(GetHalfExtentsWithoutMargin(), vec);
        }

        public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IndexedVector3[] vectors, IndexedVector4[] supportVerticesOut, int numVectors)
        {
            IndexedVector3 halfExtents = GetHalfExtentsWithoutMargin();
            for (int i = 0; i < numVectors; i++)
            {
                supportVerticesOut[i] = new IndexedVector4(CylinderLocalSupportY(halfExtents, vectors[i]),0);
            }
        }

	    public override IndexedVector3 LocalGetSupportingVertex(ref IndexedVector3 vec)
	    {
		    IndexedVector3 supVertex;
		    supVertex = LocalGetSupportingVertexWithoutMargin(ref vec);
    		
		    if (GetMargin() != 0f )
		    {
			    IndexedVector3 vecnorm = vec;
			    if (vecnorm.LengthSquared() < (MathUtil.SIMD_EPSILON*MathUtil.SIMD_EPSILON))
			    {
				    vecnorm = new IndexedVector3(-1f);
			    } 
			    vecnorm.Normalize();
			    supVertex += GetMargin() * vecnorm;
		    }
		    return supVertex;
	    }

        public int GetUpAxis()
	    {
		    return m_upAxis;
	    }

	    public virtual float GetRadius()
	    {
		    return GetHalfExtentsWithMargin().X;
    	}

	    //debugging
	    public override String GetName()
	    {
		    return "CylinderY";
	    }

        public IndexedVector3 CylinderLocalSupportX(IndexedVector3 halfExtents,IndexedVector3 v) 
        {
            return CylinderLocalSupportX(ref halfExtents,ref v);
        }

        public IndexedVector3 CylinderLocalSupportX(ref IndexedVector3 halfExtents, ref IndexedVector3 v)
        {
            return CylinderLocalSupport(ref halfExtents, ref v, 0, 1, 0, 2);
        }

        public IndexedVector3 CylinderLocalSupportY(IndexedVector3 halfExtents, IndexedVector3 v)
        {
            return CylinderLocalSupportY(ref halfExtents, ref v);
        }
        
        public IndexedVector3 CylinderLocalSupportY(ref IndexedVector3 halfExtents,ref IndexedVector3 v) 
        {
            return CylinderLocalSupport(ref halfExtents,ref v,1,0,1,2);
        }

        public IndexedVector3 CylinderLocalSupportZ(IndexedVector3 halfExtents, IndexedVector3 v)
        {
            return CylinderLocalSupportZ(ref halfExtents, ref v);
        }

        public IndexedVector3 CylinderLocalSupportZ(ref IndexedVector3 halfExtents,ref IndexedVector3 v) 
        {
            return CylinderLocalSupport(ref halfExtents,ref v,2,0,2,1);
        }


        private IndexedVector3 CylinderLocalSupport(ref IndexedVector3 halfExtents, ref IndexedVector3 v, int cylinderUpAxis, int XX, int YY, int ZZ) 
        {
            //mapping depends on how cylinder local orientation is
            // extents of the cylinder is: X,Y is for radius, and Z for height

            float radius = halfExtents[XX];
            float halfHeight = halfExtents[cylinderUpAxis];

            IndexedVector3 tmp = new IndexedVector3();
            float d =1f;

            float vx = v[XX];
            float vy = v[YY];
            float vz = v[ZZ];


            float s = (float)Math.Sqrt(vx * vx + vz * vz);
            if (s != 0.0)
            {
                d = radius / s;
                tmp[XX] = v[XX] * d;
                tmp[YY] = v[YY] < 0.0f ? -halfHeight : halfHeight;
                tmp[ZZ] = v[ZZ] * d;
                return tmp;
            }
            else
            {
                tmp[XX] = radius;
                tmp[YY] = v[YY] < 0.0f ? -halfHeight : halfHeight;
                tmp[ZZ] = 0.0f;
                return tmp;
            }
        }

        protected int m_upAxis;
    }


    public class CylinderShapeX : CylinderShape
    {
		public CylinderShapeX(IndexedVector3 halfExtents)
			: this(ref halfExtents)
		{ }

	    public CylinderShapeX (ref IndexedVector3 halfExtents) : base(ref halfExtents)
        {
            m_upAxis = 0;
        }

	    public override IndexedVector3 LocalGetSupportingVertexWithoutMargin(ref IndexedVector3 vec)
        {
            return CylinderLocalSupportX(GetHalfExtentsWithoutMargin(), vec);
        }

        public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IndexedVector3[] vectors, IndexedVector4[] supportVerticesOut, int numVectors)
        {
            IndexedVector3 halfExtents = GetHalfExtentsWithoutMargin();
            for (int i = 0; i < numVectors; i++)
            {
                supportVerticesOut[i] = new IndexedVector4(CylinderLocalSupportX(halfExtents, vectors[i]),0);
            }
        }
    	
		    //debugging
	    public override String GetName()
	    {
		    return "CylinderX";
	    }

        public override float GetRadius()
	    {
		    return GetHalfExtentsWithMargin().Y;
	    }

    }

    public class CylinderShapeZ : CylinderShape
    {
		public CylinderShapeZ(IndexedVector3 halfExtents)
			: this(ref halfExtents)
		{ }

		public CylinderShapeZ(ref IndexedVector3 halfExtents)
            : base(ref halfExtents)
        {
            m_upAxis = 2;
        }

        public override IndexedVector3 LocalGetSupportingVertexWithoutMargin(ref IndexedVector3 vec)
        {
            return CylinderLocalSupportZ(GetHalfExtentsWithoutMargin(), vec);
        }
		public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IndexedVector3[] vectors, IndexedVector4[] supportVerticesOut, int numVectors)
        {
            IndexedVector3 halfExtents = GetHalfExtentsWithoutMargin();
            for (int i = 0; i < numVectors; i++)
            {
                supportVerticesOut[i] = new IndexedVector4(CylinderLocalSupportZ(halfExtents, vectors[i]),0);
            }
        }

        //debugging
        public override String GetName()
        {
            return "CylinderZ";
        }

        public override float GetRadius()
        {
            return GetHalfExtentsWithMargin().X;
        }
    }

}
