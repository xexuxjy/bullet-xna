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
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletCollision
{
    public class CylinderShape : ConvexInternalShape
    {
		public CylinderShape(Vector3 halfExtents)
			: this(ref halfExtents)
		{
		}

		
		public CylinderShape (ref Vector3 halfExtents)
        {
            m_upAxis = 1;
            m_shapeType = BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE;
	        Vector3 margin = new Vector3(GetMargin());
	        m_implicitShapeDimensions = (halfExtents * m_localScaling) - margin;
        }

	    public virtual Vector3 GetHalfExtentsWithMargin()
	    {
		    Vector3 halfExtents = GetHalfExtentsWithoutMargin();
		    Vector3 margin = new Vector3(GetMargin());
		    halfExtents += margin;
		    return halfExtents;
	    }
    	
        public override void CalculateLocalInertia(float mass, out Vector3 inertia)
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
	Vector3 halfExtents = GetHalfExtentsWithMargin();	// get cylinder dimension
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
    float radiusExtent = MathUtil.VectorComponent(ref halfExtents,idxRadius);
    float heightExtent = MathUtil.VectorComponent(ref halfExtents,idxHeight);

	radius2 = radiusExtent * radiusExtent;
	height2 = 4.0f * heightExtent * heightExtent;

	// calculate tensor terms
	float t1 = div12 * height2 + div4 * radius2;
	float t2 = div2 * radius2;

	switch (m_upAxis)	// set diagonal elements of inertia tensor
	{
		case 0:		// cylinder is aligned along x
			inertia = new Vector3(t2,t1,t1);
			break;
		case 2:		// cylinder is aligned along z
			inertia = new Vector3(t1,t1,t2);
			break;
		default:	// cylinder is aligned along y
			inertia = new Vector3(t1,t2,t1);
            break;
	}
#else
	        //approximation of box shape, todo: implement cylinder shape inertia before people notice ;-)
	        Vector3 halfExtents = GetHalfExtentsWithMargin();

	        float lx=2.0f*(halfExtents.X);
	        float ly=2.0f*(halfExtents.Y);
	        float lz=2.0f*(halfExtents.Z);

	        inertia = new Vector3(mass/12.0f * (ly*ly + lz*lz),
					        mass/12.0f * (lx*lx + lz*lz),
					        mass/12.0f * (lx*lx + ly*ly));
#endif //USE_BOX_INERTIA_APPROXIMATION
        }

	    public override void SetMargin(float collisionMargin)
	    {
		    //correct the m_implicitShapeDimensions for the margin
		    Vector3 oldMargin = new Vector3(GetMargin());
		    Vector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
    		
		    base.SetMargin(collisionMargin);
		    Vector3 newMargin = new Vector3(GetMargin());
		    m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;
	    }



	    public Vector3 GetHalfExtentsWithoutMargin() 
	    {
		    return m_implicitShapeDimensions;//changed in Bullet 2.63: assume the scaling and margin are included
	    }
	    ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
        public override void GetAabb(ref Matrix trans, out Vector3 aabbMin, out Vector3 aabbMax)
        {
            //skip the box 'getAabb'
            // FIXME - Don't think we can call on it directly as below
            //PolyhedralConvexShape.getAabb(ref trans,ref aabbMin,ref aabbMax);
            //base.getAabb(ref trans,ref aabbMin,ref aabbMax);
            MathUtil.TransformAabb(GetHalfExtentsWithoutMargin(), GetMargin(), trans, out aabbMin, out aabbMax);
        }

        public override Vector3 LocalGetSupportingVertexWithoutMargin(ref Vector3 vec)
        {
            return CylinderLocalSupportX(GetHalfExtentsWithoutMargin(), vec);
        }

        public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector4[] supportVerticesOut, int numVectors)
        {
            Vector3 halfExtents = GetHalfExtentsWithoutMargin();
            for (int i = 0; i < numVectors; i++)
            {
                supportVerticesOut[i] = new Vector4(CylinderLocalSupportY(halfExtents, vectors[i]),0);
            }
        }

	    public override Vector3 LocalGetSupportingVertex(ref Vector3 vec)
	    {
		    Vector3 supVertex;
		    supVertex = LocalGetSupportingVertexWithoutMargin(ref vec);
    		
		    if (GetMargin() != 0f )
		    {
			    Vector3 vecnorm = vec;
			    if (vecnorm.LengthSquared() < (MathUtil.SIMD_EPSILON*MathUtil.SIMD_EPSILON))
			    {
				    vecnorm = new Vector3(-1f);
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

        public Vector3 CylinderLocalSupportX(Vector3 halfExtents,Vector3 v) 
        {
            return CylinderLocalSupportX(ref halfExtents,ref v);
        }

        public Vector3 CylinderLocalSupportX(ref Vector3 halfExtents, ref Vector3 v)
        {
            return CylinderLocalSupport(ref halfExtents, ref v, 0, 1, 0, 2);
        }

        public Vector3 CylinderLocalSupportY(Vector3 halfExtents, Vector3 v)
        {
            return CylinderLocalSupportY(ref halfExtents, ref v);
        }
        
        public Vector3 CylinderLocalSupportY(ref Vector3 halfExtents,ref Vector3 v) 
        {
            return CylinderLocalSupport(ref halfExtents,ref v,1,0,1,2);
        }

        public Vector3 CylinderLocalSupportZ(Vector3 halfExtents, Vector3 v)
        {
            return CylinderLocalSupportZ(ref halfExtents, ref v);
        }

        public Vector3 CylinderLocalSupportZ(ref Vector3 halfExtents,ref Vector3 v) 
        {
            return CylinderLocalSupport(ref halfExtents,ref v,2,0,2,1);
        }


        private Vector3 CylinderLocalSupport(ref Vector3 halfExtents, ref Vector3 v, int cylinderUpAxis, int XX, int YY, int ZZ) 
        {
            //mapping depends on how cylinder local orientation is
            // extents of the cylinder is: X,Y is for radius, and Z for height

            float radius = MathUtil.VectorComponent(ref halfExtents, XX);
            float halfHeight = MathUtil.VectorComponent(ref halfExtents,cylinderUpAxis);

            Vector3 tmp = new Vector3();
            float d =1f;

            float vx = MathUtil.VectorComponent(ref v, XX);
            float vy = MathUtil.VectorComponent(ref v, YY);
            float vz = MathUtil.VectorComponent(ref v, ZZ);


            float s = (float)Math.Sqrt(vx * vx + vz * vz);
            if (s != 0.0)
            {
                d = radius / s;  
	            MathUtil.VectorComponent(ref tmp,XX,(vx * d));
	            MathUtil.VectorComponent(ref tmp,YY, (vy < 0.0f ? -halfHeight : halfHeight));
	            MathUtil.VectorComponent(ref tmp,ZZ, (vz  * d));
	            return tmp;
            }
            else
            {
                MathUtil.VectorComponent(ref tmp,XX,radius);
	            MathUtil.VectorComponent(ref tmp,YY, (vy < 0.0f ? -halfHeight : halfHeight));
	            MathUtil.VectorComponent(ref tmp,ZZ, 0f);
	            return tmp;
            }
        }

        protected int m_upAxis;
    }


    public class CylinderShapeX : CylinderShape
    {
		public CylinderShapeX(Vector3 halfExtents)
			: this(ref halfExtents)
		{ }

	    public CylinderShapeX (ref Vector3 halfExtents) : base(ref halfExtents)
        {
            m_upAxis = 0;
        }

	    public override Vector3 LocalGetSupportingVertexWithoutMargin(ref Vector3 vec)
        {
            return CylinderLocalSupportX(GetHalfExtentsWithoutMargin(), vec);
        }

        public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector4[] supportVerticesOut, int numVectors)
        {
            Vector3 halfExtents = GetHalfExtentsWithoutMargin();
            for (int i = 0; i < numVectors; i++)
            {
                supportVerticesOut[i] = new Vector4(CylinderLocalSupportX(halfExtents, vectors[i]),0);
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
		public CylinderShapeZ(Vector3 halfExtents)
			: this(ref halfExtents)
		{ }

		public CylinderShapeZ(ref Vector3 halfExtents)
            : base(ref halfExtents)
        {
            m_upAxis = 2;
        }

        public override Vector3 LocalGetSupportingVertexWithoutMargin(ref Vector3 vec)
        {
            return CylinderLocalSupportZ(GetHalfExtentsWithoutMargin(), vec);
        }
		public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector4[] supportVerticesOut, int numVectors)
        {
            Vector3 halfExtents = GetHalfExtentsWithoutMargin();
            for (int i = 0; i < numVectors; i++)
            {
                supportVerticesOut[i] = new Vector4(CylinderLocalSupportZ(halfExtents, vectors[i]),0);
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
