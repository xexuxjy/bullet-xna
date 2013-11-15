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
    public class CapsuleShape : ConvexInternalShape
    {
        protected CapsuleShape()
        {
            m_shapeType = BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE;
        }

        public CapsuleShape(float radius,float height) : this()
        {
	        m_upAxis = 1;
	        m_implicitShapeDimensions = new IndexedVector3(radius,0.5f*height,radius);
        }
	    ///CollisionShape Interface
        public override void CalculateLocalInertia(float mass, out IndexedVector3 inertia)
        {
	        IndexedMatrix ident = IndexedMatrix.Identity;
        	
	        float radius = GetRadius();

	        IndexedVector3 halfExtents = new IndexedVector3(radius);
            float val = halfExtents[GetUpAxis()];
	        halfExtents[GetUpAxis()] = val +GetHalfHeight();

	        const float margin = CollisionMargin.CONVEX_DISTANCE_MARGIN;

	        float lx=2f*(halfExtents.X+margin);
	        float ly=2f*(halfExtents.Y+margin);
	        float lz=2f*(halfExtents.Z+margin);
	        float x2 = lx*lx;
	        float y2 = ly*ly;
	        float z2 = lz*lz;
	        float scaledmass = mass * 0.08333333f;

	        inertia = scaledmass * (new IndexedVector3(y2+z2,x2+z2,x2+y2));
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

	    public override void SetLocalScaling(ref IndexedVector3 scaling)
	    {
		    IndexedVector3 oldMargin = new IndexedVector3(GetMargin());
		    IndexedVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
		    IndexedVector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

		    base.SetLocalScaling(ref scaling);

		    m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;

	    }


	    /// btConvexShape Interface
        public override IndexedVector3 LocalGetSupportingVertexWithoutMargin(ref IndexedVector3 vec0)
        {
	        IndexedVector3 supVec = IndexedVector3.Zero;
	        float maxDot = float.MinValue;

	        IndexedVector3 vec = vec0;
	        float lenSqr = vec.LengthSquared();
	        if (lenSqr < 0.0001f)
	        {
		        vec = new IndexedVector3(1,0,0);
	        } 
            else
	        {
		        float rlen = (1.0f) / (float)Math.Sqrt(lenSqr );
		        vec *= rlen;
              //vec.Normalize();
	        }

	        IndexedVector3 vtx;
	        float newDot;
        	
	        float radius = GetRadius();

	        {
		        IndexedVector3 pos = IndexedVector3.Zero;
		        pos[GetUpAxis()] = GetHalfHeight();

		        vtx = pos +vec*(radius) - vec * GetMargin();
		        newDot = vec.Dot(ref vtx);
		        if (newDot > maxDot)
		        {
			        maxDot = newDot;
			        supVec = vtx;
		        }
	        }
	        {
                IndexedVector3 pos = IndexedVector3.Zero;
                pos[GetUpAxis()] = -GetHalfHeight();

                vtx = pos + vec * (radius) - vec * GetMargin();
                newDot = vec.Dot(ref vtx);
                if (newDot > maxDot)
                {
                    maxDot = newDot;
                    supVec = vtx;
                }
            }

	        return supVec;
        }

        public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IndexedVector3[] vectors, IndexedVector4[] supportVerticesOut, int numVectors)
        {
	        float radius = GetRadius();

	        for (int j=0;j<numVectors;j++)
	        {
		        float maxDot = float.MinValue;
		        IndexedVector3 vec = vectors[j];

		        IndexedVector3 vtx;
		        float newDot = 0f;
	            {
		            IndexedVector3 pos = IndexedVector3.Zero;
                    pos[GetUpAxis()] = GetHalfHeight();

		            vtx = pos +vec*(radius) - vec * GetMargin();
                    newDot = vec.Dot(ref vtx);
		            if (newDot > maxDot)
		            {
			            maxDot = newDot;
			            supportVerticesOut[j] = new IndexedVector4(vtx,0);
		            }
	            }
	            {
                    IndexedVector3 pos = IndexedVector3.Zero;
                    pos[GetUpAxis()] = -GetHalfHeight();

                    vtx = pos + vec * (radius) - vec * GetMargin();
                    newDot = vec.Dot(ref vtx);
                    if (newDot > maxDot)
                    {
                        maxDot = newDot;
			            supportVerticesOut[j] = new IndexedVector4(vtx,0);
                    }
                }
	        }
        }
	
	    public override void GetAabb (ref IndexedMatrix trans, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
	    {
	        IndexedVector3 halfExtents = new IndexedVector3(GetRadius());
            halfExtents[m_upAxis] = GetRadius() + GetHalfHeight();

	        halfExtents += new IndexedVector3(GetMargin());
            IndexedBasisMatrix abs_b = trans._basis.Absolute();
            IndexedVector3 center = trans._origin;
            IndexedVector3 extent = new IndexedVector3(abs_b._el0.Dot(ref halfExtents),
                                           abs_b._el1.Dot(ref halfExtents),
                                           abs_b._el2.Dot(ref halfExtents));
    		
	        aabbMin = center - extent;
	        aabbMax = center + extent;

        }

        public override String GetName()
	    {
		    return "CapsuleShape";
	    }

	    public int GetUpAxis()
	    {
		    return m_upAxis;
	    }

	    public float GetRadius()
	    {
		    int radiusAxis = (m_upAxis+2)%3;
		    return m_implicitShapeDimensions[radiusAxis];
	    }

	    public float GetHalfHeight()
	    {
            return m_implicitShapeDimensions[m_upAxis];
	    }
        protected int m_upAxis;

    }

    ///btCapsuleShapeX represents a capsule around the Z axis
    ///the total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
    public class CapsuleShapeX : CapsuleShape
    {
	    public CapsuleShapeX(float radius,float height)
        {
            m_upAxis = 0;
            m_implicitShapeDimensions = new IndexedVector3(0.5f * height, radius, radius);
        }
    		
	    //debugging
	    public override String GetName()
	    {
		    return "CapsuleX";
        }
    }

    ///btCapsuleShapeZ represents a capsule around the Z axis
    ///the total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
    public class CapsuleShapeZ : CapsuleShape
    {
	    public CapsuleShapeZ(float radius,float height)
        {
            m_upAxis = 2;
            m_implicitShapeDimensions= new IndexedVector3(radius, radius, 0.5f * height);
        }
        //debugging
	    public override String GetName()
	    {
		    return "CapsuleZ";
	    }
    }

}
