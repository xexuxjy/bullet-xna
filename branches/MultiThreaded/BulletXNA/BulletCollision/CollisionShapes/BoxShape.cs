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
using System.Diagnostics;

using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
    public class BoxShape : PolyhedralConvexShape
    {
	    public virtual IndexedVector3 GetHalfExtentsWithMargin()
	    {
            return GetHalfExtentsWithoutMargin() + new IndexedVector3(GetMargin());
		    //IndexedVector3 halfExtents = GetHalfExtentsWithoutMargin();
		    //IndexedVector3 margin = new IndexedVector3(GetMargin());
		    //halfExtents += margin;
		    //return halfExtents;
	    }
	
	    public virtual IndexedVector3 GetHalfExtentsWithoutMargin()
	    {
		    return m_implicitShapeDimensions;//changed in Bullet 2.63: assume the scaling and margin are included
	    }
	

	    public override IndexedVector3	LocalGetSupportingVertex(ref IndexedVector3 vec)
	    {
		    IndexedVector3 halfExtents = GetHalfExtentsWithoutMargin();
		    IndexedVector3 margin = new IndexedVector3(GetMargin());
		    halfExtents += margin;
    		
		    return new IndexedVector3(MathUtil.FSel(vec.X, halfExtents.X, -halfExtents.X),
                                MathUtil.FSel(vec.Y, halfExtents.Y, -halfExtents.Y),
                                MathUtil.FSel(vec.Z, halfExtents.Z, -halfExtents.Z));
	    }

	    public override IndexedVector3 LocalGetSupportingVertexWithoutMargin(ref IndexedVector3 vec)
	    {
		    IndexedVector3 halfExtents = GetHalfExtentsWithoutMargin();

            return new IndexedVector3(MathUtil.FSel(vec.X, halfExtents.X, -halfExtents.X),
                                MathUtil.FSel(vec.Y, halfExtents.Y, -halfExtents.Y),
                                MathUtil.FSel(vec.Z, halfExtents.Z, -halfExtents.Z));
        }


	    public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IndexedVector3[] vectors,IndexedVector4[] supportVerticesOut,int numVectors)
	    {
		    IndexedVector3 halfExtents = GetHalfExtentsWithoutMargin();
    	
		    for (int i=0;i<numVectors;i++)
		    {
			    IndexedVector3 vec = vectors[i];
                supportVerticesOut[i] = new IndexedVector4(MathUtil.FSel(vec.X, halfExtents.X, -halfExtents.X),
                                MathUtil.FSel(vec.Y, halfExtents.Y, -halfExtents.Y),
                                MathUtil.FSel(vec.Z, halfExtents.Z, -halfExtents.Z),0f);
		    }
	    }

        public BoxShape(IndexedVector3 boxHalfExtents) : this(ref boxHalfExtents) { }

	    public BoxShape(ref IndexedVector3 boxHalfExtents) 
		    : base()
	    {
		    m_shapeType = BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE;
            SetSafeMargin(ref boxHalfExtents);
            IndexedVector3 margin = new IndexedVector3(GetMargin());
		    m_implicitShapeDimensions = (boxHalfExtents * m_localScaling) - margin;
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

        public override void GetAabb(ref IndexedMatrix trans, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
        {
            IndexedVector3 halfExtents = GetHalfExtentsWithoutMargin();
            AabbUtil2.TransformAabb(ref halfExtents, GetMargin(), ref trans, out aabbMin, out aabbMax);
#if DEBUG            
            	if(BulletGlobals.g_streamWriter != null && BulletGlobals.debugBoxShape)
	    {
		    BulletGlobals.g_streamWriter.WriteLine("box::getAabb");
            MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "halfExtentWithout", GetHalfExtentsWithoutMargin());
            MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "transform", trans);
            MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "outMin", aabbMin);
            MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "outMax", aabbMax);
	    }
#endif
        }

        public override void CalculateLocalInertia(float mass, out IndexedVector3 inertia)
        {
	        //float margin = float(0.);
	        IndexedVector3 halfExtents = GetHalfExtentsWithMargin();

	        float lx=2f*(halfExtents.X);
	        float ly=2f*(halfExtents.Y);
	        float lz=2f*(halfExtents.Z);

            float mass12 = mass / 12.0f;
            inertia = new IndexedVector3(mass12 * (ly * ly + lz * lz),
                            mass12 * (lx * lx + lz * lz),
                            mass12 * (lx * lx + ly * ly));
        }

	    public override void GetPlane(out IndexedVector3 planeNormal, out IndexedVector3 planeSupport, int i)
	    {
		    //this plane might not be aligned...
            IndexedVector4 plane;
		    GetPlaneEquation(out plane, i);
            planeNormal = plane.ToVector3();
            IndexedVector3 negNormal = -planeNormal;
		    planeSupport = LocalGetSupportingVertex(ref negNormal);
	    }
	
	    public override int GetNumPlanes()
	    {
		    return 6;
	    }	
	
	    public override int	GetNumVertices()
	    {
		    return 8;
	    }

	    public override int GetNumEdges()
	    {
		    return 12;
	    }


	    public override  void GetVertex(int i, out IndexedVector3 vtx)
	    {
		    IndexedVector3 halfExtents = GetHalfExtentsWithoutMargin();

            vtx = new IndexedVector3(
                halfExtents.X * (1 - (i & 1)) - halfExtents.X * (i & 1),
                halfExtents.Y * (1 - ((i & 2) >> 1)) - halfExtents.Y * ((i & 2) >> 1),
                halfExtents.Z * (1 - ((i & 4) >> 2)) - halfExtents.Z * ((i & 4) >> 2));
	    }
	

	    public virtual void	GetPlaneEquation(out IndexedVector4 plane, int i)
	    {
		    IndexedVector3 halfExtents = GetHalfExtentsWithoutMargin();


		    switch (i)
		    {
		    case 0:
                plane = new IndexedVector4(IndexedVector3.Right,-halfExtents.X);
    		    break;
		    case 1:
                plane = new IndexedVector4(IndexedVector3.Left, -halfExtents.X);
			    break;
		    case 2:
                plane = new IndexedVector4(IndexedVector3.Up, -halfExtents.Y);
			    break;
		    case 3:
                plane = new IndexedVector4(IndexedVector3.Down, -halfExtents.Y);
			    break;
		    case 4:
                plane = new IndexedVector4(IndexedVector3.Backward, -halfExtents.Z);
			    break;
		    case 5:
                plane = new IndexedVector4(IndexedVector3.Forward, -halfExtents.Z);
			    break;
		    default:
			    Debug.Assert(false);
                plane = new IndexedVector4();
                break;
		    }
	    }


        public override void GetEdge(int i, out IndexedVector3 pa, out IndexedVector3 pb)
	    //virtual void getEdge(int i,Edge& edge) const
	    {
		    int edgeVert0 = 0;
		    int edgeVert1 = 0;

		    switch (i)
		    {
		    case 0:
				    edgeVert0 = 0;
				    edgeVert1 = 1;
			    break;
		    case 1:
				    edgeVert0 = 0;
				    edgeVert1 = 2;
			    break;
		    case 2:
			    edgeVert0 = 1;
			    edgeVert1 = 3;

			    break;
		    case 3:
			    edgeVert0 = 2;
			    edgeVert1 = 3;
			    break;
		    case 4:
			    edgeVert0 = 0;
			    edgeVert1 = 4;
			    break;
		    case 5:
			    edgeVert0 = 1;
			    edgeVert1 = 5;

			    break;
		    case 6:
			    edgeVert0 = 2;
			    edgeVert1 = 6;
			    break;
		    case 7:
			    edgeVert0 = 3;
			    edgeVert1 = 7;
			    break;
		    case 8:
			    edgeVert0 = 4;
			    edgeVert1 = 5;
			    break;
		    case 9:
			    edgeVert0 = 4;
			    edgeVert1 = 6;
			    break;
		    case 10:
			    edgeVert0 = 5;
			    edgeVert1 = 7;
			    break;
		    case 11:
			    edgeVert0 = 6;
			    edgeVert1 = 7;
			    break;
		    default:
			   Debug.Assert(false);
               break;
		    }

		    GetVertex(edgeVert0, out pa);
            GetVertex(edgeVert1, out pb);
	    }
	
	    public override bool IsInside(ref IndexedVector3 pt,float tolerance)
	    {
		    IndexedVector3 halfExtents = GetHalfExtentsWithoutMargin();

		    //float minDist = 2*tolerance;
    		
		    bool result =	(pt.X <= (halfExtents.X+tolerance)) &&
						    (pt.X >= (-halfExtents.X-tolerance)) &&
						    (pt.Y <= (halfExtents.Y+tolerance)) &&
						    (pt.Y >= (-halfExtents.Y-tolerance)) &&
						    (pt.Z <= (halfExtents.Z+tolerance)) &&
						    (pt.Z >= (-halfExtents.Z-tolerance));
    		
		    return result;
	    }


	    //debugging
	    public override String GetName()
	    {
		    return "Box";
	    }

	    public override int GetNumPreferredPenetrationDirections()
	    {
		    return 6;
	    }

        public override void GetPreferredPenetrationDirection(int index, out IndexedVector3 penetrationVector)
	    {
		    switch (index)
		    {
		    case 0:
			    penetrationVector = IndexedVector3.Right;
			    break;
		    case 1:
                penetrationVector = IndexedVector3.Left;
			    break;
		    case 2:
			    penetrationVector = IndexedVector3.Up;
			    break;
		    case 3:
			    penetrationVector = IndexedVector3.Down;
			    break;
		    case 4:
			    penetrationVector = IndexedVector3.Backward;
			    break;
		    case 5:
                penetrationVector = IndexedVector3.Forward;
			    break;
		    default:
                Debug.Assert(false);
                penetrationVector = IndexedVector3.Zero;
                break;
		    }
#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugBoxShape)
            {
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "Box::GetPreferredPenetrationDirection", penetrationVector);
            }
#endif
            

	    }

    }
}
