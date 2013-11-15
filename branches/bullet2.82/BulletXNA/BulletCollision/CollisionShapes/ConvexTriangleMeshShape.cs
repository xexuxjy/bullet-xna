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
    public class ConvexTriangleMeshShape : PolyhedralConvexAabbCachingShape
    {

	    public ConvexTriangleMeshShape(StridingMeshInterface meshInterface, bool calcAabb)
        {
	        m_shapeType = BroadphaseNativeTypes.CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE;
            m_stridingMesh = meshInterface;
	        if ( calcAabb )
            {
		        RecalcLocalAabb();
            }
        }

        public override void Cleanup()
        {
            base.Cleanup();
        }

	    public StridingMeshInterface GetMeshInterface()
	    {
		    return m_stridingMesh;
	    }

	    public override IndexedVector3 LocalGetSupportingVertex(ref IndexedVector3 vec)
        {
	        IndexedVector3 supVertex = LocalGetSupportingVertexWithoutMargin(ref vec);

	        if (GetMargin() != 0f)
	        {
		        IndexedVector3 vecnorm = vec;
		        if (vecnorm.LengthSquared() < (MathUtil.SIMD_EPSILON*MathUtil.SIMD_EPSILON))
		        {
			        vecnorm = new IndexedVector3(-1f);
		        } 
		        vecnorm.Normalize();
		        supVertex+= GetMargin() * vecnorm;
	        }
	        return supVertex;

        }

        public override IndexedVector3 LocalGetSupportingVertexWithoutMargin(ref IndexedVector3 vec0)
        {
	        IndexedVector3 supVec = IndexedVector3.Zero;

	        IndexedVector3 vec = vec0;
	        float lenSqr = vec.LengthSquared();
	        if (lenSqr < 0.0001f)
	        {
                vec = new IndexedVector3(1, 0, 0);
	        } 
            else
	        {
                float rlen = (1.0f) / (float)Math.Sqrt(lenSqr);
                vec *= rlen;
                //vec.Normalize();
            }

	        LocalSupportVertexCallback supportCallback = new LocalSupportVertexCallback(ref vec);
	        IndexedVector3 aabbMax = new IndexedVector3(float.MaxValue);
            IndexedVector3 aabbMin = -aabbMax;
	        m_stridingMesh.InternalProcessAllTriangles(supportCallback,ref aabbMin,ref aabbMax);
	        supVec = supportCallback.GetSupportVertexLocal();

	        return supVec;

        }
        public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IndexedVector3[] vectors, IndexedVector4[] supportVerticesOut, int numVectors)
        {
	        for (int j=0;j<numVectors;j++)
	        {
		        IndexedVector3 vec = vectors[j];
		        LocalSupportVertexCallback	supportCallback = new LocalSupportVertexCallback(ref vec);
                IndexedVector3 aabbMax = MathUtil.MAX_VECTOR;
                IndexedVector3 aabbMin = MathUtil.MIN_VECTOR;

		        m_stridingMesh.InternalProcessAllTriangles(supportCallback,ref aabbMin,ref aabbMax);
		        supportVerticesOut[j] = new IndexedVector4(supportCallback.GetSupportVertexLocal(),0);
	        }
        }
	
	    //debugging
        public override String GetName()
        {
            return "ConvexTrimesh";
        }

        public override int GetNumVertices()
        {
            return 0;
        }

        public override int GetNumEdges()
        {
            return 0;
        }
        public override void GetEdge(int i, out IndexedVector3 pa, out IndexedVector3 pb)
        {
            Debug.Assert(false);
            pa = IndexedVector3.Zero;
            pb = IndexedVector3.Zero;
        }
        public override void GetVertex(int i, out IndexedVector3 vtx)
        {
            Debug.Assert(false);
            vtx = IndexedVector3.Zero;
        }
        public override int GetNumPlanes()
        {
            return 0;
        }

        public override void GetPlane(out IndexedVector3 planeNormal, out IndexedVector3 planeSupport, int i)
        {
            Debug.Assert(false);
            planeNormal = IndexedVector3.Zero;
            planeSupport = IndexedVector3.Zero;
        }

        public override bool IsInside(ref IndexedVector3 pt, float tolerance)
        {
            Debug.Assert(false);
            return false;
        }

        public override void SetLocalScaling(ref IndexedVector3 scaling)
        {
            m_stridingMesh.SetScaling(ref scaling);
            RecalcLocalAabb();
        }

        public override IndexedVector3 GetLocalScaling()
        {
            return m_stridingMesh.GetScaling();
        }

	    ///computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes of the moment of inertia
	    ///and the center of mass to the current coordinate system. A mass of 1 is assumed, for other masses just multiply the computed "inertia"
	    ///by the mass. The resulting transform "principal" has to be applied inversely to the mesh in order for the local coordinate system of the
	    ///shape to be centered at the center of mass and to coincide with the principal axes. This also necessitates a correction of the world transform
	    ///of the collision object by the principal transform. This method also computes the volume of the convex mesh.
        public void CalculatePrincipalAxisTransform(ref IndexedMatrix principal, out IndexedVector3 inertia, float volume)
        {
            CenterCallback centerCallback = new CenterCallback();
            IndexedVector3 aabbMax = MathUtil.MAX_VECTOR;
            IndexedVector3 aabbMin = MathUtil.MIN_VECTOR;
            m_stridingMesh.InternalProcessAllTriangles(centerCallback, ref aabbMin, ref aabbMax);
            IndexedVector3 center = centerCallback.GetCenter();
            principal._origin = center;
            volume = centerCallback.GetVolume();

            InertiaCallback inertiaCallback = new InertiaCallback(ref center);
            m_stridingMesh.InternalProcessAllTriangles(inertiaCallback, ref aabbMax, ref aabbMax);

            IndexedBasisMatrix i = inertiaCallback.GetInertia();
            i.Diagonalize(out principal, 0.00001f, 20);
            //i.diagonalize(principal.getBasis(), 0.00001f, 20);
            inertia = new IndexedVector3(i[0,0],i[1,1],i[2,2]);
            inertia /= volume;
        }

	    private StridingMeshInterface m_stridingMesh;

    }


    public class LocalSupportVertexCallback: IInternalTriangleIndexCallback
    {

        public virtual bool graphics()
        {
            return false;
        }

	    public LocalSupportVertexCallback(ref IndexedVector3 supportVecLocal)
	    {
            m_supportVertexLocal = IndexedVector3.Zero;
            m_supportVecLocal = supportVecLocal;
            m_maxDot = float.MinValue;
	    }

        public virtual void InternalProcessTriangleIndex(IndexedVector3[] triangle, int partId, int triangleIndex)
	    {
		    for (int i=0;i<3;i++)
		    {
			    float dot = m_supportVecLocal.Dot(ref triangle[i]);
			    if (dot > m_maxDot)
			    {
				    m_maxDot = dot;
				    m_supportVertexLocal = triangle[i];
			    }
		    }
	    }
    	
	    public IndexedVector3 GetSupportVertexLocal()
	    {
		    return m_supportVertexLocal;
	    }

        public void Cleanup()
        {
        }

	    private IndexedVector3 m_supportVertexLocal;
	    public float m_maxDot;
	    public IndexedVector3 m_supportVecLocal;

    };

    public class CenterCallback: IInternalTriangleIndexCallback
    {
        public CenterCallback()
        {
            first = true;
            reference = new IndexedVector3();
            sum = new IndexedVector3();
            volume = 0f;
        }

        public virtual bool graphics()
        {
            return false;
        }

        public virtual void InternalProcessTriangleIndex(IndexedVector3[] triangle, int partId, int triangleIndex)
        {
            if (first)
            {
                reference = triangle[0];
                first = false;
            }
            else
            {
                IndexedVector3 a = triangle[0] - reference;
                IndexedVector3 b = triangle[1] - reference;
                IndexedVector3 c = triangle[2] - reference;
                float vol = Math.Abs(a.Triple(ref b,ref c));
                sum += (.25f * vol) * ((triangle[0] + triangle[1] + triangle[2] + reference));
                volume += vol;
            }
        }
      
        public IndexedVector3 GetCenter()
        {
            return (volume > 0) ? sum / volume : reference;
        }

        public float GetVolume()
        {
            return volume * (1f / 6f);
        }

        public void Cleanup()
        {

        }


        bool first;
        IndexedVector3 reference;
        IndexedVector3 sum;
        float volume;

    }

    public class InertiaCallback: IInternalTriangleIndexCallback
    {

        public virtual bool graphics()
        {
            return false;
        }

        public InertiaCallback(ref IndexedVector3 center)
        {
            m_sum = new IndexedBasisMatrix();
            m_center = center;
        }

        public virtual void InternalProcessTriangleIndex(IndexedVector3[] triangle, int partId, int triangleIndex)
        {
            IndexedBasisMatrix i = new IndexedBasisMatrix();
            IndexedVector3 a = triangle[0] - m_center;
            IndexedVector3 b = triangle[1] - m_center;
            IndexedVector3 c = triangle[2] - m_center;
            float volNeg = -Math.Abs(a.Triple(ref b, ref c)) * (1.0f / 6.0f);
             for (int j = 0; j < 3; j++)
             {
                for (int k = 0; k <= j; k++)
                {
                   i[j,k] = i[k,j] = volNeg * (0.1f * (a[j] * a[k] + b[j] * b[k] + c[j] * c[k])
                      + 0.05f * (a[j] * b[k] + a[k] * b[j] + a[j] * c[k] + a[k] * c[j] + b[j] * c[k] + b[k] * c[j]));
                }
             }
         float i00 = -i._el0.X;
         float i11 = -i._el1.Y;
         float i22 = -i._el2.Z;
         i[0,0] = i11 + i22; 
         i[1,1] = i22 + i00; 
         i[2,2] = i00 + i11;
         m_sum._el0 += i._el0;
         m_sum._el1 += i._el1;
         m_sum._el2 += i._el2;
        }

        public IndexedBasisMatrix GetInertia()
        {
            return m_sum;
        }

        public void Cleanup()
        {

        }

        IndexedBasisMatrix m_sum;
        IndexedVector3 m_center;
    }

}
