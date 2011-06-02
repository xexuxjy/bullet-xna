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
using System.Diagnostics;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
    public class ConvexTriangleMeshShape : PolyhedralConvexAabbCachingShape
    {

	    public ConvexTriangleMeshShape(StridingMeshInterface meshInterface, bool calcAabb)
        {
	        m_shapeType = BroadphaseNativeType.ConvexTriangleMeshShape;
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

	    public override Vector3 LocalGetSupportingVertex(ref Vector3 vec)
        {
	        Vector3 supVertex = LocalGetSupportingVertexWithoutMargin(ref vec);

	        if (Margin != 0f)
	        {
		        Vector3 vecnorm = vec;
		        if (vecnorm.LengthSquared() < (MathUtil.SIMD_EPSILON*MathUtil.SIMD_EPSILON))
		        {
			        vecnorm = new Vector3(-1f);
		        } 
		        vecnorm.Normalize();
		        supVertex+= Margin * vecnorm;
	        }
	        return supVertex;

        }

        public override Vector3 LocalGetSupportingVertexWithoutMargin(ref Vector3 vec0)
        {
	        Vector3 supVec = Vector3.Zero;

	        Vector3 vec = vec0;
	        float lenSqr = vec.LengthSquared();
	        if (lenSqr < 0.0001f)
	        {
		        vec = Vector3.Right;
	        } 
            else
	        {
                float rlen = (1.0f) / (float)Math.Sqrt(lenSqr);
                vec *= rlen;
                //vec.Normalize();
            }

	        LocalSupportVertexCallback supportCallback = new LocalSupportVertexCallback(ref vec);
	        Vector3 aabbMax = new Vector3(float.MaxValue);
            Vector3 aabbMin = -aabbMax;
	        m_stridingMesh.InternalProcessAllTriangles(supportCallback,ref aabbMin,ref aabbMax);
	        supVec = supportCallback.GetSupportVertexLocal();

	        return supVec;

        }
        public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector4[] supportVerticesOut, int numVectors)
        {
	        for (int j=0;j<numVectors;j++)
	        {
		        Vector3 vec = vectors[j];
		        LocalSupportVertexCallback	supportCallback = new LocalSupportVertexCallback(ref vec);
                Vector3 aabbMax = MathUtil.MAX_VECTOR;
                Vector3 aabbMin = MathUtil.MIN_VECTOR;

		        m_stridingMesh.InternalProcessAllTriangles(supportCallback,ref aabbMin,ref aabbMax);
		        supportVerticesOut[j] = new Vector4(supportCallback.GetSupportVertexLocal(),0);
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
        public override void GetEdge(int i, out Vector3 pa, out Vector3 pb)
        {
            Debug.Assert(false);
            pa = Vector3.Zero;
            pb = Vector3.Zero;
        }
        public override void GetVertex(int i, out Vector3 vtx)
        {
            Debug.Assert(false);
            vtx = Vector3.Zero;
        }
        public override int GetNumPlanes()
        {
            return 0;
        }

        public override void GetPlane(out Vector3 planeNormal, out Vector3 planeSupport, int i)
        {
            Debug.Assert(false);
            planeNormal = Vector3.Zero;
            planeSupport = Vector3.Zero;
        }

        public override bool IsInside(ref Vector3 pt, float tolerance)
        {
            Debug.Assert(false);
            return false;
        }

        public override void SetLocalScaling(ref Vector3 scaling)
        {
            m_stridingMesh.SetScaling(ref scaling);
            RecalcLocalAabb();
        }

        public override Vector3 GetLocalScaling()
        {
            return m_stridingMesh.GetScaling();
        }

	    ///computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes of the moment of inertia
	    ///and the center of mass to the current coordinate system. A mass of 1 is assumed, for other masses just multiply the computed "inertia"
	    ///by the mass. The resulting transform "principal" has to be applied inversely to the mesh in order for the local coordinate system of the
	    ///shape to be centered at the center of mass and to coincide with the principal axes. This also necessitates a correction of the world transform
	    ///of the collision object by the principal transform. This method also computes the volume of the convex mesh.
        public void CalculatePrincipalAxisTransform(ref Matrix principal, out Vector3 inertia, float volume)
        {
            CenterCallback centerCallback = new CenterCallback();
            Vector3 aabbMax = MathUtil.MAX_VECTOR;
            Vector3 aabbMin = MathUtil.MIN_VECTOR;
            m_stridingMesh.InternalProcessAllTriangles(centerCallback, ref aabbMin, ref aabbMax);
            Vector3 center = centerCallback.GetCenter();
            principal.Translation = center;
            volume = centerCallback.GetVolume();

            InertiaCallback inertiaCallback = new InertiaCallback(ref center);
            m_stridingMesh.InternalProcessAllTriangles(inertiaCallback, ref aabbMax, ref aabbMax);

            Matrix i = inertiaCallback.GetInertia();
            MathUtil.Diagonalize(ref i, ref principal, 0.00001f, 20);
            //i.diagonalize(principal.getBasis(), 0.00001f, 20);
            inertia = new Vector3(i.M11,i.M22,i.M33);
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

	    public LocalSupportVertexCallback(ref Vector3 supportVecLocal)
	    {
            m_supportVertexLocal = Vector3.Zero;
            m_supportVecLocal = supportVecLocal;
            m_maxDot = float.MinValue;
	    }

        public virtual void InternalProcessTriangleIndex(Vector3[] triangle, int partId, int triangleIndex)
	    {
		    for (int i=0;i<3;i++)
		    {
			    float dot;
                Vector3.Dot(ref m_supportVecLocal,ref triangle[i],out dot);
			    if (dot > m_maxDot)
			    {
				    m_maxDot = dot;
				    m_supportVertexLocal = triangle[i];
			    }
		    }
	    }
    	
	    public Vector3 GetSupportVertexLocal()
	    {
		    return m_supportVertexLocal;
	    }

        public void Cleanup()
        {
        }

	    private Vector3 m_supportVertexLocal;
	    public float m_maxDot;
	    public Vector3 m_supportVecLocal;

    };

    public class CenterCallback: IInternalTriangleIndexCallback
    {
        public CenterCallback()
        {
            first = true;
            reference = new Vector3();
            sum = new Vector3();
            volume = 0f;
        }

        public virtual bool graphics()
        {
            return false;
        }

        public virtual void InternalProcessTriangleIndex(Vector3[] triangle, int partId, int triangleIndex)
        {
            if (first)
            {
                reference = triangle[0];
                first = false;
            }
            else
            {
                Vector3 a = triangle[0] - reference;
                Vector3 b = triangle[1] - reference;
                Vector3 c = triangle[2] - reference;
                float vol = Math.Abs(MathUtil.Vector3Triple(ref a,ref b,ref c));
                sum += (.25f * vol) * ((triangle[0] + triangle[1] + triangle[2] + reference));
                volume += vol;
            }
        }
      
        public Vector3 GetCenter()
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
        Vector3 reference;
        Vector3 sum;
        float volume;

    }

    public class InertiaCallback: IInternalTriangleIndexCallback
    {

        public virtual bool graphics()
        {
            return false;
        }

        public InertiaCallback(ref Vector3 center)
        {
            m_sum = new Matrix();
            m_center = center;
        }

        public virtual void InternalProcessTriangleIndex(Vector3[] triangle, int partId, int triangleIndex)
        {
            Matrix i = new Matrix();
            Vector3 a = triangle[0] - m_center;
            Vector3 b = triangle[1] - m_center;
            Vector3 c = triangle[2] - m_center;
            float volNeg = -Math.Abs(MathUtil.Vector3Triple(ref a,ref b,ref c) * (1f / 6f));
            for (int j = 0; j < 3; j++)
            {
                for (int k = 0; k <= j; k++)
                {
                    float aj = MathUtil.VectorComponent(ref a,j);
                    float ak = MathUtil.VectorComponent(ref a,k);
                    float bj = MathUtil.VectorComponent(ref b,j);
                    float bk = MathUtil.VectorComponent(ref b,k);
                    float cj = MathUtil.VectorComponent(ref c,j);
                    float ck = MathUtil.VectorComponent(ref c,k);

                    float temp = volNeg * (.1f * (aj * ak + bj * bk + cj * ck)
                    + .05f * (aj * bk + ak * bj + aj * ck + ak * cj + bj * ck + bk * cj));

                    MathUtil.MatrixComponent(ref i,j,k,temp);
                    MathUtil.MatrixComponent(ref i,k,j,temp);
                }
            }
            float i00 = -i.M11;
            float i11 = -i.M22;
            float i22 = -i.M33;
            i.M11 = i11 + i22; 
            i.M22 = i22 + i00; 
            i.M33 = i00 + i11;
            m_sum.Right += i.Right;
            m_sum.Up += i.Up;
            m_sum.Backward += i.Backward;
        }

        public Matrix GetInertia()
        {
            return m_sum;
        }

        public void Cleanup()
        {

        }

        Matrix m_sum;
        Vector3 m_center;
    }

}
