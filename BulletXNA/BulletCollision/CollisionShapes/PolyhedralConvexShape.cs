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

#define TRUE

using System.Collections.Generic;
using System.Diagnostics;
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletCollision.CollisionShapes
{
    
    public abstract class PolyhedralConvexShape : ConvexInternalShape
    {
	    public override Vector3	LocalGetSupportingVertexWithoutMargin(ref Vector3 vec)
        {
            return Vector3.Zero;
        }

        public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IList<Vector3> vectors, IList<Vector4> supportVerticesOut, int numVectors)
        {
            	int i;

	            Vector3 vtx;
	            float newDot = 0f;

                for (i = 0; i < numVectors; i++)
                {
                    Vector4 temp = supportVerticesOut[i];
					temp.W = -MathUtil.BT_LARGE_FLOAT;
                    supportVerticesOut[i] = temp;
                }

				
	            for (int j=0;j<numVectors;j++)
	            {
            	
		            Vector3 vec = vectors[j];

		            for (i=0;i<GetNumVertices();i++)
		            {
                        GetVertex(i, out vtx);
			            newDot = Vector3.Dot(vec,vtx);
			            if (newDot > supportVerticesOut[j].W)
			            {
				            supportVerticesOut[j] = new Vector4(vtx,newDot);
			            }
		            }
	            }


        }

        public override void CalculateLocalInertia(float mass, out Vector3 inertia)
        {
	        //not yet, return box inertia

	        float margin = GetMargin();

	        Matrix ident = Matrix.Identity;

	        Vector3 aabbMin = Vector3.Zero,aabbMax = Vector3.Zero;
	        GetAabb(ref ident,out aabbMin,out aabbMax);
	        Vector3 halfExtents = (aabbMax-aabbMin)*0.5f;

	        float lx=2.0f*(halfExtents.X+margin);
	        float ly=2.0f*(halfExtents.Y+margin);
	        float lz=2.0f*(halfExtents.Z+margin);
	        float x2 = lx*lx;
	        float y2 = ly*ly;
	        float z2 = lz*lz;
	        float scaledmass = mass * 0.08333333f;

	        inertia = scaledmass * (new Vector3(y2+z2,x2+z2,x2+y2));


        }
    	
    	
	    public abstract int	GetNumVertices();
	    public abstract int GetNumEdges();
        public abstract void GetEdge(int i, out Vector3 pa, out Vector3 pb);
	    public abstract void GetVertex(int i, out Vector3 vtx);
	    public abstract int	GetNumPlanes();
        public abstract void GetPlane(out Vector3 planeNormal, out Vector3 planeSupport, int i);
    //	virtual int getIndex(int i) const = 0 ; 

	    public abstract bool IsInside(ref Vector3 pt,float tolerance);
    }

    public abstract class PolyhedralConvexAabbCachingShape : PolyhedralConvexShape
    {
        public PolyhedralConvexAabbCachingShape()
        {
            m_localAabbMin = new Vector3(1,1,1);
            m_localAabbMax = new Vector3(-1,-1,-1);
            m_isLocalAabbValid = false;
            //m_optionalHull = null;
        }

        protected void SetCachedLocalAabb(ref Vector3 aabbMin, ref Vector3 aabbMax)
	    {
		    m_isLocalAabbValid = true;
		    m_localAabbMin = aabbMin;
		    m_localAabbMax = aabbMax;
	    }

	    protected void GetCachedLocalAabb(out Vector3 aabbMin, out Vector3 aabbMax)
	    {
		    Debug.Assert(m_isLocalAabbValid);
		    aabbMin = m_localAabbMin;
		    aabbMax = m_localAabbMax;
	    }

        public void GetNonvirtualAabb(ref Matrix trans, out Vector3 aabbMin, out Vector3 aabbMax, float margin)
        {
            //lazy evaluation of local aabb
            Debug.Assert(m_isLocalAabbValid);
            MathUtil.TransformAabb(ref m_localAabbMin, ref m_localAabbMax, margin, ref trans, out aabbMin, out aabbMax);
        }

	

        public override void GetAabb(ref Matrix trans, out Vector3 aabbMin, out Vector3 aabbMax)
        {
            GetNonvirtualAabb(ref trans, out aabbMin, out aabbMax, GetMargin());
        }

        public override void SetLocalScaling(ref Vector3 scaling)
        {
	        base.SetLocalScaling(ref scaling);
	        RecalcLocalAabb();
        }

        public void RecalcLocalAabb()
        {
	        m_isLocalAabbValid = true;
        	
	        #if TRUE
	        IList<Vector3> _directions = new List<Vector3>();
            _directions.Add(Vector3.Right);
            _directions.Add(Vector3.Up);
            _directions.Add(Vector3.Backward);
            _directions.Add(Vector3.Left);
            _directions.Add(Vector3.Down);
            _directions.Add(Vector3.Forward);
        	
	        IList<Vector4> _supporting = new List<Vector4>();
            _supporting.Add(new Vector4());
            _supporting.Add(new Vector4());
            _supporting.Add(new Vector4());
            _supporting.Add(new Vector4());
            _supporting.Add(new Vector4());
            _supporting.Add(new Vector4());
        	
	        BatchedUnitVectorGetSupportingVertexWithoutMargin(_directions, _supporting, 6);
        	
	        for ( int i = 0; i < 3; ++i )
	        {
                Vector4 temp = _supporting[i];
                MathUtil.VectorComponent(ref m_localAabbMax,i,(MathUtil.VectorComponent(ref temp,i) + m_collisionMargin));
                temp = _supporting[i+3];
                MathUtil.VectorComponent(ref m_localAabbMin,i,(MathUtil.VectorComponent(ref temp,i) - m_collisionMargin));
	        }
			int ibreak = 0;  	
	        #else

	        for (int i=0;i<3;i++)
	        {
		        Vector3 vec = new Vector3();
		        MathUtil.vectorComponent(ref vec,i,1f);
		        Vector3 tmp = localGetSupportingVertex(ref vec);
                MathUtil.vectorComponent(ref m_localAabbMax,i,(MathUtil.vectorComponent(ref tmp,i) + m_collisionMargin));

                MathUtil.vectorComponent(ref vec,i,-1f);
		        Vector3 tmp = localGetSupportingVertex(ref vec);
                MathUtil.vectorComponent(ref m_localAabbMin,i,(MathUtil.vectorComponent(ref tmp,i) - m_collisionMargin));

	        }
#endif

        }
        protected Vector3 m_localAabbMin;
        protected Vector3 m_localAabbMax;
        protected bool m_isLocalAabbValid;

    }
}
