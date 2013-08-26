//#define USE_CONVEX_HULL_COMPUTER
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

using System.Diagnostics;

using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{

    public abstract class PolyhedralConvexShape : ConvexInternalShape
    {
        public override IndexedVector3 LocalGetSupportingVertexWithoutMargin(ref IndexedVector3 vec)
        {
            return IndexedVector3.Zero;
        }

        public override void Cleanup()
        {
            base.Cleanup();
        }

        ///optional method mainly used to generate multiple contact points by clipping polyhedral features (faces/edges)
        public virtual bool InitializePolyhedralFeatures()
        {
#if USE_CONVEX_HULL_COMPUTER
            if (m_polyhedron != null)
            {
                m_polyhedron = null;
            }

            m_polyhedron = new ConvexPolyhedron();

            ObjectArray<IndexedVector3> tmpVertices = new ObjectArray<IndexedVector3>();
            for (int i = 0; i < GetNumVertices(); i++)
            {
                IndexedVector3 newVertex;
                GetVertex(i, out newVertex);
                tmpVertices.Add(newVertex);
            }

            ConvexHullComputer conv = new ConvexHullComputer();
            //conv.compute(&tmpVertices[0].getX(), sizeof(IndexedVector3),tmpVertices.Count,0.0f,0.0f);
            conv.Compute(tmpVertices, 0, tmpVertices.Count, 0.0f, 0.0f);



            ObjectArray<IndexedVector3> faceNormals = new ObjectArray<IndexedVector3>();
            int numFaces = conv.faces.size();
            faceNormals.Resize(numFaces);
            ConvexHullComputer convexUtil = conv;



            m_polyhedron.m_faces.Resize(numFaces);
            int numVertices = convexUtil.vertices.Count;
            m_polyhedron.m_vertices.Resize(numVertices);
            for (int p = 0; p < numVertices; p++)
            {
                m_polyhedron.m_vertices[p] = convexUtil.vertices[p];
            }

            for (int i = 0; i < numFaces; i++)
            {
                int face = convexUtil.faces[i];
                //printf("face=%d\n",face);
                Edge firstEdge = convexUtil.edges[face];
                Edge edge = firstEdge;

                IndexedVector3[] edges = new IndexedVector3[3];
                int numEdges = 0;
                //compute face normals

                float maxCross2 = 0.0f;
                int chosenEdge = -1;

                do
                {
                    int src = edge.GetSourceVertex();
                    m_polyhedron.m_faces[i].m_indices.Add(src);
                    int targ = edge.GetTargetVertex();
                    IndexedVector3 wa = convexUtil.vertices[src];

                    IndexedVector3 wb = convexUtil.vertices[targ];
                    IndexedVector3 newEdge = wb - wa;
                    newEdge.Normalize();
                    if (numEdges < 2)
                    {
                        edges[numEdges++] = newEdge;
                    }

                    edge = edge.GetNextEdgeOfFace();
                } while (edge != firstEdge);

                float planeEq = 1e30f;


                if (numEdges == 2)
                {
                    faceNormals[i] = IndexedVector3.Cross(edges[0], edges[1]);
                    faceNormals[i].Normalize();
                    m_polyhedron.m_faces[i].m_plane[0] = -faceNormals[i].X;
                    m_polyhedron.m_faces[i].m_plane[1] = -faceNormals[i].Y;
                    m_polyhedron.m_faces[i].m_plane[2] = -faceNormals[i].Z;
                    m_polyhedron.m_faces[i].m_plane[3] = planeEq;

                }
                else
                {
                    Debug.Assert(false);//degenerate?
                    faceNormals[i] = IndexedVector3.Zero;
                }

                for (int v = 0; v < m_polyhedron.m_faces[i].m_indices.Count; v++)
                {
                    float eq = IndexedVector3.Dot(m_polyhedron.m_vertices[m_polyhedron.m_faces[i].m_indices[v]], faceNormals[i]);
                    if (planeEq > eq)
                    {
                        planeEq = eq;
                    }
                }
                m_polyhedron.m_faces[i].m_plane[3] = planeEq;
            }


            if (m_polyhedron.m_faces.Count > 0 && conv.vertices.Count > 0)
            {

                for (int f = 0; f < m_polyhedron.m_faces.Count; f++)
                {

                    IndexedVector3 planeNormal = new IndexedVector3(m_polyhedron.m_faces[f].m_plane[0], m_polyhedron.m_faces[f].m_plane[1], m_polyhedron.m_faces[f].m_plane[2]);
                    float planeEq = m_polyhedron.m_faces[f].m_plane[3];

                    IndexedVector3 supVec = LocalGetSupportingVertex(-planeNormal);

                    if (IndexedVector3.Dot(supVec, planeNormal) < planeEq)
                    {
                        m_polyhedron.m_faces[f].m_plane[0] *= -1;
                        m_polyhedron.m_faces[f].m_plane[1] *= -1;
                        m_polyhedron.m_faces[f].m_plane[2] *= -1;
                        m_polyhedron.m_faces[f].m_plane[3] *= -1;
                        int numVerts = m_polyhedron.m_faces[f].m_indices.Count;
                        for (int v = 0; v < numVerts / 2; v++)
                        {
                            int temp = m_polyhedron.m_faces[f].m_indices[v];
                            m_polyhedron.m_faces[f].m_indices[v] = m_polyhedron.m_faces[f].m_indices[numVerts - 1 - v];
                            m_polyhedron.m_faces[f].m_indices[numVerts - 1 - v] = temp;
                        }
                    }
                }
            }



            m_polyhedron.Initialize();

#endif
            return true;

        }

        public ConvexPolyhedron GetConvexPolyhedron()
        {
            return m_polyhedron;
        }

        public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(IndexedVector3[] vectors, IndexedVector4[] supportVerticesOut, int numVectors)
        {
            int i;

            IndexedVector3 vtx;
            float newDot = 0f;

            for (i = 0; i < numVectors; i++)
            {
                IndexedVector4 temp = supportVerticesOut[i];
                temp.W = -MathUtil.BT_LARGE_FLOAT;
                supportVerticesOut[i] = temp;
            }


            for (int j = 0; j < numVectors; j++)
            {

                IndexedVector3 vec = vectors[j];

                for (i = 0; i < GetNumVertices(); i++)
                {
                    GetVertex(i, out vtx);
                    newDot = IndexedVector3.Dot(vec, vtx);
                    if (newDot > supportVerticesOut[j].W)
                    {
                        supportVerticesOut[j] = new IndexedVector4(vtx, newDot);
                    }
                }
            }


        }

        public override void CalculateLocalInertia(float mass, out IndexedVector3 inertia)
        {
            //not yet, return box inertia

            float margin = GetMargin();

            IndexedMatrix ident = IndexedMatrix.Identity;

            IndexedVector3 aabbMin = IndexedVector3.Zero, aabbMax = IndexedVector3.Zero;
            GetAabb(ref ident, out aabbMin, out aabbMax);
            IndexedVector3 halfExtents = (aabbMax - aabbMin) * 0.5f;

            float lx = 2.0f * (halfExtents.X + margin);
            float ly = 2.0f * (halfExtents.Y + margin);
            float lz = 2.0f * (halfExtents.Z + margin);
            float x2 = lx * lx;
            float y2 = ly * ly;
            float z2 = lz * lz;
            float scaledmass = mass * 0.08333333f;

            inertia = scaledmass * (new IndexedVector3(y2 + z2, x2 + z2, x2 + y2));


        }


        public abstract int GetNumVertices();
        public abstract int GetNumEdges();
        public abstract void GetEdge(int i, out IndexedVector3 pa, out IndexedVector3 pb);
        public abstract void GetVertex(int i, out IndexedVector3 vtx);
        public abstract int GetNumPlanes();
        public abstract void GetPlane(out IndexedVector3 planeNormal, out IndexedVector3 planeSupport, int i);
        //	virtual int getIndex(int i) const = 0 ; 

        public abstract bool IsInside(ref IndexedVector3 pt, float tolerance);

        protected ConvexPolyhedron m_polyhedron;

    }

    public abstract class PolyhedralConvexAabbCachingShape : PolyhedralConvexShape
    {
        public PolyhedralConvexAabbCachingShape()
        {
            m_localAabbMin = new IndexedVector3(1);
            m_localAabbMax = new IndexedVector3(-1);
            m_isLocalAabbValid = false;
            //m_optionalHull = null;
        }

        protected void SetCachedLocalAabb(ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax)
        {
            m_isLocalAabbValid = true;
            m_localAabbMin = aabbMin;
            m_localAabbMax = aabbMax;
        }

        protected void GetCachedLocalAabb(out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
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



        public override void GetAabb(ref IndexedMatrix trans, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
        {
            GetNonvirtualAabb(ref trans, out aabbMin, out aabbMax, GetMargin());
        }

        public override void SetLocalScaling(ref IndexedVector3 scaling)
        {
            base.SetLocalScaling(ref scaling);
            RecalcLocalAabb();
        }

        public void RecalcLocalAabb()
        {
            m_isLocalAabbValid = true;

#if TRUE
            IndexedVector3[] _directions = new IndexedVector3[6];
            _directions[0] = new IndexedVector3(1, 0, 0);
            _directions[1] = new IndexedVector3(0, 1, 0); 
            _directions[2] = new IndexedVector3(0, 0, 1);
            _directions[3] = new IndexedVector3(-1, 0, 0);
            _directions[4] = new IndexedVector3(0, -1, 0);
            _directions[5] = new IndexedVector3(0, 0, -1);

            IndexedVector4[] _supporting = new IndexedVector4[6];

            BatchedUnitVectorGetSupportingVertexWithoutMargin(_directions, _supporting, 6);

            for (int i = 0; i < 3; ++i)
            {
                IndexedVector3 temp = new IndexedVector3(_supporting[i]);
                m_localAabbMax[i] =  temp[i] + m_collisionMargin;
                temp = new IndexedVector3(_supporting[i+3]);
                m_localAabbMin[i] = temp[i] - m_collisionMargin;
            }
            int ibreak = 0;
#else

            for (int i=0;i<3;i++)
            {
                IndexedVector3 vec = new IndexedVector3();
                MathUtil.vectorComponent(ref vec,i,1f);
                IndexedVector3 tmp = localGetSupportingVertex(ref vec);
                MathUtil.vectorComponent(ref m_localAabbMax,i,(MathUtil.vectorComponent(ref tmp,i) + m_collisionMargin));

                MathUtil.vectorComponent(ref vec,i,-1f);
                IndexedVector3 tmp = localGetSupportingVertex(ref vec);
                MathUtil.vectorComponent(ref m_localAabbMin,i,(MathUtil.vectorComponent(ref tmp,i) - m_collisionMargin));

            }
#endif

        }
        protected IndexedVector3 m_localAabbMin;
        protected IndexedVector3 m_localAabbMax;
        protected bool m_isLocalAabbValid;

    }
}
