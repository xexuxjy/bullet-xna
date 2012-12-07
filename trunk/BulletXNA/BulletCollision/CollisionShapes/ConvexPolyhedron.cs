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
#define TEST_INTERNAL_OBJECTS
using System;
using System.Collections.Generic;
using System.Diagnostics;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{


    public class Face
    {
        public ObjectArray<int> m_indices = new ObjectArray<int>();
        //public ObjectArray<int>	m_connectedFaces = new ObjectArray<int>();
        public float[] m_plane = new float[4];
    }


    public class ConvexPolyhedron
    {
        public ConvexPolyhedron()
        {
        }
        public virtual void Cleanup()
        {

        }


        public void Initialize()
        {
            Dictionary<InternalVertexPair, InternalEdge> edges = new Dictionary<InternalVertexPair, InternalEdge>();

            float TotalArea = 0.0f;

            m_localCenter = IndexedVector3.Zero;
            for (int i = 0; i < m_faces.Count; i++)
            {
                int numVertices = m_faces[i].m_indices.Count;
                int NbTris = numVertices;
                for (int j = 0; j < NbTris; j++)
                {
                    int k = (j + 1) % numVertices;
                    InternalVertexPair vp = new InternalVertexPair(m_faces[i].m_indices[j], m_faces[i].m_indices[k]);
                    InternalEdge edptr = edges[vp];
                    IndexedVector3 edge = m_vertices[vp.m_v1] - m_vertices[vp.m_v0];
                    edge.Normalize();

                    bool found = false;

                    for (int p = 0; p < m_uniqueEdges.Count; p++)
                    {

                        if (MathUtil.IsAlmostZero(m_uniqueEdges[p] - edge) ||
                            MathUtil.IsAlmostZero(m_uniqueEdges[p] + edge))
                        {
                            found = true;
                            break;
                        }
                    }

                    if (!found)
                    {
                        m_uniqueEdges.Add(edge);
                    }

                    if (edptr != null)
                    {
                        Debug.Assert(edptr.m_face0 >= 0);
                        Debug.Assert(edptr.m_face1 < 0);
                        edptr.m_face1 = (int)i;
                    }
                    else
                    {
                        InternalEdge ed = new InternalEdge();
                        ed.m_face0 = i;
                        edges[vp] = ed;
                    }
                }
            }

#if USE_CONNECTED_FACES
            for (int i = 0; i < m_faces.Count; i++)
            {
                int numVertices = m_faces[i].m_indices.Count;
                m_faces[i].m_connectedFaces.Resize(numVertices);

                for (int j = 0; j < numVertices; j++)
                {
                    int k = (j + 1) % numVertices;
                    InternalVertexPair vp = new InternalVertexPair(m_faces[i].m_indices[j], m_faces[i].m_indices[k]);
                    InternalEdge edptr = edges[vp];
                    Debug.Assert(edptr != null);
                    Debug.Assert(edptr.m_face0 >= 0);
                    Debug.Assert(edptr.m_face1 >= 0);

                    int connectedFace = (edptr.m_face0 == i) ? edptr.m_face1 : edptr.m_face0;
                    m_faces[i].m_connectedFaces[j] = connectedFace;
                }
            }
#endif
            for (int i = 0; i < m_faces.Count; i++)
            {
                int numVertices = m_faces[i].m_indices.Count;
                int NbTris = numVertices - 2;

                IndexedVector3 p0 = m_vertices[m_faces[i].m_indices[0]];
                for (int j = 1; j <= NbTris; j++)
                {
                    int k = (j + 1) % numVertices;
                    IndexedVector3 p1 = m_vertices[m_faces[i].m_indices[j]];
                    IndexedVector3 p2 = m_vertices[m_faces[i].m_indices[k]];
                    float Area = IndexedVector3.Cross((p0 - p1), (p0 - p2)).Length() * 0.5f;
                    IndexedVector3 Center = (p0 + p1 + p2) / 3.0f;
                    m_localCenter += Area * Center;
                    TotalArea += Area;
                }
            }
            m_localCenter /= TotalArea;


#if TEST_INTERNAL_OBJECTS
	if(true)
	{
		m_radius = float.MaxValue;
		for(int i=0;i<m_faces.Count;i++)
		{
			IndexedVector3 Normal = new IndexedVector3(m_faces[i].m_plane[0], m_faces[i].m_plane[1], m_faces[i].m_plane[2]);
			float dist = Math.Abs(m_localCenter.Dot(Normal) + m_faces[i].m_plane[3]);
			if(dist<m_radius)
            {
				m_radius = dist;
            }
		}

	
		float MinX = float.MaxValue;
		float MinY = float.MaxValue;
		float MinZ = float.MaxValue;
		float MaxX = float.MinValue;
		float MaxY = float.MinValue;
		float MaxZ = float.MinValue;
		for(int i=0; i<m_vertices.Count; i++)
		{
            IndexedVector3 pt = m_vertices[i];
			if(pt.X<MinX)	MinX = pt.X;
			if(pt.X>MaxX)	MaxX = pt.X;
			if(pt.Y<MinY)	MinY = pt.Y;
			if(pt.Y>MaxY)	MaxY = pt.Y;
			if(pt.Z<MinZ)	MinZ = pt.Z;
			if(pt.Z>MaxZ)	MaxZ = pt.Z;
		}
		mC = new IndexedVector3(MaxX+MinX, MaxY+MinY, MaxZ+MinZ);
		mE = new IndexedVector3(MaxX-MinX, MaxY-MinY, MaxZ-MinZ);



//		const float r = m_radius / sqrtf(2.0f);
		float r = m_radius / (float)Math.Sqrt(3.0f);
		int LargestExtent = mE.MaxAxis();
		float Step = (mE[LargestExtent]*0.5f - r)/1024.0f;
		m_extents.X = m_extents.Y = m_extents.Z = r;
		m_extents[LargestExtent] = mE[LargestExtent]*0.5f;
		bool FoundBox = false;
		for(int j=0;j<1024;j++)
		{
			if(TestContainment())
			{
				FoundBox = true;
				break;
			}

			m_extents[LargestExtent] -= Step;
		}
		if(!FoundBox)
		{
			m_extents.X = m_extents.Y = m_extents.Z = r;
		}
		else
		{
			// Refine the box
			float innerStep = (m_radius - r)/1024.0f;
			int e0 = (1<<LargestExtent) & 3;
			int e1 = (1<<e0) & 3;

			for(int j=0;j<1024;j++)
			{
				float Saved0 = m_extents[e0];
				float Saved1 = m_extents[e1];
                m_extents[e0] += innerStep;
                m_extents[e1] += innerStep;

				if(!TestContainment())
				{
					m_extents[e0] = Saved0;
					m_extents[e1] = Saved1;
					break;
				}
			}
		}
	}
#endif

        }
#if TEST_INTERNAL_OBJECTS
        public bool TestContainment()
        {
	        for(int p=0;p<8;p++)
	        {
		        IndexedVector3 LocalPt = IndexedVector3.Zero;
		        if(p==0)		LocalPt = m_localCenter + new IndexedVector3(m_extents.X, m_extents.Y, m_extents.Z);
		        else if(p==1)	LocalPt = m_localCenter + new IndexedVector3(m_extents.X, m_extents.Y, -m_extents.Z);
		        else if(p==2)	LocalPt = m_localCenter + new IndexedVector3(m_extents.X, -m_extents.Y, m_extents.Z);
		        else if(p==3)	LocalPt = m_localCenter + new IndexedVector3(m_extents.X, -m_extents.Y, -m_extents.Z);
		        else if(p==4)	LocalPt = m_localCenter + new IndexedVector3(-m_extents.X, m_extents.Y, m_extents.Z);
		        else if(p==5)	LocalPt = m_localCenter + new IndexedVector3(-m_extents.X, m_extents.Y, -m_extents.Z);
		        else if(p==6)	LocalPt = m_localCenter + new IndexedVector3(-m_extents.X, -m_extents.Y, m_extents.Z);
		        else if(p==7)	LocalPt = m_localCenter + new IndexedVector3(-m_extents.X, -m_extents.Y, -m_extents.Z);

		        for(int i=0;i<m_faces.Count;i++)
		        {
			        IndexedVector3 Normal = new IndexedVector3(m_faces[i].m_plane[0], m_faces[i].m_plane[1], m_faces[i].m_plane[2]);
			        float d = LocalPt.Dot(Normal) + m_faces[i].m_plane[3];
                    if (d > 0.0f)
                    {
                        return false;
                    }
                }
	        }
	        return true;
        }
#endif

        public void Project(ref IndexedMatrix trans, ref IndexedVector3 dir, out float min, out float max)
        {
            min = float.MaxValue;
            max = float.MinValue;
            int numVerts = m_vertices.Count;
            for (int i = 0; i < numVerts; i++)
            {
                IndexedVector3 pt = trans * m_vertices[i];
                float dp = IndexedVector3.Dot(pt, dir);
                if (dp < min) min = dp;
                if (dp > max) max = dp;
            }
            if (min > max)
            {
                float tmp = min;
                min = max;
                max = tmp;
            }
        }

        public ObjectArray<IndexedVector3> m_vertices = new ObjectArray<IndexedVector3>();
        public ObjectArray<Face> m_faces = new ObjectArray<Face>();
        public ObjectArray<IndexedVector3> m_uniqueEdges = new ObjectArray<IndexedVector3>();
        public IndexedVector3 m_localCenter;
        public IndexedVector3 m_extents;
        public float m_radius;
        public IndexedVector3 mC;
        public IndexedVector3 mE;
    }

    public class InternalVertexPair
    {
        public InternalVertexPair(int v0, int v1)
        {
            m_v0 = v1 > v0 ? v1 : v0;
            m_v1 = v1 > v0 ? v0 : v1;

        }
        public int GetHash()
        {
            return m_v0 + (m_v1 << 16);
        }
        public bool Equals(ref InternalVertexPair other)
        {
            return m_v0 == other.m_v0 && m_v1 == other.m_v1;
        }
        public int m_v0;
        public int m_v1;
    }

    public class InternalEdge
    {
        public InternalEdge()
        {
            m_face0 = -1;
            m_face1 = -1;
        }
        public int m_face0;
        public int m_face1;
    }


}
