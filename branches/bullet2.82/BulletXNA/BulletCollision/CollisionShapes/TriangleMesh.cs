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

using BulletXNA.LinearMath;


namespace BulletXNA.BulletCollision
{
    public class TriangleMesh : TriangleIndexVertexArray
    {
        public TriangleMesh()
            : this(true, true)
        {
        }

        public TriangleMesh(bool use32bitIndices, bool use4componentVertices)
        {
            m_use32bitIndices = use32bitIndices;
            m_use4componentVertices = use4componentVertices;

            IndexedMesh meshIndex = new IndexedMesh();
            m_indexedMeshes.Add(meshIndex);

            if (m_use32bitIndices)
            {
                m_32bitIndices = new ObjectArray<int>();
                m_indexedMeshes[0].m_numTriangles = m_32bitIndices.Count / 3;
                m_indexedMeshes[0].m_indexType = PHY_ScalarType.PHY_INTEGER;
                m_indexedMeshes[0].m_triangleIndexStride = 3;
            }
            else
            {
                m_16bitIndices = new ObjectArray<short>();
                m_indexedMeshes[0].m_numTriangles = m_16bitIndices.Count / 3;
                m_indexedMeshes[0].m_triangleIndexBase = null;
                m_indexedMeshes[0].m_indexType = PHY_ScalarType.PHY_SHORT;
                m_indexedMeshes[0].m_triangleIndexStride = 3;
            }

            if (m_use4componentVertices)
            {
                m_4componentVertices = new ObjectArray<IndexedVector3>();
                m_indexedMeshes[0].m_numVertices = m_4componentVertices.Count;
                m_indexedMeshes[0].m_vertexStride = 1;
                m_indexedMeshes[0].m_vertexBase = m_4componentVertices;
            }
            else
            {
                m_3componentVertices = new ObjectArray<float>();
                m_indexedMeshes[0].m_numVertices = m_3componentVertices.Count / 3;
                m_indexedMeshes[0].m_vertexStride = 3;
                m_indexedMeshes[0].m_vertexBase = m_3componentVertices;
            }

        }

#if XNA

        ///findOrAddVertex is an internal method, use addTriangle instead
        public int FindOrAddVertex(ref Microsoft.Xna.Framework.Vector3 vertex, bool removeDuplicateVertices)
        {
            IndexedVector3 iv3 = new IndexedVector3(vertex);
            return FindOrAddVertex(ref iv3,removeDuplicateVertices);
        }
#endif
        public int FindOrAddVertex(ref IndexedVector3 vertex, bool removeDuplicateVertices)
        {
            //return index of new/existing vertex
            ///@todo: could use acceleration structure for this
            if (m_use4componentVertices)
            {
                if (removeDuplicateVertices)
                {
                    IndexedVector3[] rawVertices = m_4componentVertices.GetRawArray();
                    for (int i = 0; i < m_4componentVertices.Count; i++)
                    {
                        if ((rawVertices[i] - vertex).LengthSquared() <= m_weldingThreshold)
                        {
                            return i;
                        }
                    }
                }
                m_indexedMeshes[0].m_numVertices++;
                m_4componentVertices.Add(vertex);
                (m_indexedMeshes[0].m_vertexBase as ObjectArray<IndexedVector3>).Add(vertex);
                return m_4componentVertices.Count - 1;
            }
            else
            {
                if (removeDuplicateVertices)
                {
                    float[] rawVertices = m_3componentVertices.GetRawArray();
                    for (int i = 0; i < m_3componentVertices.Count; i += 3)
                    {
                        IndexedVector3 vtx = new IndexedVector3(rawVertices[i], rawVertices[i + 1], rawVertices[i + 2]);
                        if ((vtx - vertex).LengthSquared() <= m_weldingThreshold)
                        {
                            return i / 3;
                        }
                    }
                }
                m_3componentVertices.Add(vertex.X);
                m_3componentVertices.Add(vertex.Y);
                m_3componentVertices.Add(vertex.Z);
                m_indexedMeshes[0].m_numVertices++;
                (m_indexedMeshes[0].m_vertexBase as ObjectArray<float>).Add(vertex.X);
                (m_indexedMeshes[0].m_vertexBase as ObjectArray<float>).Add(vertex.Y);
                (m_indexedMeshes[0].m_vertexBase as ObjectArray<float>).Add(vertex.Z);
                return (m_3componentVertices.Count / 3) - 1;
            }

        }

        ///addIndex is an internal method, use addTriangle instead
        public void AddIndex(int index)
        {
            if (m_use32bitIndices)
            {
                m_32bitIndices.Add(index);
                m_indexedMeshes[0].m_triangleIndexBase = m_32bitIndices;
            }
            else
            {
                m_16bitIndices.Add((short)index);
                // not really supported yet.
                m_indexedMeshes[0].m_triangleIndexBase = m_16bitIndices;
            }
        }

        public bool GetUse32bitIndices()
        {
            return m_use32bitIndices;
        }

        public bool GetUse4componentVertices()
        {
            return m_use4componentVertices;
        }

        ///By default addTriangle won't search for duplicate vertices, because the search is very slow for large triangle meshes.
        ///In general it is better to directly use btTriangleIndexVertexArray instead.
#if XNA

        public void AddTriangle(Microsoft.Xna.Framework.Vector3 vertex0, Microsoft.Xna.Framework.Vector3 vertex1, Microsoft.Xna.Framework.Vector3 vertex2)
        {
            AddTriangle(ref vertex0, ref vertex1, ref vertex2);
        }

        public void AddTriangle(ref Microsoft.Xna.Framework.Vector3 vertex0, ref Microsoft.Xna.Framework.Vector3 vertex1, ref Microsoft.Xna.Framework.Vector3 vertex2)
        {
            AddTriangle(ref vertex0, ref vertex1, ref vertex2, false);
        }
        public void AddTriangle(ref Microsoft.Xna.Framework.Vector3 vertex0, ref Microsoft.Xna.Framework.Vector3 vertex1, ref Microsoft.Xna.Framework.Vector3 vertex2, bool removeDuplicateVertices)
        {
            m_indexedMeshes[0].m_numTriangles++;
            AddIndex(FindOrAddVertex(ref vertex0, removeDuplicateVertices));
            AddIndex(FindOrAddVertex(ref vertex1, removeDuplicateVertices));
            AddIndex(FindOrAddVertex(ref vertex2, removeDuplicateVertices));
        }
#endif
        public int GetNumTriangles()
        {
            if (m_use32bitIndices)
            {
                return m_32bitIndices.Count / 3;
            }
            return m_16bitIndices.Count / 3;
        }

        public override void PreallocateVertices(int numverts)
        {
            //(void) numverts;
        }
        public override void PreallocateIndices(int numindices)
        {
            //(void) numindices;
        }

        private ObjectArray<IndexedVector3> m_4componentVertices;
        private ObjectArray<float> m_3componentVertices;
        private ObjectArray<int> m_32bitIndices;
        private ObjectArray<short> m_16bitIndices;
        private bool m_use32bitIndices;
        private bool m_use4componentVertices;

        public float m_weldingThreshold;

    }
}