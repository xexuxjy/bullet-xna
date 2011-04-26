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
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletCollision.CollisionShapes
{
    public class IndexedMesh
    {
        public int m_numTriangles;
        public ObjectArray<int> m_triangleIndexBase = new ObjectArray<int>();
        public int m_triangleIndexStride;
        public int m_numVertices;
        public ObjectArray<Vector3> m_vertexBase = new ObjectArray<Vector3>();
        public int m_vertexStride;
        //// The index type is set when adding an indexed mesh to the
        //// btTriangleIndexVertexArray, do not set it manually
		//public PHY_ScalarType	m_indexType = PHY_ScalarType.PHY_INTEGER;
		public PHY_ScalarType m_indexType;
		public PHY_ScalarType m_vertexType = PHY_ScalarType.PHY_FLOAT;
    }


    public class TriangleIndexVertexArray : StridingMeshInterface
    {
	    public TriangleIndexVertexArray() 
	    {
            m_hasAabb = false;
	    }

        //just to be backwards compatible
	    public TriangleIndexVertexArray(int numTriangles,ObjectArray<int> triangleIndexBase,int triangleIndexStride,int numVertices,ObjectArray<Vector3> vertexBase,int vertexStride)
        {
            IndexedMesh indexedMesh = new IndexedMesh();
            indexedMesh.m_numTriangles = numTriangles;
            indexedMesh.m_triangleIndexBase = triangleIndexBase;
	        indexedMesh.m_triangleIndexStride = triangleIndexStride;
	        indexedMesh.m_numVertices = numVertices;
	        indexedMesh.m_vertexBase = vertexBase;
	        indexedMesh.m_vertexStride = vertexStride;

	        AddIndexedMesh(indexedMesh,PHY_ScalarType.PHY_INTEGER);
        }

        public override void Cleanup()
        {
        }

	
	    public void	AddIndexedMesh(IndexedMesh mesh, PHY_ScalarType indexType)
	    {
		    m_indexedMeshes.Add(mesh);
		    m_indexedMeshes[m_indexedMeshes.Count-1].m_indexType = indexType;
	    }


        public override void GetLockedVertexIndexBase(out Object vertexbase, out int numverts, out PHY_ScalarType type, out int vertexStride, out Object indexbase, out int indexstride, out int numfaces, out PHY_ScalarType indicestype, int subpart)
        {
        	Debug.Assert(subpart< GetNumSubParts() );
	        IndexedMesh mesh = m_indexedMeshes[subpart];

	        numverts = mesh.m_numVertices;
	        vertexbase = mesh.m_vertexBase;
			type = mesh.m_vertexType;
	        vertexStride = mesh.m_vertexStride;

	        numfaces = mesh.m_numTriangles;

	        indexbase = mesh.m_triangleIndexBase;
	        indexstride = mesh.m_triangleIndexStride;
	        indicestype = mesh.m_indexType;
        }

        public override void getLockedReadOnlyVertexIndexBase(out Object vertexbase, out int numverts, out PHY_ScalarType type, out int vertexStride, out Object indexbase, out int indexstride, out int numfaces, out PHY_ScalarType indicestype, int subpart)
        {
            Debug.Assert(subpart< GetNumSubParts() );

	        IndexedMesh mesh = m_indexedMeshes[subpart];

	        numverts = mesh.m_numVertices;
	        vertexbase = mesh.m_vertexBase;
			type = mesh.m_vertexType;

	        vertexStride = mesh.m_vertexStride;

	        numfaces = mesh.m_numTriangles;

	        indexbase = mesh.m_triangleIndexBase;
	        indexstride = mesh.m_triangleIndexStride;
	        indicestype = mesh.m_indexType;

        }
	    /// unLockVertexBase finishes the access to a subpart of the triangle mesh
	    /// make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished
        public override void UnLockVertexBase(int subpart) 
        {
        
        }

        public override void UnLockReadOnlyVertexBase(int subpart) 
        {
            
        }

	/// getNumSubParts returns the number of seperate subparts
	/// each subpart has a continuous array of vertices and indices
        public override int GetNumSubParts() 
        { 
		    return m_indexedMeshes.Count;
	    }

	    public ObjectArray<IndexedMesh> getIndexedMeshArray()
	    {
		    return m_indexedMeshes;
	    }


        public override void PreallocateVertices(int numverts)
        {
            //(void) numverts;
        }
        public override void PreallocateIndices(int numindices)
        {
            //(void) numindices;
        }

        public override bool HasPremadeAabb()
        {
            return m_hasAabb;
        }

        public override void SetPremadeAabb(ref Vector3 aabbMin, ref Vector3 aabbMax)
        {
	        m_aabbMin = aabbMin;
	        m_aabbMax = aabbMax;
	        m_hasAabb = true; // this is intentionally an int see notes in header

        }
        public override void GetPremadeAabb(ref Vector3 aabbMin, ref Vector3 aabbMax)
        {
            aabbMin = m_aabbMin;
            aabbMax = m_aabbMax;
        }
	    protected ObjectArray<IndexedMesh>	m_indexedMeshes = new ObjectArray<IndexedMesh>();
    	protected bool m_hasAabb;
	    protected Vector3 m_aabbMin;
	    protected Vector3 m_aabbMax;
    }
}
