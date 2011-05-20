/*
Copyright (c) 2011 Ole Kniemeyer, MAXON, www.maxon.net

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace BulletXNA.LinearMath
{
	public static class ConvexHullComputer
	{
/// Convex hull implementation based on Preparata and Hong
/// See http://code.google.com/p/bullet/issues/detail?id=275
/// Ole Kniemeyer, MAXON Computer GmbH
	
	private float Compute(const void* coords, bool doubleCoords, int stride, int count, float shrink, float shrinkClamp);



		// Vertices of the output hull
		ObjectArray<Vector3> vertices = new ObjectArray<Vector3>();

		// Edges of the output hull
		ObjectArray<Edge> edges = new ObjectArray<Edge>;

		// Faces of the convex hull. Each entry is an index into the "edges" array pointing to an edge of the face. Faces are planar n-gons
		ObjectArray<int> faces = new ObjectArray<int>();

		/*
		Compute convex hull of "count" vertices stored in "coords". "stride" is the difference in bytes
		between the addresses of consecutive vertices. If "shrink" is positive, the convex hull is shrunken
		by that amount (each face is moved by "shrink" length units towards the center along its normal).
		If "shrinkClamp" is positive, "shrink" is clamped to not exceed "shrinkClamp * innerRadius", where "innerRadius"
		is the minimum distance of a face to the center of the convex hull.

		The returned value is the amount by which the hull has been shrunken. If it is negative, the amount was so large
		that the resulting convex hull is empty.

		The output convex hull can be found in the member variables "vertices", "edges", "faces".
		*/
		float compute(float[] coords, int stride, int count, float shrink, float shrinkClamp)
		{
			return compute(coords, false, stride, count, shrink, shrinkClamp);
		}


}




	}


		public class Edge
		{
			private int next;
			private 	int reverse;
				private int targetVertex;


				public int getSourceVertex() 
				{
					return (this + reverse).targetVertex;
				}

				public int getTargetVertex() 
				{
					return targetVertex;
				}

				public Edge getNextEdgeOfVertex() // counter-clockwise list of all edges of a vertex
				{
					return this + next;
				}

				public Edge getNextEdgeOfFace() // clockwise list of all edges of a face
				{
					return (this + reverse).GetNextEdgeOfVertex();
				}

				public Edge getReverseEdge()
				{
					return this + reverse;
				}
		}


}
