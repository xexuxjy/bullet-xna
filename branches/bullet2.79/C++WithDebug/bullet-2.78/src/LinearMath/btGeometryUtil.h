/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef BT_GEOMETRY_UTIL_H
#define BT_GEOMETRY_UTIL_H

#include "btVector3.h"
#include "btTransform.h"
#include "btMatrix3x3.h"
#include "btQuaternion.h"
#include "btAlignedObjectArray.h"

#include <stdio.h>
///The btGeometryUtil helper class provides a few methods to convert between plane equations and vertices.
class btGeometryUtil
{
	public:
	
		static void PrintQuaternion(FILE* file,const btQuaternion& q);
		static void PrintMatrix(FILE* file,const btTransform& m);
		static void PrintMatrix(FILE* file,const char* name,const btTransform& m);
		static void PrintMatrix(FILE* file,const btMatrix3x3& m);
		static void PrintMatrix(FILE* file,const char* name,const btMatrix3x3& m);

		static void PrintVector(FILE* file ,const btVector3& v);

		static void PrintVector(FILE* file ,const char* name,const btVector3& v);
		static void	getPlaneEquationsFromVertices(btAlignedObjectArray<btVector3>& vertices, btAlignedObjectArray<btVector3>& planeEquationsOut );

		static void	getVerticesFromPlaneEquations(const btAlignedObjectArray<btVector3>& planeEquations , btAlignedObjectArray<btVector3>& verticesOut );
	
		static bool	isInside(const btAlignedObjectArray<btVector3>& vertices, const btVector3& planeNormal, btScalar	margin);
		
		static bool	isPointInsidePlanes(const btAlignedObjectArray<btVector3>& planeEquations, const btVector3& point, btScalar	margin);

		static bool	areVerticesBehindPlane(const btVector3& planeNormal, const btAlignedObjectArray<btVector3>& vertices, btScalar	margin);

};


#endif //BT_GEOMETRY_UTIL_H

