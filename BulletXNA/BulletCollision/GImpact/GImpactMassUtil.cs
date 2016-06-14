/*
 * 
 * C# / XNA  port of Bullet (c) 2011 Mark Neale <xexuxjy@hotmail.com>
 *
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
    public static class GImpactMassUtil
    {

        public static IndexedVector3 GimGetPointInertia(ref IndexedVector3 point, float mass)
        {
            float x2 = point.X * point.X;
            float y2 = point.Y * point.Y;
            float z2 = point.Z * point.Z;
            return new IndexedVector3(mass * (y2 + z2), mass * (x2 + z2), mass * (x2 + y2));
        }

        public static IndexedVector3 GimInertiaAddTransformed(ref IndexedVector3 source_inertia, ref IndexedVector3 added_inertia, ref IndexedMatrix transform)
        {
            //    btMatrix3x3  rotatedTensor = transform.getBasis().scaled(added_inertia) * transform.getBasis().transpose();

            //    float x2 = transform.getOrigin()[0];
            //    x2*= x2;
            //    float y2 = transform.getOrigin()[1];
            //    y2*= y2;
            //    float z2 = transform.getOrigin()[2];
            //    z2*= z2;

            //    float ix = rotatedTensor[0][0]*(y2+z2);
            //    float iy = rotatedTensor[1][1]*(x2+z2);
            //    float iz = rotatedTensor[2][2]*(x2+y2);

            //    return btVector3(source_inertia[0]+ix,source_inertia[1]+iy,source_inertia[2] + iz);
            return IndexedVector3.Zero;
        }
    }
}
