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

/*! \file btQuantization.h
*\author Francisco Len Nßjera

*/
/*
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
    public class GImpactQuantization
    {

        public static void CalcQuantizationParameters(
        out Vector3 outMinBound,
        out Vector3 outMaxBound,
        out Vector3 bvhQuantization,
        ref Vector3 srcMinBound, ref Vector3 srcMaxBound,
        float quantizationMargin)
        {
            //enlarge the AABB to avoid division by zero when initializing the quantization values
            Vector3 clampValue = new Vector3(quantizationMargin);
            outMinBound = srcMinBound - clampValue;
            outMaxBound = srcMaxBound + clampValue;
            Vector3 aabbSize = outMaxBound - outMinBound;
            bvhQuantization = new Vector3(65535.0f) / aabbSize;
        }


        public static void QuantizeClamp(out UShortVector3 output,
            ref Vector3 point,
            ref Vector3 min_bound,
            ref Vector3 max_bound,
            ref Vector3 bvhQuantization)
        {

            Vector3 clampedPoint = point;
            MathUtil.VectorMax(ref min_bound, ref clampedPoint);
            MathUtil.VectorMin(ref max_bound, ref clampedPoint);

            Vector3 v = (clampedPoint - min_bound) * bvhQuantization;
            output = new UShortVector3();
            output[0] = (ushort)(v.X + 0.5f);
            output[1] = (ushort)(v.Y + 0.5f);
            output[2] = (ushort)(v.Z + 0.5f);
        }


        public static Vector3 Unquantize(
        ref UShortVector3 vecIn,
        ref Vector3 offset,
        ref Vector3 bvhQuantization)
        {
            Vector3 vecOut = new Vector3(
                (float)(vecIn[0]) / (bvhQuantization.X),
                (float)(vecIn[1]) / (bvhQuantization.Y),
                (float)(vecIn[2]) / (bvhQuantization.Z));
            vecOut += offset;
            return vecOut;
        }
    }
}
