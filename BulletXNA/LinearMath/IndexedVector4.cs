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
using System.Linq;
using System.Text;
using System.Diagnostics;

namespace BulletXNA.LinearMath
{
    public struct  IndexedVector4
    {
        public IndexedVector4(float x, float y, float z,float w)
        {
            X = x;
            Y = y;
            Z = z;
            W = w;
        }

        public IndexedVector4(float x)
        {
            X = x;
            Y = x;
            Z = x;
            W = x;
        }

        public IndexedVector4(IndexedVector4 v)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
            W = v.W;
        }

        public IndexedVector4(IndexedVector3 v,float w)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
            W = w;
        }

        public IndexedVector4(ref IndexedVector4 v)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
            W = v.W;
        }

        public IndexedVector3 ToVector3()
        {
            return new IndexedVector3(X, Y, Z);
        }

        public static IndexedVector4 Zero
        {
            get
            {
                return IndexedVector4._zero;
            }
        }

        public static IndexedVector4 operator -(IndexedVector4 value)
        {
            IndexedVector4 vector;
            vector.X = -value.X;
            vector.Y = -value.Y;
            vector.Z = -value.Z;
            vector.W = -value.W;
            return vector;
        }

        public static IndexedVector4 operator *(IndexedVector4 value, float scaleFactor)
        {
            IndexedVector4 vector;
            vector.X = value.X * scaleFactor;
            vector.Y = value.Y * scaleFactor;
            vector.Z = value.Z * scaleFactor;
            vector.W = value.W * scaleFactor;

            return vector;
        }

        public float this[int i]
        {
            get
            {
                switch (i)
                {
                    case (0): return X;
                    case (1): return Y;
                    case (2): return Z;
                    case (3): return W;
                    default:
                        {
                            Debug.Assert(false);
                            return 0.0f;
                        }
                }
            }
            set
            {
                switch (i)
                {
                    case (0): X = value; break;
                    case (1): Y = value; break;
                    case (2): Z = value; break;
                    case (3): W = value; break;
                    default:
                        {
                            Debug.Assert(false);
                            break;
                        }
                }
            }
        }

        private static IndexedVector4 _zero = new IndexedVector4(0f);
        private static IndexedVector4 _one = new IndexedVector4(1f);

        public float X;
        public float Y;
        public float Z;
        public float W;
    }
}
