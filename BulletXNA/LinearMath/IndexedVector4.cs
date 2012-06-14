using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

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



        private static IndexedVector4 _zero = new IndexedVector4(0f);
        private static IndexedVector4 _one = new IndexedVector4(1f);

        public float X;
        public float Y;
        public float Z;
        public float W;
    }
}
