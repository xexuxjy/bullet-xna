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
