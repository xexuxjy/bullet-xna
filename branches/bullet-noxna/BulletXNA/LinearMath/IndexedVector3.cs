using System;
using System.Diagnostics;
using Microsoft.Xna.Framework;

namespace BulletXNA.LinearMath
{
    public struct IndexedVector3
    {
        public IndexedVector3(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public IndexedVector3(Vector3 v) : this(ref v)
        {

        }

        public IndexedVector3(IndexedVector3 v)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
        }

        public IndexedVector3(ref Vector3 v)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
        }

        public Vector3 ToVector3()
        {
            return new Vector3(X, Y, Z);
        }

        public void ToVector3(out Vector3 result)
        {
            result = new Vector3(X, Y, Z);
        }

        public void Abs(out IndexedVector3 result)
        {
            result.X = Math.Abs(X);
            result.Y = Math.Abs(Y);
            result.Z = Math.Abs(Z);
        }

        public static float Dot(ref IndexedVector3 a, ref IndexedVector3 b)
        {
            return (a.X * b.X) + (a.Y * b.Y) + (a.Z * b.Z);
        }

        public static void Dot(ref IndexedVector3 a, ref IndexedVector3 b,out float r)
        {
            r = (a.X * b.X) + (a.Y * b.Y) + (a.Z * b.Z);
        }

        public static float Dot(Vector3 a, IndexedVector3 b)
        {
            return Dot(ref a, ref b);
        }

        public static float Dot(ref Vector3 a, ref IndexedVector3 b)
        {
            return (a.X * b.X) + (a.Y * b.Y) + (a.Z * b.Z);
        }

        public static float Dot(IndexedVector3 a, Vector3 b)
        {
            return Dot(ref a, ref b);
        }

        public static float Dot(ref IndexedVector3 a, ref Vector3 b)
        {
            return (a.X * b.X) + (a.Y * b.Y) + (a.Z * b.Z);
        }


        public static IndexedVector3 operator +(IndexedVector3 value1, IndexedVector3 value2)
        {
            IndexedVector3 vector;
            vector.X = value1.X + value2.X;
            vector.Y = value1.Y + value2.Y;
            vector.Z = value1.Z + value2.Z;
            return vector;
        }

        public static IndexedVector3 operator -(IndexedVector3 value1, IndexedVector3 value2)
        {
            IndexedVector3 vector;
            vector.X = value1.X - value2.X;
            vector.Y = value1.Y - value2.Y;
            vector.Z = value1.Z - value2.Z;
            return vector;
        }

        public static IndexedVector3 operator *(IndexedVector3 value, float scaleFactor)
        {
            IndexedVector3 vector;
            vector.X = value.X * scaleFactor;
            vector.Y = value.Y * scaleFactor;
            vector.Z = value.Z * scaleFactor;
            return vector;
        }

        public static IndexedVector3 operator -(IndexedVector3 value)
        {
            IndexedVector3 vector;
            vector.X = -value.X;
            vector.Y = -value.Y;
            vector.Z = -value.Z;
            return vector;
        }



        public static IndexedVector3 Transform(ref IndexedVector3 position, ref Matrix matrix)
        {
            IndexedVector3 vector;
            float num3 = (((position.X * matrix.M11) + (position.Y * matrix.M21)) + (position.Z * matrix.M31)) + matrix.M41;
            float num2 = (((position.X * matrix.M12) + (position.Y * matrix.M22)) + (position.Z * matrix.M32)) + matrix.M42;
            float num = (((position.X * matrix.M13) + (position.Y * matrix.M23)) + (position.Z * matrix.M33)) + matrix.M43;
            vector.X = num3;
            vector.Y = num2;
            vector.Z = num;
            return vector;
        }

        public static void Transform(ref IndexedVector3 position, ref Matrix matrix,out IndexedVector3 vector)
        {
            float num3 = (((position.X * matrix.M11) + (position.Y * matrix.M21)) + (position.Z * matrix.M31)) + matrix.M41;
            float num2 = (((position.X * matrix.M12) + (position.Y * matrix.M22)) + (position.Z * matrix.M32)) + matrix.M42;
            float num = (((position.X * matrix.M13) + (position.Y * matrix.M23)) + (position.Z * matrix.M33)) + matrix.M43;
            vector.X = num3;
            vector.Y = num2;
            vector.Z = num;
        }

        public static IndexedVector3 TransformNormal(ref IndexedVector3 position, ref Matrix matrix)
        {
            IndexedVector3 vector;
            float num3 = (((position.X * matrix.M11) + (position.Y * matrix.M21)) + (position.Z * matrix.M31));
            float num2 = (((position.X * matrix.M12) + (position.Y * matrix.M22)) + (position.Z * matrix.M32));
            float num = (((position.X * matrix.M13) + (position.Y * matrix.M23)) + (position.Z * matrix.M33));
            vector.X = num3;
            vector.Y = num2;
            vector.Z = num;
            return vector;
        }

        public static void TransformNormal(ref IndexedVector3 position, ref Matrix matrix, out IndexedVector3 vector)
        {
            float num3 = (((position.X * matrix.M11) + (position.Y * matrix.M21)) + (position.Z * matrix.M31));
            float num2 = (((position.X * matrix.M12) + (position.Y * matrix.M22)) + (position.Z * matrix.M32));
            float num = (((position.X * matrix.M13) + (position.Y * matrix.M23)) + (position.Z * matrix.M33));
            vector.X = num3;
            vector.Y = num2;
            vector.Z = num;
        }


        public float[] ToFloatArray()
        {
            return new float[] { X, Y, Z };
        }

 


        public float this[int i]
        {
            get
            {
                if (i == 0) return X;
                if (i == 1) return Y;
                if (i == 2) return Z;
                Debug.Assert(false);
                return 0.0f;
            }
            set
            {
                if (i == 0) { X = value; return; }
                if (i == 1) {Y = value; return;}
                if (i == 2) { Z = value; return; }
                Debug.Assert(false);
            }
        }

        public float X;
        public float Y;
        public float Z;
    }
}
