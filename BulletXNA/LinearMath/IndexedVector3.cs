using System;
using System.Diagnostics;


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

        public IndexedVector3(float x)
        {
            X = x;
            Y = x;
            Z = x;
        }

        public IndexedVector3(IndexedVector3 v)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
        }

        public IndexedVector3(ref IndexedVector3 v)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
        }

#if XNA
        public IndexedVector3(ref Microsoft.Xna.Framework.Vector3 v)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
        }

        public IndexedVector3(Microsoft.Xna.Framework.Vector3 v)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
        }

        public Microsoft.Xna.Framework.Vector3 ToVector3()
        {
            return new Microsoft.Xna.Framework.Vector3(X, Y, Z);
        }

        public void ToVector3(out Microsoft.Xna.Framework.Vector3 result)
        {
            result = new Microsoft.Xna.Framework.Vector3(X, Y, Z);
        }

        public static IndexedVector3 operator +(Microsoft.Xna.Framework.Vector3 value1, IndexedVector3 value2)
        {
            IndexedVector3 vector;
            vector.X = value1.X + value2.X;
            vector.Y = value1.Y + value2.Y;
            vector.Z = value1.Z + value2.Z;
            return vector;
        }

        public static IndexedVector3 operator +(IndexedVector3 value1, Microsoft.Xna.Framework.Vector3 value2)
        {
            IndexedVector3 vector;
            vector.X = value1.X + value2.X;
            vector.Y = value1.Y + value2.Y;
            vector.Z = value1.Z + value2.Z;
            return vector;
        }

        public static IndexedVector3 operator -(Microsoft.Xna.Framework.Vector3 value1, IndexedVector3 value2)
        {
            IndexedVector3 vector;
            vector.X = value1.X - value2.X;
            vector.Y = value1.Y - value2.Y;
            vector.Z = value1.Z - value2.Z;
            return vector;
        }

        public static IndexedVector3 operator -(IndexedVector3 value1, Microsoft.Xna.Framework.Vector3 value2)
        {
            IndexedVector3 vector;
            vector.X = value1.X - value2.X;
            vector.Y = value1.Y - value2.Y;
            vector.Z = value1.Z - value2.Z;
            return vector;
        }

        // User-defined conversion from IndexedVector3 to Vector3
        public static implicit operator Microsoft.Xna.Framework.Vector3(IndexedVector3 v)
        {
            return new Microsoft.Xna.Framework.Vector3(v.X, v.Y, v.Z);
        }

        // User-defined conversion from IndexedVector3 to Vector3
        public static implicit operator IndexedVector3(Microsoft.Xna.Framework.Vector3 v)
        {
            return new IndexedVector3(v.X, v.Y, v.Z);
        }

        public static float Dot(ref IndexedVector3 a, ref Microsoft.Xna.Framework.Vector3 b)
        {
            return (a.X * b.X) + (a.Y * b.Y) + (a.Z * b.Z);
        }

        public static float Dot(IndexedVector3 a, Microsoft.Xna.Framework.Vector3 b)
        {
            return (a.X * b.X) + (a.Y * b.Y) + (a.Z * b.Z);
        }

        public static float Dot(Microsoft.Xna.Framework.Vector3 a, IndexedVector3 b)
        {
            return (a.X * b.X) + (a.Y * b.Y) + (a.Z * b.Z);
        }


#endif
        public IndexedVector3(ref IndexedVector4 v)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
        }
        public IndexedVector3(IndexedVector4 v)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
        }


        public float Length()
        {
            return (float)Math.Sqrt(X * X + Y * Y + Z * Z);
        }

        public float LengthSquared()
        {
            return X * X + Y * Y + Z * Z;
        }

        public void Abs(out IndexedVector3 result)
        {
            result.X = Math.Abs(X);
            result.Y = Math.Abs(Y);
            result.Z = Math.Abs(Z);
        }

        public IndexedVector3 Abs()
        {
            return new IndexedVector3(Math.Abs(X), Math.Abs(Y), Math.Abs(Z));
        }

        public IndexedVector3 Absolute()
        {
            return new IndexedVector3(Math.Abs(X), Math.Abs(Y), Math.Abs(Z));
        }

        public void Normalize()
        {
            float num = 1f / (float)Math.Sqrt(this.X * this.X + this.Y * this.Y + this.Z * this.Z);
            this.X *= num;
            this.Y *= num;
            this.Z *= num;
        }

        public IndexedVector3 Normalized()
        {
            float num = 1f / (float)Math.Sqrt(X * X + Y * Y + Z * Z);
            return new IndexedVector3(X * num, Y * num, Z * num);


        }

        public static void Transform(IndexedVector3[] source, ref IndexedMatrix t, IndexedVector3[] dest)
        {
            for (int i = 0; i < source.Length; ++i)
            {
                dest[i] = t * source[i];
            }
        }


        public static IndexedVector3 Normalize(IndexedVector3 v)
        {
            float num = 1f / (float)Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
            return new IndexedVector3(v.X * num, v.Y * num, v.Z * num);
        }

        public static IndexedVector3 Normalize(ref IndexedVector3 v)
        {
            float num = 1f / (float)Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
            return new IndexedVector3(v.X * num, v.Y * num, v.Z * num);
        }


        public IndexedVector3 Cross(ref IndexedVector3 v)
        {
            return new IndexedVector3(
                Y * v.Z - Z * v.Y,
                Z * v.X - X * v.Z,
                X * v.Y - Y * v.X);
        }

        public IndexedVector3 Cross(IndexedVector3 v)
        {
            return new IndexedVector3(
                Y * v.Z - Z * v.Y,
                Z * v.X - X * v.Z,
                X * v.Y - Y * v.X);
        }

        public static IndexedVector3 Cross(IndexedVector3 v, IndexedVector3 v2)
        {
            return new IndexedVector3(
                v.Y * v2.Z - v.Z * v2.Y,
                v.Z * v2.X - v.X * v2.Z,
                v.X * v2.Y - v.Y * v2.X);
        }

        public static IndexedVector3 Cross(ref IndexedVector3 v, ref IndexedVector3 v2)
        {
            return new IndexedVector3(
                v.Y * v2.Z - v.Z * v2.Y,
                v.Z * v2.X - v.X * v2.Z,
                v.X * v2.Y - v.Y * v2.X);
        }




        public static float Dot(IndexedVector3 a, IndexedVector3 b)
        {
            return (a.X * b.X) + (a.Y * b.Y) + (a.Z * b.Z);
        }

        public static float Dot(ref IndexedVector3 a, ref IndexedVector3 b)
        {
            return (a.X * b.X) + (a.Y * b.Y) + (a.Z * b.Z);
        }

        public static void Dot(ref IndexedVector3 a, ref IndexedVector3 b, out float r)
        {
            r = (a.X * b.X) + (a.Y * b.Y) + (a.Z * b.Z);
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

        public static IndexedVector3 operator /(IndexedVector3 value, float scaleFactor)
        {

            float num = 1f / scaleFactor;
            IndexedVector3 vector3;
            vector3.X = value.X * num;
            vector3.Y = value.Y * num;
            vector3.Z = value.Z * num;
            return vector3;
        }


        public static IndexedVector3 operator *(float scaleFactor, IndexedVector3 value)
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

        public static IndexedVector3 operator *(IndexedVector3 value1, IndexedVector3 value2)
        {
            IndexedVector3 vector;
            vector.X = value1.X * value2.X;
            vector.Y = value1.Y * value2.Y;
            vector.Z = value1.Z * value2.Z;
            return vector;
        }

        public static void Multiply(ref IndexedVector3 output, ref IndexedVector3 value1, ref IndexedVector3 value2)
        {
            output.X = value1.X * value2.X;
            output.Y = value1.Y * value2.Y;
            output.Z = value1.Z * value2.Z;
        }

        public static void Multiply(ref IndexedVector3 output, ref IndexedVector3 value1, float value2)
        {
            output.X = value1.X * value2;
            output.Y = value1.Y * value2;
            output.Z = value1.Z * value2;
        }


        public static void Multiply(ref IndexedVector3 output, ref IndexedVector3 value1, ref IndexedVector3 value2, float value3)
        {
            output.X = value1.X * value2.X * value3;
            output.Y = value1.Y * value2.Y * value3;
            output.Z = value1.Z * value2.Z * value3;
        }

        public static void Subtract(ref IndexedVector3 output, ref IndexedVector3 value1, ref IndexedVector3 value2)
        {
            output.X = value1.X - value2.X;
            output.Y = value1.Y - value2.Y;
            output.Z = value1.Z - value2.Z;
        }


        public static void Add(ref IndexedVector3 output, ref IndexedVector3 value1, ref IndexedVector3 value2)
        {
            output.X = value1.X + value2.X;
            output.Y = value1.Y + value2.Y;
            output.Z = value1.Z + value2.Z;
        }

        public static IndexedVector3 operator /(IndexedVector3 value1, IndexedVector3 value2)
        {
            IndexedVector3 vector;
            vector.X = value1.X / value2.X;
            vector.Y = value1.Y / value2.Y;
            vector.Z = value1.Z / value2.Z;
            return vector;
        }

        //public static IndexedVector3 Transform(IndexedVector3 position, IndexedMatrix matrix)
        //{
        //    IndexedVector3 vector;
        //    float num3 = (((position.X * matrix.M11) + (position.Y * matrix.M21)) + (position.Z * matrix.M31)) + matrix.M41;
        //    float num2 = (((position.X * matrix.M12) + (position.Y * matrix.M22)) + (position.Z * matrix.M32)) + matrix.M42;
        //    float num = (((position.X * matrix.M13) + (position.Y * matrix.M23)) + (position.Z * matrix.M33)) + matrix.M43;
        //    vector.X = num3;
        //    vector.Y = num2;
        //    vector.Z = num;
        //    return vector;
        //}


        //public static IndexedVector3 Transform(ref IndexedVector3 position, ref IndexedMatrix matrix)
        //{
        //    IndexedVector3 vector;
        //    float num3 = (((position.X * matrix.M11) + (position.Y * matrix.M21)) + (position.Z * matrix.M31)) + matrix.M41;
        //    float num2 = (((position.X * matrix.M12) + (position.Y * matrix.M22)) + (position.Z * matrix.M32)) + matrix.M42;
        //    float num = (((position.X * matrix.M13) + (position.Y * matrix.M23)) + (position.Z * matrix.M33)) + matrix.M43;
        //    vector.X = num3;
        //    vector.Y = num2;
        //    vector.Z = num;
        //    return vector;
        //}

        //public static void Transform(ref IndexedVector3 position, ref IndexedMatrix matrix,out IndexedVector3 vector)
        //{
        //    float num3 = (((position.X * matrix.M11) + (position.Y * matrix.M21)) + (position.Z * matrix.M31)) + matrix.M41;
        //    float num2 = (((position.X * matrix.M12) + (position.Y * matrix.M22)) + (position.Z * matrix.M32)) + matrix.M42;
        //    float num = (((position.X * matrix.M13) + (position.Y * matrix.M23)) + (position.Z * matrix.M33)) + matrix.M43;
        //    vector.X = num3;
        //    vector.Y = num2;
        //    vector.Z = num;
        //}


        //public static IndexedVector3 TransformNormal(IndexedVector3 position, IndexedMatrix matrix)
        //{
        //    IndexedVector3 vector;
        //    float num3 = (((position.X * matrix.M11) + (position.Y * matrix.M21)) + (position.Z * matrix.M31));
        //    float num2 = (((position.X * matrix.M12) + (position.Y * matrix.M22)) + (position.Z * matrix.M32));
        //    float num = (((position.X * matrix.M13) + (position.Y * matrix.M23)) + (position.Z * matrix.M33));
        //    vector.X = num3;
        //    vector.Y = num2;
        //    vector.Z = num;
        //    return vector;
        //}

        //public static IndexedVector3 TransformNormal(ref IndexedVector3 position, ref IndexedMatrix matrix)
        //{
        //    IndexedVector3 vector;
        //    float num3 = (((position.X * matrix.M11) + (position.Y * matrix.M21)) + (position.Z * matrix.M31));
        //    float num2 = (((position.X * matrix.M12) + (position.Y * matrix.M22)) + (position.Z * matrix.M32));
        //    float num = (((position.X * matrix.M13) + (position.Y * matrix.M23)) + (position.Z * matrix.M33));
        //    vector.X = num3;
        //    vector.Y = num2;
        //    vector.Z = num;
        //    return vector;
        //}

        //public static void TransformNormal(ref IndexedVector3 position, ref IndexedMatrix matrix, out IndexedVector3 vector)
        //{
        //    float num3 = (((position.X * matrix.M11) + (position.Y * matrix.M21)) + (position.Z * matrix.M31));
        //    float num2 = (((position.X * matrix.M12) + (position.Y * matrix.M22)) + (position.Z * matrix.M32));
        //    float num = (((position.X * matrix.M13) + (position.Y * matrix.M23)) + (position.Z * matrix.M33));
        //    vector.X = num3;
        //    vector.Y = num2;
        //    vector.Z = num;
        //}


        public float[] ToFloatArray()
        {
            return new float[] { X, Y, Z };
        }

        public static void Lerp(ref IndexedVector3 a, ref IndexedVector3 b, float t, out IndexedVector3 c)
        {
            c = new IndexedVector3(
                a.X + (b.X - a.X) * t,
                a.Y + (b.Y - a.Y) * t,
                a.Z + (b.Z - a.Z) * t);
        }

        public static IndexedVector3 Lerp(ref IndexedVector3 a, ref IndexedVector3 b, float t)
        {
            return new IndexedVector3(
                a.X + (b.X - a.X) * t,
                a.Y + (b.Y - a.Y) * t,
                a.Z + (b.Z - a.Z) * t);
        }


        // FIXME - try this as a switch??
        public float this[int i]
        {
            get
            {
                switch (i)
                {
                    case (0): return X;
                    case (1): return Y;
                    case (2): return Z;
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
                    default:
                        {
                            Debug.Assert(false);
                            break;
                        }
                }
            }
        }

        public static bool operator ==(IndexedVector3 value1, IndexedVector3 value2)
        {
            if (value1.X == value2.X && value1.Y == value2.Y)
                return value1.Z == value2.Z;
            else
                return false;
        }

        public static bool operator !=(IndexedVector3 value1, IndexedVector3 value2)
        {
            if (value1.X == value2.X && value1.Y == value2.Y)
                return value1.Z != value2.Z;
            else
                return true;
        }


        public bool Equals(IndexedVector3 other)
        {
            if (this.X == other.X && this.Y == other.Y)
                return this.Z == other.Z;
            else
                return false;
        }

        public override bool Equals(object obj)
        {
            bool flag = false;
            if (obj is IndexedVector3)
                flag = this.Equals((IndexedVector3)obj);
            return flag;
        }

        public static IndexedVector3 Zero
        {
            get
            {
                return IndexedVector3._zero;
            }
        }

        public static IndexedVector3 One
        {
            get
            {
                return IndexedVector3._one;
            }
        }

        public static IndexedVector3 Up
        {
            get
            {
                return IndexedVector3._up;
            }
        }

        public static IndexedVector3 Down
        {
            get
            {
                return IndexedVector3._down;
            }
        }

        public static IndexedVector3 Right
        {
            get
            {
                return IndexedVector3._right;
            }
        }

        public static IndexedVector3 Left
        {
            get
            {
                return IndexedVector3._left;
            }
        }

        public static IndexedVector3 Forward
        {
            get
            {
                return IndexedVector3._forward;
            }
        }

        public static IndexedVector3 Backward
        {
            get
            {
                return IndexedVector3._backward;
            }
        }




        public float Dot(ref IndexedVector3 v)
        {
            return X * v.X + Y * v.Y + Z * v.Z;
        }

        public float Dot(IndexedVector3 v)
        {
            return X * v.X + Y * v.Y + Z * v.Z;
        }

        public float Triple(ref IndexedVector3 b, ref IndexedVector3 c)
        {
            return X * (b.Y * c.Z - b.Z * c.Y) +
                Y * (b.Z * c.X - b.X * c.Z) +
                Z * (b.X * c.Y - b.Y * c.X);
        }


        public void SetMin(ref IndexedVector3 v)
        {
            if (v.X < X)
            {
                X = v.X;
            }
            if (v.Y < Y)
            {
                Y = v.Y;
            }
            if (v.Z < Z)
            {
                Z = v.Z;
            }
        }


        public void SetMax(ref IndexedVector3 v)
        {
            if (v.X > X)
            {
                X = v.X;
            }
            if (v.Y > Y)
            {
                Y = v.Y;
            }
            if (v.Z > Z)
            {
                Z = v.Z;
            }
        }

        public int MaxAxis()
        {
            return X < Y ? (Y < Z ? 2 : 1) : (X < Z ? 2 : 0);
        }

        public int MinAxis()
        {
            return X < Y ? (Y < Z ? 0 : 2) : (X < Z ? 1 : 2);
        }

        public override string ToString()
        {
            return "X : " + X + " Y " + Y + " Z " + Z;
        }


        private static IndexedVector3 _zero = new IndexedVector3();
        private static IndexedVector3 _one = new IndexedVector3(1f, 1f, 1f);
        private static IndexedVector3 _unitX = new IndexedVector3(1f, 0.0f, 0.0f);
        private static IndexedVector3 _unitY = new IndexedVector3(0.0f, 1f, 0.0f);
        private static IndexedVector3 _unitZ = new IndexedVector3(0.0f, 0.0f, 1f);
        private static IndexedVector3 _up = new IndexedVector3(0.0f, 1f, 0.0f);
        private static IndexedVector3 _down = new IndexedVector3(0.0f, -1f, 0.0f);
        private static IndexedVector3 _right = new IndexedVector3(1f, 0.0f, 0.0f);
        private static IndexedVector3 _left = new IndexedVector3(-1f, 0.0f, 0.0f);
        private static IndexedVector3 _forward = new IndexedVector3(0.0f, 0.0f, -1f);
        private static IndexedVector3 _backward = new IndexedVector3(0.0f, 0.0f, 1f);


        public float X;
        public float Y;
        public float Z;
    }
}
