using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;

namespace BulletXNA.LinearMath
{
    [Serializable]
    [StructLayout(LayoutKind.Sequential)]
    public struct Vector3 : IFormattable
    {
        public Vector3(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public Vector3(float x)
        {
            X = x;
            Y = x;
            Z = x;
        }

        public Vector3(Vector3 v)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
        }

        public Vector3(ref Vector3 v)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
        }

        public Vector3(ref Vector4 v)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
        }
        public Vector3(Vector4 v)
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

        public void Abs(out Vector3 result)
        {
            result.X = Math.Abs(X);
            result.Y = Math.Abs(Y);
            result.Z = Math.Abs(Z);
        }

        public Vector3 Abs()
        {
            return new Vector3(Math.Abs(X), Math.Abs(Y), Math.Abs(Z));
        }

        public Vector3 Absolute()
        {
            return new Vector3(Math.Abs(X), Math.Abs(Y), Math.Abs(Z));
        }

        public void Normalize()
        {
            float num = 1f / (float)Math.Sqrt(this.X * this.X + this.Y * this.Y + this.Z * this.Z);
            this.X *= num;
            this.Y *= num;
            this.Z *= num;
        }

        public Vector3 Normalized()
        {
            float num = 1f / (float)Math.Sqrt(X * X + Y * Y + Z * Z);
            return new Vector3(X * num, Y * num, Z * num);


        }

        public static void Transform(Vector3[] source, ref Matrix t, Vector3[] dest)
        {
            for (int i = 0; i < source.Length; ++i)
            {
                dest[i] = t * source[i];
            }
        }


        public static Vector3 Normalize(Vector3 v)
        {
            float num = 1f / (float)Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
            return new Vector3(v.X * num, v.Y * num, v.Z * num);
        }

        public static Vector3 Normalize(ref Vector3 v)
        {
            float num = 1f / (float)Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
            return new Vector3(v.X * num, v.Y * num, v.Z * num);
        }


        public Vector3 Cross(ref Vector3 v)
        {
            return new Vector3(
                Y * v.Z - Z * v.Y,
                Z * v.X - X * v.Z,
                X * v.Y - Y * v.X);
        }

        public Vector3 Cross(Vector3 v)
        {
            return new Vector3(
                Y * v.Z - Z * v.Y,
                Z * v.X - X * v.Z,
                X * v.Y - Y * v.X);
        }

        public static Vector3 Cross(Vector3 v, Vector3 v2)
        {
            return new Vector3(
                v.Y * v2.Z - v.Z * v2.Y,
                v.Z * v2.X - v.X * v2.Z,
                v.X * v2.Y - v.Y * v2.X);
        }

        public static Vector3 Cross(ref Vector3 v, ref Vector3 v2)
        {
            return new Vector3(
                v.Y * v2.Z - v.Z * v2.Y,
                v.Z * v2.X - v.X * v2.Z,
                v.X * v2.Y - v.Y * v2.X);
        }

        public static void Cross(out Vector3 r, ref Vector3 v, ref Vector3 v2)
        {
            r = new Vector3(
                v.Y * v2.Z - v.Z * v2.Y,
                v.Z * v2.X - v.X * v2.Z,
                v.X * v2.Y - v.Y * v2.X);
        }



        public static float Dot(Vector3 a, Vector3 b)
        {
            return (a.X * b.X) + (a.Y * b.Y) + (a.Z * b.Z);
        }

        public static float Dot(ref Vector3 a, ref Vector3 b)
        {
            return (a.X * b.X) + (a.Y * b.Y) + (a.Z * b.Z);
        }

        public static Vector3 operator +(Vector3 value1, Vector3 value2)
        {
            Vector3 vector;
            vector.X = value1.X + value2.X;
            vector.Y = value1.Y + value2.Y;
            vector.Z = value1.Z + value2.Z;
            return vector;
        }

        public static Vector3 operator -(Vector3 value1, Vector3 value2)
        {
            Vector3 vector;
            vector.X = value1.X - value2.X;
            vector.Y = value1.Y - value2.Y;
            vector.Z = value1.Z - value2.Z;
            return vector;
        }

        public static Vector3 operator *(Vector3 value, float scaleFactor)
        {
            Vector3 vector;
            vector.X = value.X * scaleFactor;
            vector.Y = value.Y * scaleFactor;
            vector.Z = value.Z * scaleFactor;
            return vector;
        }

        public static void Multiply(ref Vector3 output, ref Vector3 value1, ref Vector3 value2)
        {
            output.X = value1.X * value2.X;
            output.Y = value1.Y * value2.Y;
            output.Z = value1.Z * value2.Z;
        }

        public static void Subtract(out Vector3 output, ref Vector3 value1, ref Vector3 value2)
        {
            output.X = value1.X - value2.X;
            output.Y = value1.Y - value2.Y;
            output.Z = value1.Z - value2.Z;
        }

        public static Vector3 Subtract(ref Vector3 value1, ref Vector3 value2)
        {
            return new Vector3(value1.X - value2.X, value1.Y - value2.Y, value1.Z - value2.Z);
        }

        public static Vector3 operator /(Vector3 value, float scaleFactor)
        {

            float num = 1f / scaleFactor;
            Vector3 vector3;
            vector3.X = value.X * num;
            vector3.Y = value.Y * num;
            vector3.Z = value.Z * num;
            return vector3;
        }


        public static Vector3 operator *(float scaleFactor, Vector3 value)
        {
            Vector3 vector;
            vector.X = value.X * scaleFactor;
            vector.Y = value.Y * scaleFactor;
            vector.Z = value.Z * scaleFactor;
            return vector;
        }


        public static Vector3 operator -(Vector3 value)
        {
            Vector3 vector;
            vector.X = -value.X;
            vector.Y = -value.Y;
            vector.Z = -value.Z;
            return vector;
        }

        public static Vector3 operator *(Vector3 value1, Vector3 value2)
        {
            Vector3 vector;
            vector.X = value1.X * value2.X;
            vector.Y = value1.Y * value2.Y;
            vector.Z = value1.Z * value2.Z;
            return vector;
        }


        public static Vector3 operator /(Vector3 value1, Vector3 value2)
        {
            Vector3 vector;
            vector.X = value1.X / value2.X;
            vector.Y = value1.Y / value2.Y;
            vector.Z = value1.Z / value2.Z;
            return vector;
        }

        //public static Vector3 Transform(Vector3 position, Matrix matrix)
        //{
        //    Vector3 vector;
        //    float num3 = (((position.X * matrix._basis[0, 0]) + (position.Y * matrix._basis[1, 0])) + (position.Z * matrix._basis[2, 0])) + matrix.Translation.X;
        //    float num2 = (((position.X * matrix._basis[0, 1]) + (position.Y * matrix._basis[1, 1])) + (position.Z * matrix._basis[2, 1])) + matrix.Translation.Y;
        //    float num = (((position.X * matrix._basis[0, 2]) + (position.Y * matrix._basis[1, 2])) + (position.Z * matrix._basis[2, 2])) + matrix.Translation.Z;
        //    vector.X = num3;
        //    vector.Y = num2;
        //    vector.Z = num;
        //    return vector;
        //}


        //public static Vector3 Transform(ref Vector3 position, ref Matrix matrix)
        //{
        //    Vector3 vector;
        //    float num3 = (((position.X * matrix._basis[0, 0]) + (position.Y * matrix._basis[1, 0])) + (position.Z * matrix._basis[2, 0])) + matrix.Translation.X;
        //    float num2 = (((position.X * matrix._basis[0, 1]) + (position.Y * matrix._basis[1, 1])) + (position.Z * matrix._basis[2, 1])) + matrix.Translation.Y;
        //    float num = (((position.X * matrix._basis[0, 2]) + (position.Y * matrix._basis[1, 2])) + (position.Z * matrix._basis[2, 2])) + matrix.Translation.Z;
        //    vector.X = num3;
        //    vector.Y = num2;
        //    vector.Z = num;
        //    return vector;
        //}

        //public static void Transform(ref Vector3 position, ref Matrix matrix,out Vector3 vector)
        //{
        //    float num3 = (((position.X * matrix._basis[0, 0]) + (position.Y * matrix._basis[1, 0])) + (position.Z * matrix._basis[2, 0])) + matrix.Translation.X;
        //    float num2 = (((position.X * matrix._basis[0, 1]) + (position.Y * matrix._basis[1, 1])) + (position.Z * matrix._basis[2, 1])) + matrix.Translation.Y;
        //    float num = (((position.X * matrix._basis[0, 2]) + (position.Y * matrix._basis[1, 2])) + (position.Z * matrix._basis[2, 2])) + matrix.Translation.Z;
        //    vector.X = num3;
        //    vector.Y = num2;
        //    vector.Z = num;
        //}


        //public static Vector3 TransformNormal(Vector3 position, Matrix matrix)
        //{
        //    Vector3 vector;
        //    float num3 = (((position.X * matrix._basis[0, 0]) + (position.Y * matrix._basis[1, 0])) + (position.Z * matrix._basis[2, 0]));
        //    float num2 = (((position.X * matrix._basis[0, 1]) + (position.Y * matrix._basis[1, 1])) + (position.Z * matrix._basis[2, 1]));
        //    float num = (((position.X * matrix._basis[0, 2]) + (position.Y * matrix._basis[1, 2])) + (position.Z * matrix._basis[2, 2]));
        //    vector.X = num3;
        //    vector.Y = num2;
        //    vector.Z = num;
        //    return vector;
        //}

        //public static Vector3 TransformNormal(ref Vector3 position, ref Matrix matrix)
        //{
        //    Vector3 vector;
        //    float num3 = (((position.X * matrix._basis[0, 0]) + (position.Y * matrix._basis[1, 0])) + (position.Z * matrix._basis[2, 0]));
        //    float num2 = (((position.X * matrix._basis[0, 1]) + (position.Y * matrix._basis[1, 1])) + (position.Z * matrix._basis[2, 1]));
        //    float num = (((position.X * matrix._basis[0, 2]) + (position.Y * matrix._basis[1, 2])) + (position.Z * matrix._basis[2, 2]));
        //    vector.X = num3;
        //    vector.Y = num2;
        //    vector.Z = num;
        //    return vector;
        //}

        //public static void TransformNormal(ref Vector3 position, ref Matrix matrix, out Vector3 vector)
        //{
        //    float num3 = (((position.X * matrix._basis[0, 0]) + (position.Y * matrix._basis[1, 0])) + (position.Z * matrix._basis[2, 0]));
        //    float num2 = (((position.X * matrix._basis[0, 1]) + (position.Y * matrix._basis[1, 1])) + (position.Z * matrix._basis[2, 1]));
        //    float num = (((position.X * matrix._basis[0, 2]) + (position.Y * matrix._basis[1, 2])) + (position.Z * matrix._basis[2, 2]));
        //    vector.X = num3;
        //    vector.Y = num2;
        //    vector.Z = num;
        //}


        public float[] ToFloatArray()
        {
            return new float[] { X, Y, Z };
        }

        public static void Lerp(ref Vector3 a, ref Vector3 b, float t, out Vector3 c)
        {
            c = new Vector3(
                a.X + (b.X - a.X) * t,
                a.Y + (b.Y - a.Y) * t,
                a.Z + (b.Z - a.Z) * t);
        }

        public static Vector3 Lerp(ref Vector3 a, ref Vector3 b, float t)
        {
            return new Vector3(
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

        public static bool operator ==(Vector3 value1, Vector3 value2)
        {
            if (value1.X == value2.X && value1.Y == value2.Y)
                return value1.Z == value2.Z;
            else
                return false;
        }

        public static bool operator !=(Vector3 value1, Vector3 value2)
        {
            if (value1.X == value2.X && value1.Y == value2.Y)
                return value1.Z != value2.Z;
            else
                return true;
        }


        public bool Equals(Vector3 other)
        {
            if (this.X == other.X && this.Y == other.Y)
                return this.Z == other.Z;
            else
                return false;
        }

        public override bool Equals(object obj)
        {
            bool flag = false;
            if (obj is Vector3)
                flag = this.Equals((Vector3)obj);
            return flag;
        }

        public static Vector3 Zero
        {
            get
            {
                return Vector3._zero;
            }
        }

        public static Vector3 One
        {
            get
            {
                return Vector3._one;
            }
        }

        public static Vector3 Up
        {
            get
            {
                return Vector3._up;
            }
        }

        public static Vector3 Down
        {
            get
            {
                return Vector3._down;
            }
        }

        public static Vector3 Right
        {
            get
            {
                return Vector3._right;
            }
        }

        public static Vector3 Left
        {
            get
            {
                return Vector3._left;
            }
        }

        public static Vector3 Forward
        {
            get
            {
                return Vector3._forward;
            }
        }

        public static Vector3 Backward
        {
            get
            {
                return Vector3._backward;
            }
        }



        public static Vector3 UnitX
        {
            get
            {
                return new Vector3(1, 0, 0);
            }
        }

        public static Vector3 UnitY
        {
            get
            {
                return new Vector3(0, 1, 0);
            }
        }

        public static Vector3 UnitZ
        {
            get
            {
                return new Vector3(0, 0, 1);
            }
        }


        public float Dot(ref Vector3 v)
        {
            return X * v.X + Y * v.Y + Z * v.Z;
        }

        public float Dot(Vector3 v)
        {
            return X * v.X + Y * v.Y + Z * v.Z;
        }

        public float Triple(ref Vector3 b, ref Vector3 c)
        {
            return X * (b.Y * c.Z - b.Z * c.Y) +
                Y * (b.Z * c.X - b.X * c.Z) +
                Z * (b.X * c.Y - b.Y * c.X);
        }


        public void SetMin(ref Vector3 v)
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


        public void SetMax(ref Vector3 v)
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
            StringBuilder sb = new StringBuilder(32);
            sb.Append("{X:");
            sb.Append(this.X);
            sb.Append(" Y:");
            sb.Append(this.Y);
            sb.Append(" Z:");
            sb.Append(this.Z);
            sb.Append("}");
            return sb.ToString();
        }

        /// <summary>
        /// Returns a <see cref="System.String"/> that represents this instance.
        /// </summary>
        /// <param name="formatProvider">The format provider.</param>
        /// <returns>
        /// A <see cref="System.String"/> that represents this instance.
        /// </returns>
        public string ToString(IFormatProvider formatProvider)
        {
            return string.Format(formatProvider, "X:{0} Y:{1} Z:{2}", X, Y, Z);
        }

        /// <summary>
        /// Returns a <see cref="System.String"/> that represents this instance.
        /// </summary>
        /// <param name="format">The format.</param>
        /// <param name="formatProvider">The format provider.</param>
        /// <returns>
        /// A <see cref="System.String"/> that represents this instance.
        /// </returns>
        public string ToString(string format, IFormatProvider formatProvider)
        {
            if (format == null)
                return ToString(formatProvider);

            return string.Format(formatProvider, "X:{0} Y:{1} Z:{2}", X.ToString(format, formatProvider),
                Y.ToString(format, formatProvider), Z.ToString(format, formatProvider));
        }

        private static Vector3 _zero = new Vector3();
        private static Vector3 _one = new Vector3(1f, 1f, 1f);
        private static Vector3 _unitX = new Vector3(1f, 0.0f, 0.0f);
        private static Vector3 _unitY = new Vector3(0.0f, 1f, 0.0f);
        private static Vector3 _unitZ = new Vector3(0.0f, 0.0f, 1f);
        private static Vector3 _up = new Vector3(0.0f, 1f, 0.0f);
        private static Vector3 _down = new Vector3(0.0f, -1f, 0.0f);
        private static Vector3 _right = new Vector3(1f, 0.0f, 0.0f);
        private static Vector3 _left = new Vector3(-1f, 0.0f, 0.0f);
        private static Vector3 _forward = new Vector3(0.0f, 0.0f, -1f);
        private static Vector3 _backward = new Vector3(0.0f, 0.0f, 1f);

        public float X;
        public float Y;
        public float Z;
    }
}
