using System;
using System.Diagnostics;

namespace BulletXNA.LinearMath
{
    public struct IndexedQuaternion
    {

        public IndexedQuaternion(float x, float y, float z,float w)
        {
            X = x;
            Y = y;
            Z = z;
            W = w;
        }

        public IndexedQuaternion(IndexedVector3 axis, float angle)
        {
            //SetRotation(ref axis, angle);

            float d = axis.Length();
            Debug.Assert(d != 0.0f);
            float halfAngle = angle * 0.5f;
            float s = (float)Math.Sin(halfAngle) / d;
            X = axis.X * s;
            Y = axis.Y * s;
            Z = axis.Z * s;
            W = (float)Math.Cos(halfAngle);
        }

        public void SetRotation(ref IndexedVector3 axis, float angle)
        {
            float d = axis.Length();
            Debug.Assert(d != 0.0f);
            float halfAngle = angle * 0.5f;
            float s = (float)Math.Sin(halfAngle) / d;
            X = axis.X * s;
            Y = axis.Y * s;
            Z = axis.Z * s;
            W = (float)Math.Cos(halfAngle);
        }

        public void SetValue(float x, float y, float z, float w)
        {
            X = x;
            Y = y;
            Z = z;
            W = w;
        }


        public static IndexedQuaternion operator +(IndexedQuaternion quaternion1, IndexedQuaternion quaternion2)
        {
            IndexedQuaternion quaternion;
            quaternion.X = quaternion1.X + quaternion2.X;
            quaternion.Y = quaternion1.Y + quaternion2.Y;
            quaternion.Z = quaternion1.Z + quaternion2.Z;
            quaternion.W = quaternion1.W + quaternion2.W;
            return quaternion;
        }

        /// <summary>
        /// Subtracts a quaternion from another quaternion.
        /// </summary>
        /// <param name="quaternion1">Source quaternion.</param><param name="quaternion2">Source quaternion.</param>
        public static IndexedQuaternion operator -(IndexedQuaternion quaternion1, IndexedQuaternion quaternion2)
        {
            IndexedQuaternion quaternion;
            quaternion.X = quaternion1.X - quaternion2.X;
            quaternion.Y = quaternion1.Y - quaternion2.Y;
            quaternion.Z = quaternion1.Z - quaternion2.Z;
            quaternion.W = quaternion1.W - quaternion2.W;
            return quaternion;
        }

        public static IndexedQuaternion operator *(IndexedQuaternion q1, IndexedQuaternion q2)
        {
            return new IndexedQuaternion(q1.W * q2.X + q1.X * q2.W + q1.Y * q2.Z - q1.Z * q2.Y,
                q1.W * q2.Y + q1.Y * q2.W + q1.Z * q2.X - q1.X * q2.Z,
                q1.W * q2.Z + q1.Z * q2.W + q1.X * q2.Y - q1.Y * q2.X,
                q1.W * q2.W - q1.X * q2.X - q1.Y * q2.Y - q1.Z * q2.Z);
        }


        public static IndexedQuaternion operator *(IndexedQuaternion q1, IndexedVector3 v1)
        {
            return new IndexedQuaternion(q1.W * v1.X + q1.Y * v1.Z - q1.Z * v1.Y,
                q1.W * v1.Y + q1.Z * v1.X - q1.X * v1.Z,
                q1.W * v1.Z + q1.X * v1.Y - q1.Y * v1.X,
                -q1.X * v1.X - q1.Y * v1.Y - q1.Z * v1.Z);
        }


        public static IndexedQuaternion operator *(IndexedVector3 v1, IndexedQuaternion q1)
        {
	        return new IndexedQuaternion( v1.X * q1.W + v1.Y * q1.Z - v1.Z * q1.Y,
		        v1.Y * q1.W + v1.Z * q1.X - v1.X * q1.Z,
		        v1.Z * q1.W + v1.X * q1.Y - v1.Y * q1.X,
		        -v1.X * q1.X - v1.Y * q1.Y - v1.Z * q1.Z); 
        }


        public static IndexedQuaternion operator -(IndexedQuaternion value)
        {
            IndexedQuaternion q;
            q.X = -value.X;
            q.Y = -value.Y;
            q.Z = -value.Z;
            q.W = -value.W;
            return q;
        }

#if XNA
        public IndexedQuaternion(ref Microsoft.Xna.Framework.Quaternion q)
        {
            X = q.X;
            Y = q.Y;
            Z = q.Z;
            W = q.W;
        }

        public IndexedQuaternion(Microsoft.Xna.Framework.Quaternion q)
        {
            X = q.X;
            Y = q.Y;
            Z = q.Z;
            W = q.W;
        }
        public static implicit operator Microsoft.Xna.Framework.Quaternion(IndexedQuaternion q)
        {
            return new Microsoft.Xna.Framework.Quaternion(q.X, q.Y, q.Z,q.W);
        }

        public static implicit operator IndexedQuaternion(Microsoft.Xna.Framework.Quaternion q)
        {
            return new IndexedQuaternion(q.X, q.Y, q.Z,q.W);
        }


#endif

        /// <summary>
        /// Calculates the length squared of a Quaternion.
        /// </summary>
        public float LengthSquared()
        {
            return (float)(this.X * this.X + this.Y * this.Y + this.Z * this.Z + this.W * this.W);
        }

        /// <summary>
        /// Calculates the length of a Quaternion.
        /// </summary>
        public float Length()
        {
            return (float)Math.Sqrt(this.X * this.X + this.Y * this.Y + this.Z * this.Z + this.W * this.W);
        }

        /// <summary>
        /// Divides each component of the quaternion by the length of the quaternion.
        /// </summary>
        public void Normalize()
        {
            float num = 1f / (float)Math.Sqrt(this.X * this.X + this.Y * this.Y + this.Z * this.Z + this.W * this.W);
            this.X *= num;
            this.Y *= num;
            this.Z *= num;
            this.W *= num;
        }

        public static IndexedQuaternion Inverse(IndexedQuaternion q)
        {
            return new IndexedQuaternion(-q.X, -q.Y, -q.Z, q.W);
        }

        public IndexedQuaternion Inverse()
        {
            return new IndexedQuaternion(-X, -Y, -Z, W);
        }


        public float Dot(IndexedQuaternion q)
	    {
		    return X * q.X + Y * q.Y + Z * q.Z + W * q.W;
	    }

        public static float Dot(IndexedQuaternion q, IndexedQuaternion q2)
        {
            return q.X * q2.X + q.Y * q2.Y + q.Z * q2.Z + q.W * q2.W;
        }

        public static bool operator ==(IndexedQuaternion value1, IndexedQuaternion value2)
        {
            if (value1.X == value2.X && value1.Y == value2.Y && value1.Z == value2.Z)
                return value1.W == value2.W;
            else
                return false;
        }

        public static bool operator !=(IndexedQuaternion value1, IndexedQuaternion value2)
        {
            if (value1.X == value2.X && value1.Y == value2.Y && value1.Z == value2.Z)
                return value1.W != value2.W;
            else
                return true;
        }

        public IndexedVector3 QuatRotate(IndexedQuaternion rotation, IndexedVector3 v) 
        {
	        IndexedQuaternion q = rotation * v;
	        q *= rotation.Inverse();
	        return new IndexedVector3(q.X,q.Y,q.Z);
        }


        public float X;
        public float Y;
        public float Z;
        public float W;

        public static IndexedQuaternion Identity
        {
            get { return _identity; }
        }

        private static IndexedQuaternion _identity = new IndexedQuaternion(0,0,0,1);
    }
}
