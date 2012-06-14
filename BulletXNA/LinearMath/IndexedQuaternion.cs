using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
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
            // un-necessary really :(
            X = Y = Z = W = 0f;
            SetRotation(ref axis, angle);
        }

        public void SetRotation(ref IndexedVector3 axis, float angle)
	    {
		    float d = axis.Length();
		    Debug.Assert(d != 0.0f);
            float s = (float)Math.Sin(angle * 0.5f) / d;
		    SetValue(axis.X * s, axis.Y * s, axis.Z * s, 
			    (float)Math.Cos(angle * 0.5f));
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


        public static IndexedQuaternion operator -(IndexedQuaternion value)
        {
            IndexedQuaternion q;
            q.X = -value.X;
            q.Y = -value.Y;
            q.Z = -value.Z;
            q.W = -value.W;
            return q;
        }



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
