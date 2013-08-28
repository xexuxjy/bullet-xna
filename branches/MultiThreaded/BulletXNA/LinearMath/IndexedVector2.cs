using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Diagnostics;

namespace BulletXNA.LinearMath
{
    public struct IndexedVector2
    {
        public IndexedVector2(float x, float y)
        {
            X = x;
            Y = y;
        }

        public IndexedVector2(float x)
        {
            X = x;
            Y = x;
        }

        public IndexedVector2(IndexedVector2 v)
        {
            X = v.X;
            Y = v.Y;
        }

        public IndexedVector2(ref IndexedVector2 v)
        {
            X = v.X;
            Y = v.Y;
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
                    default:
                        {
                            Debug.Assert(false);
                            break;
                        }
                }
            }
        }

        public static bool operator ==(IndexedVector2 value1, IndexedVector2 value2)
        {
            return value1.X == value2.X && value1.Y == value2.Y;
        }

        public static bool operator !=(IndexedVector2 value1, IndexedVector2 value2)
        {
            return value1.X != value2.X || value1.Y != value2.Y;
        }


        public bool Equals(IndexedVector2 other)
        {
            return  (this.X == other.X && this.Y == other.Y);
        }

        public override bool Equals(object obj)
        {
            bool flag = false;
            if (obj is IndexedVector2)
                flag = this.Equals((IndexedVector2)obj);
            return flag;
        }


        public override string ToString()
        {
            return "X : " + X + " Y " + Y ;
        }


        public float X;
        public float Y;

    }
}
