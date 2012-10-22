
using System.Diagnostics;
using System;

namespace BulletXNA.LinearMath
{
    public struct IndexedBasisMatrix
    {
        private static IndexedBasisMatrix _identity = new IndexedBasisMatrix(1f, 0.0f, 0.0f, 0.0f, 1f, 0.0f, 0.0f, 0.0f, 1f);
        public static IndexedBasisMatrix Identity
        {
            get
            {
                return IndexedBasisMatrix._identity;
            }
        }

        public IndexedBasisMatrix Scaled(IndexedVector3 s)
        {

            return new IndexedBasisMatrix(_el0.X * s.X, _el0.Y * s.Y, _el0.Z * s.Z,
                                        _el1.X * s.X, _el1.Y * s.Y, _el1.Z * s.Z,
                                        _el2.X * s.X, _el2.Y * s.Y, _el2.Z * s.Z);

        }

        public IndexedBasisMatrix Scaled(ref IndexedVector3 s)
        {

            return new IndexedBasisMatrix(_el0.X * s.X, _el0.Y * s.Y, _el0.Z * s.Z,
                                        _el1.X * s.X, _el1.Y * s.Y, _el1.Z * s.Z,
                                        _el2.X * s.X, _el2.Y * s.Y, _el2.Z * s.Z);

        }


        public IndexedBasisMatrix(float m11, float m12, float m13, float m21, float m22, float m23, float m31, float m32, float m33)
        {
            _el0 = new IndexedVector3(m11, m12, m13);
            _el1 = new IndexedVector3(m21, m22, m23);
            _el2 = new IndexedVector3(m31, m32, m33);
        }

        public IndexedBasisMatrix(IndexedVector3 row0, IndexedVector3 row1, IndexedVector3 row2)
        {
            _el0 = row0;
            _el1 = row1;
            _el2 = row2;
        }

        public IndexedBasisMatrix(ref IndexedVector3 row0, ref IndexedVector3 row1, ref IndexedVector3 row2)
        {
            _el0 = row0;
            _el1 = row1;
            _el2 = row2;
        }

        public IndexedBasisMatrix(IndexedQuaternion q)
        {
            float d = q.LengthSquared();
            Debug.Assert(d != 0.0f);
            float s = 2.0f / d;
            float xs = q.X * s, ys = q.Y * s, zs = q.Z * s;
            float wx = q.W * xs, wy = q.W * ys, wz = q.W * zs;
            float xx = q.X * xs, xy = q.X * ys, xz = q.X * zs;
            float yy = q.Y * ys, yz = q.Y * zs, zz = q.Z * zs;
            _el0 = new IndexedVector3(1.0f - (yy + zz), xy - wz, xz + wy);
            _el1 = new IndexedVector3(xy + wz, 1.0f - (xx + zz), yz - wx);
            _el2 = new IndexedVector3(xz - wy, yz + wx, 1.0f - (xx + yy));
        }


        public IndexedBasisMatrix(ref IndexedQuaternion q)
        {
            float d = q.LengthSquared();
            Debug.Assert(d != 0.0f);
            float s = 2.0f / d;
            float xs = q.X * s, ys = q.Y * s, zs = q.Z * s;
            float wx = q.W * xs, wy = q.W * ys, wz = q.W * zs;
            float xx = q.X * xs, xy = q.X * ys, xz = q.X * zs;
            float yy = q.Y * ys, yz = q.Y * zs, zz = q.Z * zs;
            _el0 = new IndexedVector3(1.0f - (yy + zz), xy - wz, xz + wy);
            _el1 = new IndexedVector3(xy + wz, 1.0f - (xx + zz), yz - wx);
            _el2 = new IndexedVector3(xz - wy, yz + wx, 1.0f - (xx + yy));
        }



        public void SetValue(float m11, float m12, float m13, float m21, float m22, float m23, float m31, float m32, float m33)
        {
            this[0] = new IndexedVector3(m11, m12, m13);
            this[1] = new IndexedVector3(m21, m22, m23);
            this[2] = new IndexedVector3(m31, m32, m33);
        }


        public IndexedVector3 GetColumn(int i)
        {
            Debug.Assert(i >= 0 && i < 3);
            return new IndexedVector3(_el0[i], _el1[i], _el2[i]);
        }

        public IndexedVector3 GetRow(int i)
        {
            Debug.Assert(i >= 0 && i < 3);
            switch (i)
            {
                case (0):
                    return _el0;
                case (1):
                    return _el1;
                case (2):
                    return _el2;
            }
            Debug.Assert(false);
            return IndexedVector3.Zero;
        }


        public float this[int i, int j]
        {
            get
            {
                switch (i)
                {
                    case (0):
                        {
                            switch (j)
                            {
                                case (0):
                                    return _el0.X;
                                case (1):
                                    return _el0.Y;
                                case (2):
                                    return _el0.Z;
                                default:
                                    break;
                            }
                            break;
                        }
                    case (1):
                        {
                            switch (j)
                            {
                                case (0):
                                    return _el1.X;
                                case (1):
                                    return _el1.Y;
                                case (2):
                                    return _el1.Z;
                                default:
                                    break;
                            }
                            break;
                        }
                    case (2):
                        {
                            switch (j)
                            {
                                case (0):
                                    return _el2.X;
                                case (1):
                                    return _el2.Y;
                                case (2):
                                    return _el2.Z;
                                default:
                                    break;
                            }
                            break;
                        }
                }
                Debug.Assert(false);
                return 0.0f;
            }
            set
            {
                switch (i)
                {
                    case (0):
                        {
                            switch (j)
                            {
                                case (0):
                                    _el0.X = value;
                                    break;
                                case (1):
                                    _el0.Y = value;
                                    break;
                                case (2):
                                    _el0.Z = value;
                                    break;
                                default:
                                    break;
                            }
                            break;
                        }
                    case (1):
                        {
                            switch (j)
                            {
                                case (0):
                                    _el1.X = value;
                                    break;
                                case (1):
                                    _el1.Y = value;
                                    break;
                                case (2):
                                    _el1.Z = value;
                                    break;
                                default:
                                    break;
                            }
                            break;
                        }
                    case (2):
                        {
                            switch (j)
                            {
                                case (0):
                                    _el2.X = value;
                                    break;
                                case (1):
                                    _el2.Y = value;
                                    break;
                                case (2):
                                    _el2.Z = value;
                                    break;
                                default:
                                    break;
                            }
                            break;
                        }
                }
            }

        }


        public IndexedVector3 this[int i]
        {
            get
            {
                Debug.Assert(i >= 0 && i < 3);
                switch (i)
                {
                    case (0):
                        return _el0;
                    case (1):
                        return _el1;
                    case (2):
                        return _el2;
                }
                Debug.Assert(false);
                return IndexedVector3.Zero;
            }
            set
            {
                Debug.Assert(i >= 0 && i < 3);

                switch (i)
                {
                    case (0):
                        _el0 = value;
                        break;
                    case (1):
                        _el1 = value;
                        break;
                    case (2):
                        _el2 = value;
                        break;
                }
            }
        }



        public static IndexedBasisMatrix Transpose(IndexedBasisMatrix IndexedMatrix)
        {
            return new IndexedBasisMatrix(IndexedMatrix._el0.X, IndexedMatrix._el1.X, IndexedMatrix._el2.X,
                IndexedMatrix._el0.Y, IndexedMatrix._el1.Y, IndexedMatrix._el2.Y,
                IndexedMatrix._el0.Z, IndexedMatrix._el1.Z, IndexedMatrix._el2.Z);

        }

        public static void Transpose(ref IndexedBasisMatrix IndexedMatrix, out IndexedBasisMatrix result)
        {
            result = new IndexedBasisMatrix(IndexedMatrix._el0.X, IndexedMatrix._el1.X, IndexedMatrix._el2.X,
                IndexedMatrix._el0.Y, IndexedMatrix._el1.Y, IndexedMatrix._el2.Y,
                IndexedMatrix._el0.Z, IndexedMatrix._el1.Z, IndexedMatrix._el2.Z);
        }

        public static bool operator ==(IndexedBasisMatrix matrix1, IndexedBasisMatrix matrix2)
        {
            return matrix1._el0 == matrix2._el0 &&
                matrix1._el1 == matrix2._el1 &&
                matrix1._el2 == matrix2._el2;
        }

        public static bool operator !=(IndexedBasisMatrix matrix1, IndexedBasisMatrix matrix2)
        {
            return matrix1._el0 != matrix2._el0 ||
                matrix1._el1 != matrix2._el1 ||
                matrix1._el2 != matrix2._el2;
        }

        public static IndexedVector3 operator *(IndexedBasisMatrix m, IndexedVector3 v)
        {
            return new IndexedVector3(m._el0.X * v.X + m._el0.Y * v.Y + m._el0.Z * v.Z,
            m._el1.X * v.X + m._el1.Y * v.Y + m._el1.Z * v.Z,
            m._el2.X * v.X + m._el2.Y * v.Y + m._el2.Z * v.Z);
        }

        public static void Multiply(ref IndexedVector3 vout, ref IndexedBasisMatrix m, ref IndexedVector3 v)
        {
            vout = new IndexedVector3(m._el0.X * v.X + m._el0.Y * v.Y + m._el0.Z * v.Z,
            m._el1.X * v.X + m._el1.Y * v.Y + m._el1.Z * v.Z,
            m._el2.X * v.X + m._el2.Y * v.Y + m._el2.Z * v.Z);
        }


        public static IndexedVector3 operator *(IndexedVector3 v, IndexedBasisMatrix m)
        {
            return new IndexedVector3(m.TDotX(ref v), m.TDotY(ref v), m.TDotZ(ref v));
        }

        public static void Multiply(ref IndexedVector3 vout, ref IndexedVector3 vin, ref IndexedBasisMatrix m)
        {
            vout = new IndexedVector3(m.TDotX(ref vin), m.TDotY(ref vin), m.TDotZ(ref vin));
        }


        public static IndexedBasisMatrix operator *(IndexedBasisMatrix m1, IndexedBasisMatrix m2)
        {
            return new IndexedBasisMatrix(
                m2.TDotX(ref m1._el0), m2.TDotY(ref m1._el0), m2.TDotZ(ref m1._el0),
                m2.TDotX(ref m1._el1), m2.TDotY(ref m1._el1), m2.TDotZ(ref m1._el1),
                m2.TDotX(ref m1._el2), m2.TDotY(ref m1._el2), m2.TDotZ(ref m1._el2));
        }


        public static IndexedBasisMatrix operator *(IndexedBasisMatrix m1, float s)
        {
            return new IndexedBasisMatrix(m1._el0 * s, m1._el1 * s, m1._el2 * s);
        }

        public static IndexedBasisMatrix CreateScale(IndexedVector3 scale)
        {
            return new IndexedBasisMatrix(new IndexedVector3(scale.X, 0, 0),
            new IndexedVector3(0, scale.Y, 0),
            new IndexedVector3(0, 0, scale.Z));

        }


        public void SetEulerZYX(float eulerX, float eulerY, float eulerZ)
        {
            float ci = (float)Math.Cos(eulerX);
            float cj = (float)Math.Cos(eulerY);
            float ch = (float)Math.Cos(eulerZ);
            float si = (float)Math.Sin(eulerX);
            float sj = (float)Math.Sin(eulerY);
            float sh = (float)Math.Sin(eulerZ);
            float cc = ci * ch;
            float cs = ci * sh;
            float sc = si * ch;
            float ss = si * sh;

            SetValue(cj * ch, sj * sc - cs, sj * cc + ss, cj * sh, sj * ss + cc, sj * cs - sc, -sj, cj * si, cj * ci);

        }

        public float TDotX(ref IndexedVector3 v)
        {
            return _el0.X * v.X + _el1.X * v.Y + _el2.X * v.Z;
        }
        public float TDotY(ref IndexedVector3 v)
        {
            return _el0.Y * v.X + _el1.Y * v.Y + _el2.Y * v.Z;
        }
        public float TDotZ(ref IndexedVector3 v)
        {
            return _el0.Z * v.X + _el1.Z * v.Y + _el2.Z * v.Z;
        }

        public IndexedBasisMatrix Inverse()
        {
            IndexedVector3 co = new IndexedVector3(Cofac(1, 1, 2, 2), Cofac(1, 2, 2, 0), Cofac(1, 0, 2, 1));
            float det = this[0].Dot(co);
            Debug.Assert(det != 0.0f);
            float s = 1.0f / det;
            return new IndexedBasisMatrix(co.X * s, Cofac(0, 2, 2, 1) * s, Cofac(0, 1, 1, 2) * s,
                co.Y * s, Cofac(0, 0, 2, 2) * s, Cofac(0, 2, 1, 0) * s,
                co.Z * s, Cofac(0, 1, 2, 0) * s, Cofac(0, 0, 1, 1) * s);

        }

        public float Cofac(int r1, int c1, int r2, int c2)
        {
            // slow?
            return this[r1][c1] * this[r2][c2] - this[r1][c2] * this[r2][c1];
        }

        public IndexedBasisMatrix TransposeTimes(IndexedBasisMatrix m)
        {
            return new IndexedBasisMatrix(
        _el0.X * m._el0.X + _el1.X * m._el1.X + _el2.X * m._el2.X,
        _el0.X * m._el0.Y + _el1.X * m._el1.Y + _el2.X * m._el2.Y,
        _el0.X * m._el0.Z + _el1.X * m._el1.Z + _el2.X * m._el2.Z,
        _el0.Y * m._el0.X + _el1.Y * m._el1.X + _el2.Y * m._el2.X,
        _el0.Y * m._el0.Y + _el1.Y * m._el1.Y + _el2.Y * m._el2.Y,
        _el0.Y * m._el0.Z + _el1.Y * m._el1.Z + _el2.Y * m._el2.Z,
        _el0.Z * m._el0.X + _el1.Z * m._el1.X + _el2.Z * m._el2.X,
        _el0.Z * m._el0.Y + _el1.Z * m._el1.Y + _el2.Z * m._el2.Y,
        _el0.Z * m._el0.Z + _el1.Z * m._el1.Z + _el2.Z * m._el2.Z);

        }

        public IndexedBasisMatrix TransposeTimes(ref IndexedBasisMatrix m)
        {
            return new IndexedBasisMatrix(
        _el0.X * m._el0.X + _el1.X * m._el1.X + _el2.X * m._el2.X,
        _el0.X * m._el0.Y + _el1.X * m._el1.Y + _el2.X * m._el2.Y,
        _el0.X * m._el0.Z + _el1.X * m._el1.Z + _el2.X * m._el2.Z,
        _el0.Y * m._el0.X + _el1.Y * m._el1.X + _el2.Y * m._el2.X,
        _el0.Y * m._el0.Y + _el1.Y * m._el1.Y + _el2.Y * m._el2.Y,
        _el0.Y * m._el0.Z + _el1.Y * m._el1.Z + _el2.Y * m._el2.Z,
        _el0.Z * m._el0.X + _el1.Z * m._el1.X + _el2.Z * m._el2.X,
        _el0.Z * m._el0.Y + _el1.Z * m._el1.Y + _el2.Z * m._el2.Y,
        _el0.Z * m._el0.Z + _el1.Z * m._el1.Z + _el2.Z * m._el2.Z);

        }

        public IndexedBasisMatrix TimesTranspose(IndexedBasisMatrix m)
        {
            return new IndexedBasisMatrix(
                _el0.Dot(m._el0), _el0.Dot(m._el1), _el0.Dot(m._el2),
                _el1.Dot(m._el0), _el1.Dot(m._el1), _el1.Dot(m._el2),
                _el2.Dot(m._el0), _el2.Dot(m._el1), _el2.Dot(m._el2));
        }


        public IndexedBasisMatrix Transpose()
        {
            return new IndexedBasisMatrix(_el0.X, _el1.X, _el2.X,
                _el0.Y, _el1.Y, _el2.Y,
                _el0.Z, _el1.Z, _el2.Z);
        }



        public IndexedBasisMatrix Absolute()
        {
            return new IndexedBasisMatrix(_el0.Abs(), _el1.Abs(), _el2.Abs());
        }

        public IndexedQuaternion GetRotation()
        {
            float trace = _el0.X + _el1.Y + _el2.Z;
            IndexedVector3 temp = new IndexedVector3();
            float temp2 = 0f;
            if (trace > 0.0f)
            {
                float s = (float)Math.Sqrt(trace + 1.0f);
                temp2 = (s * 0.5f);
                s = 0.5f / s;

                temp[0] = ((_el2.Y - _el1.Z) * s);
                temp[1] = ((_el0.Z - _el2.X) * s);
                temp[2] = ((_el1.X - _el0.Y) * s);
            }
            else
            {
                int i = _el0.X < _el1.Y ?
                    (_el1.Y < _el2.Z ? 2 : 1) :
                    (_el0.X < _el2.Z ? 2 : 0);
                int j = (i + 1) % 3;
                int k = (i + 2) % 3;

                float s = (float)Math.Sqrt(this[i][i] - this[j][j] - this[k][k] + 1.0f);
                temp[i] = s * 0.5f;
                s = 0.5f / s;

                temp2 = (this[k][j] - this[j][k]) * s;
                temp[j] = (this[j][i] + this[i][j]) * s;
                temp[k] = (this[k][i] + this[i][k]) * s;
            }
            return new IndexedQuaternion(temp[0], temp[1], temp[2], temp2);

        }



        public void SetRotation(IndexedQuaternion q)
        {
            float d = q.LengthSquared();
            Debug.Assert(d != 0.0f);
            float s = 2.0f / d;
            float xs = q.X * s, ys = q.Y * s, zs = q.Z * s;
            float wx = q.W * xs, wy = q.W * ys, wz = q.W * zs;
            float xx = q.X * xs, xy = q.X * ys, xz = q.X * zs;
            float yy = q.Y * ys, yz = q.Y * zs, zz = q.Z * zs;
            SetValue(1.0f - (yy + zz), xy - wz, xz + wy,
                xy + wz, 1.0f - (xx + zz), yz - wx,
                xz - wy, yz + wx, 1.0f - (xx + yy));
        }

        public void SetRotation(ref IndexedQuaternion q)
        {
            float d = q.LengthSquared();
            Debug.Assert(d != 0.0f);
            float s = 2.0f / d;
            float xs = q.X * s, ys = q.Y * s, zs = q.Z * s;
            float wx = q.W * xs, wy = q.W * ys, wz = q.W * zs;
            float xx = q.X * xs, xy = q.X * ys, xz = q.X * zs;
            float yy = q.Y * ys, yz = q.Y * zs, zz = q.Z * zs;
            SetValue(1.0f - (yy + zz), xy - wz, xz + wy,
                xy + wz, 1.0f - (xx + zz), yz - wx,
                xz - wy, yz + wx, 1.0f - (xx + yy));
        }



        /**@brief diagonalizes this matrix by the Jacobi method.
       * @param rot stores the rotation from the coordinate system in which the matrix is diagonal to the original
       * coordinate system, i.e., old_this = rot * new_this * rot^T. 
       * @param threshold See iteration
       * @param iteration The iteration stops when all off-diagonal elements are less than the threshold multiplied 
       * by the sum of the absolute values of the diagonal, or when maxSteps have been executed. 
       * 
       * Note that this matrix is assumed to be symmetric. 
       */
        public void Diagonalize(out IndexedMatrix rot, float threshold, int maxSteps)
        {

            rot = IndexedMatrix.Identity;
            for (int step = maxSteps; step > 0; step--)
            {
                // find off-diagonal element [p][q] with largest magnitude
                int p = 0;
                int q = 1;
                int r = 2;
                float max = Math.Abs(this[0][1]);
                float v = Math.Abs(this[0][2]);
                if (v > max)
                {
                    q = 2;
                    r = 1;
                    max = v;
                }
                v = Math.Abs(this[1][2]);
                if (v > max)
                {
                    p = 1;
                    q = 2;
                    r = 0;
                    max = v;
                }

                float t = threshold * (Math.Abs(this[0][0]) + Math.Abs(this[1][1]) + Math.Abs(this[2][2]));
                if (max <= t)
                {
                    if (max <= MathUtil.SIMD_EPSILON * t)
                    {
                        return;
                    }
                    step = 1;
                }

                // compute Jacobi rotation J which leads to a zero for element [p][q] 
                float mpq = this[p][q];
                float theta = (this[q][q] - this[p][p]) / (2 * mpq);
                float theta2 = theta * theta;
                float cos;
                float sin;
                if (theta2 * theta2 < (10.0f / MathUtil.SIMD_EPSILON))
                {
                    t = (theta >= 0.0f) ? (1.0f / (float)(theta + (float)Math.Sqrt(1 + theta2)))
                        : (1 / (theta - (float)Math.Sqrt(1 + theta2)));
                    cos = 1.0f / (float)Math.Sqrt(1 + t * t);
                    sin = cos * t;
                }
                else
                {
                    // approximation for large theta-value, i.e., a nearly diagonal matrix
                    t = 1 / (theta * (2 + 0.5f / theta2));
                    cos = 1 - 0.5f * t * t;
                    sin = cos * t;
                }

                // apply rotation to matrix (this = J^T * this * J)
                this[p, q] = 0;
                this[q, p] = 0;
                this[p, p] -= t * mpq;
                this[q, q] += t * mpq;
                float mrp = this[r][p];
                float mrq = this[r][q];
                this[r, p] = this[p, r] = cos * mrp - sin * mrq;
                this[r, q] = this[q, r] = cos * mrq + sin * mrp;

                // apply rotation to rot (rot = rot * J)
                for (int i = 0; i < 3; i++)
                {
                    mrp = this[i, p];
                    mrq = this[i, q];
                    this[i, p] = cos * mrp - sin * mrq;
                    this[i, q] = cos * mrq + sin * mrp;
                }
            }
        }


        //public void SetNewForward(IndexedVector3 forward)
        //{
        //    forward.Normalize();
        //    // Re-calculate Right
        //    IndexedVector3 right = Vector3.Cross(forward, this[0]);

        //    // The same instability may cause the 3 orientation vectors may
        //    // also diverge. Either the Up or Direction vector needs to be
        //    // re-computed with a cross product to ensure orthagonality
        //    IndexedVector3 up = Vector3.Cross(right, forward);
        //    this[0] = right;
        //    this[1] = up;
        //    this[2] = forward;
        //   }

        public static IndexedBasisMatrix CreateRotationX(float radians)
        {
            float num1 = (float)Math.Cos((double)radians);
            float num2 = (float)Math.Sin((double)radians);
            IndexedBasisMatrix _basis;
            _basis._el0 = new IndexedVector3(1, 0, 0);
            _basis._el1 = new IndexedVector3(0, num1, num2);
            _basis._el2 = new IndexedVector3(0, -num2, num1);
            return _basis;
        }

        public static void CreateRotationX(float radians, out IndexedBasisMatrix _basis)
        {
            float num1 = (float)Math.Cos((double)radians);
            float num2 = (float)Math.Sin((double)radians);
            _basis._el0 = new IndexedVector3(1, 0, 0);
            _basis._el1 = new IndexedVector3(0, num1, num2);
            _basis._el2 = new IndexedVector3(0, -num2, num1);
        }

        public static IndexedBasisMatrix CreateRotationZ(float radians)
        {
            float num1 = (float)Math.Cos((double)radians);
            float num2 = (float)Math.Sin((double)radians);
            IndexedBasisMatrix _basis;
            _basis._el0 = new IndexedVector3(num1, num2, 0);
            _basis._el1 = new IndexedVector3(-num2, num1, 0);
            _basis._el2 = new IndexedVector3(0, 0, 1);
            return _basis;
        }

        public static void CreateRotationZ(float radians, out IndexedBasisMatrix _basis)
        {
            float num1 = (float)Math.Cos((double)radians);
            float num2 = (float)Math.Sin((double)radians);
            _basis._el0 = new IndexedVector3(num1, num2, 0);
            _basis._el1 = new IndexedVector3(-num2, num1, 0);
            _basis._el2 = new IndexedVector3(0, 0, 1);
        }




        public static IndexedBasisMatrix CreateRotationY(float radians)
        {
            float num1 = (float)Math.Cos((double)radians);
            float num2 = (float)Math.Sin((double)radians);
            IndexedBasisMatrix ibm = new IndexedBasisMatrix(num1,0.0f, -num2,0.0f,1f,0.0f,num2,0.0f,num1);
            return ibm;
        }

        public static IndexedBasisMatrix CreateFromAxisAngle(IndexedVector3 axis, float angle)
        {
            float num1 = axis.X;
            float num2 = axis.Y;
            float num3 = axis.Z;
            float num4 = (float)Math.Sin((double)angle);
            float num5 = (float)Math.Cos((double)angle);
            float num6 = num1 * num1;
            float num7 = num2 * num2;
            float num8 = num3 * num3;
            float num9 = num1 * num2;
            float num10 = num1 * num3;
            float num11 = num2 * num3;
            IndexedBasisMatrix ibm = new IndexedBasisMatrix(num6 + num5 * (1f - num6),
                (float)((double)num9 - (double)num5 * (double)num9 + (double)num4 * (double)num3),
                (float)((double)num10 - (double)num5 * (double)num10 - (double)num4 * (double)num2),
                (float)((double)num9 - (double)num5 * (double)num9 - (double)num4 * (double)num3),
                num7 + num5 * (1f - num7),
                (float)((double)num11 - (double)num5 * (double)num11 + (double)num4 * (double)num1),
                (float)((double)num10 - (double)num5 * (double)num10 + (double)num4 * (double)num2),
                (float)((double)num11 - (double)num5 * (double)num11 - (double)num4 * (double)num1),
                num8 + num5 * (1f - num8));
            return ibm;
        }

        public IndexedVector3 Right
        {
            get { return this[0]; }
            set{this[0] = value;}
        }

        public IndexedVector3 Left
        {
            get { return -this[0]; }
            set {this[0] = -value;}
        }

        public IndexedVector3 Up
        {
            get { return this[1]; }
            set { this[1] = value;}
        }

        public IndexedVector3 Down
        {
            get { return -this[1]; }
            set { this[1] = -value; }
        }

        public IndexedVector3 Forward
        {
            get { return -this[2]; }
            set { this[2] = -value; }
        }

        public IndexedVector3 Backward
        {
            get { return this[2]; }
            set { this[2] = value; }
        }

        public void GetOpenGLMatrix(out IndexedVector3 v1,out IndexedVector3 v2,out IndexedVector3 v3)
        {
		    v1.X  = _el0.X; 
		    v1.Y  = _el1.X;
		    v1.Z  = _el2.X;
		    //m[3]  = btScalar(0.0); 
		    v2.X  = _el0.Y;
		    v2.Y  = _el1.Y;
		    v2.Z  = _el2.Y;
		    //m[7]  = btScalar(0.0); 
		    v3.X  = _el0.Z; 
		    v3.Y  = _el1.Z;
		    v3.Z = _el2.Z;
		    //m[11] = btScalar(0.0); 

        }
        public void SetOpenGLMatrix(IndexedVector3 v1, IndexedVector3 v2, IndexedVector3 v3)
        {
            SetOpenGLMatrix(ref v1, ref v2, ref v3);
        }

        public void SetOpenGLMatrix(ref IndexedVector3 v1,ref IndexedVector3 v2,ref IndexedVector3 v3)
        {
		    _el0 = new IndexedVector3(v1.X,v2.X,v3.X);
            _el1 = new IndexedVector3(v1.Y, v2.Y, v3.Y);
            _el2 = new IndexedVector3(v1.Z, v2.Z, v3.Z);
        }


        public IndexedVector3 _el0;
        public IndexedVector3 _el1;
        public IndexedVector3 _el2;

    }
}
