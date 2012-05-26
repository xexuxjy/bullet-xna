using System;

namespace BulletXNA.LinearMath
{
    public struct Matrix : IEquatable<Matrix>
    {
        private static Matrix _identity = new Matrix(1f, 0.0f, 0.0f, 0.0f, 1f, 0.0f,  0.0f, 0.0f, 1f,  0.0f, 0.0f, 0.0f);
        public static Matrix Identity
        {
            get
            {
                return Matrix._identity;
            }
        }

        static Matrix()
        {
        }

        public Matrix(float m11, float m12, float m13, float m21, float m22, float m23,  float m31, float m32, float m33,  float m41, float m42, float m43)
        {
            _basis = new IndexedBasisMatrix(m11, m12, m13, m21, m22, m23, m31, m32, m33);
            Translation = new Vector3(m41, m42, m43);

        }

        public Matrix(IndexedBasisMatrix basis, Vector3 origin)
        {
            _basis = basis;
            Translation = origin;
        }

        public static Matrix CreateLookAt(Vector3 cameraPosition, Vector3 cameraTarget, Vector3 cameraUpVector)
        {
            Vector3 vector3_1 = Vector3.Normalize(cameraTarget - cameraPosition);
            Vector3 vector3_2 = Vector3.Normalize(Vector3.Cross(cameraUpVector, vector3_1));
            Vector3 vector1 = Vector3.Cross(vector3_1, vector3_2);
            Matrix matrix = Matrix.Identity;
            //matrix._basis = new IndexedBasisMatrix(vector3_2.X, vector1.X, vector3_1.X, vector3_2.Y, vector1.Y, vector3_1.Y, vector3_2.Z, vector1.Z, vector3_1.Z);
            matrix._basis = new IndexedBasisMatrix(ref vector3_2, ref vector1, ref vector3_1).Transpose();

            matrix.Translation = new Vector3(-Vector3.Dot(vector3_2, cameraPosition),
            -Vector3.Dot(vector1, cameraPosition),
            -Vector3.Dot(vector3_1, cameraPosition));
            return matrix;
        }

        public static Matrix CreatePerspectiveFieldOfView(float fieldOfView, float aspectRatio, float nearPlaneDistance, float farPlaneDistance)
        {
        //    if ((double)fieldOfView <= 0.0 || (double)fieldOfView >= 3.14159274101257)
        //        throw new ArgumentOutOfRangeException("fieldOfView", string.Format((IFormatProvider)CultureInfo.CurrentCulture, FrameworkResources.OutRangeFieldOfView, new object[1]
        //{
        //  (object) "fieldOfView"
        //}));
        //    else if ((double)nearPlaneDistance <= 0.0)
        //        throw new ArgumentOutOfRangeException("nearPlaneDistance", string.Format((IFormatProvider)CultureInfo.CurrentCulture, FrameworkResources.NegativePlaneDistance, new object[1]
        //{
        //  (object) "nearPlaneDistance"
        //}));
        //    else if ((double)farPlaneDistance <= 0.0)
        //    {
        //        throw new ArgumentOutOfRangeException("farPlaneDistance", string.Format((IFormatProvider)CultureInfo.CurrentCulture, FrameworkResources.NegativePlaneDistance, new object[1]
        //{
        //  (object) "farPlaneDistance"
        //}));
        //    }
        //    else
            {
                //if ((double)nearPlaneDistance >= (double)farPlaneDistance)
                //    throw new ArgumentOutOfRangeException("nearPlaneDistance", FrameworkResources.OppositePlanes);
                float num1 = 1f / (float)Math.Tan((double)fieldOfView * 0.5);
                float num2 = num1 / aspectRatio;
                Matrix matrix = Matrix.Identity ;
                matrix._basis = new IndexedBasisMatrix(num2, 0, 0, 0, num1, 0, 0, 0, farPlaneDistance / (nearPlaneDistance - farPlaneDistance));
                matrix.Translation = new Vector3(0,0,(float)((double)nearPlaneDistance * (double)farPlaneDistance / ((double)nearPlaneDistance - (double)farPlaneDistance)));
                
                return matrix;
            }
        }


        public static bool operator ==(Matrix matrix1, Matrix matrix2)
        {
            return matrix1._basis == matrix2._basis &&
                    matrix1.Translation == matrix2.Translation; 
        }

        public static bool operator !=(Matrix matrix1, Matrix matrix2)
        {
            return matrix1._basis != matrix2._basis ||
                    matrix1.Translation != matrix2.Translation;
        }

	    public static Vector3 operator *(Matrix matrix1,Vector3 v)
	    {
            //return new Vector3(matrix1._basis[0].Dot(ref v) + matrix1.Translation.X, 
            //                           matrix1._basis[1].Dot(ref v) + matrix1.Translation.Y,
            //                            matrix1._basis[2].Dot(ref v) + matrix1.Translation.Z);
            return new Vector3(matrix1._basis._Row0.Dot(ref v) + matrix1.Translation.X,
                                                   matrix1._basis._Row1.Dot(ref v) + matrix1.Translation.Y,
                                                    matrix1._basis._Row2.Dot(ref v) + matrix1.Translation.Z);
        }

        //public static Vector3 operator *(Vector3 v,Matrix matrix1)
        //{
        //    return new Vector3(matrix1._basis[0].Dot(ref v) + matrix1.Translation.X,
        //                               matrix1._basis[1].Dot(ref v) + matrix1.Translation.Y,
        //                                matrix1._basis[2].Dot(ref v) + matrix1.Translation.Z);
        //}

        public static Matrix operator *(Matrix matrix1, Matrix matrix2)
        {
            Matrix IndexedMatrix;
            IndexedMatrix._basis = matrix1._basis * matrix2._basis;
            IndexedMatrix.Translation = matrix1 * matrix2.Translation;

            return IndexedMatrix;
        }

        //public static Matrix operator *(Matrix matrix1, float scaleFactor)
        //{
        //    float num = scaleFactor;
        //    Matrix Matrix;
        //    Matrix._basis._Row0 = matrix1._Row0  * scaleFactor;
        //    Matrix._basis._Row1 = matrix1._Row1 * scaleFactor;
        //    Matrix._basis._Row3 = matrix1._Row3 * scaleFactor;
        //    Matrix._basis.Row3 = matrix1.Row3 * scaleFactor;
        //    return Matrix;
        //}

        //public static Matrix operator *(float scaleFactor, Matrix matrix1)
        //{
        //    Matrix Matrix;
        //    Matrix._basis._Row0 = matrix1._basis._Row0 * scaleFactor;
        //    Matrix._basis._Row1 = matrix1._basis._Row1 * scaleFactor;
        //    Matrix._basis._Row2 = matrix1._basis._Row2 * scaleFactor;
        //    Matrix.Translation = matrix1.Translation* scaleFactor;
        //    return Matrix;
        //}

        public static Matrix operator /(Matrix matrix1, Matrix matrix2)
        {
            Matrix IndexedMatrix;
            IndexedMatrix._basis._Row0 = matrix1._basis._Row0 / matrix2._basis._Row0;
            IndexedMatrix._basis._Row1 = matrix1._basis._Row1 / matrix2._basis._Row1;
            IndexedMatrix._basis._Row2 = matrix1._basis._Row2 / matrix2._basis._Row2;
            IndexedMatrix.Translation = matrix1.Translation / matrix2.Translation;
            return IndexedMatrix;
        }

        public static Matrix operator /(Matrix matrix1, float divider)
        {
            float num = 1f / divider;
            Matrix IndexedMatrix;
            IndexedMatrix._basis._Row0 = matrix1._basis._Row0 * num;
            IndexedMatrix._basis._Row1 = matrix1._basis._Row1 * num;
            IndexedMatrix._basis._Row2 = matrix1._basis._Row2 * num;
            IndexedMatrix.Translation = matrix1.Translation * num;
            return IndexedMatrix;
        }


        public static Matrix CreateTranslation(Vector3 position)
        {
            Matrix IndexedMatrix;
            IndexedMatrix._basis._Row0 = new Vector3(1, 0, 0);
            IndexedMatrix._basis._Row1 = new Vector3(0, 1, 0);
            IndexedMatrix._basis._Row2 = new Vector3(0, 0, 1);
            IndexedMatrix.Translation = position;            
            return IndexedMatrix;
        }

        public static void CreateTranslation(ref Vector3 position, out Matrix result)
        {
            result._basis._Row0 = new Vector3(1, 0, 0);
            result._basis._Row1 = new Vector3(0, 1, 0);
            result._basis._Row2 = new Vector3(0, 0, 1);
            result.Translation =  position;
        }

        public static Matrix CreateTranslation(float xPosition, float yPosition, float zPosition)
        {
            Matrix IndexedMatrix;
            IndexedMatrix._basis._Row0 = new Vector3(1, 0, 0);
            IndexedMatrix._basis._Row1 = new Vector3(0, 1, 0);
            IndexedMatrix._basis._Row2 = new Vector3(0, 0, 1);
            IndexedMatrix.Translation = new Vector3(xPosition, yPosition, zPosition);
            return IndexedMatrix;
        }

        public static void CreateTranslation(float xPosition, float yPosition, float zPosition, out Matrix result)
        {
            result._basis._Row0 = new Vector3(1, 0, 0);
            result._basis._Row1 = new Vector3(0, 1, 0);
            result._basis._Row2 = new Vector3(0, 0, 1);
            result.Translation = new Vector3(xPosition, yPosition, zPosition);
        }

        public static Matrix CreateScale(float xScale, float yScale, float zScale)
        {
            Matrix IndexedMatrix;
            IndexedMatrix._basis._Row0 = new Vector3(xScale, 0, 0);
            IndexedMatrix._basis._Row1 = new Vector3(0, yScale, 0);
            IndexedMatrix._basis._Row2 = new Vector3(0, 0, zScale);
            IndexedMatrix.Translation = new Vector3(0, 0, 0);
            return IndexedMatrix;
        }

        public static void CreateScale(float xScale, float yScale, float zScale, out Matrix result)
        {
            result._basis._Row0 = new Vector3(xScale, 0, 0);
            result._basis._Row1 = new Vector3(0, yScale, 0);
            result._basis._Row2 = new Vector3(0, 0, zScale);
            result.Translation = new Vector3(0,0,0);
        }

        public static Matrix CreateScale(Vector3 scales)
        {
            Matrix IndexedMatrix;
            IndexedMatrix._basis._Row0 = new Vector3(scales.X, 0, 0);
            IndexedMatrix._basis._Row1 = new Vector3(0, scales.Y, 0);
            IndexedMatrix._basis._Row2 = new Vector3(0, 0, scales.Z);
            IndexedMatrix.Translation = new Vector3(0, 0, 0);
            return IndexedMatrix;
        }

        public static void CreateScale(ref Vector3 scales, out Matrix result)
        {
            result._basis._Row0 = new Vector3(scales.X, 0, 0);
            result._basis._Row1 = new Vector3(0, scales.Y, 0);
            result._basis._Row2 = new Vector3(0, 0, scales.Z);
            result.Translation = new Vector3(0, 0, 0);
        }

        public static Matrix CreateScale(float scale)
        {
            Matrix IndexedMatrix;
            IndexedMatrix._basis._Row0 = new Vector3(scale, 0, 0);
            IndexedMatrix._basis._Row1 = new Vector3(0, scale, 0);
            IndexedMatrix._basis._Row2 = new Vector3(0, 0, scale);
            IndexedMatrix.Translation = new Vector3(0, 0, 0);
            return IndexedMatrix;
        }

        public static void CreateScale(float scale, out Matrix result)
        {
            result._basis._Row0 = new Vector3(scale, 0, 0);
            result._basis._Row1 = new Vector3(0, scale, 0);
            result._basis._Row2 = new Vector3(0, 0, scale);
            result.Translation = new Vector3(0, 0, 0);
        }

        public static Matrix CreateRotationX(float radians)
        {
            float num1 = (float)Math.Cos((double)radians);
            float num2 = (float)Math.Sin((double)radians);
            Matrix IndexedMatrix;
            IndexedMatrix._basis._Row0 = new Vector3(1, 0, 0);
            IndexedMatrix._basis._Row1 = new Vector3(0, num1, num2);
            IndexedMatrix._basis._Row2 = new Vector3(0, -num2, num1);
            IndexedMatrix.Translation = new Vector3(0, 0, 0);
            
            return IndexedMatrix;
        }

        public static void CreateRotationX(float radians, out Matrix result)
        {
            float num1 = (float)Math.Cos((double)radians);
            float num2 = (float)Math.Sin((double)radians);
            result._basis._Row0 = new Vector3(1, 0, 0);
            result._basis._Row1 = new Vector3(0, num1, num2);
            result._basis._Row2 = new Vector3(0, -num2, num1);
            result.Translation = new Vector3(0, 0, 0);
        }

        public static Matrix CreateRotationY(float radians)
        {
            float num1 = (float)Math.Cos((double)radians);
            float num2 = (float)Math.Sin((double)radians);
            Matrix IndexedMatrix;
            IndexedMatrix._basis._Row0 = new Vector3(num1, 0, -num2);
            IndexedMatrix._basis._Row1 = new Vector3(0, 1, 0);
            IndexedMatrix._basis._Row2 = new Vector3(num2, -0, num1);
            IndexedMatrix.Translation = new Vector3(0, 0, 0);
            return IndexedMatrix;
        }

        public static void CreateRotationY(float radians, out Matrix result)
        {
            float num1 = (float)Math.Cos((double)radians);
            float num2 = (float)Math.Sin((double)radians);
            result._basis._Row0 = new Vector3(num1, 0, -num2);
            result._basis._Row1 = new Vector3(0, 1, 0);
            result._basis._Row2 = new Vector3(num2, -0, num1);
            result.Translation = new Vector3(0, 0, 0);
        }

        public static Matrix CreateRotationZ(float radians)
        {
            float num1 = (float)Math.Cos((double)radians);
            float num2 = (float)Math.Sin((double)radians);
            Matrix IndexedMatrix;
            IndexedMatrix._basis._Row0 = new Vector3(num1, num2, 0);
            IndexedMatrix._basis._Row1 = new Vector3(-num2, num1, 0);
            IndexedMatrix._basis._Row2 = new Vector3(0,0,1);
            IndexedMatrix.Translation = new Vector3(0, 0, 0);
            return IndexedMatrix;
        }

        public static void CreateRotationZ(float radians, out Matrix result)
        {
            float num1 = (float)Math.Cos((double)radians);
            float num2 = (float)Math.Sin((double)radians);
            result._basis._Row0 = new Vector3(num1, num2, 0);
            result._basis._Row1 = new Vector3(-num2, num1, 0);
            result._basis._Row2 = new Vector3(0, 0, 1);
            result.Translation = new Vector3(0, 0, 0);
        }

        public static Matrix CreateRotationAxis(Vector3 axis, float angle)
        {
            float x = axis.X;
            float y = axis.Y;
            float z = axis.Z;
            float cos = (float)Math.Cos(angle);
            float sin = (float)Math.Sin(angle);
            float xx = x * x;
            float yy = y * y;
            float zz = z * z;
            float xy = x * y;
            float xz = x * z;
            float yz = y * z;

            Matrix result = new Matrix(
                xx + (cos * (1.0f - xx)),
                (xy - (cos * xy)) + (sin * z),
                (xz - (cos * xz)) - (sin * y),
                (xy - (cos * xy)) - (sin * z),
                yy + (cos * (1.0f - yy)),
                (yz - (cos * yz)) + (sin * x),
                (xz - (cos * xz)) + (sin * y),
                (yz - (cos * yz)) - (sin * x),
                zz + (cos * (1.0f - zz)),
                0, 0, 0);

            return result;
        }

        public bool Equals(Matrix other)
        {
            return _basis.Equals(other._basis) && Translation.Equals(other.Translation);
        }

        public override bool Equals(object obj)
        {
            bool flag = false;
            if (obj is Matrix)
                flag = this.Equals((Matrix)obj);
            return flag;
        }

        public override int GetHashCode()
        {
            return this._basis.GetHashCode() + this.Translation.GetHashCode();
        }

        public Matrix Inverse()
	    { 
		    IndexedBasisMatrix inv = _basis.Transpose();
            return new Matrix(inv, inv * -Translation);
	    }

        public Vector3 InvXform(Vector3 inVec)
        {
            Vector3 v = inVec - Translation;
            return (_basis.Transpose() * v);
        }

        public Vector3 InvXform(ref Vector3 inVec)
        {
	        Vector3 v = inVec - Translation;
	        return (_basis.Transpose() * v);
        }

        public Matrix InverseTimes(ref Matrix t)
        {
            Vector3 v = t.Translation - Translation;
            return new Matrix(_basis.TransposeTimes(t._basis),
			        v * _basis);
        }

	    public Quaternion GetRotation() 
        { 
		    return _basis.GetRotation();
	    }

        public static Matrix CreateFromQuaternion(Quaternion q)
        {
            Matrix i = new Matrix();
            i._basis.SetRotation(ref q);
            return i;
        }

        public static Matrix CreateFromQuaternion(ref Quaternion q)
        {
            Matrix i = new Matrix();
            i._basis.SetRotation(ref q);
            return i;
        }

        public Vector3 Left
        {
            get { return _basis.Left; }
        }

        public Vector3 Right
        {
            get { return _basis.Right; }
        }

        public Vector3 Up
        {
            get
            {
                return _basis.Up;
            }
        }

        public Vector3 Down
        {
            get
            {
                return _basis.Down;
            }
        }

        public Vector3 Forward
        {
            get
            {
                return _basis.Forward;
            }
        }

        public Vector3 Backward
        {
            get
            {
                return _basis.Backward;
            }
        }



        public IndexedBasisMatrix _basis;
        public Vector3 Translation;
    
    }
}
