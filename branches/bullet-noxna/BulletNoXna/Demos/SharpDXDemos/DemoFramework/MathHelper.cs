namespace DemoFramework
{
    public class MathHelper
    {
        public static BulletXNA.LinearMath.Vector3 Vector3(SharpDX.Vector3 vector)
        {
            return new BulletXNA.LinearMath.Vector3(vector.X, vector.Y, vector.Z);
        }

        public static SharpDX.Vector3 Vector3(BulletXNA.LinearMath.Vector3 vector)
        {
            return new SharpDX.Vector3(vector.X, vector.Y, vector.Z);
        }

        public static BulletXNA.LinearMath.Vector4 Vector4(SharpDX.Vector4 vector)
        {
            return new BulletXNA.LinearMath.Vector4(vector.X, vector.Y, vector.Z, vector.W);
        }

        public static SharpDX.Vector4 Vector4(BulletXNA.LinearMath.Vector4 vector)
        {
            return new SharpDX.Vector4(vector.X, vector.Y, vector.Z, vector.W);
        }

        public static BulletXNA.LinearMath.Matrix Matrix(SharpDX.Matrix vector)
        {
            return new BulletXNA.LinearMath.Matrix(
                vector.M11, vector.M12, vector.M13,
                vector.M21, vector.M22, vector.M23,
                vector.M31, vector.M32, vector.M33,
                vector.M41, vector.M42, vector.M43);
        }

        public static SharpDX.Matrix Matrix(BulletXNA.LinearMath.Matrix vector)
        {
            return new SharpDX.Matrix(
                vector._basis._Row0.X, vector._basis._Row1.X, vector._basis._Row2.X, 0,
                vector._basis._Row0.Y, vector._basis._Row1.Y, vector._basis._Row2.Y, 0,
                vector._basis._Row0.Z, vector._basis._Row1.Z, vector._basis._Row2.Z, 0,
                vector.Translation.X, vector.Translation.Y, vector.Translation.Z, 1);
        }
    }
}
