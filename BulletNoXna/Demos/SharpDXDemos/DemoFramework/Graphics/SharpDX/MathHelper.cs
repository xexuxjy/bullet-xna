using SharpDX;

namespace DemoFramework.SharpDX
{
	static class MathHelper
    {
        public static Matrix Convert(ref BulletXNA.LinearMath.Matrix m)
        {
            return new Matrix(
                m._basis._el0.X, m._basis._el1.X, m._basis._el2.X, 0,
                m._basis._el0.Y, m._basis._el1.Y, m._basis._el2.Y, 0,
                m._basis._el0.Z, m._basis._el1.Z, m._basis._el2.Z, 0,
                m.Translation.X, m.Translation.Y, m.Translation.Z, 1);
        }

        public static BulletXNA.LinearMath.Matrix Convert(ref Matrix m)
        {
            return new BulletXNA.LinearMath.Matrix(
                m.M11, m.M12, m.M13,
                m.M21, m.M22, m.M23,
                m.M31, m.M32, m.M33,
                m.M41, m.M42, m.M43);
        }

        public static Vector3 Convert(BulletXNA.LinearMath.Vector3 v)
        {
            return new Vector3(v.X, v.Y, v.Z);
        }
    }
}
