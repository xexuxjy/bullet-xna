using Microsoft.Xna.Framework;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
    public static class GImpactMassUtil
    {

        public static IndexedVector3 GimGetPointInertia(ref IndexedVector3 point, float mass)
        {
            float x2 = point.X * point.X;
            float y2 = point.Y * point.Y;
            float z2 = point.Z * point.Z;
            return new IndexedVector3(mass * (y2 + z2), mass * (x2 + z2), mass * (x2 + y2));
        }

        public static IndexedVector3 GimInertiaAddTransformed(ref IndexedVector3 source_inertia, ref IndexedVector3 added_inertia, ref IndexedMatrix transform)
        {
            IndexedBasisMatrix rotatedTensor = transform._basis.Scaled(added_inertia) * transform._basis.Transpose();

            float x2 = transform._origin[0];
            x2 *= x2;
            float y2 = transform._origin[1];
            y2 *= y2;
            float z2 = transform._origin[2];
            z2 *= z2;

            float ix = rotatedTensor[0,0] * (y2 + z2);
            float iy = rotatedTensor[1,1] * (x2 + z2);
            float iz = rotatedTensor[2,2] * (x2 + y2);

            return new IndexedVector3(source_inertia[0] + ix, source_inertia[1] + iy, source_inertia[2] + iz);

        }
    }
}
