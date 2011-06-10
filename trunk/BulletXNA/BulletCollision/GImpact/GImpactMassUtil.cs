using Microsoft.Xna.Framework;

namespace BulletXNA.BulletCollision
{
    public static class GImpactMassUtil
    {

        public static Vector3 GimGetPointInertia(ref Vector3 point, float mass)
        {
            float x2 = point.X * point.X;
            float y2 = point.Y * point.Y;
            float z2 = point.Z * point.Z;
            return new Vector3(mass * (y2 + z2), mass * (x2 + z2), mass * (x2 + y2));
        }

        public static Vector3 GimInertiaAddTransformed(ref Vector3 source_inertia, ref Vector3 added_inertia, ref Matrix transform)
        {
            //    btMatrix3x3  rotatedTensor = transform.getBasis().scaled(added_inertia) * transform.getBasis().transpose();

            //    btScalar x2 = transform.getOrigin()[0];
            //    x2*= x2;
            //    btScalar y2 = transform.getOrigin()[1];
            //    y2*= y2;
            //    btScalar z2 = transform.getOrigin()[2];
            //    z2*= z2;

            //    btScalar ix = rotatedTensor[0][0]*(y2+z2);
            //    btScalar iy = rotatedTensor[1][1]*(x2+z2);
            //    btScalar iz = rotatedTensor[2][2]*(x2+y2);

            //    return btVector3(source_inertia[0]+ix,source_inertia[1]+iy,source_inertia[2] + iz);
            return Vector3.Zero;
        }
    }
}
