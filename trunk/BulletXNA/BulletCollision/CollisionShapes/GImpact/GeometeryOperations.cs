using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletCollision.CollisionShapes.GImpact
{
    public static class GeometeryOperations
    {
        public static void bt_edge_plane(ref Vector3  e1,ref Vector3  e2, ref Vector3 normal, out Vector4 plane)
        {
	        Vector3 planenormal = Vector3.Cross(e2-e1,normal);
	        planenormal.Normalize();
	        plane = new Vector4(planenormal,Vector3.Dot(e2,planenormal));
        }

    }
}
