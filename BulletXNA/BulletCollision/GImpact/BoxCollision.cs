/*
 * 
 * C# / XNA  port of Bullet (c) 2011 Mark Neale <xexuxjy@hotmail.com>
 *
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision.GImpact
{
    public enum BT_PLANE_INTERSECTION_TYPE
    {
        BT_CONST_BACK_PLANE = 0,
        BT_CONST_COLLIDE_PLANE,
        BT_CONST_FRONT_PLANE
    }

    public class BoxCollision
    {



        ///Swap numbers
        //#define BT_SWAP_NUMBERS(a,b){ \
        //    a = a+b; \
        //    b = a-b; \
        //    a = a-b; \
        //}\


        //#define BoxCollision.BT_MAX(a,b) (a<b?b:a)
        //#define BoxCollision.BT_MIN(a,b) (a>b?b:a)

        //#define BT_GREATER(x, y)	Math.Abs(x) > (y)

        //#define BoxCollision.BT_MAX3(a,b,c) BoxCollision.BT_MAX(a,BT_MAX(b,c))
        //#define BoxCollision.BT_MIN3(a,b,c) BoxCollision.BT_MIN(a,BT_MIN(b,c))

        public static bool BT_GREATER(float x, float y)
        {
            return Math.Abs(x) > y;
        }

        public static float BT_MAX(float a, float b)
        {
            return Math.Max(a, b);
        }

        public static float BT_MIN(float a, float b)
        {
            return Math.Min(a, b);
        }

        public static float BT_MAX3(float a, float b, float c)
        {
            return Math.Max(a, Math.Max(b, c));
        }

        public static float BT_MIN3(float a, float b, float c)
        {
            return Math.Min(a, Math.Min(b, c));
        }



        // bool test_cross_edge_box(
        //	 ref Vector3 edge,
        //	 ref Vector3 absolute_edge,
        //	 ref Vector3 pointa,
        //	 ref Vector3 pointb,  ref Vector3 extend,
        //	int dir_index0,
        //	int dir_index1
        //	int component_index0,
        //	int component_index1)
        //{
        //	// dir coords are -z and y
        //
        //	 float dir0 = -edge[dir_index0];
        //	 float dir1 = edge[dir_index1];
        //	float pmin = pointa[component_index0]*dir0 + pointa[component_index1]*dir1;
        //	float pmax = pointb[component_index0]*dir0 + pointb[component_index1]*dir1;
        //	//find minmax
        //	if(pmin>pmax)
        //	{
        //		BT_SWAP_NUMBERS(pmin,pmax);
        //	}
        //	//find extends
        //	 float rad = extend[component_index0] * absolute_edge[dir_index0] +
        //					extend[component_index1] * absolute_edge[dir_index1];
        //
        //	if(pmin>rad || -rad>pmax) return false;
        //	return true;
        //}
        //
        // bool test_cross_edge_box_X_axis(
        //	 ref Vector3 edge,
        //	 ref Vector3 absolute_edge,
        //	 ref Vector3 pointa,
        //	 ref Vector3 pointb, ref Vector3 extend)
        //{
        //
        //	return test_cross_edge_box(edge,absolute_edge,pointa,pointb,extend,2,1,1,2);
        //}
        //
        //
        // bool test_cross_edge_box_Y_axis(
        //	 ref Vector3 edge,
        //	 ref Vector3 absolute_edge,
        //	 ref Vector3 pointa,
        //	 ref Vector3 pointb, ref Vector3 extend)
        //{
        //
        //	return test_cross_edge_box(edge,absolute_edge,pointa,pointb,extend,0,2,2,0);
        //}
        //
        // bool test_cross_edge_box_Z_axis(
        //	 ref Vector3 edge,
        //	 ref Vector3 absolute_edge,
        //	 ref Vector3 pointa,
        //	 ref Vector3 pointb, ref Vector3 extend)
        //{
        //
        //	return test_cross_edge_box(edge,absolute_edge,pointa,pointb,extend,1,0,0,1);
        //}


        public static bool TEST_CROSS_EDGE_BOX_MCR(ref Vector3 edge, ref Vector3 absolute_edge, ref Vector3 pointa, ref Vector3 pointb, ref Vector3 _extend, int i_dir_0, int i_dir_1, int i_comp_0, int i_comp_1)
        {
            float dir0 = -MathUtil.VectorComponent(ref edge, i_dir_0);
            float dir1 = MathUtil.VectorComponent(ref edge, i_dir_1);
            float pmin = MathUtil.VectorComponent(ref pointa, i_comp_0) * dir0 + MathUtil.VectorComponent(ref pointa, i_comp_1) * dir1;
            float pmax = MathUtil.VectorComponent(ref pointb, i_comp_0) * dir0 + MathUtil.VectorComponent(ref pointb, i_comp_1) * dir1;
            if (pmin > pmax)
            {
                pmin = pmin + pmax;
                pmax = pmin - pmax;
                pmin = pmin - pmax;
            }
            float abs_dir0 = MathUtil.VectorComponent(ref absolute_edge, i_dir_0);
            float abs_dir1 = MathUtil.VectorComponent(ref absolute_edge, i_dir_1);
            float rad = MathUtil.VectorComponent(ref _extend, i_comp_0) * abs_dir0 + MathUtil.VectorComponent(ref _extend, i_comp_1) * abs_dir1;
            if (pmin > rad || -rad > pmax)
            {
                return false;
            }
            return true;
        }


        public static bool TEST_CROSS_EDGE_BOX_X_AXIS_MCR(ref Vector3 edge, ref Vector3 absolute_edge, ref Vector3 pointa, ref Vector3 pointb, ref Vector3 _extend)
        {
            return TEST_CROSS_EDGE_BOX_MCR(ref edge, ref absolute_edge, ref pointa, ref pointb, ref _extend, 2, 1, 1, 2);
        }

        public static bool TEST_CROSS_EDGE_BOX_Y_AXIS_MCR(ref Vector3 edge, ref Vector3 absolute_edge, ref Vector3 pointa, ref Vector3 pointb, ref Vector3 _extend)
        {
            return TEST_CROSS_EDGE_BOX_MCR(ref edge, ref absolute_edge, ref pointa, ref pointb, ref _extend, 0, 2, 2, 0);
        }

        public static bool TEST_CROSS_EDGE_BOX_Z_AXIS_MCR(ref Vector3 edge, ref Vector3 absolute_edge, ref Vector3 pointa, ref Vector3 pointb, ref Vector3 _extend)
        {
            return TEST_CROSS_EDGE_BOX_MCR(ref edge, ref absolute_edge, ref pointa, ref pointb, ref _extend, 1, 0, 0, 1);
        }


        //! Returns the dot product between a vec3f and the col of a matrix
        // float bt_mat3_dot_col(
        // btMatrix3x3 & mat,  ref Vector3 vec3, int colindex)
        //{
        //    return vec3[0]*mat[0][colindex] + vec3[1]*mat[1][colindex] + vec3[2]*mat[2][colindex];
        //}




        public const float BOX_PLANE_EPSILON = 0.000001f;



        //! Compairison of transformation objects
        bool CompareTransformsEqual(ref Matrix t1, ref Matrix t2)
        {
            return t1.Equals(t2);
        }
    }

    //! Axis aligned box
    public struct AABB
    {
        public Vector3 m_min;
        public Vector3 m_max;

        public AABB(ref Vector3 V1,
                 ref Vector3 V2,
                 ref Vector3 V3)
        {
            m_min.X = BoxCollision.BT_MIN3(V1.X, V2.X, V3.X);
            m_min.Y = BoxCollision.BT_MIN3(V1.Y, V2.Y, V3.Y);
            m_min.Z = BoxCollision.BT_MIN3(V1.Z, V2.Z, V3.Z);

            m_max.X = BoxCollision.BT_MAX3(V1.X, V2.X, V3.X);
            m_max.Y = BoxCollision.BT_MAX3(V1.Y, V2.Y, V3.Y);
            m_max.Z = BoxCollision.BT_MAX3(V1.Z, V2.Z, V3.Z);
        }

        public AABB(ref Vector3 V1, ref Vector3 V2, ref Vector3 V3, float margin)
        {
            m_min.X = BoxCollision.BT_MIN3(V1.X, V2.X, V3.X);
            m_min.Y = BoxCollision.BT_MIN3(V1.Y, V2.Y, V3.Y);
            m_min.Z = BoxCollision.BT_MIN3(V1.Z, V2.Z, V3.Z);

            m_max.X = BoxCollision.BT_MAX3(V1.X, V2.X, V3.X);
            m_max.Y = BoxCollision.BT_MAX3(V1.Y, V2.Y, V3.Y);
            m_max.Z = BoxCollision.BT_MAX3(V1.Z, V2.Z, V3.Z);

            m_min.X -= margin;
            m_min.Y -= margin;
            m_min.Z -= margin;
            m_max.X += margin;
            m_max.Y += margin;
            m_max.Z += margin;
        }

        public AABB(ref AABB other)
        {
            m_max = other.m_max;
            m_min = other.m_min;
        }

        public AABB(ref AABB other, float margin)
        {
            m_max = other.m_max;
            m_min = other.m_min;

            m_min.X -= margin;
            m_min.Y -= margin;
            m_min.Z -= margin;
            m_max.X += margin;
            m_max.Y += margin;
            m_max.Z += margin;
        }

        public void Invalidate()
        {
            m_min.X = MathUtil.SIMD_INFINITY;
            m_min.Y = MathUtil.SIMD_INFINITY;
            m_min.Z = MathUtil.SIMD_INFINITY;
            m_max.X = -MathUtil.SIMD_INFINITY;
            m_max.Y = -MathUtil.SIMD_INFINITY;
            m_max.Z = -MathUtil.SIMD_INFINITY;
        }

        public void IncrementMargin(float margin)
        {
            m_min.X -= margin;
            m_min.Y -= margin;
            m_min.Z -= margin;
            m_max.X += margin;
            m_max.Y += margin;
            m_max.Z += margin;
        }

        public void CopyWithMargin(ref AABB other, float margin)
        {
            m_min.X = other.m_min.X - margin;
            m_min.Y = other.m_min.Y - margin;
            m_min.Z = other.m_min.Z - margin;

            m_max.X = other.m_max.X + margin;
            m_max.Y = other.m_max.Y + margin;
            m_max.Z = other.m_max.Z + margin;
        }

        public void CalcFromTriangle(ref Vector3 V1, ref Vector3 V2, ref Vector3 V3)
        {
            m_min.X = BoxCollision.BT_MIN3(V1.X, V2.X, V3.X);
            m_min.Y = BoxCollision.BT_MIN3(V1.Y, V2.Y, V3.Y);
            m_min.Z = BoxCollision.BT_MIN3(V1.Z, V2.Z, V3.Z);

            m_max.X = BoxCollision.BT_MAX3(V1.X, V2.X, V3.X);
            m_max.Y = BoxCollision.BT_MAX3(V1.Y, V2.Y, V3.Y);
            m_max.Z = BoxCollision.BT_MAX3(V1.Z, V2.Z, V3.Z);

        }

        public void CalcFromTriangleMargin(ref Vector3 V1, ref Vector3 V2, ref Vector3 V3, float margin)
        {
            m_min.X = BoxCollision.BT_MIN3(V1.X, V2.X, V3.X);
            m_min.Y = BoxCollision.BT_MIN3(V1.Y, V2.Y, V3.Y);
            m_min.Z = BoxCollision.BT_MIN3(V1.Z, V2.Z, V3.Z);

            m_max.X = BoxCollision.BT_MAX3(V1.X, V2.X, V3.X);
            m_max.Y = BoxCollision.BT_MAX3(V1.Y, V2.Y, V3.Y);
            m_max.Z = BoxCollision.BT_MAX3(V1.Z, V2.Z, V3.Z);

            m_min.X -= margin;
            m_min.Y -= margin;
            m_min.Z -= margin;
            m_max.X += margin;
            m_max.Y += margin;
            m_max.Z += margin;
        }

        //! Apply a transform to an AABB
        public void ApplyTransform(ref Matrix trans)
        {
            Vector3 center = (m_max + m_min) * 0.5f;
            Vector3 extends = m_max - center;
            // Compute new center
            Vector3.Transform(ref center, ref trans, out center);
            Matrix absMatrix = MathUtil.AbsoluteMatrix(trans);

            Vector3 textends = new Vector3(Vector3.Dot(extends, absMatrix.Right),
            Vector3.Dot(extends, absMatrix.Up),
            Vector3.Dot(extends, absMatrix.Backward));

            m_min = center - textends;
            m_max = center + textends;
        }


        //! Apply a transform to an AABB
        public void AppyTransformTransCache(BT_BOX_BOX_TRANSFORM_CACHE trans)
        {
            Vector3 center = (m_max + m_min) * 0.5f;
            Vector3 extends = m_max - center;
            // Compute new center
            center = trans.Transform(ref center);

            Matrix absMatrix = MathUtil.AbsoluteMatrix(trans.m_R1to0);

            Vector3 textends = new Vector3(Vector3.Dot(extends, absMatrix.Right),
            Vector3.Dot(extends, absMatrix.Up),
            Vector3.Dot(extends, absMatrix.Backward));

            m_min = center - textends;
            m_max = center + textends;
        }

        //! Merges a Box
        public void Merge(AABB box)
        {
            Merge(ref box);
        }

        public void Merge(ref AABB box)
        {
            m_min.X = BoxCollision.BT_MIN(m_min.X, box.m_min.X);
            m_min.Y = BoxCollision.BT_MIN(m_min.Y, box.m_min.Y);
            m_min.Z = BoxCollision.BT_MIN(m_min.Z, box.m_min.Z);

            m_max.X = BoxCollision.BT_MAX(m_max.X, box.m_max.X);
            m_max.Y = BoxCollision.BT_MAX(m_max.Y, box.m_max.Y);
            m_max.Z = BoxCollision.BT_MAX(m_max.Z, box.m_max.Z);
        }

        //! Merges a point
        public void MergePoint(ref Vector3 point)
        {
            m_min.X = BoxCollision.BT_MIN(m_min.X, point.X);
            m_min.Y = BoxCollision.BT_MIN(m_min.Y, point.Y);
            m_min.Z = BoxCollision.BT_MIN(m_min.Z, point.Z);

            m_max.X = BoxCollision.BT_MAX(m_max.X, point.X);
            m_max.Y = BoxCollision.BT_MAX(m_max.Y, point.Y);
            m_max.Z = BoxCollision.BT_MAX(m_max.Z, point.Z);
        }

        //! Gets the extend and center
        public void GetCenterExtend(out IndexedVector3 center, out IndexedVector3 extend)
        {
            center = new IndexedVector3((m_max + m_min) * 0.5f);
            extend = new IndexedVector3(new IndexedVector3(m_max) - center);
        }


        public void GetCenterExtend(out Vector3 center, out Vector3 extend)
        {
            center = (m_max + m_min) * 0.5f;
            extend = m_max - center;
        }

        //! Finds the intersecting box between this box and the other.
        public void FindIntersection(ref AABB other, ref AABB intersection)
        {
            intersection.m_min.X = BoxCollision.BT_MAX(other.m_min.X, m_min.X);
            intersection.m_min.Y = BoxCollision.BT_MAX(other.m_min.Y, m_min.Y);
            intersection.m_min.Z = BoxCollision.BT_MAX(other.m_min.Z, m_min.Z);

            intersection.m_max.X = BoxCollision.BT_MIN(other.m_max.X, m_max.X);
            intersection.m_max.Y = BoxCollision.BT_MIN(other.m_max.Y, m_max.Y);
            intersection.m_max.Z = BoxCollision.BT_MIN(other.m_max.Z, m_max.Z);
        }


        public bool HasCollision(ref AABB other)
        {
            if (m_min.X > other.m_max.X ||
               m_max.X < other.m_min.X ||
               m_min.Y > other.m_max.Y ||
               m_max.Y < other.m_min.Y ||
               m_min.Z > other.m_max.Z ||
               m_max.Z < other.m_min.Z)
            {
                return false;
            }
            return true;
        }

        /*! \brief Finds the Ray intersection parameter.
        \param aabb Aligned box
        \param vorigin A vec3f with the origin of the ray
        \param vdir A vec3f with the direction of the ray
        */
        public bool CollideRay(ref Vector3 vorigin, ref Vector3 vdir)
        {
            Vector3 extents, center;
            GetCenterExtend(out center, out extents);

            float Dx = vorigin.X - center.X;
            if (BoxCollision.BT_GREATER(Dx, extents.X) && Dx * vdir.X >= 0.0f) return false;
            float Dy = vorigin.Y - center.Y;
            if (BoxCollision.BT_GREATER(Dy, extents.Y) && Dy * vdir.Y >= 0.0f) return false;
            float Dz = vorigin.Z - center.Z;
            if (BoxCollision.BT_GREATER(Dz, extents.Z) && Dz * vdir.Z >= 0.0f) return false;


            float f = vdir.Y * Dz - vdir.Z * Dy;
            if (Math.Abs(f) > extents.Y * Math.Abs(vdir.Z) + extents.Z * Math.Abs(vdir.Y)) return false;
            f = vdir.Z * Dx - vdir.X * Dz;
            if (Math.Abs(f) > extents.X * Math.Abs(vdir.Z) + extents.Z * Math.Abs(vdir.X)) return false;
            f = vdir.X * Dy - vdir.Y * Dx;
            if (Math.Abs(f) > extents.X * Math.Abs(vdir.Y) + extents.Y * Math.Abs(vdir.X)) return false;
            return true;
        }


        public void ProjectionInterval(ref Vector4 direction, out float vmin, out float vmax)
        {
            Vector3 temp = new Vector3(direction.X, direction.Y, direction.Z);
            ProjectionInterval(ref temp, out vmin, out vmax);
        }

        public void ProjectionInterval(ref Vector3 direction, out float vmin, out float vmax)
        {
            Vector3 center = (m_max + m_min) * 0.5f;
            Vector3 extend = m_max - center;
            Vector3 absDirection;
            MathUtil.AbsoluteVector(ref direction, out absDirection);

            float _fOrigin = Vector3.Dot(direction, center);
            float _fMaximumExtent = Vector3.Dot(extend, absDirection);
            vmin = _fOrigin - _fMaximumExtent;
            vmax = _fOrigin + _fMaximumExtent;
        }

        // seems silly to duplicate this stuff when it's in xna plane, but a cleanup will wait
        // till it's 'working'
        public BT_PLANE_INTERSECTION_TYPE PlaneClassify(ref Vector4 plane)
        {
            float _fmin, _fmax;
            ProjectionInterval(ref plane, out _fmin, out _fmax);

            if (plane.W > _fmax + BoxCollision.BOX_PLANE_EPSILON)
            {
                return BT_PLANE_INTERSECTION_TYPE.BT_CONST_BACK_PLANE; // 0
            }

            if (plane.W + BoxCollision.BOX_PLANE_EPSILON >= _fmin)
            {
                return BT_PLANE_INTERSECTION_TYPE.BT_CONST_COLLIDE_PLANE; //1
            }
            return BT_PLANE_INTERSECTION_TYPE.BT_CONST_FRONT_PLANE;//2
        }

        public bool OverlappingTransConservative(ref AABB box, ref Matrix trans1_to_0)
        {
            AABB tbox = box;
            tbox.ApplyTransform(ref trans1_to_0);
            return HasCollision(ref tbox);
        }

        public bool OverlappingTransConservative2(ref AABB box,
            BT_BOX_BOX_TRANSFORM_CACHE trans1_to_0)
        {
            AABB tbox = box;
            tbox.AppyTransformTransCache(trans1_to_0);
            return HasCollision(ref tbox);
        }

        //! transcache is the transformation cache from box to this AABB
        public bool OverlappingTransCache(ref AABB box, ref BT_BOX_BOX_TRANSFORM_CACHE transcache, bool fulltest)
        {

            //Taken from OPCODE
            IndexedVector3 ea, eb;//extends
            IndexedVector3 ca, cb;//extends
            GetCenterExtend(out ca, out ea);
            box.GetCenterExtend(out cb, out eb);



            IndexedVector3 T = new IndexedVector3(0, 0, 0);
            float t, t2;
            int i;

            // Class I : A's basis vectors

            for (i = 0; i < 3; i++)
            {
                T[i] = (IndexedVector3.Dot(MathUtil.MatrixColumn(ref transcache.m_R1to0, i), cb) + MathUtil.VectorComponent(ref transcache.m_T1to0, i) - ca[i]);
                t = IndexedVector3.Dot(MathUtil.MatrixColumn(ref transcache.m_AR, i), eb) + ea[i];
                if (BoxCollision.BT_GREATER(T[i], t))
                {
                    return false;
                }
            }
            // Class II : B's basis vectors
            for (i = 0; i < 3; i++)
            {
                t = Mat3DotCol(ref transcache.m_R1to0, ref T, i);
                t2 = Mat3DotCol(ref transcache.m_AR, ref ea, i) + eb[i];
                if (BoxCollision.BT_GREATER(t, t2))
                {
                    return false;
                }
            }
            // Class III : 9 cross products
            if (fulltest)
            {
                // check to see if these need to be restored back or are read-only
                float[,] m_R1to0 = MathUtil.BasisMatrixToFloatArray(ref transcache.m_R1to0);
                float[,] m_AR = MathUtil.BasisMatrixToFloatArray(ref transcache.m_AR);


                int j, m, n, o, p, q, r;
                for (i = 0; i < 3; i++)
                {
                    m = (i + 1) % 3;
                    n = (i + 2) % 3;
                    o = i == 0 ? 1 : 0;
                    p = i == 2 ? 1 : 2;
                    for (j = 0; j < 3; j++)
                    {
                        q = j == 2 ? 1 : 2;
                        r = j == 0 ? 1 : 0;
                        t = T[n] * m_R1to0[m, j] - T[m] * m_R1to0[n, j];
                        t2 = ea[o] * m_AR[p, j] + ea[p] * m_AR[o, j] +
                            eb[r] * m_AR[i, q] + eb[q] * m_AR[i, r];
                        if (BoxCollision.BT_GREATER(t, t2)) return false;
                    }
                }
            }
            return true;
        }

        public static float Mat3DotCol(ref Matrix m, ref IndexedVector3 v, int column)
        {
            IndexedVector3 vcolumn;
            if (column == 0)
            {
                vcolumn = new IndexedVector3(m.Left);
            }
            else if (column == 1)
            {
                vcolumn = new IndexedVector3(m.Up);
            }
            else
            {
                vcolumn = new IndexedVector3(m.Backward);
            }
            return IndexedVector3.Dot(ref v, ref vcolumn);
        }


        //! Simple test for planes.
        public bool CollidePlane(ref Vector4 plane)
        {
            BT_PLANE_INTERSECTION_TYPE classify = PlaneClassify(ref plane);
            return (classify == BT_PLANE_INTERSECTION_TYPE.BT_CONST_COLLIDE_PLANE);
        }

        //! test for a triangle, with edges
        public bool CollideTriangleExact(ref Vector3 p1, ref Vector3 p2, ref Vector3 p3, ref Vector4 triangle_plane)
        {
            if (!CollidePlane(ref triangle_plane))
            {
                return false;
            }

            Vector3 center, extends;
            GetCenterExtend(out center, out extends);

            Vector3 v1 = (p1 - center);
            Vector3 v2 = (p2 - center);
            Vector3 v3 = (p3 - center);

            //First axis
            Vector3 diff = (v2 - v1);
            Vector3 abs_diff;
            MathUtil.AbsoluteVector(ref diff, out abs_diff);

            //Test With X axis
            BoxCollision.TEST_CROSS_EDGE_BOX_X_AXIS_MCR(ref diff, ref abs_diff, ref v1, ref v3, ref extends);
            //Test With Y axis
            BoxCollision.TEST_CROSS_EDGE_BOX_Y_AXIS_MCR(ref diff, ref abs_diff, ref v1, ref v3, ref extends);
            //Test With Z axis
            BoxCollision.TEST_CROSS_EDGE_BOX_Z_AXIS_MCR(ref diff, ref abs_diff, ref v1, ref v3, ref extends);


            diff = v3 - v2;
            MathUtil.AbsoluteVector(ref diff, out abs_diff);
            //Test With X axis
            BoxCollision.TEST_CROSS_EDGE_BOX_X_AXIS_MCR(ref diff, ref abs_diff, ref v2, ref v1, ref extends);
            //Test With Y axis
            BoxCollision.TEST_CROSS_EDGE_BOX_Y_AXIS_MCR(ref diff, ref abs_diff, ref v2, ref v1, ref extends);
            //Test With Z axis
            BoxCollision.TEST_CROSS_EDGE_BOX_Z_AXIS_MCR(ref diff, ref abs_diff, ref v2, ref v1, ref extends);

            diff = v1 - v3;
            MathUtil.AbsoluteVector(ref diff, out abs_diff);

            //Test With X axis
            BoxCollision.TEST_CROSS_EDGE_BOX_X_AXIS_MCR(ref diff, ref abs_diff, ref v3, ref v2, ref extends);
            //Test With Y axis
            BoxCollision.TEST_CROSS_EDGE_BOX_Y_AXIS_MCR(ref diff, ref abs_diff, ref v3, ref v2, ref extends);
            //Test With Z axis
            BoxCollision.TEST_CROSS_EDGE_BOX_Z_AXIS_MCR(ref diff, ref abs_diff, ref v3, ref v2, ref extends);

            return true;
        }
    }
    //!  Class for transforming a model1 to the space of model0
    public struct BT_BOX_BOX_TRANSFORM_CACHE
    {
        public Vector3 m_T1to0;//!< Transforms translation of model1 to model 0
        public Matrix m_R1to0;//!< Transforms Rotation of model1 to model 0, equal  to R0' * R1
        public Matrix m_AR;//!< Absolute value of m_R1to0

        public void CalcAbsoluteMatrix()
        {
            //		static  Vector3 vepsi(1e-6f,1e-6f,1e-6f);
            //		m_AR[0] = vepsi + m_R1to0[0].absolute();
            //		m_AR[1] = vepsi + m_R1to0[1].absolute();
            //		m_AR[2] = vepsi + m_R1to0[2].absolute();

            int i, j;

            for (i = 0; i < 3; i++)
            {
                for (j = 0; j < 3; j++)
                {
                    MathUtil.MatrixComponent(ref m_AR, i, j, 1e-6f + Math.Abs(MathUtil.MatrixComponent(ref m_R1to0, i, j)));
                }
            }
        }


        //! Calc the transformation relative  1 to 0. Inverts matrics by transposing
        public void CalcFromHomogenic(ref Matrix trans0, ref Matrix trans1)
        {
            Matrix temp_trans;
            Matrix.Invert(ref trans0, out temp_trans);
            temp_trans = temp_trans * trans1;

            m_T1to0 = temp_trans.Translation;
            MathUtil.BasisMatrix(ref temp_trans, out m_R1to0);

            CalcAbsoluteMatrix();
        }

        //! Calcs the full invertion of the matrices. Useful for scaling matrices
        public void CalcFromFullInvert(ref Matrix trans0, ref Matrix trans1)
        {
            m_R1to0 = Matrix.Invert(trans0);
            m_T1to0 = Vector3.Transform(-trans0.Translation, m_R1to0);

            m_T1to0 += Vector3.Transform(trans1.Translation, m_R1to0);
            // check me, order of multiply
            m_R1to0 *= MathUtil.BasisMatrix(ref trans1);

            CalcAbsoluteMatrix();
        }

        public Vector3 Transform(ref Vector3 point)
        {
            Matrix m = m_R1to0;
            m.Translation = m_T1to0;
            return Vector3.Transform(point, m);
        }
    }

}