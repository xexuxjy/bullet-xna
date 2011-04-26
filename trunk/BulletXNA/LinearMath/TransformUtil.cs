/*
 * C# / XNA  port of Bullet (c) 2011 Mark Neale <xexuxjy@hotmail.com>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

using System;
using System.Collections.Generic;

using Microsoft.Xna.Framework;

namespace BulletXNA
{
    public static class TransformUtil
    {
        public static IList<float> FloatToList(float f)
        {
            IList<float> list = new List<float>();
            list.Add(f);
            return list;
        }

        public static IList<float> VectorToList(Vector3 vector)
        {
            return VectorToList(ref vector);
        }

        public static IList<float> VectorToList(ref Vector3 vector)
        {
            IList<float> list = new List<float>();
            list.Add(vector.X);
            list.Add(vector.Y);
            list.Add(vector.Z);
            return list;
        }

        public static IList<Vector3> VectorsFromList(IList<float> list)
        {
            IList<Vector3> vecList = new List<Vector3>();
            int numVectors = list.Count / 3;
            for(int i=0;i<numVectors;++i)
            {
                Vector3 vec = new Vector3(list[3*i],list[(3*i)+1],list[(3*i)+2]);
                vecList.Add(vec);
            }
            return vecList;
        }

        //public static Vector3 InverseTransform(ref Vector3 v, ref Matrix m)
        //{
        //    return Vector3.Transform(v, Matrix.Invert(m));
        //}
        
        public static void PlaneSpace1(ref Vector3 n, out Vector3 p, out Vector3 q)
        {
            if (Math.Abs(n.Z) > MathUtil.SIMDSQRT12)
            {
                // choose p in y-z plane
                float a = n.Y * n.Y + n.Z * n.Z;
                float k = MathUtil.RecipSqrt(a);
                p = new Vector3(0, -n.Z * k, n.Y * k);
                // set q = n x p
                q = new Vector3(a * k, -n.X * p.Z, n.X * p.Y);
            }
            else
            {
                // choose p in x-y plane
                float a = n.X * n.X + n.Y * n.Y;
                float k = MathUtil.RecipSqrt(a);
                p = new Vector3(-n.Y * k, n.X * k, 0);
                // set q = n x p
                q = new Vector3(-n.Z * p.Y, n.Z * p.X, a * k);
            }
        }


        public static Vector3 AabbSupport(ref Vector3 halfExtents,ref Vector3 supportDir)
        {
	        return new Vector3(supportDir.X < 0f ? -halfExtents.X : halfExtents.X,
              supportDir.Y < 0f ? -halfExtents.Y : halfExtents.Y,
              supportDir.Z < 0f ? -halfExtents.Z : halfExtents.Z); 
        }

        public static void IntegrateTransform(Matrix curTrans, Vector3 linvel, Vector3 angvel, float timeStep, ref Matrix predictedTransform)
        {
            IntegrateTransform(ref curTrans, ref linvel, ref angvel, timeStep, ref predictedTransform);
        }

	    public static void IntegrateTransform(ref Matrix curTrans,ref Vector3 linvel,ref Vector3 angvel,float timeStep,ref Matrix predictedTransform)
	    {
            predictedTransform = Matrix.Identity;
		    predictedTransform.Translation = (curTrans.Translation + linvel * timeStep);
    //	#define QUATERNION_DERIVATIVE
	    #if QUATERNION_DERIVATIVE
            Vector3 pos;
            Quaternion predictedOrn;
            Vector3 scale;

            curTrans.Decompose(ref scale, ref predictedOrn, ref pos);


		    predictedOrn += (angvel * predictedOrn) * (timeStep * .5f));
		    predictedOrn.Normalize();
        #else
            //Exponential map
		    //google for "Practical Parameterization of Rotations Using the Exponential Map", F. Sebastian Grassia

		    Vector3 axis;
		    float	fAngle = angvel.Length(); 
		    //limit the angular motion
		    if (fAngle*timeStep > ANGULAR_MOTION_THRESHOLD)
		    {
			    fAngle = ANGULAR_MOTION_THRESHOLD / timeStep;
		    }

		    if ( fAngle < 0.001f )
		    {
			    // use Taylor's expansions of sync function
			    axis   = angvel*( 0.5f*timeStep-(timeStep*timeStep*timeStep)*(0.020833333333f)*fAngle*fAngle );
		    }
		    else
		    {
			    // sync(fAngle) = sin(c*fAngle)/t
			    axis   = angvel*( (float)Math.Sin(0.5f*fAngle*timeStep)/fAngle );
		    }
		    Quaternion dorn = new Quaternion(axis.X,axis.Y,axis.Z,(float)Math.Cos( fAngle*timeStep*.5f) );
            Vector3 pos;
            Quaternion rot;
            Vector3 scale;

            curTrans.Decompose(out scale, out rot, out pos);
            Quaternion orn0 = rot;

		    Quaternion predictedOrn = dorn * orn0;
		    predictedOrn.Normalize();
	    #endif

            Matrix newMatrix = Matrix.CreateFromQuaternion(predictedOrn);
            CopyMatrixRotation(ref newMatrix, ref predictedTransform);
	    }

        public static void CalculateVelocityQuaternion(ref Vector3 pos0,ref Vector3 pos1,ref Quaternion orn0,ref Quaternion orn1,float timeStep,ref Vector3 linVel,ref Vector3 angVel)
	    {
		    linVel = (pos1 - pos0) / timeStep;
		    Vector3 axis = Vector3.Zero;
		    float angle = 0f;
		    if (orn0 != orn1)
		    {
			    CalculateDiffAxisAngleQuaternion(ref orn0,ref orn1,ref axis,ref angle);
			    angVel = axis * (angle / timeStep);
		    } 
            else
		    {
			    angVel = Vector3.Zero;
		    }
	    }

	    public static void CalculateDiffAxisAngleQuaternion(ref Quaternion orn0,ref Quaternion orn1a,ref Vector3 axis,ref float angle)
	    {
            Quaternion orn1 = MathUtil.QuatFurthest(ref orn0,ref orn1a);
            Quaternion dorn = orn1 * MathUtil.QuaternionInverse(ref orn0);

		    ///floating point inaccuracy can lead to w component > 1..., which breaks 
		    dorn.Normalize();
		    angle = MathUtil.QuatAngle(ref dorn);
		    axis = new Vector3(dorn.X,dorn.Y,dorn.Z);

		    //check for axis length
		    float len = axis.LengthSquared();
		    if (len < MathUtil.SIMD_EPSILON*MathUtil.SIMD_EPSILON)
            {
			    axis = new Vector3(1f,0,0);
            }
		    else
            {
			    axis.Normalize();
            }
	    }

	    public static void CalculateVelocity(ref Matrix transform0,ref Matrix transform1,float timeStep,ref Vector3 linVel,ref Vector3 angVel)
	    {
		    linVel = (transform1.Translation - transform0.Translation) / timeStep;
            MathUtil.SanityCheckVector(ref linVel);
		    Vector3 axis = Vector3.Zero;
		    float  angle = 0f;
		    CalculateDiffAxisAngle(ref transform0,ref transform1,ref axis,ref angle);
		    angVel = axis * (angle / timeStep);
            MathUtil.SanityCheckVector(ref angVel);
	    }

	    public static void CalculateDiffAxisAngle(ref Matrix transform0,ref Matrix transform1,ref Vector3 axis,ref float angle)
	    {
            //Matrix dmat = GetRotateMatrix(ref transform1) * Matrix.Invert(GetRotateMatrix(ref transform0));
            Matrix dmat = MathUtil.BulletMatrixMultiplyBasis(transform1, Matrix.Invert(transform0));
		    Quaternion dorn = Quaternion.Identity;
		    GetRotation(ref dmat,out dorn);

		    ///floating point inaccuracy can lead to w component > 1..., which breaks 
		    dorn.Normalize();
    		
		    angle = MathUtil.QuatAngle(ref dorn);
            
		    axis = new Vector3(dorn.X,dorn.Y,dorn.Z);
            //axis[3] = float(0.);
		    //check for axis length
		    float len = axis.LengthSquared();
            if (len < MathUtil.SIMD_EPSILON * MathUtil.SIMD_EPSILON)
            {
                axis = Vector3.Right;
            }
            else
            {
                axis.Normalize();
            }
	    }

        public static void CopyMatrixRotation(ref Matrix from, ref Matrix to)
        {
            to.Left = from.Left;
            to.Up = from.Up;
            to.Backward = from.Backward;
        }

        //public static void GetRotateMatrix(ref Matrix a  , ref Matrix b)
        //{
        //    b = Matrix.Identity;
        //    b.Right = a.Right;
        //    b.Up = a.Up;
        //    b.Backward = a.Backward;
        //}

        //public static Matrix GetRotateMatrix(ref Matrix a)
        //{
        //    Matrix b = Matrix.Identity;
        //    b.Right = a.Right;
        //    b.Up = a.Up;
        //    b.Backward = a.Backward;
        //    return b;
        //}

        public static void GetRotation(ref Matrix a, out Quaternion rot)
        {
            Vector3 pos;
            Vector3 scale;

            a.Decompose(out scale, out rot, out pos);
        }

        public static Quaternion GetRotation(ref Matrix a)
        {
            Vector3 pos;
            Vector3 scale;
            Quaternion rot;
            a.Decompose(out scale, out rot, out pos);
            return rot;
        }

        public static float ANGULAR_MOTION_THRESHOLD = .5f * MathUtil.SIMD_HALF_PI;

    }


    ///The btConvexSeparatingDistanceUtil can help speed up convex collision detection 
    ///by conservatively updating a cached separating distance/vector instead of re-calculating the closest distance
    public class ConvexSeparatingDistanceUtil
    {
        private Quaternion m_ornA;
        private Quaternion m_ornB;
        private Vector3 m_posA;
        private Vector3 m_posB;

        private Vector3 m_separatingNormal;

        private float m_boundingRadiusA;
        private float m_boundingRadiusB;
        private float m_separatingDistance;

        public ConvexSeparatingDistanceUtil(float boundingRadiusA, float boundingRadiusB)
        {
            m_boundingRadiusA = boundingRadiusA;
            m_boundingRadiusB = boundingRadiusB;
            m_separatingDistance = 0f;
        }

        public float GetConservativeSeparatingDistance()
        {
            return m_separatingDistance;
        }

        public void UpdateSeparatingDistance(ref Matrix transA, ref Matrix transB)
        {
            Vector3 toPosA = transA.Translation;
            Vector3 toPosB = transB.Translation;
            Quaternion toOrnA = TransformUtil.GetRotation(ref transA);
            Quaternion toOrnB = TransformUtil.GetRotation(ref transB);

            if (m_separatingDistance > 0.0f)
            {
                Vector3 linVelA = Vector3.Zero;
                Vector3 angVelA = Vector3.Zero;
                Vector3 linVelB = Vector3.Zero;
                Vector3 angVelB = Vector3.Zero;

                TransformUtil.CalculateVelocityQuaternion(ref m_posA, ref toPosA, ref m_ornA, ref toOrnA, 1f, ref linVelA, ref angVelA);
                TransformUtil.CalculateVelocityQuaternion(ref m_posB, ref toPosB, ref m_ornB, ref toOrnB, 1f, ref linVelB, ref angVelB);
                float maxAngularProjectedVelocity = angVelA.Length() * m_boundingRadiusA + angVelB.Length() * m_boundingRadiusB;
                Vector3 relLinVel = (linVelB - linVelA);
                float relLinVelocLength = Vector3.Dot((linVelB - linVelA), m_separatingNormal);
                if (relLinVelocLength < 0f)
                {
                    relLinVelocLength = 0f;
                }

                float projectedMotion = maxAngularProjectedVelocity + relLinVelocLength;
                m_separatingDistance -= projectedMotion;
            }

            m_posA = toPosA;
            m_posB = toPosB;
            m_ornA = toOrnA;
            m_ornB = toOrnB;
        }

        void InitSeparatingDistance(ref Vector3 separatingVector, float separatingDistance, ref Matrix transA, ref Matrix transB)
	    {
		    m_separatingNormal = separatingVector;
		    m_separatingDistance = separatingDistance;
    		
		    Vector3 toPosA = transA.Translation;
		    Vector3 toPosB = transB.Translation;
            Quaternion toOrnA = TransformUtil.GetRotation(ref transA);
            Quaternion toOrnB = TransformUtil.GetRotation(ref transB);
		    m_posA = toPosA;
		    m_posB = toPosB;
		    m_ornA = toOrnA;
		    m_ornB = toOrnB;
	    }
    }
}
