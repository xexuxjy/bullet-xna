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
using System.IO;
using BulletXNA;
using Microsoft.Xna.Framework;
using BulletXNA.LinearMath;

namespace BulletXNADemos.TestDebug
{
	public class MathUtilDebug
	{

		public static void runTests()
		{
			String filename = @"e:\users\man\bullet\xna-math-debug-output.txt";
            FileStream filestream = File.Open(filename, FileMode.Create, FileAccess.Write, FileShare.None);
            StreamWriter streamWriter = new StreamWriter(filestream);

			streamWriter.WriteLine("Identity");
			IndexedMatrix m = IndexedMatrix.Identity;
			MathUtil.PrintMatrix(streamWriter,m);
			streamWriter.WriteLine("setEuler 2,0,0");
			m._basis.SetEulerZYX(2, 0, 0);
			MathUtil.PrintMatrix(streamWriter,m);
			streamWriter.WriteLine("setEuler 0,2,0");
            m = IndexedMatrix.Identity;
			m._basis.SetEulerZYX(0, 2, 0);
			MathUtil.PrintMatrix(streamWriter,m);
			streamWriter.WriteLine("setEuler 0,0,2");
            m = IndexedMatrix.Identity;
			m._basis.SetEulerZYX(0, 0, 2);
			MathUtil.PrintMatrix(streamWriter,m);
			streamWriter.WriteLine("setEuler 2,2,0");
            m = IndexedMatrix.Identity;
			m._basis.SetEulerZYX(2, 2, 0);
			MathUtil.PrintMatrix(streamWriter,m);
			streamWriter.WriteLine("setEuler 2,0,2");
            m = IndexedMatrix.Identity;
			m._basis.SetEulerZYX(2, 0, 2);
			MathUtil.PrintMatrix(streamWriter,m);
			streamWriter.WriteLine("setEuler 2,2,2");
            m = IndexedMatrix.Identity;
			m._basis.SetEulerZYX(2, 2, 2);
			MathUtil.PrintMatrix(streamWriter,m);

			streamWriter.WriteLine("setEuler 0.5*PI,0,0 Trans 100,100,100 MULTIPLIED BY setEuler 0,0,0.5*PI Trans -10,10,0");
            m = IndexedMatrix.Identity;
			m._basis.SetEulerZYX(MathUtil.SIMD_HALF_PI, 0, 0);
			m._origin = new IndexedVector3(100, 100, 100);

            IndexedMatrix m2 = IndexedMatrix.Identity; 
            m2._basis .SetEulerZYX(0, 0, MathUtil.SIMD_HALF_PI);
			m2._origin = new IndexedVector3(-10,10,0);

			IndexedMatrix m3 = m *  m2;
			MathUtil.PrintMatrix(streamWriter,m3);

            //streamWriter.WriteLine("Broken axis comparison");
            //m = IndexedMatrix.Identity;
            //m.Right = new IndexedVector3(1, 0, 0);
            //m.Up = new IndexedVector3(0, 1, 0);
            //m.Backward = new IndexedVector3(0, 0, 1);
            //m._origin = new IndexedVector3(0, 20, 0);

            //m2 = IndexedMatrix.Identity;
            //m2.Right = new IndexedVector3(0, -1, 0);
            //m2.Up = new IndexedVector3(1, 0, 0);
            //m2.Backward = new IndexedVector3(0, 0, 1);
            //m2._origin = new IndexedVector3(1, -1, -1);

            //m3 = m *  m2;
            //MathUtil.PrintMatrix(streamWriter, m3);

            
            //streamWriter.WriteLine("setEuler 0.5*PI,0,0 Trans 0,0,0 MULTIPLIED BY setEuler 0,0,0.5*PI Trans -10,10,0");
            //m = IndexedMatrix.Identity;
            //m._basis.SetEulerZYX(MathUtil.SIMD_HALF_PI, 0, 0);
            //m._origin = new IndexedVector3(0, 0, 0);

            //m2 = IndexedMatrix.Identity;
            //m2._basis.SetEulerZYX(0, 0, MathUtil.SIMD_HALF_PI);
            //m2._origin = new IndexedVector3(-10, 10, 0);

            //m3 = m * m2;
            //MathUtil.PrintMatrix(streamWriter, m3);

            //streamWriter.WriteLine("setEuler 0.25*PI,0,0 Trans 33,0,0 MULTIPLIED BY setEuler 0,0,0.5*PI Trans 0,0,0");
            //m = IndexedMatrix.Identity;
            //m._basis.SetEulerZYX(MathUtil.SIMD_QUARTER_PI, 0, 0);
            //m._origin = new IndexedVector3(33, 0, 0);

            //m2 = IndexedMatrix.Identity; 
            //m2._basis.SetEulerZYX(0, 0, MathUtil.SIMD_HALF_PI);
            //m2._origin = new IndexedVector3(0, 0, 0);

            //m3 = m *m2;
            //MathUtil.PrintMatrix(streamWriter, m3);



			streamWriter.WriteLine("transposeTimes");
            m = IndexedMatrix.Identity;
			m._basis.SetEulerZYX(2, 1, 2);
			m._origin = new IndexedVector3(3,3,3);
			streamWriter.WriteLine("");
            //MathUtil.PrintMatrix(streamWriter, m);

            m2 = IndexedMatrix.Identity;
			m2._basis.SetEulerZYX(1, 2, -2);
			m2._origin = new IndexedVector3(5,2,13);
            //MathUtil.PrintMatrix(streamWriter, m2);

            IndexedBasisMatrix im3 = m._basis.TransposeTimes(m2._basis);
 
			MathUtil.PrintMatrix(streamWriter,im3);

			streamWriter.WriteLine("inverseTransform.");
            m =IndexedMatrix.Identity;
            m._basis.SetEulerZYX(1,-2,-1);
            m._origin = new IndexedVector3(-1,2,-3);
            IndexedVector3 v = new IndexedVector3(20,25,30);
            IndexedVector3 result = MathUtil.InverseTransform(ref m, ref v);
            MathUtil.PrintVector3(streamWriter,result);
            streamWriter.WriteLine("");

            streamWriter.WriteLine("inverseTimes.");
            m = IndexedMatrix.Identity;
            m._basis.SetEulerZYX(1, -2, -1);
            m._origin = new IndexedVector3(-1, 2, -3);

            m2 = IndexedMatrix.Identity;
            m2._basis.SetEulerZYX(0.3f, 0.8f, 2.3f);
            m2._origin = new IndexedVector3(20, 25, 30);
            m3 = m.InverseTimes(ref m2);
            MathUtil.PrintMatrix(streamWriter, m3);

			
            streamWriter.WriteLine("Transform.");
            m = IndexedMatrix.Identity;
            m._basis.SetEulerZYX(1, -2, -1);
            m._origin = new IndexedVector3(-1, 2, -3);
            v = new IndexedVector3(20, 25, 30);
            result = m * v;
            MathUtil.PrintVector3(streamWriter, result);
			streamWriter.WriteLine("");

            streamWriter.WriteLine("TransformNormal.");
            m = IndexedMatrix.Identity;
            m._basis.SetEulerZYX(1, -2, -1);
            m._origin = new IndexedVector3(-1, 2, -3);
            v = new IndexedVector3(20, 25, 30);

			//IndexedMatrix tm = IndexedMatrix.Transpose(m);
			//result = IndexedVector3.TransformNormal(v,tm);
            result = m._basis * v;

            MathUtil.PrintVector3(streamWriter, result);
			streamWriter.WriteLine("");

			streamWriter.WriteLine("quatAngle");
			Quaternion q = Quaternion.CreateFromYawPitchRoll(0.3f, 0.2f, 0.7f);
			float fresult = MathUtil.QuatAngle(ref q);
			streamWriter.WriteLine(String.Format("{0:0.00000000}",fresult));

			streamWriter.WriteLine("quatRotate");
			v = new IndexedVector3(20, -25, 30);
			result = MathUtil.QuatRotate(q, v);
			MathUtil.PrintVector3(streamWriter,result);
			streamWriter.WriteLine("");

			streamWriter.WriteLine("shortestArcQuat");
			v = new IndexedVector3(2f, -1f, 3f);
			IndexedVector3 v2 = new IndexedVector3(0.5f, 0.1f, 0.7f);
			q = MathUtil.ShortestArcQuat(v, v2);
			MathUtil.PrintQuaternion(streamWriter, q);
				
			streamWriter.WriteLine("");

			streamWriter.WriteLine("quaternionMultiply");
			q = Quaternion.CreateFromYawPitchRoll(0.3f, 0.2f, 0.7f);
			Quaternion q2 = Quaternion.CreateFromYawPitchRoll(-0.71f,0.8f,0.3f);
			q = MathUtil.QuaternionMultiply(q,q2);
			MathUtil.PrintQuaternion(streamWriter, q);
			streamWriter.WriteLine("");

            //streamWriter.WriteLine("matrixToEulerXYZ");
            //m = IndexedMatrix.Identity;
            //m._basis..SetEulerZYX(1, -2, -1);
            //m._origin = new IndexedVector3(-1, 2, -3);
            //MathUtil.MatrixToEulerXYZ(ref m, out result);
            //MathUtil.PrintVector3(streamWriter, result);
            //streamWriter.WriteLine("");


			streamWriter.WriteLine("getSkewSymmetrixMatrix");
			v = new IndexedVector3(0.2f, 0.7f, -0.3f);
			IndexedVector3 v3;
			IndexedVector3 v4; ;
			MathUtil.GetSkewSymmetricMatrix(ref v, out v2, out v3, out v4);
			MathUtil.PrintVector3(streamWriter, v2);
			streamWriter.WriteLine("");
			MathUtil.PrintVector3(streamWriter, v3);
			streamWriter.WriteLine("");
			MathUtil.PrintVector3(streamWriter, v4);
			streamWriter.WriteLine("");


			streamWriter.WriteLine("quaternion create");
            m = IndexedMatrix.Identity;
			m._basis.SetEulerZYX(0, 0, MathUtil.SIMD_HALF_PI);
			m._origin = new IndexedVector3(0.0f, 0.30f, 0.0f);
            m2 = IndexedMatrix.Identity;
			m2._basis.SetEulerZYX(0, 0, MathUtil.SIMD_HALF_PI);
			m2._origin = new IndexedVector3(0.0f, -0.14f, 0.0f);

			q = m.GetRotation();
			q2 = m2.GetRotation();
			MathUtil.PrintQuaternion(streamWriter, q);
			streamWriter.WriteLine("");
			MathUtil.PrintQuaternion(streamWriter, q2);
			streamWriter.WriteLine("");



            streamWriter.WriteLine("row and column");
            m = IndexedMatrix.Identity;
            m._basis.SetEulerZYX(0.3f, 0.7f, 0.5f);
            m._origin = new IndexedVector3(11, 22, 33);
            MathUtil.PrintVector3(streamWriter, "col0", m._basis.GetColumn(0));
            MathUtil.PrintVector3(streamWriter, "col1", m._basis.GetColumn(1));
            MathUtil.PrintVector3(streamWriter, "col2", m._basis.GetColumn(2));
            //MathUtil.PrintVector3(streamWriter, "col3", MathUtil.MatrixColumn(m, 0));

            MathUtil.PrintVector3(streamWriter, "row0", m._basis.GetRow(0));
            MathUtil.PrintVector3(streamWriter, "row1", m._basis.GetRow(1));
            MathUtil.PrintVector3(streamWriter, "row2", m._basis.GetRow(2));
            //MathUtil.PrintVector3(streamWriter, "row3", MathUtil.MatrixRow(ref m, 3));


            Matrix lookat1 = Matrix.CreateLookAt(new Vector3(5, 5, 5), new Vector3(10, 10, 10), new Vector3(0, 1, 0));
            IndexedMatrix lookat2 = IndexedMatrix.CreateLookAt(new IndexedVector3(5, 5, 5), new IndexedVector3(10, 10, 10), new IndexedVector3(0, 1, 0));
            Matrix compare = lookat2.ToMatrix();

            MathUtil.PrintMatrix(streamWriter, "lookat1", lookat1);
            MathUtil.PrintMatrix(streamWriter, "lookat2", lookat2);
            MathUtil.PrintMatrix(streamWriter, "lookat compare", compare);


            float aspect = (float)(800.0f / 600.0f);
            float fov = MathHelper.ToRadians(40.0f);
            float near = 1f;
            float far = 500f;

            Matrix pov = Matrix.CreatePerspectiveFieldOfView(fov, aspect, near, far);
            IndexedMatrix pov2 = IndexedMatrix.CreatePerspectiveFieldOfView(fov, aspect, near, far);
            Matrix pov3 = pov2.ToMatrix();

            MathUtil.PrintMatrix(streamWriter, "pov1", pov);
            MathUtil.PrintMatrix(streamWriter, "pov2", pov2);
            MathUtil.PrintMatrix(streamWriter, "pov compare", pov3);


            streamWriter.WriteLine("Complete.");
            streamWriter.Flush();
            filestream.Close();

		}


		static void Main(string[] args)
		{
			runTests();
		}
	}
}
