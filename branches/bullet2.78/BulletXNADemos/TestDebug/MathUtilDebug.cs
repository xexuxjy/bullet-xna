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

namespace BulletXNADemos.TestDebug
{
	public class MathUtilDebug
	{

		public static void runTests()
		{
			String filename = @"C:\users\man\xna-math-debug-output.txt";
            FileStream filestream = File.Open(filename, FileMode.Create, FileAccess.Write, FileShare.None);
            StreamWriter streamWriter = new StreamWriter(filestream);

			streamWriter.WriteLine("Identity");
			Matrix m = Matrix.Identity;
			MathUtil.PrintMatrix(streamWriter,m);
			streamWriter.WriteLine("setEuler 2,0,0");
			m = MathUtil.SetEulerZYX(2, 0, 0);
			MathUtil.PrintMatrix(streamWriter,m);
			streamWriter.WriteLine("setEuler 0,2,0");
			m = MathUtil.SetEulerZYX(0, 2, 0);
			MathUtil.PrintMatrix(streamWriter,m);
			streamWriter.WriteLine("setEuler 0,0,2");
			m = MathUtil.SetEulerZYX(0, 0, 2);
			MathUtil.PrintMatrix(streamWriter,m);
			streamWriter.WriteLine("setEuler 2,2,0");
			m = MathUtil.SetEulerZYX(2, 2, 0);
			MathUtil.PrintMatrix(streamWriter,m);
			streamWriter.WriteLine("setEuler 2,0,2");
			m = MathUtil.SetEulerZYX(2, 0, 2);
			MathUtil.PrintMatrix(streamWriter,m);
			streamWriter.WriteLine("setEuler 2,2,2");
			m = MathUtil.SetEulerZYX(2, 2, 2);
			MathUtil.PrintMatrix(streamWriter,m);

			streamWriter.WriteLine("setEuler 0.5*PI,0,0 Trans 100,100,100 MULTIPLIED BY setEuler 0,0,0.5*PI Trans -10,10,0");
			m = MathUtil.SetEulerZYX(MathUtil.SIMD_HALF_PI, 0, 0);
			m.Translation = new Vector3(100, 100, 100);

			Matrix m2 = MathUtil.SetEulerZYX(0,0,MathUtil.SIMD_HALF_PI);
			m2.Translation = new Vector3(-10,10,0);

			Matrix m3 = MathUtil.BulletMatrixMultiply(ref m, ref m2);
			MathUtil.PrintMatrix(streamWriter,m3);

            streamWriter.WriteLine("Broken axis comparison");
            m = Matrix.Identity;
            m.Right = new Vector3(1, 0, 0);
            m.Up = new Vector3(0, 1, 0);
            m.Backward = new Vector3(0, 0, 1);
            m.Translation = new Vector3(0, 20, 0);

            m2 = Matrix.Identity;
            m2.Right = new Vector3(0, -1, 0);
            m2.Up = new Vector3(1, 0, 0);
            m2.Backward = new Vector3(0, 0, 1);
            m2.Translation = new Vector3(1, -1, -1);

            m3 = MathUtil.BulletMatrixMultiply(ref m, ref m2);
            MathUtil.PrintMatrix(streamWriter, m3);

            
            streamWriter.WriteLine("setEuler 0.5*PI,0,0 Trans 0,0,0 MULTIPLIED BY setEuler 0,0,0.5*PI Trans -10,10,0");
            m = MathUtil.SetEulerZYX(MathUtil.SIMD_HALF_PI, 0, 0);
            m.Translation = new Vector3(0, 0, 0);

            m2 = MathUtil.SetEulerZYX(0, 0, MathUtil.SIMD_HALF_PI);
            m2.Translation = new Vector3(-10, 10, 0);

            m3 = MathUtil.BulletMatrixMultiply(ref m, ref m2);
            MathUtil.PrintMatrix(streamWriter, m3);

            streamWriter.WriteLine("setEuler 0.25*PI,0,0 Trans 33,0,0 MULTIPLIED BY setEuler 0,0,0.5*PI Trans 0,0,0");
            m = MathUtil.SetEulerZYX(MathUtil.SIMD_QUARTER_PI, 0, 0);
            m.Translation = new Vector3(33, 0, 0);

            m2 = MathUtil.SetEulerZYX(0, 0, MathUtil.SIMD_HALF_PI);
            m2.Translation = new Vector3(0, 0, 0);

            m3 = MathUtil.BulletMatrixMultiply(ref m, ref m2);
            MathUtil.PrintMatrix(streamWriter, m3);



			streamWriter.WriteLine("transposeTimes");
			m = MathUtil.SetEulerZYX(2, 1, 2);
			m.Translation = new Vector3(3,3,3);
			streamWriter.WriteLine("");
            //MathUtil.PrintMatrix(streamWriter, m);

			m2 = MathUtil.SetEulerZYX(1, 2, -2);
			m2.Translation = new Vector3(5,2,13);
            //MathUtil.PrintMatrix(streamWriter, m2);

			m3 = MathUtil.TransposeTimesBasis(ref m,ref m2);
			MathUtil.PrintMatrix(streamWriter,m3);

			streamWriter.WriteLine("inverseTransform.");
            m =Matrix.Identity;
            m = MathUtil.SetEulerZYX(1,-2,-1);
            m.Translation = new Vector3(-1,2,-3);
            Vector3 v = new Vector3(20,25,30);
            Vector3 result = MathUtil.InverseTransform(ref m, ref v);
            MathUtil.PrintVector3(streamWriter,result);
            streamWriter.WriteLine("");

            streamWriter.WriteLine("inverseTimes.");
            m = Matrix.Identity;
            m = MathUtil.SetEulerZYX(1, -2, -1);
            m.Translation = new Vector3(-1, 2, -3);

            m2 = Matrix.Identity;
            m2 = MathUtil.SetEulerZYX(0.3f, 0.8f, 2.3f);
            m2.Translation = new Vector3(20, 25, 30);
            m3 = MathUtil.InverseTimes(ref m, ref m2);
            MathUtil.PrintMatrix(streamWriter, m3);

			
            streamWriter.WriteLine("Transform.");
            m = Matrix.Identity;
            m = MathUtil.SetEulerZYX(1, -2, -1);
            m.Translation = new Vector3(-1, 2, -3);
            v = new Vector3(20, 25, 30);
            result = Vector3.Transform(v,m);
            MathUtil.PrintVector3(streamWriter, result);
			streamWriter.WriteLine("");

            streamWriter.WriteLine("TransformNormal.");
            m = Matrix.Identity;
            m = MathUtil.SetEulerZYX(1, -2, -1);
            m.Translation = new Vector3(-1, 2, -3);
            v = new Vector3(20, 25, 30);

			//Matrix tm = Matrix.Transpose(m);
			//result = Vector3.TransformNormal(v,tm);
			result = Vector3.TransformNormal(v, m);

            MathUtil.PrintVector3(streamWriter, result);
			streamWriter.WriteLine("");

			streamWriter.WriteLine("quatAngle");
			Quaternion q = Quaternion.CreateFromYawPitchRoll(0.3f, 0.2f, 0.7f);
			float fresult = MathUtil.QuatAngle(ref q);
			streamWriter.WriteLine(String.Format("{0:0.00000000}",fresult));

			streamWriter.WriteLine("quatRotate");
			v = new Vector3(20, -25, 30);
			result = MathUtil.QuatRotate(q, v);
			MathUtil.PrintVector3(streamWriter,result);
			streamWriter.WriteLine("");

			streamWriter.WriteLine("shortestArcQuat");
			v = new Vector3(2f, -1f, 3f);
			Vector3 v2 = new Vector3(0.5f, 0.1f, 0.7f);
			q = MathUtil.ShortestArcQuat(v, v2);
			MathUtil.PrintQuaternion(streamWriter, q);
				
			streamWriter.WriteLine("");

			streamWriter.WriteLine("quaternionMultiply");
			q = Quaternion.CreateFromYawPitchRoll(0.3f, 0.2f, 0.7f);
			Quaternion q2 = Quaternion.CreateFromYawPitchRoll(-0.71f,0.8f,0.3f);
			q = MathUtil.QuaternionMultiply(q,q2);
			MathUtil.PrintQuaternion(streamWriter, q);
			streamWriter.WriteLine("");

			streamWriter.WriteLine("matrixToEulerXYZ");
			m = Matrix.Identity;
			m = MathUtil.SetEulerZYX(1, -2, -1);
			m.Translation = new Vector3(-1, 2, -3);
			MathUtil.MatrixToEulerXYZ(ref m, out result);
			MathUtil.PrintVector3(streamWriter, result);
			streamWriter.WriteLine("");


			streamWriter.WriteLine("getSkewSymmetrixMatrix");
			v = new Vector3(0.2f, 0.7f, -0.3f);
			Vector3 v3;
			Vector3 v4; ;
			MathUtil.GetSkewSymmetricMatrix(ref v, out v2, out v3, out v4);
			MathUtil.PrintVector3(streamWriter, v2);
			streamWriter.WriteLine("");
			MathUtil.PrintVector3(streamWriter, v3);
			streamWriter.WriteLine("");
			MathUtil.PrintVector3(streamWriter, v4);
			streamWriter.WriteLine("");


			streamWriter.WriteLine("quaternion create");
			m = MathUtil.SetEulerZYX(0, 0, MathUtil.SIMD_HALF_PI);
			m.Translation = new Vector3(0.0f, 0.30f, 0.0f);
			m2 = MathUtil.SetEulerZYX(0, 0, MathUtil.SIMD_HALF_PI);
			m2.Translation = new Vector3(0.0f, -0.14f, 0.0f);

			q = Quaternion.CreateFromRotationMatrix(m);
			q2 = Quaternion.CreateFromRotationMatrix(m2);
			MathUtil.PrintQuaternion(streamWriter, q);
			streamWriter.WriteLine("");
			MathUtil.PrintQuaternion(streamWriter, q2);
			streamWriter.WriteLine("");

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
