using System;
using System.Collections.Generic;
using BulletXNA;
using BulletXNA.BulletCollision;
using BulletXNA.BulletDynamics;
using Microsoft.Xna.Framework;
using BulletXNA.LinearMath;
using System.IO;

namespace BulletXNADemos.Demos
{
		
	public class TestRig
	{
		public const int NUM_LEGS=6;
		public const int BODYPART_COUNT = 2 * NUM_LEGS + 1;
		public const int JOINT_COUNT = BODYPART_COUNT - 1;

		CollisionShape[] m_shapes = new CollisionShape[BODYPART_COUNT];
		RigidBody[] m_bodies = new RigidBody[BODYPART_COUNT];
		HingeConstraint[] m_joints = new HingeConstraint[JOINT_COUNT];

		DynamicsWorld m_dynamicsWorld;

		public TestRig (DemoApplication demoApplication,DynamicsWorld ownerWorld, ref IndexedVector3 positionOffset, bool bFixed)
		{
			m_dynamicsWorld = ownerWorld;
            IndexedVector3 vUp = new IndexedVector3(0, 1, 0);

			//
			// Setup geometry
			//
			float fBodySize  = 0.25f;
			float fLegLength = 0.45f;
			float fForeLegLength = 0.75f;
			m_shapes[0] = new CapsuleShape(fBodySize, 0.10f);
			for (int i=0; i<NUM_LEGS; i++)
			{
				m_shapes[1 + 2*i] = new CapsuleShape(0.10f, fLegLength);
				m_shapes[2 + 2*i] = new CapsuleShape(0.08f, fForeLegLength);
			}

			//
			// Setup rigid bodies
			//
			float fHeight = 0.5f;
			IndexedMatrix offset = IndexedMatrix.Identity;
			offset._origin = positionOffset;		

			// root
			IndexedVector3 vRoot = new IndexedVector3(0,fHeight,0);
			IndexedMatrix transform = IndexedMatrix.Identity;
			transform._origin = vRoot;

			if (bFixed)
			{
				m_bodies[0] = demoApplication.LocalCreateRigidBody(0.0f, offset * transform, m_shapes[0]);
			} else
			{
				m_bodies[0] = demoApplication.LocalCreateRigidBody(1.0f, offset * transform, m_shapes[0]);
			}
			// legs
			for (int i=0; i<NUM_LEGS; i++)
			{
				float fAngle = MathUtil.SIMD_2_PI * i / NUM_LEGS;
				float fSin = (float)Math.Sin(fAngle);
				float fCos = (float)Math.Cos(fAngle);

				transform = IndexedMatrix.Identity;
				IndexedVector3 vBoneOrigin = new IndexedVector3(fCos*(fBodySize+0.5f*fLegLength), fHeight, fSin*(fBodySize+0.5f*fLegLength));
				transform._origin = vBoneOrigin;

				// thigh
				IndexedVector3 vToBone = (vBoneOrigin - vRoot);
				vToBone.Normalize();

				IndexedVector3 vAxis = IndexedVector3.Cross(vToBone,vUp);	
				transform._basis = new IndexedBasisMatrix(Quaternion.CreateFromAxisAngle(vAxis.ToVector3(), MathUtil.SIMD_HALF_PI));
				transform._origin = vBoneOrigin;
				m_bodies[1+2*i] = demoApplication.LocalCreateRigidBody(1.0f, offset * transform, m_shapes[1+2*i]);

				// shin
				transform = IndexedMatrix.Identity;
				transform._origin = new IndexedVector3(fCos*(fBodySize+fLegLength), fHeight-0.5f*fForeLegLength, fSin*(fBodySize+fLegLength));
				m_bodies[2+2*i] = demoApplication.LocalCreateRigidBody(1.0f, offset * transform, m_shapes[2+2*i]);
			}

			// Setup some damping on the m_bodies
			for (int i = 0; i < BODYPART_COUNT; ++i)
			{
				m_bodies[i].SetDamping(0.05f, 0.85f);
				m_bodies[i].SetDeactivationTime(0.8f);
				//m_bodies[i].setSleepingThresholds(1.6, 2.5);
				m_bodies[i].SetSleepingThresholds(0.5f, 0.5f);
			}


			//
			// Setup the constraints
			//
			HingeConstraint hingeC;

			IndexedMatrix localA, localB, localC;

			for (int i=0; i<NUM_LEGS; i++)
			{
				float fAngle = MathUtil.SIMD_2_PI * i / NUM_LEGS;
				float fSin = (float)Math.Sin(fAngle);
				float fCos = (float)Math.Cos(fAngle);

				// hip joints
				localA = IndexedMatrix.Identity; 
				localB= IndexedMatrix.Identity;

				localA = MathUtil.SetEulerZYX(0f,-fAngle,0f);	
				localA._origin = new IndexedVector3(fCos*fBodySize, 0.0f, fSin*fBodySize);
                localB = m_bodies[1 + 2 * i].GetWorldTransform().Inverse() * m_bodies[0].GetWorldTransform() * localA;


                if (BulletGlobals.g_streamWriter != null && false)
				{
					MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "Hip LocalA", localA);
					MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "Hip LocalB", localB);
				}

				hingeC = new HingeConstraint(m_bodies[0], m_bodies[1+2*i], ref localA, ref localB);
				hingeC.SetLimit(-0.75f * MathUtil.SIMD_QUARTER_PI, MathUtil.SIMD_QUARTER_PI/2f);
				m_joints[2*i] = hingeC;
				m_dynamicsWorld.AddConstraint(m_joints[2*i], true);

				// knee joints
				localA = IndexedMatrix.Identity; 
				localB= IndexedMatrix.Identity;
				localC = IndexedMatrix.Identity;

				localA = MathUtil.SetEulerZYX(0f,-fAngle,0f);	
				localA._origin = new IndexedVector3(fCos*(fBodySize+fLegLength), 0.0f, fSin*(fBodySize+fLegLength));

                localB = m_bodies[1 + 2 * i].GetWorldTransform().Inverse() * m_bodies[0].GetWorldTransform() * localA;
                localC = m_bodies[2 + 2 * i].GetWorldTransform().Inverse() * m_bodies[0].GetWorldTransform() * localA;


				if (BulletGlobals.g_streamWriter != null && false)
				{
					MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "Knee LocalA", localA);
					MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "Knee LocalB", localB);
					MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "Knee LocalC", localC);
				}

				hingeC = new HingeConstraint(m_bodies[1 + 2 * i], m_bodies[2 + 2 * i], ref localB, ref localC);
				//hingeC.setLimit(float(-0.01), float(0.01));
				hingeC.SetLimit(-MathUtil.SIMD_QUARTER_PI/2f, 0.2f);
				m_joints[1+2*i] = hingeC;
				m_dynamicsWorld.AddConstraint(m_joints[1+2*i], true);
			}
		}

		public HingeConstraint[] GetJoints()
		{
			return m_joints;
		}

	}

	public class MotorPreTickCallback : IInternalTickCallback
	{
		public void InternalTickCallback(DynamicsWorld world, float timeStep)
		{
			MotorDemo motorDemo = (MotorDemo)world.GetWorldUserInfo();
			motorDemo.SetMotorTargets(timeStep);
		}
	}


		public class MotorDemo : DemoApplication
		{


		public override void InitializeDemo()
		{
            //string filename = @"C:\users\man\bullet\xna-motor-output.txt";
            //FileStream filestream = File.Open(filename, FileMode.Create, FileAccess.Write, FileShare.Read);
            //BulletGlobals.g_streamWriter = new StreamWriter(filestream);


			m_Time = 0;
			m_fCyclePeriod = 2000.0f; // in milliseconds

			// new SIMD solver for joints clips accumulated impulse, so the new limits for the motor
			// should be (numberOfsolverIterations * oldLimits)
			// currently solver uses 10 iterations, so:
			m_fMuscleStrength = 0.5f;

			SetCameraDistance(5.0f);

			m_collisionConfiguration = new DefaultCollisionConfiguration();

			m_dispatcher = new CollisionDispatcher(m_collisionConfiguration);

			m_broadphase = new DbvtBroadphase();
			IOverlappingPairCache pairCache = null;


			m_broadphase = new SimpleBroadphase(1000, pairCache);

			SequentialImpulseConstraintSolver sol = new SequentialImpulseConstraintSolver();
			m_constraintSolver = sol;

			m_dynamicsWorld = new DiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);

			IndexedVector3 gravity = new IndexedVector3(0, -10, 0);
			m_dynamicsWorld.SetGravity(ref gravity);

			m_dynamicsWorld.SetInternalTickCallback(new MotorPreTickCallback(),this,true);


			// Setup a big ground box
			{
				CollisionShape groundShape = new BoxShape(new IndexedVector3(200.0f,10.0f,200.0f));
				m_collisionShapes.Add(groundShape);
				IndexedMatrix groundTransform = IndexedMatrix.CreateTranslation(0,-10,0);
				LocalCreateRigidBody(0f,ref groundTransform,groundShape);
			}

			// Spawn one ragdoll
			IndexedVector3 startOffset = new IndexedVector3(1,0.5f,0);
			SpawnTestRig(ref startOffset, false);
			startOffset = new IndexedVector3(-2, 0.5f, 0);
            SpawnTestRig(ref startOffset, true);

			ClientResetScene();
		}


		public void SpawnTestRig(ref IndexedVector3 startOffset, bool bFixed)
		{
			TestRig rig = new TestRig(this,m_dynamicsWorld, ref startOffset, bFixed);
			m_rigs.Add(rig);
		}

		public void	PreStep()
		{

		}

		public void SetMotorTargets(float deltaTime)
		{

			float ms = deltaTime*1000000.0f;
			float minFPS = 1000000.0f/60.0f;
			if (ms > minFPS)
			{
				ms = minFPS;
			}

			m_Time += ms;

			//
			// set per-frame sinusoidal position targets using angular motor (hacky?)
			//	
			for (int r=0; r<m_rigs.Count; r++)
			{
				HingeConstraint[] hingeConstraints = m_rigs[r].GetJoints();
				for (int i=0; i<2*TestRig.NUM_LEGS; i++)
				{
					HingeConstraint hingeC = hingeConstraints[i];
					float fCurAngle = hingeC.GetHingeAngle();
			
					float fTargetPercent = ((int)(m_Time / 1000) % (int)(m_fCyclePeriod)) / m_fCyclePeriod;
					float fTargetAngle   = 0.5f * (1 + (float)Math.Sin(MathUtil.SIMD_2_PI * fTargetPercent));
					float fTargetLimitAngle = hingeC.GetLowerLimit() + fTargetAngle * (hingeC.GetUpperLimit() - hingeC.GetLowerLimit());
					float fAngleError  = fTargetLimitAngle - fCurAngle;
					float fDesiredAngularVel = 1000000.0f * fAngleError/ms;
					hingeC.EnableAngularMotor(true, fDesiredAngularVel, m_fMuscleStrength);
				}
			}
		}

		static void Main(string[] args)
		{
			using (MotorDemo game = new MotorDemo())
			{
				game.Run();
			}
		}


		float m_Time;
		float m_fCyclePeriod; // in milliseconds
		float m_fMuscleStrength;

		List<TestRig> m_rigs = new List<TestRig>();

	}
}
