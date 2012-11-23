using System;
using BulletXNA.BulletCollision;
using BulletXNA.BulletDynamics;
using BulletXNA.LinearMath;
using DemoFramework;

namespace CcdPhysicsDemo
{
    class CcdPhysicsDemo : Demo
    {
        bool ccdMode = true;

        Vector3 eye = new Vector3(30, 20, 10);
        Vector3 target = new Vector3(0, 5, -4);

        const float CubeHalfExtents = 1.0f;
        const float ExtraHeight = 1.0f;

        string infoText = "Move using mouse and WASD+shift\n" +
            "F3 - Toggle debug\n" +
            //"F11 - Toggle fullscreen\n" +
            "Space - Shoot box";

        protected override void OnInitialize()
        {
            Freelook.SetEyeTarget(eye, target);

            Graphics.SetFormText("Bullet - CCD Demo");
            Graphics.SetInfoText(infoText + "\nCCD enabled (P to disable)");
        }

        protected override void OnInitializePhysics()
        {
            int i;

            shootBoxInitialSpeed = 4000;

            // collision configuration contains default setup for memory, collision setup
            CollisionConf = new DefaultCollisionConfiguration();

            Dispatcher = new CollisionDispatcher(CollisionConf);
            //Dispatcher.RegisterCollisionCreateFunc(BroadphaseNativeType.BoxShape, BroadphaseNativeType.BoxShape,
            //    CollisionConf.GetCollisionAlgorithmCreateFunc(BroadphaseNativeType.ConvexShape, BroadphaseNativeType.ConvexShape));

            Broadphase = new DbvtBroadphase();


            // the default constraint solver.
            Solver = new SequentialImpulseConstraintSolver();

            World = new DiscreteDynamicsWorld(Dispatcher, Broadphase, Solver, CollisionConf);
            World.GetSolverInfo().m_solverMode |= SolverMode.Use2FrictionDirections | SolverMode.RandomizeOrder;
            //World.GetSolverInfo().m_splitImpulse = 0;
            World.GetSolverInfo().m_numIterations = 20;

            World.DispatchInfo.m_useContinuous = ccdMode;

            World.SetGravity(new Vector3(0, -10, 0));

            BoxShape ground = new BoxShape(new Vector3(200, 1, 200));
            ground.InitializePolyhedralFeatures();
            CollisionShapes.Add(ground);
            RigidBody body = LocalCreateRigidBody(0, Matrix.Identity, ground);
            body.Friction = 0.5f;
            //body.RollingFriction = 0.3f;
            body.UserObject = "Ground";

            //CollisionShape shape = new CylinderShape(CubeHalfExtents, CubeHalfExtents, CubeHalfExtents);
            CollisionShape shape = new BoxShape(new Vector3(CubeHalfExtents, CubeHalfExtents, CubeHalfExtents));
            CollisionShapes.Add(shape);

            const int numObjects = 120;
            for (i = 0; i < numObjects; i++)
            {
                //stack them
                const int colsize = 10;
                int row = (int)((i * CubeHalfExtents * 2) / (colsize * 2 * CubeHalfExtents));
                int row2 = row;
                int col = (i) % (colsize) - colsize / 2;

                if (col > 3)
                {
                    col = 11;
                    row2 |= 1;
                }

                Matrix trans = Matrix.CreateTranslation(col * 2 * CubeHalfExtents + (row2 % 2) * CubeHalfExtents,
                    row * 2 * CubeHalfExtents + CubeHalfExtents + ExtraHeight, 0);

                body = LocalCreateRigidBody(1, trans, shape);
                //body.SetAnisotropicFriction(shape.AnisotropicRollingFrictionDirection, AnisotropicFrictionFlags.AnisotropicRollingFriction);
                body.Friction = 0.5f;
                //body.RollingFriction = 0.3f;

                if (ccdMode)
                {
                    body.CcdMotionThreshold = 1e-7f;
                    body.CcdSweptSphereRadius = 0.9f * CubeHalfExtents;
                }
            }
        }
    }

    static class Program
    {
        [STAThread]
        static void Main()
        {
            if (LibraryTest.Test() == false)
                return;

            RunDemo();
        }

        static void RunDemo()
        {
            using (Demo demo = new CcdPhysicsDemo())
            {
                demo.Run();
            }
        }
    }
}
