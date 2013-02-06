using System;
using System.Collections.Generic;
using BulletXNA.BulletCollision;
using BulletXNA.BulletDynamics;
using BulletXNA.LinearMath;
using DemoFramework;

namespace RagdollDemo
{
    class RagdollDemo : Demo
    {
        Vector3 eye = new Vector3(0, 1, 5);
        Vector3 target = new Vector3(0, 1, 0);

        List<Ragdoll> ragdolls = new List<Ragdoll>();

        protected override void OnInitialize()
        {
            Freelook.SetEyeTarget(eye, target);

            Graphics.SetFormText("BulletSharp - Ragdoll Demo");
            Graphics.SetInfoText("Move using mouse and WASD+shift\n" +
                "F3 - Toggle debug\n" +
                //"F11 - Toggle fullscreen\n" +
                "Space - Shoot box");
        }

        protected override void OnInitializePhysics()
        {
            // collision configuration contains default setup for memory, collision setup
            CollisionConf = new DefaultCollisionConfiguration();
            Dispatcher = new CollisionDispatcher(CollisionConf);

            Vector3 worldAabbMin = new Vector3(-10000, -10000, -10000);
            Vector3 worldAabbMax = new Vector3(10000, 10000, 10000);
            Broadphase = new AxisSweep3Internal(ref worldAabbMin, ref worldAabbMax, 0xfffe, 0xffff, 16384, null, true);
            Solver = new SequentialImpulseConstraintSolver();

            World = new DiscreteDynamicsWorld(Dispatcher, Broadphase, Solver, CollisionConf);
            World.SetGravity(new Vector3(0, -10, 0));

            //World.DispatchInfo.UseConvexConservativeDistanceUtil = true;
            //World.DispatchInfo.ConvexConservativeDistanceThreshold = 0.01f;

            // Setup a big ground box
            CollisionShape groundShape = new BoxShape(new Vector3(100, 10, 100));
            CollisionShapes.Add(groundShape);
            Matrix groundTransform = Matrix.CreateTranslation(0, -10, 0);

            RigidBody ground = LocalCreateRigidBody(0, groundTransform, groundShape);
            ground.UserObject = "Ground";

            // Spawn one ragdoll
            SpawnRagdoll(new Vector3(1, 0.5f, 0));
            SpawnRagdoll(new Vector3(-1, 0.5f, 0));
        }

        void SpawnRagdoll(Vector3 startOffset)
        {
            Ragdoll ragdoll = new Ragdoll(World, startOffset);
            ragdolls.Add(ragdoll);
        }
    }

    static class Program
    {
        [STAThread]
        static void Main()
        {
            using (Demo demo = new RagdollDemo())
            {
                demo.Run();
            }
        }
    }
}
