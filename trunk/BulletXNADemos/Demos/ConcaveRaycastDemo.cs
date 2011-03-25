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
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using BulletXNA.BullettCollision.CollisionDispatch;
using BulletXNA.BulletCollision.CollisionShapes;
using BulletXNA.LinearMath;
using Microsoft.Xna.Framework.Input;
using BulletXNA.BullettDynamics.Dynamics;
using BulletXNA.BullettCollision.CollisionShapes;
using BulletXNA;
using BulletXNA.BulletDynamics.ConstraintSolver;
using BulletXNA.BulletCollision.CollisionDispatch;
using BulletXNA.BulletDynamics.Dynamics;

namespace BulletXNADemos.Demos
{
    public class ConcaveRaycastDemo : DemoApplication
    {
        public void	SetVertexPositions(float waveheight, float offset)
        {
	        for (int i=0;i<NUM_VERTS_X;i++)
	        {
		        for (int j=0;j<NUM_VERTS_Y;j++)
		        {
			        gVertices[i+j*NUM_VERTS_X] = new Vector3((i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
                        //0.0f,
                        (float)(waveheight * Math.Sin((float)i + offset) * Math.Cos((float)j + offset)),
				        (j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
		        }
	        }
        }

        public override void KeyboardCallback(Keys key,int x,int y,GameTime gameTime,bool released,ref KeyboardState newState,ref KeyboardState oldState)
        {
	        if (key == Keys.G)
	        {
		        m_animatedMesh = !m_animatedMesh;
		        if (m_animatedMesh)
		        {
			        staticBody.SetCollisionFlags( staticBody.GetCollisionFlags() | CollisionFlags.CF_KINEMATIC_OBJECT);
			        staticBody.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
		        } 
                else
		        {
			        staticBody.SetCollisionFlags( staticBody.GetCollisionFlags() & ~CollisionFlags.CF_KINEMATIC_OBJECT);
			        staticBody.ForceActivationState(ActivationState.ACTIVE_TAG);
		        }
	        }

	        base.KeyboardCallback(key,x,y,gameTime,released,ref newState,ref oldState);

        }

        public override void InitializeDemo()
        {
			m_cameraDistance = 400f;
			m_farClip = 1500f;
			m_perspective = Matrix.CreatePerspectiveFieldOfView(MathHelper.ToRadians(40.0f), m_aspect, m_nearClip, m_farClip);


            m_animatedMesh = true;
			base.InitializeDemo();
			int totalTriangles = 2 * (NUM_VERTS_X - 1) * (NUM_VERTS_Y - 1);

			gVertices = new ObjectArray<Vector3>(totalVerts);
			int indicesTotal = totalTriangles * 3;
            gIndices = new ObjectArray<int>(indicesTotal);

			BulletGlobals.gContactAddedCallback = null;

			SetVertexPositions(waveheight,0f);

			int vertStride = 1;
			int indexStride = 3;

			int index=0;
            //for (int i=0;i<NUM_VERTS_X-1;i++)
            //{
            //    for (int j=0;j<NUM_VERTS_Y-1;j++)
            //    {
            //        gIndices[index++] = j * NUM_VERTS_X + i;
            //        gIndices[index++] = (j + 1) * NUM_VERTS_X + i + 1;
            //        gIndices[index++] = j * NUM_VERTS_X + i + 1;

            //        gIndices[index++] = j * NUM_VERTS_X + i;
            //        gIndices[index++] = (j + 1) * NUM_VERTS_X + i;
            //        gIndices[index++] = (j + 1) * NUM_VERTS_X + i + 1;

            //    }
            //}

            for (int i = 0; i < NUM_VERTS_X - 1; i++)
            {
                for (int j = 0; j < NUM_VERTS_Y - 1; j++)
                {
                    gIndices[index++] = j * NUM_VERTS_X + i;
                    gIndices[index++] = j * NUM_VERTS_X + i + 1;
                    gIndices[index++] = (j + 1) * NUM_VERTS_X + i + 1;

                    gIndices[index++] = j * NUM_VERTS_X + i;
                    gIndices[index++] = (j + 1) * NUM_VERTS_X + i + 1;
                    gIndices[index++] = (j + 1) * NUM_VERTS_X + i;
                }
            }

			TriangleIndexVertexArray indexVertexArrays = new TriangleIndexVertexArray(totalTriangles,
				gIndices,indexStride,totalVerts,gVertices,vertStride);

			bool useQuantizedAabbCompression = true;

			float minX = NUM_VERTS_X * TRIANGLE_SIZE * 0.5f;
			float minZ = NUM_VERTS_Y * TRIANGLE_SIZE * 0.5f;

			//OptimizedBvh bvh = new OptimizedBvh();
	        Vector3 aabbMin = new Vector3(-minX,-5,-minZ);
	        Vector3 aabbMax = new Vector3(minX,5,minZ);




            m_trimeshShape = new BvhTriangleMeshShape(indexVertexArrays, useQuantizedAabbCompression, ref aabbMin, ref aabbMax, true);
            //m_trimeshShape = new BvhTriangleMeshShape(indexVertexArrays, useQuantizedAabbCompression,true);
	        Vector3 scaling = Vector3.One;
			CollisionShape groundShape = m_trimeshShape;
			//groundShape = new TriangleMeshShape(indexVertexArrays);
			//groundShape = new StaticPlaneShape(Vector3.Up, 0f);
            Vector3 up = new Vector3(0.4f,1,0);
            up.Normalize();
			//groundShape = new StaticPlaneShape(up, 0f);
			//groundShape = new BoxShape(new Vector3(1000, 10, 1000));
			m_collisionConfiguration = new DefaultCollisionConfiguration();
			m_dispatcher = new	CollisionDispatcher(m_collisionConfiguration);

			Vector3 worldMin = aabbMin;
			Vector3 worldMax = aabbMax;

			m_broadphase = new AxisSweep3Internal(ref worldMin, ref worldMax, 0xfffe, 0xffff, 16384, null, false);
			m_constraintSolver = new SequentialImpulseConstraintSolver();
			m_dynamicsWorld = new DiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);

			float mass = 0f;
			Matrix startTransform = Matrix.CreateTranslation(new Vector3(2,-2,0));


			startTransform = Matrix.Identity;
			staticBody = LocalCreateRigidBody(mass, ref startTransform,groundShape);

			staticBody.SetCollisionFlags(staticBody.GetCollisionFlags() | CollisionFlags.CF_KINEMATIC_OBJECT);//STATIC_OBJECT);

			//enable custom material callback
			staticBody.SetCollisionFlags(staticBody.GetCollisionFlags() | CollisionFlags.CF_CUSTOM_MATERIAL_CALLBACK);

			//m_raycastBar = new btRaycastBar (m_debugDraw,4000.0f, 0.0f,-1000,30);
			m_raycastBar = new btRaycastBar(m_debugDraw, 300.0f, 0.0f, -1000, 200,worldMin.X,worldMax.X);
			m_raycastBar.min_x = -100;
			m_raycastBar.max_x = -100;

            ClientResetScene();

        }

		//-----------------------------------------------------------------------------------------------

		public override void  ClientMoveAndDisplay(Microsoft.Xna.Framework.GameTime gameTime)
		{
            m_raycastBar.m_debugDraw = m_debugDraw;
            base.ClientMoveAndDisplay(gameTime);
            float dt = GetDeltaTimeMicroseconds() * 0.000001f;

			if (m_animatedMesh)
			{
				m_offset+=dt;

                SetVertexPositions(waveheight, m_offset);
				
				int i;
				int j;
				Vector3 aabbMin = MathUtil.MAX_VECTOR;
				Vector3 aabbMax = MathUtil.MIN_VECTOR;

				//for ( i=NUM_VERTS_X/2-3;i<NUM_VERTS_X/2+2;i++)
				//{
				//    for (j=NUM_VERTS_X/2-3;j<NUM_VERTS_Y/2+2;j++)
				//    {
					
				//        MathUtil.vectorMin(gVertices[i+j*NUM_VERTS_X],ref aabbMin);
				//        MathUtil.vectorMax(gVertices[i+j*NUM_VERTS_X],ref aabbMax);

				//        float sin = (float)Math.Sin((float)i + m_offset);
				//        float cos = (float)Math.Cos((float)j + m_offset);


				//        gVertices[i+j*NUM_VERTS_X] = new Vector3((i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
				//            //0.f,
				//            waveheight * sin * cos,
				//            (j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);

				//        MathUtil.vectorMin(gVertices[i + j * NUM_VERTS_X], ref aabbMin);
				//        MathUtil.vectorMax(gVertices[i + j * NUM_VERTS_X], ref aabbMax);

				//    }
				//}

				//m_trimeshShape.partialRefitTree(ref aabbMin,ref aabbMax);

				//clear all contact points involving mesh proxy. Note: this is a slow/unoptimized operation.
				m_dynamicsWorld.GetBroadphase().GetOverlappingPairCache().CleanProxyFromPairs(staticBody.GetBroadphaseHandle(),GetDynamicsWorld().GetDispatcher());
                m_raycastBar.move (dt);
	            m_raycastBar.cast (m_dynamicsWorld);
	            m_raycastBar.draw ();

			}
		}

		static void Main(string[] args)
		{
			using (ConcaveRaycastDemo game = new ConcaveRaycastDemo())
			{
				game.Run();
			}
		}

        private float m_offset;
        public BvhTriangleMeshShape m_trimeshShape = null;
        private bool m_animatedMesh;
        private btRaycastBar m_raycastBar = null;
        private ObjectArray<Vector3> gVertices = null;
        private ObjectArray<int> gIndices = null;
        private BvhTriangleMeshShape trimeshShape =null;
        private RigidBody staticBody = null;
        const float waveheight = 5.0f;
        const float TRIANGLE_SIZE=10.0f;
        const int NUM_VERTS_X = 30;
        const int NUM_VERTS_Y = 30;
        const int totalVerts = NUM_VERTS_X * NUM_VERTS_Y;

    }


/* Scrolls back and forth over terrain */
public class btRaycastBar
{
    public IDebugDraw m_debugDraw;
    const int NUMRAYS_IN_BAR  = 30;
	public Vector3[] source = new Vector3[NUMRAYS_IN_BAR];
	public Vector3[] dest = new Vector3[NUMRAYS_IN_BAR];
	public Vector3[] direction = new Vector3[NUMRAYS_IN_BAR];
	public Vector3[] hit = new Vector3[NUMRAYS_IN_BAR];
	public Vector3[] normal = new Vector3[NUMRAYS_IN_BAR];

	public int frame_counter;
	public int ms;
	public int sum_ms;
	public int sum_ms_samples;
	public int min_ms;
	public int max_ms;

#if USE_BT_CLOCK
	btClock frame_timer;
#endif //USE_BT_CLOCK

	public float dx;
	public float min_x;
	public float max_x;
	public float min_y;
	public float max_y;
	public float sign;

	public btRaycastBar (IDebugDraw debugDraw)
	{
		ms = 0;
		max_ms = 0;
		min_ms = 9999;
		sum_ms_samples = 0;
		sum_ms = 0;
        m_debugDraw = debugDraw;
	}

	public btRaycastBar (IDebugDraw debugDraw,bool unused, float ray_length, float min_z, float max_z, float min_y, float max_y)
	{
        m_debugDraw = debugDraw;
		frame_counter = 0;
		ms = 0;
		max_ms = 0;
		min_ms = 9999;
		sum_ms_samples = 0;
		sum_ms = 0;
		dx = 10.0f;
		min_x = -40;
		max_x = 20;
		this.min_y = min_y;
		this.max_y = max_y;
		sign = 1.0f;
		float dalpha = 2*MathUtil.SIMD_2_PI/NUMRAYS_IN_BAR;
		for (int i = 0; i < NUMRAYS_IN_BAR; i++)
		{
			float z = (max_z-min_z)/(((float)(NUMRAYS_IN_BAR)) * i) + min_z;
			source[i] = new Vector3(min_x, max_y, z);
			dest[i] = new Vector3(min_x + ray_length, min_y, z);
			normal[i] = new Vector3(1.0f, 0.0f, 0.0f);
		}
	}
    //  min_y = -1000, max_y = 10
    public btRaycastBar(IDebugDraw debugDraw, float ray_length, float z, float min_y, float max_y,float minX,float maxX)
	{
        m_debugDraw = debugDraw;
		frame_counter = 0;
		ms = 0;
		max_ms = 0;
		min_ms = 9999;
		sum_ms_samples = 0;
		sum_ms = 0;
		dx = 10.0f;
		//min_x = -20;
		//max_x = 20;
		this.min_x = minX;
		this.max_x = maxX;

		float startX = (minX + maxX) / 2;

		this.min_y = min_y;
		this.max_y = max_y;
		sign = 1.0f;
		float dalpha = MathUtil.SIMD_2_PI/NUMRAYS_IN_BAR;
		for (int i = 0; i < NUMRAYS_IN_BAR; i++)
		{
			float alpha = dalpha * i;
			// rotate around by alpha degrees y 
            Matrix tr = Matrix.CreateFromQuaternion(Quaternion.CreateFromAxisAngle(Vector3.Up,alpha));
			direction[i] = new Vector3(1.0f, 0.0f, 0.0f);
			direction[i] = Vector3.Transform(direction[i],tr);
			direction[i] = direction[i] * ray_length;
			source[i] = new Vector3(startX, max_y, z);
			dest[i] = source[i] + direction[i];
			dest[i].Y = min_y;
			normal[i] = new Vector3(1.0f, 0.0f, 0.0f);
		}
	}

	public void move (float dt)
	{
		if (dt > (1.0f/60.0f))
        {
			dt = 1.0f/60.0f;
        }
		for (int i = 0; i < NUMRAYS_IN_BAR; i++)
		{
            Vector3 tempSource = source[i];
			tempSource.X += dx * dt * sign;
            source[i] = tempSource;

            Vector3 tempDest = dest[i];
			tempDest.X += dx * dt * sign;
            dest[i] = tempDest;
		}
		if (source[0].X < min_x)
        {
			sign = 1.0f;
        }
		else if (source[0].X > max_x)
        {
			sign = -1.0f;
        }
	}

	public void cast (CollisionWorld cw)
	{
#if USE_BT_CLOCK
		frame_timer.reset ();
#endif //USE_BT_CLOCK

#if BATCH_RAYCASTER
		if (gBatchRaycaster == null)
        {
			return;
        }

		gBatchRaycaster.clearRays ();
		for (int i = 0; i < NUMRAYS_IN_BAR; i++)
		{
			gBatchRaycaster.addRay (source[i], dest[i]);
		}
		gBatchRaycaster.performBatchRaycast ();
		for (int i = 0; i < gBatchRaycaster.getNumRays (); i++)
		{
			const SpuRaycastTaskWorkUnitOut& outResult = (*gBatchRaycaster)[i];
			hit[i].setInterpolate3(source[i],dest[i],outResult.hitFraction);
			normal[i] = outResult.hitNormal;
			normal[i] = normal[i].Normalize();
		}
#else
		for (int i = 0; i < NUMRAYS_IN_BAR; i++)
		{
			ClosestRayResultCallback cb = new ClosestRayResultCallback(source[i], dest[i]);
			
			cw.RayTest (ref source[i], ref dest[i], cb);
			if (cb.HasHit ())
			{
				hit[i] = cb.m_hitPointWorld;
				normal[i] = cb.m_hitNormalWorld;
			    normal[i] = Vector3.Normalize(normal[i]);
			} 
            else 
            {
				hit[i] = dest[i];
				normal[i] = new Vector3(1.0f, 0.0f, 0.0f);
			}

		}
#if USE_BT_CLOCK
		ms += frame_timer.getTimeMilliseconds ();
#endif //USE_BT_CLOCK
		frame_counter++;
		if (frame_counter > 50)
		{
			min_ms = ms < min_ms ? ms : min_ms;
			max_ms = ms > max_ms ? ms : max_ms;
			sum_ms += ms;
			sum_ms_samples++;
			float mean_ms = (float)sum_ms/(float)sum_ms_samples;
            //printf("%d rays in %d ms %d %d %f\n", NUMRAYS_IN_BAR * frame_counter, ms, min_ms, max_ms, mean_ms);
			ms = 0;
			frame_counter = 0;
		}
#endif
	}

	public void draw ()
	{
        if(m_debugDraw != null)
        {
		    int i;

		    for (i = 0; i < NUMRAYS_IN_BAR; i++)
		    {
                m_debugDraw.DrawLine(source[i],hit[i],new Vector3(0,1,0));
		    }
		    for (i = 0; i < NUMRAYS_IN_BAR; i++)
		    {
                m_debugDraw.DrawLine(hit[i],hit[i]+(normal[i] * 5),new Vector3(1,1,1));
		    }
            Vector3 pointSize = new Vector3(0.5f,0.5f,0.5f);
		    for ( i = 0; i < NUMRAYS_IN_BAR; i++)
		    {
                m_debugDraw.DrawLine(hit[i],hit[i]+pointSize,new Vector3(0,1,1));
		    }
        }
	}
}

}
