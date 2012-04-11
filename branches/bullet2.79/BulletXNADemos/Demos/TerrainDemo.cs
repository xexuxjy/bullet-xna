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
using System.Diagnostics;
using BulletXNA;
using BulletXNA.BulletCollision;
using BulletXNA.BulletDynamics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;
using BulletXNA.LinearMath;

namespace BulletXNADemos.Demos
{
    public class TerrainDemo : DemoApplication
    {
        public TerrainDemo()
        {
            m_collisionConfiguration = null;
            m_dispatcher = null;
            m_broadphase = null;
            m_constraintSolver= null;
            m_upAxis = 1;
            m_type = PHY_ScalarType.PHY_FLOAT;
            m_model = eTerrainModel.eFractal;
            m_rawHeightfieldData= null;
            m_phase = 0.0f;
            m_isDynamic = true;
        }

        ~TerrainDemo()
        {

        }


        public override void InitializeDemo()
        {
        //	std::cerr << "initializing...\n";

            m_nearClip = 1f;
            m_farClip = 1000f;

            m_aspect = m_glutScreenWidth / m_glutScreenHeight;
            m_perspective = IndexedMatrix.CreatePerspectiveFieldOfView(MathHelper.ToRadians(40.0f), m_aspect, m_nearClip, m_farClip);


	        // set up basic state
	        m_upAxis = 1;		// start with Y-axis as "up"
	        m_type = PHY_ScalarType.PHY_FLOAT;
            //m_model = eTerrainModel.eRadial;//eFractal;
	        m_isDynamic = false;

	        // set up the physics world
	        m_collisionConfiguration = new DefaultCollisionConfiguration();
	        m_dispatcher = new CollisionDispatcher(m_collisionConfiguration);
	        IndexedVector3 worldMin = new IndexedVector3(-1000,-1000,-1000);
            IndexedVector3 worldMax = new IndexedVector3(1000, 1000, 1000);
            //m_broadphase = new AxisSweep3Internal(ref worldMin,ref worldMax);
            m_broadphase = new AxisSweep3Internal(ref worldMin, ref worldMax, 0xfffe, 0xffff, 16384, null, false);

	        m_constraintSolver = new SequentialImpulseConstraintSolver();
	        m_dynamicsWorld = new DiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_constraintSolver,m_collisionConfiguration);

	        // initialize axis- or type-dependent physics from here
            ClientResetScene();
        }

        ////////////////////////////////////////////////////////////////////////////////
        //
        //	TerrainDemo -- private helper methods
        //
        ////////////////////////////////////////////////////////////////////////////////

        /// called whenever key terrain attribute is changed
        public override void ClientResetScene()
        {
            base.ClientResetScene();
	        // remove old heightfield
	        m_rawHeightfieldData = null;

	        // reset gravity to point in appropriate direction
            //m_dynamicsWorld.setGravity(getUpVector(m_upAxis, 0.0f, -s_gravity));
            m_dynamicsWorld.SetGravity(ref m_defaultGravity);

	        // get new heightfield of appropriate type
	        m_rawHeightfieldData =
	            GetRawHeightfieldData(m_model, m_type, ref m_minHeight, ref m_maxHeight);
	        Debug.Assert(m_rawHeightfieldData != null , "failed to create raw heightfield");

            if (m_terrainRigidBody != null)
            {
                m_dynamicsWorld.RemoveCollisionObject(m_terrainRigidBody);
                m_terrainRigidBody = null;
                m_terrainShape = null;
                m_collisionShapes.Remove(m_terrainShape);
            }

	        bool flipQuadEdges = false;
	        m_terrainShape =
	            new HeightfieldTerrainShape(s_gridSize, s_gridSize,
					          m_rawHeightfieldData,
					          s_gridHeightScale,
					          m_minHeight, m_maxHeight,
					          m_upAxis, m_type, flipQuadEdges);

            m_terrainShape.RebuildQuadTree(5,2);

	        Debug.Assert(m_terrainShape != null, "null heightfield");

	        // scale the shape
	        IndexedVector3 localScaling = GetUpVector(m_upAxis, s_gridSpacing, 1.0f);
            m_terrainShape.SetLocalScaling(ref localScaling);

	        // stash this shape away
	        m_collisionShapes.Add(m_terrainShape);

	        // set origin to middle of heightfield
	        IndexedMatrix tr = IndexedMatrix.CreateTranslation(new IndexedVector3(0,-20,0));

	        // create ground object
	        float mass = 0.0f;
            m_terrainRigidBody = LocalCreateRigidBody(mass, ref tr, m_terrainShape);

            CollisionShape sphere = new SphereShape(0.5f);
            tr = IndexedMatrix.CreateTranslation(new IndexedVector3(0, 0, 0));

            LocalCreateRigidBody(1f,ref tr,sphere);


        }

        public override void ShutdownDemo()
        {
            // delete raw heightfield data
            m_rawHeightfieldData = null;
        }


        public String GetTerrainTypeName(eTerrainModel model)
        {
	        switch (model) 
            {
	        case eTerrainModel.eRadial:
		        return "Radial";

            case eTerrainModel.eFractal:
		        return "Fractal";

	        default:
		        Debug.Assert(false,"bad terrain model type");
                break;
	        }

	        return null;
        }

        public String GetDataTypeName(PHY_ScalarType type)
        {
	        switch (type) 
            {
                case PHY_ScalarType.PHY_UCHAR:
		            return "UnsignedChar";

                case PHY_ScalarType.PHY_SHORT:
		            return "Short";

                case PHY_ScalarType.PHY_FLOAT:
		            return "Float";

	        default:
		        Debug.Assert(false,"bad heightfield data type");
                break;
	        }

	        return null;
        }

        public String GetUpAxisName(int axis)
        {
	        switch (axis) 
            {
	        case 0:
		        return "X";

	        case 1:
		        return "Y";

	        case 2:
		        return "Z";

	        default:
		        Debug.Assert(false,"bad up axis");
                break;
	        }

            return null ;
        }

        public IndexedVector3 GetUpVector(int upAxis,float regularValue,float upValue)
        {
	        Debug.Assert(upAxis >= 0 && upAxis <= 2,  "bad up axis");

	        IndexedVector3 v = new IndexedVector3(regularValue, regularValue, regularValue);
            v[upAxis] = upValue;

	        return v;
        }

        // TODO: it would probably cleaner to have a struct per data type, so
        // 	you could lookup byte sizes, conversion functions, etc.
        public int GetByteSize(PHY_ScalarType type)
        {
            int size = 0;

            switch (type) {
            case PHY_ScalarType.PHY_FLOAT:
                size = 4;
                break;

            case PHY_ScalarType.PHY_UCHAR:
                size = 1;
                break;

            case PHY_ScalarType.PHY_SHORT:
                size = 2;
                break;

            default:
                Debug.Assert(false,"Bad heightfield data type");
                break;
            }

            return size;
        }



        public float ConvertToFloat(byte[] p,int pindex,PHY_ScalarType type)
        {
            Debug.Assert(p != null);

            switch (type) 
            {
            case PHY_ScalarType.PHY_FLOAT:
                {
                    return BitConverter.ToSingle(p,pindex);
                }

            case PHY_ScalarType.PHY_UCHAR:
                {
                    return p[pindex] * s_gridHeightScale;
                }

            case PHY_ScalarType.PHY_SHORT:
                {
                    short temp = BitConverter.ToInt16(p, pindex);
                    return ((temp) * s_gridHeightScale);
                }

            default:
                Debug.Assert(false,"bad type");
                break;
            }

            return 0;
        }



        public float GetGridHeight(byte[] grid,int i,int j,PHY_ScalarType type)
        {
            Debug.Assert(grid != null);
            Debug.Assert(i >= 0 && i < s_gridSize);
            Debug.Assert(j >= 0 && j < s_gridSize);

            int bpe = GetByteSize(type);
            Debug.Assert(bpe > 0,"bad bytes per element");

            int idx = (j * s_gridSize) + i;
            int offset = ((int)bpe) * idx;

            //byte_t* p = grid + offset;

            return ConvertToFloat(grid,offset, type);
        }



        public static void ConvertFromFloat(byte[] p,int pindex, float value, PHY_ScalarType type)
        {
            Debug.Assert(p != null ,"null");

            switch (type) 
            {
            case PHY_ScalarType.PHY_FLOAT:
                {
                    byte[] temp = BitConverter.GetBytes(value);
                    Array.Copy(temp,0,p,pindex,temp.Length);
                }
                break;

            case PHY_ScalarType.PHY_UCHAR:
                {
                    p[pindex] = (byte) (value / s_gridHeightScale);
                }
                break;

            case PHY_ScalarType.PHY_SHORT:
                {
                    short temp = (short) (value / s_gridHeightScale);
                    byte[] temp2 = BitConverter.GetBytes(temp);
                    Array.Copy(temp2, 0, p, pindex, temp2.Length);
                }
                break;

            default:
                Debug.Assert(false,"bad type");
                break;
            }
        }


        // creates a radially-varying heightfield
        public void SetRadial(byte[] grid,int bytesPerElement,PHY_ScalarType type)
        {
            SetRadial(grid,bytesPerElement,type,0.0f);
        }

        public void SetRadial(byte[] grid,int bytesPerElement,PHY_ScalarType type,float phase)
        {
	        Debug.Assert(grid != null);
	        Debug.Assert(bytesPerElement > 0);

	        // min/max
	        float period = 0.5f / s_gridSpacing;
	        float floor = 0.0f;
	        float min_r = (float)(3.0f * Math.Sqrt(s_gridSpacing));
	        float magnitude = (float)(50.0f * Math.Sqrt(s_gridSpacing));

	        // pick a base_phase such that phase = 0 results in max height
	        //   (this way, if you create a heightfield with phase = 0,
	        //    you can rely on the min/max heights that result)
	        float base_phase = (0.5f * MathUtil.SIMD_PI) - (period * min_r);
	        phase += base_phase;

	        // center of grid
	        float cx = 0.5f * s_gridSize * s_gridSpacing;
	        float cy = cx;		// assume square grid
	        byte[] p = grid;
            int pindex = 0;
	        for (int i = 0; i < s_gridSize; ++i) 
            {
		        float x = i * s_gridSpacing;
		        for (int j = 0; j < s_gridSize; ++j) 
                {
			        float y = j * s_gridSpacing;

			        float dx = x - cx;
			        float dy = y - cy;

			        float r = (float)Math.Sqrt((dx * dx) + (dy * dy));

			        float z = period;
			        if (r < min_r) 
                    {
				        r = min_r;
			        }
			        z = (float)((1.0f / r) * Math.Sin(period * r + phase));
			        if (z > period) 
                    {
				        z = period;
			        } else if (z < -period) 
                    {
				        z = -period;
			        }
			        z = floor + magnitude * z;

			        ConvertFromFloat(p,pindex, z, type);
			        pindex += bytesPerElement;
		        }
	        }
        }



        public float RandomHeight(int step)
        {
            return (0.33f * s_gridSpacing * s_gridSize * step * (m_random.Next(m_randomMax) - (0.5f * m_randomMax))) / (1.0f * m_randomMax * s_gridSize);
        }

        public void UpdateHeight(byte[] p,int index,float new_val,PHY_ScalarType type)
        {
            float old_val = ConvertToFloat(p,index, type);
            //if (old_val != 0.0f)
            {
                ConvertFromFloat(p, index,new_val, type);
            }
        }



        // creates a random, fractal heightfield
        public void SetFractal(byte[] grid,int gridIndex,int bytesPerElement,PHY_ScalarType type,int step)
        {
	        Debug.Assert(grid != null);
	        Debug.Assert(bytesPerElement > 0);
	        Debug.Assert(step > 0);
	        Debug.Assert(step < s_gridSize);

	        int newStep = step / 2;
        //	std::cerr << "Computing grid with step = " << step << ": before\n";
        //	dumpGrid(grid, bytesPerElement, type, step + 1);

	        // special case: starting (must set four corners)
	        if (s_gridSize - 1 == step) {
		        // pick a non-zero (possibly negative) base elevation for testing
		        float baseValue = RandomHeight(step / 2);

                ConvertFromFloat(grid ,gridIndex, baseValue, type);
                ConvertFromFloat(grid ,gridIndex + step * bytesPerElement, baseValue, type);
                ConvertFromFloat(grid ,gridIndex + step * s_gridSize * bytesPerElement, baseValue, type);
                ConvertFromFloat(grid ,gridIndex + (step * s_gridSize + step) * bytesPerElement, baseValue, type);
	        }

	        // determine elevation of each corner
	        float c00 = ConvertToFloat(grid,gridIndex, type);
            float c01 = ConvertToFloat(grid, gridIndex + step * bytesPerElement, type);
            float c10 = ConvertToFloat(grid, gridIndex + (step * s_gridSize) * bytesPerElement, type);
            float c11 = ConvertToFloat(grid, gridIndex + (step * s_gridSize + step) * bytesPerElement, type);

	        // set top middle
            UpdateHeight(grid, gridIndex + newStep * bytesPerElement, 0.5f * (c00 + c01) + RandomHeight(step), type);

	        // set left middle
            UpdateHeight(grid, gridIndex + (newStep * s_gridSize) * bytesPerElement, 0.5f * (c00 + c10) + RandomHeight(step), type);

	        // set right middle
            UpdateHeight(grid, gridIndex + (newStep * s_gridSize + step) * bytesPerElement, 0.5f * (c01 + c11) + RandomHeight(step), type);

	        // set bottom middle
            UpdateHeight(grid, gridIndex + (step * s_gridSize + newStep) * bytesPerElement, 0.5f * (c10 + c11) + RandomHeight(step), type);

	        // set middle
            UpdateHeight(grid, gridIndex + (newStep * s_gridSize + newStep) * bytesPerElement, 0.25f * (c00 + c01 + c10 + c11) + RandomHeight(step), type);

        //	std::cerr << "Computing grid with step = " << step << ": after\n";
        //	dumpGrid(grid, bytesPerElement, type, step + 1);

	        // terminate?
	        if (newStep < 2) 
            {
		        return;
	        }

	        // recurse
	        SetFractal(grid,gridIndex, bytesPerElement, type, newStep);
            SetFractal(grid, gridIndex + newStep * bytesPerElement, bytesPerElement, type, newStep);
            SetFractal(grid, gridIndex + (newStep * s_gridSize) * bytesPerElement, bytesPerElement, type, newStep);
            SetFractal(grid, gridIndex + ((newStep * s_gridSize) + newStep) * bytesPerElement, bytesPerElement, type, newStep);
        }


        public byte[] GetRawHeightfieldData(eTerrainModel model,PHY_ScalarType type,ref float minHeight,ref float maxHeight)
        {
        //	std::cerr << "\nRegenerating terrain\n";
        //	std::cerr << "  model = " << model << "\n";
        //	std::cerr << "  type = " << type << "\n";

	        long nElements = ((long) s_gridSize) * s_gridSize;
        //	std::cerr << "  nElements = " << nElements << "\n";

	        int bytesPerElement = GetByteSize(type);
        //	std::cerr << "  bytesPerElement = " << bytesPerElement << "\n";
	        Debug.Assert(bytesPerElement > 0,"bad bytes per element");

	        long nBytes = nElements * bytesPerElement;
        //	std::cerr << "  nBytes = " << nBytes << "\n";
	        byte[] raw = new byte[nBytes];
	        Debug.Assert(raw != null ,"out of memory");

	        // reseed randomization every 30 seconds
        //	srand(time(NULL) / 30);

	        // populate based on model
	        switch (model) 
            {
	        case eTerrainModel.eRadial:
		        SetRadial(raw, bytesPerElement, type);
		        break;

            case eTerrainModel.eFractal:
		        for (int i = 0; i < nBytes; i++)
		        {
			        raw[i] = 0;
		        }
		        SetFractal(raw,0, bytesPerElement, type, s_gridSize - 1);
		        break;

	        default:
		        Debug.Assert(false,"bad model type");
                break;
	        }

	        if (false) 
            {
		        // inside if(0) so it keeps compiling but isn't
		        // 	exercised and doesn't cause warnings
        //		std::cerr << "final grid:\n";
		        DumpGrid(raw,bytesPerElement, type, s_gridSize - 1);
	        }

	        // find min/max
	        for (int i = 0; i < s_gridSize; ++i) 
            {
		        for (int j = 0; j < s_gridSize; ++j) 
                {
			        float z = GetGridHeight(raw, i, j, type);
        //			std::cerr << "i=" << i << ", j=" << j << ": z=" << z << "\n";

			        // update min/max
			        if (i == 0 && j == 0) 
                    {
				        minHeight = z;
				        maxHeight = z;
			        } 
                    else 
                    {
				        if (z < minHeight) 
                        {
					        minHeight = z;
				        }
				        if (z > maxHeight) 
                        {
					        maxHeight = z;
				        }
			        }
		        }
	        }

	        if (maxHeight < -minHeight) 
            {
		        maxHeight = -minHeight;
	        }
	        if (minHeight > -maxHeight) 
            {
		        minHeight = -maxHeight;
	        }

        //	std::cerr << "  minHeight = " << minHeight << "\n";
        //	std::cerr << "  maxHeight = " << maxHeight << "\n";

	        return raw;
        }

        public void DumpGrid(byte[] grid,int bytesPerElement,PHY_ScalarType type,int max)
        {
	        //std::cerr << "Grid:\n";
	        for (int j = 0; j < max; ++j) 
            {
		        for (int i = 0; i < max; ++i) 
                {
			        long offset = j * s_gridSize + i;
			        float z = ConvertToFloat(grid, (int)(offset * bytesPerElement), type);
                    //sprintf(buffer, "%6.2f", z);
			        //std::cerr << "  " << buffer;
		        }
		        //std::cerr << "\n";
	        }
        }

        public override void KeyboardCallback(Keys key,int x,int y,GameTime gameTime,bool released,ref KeyboardState newState, ref KeyboardState oldState)
        {

	        if (Keys.OemComma == key) 
            {
		        // increment model
		        m_model = (eTerrainModel.eFractal == m_model) ? eTerrainModel.eRadial : eTerrainModel.eFractal;
                ClientResetScene();
	        }
	        else if (Keys.OemQuestion == key) 
            {
		        // increment type
		        m_type = NextType(m_type);
                ClientResetScene();
	        }
            else if (Keys.OemBackslash == key) 
            {
		        // increment axis
		        m_upAxis++;
		        if (m_upAxis > 2) 
                {
			        m_upAxis = 0;
		        }
                ClientResetScene();
	        }
            else if (Keys.OemOpenBrackets == key) 
            {
		        // toggle dynamics
		        m_isDynamic = !m_isDynamic;
	        }

	        // let demo base class handle!
	        base.KeyboardCallback(key, x, y,gameTime,released,ref newState,ref oldState);
        }

        public override void ClientMoveAndDisplay(GameTime gameTime)
        {
		    if (m_rawHeightfieldData != null && m_isDynamic && eTerrainModel.eRadial == m_model) 
            {
                float ms = (float)gameTime.ElapsedGameTime.TotalSeconds;

			    m_phase += s_deltaPhase * ms;
			    if (m_phase > 2.0f * MathUtil.SIMD_PI) 
                {
                    m_phase -= 2.0f * MathUtil.SIMD_PI;
			    }
			    int bpe = GetByteSize(m_type);
			    Debug.Assert(bpe > 0 ,"Bad bytes per element");
			    SetRadial(m_rawHeightfieldData, bpe, m_type, m_phase);
		    }
            TestRay();

            base.ClientMoveAndDisplay(gameTime);
        }


        public PHY_ScalarType NextType(PHY_ScalarType type)
        {
            switch (type)
            {
                case PHY_ScalarType.PHY_FLOAT:
                    return PHY_ScalarType.PHY_SHORT;
                case PHY_ScalarType.PHY_SHORT:
                    return PHY_ScalarType.PHY_UCHAR;
                case PHY_ScalarType.PHY_UCHAR:
                    return PHY_ScalarType.PHY_FLOAT;
            }
            return PHY_ScalarType.PHY_FLOAT;
        }

        static void Main(string[] args)
        {
            using (TerrainDemo game = new TerrainDemo())
            {
                game.Run();
            }
        }


        private int m_upAxis;
        private PHY_ScalarType m_type;
        private eTerrainModel m_model;
        private byte[] m_rawHeightfieldData;
        private float m_minHeight;
        private float m_maxHeight;
        private float m_phase;	// for dynamics
        private bool m_isDynamic;
        private HeightfieldTerrainShape m_terrainShape = null;
        private RigidBody m_terrainRigidBody = null;


        private Random m_random = new Random();
        const int m_randomMax = 0x7fff;
        const int s_gridSize = 64 + 1;  // must be (2^N) + 1
        const float s_gridSpacing = 5.0f;

        const float s_gridHeightScale = 0.2f;

        // the singularity at the center of the radial model means we need a lot of
        //   finely-spaced time steps to get the physics right.
        // These numbers are probably too aggressive for a real game!
        const int s_requestedHz = 180;
        const float s_engineTimeStep = 1.0f / s_requestedHz;

        // delta phase: radians per second
        const float s_deltaPhase = 0.25f * 2.0f * MathUtil.SIMD_PI;

        // what type of terrain is generated?
        public enum eTerrainModel
        {
            eRadial = 1,	// deterministic
            eFractal = 2	// random
        };

        public void TestRay()
        {
            // add something to draw and test collision?
            if (true)
            {
                    int rayLength = 1000;
                    int normalLength = 10;
                    IndexedVector3 startPos = m_cameraPosition;
                    IndexedVector3 direction = -m_lookAt._basis[2];
                    IndexedVector3 right = m_lookAt._basis[0];
                    IndexedVector3 endPos = startPos + (direction * rayLength);

                    Vector3 collisionPoint = Vector3.Zero;
                    Vector3 collisionNormal = Vector3.Zero;

                    if (BulletGlobals.gDebugDraw != null)
                    {
                        IndexedVector3 rayColor = new IndexedVector3(1, 1, 1);
                        IndexedVector3 normalColor = new IndexedVector3(1, 0, 0);
                        
                        
                        // offset the pos by the camera right a b it
                        BulletGlobals.gDebugDraw.DrawLine(startPos+(right * 10), endPos, rayColor);
                        BulletGlobals.gDebugDraw.DrawLine(startPos - (right * 10), endPos, rayColor);


                        IndexedVector3 location = new IndexedVector3(20, 0, 20);
                        IndexedVector3 colour = new IndexedVector3(1, 1, 1);

                        ClosestRayResultCallback cb = new ClosestRayResultCallback(ref startPos, ref endPos);
                        m_dynamicsWorld.RayTest(ref startPos, ref endPos, cb);

                        if (cb.HasHit())
                        {
                            collisionPoint = cb.m_hitPointWorld;
                            collisionNormal = cb.m_hitNormalWorld;

                            
                            Vector3 normalStart = collisionPoint;
                            Vector3 normalEnd = normalStart + (collisionNormal * normalLength);
                            BulletGlobals.gDebugDraw.DrawLine(normalStart, normalEnd, normalColor);
                            BulletGlobals.gDebugDraw.DrawText(String.Format("Camera Pos[{0} Forward[{1}] Collide[{2}] Normal[{3}].", startPos, direction, collisionPoint, collisionNormal), location, colour);

                        }

                    }
                }



        }

    }
}
