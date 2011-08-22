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
using System.Diagnostics;
using BulletXNA;
using BulletXNA.BulletCollision;
using BulletXNA.LinearMath;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace BulletXNADemos.Demos
{
	public class Edge 
    { 
        public IndexedVector3[] n = new IndexedVector3[2];
        public int[] v = new int[2]; 
    }
    
    public class ShapeCache
	{
	    public ShapeCache(ConvexShape s)
        {
            m_shapehull = new ShapeHull(s);
        }

	    public ShapeHull m_shapehull;
	    public IList<Edge>	m_edges = new ObjectArray<Edge>();
	};
	//clean-up memory of dynamically created shape hulls


    public class XNA_ShapeDrawer : IDebugDraw
    {

        public XNA_ShapeDrawer(DemoApplication game)
        {
            m_game = game;
        }


        public void LoadContent()
        {
			m_debugEffect = new BasicEffect(m_game.GraphicsDevice);
			m_debugEffect.VertexColorEnabled = true;

            m_game.Content.RootDirectory = "./content";
			//m_effect = m_game.Content.Load<Effect>("Standard");
			m_modelEffect = new BasicEffect(m_game.GraphicsDevice);
            m_vertexEffect = new BasicEffect(m_game.GraphicsDevice);
            

            m_spriteBatch = new SpriteBatch(m_game.GraphicsDevice);
            m_spriteFont = m_game.Content.Load<SpriteFont>("DebugFont8");
            m_vertexDeclaration = VertexPositionNormalTexture.VertexDeclaration;
            m_lineVertexDeclaration = VertexPositionColor.VertexDeclaration;
            m_lightModel = m_game.Content.Load<Model>("SphereHighPoly");
            m_lightTexture = m_game.Content.Load<Texture2D>("whitetexture");
			m_cubeModel = m_game.Content.Load<Model>("unitcube");
			m_sphereModel = m_game.Content.Load<Model>("unitsphere");
			m_cylinderModel = m_game.Content.Load<Model>("unitcylinder");
			//m_coneModel = m_game.Content.Load<Model>("unitcone");

			m_coneModel = m_game.Content.Load<Model>("unitcube");

			m_modelEffect.TextureEnabled = true;
			m_modelEffect.EnableDefaultLighting();

			RemapModel(m_cubeModel, m_modelEffect);
            RemapModel(m_sphereModel, m_modelEffect);
            RemapModel(m_cylinderModel, m_modelEffect);
            RemapModel(m_coneModel, m_modelEffect);

            if (m_textureEnabled && (!m_textureInitialized))
            {
                int textureWidth = 256;
                int textureBreadth = 256;
                Color[] image = new Color[(textureWidth * textureBreadth)];
                m_generatedTexture = new Texture2D(m_game.GraphicsDevice, textureWidth, textureBreadth, false, SurfaceFormat.Color);
                for (int y = 0; y < textureBreadth; ++y)
                {
                    byte t = (byte)(y >> 4);
                    int piIndex = (y * textureBreadth);
                    for (int x = 0; x < textureWidth; ++x)
                    {
                        byte s = (byte)(x >> 4);
                        byte b = 180;
                        byte c = (byte)(b + ((s + t & 1) & 1) * (255 - b));

                        Color col = new Color(c, c, c);
                        image[piIndex + x] = col;
                    }
                }

                m_generatedTexture.SetData<Color>(image);
                m_textureInitialized = true;
                m_vertexEffect.Texture = m_generatedTexture;
                m_vertexEffect.TextureEnabled = true;
                m_vertexEffect.EnableDefaultLighting();

            }

            // Establish viewport positions for the main scene 
            // and depth texture display
            m_defaultViewport = m_game.GraphicsDevice.Viewport;
            m_pipViewport = m_game.GraphicsDevice.Viewport;
            m_pipViewport.Height /= 3;
            m_pipViewport.Width /= 3;
            m_pipViewport.X =
                m_game.GraphicsDevice.Viewport.Width - m_pipViewport.Width - 50;
            m_pipViewport.Y =
                m_game.GraphicsDevice.Viewport.Height - m_pipViewport.Height - 50;


            //RasterizerState rasterizerState = new RasterizerState();
            //rasterizerState.CullMode = CullMode.CullClockwiseFace;
            //m_game.GraphicsDevice.RasterizerState = rasterizerState;
            //m_game.GraphicsDevice.BlendState = BlendState.AlphaBlend;
        
        }
        
        
        public bool EnableTexture(bool flag)
        {
            m_textureEnabled = flag;
            return HasTextureEnabled();
        }

        public bool HasTextureEnabled()
        {
            return m_textureEnabled;
        }

        //public void startDraw(GraphicsDevice graphicsDevice,ref IndexedMatrix view, ref IndexedMatrix projection)
        //{
        //    ((DefaultDebugDraw)m_debugDraw).update(graphicsDevice, ref view, ref projection);
        //}
        public virtual void DrawShadow(IndexedMatrix m, IndexedVector3 extrusion, CollisionShape shape, IndexedVector3 worldBoundsMin, IndexedVector3 worldBoundsMax)
        {
            DrawShadow(m, extrusion, shape, worldBoundsMin, worldBoundsMax);
        }

		public virtual void	DrawShadow(ref IndexedMatrix m, ref IndexedVector3 extrusion,CollisionShape shape,ref IndexedVector3 worldBoundsMin,ref IndexedVector3 worldBoundsMax)
        {
	        if(shape.GetShapeType() == BroadphaseNativeTypes.UNIFORM_SCALING_SHAPE_PROXYTYPE)
	        {
		        UniformScalingShape scalingShape = (UniformScalingShape)(shape);
		        ConvexShape convexShape = scalingShape.GetChildShape();
		        float	scalingFactor = (float)scalingShape.GetUniformScalingFactor();
		        IndexedMatrix tmpScaling = IndexedMatrix.CreateScale(scalingFactor);
                tmpScaling *= m;
		        DrawShadow(ref tmpScaling,ref extrusion,convexShape,ref worldBoundsMin,ref worldBoundsMax);

		        return;
	        }
	        else if(shape.GetShapeType()==BroadphaseNativeTypes.COMPOUND_SHAPE_PROXYTYPE)
	        {
		        CompoundShape compoundShape = (CompoundShape)(shape);
		        for (int i=compoundShape.GetNumChildShapes()-1;i>=0;i--)
		        {
			        IndexedMatrix childTrans = compoundShape.GetChildTransform(i);
			        CollisionShape colShape = compoundShape.GetChildShape(i);
                    //float childMat[16];
                    //childTrans.getOpenGLMatrix(childMat);
                    IndexedVector3 transformedExtrude = childTrans._basis * extrusion;
			        DrawShadow(ref childTrans,ref transformedExtrude,colShape,ref worldBoundsMin,ref worldBoundsMax);
		        }
	        }
	        else
	        {
		        bool useWireframeFallback = true;
                if (shape.IsConvex())
                {
                    ShapeCache	sc=Cache(shape as ConvexShape);
                    ShapeHull hull  = sc.m_shapehull;
                    //glBegin(GL_QUADS);
                    for(int i=0;i<sc.m_edges.Count;++i)
                    {			
                        float d=IndexedVector3.Dot(sc.m_edges[i].n[0],extrusion);
                        if((d*IndexedVector3.Dot(sc.m_edges[i].n[1],extrusion))<0)
                        {
                            int	q=	d<0?1:0;
                            IndexedVector3	a=	hull.m_vertices[sc.m_edges[i].v[q]];
                            IndexedVector3	b=	hull.m_vertices[sc.m_edges[i].v[1-q]];
                            IndexedVector3 ae = a + extrusion;
                            IndexedVector3 be = b + extrusion;
                            Vector2 tex = new Vector2(0,0);
                            // fix me.
                            IndexedVector3 normal = new IndexedVector3(0,1,0);
                            // gl_quad turned into two triangles.
                            AddVertex(ref a, ref normal,ref tex);
                            AddVertex(ref b, ref normal, ref tex);
                            AddVertex(ref be, ref normal, ref tex);
                            AddVertex(ref be, ref normal, ref tex);
                            AddVertex(ref ae, ref normal, ref tex);
                            AddVertex(ref a, ref normal, ref tex);

                        }
                    }
                    //glEnd();
                }

	        }

	        if (shape.IsConcave())//>getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE||shape.getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
		        //		if (shape.getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
	        {
		        ConcaveShape concaveMesh = (ConcaveShape) shape;

		        XNADrawcallback drawCallback = new XNADrawcallback(this,ref m);
		        drawCallback.m_wireframe = false;

		        concaveMesh.ProcessAllTriangles(drawCallback,ref worldBoundsMin,ref worldBoundsMax);

	        }
            //glPopMatrix();

        }

        public void DrawSolidCube(ref IndexedVector3 halfExtents, ref IndexedMatrix matrix, ref IndexedMatrix view, ref IndexedMatrix projection, ref IndexedVector3 color)
        {
			ModelScalingData modelScalingData = new ModelScalingData(m_cubeModel,halfExtents,matrix);
			modelScalingData.color = color;
			m_modelScalingData.Add(modelScalingData);
        }

        private void AddVertex(IndexedVector3 vec, IndexedVector3 normal, Vector2 tex)
        {
            AddVertex(ref vec,ref normal, ref tex);
        }

        private void AddVertex(ref IndexedVector3 vec, ref IndexedVector3 normal, ref Vector2 tex)
        {
            if(m_texturedVertexCount < m_textureVertexMaxSize-1)
            {
                m_texturedVertices[m_texturedVertexCount++] = new VertexPositionNormalTexture(vec.ToVector3(), normal.ToVector3(),tex);
            }
        }

        public void DrawText(String text, IndexedVector3 position, IndexedVector3 color)
        {
            DrawText(text, ref position, ref color);
        }

        public void DrawText(String text, ref IndexedVector3 position, ref IndexedVector3 color)
        {
            TextPositionColor tpc = new TextPositionColor();
            tpc.m_text = text;
            tpc.m_position = new Vector2(position.X, position.Y);
            tpc.m_color = new Color(color.ToVector3());

            m_textPositionColours.Add(tpc);
        }

        public void DrawSolidTriangle(IndexedVector3[] points)
        {
            IndexedVector3 d = points[1] - points[0];
            IndexedVector3 e = points[2] - points[0];
            // Reverse the cross here to account for winding, shouldn't change the way rest of bullet works.
            //IndexedVector3 normal = IndexedVector3.Cross(d, e);
            IndexedVector3 normal = IndexedVector3.Cross(e, d);

            normal.Normalize();
            //AddVertex(points[0], normal, new Vector2(0, 0));
            //AddVertex(points[1], normal, new Vector2(0, 1));
            //AddVertex(points[2], normal, new Vector2(1, 1));
            AddVertex(points[0], normal, new Vector2(0, 0));
            AddVertex(points[2], normal, new Vector2(1, 1));
            AddVertex(points[1], normal, new Vector2(0, 1));

        }



        //public void drawWireframeCube(float sideLength)
        //{
        //}


        public void DrawSolidSphere(float radius, int slices, int stacks, ref IndexedMatrix matrix, ref IndexedMatrix view, ref IndexedMatrix projection,ref IndexedVector3 color)
        {

			ModelScalingData modelScalingData = new ModelScalingData(m_sphereModel,new IndexedVector3(radius),matrix);
			modelScalingData.color = color;
			m_modelScalingData.Add(modelScalingData);
        }


		public void DrawSolidCone(float height, float radius, ref IndexedMatrix matrix, ref IndexedMatrix view, ref IndexedMatrix projection, ref IndexedVector3 color)
        {
			IndexedVector3 scale = new IndexedVector3(radius, height, radius);
			ModelScalingData modelScalingData = new ModelScalingData(m_coneModel, scale, matrix);
			modelScalingData.color = color;
			m_modelScalingData.Add(modelScalingData);
        }



		public void DrawCylinder(float radius, float halfHeight, int upAxis, ref IndexedMatrix matrix, ref IndexedMatrix view, ref IndexedMatrix projection, ref IndexedVector3 color)
        {
            DrawCylinder(radius, radius, halfHeight, upAxis, ref matrix, ref view, ref projection,ref color);
        }


		public void DrawCylinder(float topRadius, float bottomRadius, float halfHeight, int upAxis, ref IndexedMatrix matrix, ref IndexedMatrix view, ref IndexedMatrix projection, ref IndexedVector3 color)
        {
			IndexedVector3 scale = new IndexedVector3(topRadius, halfHeight, bottomRadius);
			IndexedMatrix rotate = IndexedMatrix.Identity;

            if (upAxis == 0)
            {
                rotate = IndexedMatrix.CreateRotationZ(MathUtil.SIMD_HALF_PI);
            }
            if (upAxis == 1)
			{
				rotate = IndexedMatrix.CreateRotationY(MathUtil.SIMD_HALF_PI);
			}
			else if (upAxis == 2)
			{
				rotate = IndexedMatrix.CreateRotationX(MathUtil.SIMD_HALF_PI);
			}

			IndexedMatrix copy = matrix;
			copy._origin = IndexedVector3.Zero;
			copy = copy * rotate;
            copy._origin = matrix._origin;

			ModelScalingData modelScalingData = new ModelScalingData(m_cylinderModel, scale, copy);
			modelScalingData.color = color;
			m_modelScalingData.Add(modelScalingData);
		}
        
        public void DrawCoordSystem()
        {

        }

        public void DrawXNA(IndexedMatrix m, CollisionShape shape, IndexedVector3 color, DebugDrawModes debugMode, IndexedVector3 worldBoundsMin, IndexedVector3 worldBoundsMax, IndexedMatrix view, IndexedMatrix projection)
        {
            DrawXNA(ref m, shape, ref color, debugMode, ref worldBoundsMin, ref worldBoundsMax, ref view, ref projection);
        }

        public void DrawXNA(ref IndexedMatrix m, CollisionShape shape, ref IndexedVector3 color, DebugDrawModes debugMode, ref IndexedVector3 worldBoundsMin, ref IndexedVector3 worldBoundsMax, ref IndexedMatrix view, ref IndexedMatrix projection)
        {
            //btglMultMatrix(m);
            if (shape == null)
            {
                return;
            }

            if (shape.GetShapeType() == BroadphaseNativeTypes.UNIFORM_SCALING_SHAPE_PROXYTYPE)
            {
                UniformScalingShape scalingShape = (UniformScalingShape)shape;
                ConvexShape convexShape = scalingShape.GetChildShape();
                float scalingFactor = scalingShape.GetUniformScalingFactor();
                IndexedMatrix scaleMatrix = IndexedMatrix.CreateScale(scalingFactor);
                IndexedMatrix finalMatrix = scaleMatrix * m;
                DrawXNA(ref finalMatrix, convexShape, ref color, debugMode, ref worldBoundsMin, ref worldBoundsMax,ref view,ref projection);
                return;
            }
            if (shape.GetShapeType() == BroadphaseNativeTypes.COMPOUND_SHAPE_PROXYTYPE)
            {
                CompoundShape compoundShape = (CompoundShape)shape;
                for (int i = compoundShape.GetNumChildShapes() - 1; i >= 0; i--)
                {
                    IndexedMatrix childTrans = compoundShape.GetChildTransform(i);
                    CollisionShape colShape = compoundShape.GetChildShape(i);
                    IndexedMatrix childMat = childTrans;

					//childMat = MathUtil.bulletMatrixMultiply(m, childMat);
                    //childMat = childMat * m;
                    childMat = m * childMat;

                    
					
					DrawXNA(ref childMat, colShape, ref color, debugMode, ref worldBoundsMin, ref worldBoundsMax,ref view,ref projection);
                }
            }
            else
            {

                bool useWireframeFallback = true;

                if ((debugMode & DebugDrawModes.DBG_DrawWireframe) == 0)
                {
                    ///you can comment out any of the specific cases, and use the default
                    ///the benefit of 'default' is that it approximates the actual collision shape including collision margin
                    //BroadphaseNativeTypes shapetype = m_textureEnabled ? BroadphaseNativeTypes.MAX_BROADPHASE_COLLISION_TYPES : shape.getShapeType();
                    BroadphaseNativeTypes shapetype = shape.GetShapeType();
                    switch (shapetype)
                    {
                        case BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE:
                            {
                                BoxShape boxShape = shape as BoxShape;
                                IndexedVector3 halfExtents = boxShape.GetHalfExtentsWithMargin();

                                DrawSolidCube(ref halfExtents, ref m, ref view, ref projection,ref color);
                                //drawSolidSphere(halfExtents.X, 10, 10, ref m, ref view, ref projection);
                                //drawCylinder(halfExtents.X, halfExtents.Y, 1, ref m, ref view, ref projection);
                                //drawSolidCone(halfExtents.Y, halfExtents.X, ref m, ref view, ref projection);

                                //DrawText("Hello World", new IndexedVector3(20, 20, 0), new IndexedVector3(255, 255, 255));
                                useWireframeFallback = false;
                                break;
                            }


                        case BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE:
                            {
                                SphereShape sphereShape = shape as SphereShape;
                                float radius = sphereShape.GetMargin();//radius doesn't include the margin, so draw with margin
								DrawSolidSphere(radius, 10, 10, ref m, ref view, ref projection, ref color);
                                //glutSolidSphere(radius,10,10);
                                useWireframeFallback = false;
                                break;
                            }
                        case BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE:
		                    {
                                CapsuleShape capsuleShape = shape as CapsuleShape;

			                    float radius = capsuleShape.GetRadius();
			                    float halfHeight = capsuleShape.GetHalfHeight();

			                    int upAxis = capsuleShape.GetUpAxis();

			                    IndexedVector3 capStart = IndexedVector3.Zero;
			                    capStart[upAxis] = -halfHeight;

                                IndexedVector3 capEnd = IndexedVector3.Zero;
                                capEnd[upAxis] = halfHeight;

			                    // Draw the ends
			                    {

				                    IndexedMatrix childTransform = IndexedMatrix.Identity;
                                    childTransform._origin = m * capStart;
									DrawSolidSphere(radius, 5, 5, ref childTransform, ref view, ref projection, ref color);
			                    }

			                    {
                                    IndexedMatrix childTransform = IndexedMatrix.Identity;
                                    childTransform._origin = m * capEnd;
									DrawSolidSphere(radius, 5, 5, ref childTransform, ref view, ref projection, ref color);
                                }

                                DrawCylinder(radius, halfHeight, upAxis, ref m, ref view, ref projection,ref color);
                                break;
		                    }
                        case BroadphaseNativeTypes.CONE_SHAPE_PROXYTYPE:
                            {
                                ConeShape coneShape = (ConeShape)(shape);
                                int upIndex = coneShape.GetConeUpIndex();
                                float radius = coneShape.GetRadius();//+coneShape.getMargin();
                                float height = coneShape.GetHeight();//+coneShape.getMargin();
                                IndexedMatrix rotateMatrix = IndexedMatrix.Identity;


                                switch (upIndex)
                                {
                                    case 0:
                                        rotateMatrix = IndexedMatrix.CreateRotationX(MathUtil.SIMD_HALF_PI);
                                        break;
                                    case 1:
                                        rotateMatrix = IndexedMatrix.CreateRotationX(-MathUtil.SIMD_HALF_PI);
                                        break;
                                    case 2:
                                        break;
                                    default:
                                        {
                                            break;
                                        }
                                };

                                IndexedMatrix translationMatrix = IndexedMatrix.CreateTranslation(0f, 0f, -0.5f * height);

                                IndexedMatrix resultant = translationMatrix * rotateMatrix * m;
								DrawSolidCone(radius, height, ref resultant, ref view, ref projection, ref color);
                                useWireframeFallback = false;
                                break;

                            }


                        case BroadphaseNativeTypes.STATIC_PLANE_PROXYTYPE:
                            {
                                StaticPlaneShape staticPlaneShape = shape as StaticPlaneShape;
                                float planeConst = staticPlaneShape.GetPlaneConstant();
                                IndexedVector3 planeNormal = staticPlaneShape.GetPlaneNormal();
                                IndexedVector3 planeOrigin = planeNormal * planeConst;
                                IndexedVector3 vec0, vec1;
                                TransformUtil.PlaneSpace1(ref planeNormal, out vec0, out vec1);
                                float vecLen = 100f;
                                IndexedVector3 pt0 = planeOrigin + vec0 * vecLen;
                                IndexedVector3 pt1 = planeOrigin - vec0 * vecLen;
                                IndexedVector3 pt2 = planeOrigin + vec1 * vecLen;
                                IndexedVector3 pt3 = planeOrigin - vec1 * vecLen;

                                // Fallback to debug draw - needs tidying
                                IndexedVector3 colour = new IndexedVector3(255, 255, 255);
                                DrawLine(ref pt0, ref pt1, ref colour);
                                DrawLine(ref pt1, ref pt2, ref colour);
                                DrawLine(ref pt2, ref pt3, ref colour);
                                DrawLine(ref pt3, ref pt1, ref colour);

                                break;

                            }

                        case BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE:
                            {
                                CylinderShape cylinder = (CylinderShape)(shape);
                                int upAxis = cylinder.GetUpAxis();

                                float radius = cylinder.GetRadius();
                                float halfHeight = cylinder.GetHalfExtentsWithMargin()[upAxis];
								DrawCylinder(radius, halfHeight, upAxis, ref m, ref view, ref projection, ref color);
                                break;
                            }

                        default:
                            {
                                if (shape.IsConvex())
                                {
                                    ShapeCache	sc=Cache(shape as ConvexShape);

                                    //if (shape.getUserPointer())
                                    {
                                        //glutSolidCube(1.0);
                                        ShapeHull hull = sc.m_shapehull/*(btShapeHull*)shape.getUserPointer()*/;

                                        int numTriangles = hull.NumTriangles();
                                        int numIndices = hull.NumIndices();
                                        int numVertices = hull.NumVertices(); 
                                        if (numTriangles > 0)
                                        {
                                            int index = 0;
                                            IList<int> idx = hull.m_indices;
                                            IList<IndexedVector3> vtx = hull.m_vertices;

                                            for (int i = 0; i < numTriangles; i++)
                                            {
                                                int i1 = index++;
                                                int i2 = index++;
                                                int i3 = index++;
                                                Debug.Assert(i1 < numIndices &&
                                                    i2 < numIndices &&
                                                    i3 < numIndices);

                                                int index1 = idx[i1];
                                                int index2 = idx[i2];
                                                int index3 = idx[i3];
                                                Debug.Assert(index1 < numVertices &&
                                                    index2 < numVertices &&
                                                    index3 < numVertices);

                                                IndexedVector3 v1 = m * vtx[index1];
                                                IndexedVector3 v2 = m * vtx[index2];
                                                IndexedVector3 v3 = m * vtx[index3];
                                                IndexedVector3 normal = IndexedVector3.Cross((v3-v1),(v2-v1));
                                                normal.Normalize();

                                                Vector2 tex = new Vector2(0,0);
                                                AddVertex(ref v1, ref normal,ref tex);
                                                AddVertex(ref v2, ref normal, ref tex);
                                                AddVertex(ref v3, ref normal, ref tex);
                                            }
                                        }
                                    }
                                }
                                break;
                            }
                        }
                    }

                    /// for polyhedral shapes
                    if (debugMode == DebugDrawModes.DBG_DrawFeaturesText && (shape.IsPolyhedral()))
                    {
                        PolyhedralConvexShape polyshape = (PolyhedralConvexShape)shape;
                        {
                            //BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),polyshape.getExtraDebugInfo());

                            IndexedVector3 colour = new IndexedVector3(255, 255, 255);
                            for (int i = 0; i < polyshape.GetNumVertices(); i++)
                            {
                                IndexedVector3 vtx;
                                polyshape.GetVertex(i, out vtx);
                                String buf = " " + i;
                                DrawText(buf, ref vtx, ref colour);
                            }

                            for (int i = 0; i < polyshape.GetNumPlanes(); i++)
                            {
                                IndexedVector3 normal;
                                IndexedVector3 vtx;
                                polyshape.GetPlane(out normal, out vtx, i);
                                float d = IndexedVector3.Dot(vtx, normal);
                                vtx *= d;

                                String buf = " plane " + i;
                                DrawText(buf, ref vtx, ref colour);
                            }
                        }
                    }

                    if (shape.IsConcave() && !shape.IsInfinite())//>getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE||shape.getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
                    //		if (shape.getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
                    {
                        ConcaveShape concaveMesh = shape as ConcaveShape;

                        XNADrawcallback drawCallback = new XNADrawcallback(this,ref m);
                        drawCallback.m_wireframe = (debugMode & DebugDrawModes.DBG_DrawWireframe) != 0;

                        concaveMesh.ProcessAllTriangles(drawCallback, ref worldBoundsMin, ref worldBoundsMax);

                    }

                    //glDisable(GL_DEPTH_TEST);
                    //glRasterPos3f(0,0,0);//mvtx.x(),  vtx.y(),  vtx.z());
                    if ((debugMode & DebugDrawModes.DBG_DrawText) != 0)
                    {
                        IndexedVector3 position = IndexedVector3.Zero;
                        IndexedVector3 colour = new IndexedVector3(255, 255, 255);
                        DrawText(shape.GetName(), ref position, ref colour);
                    }

                    if ((debugMode & DebugDrawModes.DBG_DrawFeaturesText) != 0)
                    {
                        //drawText(shape.getEx]
                        //BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),shape.getExtraDebugInfo());
                    }
                    //glEnable(GL_DEPTH_TEST);

                    ////	glPopMatrix();	
                    //if(m_textureenabled) glDisable(GL_TEXTURE_2D);
                    //  }
                    //    glPopMatrix();
            }
        }

        public ShapeCache Cache(ConvexShape shape)
        {
	        ShapeCache sc=(ShapeCache)shape.GetUserPointer();
	        if(sc == null)
	        {
		        sc=new ShapeCache(shape);
		        sc.m_shapehull.BuildHull(shape.GetMargin());
		        m_shapecaches.Add(sc);
		        shape.SetUserPointer(sc);
		        /* Build edges	*/ 
		        int	ni=sc.m_shapehull.NumIndices();
		        int	nv=sc.m_shapehull.NumVertices();
		        IList<int>	pi=sc.m_shapehull.m_indices;
		        IList<IndexedVector3> pv=sc.m_shapehull.m_vertices;
		        IList<Edge> edges = new ObjectArray<Edge>(ni);
		        for(int i=0;i<ni;i+=3)
		        {
			        IndexedVector3	nrm= IndexedVector3.Normalize(IndexedVector3.Cross(pv[pi[i+1]]-pv[pi[i]],pv[pi[i+2]]-pv[pi[i]]));
			        for(int j=2,k=0;k<3;j=k++)
			        {
				        int	a=pi[i+j];
				        int	b=pi[i+k];
				        Edge e=edges[Math.Min(a,b)*nv+Math.Max(a,b)];
				        if(e != null)
				        {
					        sc.m_edges.Add(new Edge());
					        e=sc.m_edges[sc.m_edges.Count-1];
					        e.n[0]=nrm;e.n[1]=-nrm;
					        e.v[0]=a;e.v[1]=b;
				        }
				        else
				        {
					        e.n[1]=nrm;
				        }
			        }
		        }
	        }
	        return(sc);
        }

        public void RenderStandard(GameTime gameTime, ref IndexedMatrix view, ref IndexedMatrix projection)
        {
            // Always clear?
            m_game.GraphicsDevice.Clear(Color.CornflowerBlue);
            {
				DrawPrimitives(gameTime, ref view, ref projection);
                m_texturedVertexCount = 0;
            }
        }

        public void RenderOthers(GameTime gameTime, IndexedMatrix view, IndexedMatrix projection)
        {
            RenderOthers(gameTime, ref view, ref projection);
        }

        public void RenderOthers(GameTime gameTime, ref IndexedMatrix view, ref IndexedMatrix projection)
        {
            m_spriteBatch.Begin();
            for (int i = 0; i < m_textPositionColours.Count; ++i)
            {
                m_spriteBatch.DrawString(m_spriteFont, m_textPositionColours[i].m_text, m_textPositionColours[i].m_position, m_textPositionColours[i].m_color);
            }

            // set mouse cursor to true so we can see where we're aiming/picking.
            m_game.IsMouseVisible = true;
            //if(m_game.Mou


            m_spriteBatch.End();
            m_textPositionColours.Clear();
            // restore from sprite batch changes.
            m_game.GraphicsDevice.BlendState = BlendState.Opaque;
            m_game.GraphicsDevice.DepthStencilState = DepthStencilState.Default;

        }


        private void DrawPrimitives(GameTime gameTime, ref IndexedMatrix view, ref IndexedMatrix projection)
        {
			if(m_texturedVertexCount > 0)
			{
                m_vertexEffect.View = view.ToMatrix();
                m_vertexEffect.Projection = projection.ToMatrixProjection();

                foreach (EffectPass pass in m_vertexEffect.CurrentTechnique.Passes)
				{
					pass.Apply();
					m_game.GraphicsDevice.DrawUserPrimitives<VertexPositionNormalTexture>(PrimitiveType.TriangleList, m_texturedVertices, 0, m_texturedVertexCount / 3);
				}
			}


            foreach (ModelScalingData modelScalingData in m_modelScalingData)
            {
                //IndexedMatrix scale = IndexedMatrix.CreateScale(modelScalingData.scale);
                Matrix[] transforms = new Matrix[modelScalingData.model.Bones.Count];
                foreach (ModelMesh mesh in modelScalingData.model.Meshes)
                {
                    modelScalingData.model.CopyAbsoluteBoneTransformsTo(transforms);
                    foreach (BasicEffect effect in mesh.Effects)
                    {
						effect.Texture = GetTexture(ref modelScalingData.color);
                        effect.View = view.ToMatrix();
                        effect.Projection = projection.ToMatrixProjection();


                        effect.World = transforms[mesh.ParentBone.Index] * modelScalingData.transform.ToMatrix();
                        //effect.World = modelScalingData.transform.ToMatrix() * transforms[mesh.ParentBone.Index];

                        //if (BulletGlobals.g_streamWriter != null)
                        //{
                        //    MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "draw-bone", transforms[mesh.ParentBone.Index]);
                        //    MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "draw-model", modelScalingData.transform.ToMatrix());
                        //    MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "draw-view", effect.View);
                        //    MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "draw-proj", effect.Projection);
                        //    MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "draw-world", effect.World);
                        //}
                    }
                    mesh.Draw();
                }
            }
			m_modelScalingData.Clear();
        }

        public void RenderDebugLines(GameTime gameTime, ref IndexedMatrix view, ref IndexedMatrix projection)
        {
            m_debugEffect.World = Matrix.Identity;
            m_debugEffect.View = view.ToMatrix();
            m_debugEffect.Projection = projection.ToMatrixProjection();

            if (m_lineIndex > 0)
            {
                //m_game.GraphicsDevice.VertexDeclaration = m_lineVertexDeclaration;
                //m_debugEffect.Begin();
                foreach (EffectPass pass in m_debugEffect.CurrentTechnique.Passes)
                {
                    //pass.Begin();
                    pass.Apply();
                    m_game.GraphicsDevice.DrawUserPrimitives<VertexPositionColor>(PrimitiveType.LineList, m_lineVertices, 0, m_lineIndex / 2);
                    //pass.End();
                }
                //m_debugEffect.End();
            }
            m_lineIndex = 0;
        }


        private void RemapModel(Model model, Effect effect)
        {
            foreach (ModelMesh mesh in model.Meshes)
            {
                foreach (ModelMeshPart part in mesh.MeshParts)
                {
                    part.Effect = effect;
                }
            }
        }

        #region IDebugDraw Members
        public void DrawLine(IndexedVector3 from, IndexedVector3 to, IndexedVector3 fromColor)
        {
            DrawLine(ref from, ref to, ref fromColor);
        }

        public void DrawLine(ref IndexedVector3 from, ref IndexedVector3 to, ref IndexedVector3 fromColor)
        {
            DrawLine(ref from, ref to, ref fromColor, ref fromColor);
        }

        public void DrawLine(ref IndexedVector3 from, ref IndexedVector3 to, ref IndexedVector3 fromColor, ref IndexedVector3 toColor)
        {
            if (m_lineIndex < m_lineVertexMaxSize - 2)
            {
                m_lineVertices[m_lineIndex].Position = from.ToVector3();
                m_lineVertices[m_lineIndex++].Color = new Color(fromColor.ToVector3());

                m_lineVertices[m_lineIndex].Position = to.ToVector3();
                m_lineVertices[m_lineIndex++].Color = new Color(toColor.ToVector3());
            }
        }
        public void DrawBox(ref IndexedVector3 boxMin, ref IndexedVector3 boxMax, ref IndexedMatrix trans, ref IndexedVector3 color)
        {
            DrawBox(ref boxMin, ref boxMax, ref trans, ref color, 1f);
        }

        public void DrawBox(ref IndexedVector3 boxMin, ref IndexedVector3 boxMax, ref IndexedMatrix transform, ref IndexedVector3 color, float alpha)
        {
            IndexedVector3 sideLengths = boxMax - boxMin;
            IndexedVector3 position = boxMin + (sideLengths * 0.5f);

            IndexedVector3 min = transform * boxMin;
            IndexedVector3 max = transform * boxMax;

			DrawLine(new IndexedVector3(min.X, min.Y, min.Z), new IndexedVector3(max.X, min.Y, min.Z), color);
			DrawLine(new IndexedVector3(max.X, min.Y, min.Z), new IndexedVector3(max.X, max.Y, min.Z), color);
			DrawLine(new IndexedVector3(max.X, max.Y, min.Z), new IndexedVector3(min.X, max.Y, min.Z), color);
			DrawLine(new IndexedVector3(min.X, max.Y, min.Z), new IndexedVector3(min.X, min.Y, min.Z), color);
			DrawLine(new IndexedVector3(min.X, min.Y, min.Z), new IndexedVector3(min.X, min.Y, max.Z), color);
			DrawLine(new IndexedVector3(max.X, min.Y, min.Z), new IndexedVector3(max.X, min.Y, max.Z), color);
			DrawLine(new IndexedVector3(max.X, max.Y, min.Z), new IndexedVector3(max.X, max.Y, max.Z), color);
			DrawLine(new IndexedVector3(min.X, max.Y, min.Z), new IndexedVector3(min.X, max.Y, max.Z), color);
			DrawLine(new IndexedVector3(min.X, min.Y, max.Z), new IndexedVector3(max.X, min.Y, max.Z), color);
			DrawLine(new IndexedVector3(max.X, min.Y, max.Z), new IndexedVector3(max.X, max.Y, max.Z), color);
			DrawLine(new IndexedVector3(max.X, max.Y, max.Z), new IndexedVector3(min.X, max.Y, max.Z), color);
			DrawLine(new IndexedVector3(min.X, max.Y, max.Z), new IndexedVector3(min.X, min.Y, max.Z), color);

			//drawLine(IndexedVector3.Transform(new IndexedVector3(boxMin.X, boxMin.Y, boxMin.Z),transform), IndexedVector3.Transform(new IndexedVector3(boxMax.X, boxMin.Y, boxMin.Z),transform), color);
			//drawLine(IndexedVector3.Transform(new IndexedVector3(boxMax.X, boxMin.Y, boxMin.Z),transform), IndexedVector3.Transform(new IndexedVector3(boxMax.X, boxMax.Y, boxMin.Z),transform), color);
			//drawLine(IndexedVector3.Transform(new IndexedVector3(boxMax.X, boxMax.Y, boxMin.Z),transform), IndexedVector3.Transform(new IndexedVector3(boxMin.X, boxMax.Y, boxMin.Z),transform), color);
			//drawLine(IndexedVector3.Transform(new IndexedVector3(boxMin.X, boxMax.Y, boxMin.Z),transform), IndexedVector3.Transform(new IndexedVector3(boxMin.X, boxMin.Y, boxMin.Z),transform), color);
			//drawLine(IndexedVector3.Transform(new IndexedVector3(boxMin.X, boxMin.Y, boxMin.Z),transform), IndexedVector3.Transform(new IndexedVector3(boxMin.X, boxMin.Y, boxMax.Z),transform), color);
			//drawLine(IndexedVector3.Transform(new IndexedVector3(boxMax.X, boxMin.Y, boxMin.Z),transform), IndexedVector3.Transform(new IndexedVector3(boxMax.X, boxMin.Y, boxMax.Z),transform), color);
			//drawLine(IndexedVector3.Transform(new IndexedVector3(boxMax.X, boxMax.Y, boxMin.Z),transform), IndexedVector3.Transform(new IndexedVector3(boxMax.X, boxMax.Y, boxMax.Z),transform), color);
			//drawLine(IndexedVector3.Transform(new IndexedVector3(boxMin.X, boxMax.Y, boxMin.Z),transform), IndexedVector3.Transform(new IndexedVector3(boxMin.X, boxMax.Y, boxMax.Z),transform), color);
			//drawLine(IndexedVector3.Transform(new IndexedVector3(boxMin.X, boxMin.Y, boxMax.Z),transform), IndexedVector3.Transform(new IndexedVector3(boxMax.X, boxMin.Y, boxMax.Z),transform), color);
			//drawLi    ne(IndexedVector3.Transform(new IndexedVector3(boxMax.X, boxMin.Y, boxMax.Z),transform), IndexedVector3.Transform(new IndexedVector3(boxMax.X, boxMax.Y, boxMax.Z),transform), color);
			//drawLine(IndexedVector3.Transform(new IndexedVector3(boxMax.X, boxMax.Y, boxMax.Z),transform), IndexedVector3.Transform(new IndexedVector3(boxMin.X, boxMax.Y, boxMax.Z),transform), color);
			//drawLine(IndexedVector3.Transform(new IndexedVector3(boxMin.X, boxMax.Y, boxMax.Z),transform), IndexedVector3.Transform(new IndexedVector3(boxMin.X, boxMin.Y, boxMax.Z), transform), color);

            //m_shapeList.Add(DrawHelper.createBox(position, sideLengths, new Color(color), ref transform));
        }


        public void DrawSphere(IndexedVector3 p, float radius, IndexedVector3 color)
        {
            DrawSphere(ref p, radius, ref color);
        }

        public void DrawSphere(ref IndexedVector3 p, float radius, ref IndexedVector3 color)
        {
            //throw new NotImplementedException();
        }

        public void DrawTriangle(ref IndexedVector3 v0, ref IndexedVector3 v1, ref IndexedVector3 v2, ref IndexedVector3 n0, ref IndexedVector3 n1, ref IndexedVector3 n2, ref IndexedVector3 color, float alpha)
        {
            DrawTriangle(ref v0, ref v1, ref v2, ref color, alpha);
        }

        public void DrawTriangle(ref IndexedVector3 v0, ref IndexedVector3 v1, ref IndexedVector3 v2, ref IndexedVector3 color, float alpha)
        {
            DrawLine(ref v0, ref v1, ref color);
            DrawLine(ref v1, ref v2, ref color);
            DrawLine(ref v2, ref v0, ref color);
        }

        public void DrawContactPoint(IndexedVector3 PointOnB, IndexedVector3 normalOnB, float distance, int lifeTime, IndexedVector3 color)
        {
            DrawContactPoint(ref PointOnB, ref normalOnB, distance, lifeTime, ref color);
        }

        public void DrawContactPoint(ref IndexedVector3 PointOnB, ref IndexedVector3 normalOnB, float distance, int lifeTime, ref IndexedVector3 color)
        {
            IndexedVector3 from = PointOnB;
            IndexedVector3 to = PointOnB + (normalOnB * 1f);
            DrawLine(ref from, ref to, ref color);
        }

        public void ReportErrorWarning(string warningString)
        {
            //throw new NotImplementedException();
        }

        public void Draw3dText(ref IndexedVector3 location, string textString)
        {
            //throw new NotImplementedException();
        }

        public void SetDebugMode(DebugDrawModes debugMode)
        {
            m_game.SetDebugMode(debugMode);
        }

        public DebugDrawModes GetDebugMode()
        {
            return m_game.GetDebugMode();
        }

        public void DrawAabb(IndexedVector3 from, IndexedVector3 to, IndexedVector3 color)
        {
            DrawAabb(ref from, ref to, ref color);
        }

        public void DrawAabb(ref IndexedVector3 from, ref IndexedVector3 to, ref IndexedVector3 color)
        {
            IndexedMatrix identity = IndexedMatrix.Identity;
            DrawBox(ref from, ref to, ref identity, ref color, 0f);
        }

        public void DrawTransform(ref IndexedMatrix transform, float orthoLen)
        {
            IndexedVector3 start = transform._origin;
            IndexedVector3 temp = start + transform._basis * new IndexedVector3(orthoLen, 0, 0);
            IndexedVector3 colour = new IndexedVector3(0.7f, 0, 0);
            DrawLine(ref start, ref temp, ref colour);
            temp = start + transform._basis * new IndexedVector3(0, orthoLen, 0);
            colour = new IndexedVector3(0, 0.7f, 0);
            DrawLine(ref start, ref temp, ref colour);
            temp = start + transform._basis * new IndexedVector3(0, 0, orthoLen);
            colour = new IndexedVector3(0, 0, 0.7f);
            DrawLine(ref start, ref temp, ref colour);
        }

        public void DrawArc(ref IndexedVector3 center, ref IndexedVector3 normal, ref IndexedVector3 axis, float radiusA, float radiusB, float minAngle, float maxAngle, ref IndexedVector3 color, bool drawSect)
        {
            DrawArc(ref center, ref normal, ref axis, radiusA, radiusB, minAngle, maxAngle, ref color, drawSect, 10f);
        }

        public void DrawArc(ref IndexedVector3 center, ref IndexedVector3 normal, ref IndexedVector3 axis, float radiusA, float radiusB, float minAngle, float maxAngle, ref IndexedVector3 color, bool drawSect, float stepDegrees)
        {
            IndexedVector3 vx = axis;
            IndexedVector3 vy = IndexedVector3.Cross(normal, axis);
            float step = stepDegrees * MathUtil.SIMD_RADS_PER_DEG;
            int nSteps = (int)((maxAngle - minAngle) / step);
            if (nSteps == 0)
            {
                nSteps = 1;
            }
            IndexedVector3 prev = center + radiusA * vx * (float)Math.Cos(minAngle) + radiusB * vy * (float)Math.Sin(minAngle);
            if (drawSect)
            {
                DrawLine(ref center, ref prev, ref color);
            }
            for (int i = 1; i <= nSteps; i++)
            {
                float angle = minAngle + (maxAngle - minAngle) * i / nSteps;
                IndexedVector3 next = center + radiusA * vx * (float)Math.Cos(angle) + radiusB * vy * (float)Math.Sin(angle);
                DrawLine(ref prev, ref next, ref color);
                prev = next;
            }
            if (drawSect)
            {
                DrawLine(ref center, ref prev, ref color);
            }
        }

        public void DrawSpherePatch(ref IndexedVector3 center, ref IndexedVector3 up, ref IndexedVector3 axis, float radius, float minTh, float maxTh, float minPs, float maxPs, ref IndexedVector3 color)
        {
            DrawSpherePatch(ref center, ref up, ref axis, radius, minTh, maxTh, minPs, maxPs, ref color, 10);
        }

        public void DrawSpherePatch(ref IndexedVector3 center, ref IndexedVector3 up, ref IndexedVector3 axis, float radius, float minTh, float maxTh, float minPs, float maxPs, ref IndexedVector3 color, float stepDegrees)
        {
            IndexedVector3[] vA;
            IndexedVector3[] vB;
            IndexedVector3[] pvA, pvB, pT;
            IndexedVector3 npole = center + up * radius;
            IndexedVector3 spole = center - up * radius;
            IndexedVector3 arcStart = IndexedVector3.Zero;
            float step = stepDegrees * MathUtil.SIMD_RADS_PER_DEG;
            IndexedVector3 kv = up;
            IndexedVector3 iv = axis;

            IndexedVector3 jv = IndexedVector3.Cross(kv, iv);
            bool drawN = false;
            bool drawS = false;
            if (minTh <= -MathUtil.SIMD_HALF_PI)
            {
                minTh = -MathUtil.SIMD_HALF_PI + step;
                drawN = true;
            }
            if (maxTh >= MathUtil.SIMD_HALF_PI)
            {
                maxTh = MathUtil.SIMD_HALF_PI - step;
                drawS = true;
            }
            if (minTh > maxTh)
            {
                minTh = -MathUtil.SIMD_HALF_PI + step;
                maxTh = MathUtil.SIMD_HALF_PI - step;
                drawN = drawS = true;
            }
            int n_hor = (int)((maxTh - minTh) / step) + 1;
            if (n_hor < 2) n_hor = 2;
            float step_h = (maxTh - minTh) / (n_hor - 1);
            bool isClosed = false;
            if (minPs > maxPs)
            {
                minPs = -MathUtil.SIMD_PI + step;
                maxPs = MathUtil.SIMD_PI;
                isClosed = true;
            }
            else if ((maxPs - minPs) >= MathUtil.SIMD_PI * 2f)
            {
                isClosed = true;
            }
            else
            {
                isClosed = false;
            }
            int n_vert = (int)((maxPs - minPs) / step) + 1;
            if (n_vert < 2) n_vert = 2;

            vA = new IndexedVector3[n_vert];
            vB = new IndexedVector3[n_vert];
            pvA = vA; pvB = vB;

            float step_v = (maxPs - minPs) / (float)(n_vert - 1);
            for (int i = 0; i < n_hor; i++)
            {
                float th = minTh + i * step_h;
                float sth = radius * (float)Math.Sin(th);
                float cth = radius * (float)Math.Cos(th);
                for (int j = 0; j < n_vert; j++)
                {
                    float psi = minPs + (float)j * step_v;
                    float sps = (float)Math.Sin(psi);
                    float cps = (float)Math.Cos(psi);
                    pvB[j] = center + cth * cps * iv + cth * sps * jv + sth * kv;
                    if (i != 0)
                    {
                        DrawLine(pvA[j], pvB[j], color);
                    }
                    else if (drawS)
                    {
                        DrawLine(spole, pvB[j], color);
                    }
                    if (j != 0)
                    {
                        DrawLine(pvB[j - 1], pvB[j], color);
                    }
                    else
                    {
                        arcStart = pvB[j];
                    }
                    if ((i == (n_hor - 1)) && drawN)
                    {
                        DrawLine(npole, pvB[j], color);
                    }
                    if (isClosed)
                    {
                        if (j == (n_vert - 1))
                        {
                            DrawLine(arcStart, pvB[j], color);
                        }
                    }
                    else
                    {
                        if (((i == 0) || (i == (n_hor - 1))) && ((j == 0) || (j == (n_vert - 1))))
                        {
                            DrawLine(center, pvB[j], color);
                        }
                    }
                }
                pT = pvA; pvA = pvB; pvB = pT;
            }

        }

        #endregion


	    private IList<ShapeCache> m_shapecaches = new List<ShapeCache>();

        private const int m_textureVertexMaxSize = 10000;
        private const int m_lineVertexMaxSize = 500000;

        private int m_texturedVertexCount;
        private int m_lineIndex = 0;
        
        
        private VertexPositionNormalTexture[] m_texturedVertices = new VertexPositionNormalTexture[m_textureVertexMaxSize];
        private VertexPositionColor[] m_lineVertices = new VertexPositionColor[m_lineVertexMaxSize];

        private VertexDeclaration m_vertexDeclaration;
        private VertexDeclaration m_lineVertexDeclaration;

        private IList<TextPositionColor> m_textPositionColours = new List<TextPositionColor>();

		private ObjectArray<ModelScalingData> m_modelScalingData = new ObjectArray<ModelScalingData>();

        private RenderTarget2D m_shadowRenderTarget;

        private bool m_textureEnabled = true;
        private bool m_textureInitialized;
        private Texture2D m_generatedTexture;
        private Texture2D m_shadowMapTexture;

        private SpriteBatch m_spriteBatch;
        private SpriteFont m_spriteFont;

        private Model m_lightModel;
        private Texture2D m_lightTexture;

		private Texture2D GetTexture(ref IndexedVector3 color)
		{
			if(!m_colorMap.ContainsKey(color))
			{
				Texture2D newTexture = new Texture2D(m_game.GraphicsDevice,1,1);
				Color[] colorData = new Color[1];
				newTexture.GetData<Color>(colorData);
				colorData[0] = new Color(color.ToVector3());
				newTexture.SetData(colorData);
				m_colorMap[color] = newTexture;
			}
			return m_colorMap[color];
		}

		private Dictionary<IndexedVector3, Texture2D> m_colorMap = new Dictionary<IndexedVector3, Texture2D>();




		private Model m_cubeModel;
		private Model m_sphereModel;
		private Model m_cylinderModel;
		private Model m_coneModel;
		
		
		private Viewport m_pipViewport;
        private Viewport m_defaultViewport;

        private DemoApplication m_game;
		//private BasicEffect m_lightEffect;
		private BasicEffect m_debugEffect;
		private BasicEffect m_modelEffect;
        private BasicEffect m_vertexEffect;
    }

	class ModelScalingData
	{
		public ModelScalingData() { }
		public ModelScalingData(Model _model, IndexedVector3 _scale, IndexedMatrix _transform)
		{
			IndexedMatrix scale = IndexedMatrix.CreateScale(_scale);
			IndexedMatrix copy = _transform;
			copy._origin = IndexedVector3.Zero;
            copy = copy * scale;
            copy._origin = _transform._origin;

            model = _model;
			transform = copy;

			color = IndexedVector3.One;
		}

		public Model model;
		public IndexedMatrix transform;
		public IndexedVector3 color;
	}






    class TextPositionColor
    {
        public String m_text;
        public Vector2 m_position;
        public Color m_color;
    }


    public class XNADrawcallback : ITriangleCallback
    {

        public virtual bool graphics()
        {
            return true;
        }

	    public bool	m_wireframe;
        private XNA_ShapeDrawer m_shapeDrawer;
        private static IndexedVector3 RED = new IndexedVector3(1, 0, 0);
        private static IndexedVector3 BLUE = new IndexedVector3(0, 1, 0);
        private static IndexedVector3 GREEN = new IndexedVector3(0, 0, 1);
		private IndexedMatrix matrix; // not included up till now
        
	    public XNADrawcallback(XNA_ShapeDrawer shapeDrawer,ref IndexedMatrix m)
	    {
            m_wireframe = false;
            m_shapeDrawer = shapeDrawer;
			matrix = m;
	    }

        public virtual void ProcessTriangle(IndexedVector3[] triangle, int partId, int triangleIndex)
	    {
		    if (m_wireframe)
		    {
				// put them in object space.
				IndexedVector3.Transform(triangle, ref matrix, triangle);


                m_shapeDrawer.DrawLine(ref triangle[0], ref triangle[1], ref RED);
                m_shapeDrawer.DrawLine(ref triangle[1], ref triangle[2], ref GREEN);
                m_shapeDrawer.DrawLine(ref triangle[2], ref triangle[0], ref BLUE);

                //draw normal?
                IndexedVector3 d = triangle[1] - triangle[0];
                IndexedVector3 e = triangle[2] - triangle[0];
                // Reverse the cross here to account for winding, shouldn't change the way rest of bullet works.
				IndexedVector3 cross = IndexedVector3.Cross(d, e);
				//IndexedVector3 cross = IndexedVector3.Cross(e,d);

                IndexedVector3 colour = new IndexedVector3(1, 0, 1);
                int ibreak = 0;
                IndexedVector3 center = (triangle[0]+triangle[1]+triangle[2])*(1.0f/3.0f);

                cross += center;
                //m_shapeDrawer.DrawLine(ref center, ref cross, ref colour);                

		    } 
            else
		    {
                IndexedVector3.Transform(triangle, ref matrix, triangle);
                m_shapeDrawer.DrawSolidTriangle(triangle);
		    }
	    }

        public virtual void Cleanup()
        {
        }


    }

    public class TriangleGlDrawcallback : IInternalTriangleIndexCallback
    {
        public virtual bool graphics()
        {
            return true;
        }

        public TriangleGlDrawcallback(XNA_ShapeDrawer shapeDrawer)
        {
            m_shapeDrawer = shapeDrawer;
        }

        public virtual void InternalProcessTriangleIndex(IndexedVector3[] triangle, int partId, int triangleIndex)
	    {
            m_shapeDrawer.DrawSolidTriangle(triangle);
	    }

        public virtual void Cleanup()
        {
        }

        private XNA_ShapeDrawer m_shapeDrawer;



    }
}
