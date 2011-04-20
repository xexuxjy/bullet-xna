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
using BulletXNA.BullettCollision.CollisionShapes;
using Microsoft.Xna.Framework;
using BulletXNA.BullettCollision.BroadphaseCollision;
using BulletXNA.BulletCollision.CollisionShapes;
using BulletXNA.LinearMath;
using Microsoft.Xna.Framework.Graphics;
using BulletXNA;
using System.Diagnostics;

namespace BulletXNADemos.Demos
{
	public class Edge 
    { 
        public Vector3[] n = new Vector3[2];
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
			m_coneModel = m_game.Content.Load<Model>("unitcone");

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
			//rasterizerState.CullMode = CullMode.CullCounterClockwiseFace;
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

        //public void startDraw(GraphicsDevice graphicsDevice,ref Matrix view, ref Matrix projection)
        //{
        //    ((DefaultDebugDraw)m_debugDraw).update(graphicsDevice, ref view, ref projection);
        //}

		public virtual void	DrawShadow(ref Matrix m, ref Vector3 extrusion,CollisionShape shape,ref Vector3 worldBoundsMin,ref Vector3 worldBoundsMax)
        {
	        if(shape.GetShapeType() == BroadphaseNativeTypes.UNIFORM_SCALING_SHAPE_PROXYTYPE)
	        {
		        UniformScalingShape scalingShape = (UniformScalingShape)(shape);
		        ConvexShape convexShape = scalingShape.GetChildShape();
		        float	scalingFactor = (float)scalingShape.GetUniformScalingFactor();
		        Matrix tmpScaling = Matrix.CreateScale(scalingFactor);
                tmpScaling *= m;
		        DrawShadow(ref tmpScaling,ref extrusion,convexShape,ref worldBoundsMin,ref worldBoundsMax);

		        return;
	        }
	        else if(shape.GetShapeType()==BroadphaseNativeTypes.COMPOUND_SHAPE_PROXYTYPE)
	        {
		        CompoundShape compoundShape = (CompoundShape)(shape);
		        for (int i=compoundShape.GetNumChildShapes()-1;i>=0;i--)
		        {
			        Matrix childTrans = compoundShape.GetChildTransform(i);
			        CollisionShape colShape = compoundShape.GetChildShape(i);
                    //float childMat[16];
                    //childTrans.getOpenGLMatrix(childMat);
                    Vector3 transformedExtrude = Vector3.TransformNormal(extrusion,childTrans);
			        DrawShadow(ref childTrans,ref transformedExtrude,colShape,ref worldBoundsMin,ref worldBoundsMax);
		        }
	        }
	        else
	        {
		        bool useWireframeFallback = true;
                if (shape.IsConvex())
                {
                    ShapeCache	sc=Cache((ConvexShape)shape);
                    ShapeHull hull  = sc.m_shapehull;
                    //glBegin(GL_QUADS);
                    for(int i=0;i<sc.m_edges.Count;++i)
                    {			
                        float d=Vector3.Dot(sc.m_edges[i].n[0],extrusion);
                        if((d*Vector3.Dot(sc.m_edges[i].n[1],extrusion))<0)
                        {
                            int	q=	d<0?1:0;
                            Vector3	a=	hull.m_vertices[sc.m_edges[i].v[q]];
                            Vector3	b=	hull.m_vertices[sc.m_edges[i].v[1-q]];
                            Vector3 ae = a + extrusion;
                            Vector3 be = b + extrusion;
                            Vector2 tex = new Vector2(0,0);
                            // fix me.
                            Vector3 normal = Vector3.Up;
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

        public void DrawSolidCube(ref Vector3 halfExtents, ref Matrix matrix, ref Matrix view, ref Matrix projection)
        {
			ModelScalingData modelScalingData = new ModelScalingData(m_cubeModel,halfExtents,matrix);
			m_modelScalingData.Add(modelScalingData);
        }

        private void AddVertex(Vector3 vec, Vector3 normal, Vector2 tex)
        {
            AddVertex(ref vec,ref normal, ref tex);
        }

        private void AddVertex(ref Vector3 vec, ref Vector3 normal, ref Vector2 tex)
        {
            if(m_texturedVertexCount < m_textureVertexMaxSize-1)
            {
                m_texturedVertices[m_texturedVertexCount++] = new VertexPositionNormalTexture(vec, normal,tex);
            }
        }

        public void DrawText(String text, Vector3 position, Vector3 color)
        {
            DrawText(text, ref position, ref color);
        }

        public void DrawText(String text, ref Vector3 position, ref Vector3 color)
        {
            TextPositionColor tpc = new TextPositionColor();
            tpc.m_text = text;
            tpc.m_position = new Vector2(position.X, position.Y);
            tpc.m_color = new Color(color);

            m_textPositionColours.Add(tpc);
        }

        public void DrawSolidTriangle(IList<Vector3> points)
        {
            Vector3 normal = Vector3.Cross(points[1] - points[0], points[2] - points[0]);
            normal.Normalize();
            AddVertex(points[0], normal,new Vector2(0,0));
            AddVertex(points[1], normal, new Vector2(0, 1));
            AddVertex(points[2], normal, new Vector2(1, 1));
        }



        //public void drawWireframeCube(float sideLength)
        //{
        //}


        public void DrawSolidSphere(float radius, int slices, int stacks, ref Matrix matrix, ref Matrix view, ref Matrix projection)
        {

			ModelScalingData modelScalingData = new ModelScalingData(m_sphereModel,new Vector3(radius,radius,radius),matrix);
			m_modelScalingData.Add(modelScalingData);
        }


        public void DrawSolidCone(float height, float radius, ref Matrix matrix, ref Matrix view, ref Matrix projection)
        {
			Vector3 scale = new Vector3(radius, height, radius);
			ModelScalingData modelScalingData = new ModelScalingData(m_coneModel, scale, matrix);
			m_modelScalingData.Add(modelScalingData);
        }



        public void DrawCylinder(float radius, float halfHeight, int upAxis, ref Matrix matrix, ref Matrix view, ref Matrix projection)
        {
            DrawCylinder(radius, radius, halfHeight, upAxis, ref matrix, ref view, ref projection);
        }


        public void DrawCylinder(float topRadius,float bottomRadius,float halfHeight, int upAxis,ref Matrix matrix, ref Matrix view, ref Matrix projection)
        {
			Vector3 scale = new Vector3(topRadius, halfHeight, bottomRadius);
			Matrix rotate = Matrix.Identity;

            if (upAxis == 0)
            {
                rotate = Matrix.CreateRotationZ(MathUtil.SIMD_HALF_PI);
            }
            if (upAxis == 1)
			{
				rotate = Matrix.CreateRotationY(MathUtil.SIMD_HALF_PI);
			}
			else if (upAxis == 2)
			{
				rotate = Matrix.CreateRotationX(MathUtil.SIMD_HALF_PI);
			}

			Matrix copy = matrix;
			copy.Translation = Vector3.Zero;
			copy = rotate * copy;
			copy.Translation = matrix.Translation;

			ModelScalingData modelScalingData = new ModelScalingData(m_cylinderModel, scale, copy);
			m_modelScalingData.Add(modelScalingData);
		}
        
        public void DrawCoordSystem()
        {

        }

        public void DrawXNA(ref Matrix m, CollisionShape shape, ref Vector3 color, DebugDrawModes debugMode, ref Vector3 worldBoundsMin, ref Vector3 worldBoundsMax, ref Matrix view, ref Matrix projection)
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
                Matrix scaleMatrix = Matrix.CreateScale(scalingFactor);
                Matrix finalMatrix = scaleMatrix * m;
                DrawXNA(ref finalMatrix, convexShape, ref color, debugMode, ref worldBoundsMin, ref worldBoundsMax,ref view,ref projection);
                return;
            }
            if (shape.GetShapeType() == BroadphaseNativeTypes.COMPOUND_SHAPE_PROXYTYPE)
            {
                CompoundShape compoundShape = (CompoundShape)shape;
                for (int i = compoundShape.GetNumChildShapes() - 1; i >= 0; i--)
                {
                    Matrix childTrans = compoundShape.GetChildTransform(i);
                    CollisionShape colShape = compoundShape.GetChildShape(i);
                    Matrix childMat = childTrans;

					//childMat = MathUtil.bulletMatrixMultiply(m, childMat);
					childMat = childMat * m;
					//childMat = m * childMat;

                    
					
					DrawXNA(ref childMat, colShape, ref color, debugMode, ref worldBoundsMin, ref worldBoundsMax,ref view,ref projection);
                }
            }
            else
            {

                bool useWireframeFallback = true;

                if ((debugMode & DebugDrawModes.DBG_DrawWireframe) != 0)
                {
                    ///you can comment out any of the specific cases, and use the default
                    ///the benefit of 'default' is that it approximates the actual collision shape including collision margin
                    //BroadphaseNativeTypes shapetype = m_textureEnabled ? BroadphaseNativeTypes.MAX_BROADPHASE_COLLISION_TYPES : shape.getShapeType();
                    BroadphaseNativeTypes shapetype = shape.GetShapeType();
                    switch (shapetype)
                    {
                        case BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE:
                            {
                                BoxShape boxShape = (BoxShape)(shape);
                                Vector3 halfExtents = boxShape.GetHalfExtentsWithMargin();

                                DrawSolidCube(ref halfExtents, ref m, ref view, ref projection);
                                //drawSolidSphere(halfExtents.X, 10, 10, ref m, ref view, ref projection);
                                //drawCylinder(halfExtents.X, halfExtents.Y, 1, ref m, ref view, ref projection);
                                //drawSolidCone(halfExtents.Y, halfExtents.X, ref m, ref view, ref projection);

                                DrawText("Hello World", new Vector3(20, 20, 0), new Vector3(255, 255, 255));
                                useWireframeFallback = false;
                                break;
                            }


                        case BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE:
                            {
                                SphereShape sphereShape = (SphereShape)(shape);
                                float radius = sphereShape.GetMargin();//radius doesn't include the margin, so draw with margin
                                DrawSolidSphere(radius,10,10,ref m,ref view, ref projection);
                                //glutSolidSphere(radius,10,10);
                                useWireframeFallback = false;
                                break;
                            }
                        case BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE:
		                    {
			                    CapsuleShape capsuleShape = (CapsuleShape)(shape);

			                    float radius = capsuleShape.getRadius();
			                    float halfHeight = capsuleShape.getHalfHeight();

			                    int upAxis = capsuleShape.GetUpAxis();

			                    Vector3 capStart = Vector3.Zero;
			                    MathUtil.VectorComponent(ref capStart,upAxis,-halfHeight);

                                Vector3 capEnd = Vector3.Zero;
                                MathUtil.VectorComponent(ref capEnd, upAxis, halfHeight);

			                    // Draw the ends
			                    {

				                    Matrix childTransform = Matrix.Identity;
				                    childTransform.Translation = Vector3.Transform(capStart,m);
				                    DrawSolidSphere(radius,5,5, ref childTransform, ref view,ref projection);
			                    }

			                    {
                                    Matrix childTransform = Matrix.Identity;
                                    childTransform.Translation = Vector3.Transform(capEnd, m);
                                    DrawSolidSphere(radius, 5, 5, ref childTransform, ref view, ref projection);
                                }

                                DrawCylinder(radius, halfHeight, upAxis, ref m, ref view, ref projection);
                                break;
		                    }
                        case BroadphaseNativeTypes.CONE_SHAPE_PROXYTYPE:
                            {
                                ConeShape coneShape = (ConeShape)(shape);
                                int upIndex = coneShape.GetConeUpIndex();
                                float radius = coneShape.GetRadius();//+coneShape.getMargin();
                                float height = coneShape.GetHeight();//+coneShape.getMargin();
                                Matrix rotateMatrix = Matrix.Identity;


                                switch (upIndex)
                                {
                                    case 0:
                                        rotateMatrix = Matrix.CreateRotationX(MathUtil.SIMD_HALF_PI);
                                        break;
                                    case 1:
                                        rotateMatrix = Matrix.CreateRotationX(-MathUtil.SIMD_HALF_PI);
                                        break;
                                    case 2:
                                        break;
                                    default:
                                        {
                                            break;
                                        }
                                };

                                Matrix translationMatrix = Matrix.CreateTranslation(0f, 0f, -0.5f * height);

                                Matrix resultant = translationMatrix * rotateMatrix * m;
                                DrawSolidCone(radius, height,ref resultant,ref view, ref projection);
                                useWireframeFallback = false;
                                break;

                            }


                        case BroadphaseNativeTypes.STATIC_PLANE_PROXYTYPE:
                            {
                                StaticPlaneShape staticPlaneShape = (StaticPlaneShape)(shape);
                                float planeConst = staticPlaneShape.GetPlaneConstant();
                                Vector3 planeNormal = staticPlaneShape.GetPlaneNormal();
                                Vector3 planeOrigin = planeNormal * planeConst;
                                Vector3 vec0 = Vector3.Zero, vec1 = Vector3.Zero;
                                TransformUtil.PlaneSpace1(ref planeNormal, ref vec0, ref vec1);
                                float vecLen = 100f;
                                Vector3 pt0 = planeOrigin + vec0 * vecLen;
                                Vector3 pt1 = planeOrigin - vec0 * vecLen;
                                Vector3 pt2 = planeOrigin + vec1 * vecLen;
                                Vector3 pt3 = planeOrigin - vec1 * vecLen;

                                // Fallback to debug draw - needs tidying
                                Vector3 colour = new Vector3(255, 255, 255);
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
                                float halfHeight = MathUtil.VectorComponent(cylinder.GetHalfExtentsWithMargin(), upAxis);
                                DrawCylinder(radius, halfHeight, upAxis,ref m,ref view,ref projection);
                                break;
                            }

                        default:
                            {
                                if (shape.IsConvex())
                                {
                                    ShapeCache	sc=Cache((ConvexShape)shape);

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
                                            IList<Vector3> vtx = hull.m_vertices;

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

                                                Vector3 v1 = Vector3.Transform(vtx[index1],m);
                                                Vector3 v2 = Vector3.Transform(vtx[index2],m);
                                                Vector3 v3 = Vector3.Transform(vtx[index3],m);
                                                Vector3 normal = Vector3.Cross((v3-v1),(v2-v1));
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

                            Vector3 colour = new Vector3(255, 255, 255);
                            for (int i = 0; i < polyshape.GetNumVertices(); i++)
                            {
                                Vector3 vtx = Vector3.Zero;
                                polyshape.GetVertex(i, ref vtx);
                                String buf = " " + i;
                                DrawText(buf, ref vtx, ref colour);
                            }

                            for (int i = 0; i < polyshape.GetNumPlanes(); i++)
                            {
                                Vector3 normal = Vector3.Zero;
                                Vector3 vtx = Vector3.Zero;
                                polyshape.GetPlane(ref normal, ref vtx, i);
                                float d = Vector3.Dot(vtx, normal);
                                vtx *= d;

                                String buf = " plane " + i;
                                DrawText(buf, ref vtx, ref colour);
                            }
                        }
                    }

                    if (shape.IsConcave())//>getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE||shape.getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
                    //		if (shape.getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
                    {
                        ConcaveShape concaveMesh = (ConcaveShape)shape;

                        XNADrawcallback drawCallback = new XNADrawcallback(this,ref m);
                        drawCallback.m_wireframe = (debugMode & DebugDrawModes.DBG_DrawWireframe) != 0;

                        concaveMesh.ProcessAllTriangles(drawCallback, ref worldBoundsMin, ref worldBoundsMax);

                    }

                    //glDisable(GL_DEPTH_TEST);
                    //glRasterPos3f(0,0,0);//mvtx.x(),  vtx.y(),  vtx.z());
                    if ((debugMode & DebugDrawModes.DBG_DrawText) != 0)
                    {
                        Vector3 position = Vector3.Zero;
                        Vector3 colour = new Vector3(255, 255, 255);
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
		        IList<Vector3> pv=sc.m_shapehull.m_vertices;
		        IList<Edge> edges = new ObjectArray<Edge>(ni);
		        for(int i=0;i<ni;i+=3)
		        {
			        Vector3	nrm= Vector3.Normalize(Vector3.Cross(pv[pi[i+1]]-pv[pi[i]],pv[pi[i+2]]-pv[pi[i]]));
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

        public void RenderStandard(GameTime gameTime, ref Matrix view, ref Matrix projection)
        {
            // Always clear?
            m_game.GraphicsDevice.Clear(Color.CornflowerBlue);
            {
				DrawPrimitives(gameTime, ref view, ref projection);
                m_texturedVertexCount = 0;
            }
        }

        public void RenderOthers(GameTime gameTime, ref Matrix view, ref Matrix projection)
        {
            m_spriteBatch.Begin();
            for (int i = 0; i < m_textPositionColours.Count; ++i)
            {
                m_spriteBatch.DrawString(m_spriteFont, m_textPositionColours[i].m_text, m_textPositionColours[i].m_position, m_textPositionColours[i].m_color);
            }
            m_spriteBatch.End();
            m_textPositionColours.Clear();
			// restore from sprite batch changes.
			m_game.GraphicsDevice.BlendState = BlendState.Opaque;
			m_game.GraphicsDevice.DepthStencilState = DepthStencilState.Default;

        }


        private void DrawPrimitives(GameTime gameTime, ref Matrix view, ref Matrix projection)
        {
			if(m_texturedVertexCount > 0)
			{
                m_vertexEffect.View = view;
                m_vertexEffect.Projection = projection;
                foreach (EffectPass pass in m_vertexEffect.CurrentTechnique.Passes)
				{
					pass.Apply();
					m_game.GraphicsDevice.DrawUserPrimitives<VertexPositionNormalTexture>(PrimitiveType.TriangleList, m_texturedVertices, 0, m_texturedVertexCount / 3);
				}
			}


            foreach (ModelScalingData modelScalingData in m_modelScalingData)
            {
                //Matrix scale = Matrix.CreateScale(modelScalingData.scale);
                Matrix[] transforms = new Matrix[modelScalingData.model.Bones.Count];
                foreach (ModelMesh mesh in modelScalingData.model.Meshes)
                {
                    modelScalingData.model.CopyAbsoluteBoneTransformsTo(transforms);
                    foreach (BasicEffect effect in mesh.Effects)
                    {
                        effect.EnableDefaultLighting();
                        effect.View = view;
                        effect.Projection = projection;
                        effect.World = transforms[mesh.ParentBone.Index] * modelScalingData.transform;
                    }
                    mesh.Draw();
                }
            }
			m_modelScalingData.Clear();
        }

        public void RenderDebugLines(GameTime gameTime, ref Matrix view, ref Matrix projection)
        {
            m_debugEffect.World = Matrix.Identity;
            m_debugEffect.View = view;
            m_debugEffect.Projection = projection;

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
        public void DrawLine(Vector3 from, Vector3 to, Vector3 fromColor)
        {
            DrawLine(ref from, ref to, ref fromColor);
        }

        public void DrawLine(ref Vector3 from, ref Vector3 to, ref Vector3 fromColor)
        {
            DrawLine(ref from, ref to, ref fromColor, ref fromColor);
        }

        public void DrawLine(ref Vector3 from, ref Vector3 to, ref Vector3 fromColor, ref Vector3 toColor)
        {
            if (m_lineIndex < m_lineVertexMaxSize - 2)
            {
                m_lineVertices[m_lineIndex].Position = from;
                m_lineVertices[m_lineIndex++].Color = new Color(fromColor);

                m_lineVertices[m_lineIndex].Position = to;
                m_lineVertices[m_lineIndex++].Color = new Color(toColor);
            }
        }
        public void DrawBox(ref Vector3 boxMin, ref Vector3 boxMax, ref Matrix trans, ref Vector3 color)
        {
            DrawBox(ref boxMin, ref boxMax, ref trans, ref color, 1f);
        }

        public void DrawBox(ref Vector3 boxMin, ref Vector3 boxMax, ref Matrix transform, ref Vector3 color, float alpha)
        {
            Vector3 sideLengths = boxMax - boxMin;
            Vector3 position = boxMin + (sideLengths * 0.5f);

			Vector3 min = Vector3.Transform(boxMin, transform);
			Vector3 max = Vector3.Transform(boxMax, transform);

			DrawLine(new Vector3(min.X, min.Y, min.Z), new Vector3(max.X, min.Y, min.Z), color);
			DrawLine(new Vector3(max.X, min.Y, min.Z), new Vector3(max.X, max.Y, min.Z), color);
			DrawLine(new Vector3(max.X, max.Y, min.Z), new Vector3(min.X, max.Y, min.Z), color);
			DrawLine(new Vector3(min.X, max.Y, min.Z), new Vector3(min.X, min.Y, min.Z), color);
			DrawLine(new Vector3(min.X, min.Y, min.Z), new Vector3(min.X, min.Y, max.Z), color);
			DrawLine(new Vector3(max.X, min.Y, min.Z), new Vector3(max.X, min.Y, max.Z), color);
			DrawLine(new Vector3(max.X, max.Y, min.Z), new Vector3(max.X, max.Y, max.Z), color);
			DrawLine(new Vector3(min.X, max.Y, min.Z), new Vector3(min.X, max.Y, max.Z), color);
			DrawLine(new Vector3(min.X, min.Y, max.Z), new Vector3(max.X, min.Y, max.Z), color);
			DrawLine(new Vector3(max.X, min.Y, max.Z), new Vector3(max.X, max.Y, max.Z), color);
			DrawLine(new Vector3(max.X, max.Y, max.Z), new Vector3(min.X, max.Y, max.Z), color);
			DrawLine(new Vector3(min.X, max.Y, max.Z), new Vector3(min.X, min.Y, max.Z), color);

			//drawLine(Vector3.Transform(new Vector3(boxMin.X, boxMin.Y, boxMin.Z),transform), Vector3.Transform(new Vector3(boxMax.X, boxMin.Y, boxMin.Z),transform), color);
			//drawLine(Vector3.Transform(new Vector3(boxMax.X, boxMin.Y, boxMin.Z),transform), Vector3.Transform(new Vector3(boxMax.X, boxMax.Y, boxMin.Z),transform), color);
			//drawLine(Vector3.Transform(new Vector3(boxMax.X, boxMax.Y, boxMin.Z),transform), Vector3.Transform(new Vector3(boxMin.X, boxMax.Y, boxMin.Z),transform), color);
			//drawLine(Vector3.Transform(new Vector3(boxMin.X, boxMax.Y, boxMin.Z),transform), Vector3.Transform(new Vector3(boxMin.X, boxMin.Y, boxMin.Z),transform), color);
			//drawLine(Vector3.Transform(new Vector3(boxMin.X, boxMin.Y, boxMin.Z),transform), Vector3.Transform(new Vector3(boxMin.X, boxMin.Y, boxMax.Z),transform), color);
			//drawLine(Vector3.Transform(new Vector3(boxMax.X, boxMin.Y, boxMin.Z),transform), Vector3.Transform(new Vector3(boxMax.X, boxMin.Y, boxMax.Z),transform), color);
			//drawLine(Vector3.Transform(new Vector3(boxMax.X, boxMax.Y, boxMin.Z),transform), Vector3.Transform(new Vector3(boxMax.X, boxMax.Y, boxMax.Z),transform), color);
			//drawLine(Vector3.Transform(new Vector3(boxMin.X, boxMax.Y, boxMin.Z),transform), Vector3.Transform(new Vector3(boxMin.X, boxMax.Y, boxMax.Z),transform), color);
			//drawLine(Vector3.Transform(new Vector3(boxMin.X, boxMin.Y, boxMax.Z),transform), Vector3.Transform(new Vector3(boxMax.X, boxMin.Y, boxMax.Z),transform), color);
			//drawLine(Vector3.Transform(new Vector3(boxMax.X, boxMin.Y, boxMax.Z),transform), Vector3.Transform(new Vector3(boxMax.X, boxMax.Y, boxMax.Z),transform), color);
			//drawLine(Vector3.Transform(new Vector3(boxMax.X, boxMax.Y, boxMax.Z),transform), Vector3.Transform(new Vector3(boxMin.X, boxMax.Y, boxMax.Z),transform), color);
			//drawLine(Vector3.Transform(new Vector3(boxMin.X, boxMax.Y, boxMax.Z),transform), Vector3.Transform(new Vector3(boxMin.X, boxMin.Y, boxMax.Z), transform), color);

            //m_shapeList.Add(DrawHelper.createBox(position, sideLengths, new Color(color), ref transform));
        }


        public void DrawSphere(Vector3 p, float radius, Vector3 color)
        {
            DrawSphere(ref p, radius, ref color);
        }

        public void DrawSphere(ref Vector3 p, float radius, ref Vector3 color)
        {
            //throw new NotImplementedException();
        }

        public void DrawTriangle(ref Vector3 v0, ref Vector3 v1, ref Vector3 v2, ref Vector3 n0, ref Vector3 n1, ref Vector3 n2, ref Vector3 color, float alpha)
        {
            DrawTriangle(ref v0, ref v1, ref v2, ref color, alpha);
        }

        public void DrawTriangle(ref Vector3 v0, ref Vector3 v1, ref Vector3 v2, ref Vector3 color, float alpha)
        {
            DrawLine(ref v0, ref v1, ref color);
            DrawLine(ref v1, ref v2, ref color);
            DrawLine(ref v2, ref v0, ref color);
        }

        public void DrawContactPoint(Vector3 PointOnB, Vector3 normalOnB, float distance, int lifeTime, Vector3 color)
        {
            DrawContactPoint(ref PointOnB, ref normalOnB, distance, lifeTime, ref color);
        }

        public void DrawContactPoint(ref Vector3 PointOnB, ref Vector3 normalOnB, float distance, int lifeTime, ref Vector3 color)
        {
            //throw new NotImplementedException();
        }

        public void ReportErrorWarning(string warningString)
        {
            //throw new NotImplementedException();
        }

        public void Draw3dText(ref Vector3 location, string textString)
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

        public void DrawAabb(Vector3 from, Vector3 to, Vector3 color)
        {
            DrawAabb(ref from, ref to, ref color);
        }

        public void DrawAabb(ref Vector3 from, ref Vector3 to, ref Vector3 color)
        {
            Matrix identity = Matrix.Identity;
            DrawBox(ref from, ref to, ref identity, ref color, 0f);
        }

        public void DrawTransform(ref Matrix transform, float orthoLen)
        {
            Vector3 start = transform.Translation;
            Vector3 temp = start + Vector3.TransformNormal(new Vector3(orthoLen, 0, 0), transform);
            Vector3 colour = new Vector3(0.7f, 0, 0);
            DrawLine(ref start, ref temp, ref colour);
            temp = start + Vector3.TransformNormal(new Vector3(0, orthoLen, 0), transform);
            colour = new Vector3(0, 0.7f, 0);
            DrawLine(ref start, ref temp, ref colour);
            temp = start + Vector3.TransformNormal(new Vector3(0, 0, orthoLen), transform);
            colour = new Vector3(0, 0, 0.7f);
            DrawLine(ref start, ref temp, ref colour);
        }

        public void DrawArc(ref Vector3 center, ref Vector3 normal, ref Vector3 axis, float radiusA, float radiusB, float minAngle, float maxAngle, ref Vector3 color, bool drawSect)
        {
            DrawArc(ref center, ref normal, ref axis, radiusA, radiusB, minAngle, maxAngle, ref color, drawSect, 10f);
        }

        public void DrawArc(ref Vector3 center, ref Vector3 normal, ref Vector3 axis, float radiusA, float radiusB, float minAngle, float maxAngle, ref Vector3 color, bool drawSect, float stepDegrees)
        {
            Vector3 vx = axis;
            Vector3 vy = Vector3.Cross(normal, axis);
            float step = stepDegrees * MathUtil.SIMD_RADS_PER_DEG;
            int nSteps = (int)((maxAngle - minAngle) / step);
            if (nSteps == 0)
            {
                nSteps = 1;
            }
            Vector3 prev = center + radiusA * vx * (float)Math.Cos(minAngle) + radiusB * vy * (float)Math.Sin(minAngle);
            if (drawSect)
            {
                DrawLine(ref center, ref prev, ref color);
            }
            for (int i = 1; i <= nSteps; i++)
            {
                float angle = minAngle + (maxAngle - minAngle) * i / nSteps;
                Vector3 next = center + radiusA * vx * (float)Math.Cos(angle) + radiusB * vy * (float)Math.Sin(angle);
                DrawLine(ref prev, ref next, ref color);
                prev = next;
            }
            if (drawSect)
            {
                DrawLine(ref center, ref prev, ref color);
            }
        }

        public void DrawSpherePatch(ref Vector3 center, ref Vector3 up, ref Vector3 axis, float radius, float minTh, float maxTh, float minPs, float maxPs, ref Vector3 color)
        {
            DrawSpherePatch(ref center, ref up, ref axis, radius, minTh, maxTh, minPs, maxPs, ref color, 10);
        }

        public void DrawSpherePatch(ref Vector3 center, ref Vector3 up, ref Vector3 axis, float radius, float minTh, float maxTh, float minPs, float maxPs, ref Vector3 color, float stepDegrees)
        {
            Vector3[] vA = new Vector3[74];
            Vector3[] vB = new Vector3[74];
            Vector3[] pvA = vA, pvB = vB, pT;
            Vector3 npole = center + up * radius;
            Vector3 spole = center - up * radius;
            Vector3 arcStart = Vector3.Zero;
            float step = stepDegrees * MathUtil.SIMD_RADS_PER_DEG;
            Vector3 kv = up;
            Vector3 iv = axis;

            Vector3 jv = Vector3.Cross(kv, iv);
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
        private const int m_lineVertexMaxSize = 50000;

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
		public ModelScalingData(Model _model, Vector3 _scale, Matrix _transform)
		{
			Matrix scale = Matrix.CreateScale(_scale);
			Matrix copy = _transform;
			copy.Translation = Vector3.Zero;
			copy = scale * copy;
			copy.Translation = _transform.Translation;
			model = _model;
			transform = copy;
		}

		public Model model;
		public Matrix transform;
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
        private static Vector3 RED = new Vector3(1, 0, 0);
        private static Vector3 BLUE = new Vector3(0, 1, 0);
        private static Vector3 GREEN = new Vector3(0, 0, 1);
		private Matrix matrix; // not included up till now
        
	    public XNADrawcallback(XNA_ShapeDrawer shapeDrawer,ref Matrix m)
	    {
            m_wireframe = false;
            m_shapeDrawer = shapeDrawer;
			matrix = m;
	    }

        public virtual void ProcessTriangle(ObjectArray<Vector3> triangle, int partId, int triangleIndex)
	    {
		    if (m_wireframe)
		    {
                Vector3[] raw = triangle.GetRawArray();

				// put them in object space.
				Vector3.Transform(raw, ref matrix, raw);


                m_shapeDrawer.DrawLine(ref raw[0], ref raw[1], ref RED);
                m_shapeDrawer.DrawLine(ref raw[1], ref raw[2], ref GREEN);
                m_shapeDrawer.DrawLine(ref raw[2], ref raw[0], ref BLUE);

                //draw normal?
                Vector3 d = raw[1] - raw[0];
                Vector3 e = raw[2] - raw[0];
                // Reverse the cross here to account for winding, shouldn't change the way rest of bullet works.
				Vector3 cross = Vector3.Cross(d, e);
				//Vector3 cross = Vector3.Cross(e,d);

                Vector3 colour = new Vector3(1, 0, 1);
                int ibreak = 0;
                Vector3 center = (raw[0]+raw[1]+raw[2])*(1.0f/3.0f);

                cross += center;
				m_shapeDrawer.DrawLine(ref center, ref cross, ref colour);                

		    } 
            else
		    {
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

        public virtual void InternalProcessTriangleIndex(ObjectArray<Vector3> triangle, int partId, int triangleIndex)
	    {
            m_shapeDrawer.DrawSolidTriangle(triangle);
	    }

        public virtual void Cleanup()
        {
        }

        private XNA_ShapeDrawer m_shapeDrawer;
    }
}
