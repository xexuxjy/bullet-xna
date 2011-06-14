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
using BulletXNA.BulletCollision;
using BulletXNA.BulletDynamics;
using BulletXNA.LinearMath;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace BulletXNA
{
	public static class DrawHelper
	{
		public static void DebugDrawObject(ref Matrix worldTransform, CollisionShape shape, ref Vector3 color, IDebugDraw debugDraw)
		{
			// Draw a small simplex at the center of the object
			{
				//Vector3 start = worldTransform.Translation;
				//float scale = 10f;
				//debugDraw.DrawLine(start, start + (Vector3.TransformNormal(Vector3.Right, worldTransform) * scale), Vector3.Right);
				//debugDraw.DrawLine(start, start + (Vector3.TransformNormal(Vector3.Up, worldTransform) * scale), Vector3.Up);
				//debugDraw.DrawLine(start, start + (Vector3.TransformNormal(Vector3.Backward, worldTransform) * scale), Vector3.Backward);
				debugDraw.DrawTransform(ref worldTransform, 1.0f);
			}
			//return;
			if (shape.GetShapeType() == BroadphaseNativeTypes.COMPOUND_SHAPE_PROXYTYPE)
			{
				CompoundShape compoundShape = (CompoundShape)shape;
				for (int i = compoundShape.GetNumChildShapes() - 1; i >= 0; i--)
				{
					Matrix childTrans = compoundShape.GetChildTransform(i);
					CollisionShape colShape = compoundShape.GetChildShape(i);
					Matrix temp = MathUtil.BulletMatrixMultiply(worldTransform, childTrans);
					DebugDrawObject(ref temp, colShape, ref color, debugDraw);
				}
			}
			else
			{
				switch (shape.GetShapeType())
				{
					case (BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE):
						{
							BoxShape boxShape = shape as BoxShape;
							Vector3 halfExtents = boxShape.GetHalfExtentsWithMargin();
							Vector3 negHalfExtents = -halfExtents;
							debugDraw.DrawBox(ref negHalfExtents, ref halfExtents, ref worldTransform, ref color);
							break;
						}

					case BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE:
						{
							SphereShape sphereShape = shape as SphereShape;
							float radius = sphereShape.GetMargin();//radius doesn't include the margin, so draw with margin
							DebugDrawSphere(radius, ref worldTransform, ref color, debugDraw);
							break;
						}
					case BroadphaseNativeTypes.MULTI_SPHERE_SHAPE_PROXYTYPE:
						{
							MultiSphereShape multiSphereShape = (MultiSphereShape)shape;

							for (int i = multiSphereShape.GetSphereCount() - 1; i >= 0; i--)
							{
								Matrix childTransform = worldTransform;
								childTransform.Translation += multiSphereShape.GetSpherePosition(i);
								DebugDrawSphere(multiSphereShape.GetSphereRadius(i), ref childTransform, ref color, debugDraw);
							}

							break;
						}
					case BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE:
						{
							CapsuleShape capsuleShape = shape as CapsuleShape;

							float radius = capsuleShape.getRadius();
							float halfHeight = capsuleShape.getHalfHeight();

							int upAxis = capsuleShape.GetUpAxis();



							Vector3 capStart = Vector3.Zero; ;
							MathUtil.VectorComponent(ref capStart, upAxis, -halfHeight);

							Vector3 capEnd = Vector3.Zero;
							MathUtil.VectorComponent(ref capEnd, upAxis, halfHeight);

							// Draw the ends
							{

								Matrix childTransform = worldTransform;
								childTransform.Translation = Vector3.Transform(capStart, worldTransform);
								DebugDrawSphere(radius, ref childTransform, ref color, debugDraw);
							}

							{
								Matrix childTransform = worldTransform;
								childTransform.Translation = Vector3.Transform(capEnd, worldTransform);
								DebugDrawSphere(radius, ref childTransform, ref color, debugDraw);
							}

							// Draw some additional lines
							Vector3 start = worldTransform.Translation;

							MathUtil.VectorComponent(ref capStart, (upAxis + 1) % 3, radius);
							MathUtil.VectorComponent(ref capEnd, (upAxis + 1) % 3, radius);
							debugDraw.DrawLine(start + Vector3.TransformNormal(capStart, worldTransform), start + Vector3.TransformNormal(capEnd, worldTransform), color);

							MathUtil.VectorComponent(ref capStart, (upAxis + 1) % 3, -radius);
							MathUtil.VectorComponent(ref capEnd, (upAxis + 1) % 3, -radius);
							debugDraw.DrawLine(start + Vector3.TransformNormal(capStart, worldTransform), start + Vector3.TransformNormal(capEnd, worldTransform), color);

							MathUtil.VectorComponent(ref capStart, (upAxis + 1) % 3, radius);
							MathUtil.VectorComponent(ref capEnd, (upAxis + 1) % 3, radius);

							MathUtil.VectorComponent(ref capStart, (upAxis + 2) % 3, radius);
							MathUtil.VectorComponent(ref capEnd, (upAxis + 2) % 3, radius);
							debugDraw.DrawLine(start + Vector3.TransformNormal(capStart, worldTransform), start + Vector3.TransformNormal(capEnd, worldTransform), color);

							MathUtil.VectorComponent(ref capStart, (upAxis + 2) % 3, -radius);
							MathUtil.VectorComponent(ref capEnd, (upAxis + 2) % 3, -radius);
							debugDraw.DrawLine(start + Vector3.TransformNormal(capStart, worldTransform), start + Vector3.TransformNormal(capEnd, worldTransform), color);

							break;
						}
					case BroadphaseNativeTypes.CONE_SHAPE_PROXYTYPE:
						{
							ConeShape coneShape = (ConeShape)shape;
							float radius = coneShape.GetRadius();//+coneShape->getMargin();
							float height = coneShape.GetHeight();//+coneShape->getMargin();
							Vector3 start = worldTransform.Translation;

							int upAxis = coneShape.GetConeUpIndex();


							Vector3 offsetHeight = Vector3.Zero;
							MathUtil.VectorComponent(ref offsetHeight, upAxis, height * 0.5f);
							Vector3 offsetRadius = Vector3.Zero;
							MathUtil.VectorComponent(ref offsetRadius, (upAxis + 1) % 3, radius);

							Vector3 offset2Radius = Vector3.Zero;
							MathUtil.VectorComponent(ref offsetRadius, (upAxis + 2) % 3, radius);

							debugDraw.DrawLine(start + Vector3.TransformNormal(offsetHeight, worldTransform), start + Vector3.TransformNormal(-offsetHeight + offsetRadius, worldTransform), color);
							debugDraw.DrawLine(start + Vector3.TransformNormal(offsetHeight, worldTransform), start + Vector3.TransformNormal(-offsetHeight - offsetRadius, worldTransform), color);
							debugDraw.DrawLine(start + Vector3.TransformNormal(offsetHeight, worldTransform), start + Vector3.TransformNormal(-offsetHeight + offset2Radius, worldTransform), color);
							debugDraw.DrawLine(start + Vector3.TransformNormal(offsetHeight, worldTransform), start + Vector3.TransformNormal(-offsetHeight - offset2Radius, worldTransform), color);

							break;

						}
					case BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE:
						{
							CylinderShape cylinder = (CylinderShape)shape;
							int upAxis = cylinder.GetUpAxis();
							float radius = cylinder.GetRadius();

							float halfHeight = MathUtil.VectorComponent(cylinder.GetHalfExtentsWithMargin(), upAxis);
							Vector3 start = worldTransform.Translation;
							Vector3 offsetHeight = Vector3.Zero;
							MathUtil.VectorComponent(ref offsetHeight, upAxis, halfHeight);
							Vector3 offsetRadius = Vector3.Zero;
							MathUtil.VectorComponent(ref offsetRadius, (upAxis + 1) % 3, radius);
							debugDraw.DrawLine(start + Vector3.TransformNormal(offsetHeight + offsetRadius, worldTransform), start + Vector3.TransformNormal(-offsetHeight + offsetRadius, worldTransform), color);
							debugDraw.DrawLine(start + Vector3.TransformNormal(offsetHeight - offsetRadius, worldTransform), start + Vector3.TransformNormal(-offsetHeight - offsetRadius, worldTransform), color);
							break;
						}

					case BroadphaseNativeTypes.STATIC_PLANE_PROXYTYPE:
						{
							StaticPlaneShape staticPlaneShape = shape as StaticPlaneShape;
							float planeConst = staticPlaneShape.GetPlaneConstant();
							Vector3 planeNormal = staticPlaneShape.GetPlaneNormal();
							Vector3 planeOrigin = planeNormal * planeConst;
							Vector3 vec0, vec1;
							TransformUtil.PlaneSpace1(ref planeNormal, out vec0, out vec1);
							float vecLen = 100f;
							Vector3 pt0 = planeOrigin + vec0 * vecLen;
							Vector3 pt1 = planeOrigin - vec0 * vecLen;
							Vector3 pt2 = planeOrigin + vec1 * vecLen;
							Vector3 pt3 = planeOrigin - vec1 * vecLen;
							debugDraw.DrawLine(Vector3.Transform(pt0, worldTransform), Vector3.Transform(pt1, worldTransform), color);
							debugDraw.DrawLine(Vector3.Transform(pt2, worldTransform), Vector3.Transform(pt3, worldTransform), color);
							break;

						}
					default:
						{
							if (shape.IsConcave())
							{
								ConcaveShape concaveMesh = (ConcaveShape)shape;

								///@todo pass camera, for some culling? no -> we are not a graphics lib
								Vector3 aabbMax = MathUtil.MAX_VECTOR;
								Vector3 aabbMin = MathUtil.MIN_VECTOR;

								DebugDrawcallback drawCallback = new DebugDrawcallback(debugDraw, ref worldTransform, ref color);
								concaveMesh.ProcessAllTriangles(drawCallback, ref aabbMin, ref aabbMax);
								drawCallback.Cleanup();
							}
							else if (shape.GetShapeType() == BroadphaseNativeTypes.CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE)
							{
								ConvexTriangleMeshShape convexMesh = (ConvexTriangleMeshShape)shape;
								//todo: pass camera for some culling			
								Vector3 aabbMax = MathUtil.MAX_VECTOR;
								Vector3 aabbMin = MathUtil.MIN_VECTOR;

								//DebugDrawcallback drawCallback;
								DebugDrawcallback drawCallback = new DebugDrawcallback(debugDraw, ref worldTransform, ref color);
								convexMesh.GetMeshInterface().InternalProcessAllTriangles(drawCallback, ref aabbMin, ref aabbMax);
								drawCallback.Cleanup();
							}
							else if (shape.IsPolyhedral())/// for polyhedral shapes
							{
								PolyhedralConvexShape polyshape = (PolyhedralConvexShape)shape;
								if (polyshape.GetConvexPolyhedron() != null)
								{
									ConvexPolyhedron poly = polyshape.GetConvexPolyhedron();
									for (int i = 0; i < poly.m_faces.Count; i++)
									{
										Vector3 centroid = Vector3.Zero;
										int numVerts = poly.m_faces[i].m_indices.Count;
										if (numVerts != 0)
										{
											int lastV = poly.m_faces[i].m_indices[numVerts - 1];
											for (int v = 0; v < poly.m_faces[i].m_indices.Count; v++)
											{
												int curVert = poly.m_faces[i].m_indices[v];
												centroid += poly.m_vertices[curVert];
												debugDraw.DrawLine(Vector3.Transform(poly.m_vertices[lastV], worldTransform), Vector3.Transform(poly.m_vertices[curVert], worldTransform), color);
												lastV = curVert;
											}
										}
										centroid *= 1.0f / (float)(numVerts);

										Vector3 normalColor = new Vector3(1, 1, 0);
										Vector3 faceNormal = new Vector3(poly.m_faces[i].m_plane[0], poly.m_faces[i].m_plane[1], poly.m_faces[i].m_plane[2]);
										debugDraw.DrawLine(Vector3.Transform(centroid, worldTransform), Vector3.Transform((centroid + faceNormal), worldTransform), normalColor);
									}

								}
								else
								{
									for (int i = 0; i < polyshape.GetNumEdges(); i++)
									{
										Vector3 a, b;
										polyshape.GetEdge(i, out a, out b);
										Vector3 wa = Vector3.Transform(a, worldTransform);
										Vector3 wb = Vector3.Transform(b, worldTransform);
										debugDraw.DrawLine(ref wa, ref wb, ref color);
									}
								}
							}
							break;
						}
				}
			}
		}

		public static void DebugDrawConstraint(TypedConstraint constraint, IDebugDraw debugDraw)
		{
			bool drawFrames = (debugDraw.GetDebugMode() & DebugDrawModes.DBG_DrawConstraints) != 0;
			bool drawLimits = (debugDraw.GetDebugMode() & DebugDrawModes.DBG_DrawConstraintLimits) != 0;
			float dbgDrawSize = constraint.GetDbgDrawSize();
			if (dbgDrawSize <= 0f)
			{
				return;
			}

			switch (constraint.GetConstraintType())
			{
				case TypedConstraintType.POINT2POINT_CONSTRAINT_TYPE:
					{
						Point2PointConstraint p2pC = constraint as Point2PointConstraint;
						Matrix tr = Matrix.Identity;
						Vector3 pivot = p2pC.GetPivotInA();
						pivot = Vector3.Transform(pivot, p2pC.GetRigidBodyA().GetCenterOfMassTransform());
						tr.Translation = pivot;
						debugDraw.DrawTransform(ref tr, dbgDrawSize);
						// that ideally should draw the same frame	
						pivot = p2pC.GetPivotInB();
						pivot = Vector3.Transform(pivot, p2pC.GetRigidBodyB().GetCenterOfMassTransform());
						tr.Translation = pivot;
						if (drawFrames) debugDraw.DrawTransform(ref tr, dbgDrawSize);
					}
					break;
				case TypedConstraintType.HINGE_CONSTRAINT_TYPE:
					{
						HingeConstraint pHinge = constraint as HingeConstraint;
						Matrix tr = MathUtil.BulletMatrixMultiply(pHinge.GetRigidBodyA().GetCenterOfMassTransform(), pHinge.GetAFrame());
						if (drawFrames)
						{
							debugDraw.DrawTransform(ref tr, dbgDrawSize);
						}
						tr = MathUtil.BulletMatrixMultiply(pHinge.GetRigidBodyB().GetCenterOfMassTransform(), pHinge.GetBFrame());
						if (drawFrames)
						{
							debugDraw.DrawTransform(ref tr, dbgDrawSize);
						}
						float minAng = pHinge.GetLowerLimit();
						float maxAng = pHinge.GetUpperLimit();
						if (minAng == maxAng)
						{
							break;
						}
						bool drawSect = true;
						if (minAng > maxAng)
						{
							minAng = 0f;
							maxAng = MathUtil.SIMD_2_PI;
							drawSect = false;
						}
						if (drawLimits)
						{
							Vector3 center = tr.Translation;
							Vector3 normal = MathUtil.MatrixColumn(ref tr, 2);
							Vector3 axis = MathUtil.MatrixColumn(ref tr, 0);
							Vector3 zero = Vector3.Zero;
							debugDraw.DrawArc(ref center, ref normal, ref axis, dbgDrawSize, dbgDrawSize, minAng, maxAng, ref zero, drawSect);
						}
					}
					break;
				case TypedConstraintType.CONETWIST_CONSTRAINT_TYPE:
					{
						ConeTwistConstraint pCT = constraint as ConeTwistConstraint;
						Matrix tr = MathUtil.BulletMatrixMultiply(pCT.GetRigidBodyA().GetCenterOfMassTransform(), pCT.GetAFrame());
						if (drawFrames) debugDraw.DrawTransform(ref tr, dbgDrawSize);
						tr = MathUtil.BulletMatrixMultiply(pCT.GetRigidBodyB().GetCenterOfMassTransform(), pCT.GetBFrame());
						if (drawFrames) debugDraw.DrawTransform(ref tr, dbgDrawSize);
						Vector3 zero = Vector3.Zero;

						if (drawLimits)
						{
							//const float length = float(5);
							float length = dbgDrawSize;
							int nSegments = 8 * 4;
							float fAngleInRadians = MathUtil.SIMD_2_PI * (float)(nSegments - 1) / (float)nSegments;
							Vector3 pPrev = pCT.GetPointForAngle(fAngleInRadians, length);
							pPrev = Vector3.Transform(pPrev, tr);
							for (int i = 0; i < nSegments; i++)
							{
								fAngleInRadians = MathUtil.SIMD_2_PI * (float)i / (float)nSegments;
								Vector3 pCur = pCT.GetPointForAngle(fAngleInRadians, length);
								pCur = Vector3.Transform(pCur, tr);
								debugDraw.DrawLine(ref pPrev, ref pCur, ref zero);

								if (i % (nSegments / 8) == 0)
								{
									Vector3 origin = tr.Translation;
									debugDraw.DrawLine(ref origin, ref pCur, ref zero);
								}

								pPrev = pCur;
							}
							float tws = pCT.GetTwistSpan();
							float twa = pCT.GetTwistAngle();
							bool useFrameB = (pCT.GetRigidBodyB().GetInvMass() > 0f);
							if (useFrameB)
							{
								tr = MathUtil.BulletMatrixMultiply(pCT.GetRigidBodyB().GetCenterOfMassTransform(), pCT.GetBFrame());
							}
							else
							{
								tr = MathUtil.BulletMatrixMultiply(pCT.GetRigidBodyA().GetCenterOfMassTransform(), pCT.GetAFrame());
							}
							Vector3 pivot = tr.Translation;
							Vector3 normal = MathUtil.MatrixColumn(ref tr, 0);
							Vector3 axis1 = MathUtil.MatrixColumn(ref tr, 1);

							debugDraw.DrawArc(ref pivot, ref normal, ref axis1, dbgDrawSize, dbgDrawSize, -twa - tws, -twa + tws, ref zero, true);
						}
					}
					break;
				case TypedConstraintType.D6_CONSTRAINT_TYPE:
				case TypedConstraintType.D6_SPRING_CONSTRAINT_TYPE:
					{
						Generic6DofConstraint p6DOF = constraint as Generic6DofConstraint;
						Matrix tr = p6DOF.GetCalculatedTransformA();
						if (drawFrames)
						{
							debugDraw.DrawTransform(ref tr, dbgDrawSize);
						}
						tr = p6DOF.GetCalculatedTransformB();
						if (drawFrames)
						{
							debugDraw.DrawTransform(ref tr, dbgDrawSize);
						}
						Vector3 zero = Vector3.Zero;
						if (drawLimits)
						{
							tr = p6DOF.GetCalculatedTransformA();
							Vector3 center = p6DOF.GetCalculatedTransformB().Translation;
							// up is axis 1 not 2 ?
							Vector3 up = MathUtil.MatrixColumn(ref tr, 1);
							Vector3 axis = MathUtil.MatrixColumn(ref tr, 0);
							float minTh = p6DOF.GetRotationalLimitMotor(1).m_loLimit;
							float maxTh = p6DOF.GetRotationalLimitMotor(1).m_hiLimit;
							float minPs = p6DOF.GetRotationalLimitMotor(2).m_loLimit;
							float maxPs = p6DOF.GetRotationalLimitMotor(2).m_hiLimit;
							debugDraw.DrawSpherePatch(ref center, ref up, ref axis, dbgDrawSize * .9f, minTh, maxTh, minPs, maxPs, ref zero);
							axis = MathUtil.MatrixColumn(ref tr, 1);
							float ay = p6DOF.GetAngle(1);
							float az = p6DOF.GetAngle(2);
							float cy = (float)Math.Cos(ay);
							float sy = (float)Math.Sin(ay);
							float cz = (float)Math.Cos(az);
							float sz = (float)Math.Sin(az);
							Vector3 ref1 = new Vector3(
							    cy * cz * axis.X + cy * sz * axis.Y - sy * axis.Z,
							    -sz * axis.X + cz * axis.Y,
							    cz * sy * axis.X + sz * sy * axis.Y + cy * axis.Z);
							tr = p6DOF.GetCalculatedTransformB();
							Vector3 normal = -MathUtil.MatrixColumn(ref tr, 0);
							float minFi = p6DOF.GetRotationalLimitMotor(0).m_loLimit;
							float maxFi = p6DOF.GetRotationalLimitMotor(0).m_hiLimit;
							if (minFi > maxFi)
							{
								debugDraw.DrawArc(ref center, ref normal, ref ref1, dbgDrawSize, dbgDrawSize, -MathUtil.SIMD_PI, MathUtil.SIMD_PI, ref zero, false);
							}
							else if (minFi < maxFi)
							{
								debugDraw.DrawArc(ref center, ref normal, ref ref1, dbgDrawSize, dbgDrawSize, minFi, maxFi, ref zero, false);
							}
							tr = p6DOF.GetCalculatedTransformA();
							Vector3 bbMin = p6DOF.GetTranslationalLimitMotor().m_lowerLimit;
							Vector3 bbMax = p6DOF.GetTranslationalLimitMotor().m_upperLimit;
							debugDraw.DrawBox(ref bbMin, ref bbMax, ref tr, ref zero);
						}
					}
					break;
				case TypedConstraintType.SLIDER_CONSTRAINT_TYPE:
					{
						SliderConstraint pSlider = constraint as SliderConstraint;
						Matrix tr = pSlider.GetCalculatedTransformA();
						if (drawFrames) debugDraw.DrawTransform(ref tr, dbgDrawSize);
						tr = pSlider.GetCalculatedTransformB();
						if (drawFrames) debugDraw.DrawTransform(ref tr, dbgDrawSize);
						Vector3 zero = Vector3.Zero;
						if (drawLimits)
						{
							Matrix tr2 = pSlider.GetCalculatedTransformA();
							Vector3 li_min = Vector3.Transform(new Vector3(pSlider.GetLowerLinLimit(), 0f, 0f), tr2);
							Vector3 li_max = Vector3.Transform(new Vector3(pSlider.GetUpperLinLimit(), 0f, 0f), tr2);
							debugDraw.DrawLine(ref li_min, ref li_max, ref zero);
							Vector3 normal = MathUtil.MatrixColumn(ref tr, 0);
							Vector3 axis = MathUtil.MatrixColumn(ref tr, 1);
							float a_min = pSlider.GetLowerAngLimit();
							float a_max = pSlider.GetUpperAngLimit();
							Vector3 center = pSlider.GetCalculatedTransformB().Translation;
							debugDraw.DrawArc(ref center, ref normal, ref axis, dbgDrawSize, dbgDrawSize, a_min, a_max, ref zero, true);
						}
					}
					break;
				default:
					break;
			}
			return;
		}


		private static void DebugDrawSphere(float radius, ref Matrix transform, ref Vector3 color, IDebugDraw debugDraw)
		{
			Vector3 start = transform.Translation;

			Vector3 xoffs = Vector3.TransformNormal(new Vector3(radius, 0, 0), transform);
			Vector3 yoffs = Vector3.TransformNormal(new Vector3(0, radius, 0), transform);
			Vector3 zoffs = Vector3.TransformNormal(new Vector3(0, 0, radius), transform);

			// XY 
			debugDraw.DrawLine(start - xoffs, start + yoffs, color);
			debugDraw.DrawLine(start + yoffs, start + xoffs, color);
			debugDraw.DrawLine(start + xoffs, start - yoffs, color);
			debugDraw.DrawLine(start - yoffs, start - xoffs, color);

			// XZ
			debugDraw.DrawLine(start - xoffs, start + zoffs, color);
			debugDraw.DrawLine(start + zoffs, start + xoffs, color);
			debugDraw.DrawLine(start + xoffs, start - zoffs, color);
			debugDraw.DrawLine(start - zoffs, start - xoffs, color);

			// YZ
			debugDraw.DrawLine(start - yoffs, start + zoffs, color);
			debugDraw.DrawLine(start + zoffs, start + yoffs, color);
			debugDraw.DrawLine(start + yoffs, start - zoffs, color);
			debugDraw.DrawLine(start - zoffs, start - yoffs, color);

		}

		public static ShapeData CreateCube()
		{
			Matrix identity = Matrix.Identity;
			return CreateBox(Vector3.Zero, new Vector3(1), Color.Yellow, ref identity);
		}

		public static ShapeData CreateBox(Vector3 position, Vector3 sideLength, Color color, ref Matrix transform)
		{
			ShapeData shapeData = new ShapeData(8, 36);
			int index = 0;
			shapeData.m_verticesArray[index++] = new VertexPositionColor(Vector3.Transform(position + new Vector3(0, 0, 0), transform), color);
			shapeData.m_verticesArray[index++] = new VertexPositionColor(Vector3.Transform(position + new Vector3(sideLength.X, 0, 0), transform), color);
			shapeData.m_verticesArray[index++] = new VertexPositionColor(Vector3.Transform(position + new Vector3(sideLength.X, 0, sideLength.Z), transform), color);
			shapeData.m_verticesArray[index++] = new VertexPositionColor(Vector3.Transform(position + new Vector3(0, 0, sideLength.Z), transform), color);
			shapeData.m_verticesArray[index++] = new VertexPositionColor(Vector3.Transform(position + new Vector3(0, sideLength.Y, 0), transform), color);
			shapeData.m_verticesArray[index++] = new VertexPositionColor(Vector3.Transform(position + new Vector3(sideLength.X, sideLength.Y, 0), transform), color);
			shapeData.m_verticesArray[index++] = new VertexPositionColor(Vector3.Transform(position + new Vector3(sideLength.X, sideLength.Y, sideLength.Z), transform), color);
			shapeData.m_verticesArray[index++] = new VertexPositionColor(Vector3.Transform(position + new Vector3(0, sideLength.Y, sideLength.Z), transform), color);
			shapeData.m_indexArray = DrawHelper.s_cubeIndices;
			return shapeData;
		}

		public static ShapeData CreateSphere(int slices, int stacks, float radius, Color color)
		{
			ShapeData shapeData = new ShapeData((slices + 1) * (stacks + 1), (slices * stacks * 6));

			float phi = 0f;
			float theta = 0f; ;
			float deltaPhi = MathHelper.Pi / stacks;
			float dtheta = MathHelper.TwoPi / slices;
			float x, y, z, sc;

			short index = 0;

			for (int stack = 0; stack <= stacks; stack++)
			{
				phi = MathHelper.PiOver2 - (stack * deltaPhi);
				y = radius * (float)Math.Sin(phi);
				sc = -radius * (float)Math.Cos(phi);

				for (int slice = 0; slice <= slices; slice++)
				{
					theta = slice * dtheta;
					x = sc * (float)Math.Sin(theta);
					z = sc * (float)Math.Cos(theta);

					//s_sphereVertices[index++] = new VertexPositionNormalTexture(new Vector3(x, y, z),
					//                            new Vector3(x, y, z),
					//                            new Vector2((float)slice / (float)slices, (float)stack / (float)stacks));

					shapeData.m_verticesArray[index++] = new VertexPositionColor(new Vector3(x, y, z), color);
				}
			}
			int stride = slices + 1;
			index = 0;
			for (int stack = 0; stack < stacks; stack++)
			{
				for (int slice = 0; slice < slices; slice++)
				{
					shapeData.m_indexList[index++] = (short)((stack + 0) * stride + slice);
					shapeData.m_indexList[index++] = (short)((stack + 1) * stride + slice);
					shapeData.m_indexList[index++] = (short)((stack + 0) * stride + slice + 1);

					shapeData.m_indexList[index++] = (short)((stack + 0) * stride + slice + 1);
					shapeData.m_indexList[index++] = (short)((stack + 1) * stride + slice);
					shapeData.m_indexList[index++] = (short)((stack + 1) * stride + slice + 1);
				}
			}
			return shapeData;
		}


		public static short[] s_cubeIndices = new short[]{
                             0,1,2,2,3,0, // face A
                             0,1,5,5,4,0, // face B
                             1,2,6,6,5,1, // face c
                             2,6,7,7,3,2, // face d
                             3,7,4,4,0,3, // face e
                             4,5,6,6,7,4}; // face f
	}

	public class DebugDrawcallback : ITriangleCallback, IInternalTriangleIndexCallback
	{
		IDebugDraw m_debugDrawer;
		Vector3 m_color;
		Matrix m_worldTrans;

		public virtual bool graphics()
		{
			return true;
		}

		public DebugDrawcallback(IDebugDraw debugDrawer, ref Matrix worldTrans, ref Vector3 color)
		{
			m_debugDrawer = debugDrawer;
			m_color = color;
			m_worldTrans = worldTrans;
		}

		public virtual void InternalProcessTriangleIndex(Vector3[] triangle, int partId, int triangleIndex)
		{
			ProcessTriangle(triangle, partId, triangleIndex);
		}

		public virtual void ProcessTriangle(Vector3[] triangle, int partId, int triangleIndex)
		{
			//(void)partId;
			//(void)triangleIndex;

			Vector3 wv0, wv1, wv2;
			Vector3.Transform(ref triangle[0], ref m_worldTrans, out wv0);
			Vector3.Transform(ref triangle[1], ref m_worldTrans, out wv1);
			Vector3.Transform(ref triangle[2], ref m_worldTrans, out wv2);

			m_debugDrawer.DrawLine(ref wv0, ref wv1, ref m_color);
			m_debugDrawer.DrawLine(ref wv1, ref wv2, ref m_color);
			m_debugDrawer.DrawLine(ref wv2, ref wv0, ref m_color);
		}

		//public static void drawUnitSphere(GraphicsDevice gd)
		//{
		//    gd.VertexDeclaration = s_vertexDeclaration;
		//    int primCount = s_sphereIndices.Length / 3;
		//    //int primCount = 2;
		//    int indexStart = 0;
		//    gd.DrawUserIndexedPrimitives(PrimitiveType.TriangleList, s_sphereVertices, 0, s_sphereVertices.Length, s_sphereIndices, indexStart, primCount);
		//}
		public virtual void Cleanup()
		{
		}

	}

	public class ShapeData
	{
		public ShapeData()
		{
			m_verticesList = new List<VertexPositionColor>();
			m_indexList = new List<short>();

		}

		public ShapeData(int numVert, int numIndices)
		{
			m_verticesArray = new VertexPositionColor[numVert];
			m_indexArray = new short[numIndices];

		}

		public VertexPositionColor[] m_verticesArray;
		public IList<VertexPositionColor> m_verticesList;
		public short[] m_indexArray; // This will remain fixed regardless so try and re-use
		public IList<short> m_indexList; // or if not appropiate then this one.
	}
}
