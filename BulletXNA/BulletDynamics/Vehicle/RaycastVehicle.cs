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

#define ROLLING_INFLUENCE_FIX

using System;
using System.Collections.Generic;
using System.Diagnostics;
using BulletXNA.BulletCollision;
using BulletXNA.LinearMath;


namespace BulletXNA.BulletDynamics
{
    public class RaycastVehicle : IActionInterface
    {
        public RaycastVehicle(VehicleTuning tuning, RigidBody chassis, IVehicleRaycaster raycaster)
        {
            m_vehicleRaycaster = raycaster;
            m_pitchControl = 0f;
            m_chassisBody = chassis;
            m_indexRightAxis = 0;
            m_indexUpAxis = 1;
            m_indexForwardAxis = 2;
            DefaultInit(ref tuning);
        }

	    ///btActionInterface interface
	    public virtual void UpdateAction(CollisionWorld collisionWorld, float step)
	    {
		    UpdateVehicle(step);
	    }
    	

	    ///btActionInterface interface
        public virtual void DebugDraw(IDebugDraw debugDrawer)
        {
	        for (int v=0;v<GetNumWheels();v++)
	        {
		        IndexedVector3 wheelColor = new IndexedVector3(0,1,1);
		        if (GetWheelInfo(v).m_raycastInfo.m_isInContact)
		        {
			        wheelColor = new IndexedVector3(0,0,1);
		        } else
		        {
			        wheelColor= new IndexedVector3(1,0,1);
		        }

		        IndexedVector3 wheelPosWS = GetWheelInfo(v).m_worldTransform._origin;

                IndexedMatrix temp = GetWheelInfo(v).m_worldTransform;
                IndexedVector3 axle = new IndexedVector3(
        GetWheelInfo(v).m_worldTransform._basis._el0[GetRightAxis()],
        GetWheelInfo(v).m_worldTransform._basis._el1[GetRightAxis()],
        GetWheelInfo(v).m_worldTransform._basis._el2[GetRightAxis()]);

		        //debug wheels (cylinders)
		        debugDrawer.DrawLine(wheelPosWS,wheelPosWS+axle,wheelColor);
		        debugDrawer.DrawLine(wheelPosWS,GetWheelInfo(v).m_raycastInfo.m_contactPointWS,wheelColor);

	        }
        }

        public IndexedMatrix GetChassisWorldTransform()
        {
            /*if (getRigidBody()->getMotionState())
            {
                btTransform chassisWorldTrans;
                getRigidBody()->getMotionState()->getWorldTransform(chassisWorldTrans);
                return chassisWorldTrans;
            }
            */
            return GetRigidBody().GetCenterOfMassTransform();

        }

        public float RayCast(WheelInfo wheel)
        {
            UpdateWheelTransformsWS(wheel, false);

            float depth = -1;

            float raylen = wheel.GetSuspensionRestLength() + wheel.m_wheelsRadius;

            IndexedVector3 rayvector = wheel.m_raycastInfo.m_wheelDirectionWS * (raylen);
            IndexedVector3 source = wheel.m_raycastInfo.m_hardPointWS;
            wheel.m_raycastInfo.m_contactPointWS = source + rayvector;
            IndexedVector3 target = wheel.m_raycastInfo.m_contactPointWS;

            float param = 0f;

            VehicleRaycasterResult rayResults = new VehicleRaycasterResult();

            Debug.Assert(m_vehicleRaycaster != null);

            Object object1 = m_vehicleRaycaster.CastRay(ref source, ref target, ref rayResults);

            if (object1 != null && (object1 as RigidBody).m_debugBodyId != 1)
            {
                int ibreak = 0;
            }

            //{
            //    IndexedVector3 from1 = new IndexedVector3(0.7957098f, -9.13606f, 1.794605f);
            //    IndexedVector3 to1 = new IndexedVector3(0.886791f, -10.23207f, 1.815941f);
            //    VehicleRaycasterResult results1  = new VehicleRaycasterResult();
            //    Object o2 = m_vehicleRaycaster.castRay(ref from1, ref to1, ref results1);

            //    IndexedVector3 from2 = new IndexedVector3(0.7957281f, -9.136093f, 1.794625f);
            //    IndexedVector3 to2 = new IndexedVector3(0.8867911f, -10.23211f, 1.815956f);
            //    VehicleRaycasterResult results2 = new VehicleRaycasterResult();
            //    Object o3 = m_vehicleRaycaster.castRay(ref from2, ref to2, ref results2);

            //    if (Math.Abs(results1.m_distFraction - results2.m_distFraction) > 0.1f)
            //    {
            //        int ibreak = 0;
            //    }
            //}



            wheel.m_raycastInfo.m_groundObject = null;

            if (object1 != null)
            {
                param = rayResults.m_distFraction;
                depth = raylen * rayResults.m_distFraction;
                wheel.m_raycastInfo.m_contactNormalWS = rayResults.m_hitNormalInWorld;
                wheel.m_raycastInfo.m_isInContact = true;

                wheel.m_raycastInfo.m_groundObject = s_fixedObject;///@todo for driving on dynamic/movable objects!;
                //wheel.m_raycastInfo.m_groundObject = object;

                float hitDistance = param * raylen;
                wheel.m_raycastInfo.m_suspensionLength = hitDistance - wheel.m_wheelsRadius;



                //clamp on max suspension travel

                float minSuspensionLength = wheel.GetSuspensionRestLength() - wheel.m_maxSuspensionTravelCm * 0.01f;
                float maxSuspensionLength = wheel.GetSuspensionRestLength() + wheel.m_maxSuspensionTravelCm * 0.01f;
                if (wheel.m_raycastInfo.m_suspensionLength < minSuspensionLength)
                {
                    wheel.m_raycastInfo.m_suspensionLength = minSuspensionLength;
                }
                if (wheel.m_raycastInfo.m_suspensionLength > maxSuspensionLength)
                {
                    wheel.m_raycastInfo.m_suspensionLength = maxSuspensionLength;
                }

                if (Math.Abs(wheel.m_raycastInfo.m_suspensionLength - wheel.m_raycastInfo.m_suspensionLengthBak) > 0.1f)
                {
                    int ibreak = 0;
                }

                wheel.m_raycastInfo.m_suspensionLengthBak = wheel.m_raycastInfo.m_suspensionLength;

                wheel.m_raycastInfo.m_contactPointWS = rayResults.m_hitPointInWorld;

                float denominator = IndexedVector3.Dot(wheel.m_raycastInfo.m_contactNormalWS, wheel.m_raycastInfo.m_wheelDirectionWS);

                IndexedVector3 chassis_velocity_at_contactPoint;
                IndexedVector3 relpos = wheel.m_raycastInfo.m_contactPointWS - GetRigidBody().GetCenterOfMassPosition();

                chassis_velocity_at_contactPoint = GetRigidBody().GetVelocityInLocalPoint(ref relpos);

                float projVel = IndexedVector3.Dot(wheel.m_raycastInfo.m_contactNormalWS, chassis_velocity_at_contactPoint);

                if (projVel > 1f)
                {
                    int ibreak = 0;
                }

                if (denominator >= -0.1f)
                {
                    wheel.m_suspensionRelativeVelocity = 0f;
                    wheel.m_clippedInvContactDotSuspension = 1.0f / 0.1f;
                }
                else
                {
                    float inv = -1f / denominator;
                    wheel.m_suspensionRelativeVelocity = projVel * inv;
                    wheel.m_clippedInvContactDotSuspension = inv;
                }

            }
            else
            {
                //put wheel info as in rest position
                wheel.m_raycastInfo.m_suspensionLength = wheel.GetSuspensionRestLength();
                wheel.m_suspensionRelativeVelocity = 0.0f;
                wheel.m_raycastInfo.m_contactNormalWS = -wheel.m_raycastInfo.m_wheelDirectionWS;
                wheel.m_clippedInvContactDotSuspension = 1.0f;
            }
    
            return depth;
        }

        public virtual void UpdateVehicle(float step)
        {
            int numWheels = GetNumWheels();

	        {
		        for (int i=0;i<numWheels;i++)
		        {
			        UpdateWheelTransform(i,false);
		        }
	        }

	        m_currentVehicleSpeedKmHour = 3.6f * GetRigidBody().GetLinearVelocity().Length();
        	
	        IndexedMatrix chassisTrans = GetChassisWorldTransform();

            IndexedVector3 forwardW = new IndexedVector3(
                chassisTrans._basis[0,m_indexForwardAxis],
                chassisTrans._basis[1,m_indexForwardAxis],
                chassisTrans._basis[2,m_indexForwardAxis]);


	        if (IndexedVector3.Dot(forwardW,GetRigidBody().GetLinearVelocity()) < 0f)
	        {
		        m_currentVehicleSpeedKmHour *= -1f;
	        }

	        //
	        // simulate suspension
	        //
            float[] depthData = new float[numWheels];
            float depth = 0f;
	        for (int i=0;i<numWheels;i++)
	        {
		        depth = RayCast( m_wheelInfo[i]);
                depthData[i] = depth;

                if (m_wheelInfo[i].m_raycastInfo.m_isInContact == false)
                {
                    int ibreak = 0;
                    depth = RayCast(m_wheelInfo[i]);
                    depthData[i] = depth;
                }

            }

            UpdateSuspension(step);

        	
	        for(int i=0;i<numWheels;++i)
	        {
                WheelInfo wheel = m_wheelInfo[i];
		        //apply suspension force
		        float suspensionForce = wheel.m_wheelsSuspensionForce;

                if (suspensionForce > wheel.m_maxSuspensionForce)
                {
                    suspensionForce = wheel.m_maxSuspensionForce;
                }

                IndexedVector3 impulse = wheel.m_raycastInfo.m_contactNormalWS * suspensionForce * step;
		        IndexedVector3 relpos = wheel.m_raycastInfo.m_contactPointWS - GetRigidBody().GetCenterOfMassPosition();
                if (impulse.Y < 30 || impulse.Y > 40)
                {
                    int ibreak = 0;
                }
                //impulse = new IndexedVector3(0f, 1f, 0f);
                GetRigidBody().ApplyImpulse(ref impulse, ref relpos);
        	
	        }

            UpdateFriction(step);

	        for(int i=0;i<numWheels;++i)
	        {
                WheelInfo wheel = m_wheelInfo[i];
		        IndexedVector3 relpos = wheel.m_raycastInfo.m_hardPointWS - GetRigidBody().GetCenterOfMassPosition();
		        IndexedVector3 vel = GetRigidBody().GetVelocityInLocalPoint( ref relpos );

		        if (wheel.m_raycastInfo.m_isInContact)
		        {
			        IndexedMatrix chassisWorldTransform = GetChassisWorldTransform();

                    IndexedVector3 fwd = new IndexedVector3(
                        chassisWorldTransform._basis[0,m_indexForwardAxis],
                        chassisWorldTransform._basis[1,m_indexForwardAxis],
                        chassisWorldTransform._basis[2,m_indexForwardAxis]);

			        float proj = IndexedVector3.Dot(fwd,wheel.m_raycastInfo.m_contactNormalWS);
			        fwd -= wheel.m_raycastInfo.m_contactNormalWS * proj;

			        float proj2 = IndexedVector3.Dot(fwd,vel);
        			
			        wheel.m_deltaRotation = (proj2 * step) / (wheel.m_wheelsRadius);
			        wheel.m_rotation += wheel.m_deltaRotation;

		        } 
                else
		        {
			        wheel.m_rotation += wheel.m_deltaRotation;
		        }
        		
		        wheel.m_deltaRotation *= 0.99f;//damping of rotation when not in contact
	        }
        }

        public void ResetSuspension()
        {
            foreach(WheelInfo wheel in m_wheelInfo)
            {
                wheel.m_raycastInfo.m_suspensionLength = wheel.GetSuspensionRestLength();
                wheel.m_suspensionRelativeVelocity = 0f;

                wheel.m_raycastInfo.m_contactNormalWS = -wheel.m_raycastInfo.m_wheelDirectionWS;
                //wheel_info.setContactFriction(float(0.0));
                wheel.m_clippedInvContactDotSuspension = 1f;
            }
        }

        public float GetSteeringValue(int wheel)
        {
            return GetWheelInfo(wheel).m_steering;
        }

        public void SetSteeringValue(float steering, int wheel)
        {
            Debug.Assert(wheel >= 0 && wheel < GetNumWheels());

            WheelInfo  wheelInfo = GetWheelInfo(wheel);
            wheelInfo.m_steering = steering;
        }

        public void ApplyEngineForce(float force, int wheel)
        {
            Debug.Assert(wheel >= 0 && wheel < GetNumWheels());
            WheelInfo  wheelInfo = GetWheelInfo(wheel);
            wheelInfo.m_engineForce = force;
        }

        public IndexedMatrix GetWheelTransformWS(int wheelIndex)
        {
	        Debug.Assert(wheelIndex < GetNumWheels());
	        WheelInfo wheel = m_wheelInfo[wheelIndex];
	        return wheel.m_worldTransform;
        }

        public void UpdateWheelTransform(int wheelIndex, bool interpolatedTransform)
        {
	        WheelInfo wheel = m_wheelInfo[ wheelIndex ];

            UpdateWheelTransformsWS(wheel,interpolatedTransform);
	        IndexedVector3 up = -wheel.m_raycastInfo.m_wheelDirectionWS;
	        IndexedVector3 right = wheel.m_raycastInfo.m_wheelAxleWS;
	        IndexedVector3 fwd = IndexedVector3.Cross(up,right);
	        fwd.Normalize();
        //	up = right.cross(fwd);
        //	up.normalize();

	        //rotate around steering over de wheelAxleWS
	        float steering = wheel.m_steering;
        	
	        IndexedQuaternion steeringOrn = new IndexedQuaternion(up,steering);//wheel.m_steering);
            IndexedBasisMatrix steeringMat = new IndexedBasisMatrix(ref steeringOrn);

            IndexedQuaternion rotatingOrn = new IndexedQuaternion(right, -wheel.m_rotation);
            IndexedBasisMatrix rotatingMat = new IndexedBasisMatrix(ref rotatingOrn);


            IndexedBasisMatrix basis2 = new IndexedBasisMatrix(
                right.X,fwd.X,up.X,
                right.Y,fwd.Y,up.Y,
                right.Z,fwd.Z,up.Z
            );
        	
            // FIXME MAN - MATRIX ORDER
            //wheel.m_worldTransform = steeringMat * rotatingMat * basis2;
            wheel.m_worldTransform._basis = steeringMat * rotatingMat * basis2;
	        wheel.m_worldTransform._origin = 
		        wheel.m_raycastInfo.m_hardPointWS + (wheel.m_raycastInfo.m_wheelDirectionWS * wheel.m_raycastInfo.m_suspensionLength);
       
        }

        public WheelInfo AddWheel(ref IndexedVector3 connectionPointCS0, ref IndexedVector3 wheelDirectionCS0, ref IndexedVector3 wheelAxleCS, float suspensionRestLength, float wheelRadius, VehicleTuning tuning, bool isFrontWheel)
        {
            WheelInfoConstructionInfo ci = new WheelInfoConstructionInfo();

            ci.m_chassisConnectionCS = connectionPointCS0;
            ci.m_wheelDirectionCS = wheelDirectionCS0;
            ci.m_wheelAxleCS = wheelAxleCS;
            ci.m_suspensionRestLength = suspensionRestLength;
            ci.m_wheelRadius = wheelRadius;
            ci.m_suspensionStiffness = tuning.m_suspensionStiffness;
            ci.m_wheelsDampingCompression = tuning.m_suspensionCompression;
            ci.m_wheelsDampingRelaxation = tuning.m_suspensionDamping;
            ci.m_frictionSlip = tuning.m_frictionSlip;
            ci.m_bIsFrontWheel = isFrontWheel;
            ci.m_maxSuspensionTravelCm = tuning.m_maxSuspensionTravelCm;
            ci.m_maxSuspensionForce = tuning.m_maxSuspensionForce;

            WheelInfo wheel = new WheelInfo(ref ci);
            m_wheelInfo.Add(wheel);

            UpdateWheelTransformsWS(wheel, false);
            UpdateWheelTransform(GetNumWheels() - 1, false);
            return wheel;
        }

	    public int GetNumWheels() 
        {
		    return m_wheelInfo.Count;
	    }

        public virtual WheelInfo GetWheelInfo(int index)
        {
            return m_wheelInfo[index];
        }

        public void UpdateWheelTransformsWS(WheelInfo wheel, bool interpolatedTransform)
        {
            wheel.m_raycastInfo.m_isInContact = false;

            IndexedMatrix chassisTrans = GetChassisWorldTransform();
            if (interpolatedTransform && (GetRigidBody().GetMotionState() != null))
            {
                GetRigidBody().GetMotionState().GetWorldTransform(out chassisTrans);
            }

            wheel.m_raycastInfo.m_hardPointWS = chassisTrans * wheel.m_chassisConnectionPointCS;
            wheel.m_raycastInfo.m_wheelDirectionWS = chassisTrans._basis * wheel.m_wheelDirectionCS;
            wheel.m_raycastInfo.m_wheelAxleWS = chassisTrans._basis * wheel.m_wheelAxleCS;
        }

        public void SetBrake(float brake, int wheelIndex)
        {
            Debug.Assert((wheelIndex >= 0) && (wheelIndex < GetNumWheels()));
            GetWheelInfo(wheelIndex).m_brake = brake;
        }

	    public void	SetPitchControl(float pitch)
	    {
		    m_pitchControl = pitch;
	    }
    	
	    public void	UpdateSuspension(float deltaTime)
        {
	        float chassisMass = 1f / m_chassisBody.GetInvMass();
        	
	        for (int w_it=0; w_it<GetNumWheels(); w_it++)
	        {
		        WheelInfo wheel_info = m_wheelInfo[w_it];
        		
		        if ( wheel_info.m_raycastInfo.m_isInContact )
		        {
			        float force = 0f;
			        //	Spring
			        {
                        float susp_length = wheel_info.GetSuspensionRestLength();
                        float current_length = wheel_info.m_raycastInfo.m_suspensionLength;

                        float length_diff = (susp_length - current_length);

                        //length_diff = current_length;

                        force = wheel_info.m_suspensionStiffness
                            * length_diff * wheel_info.m_clippedInvContactDotSuspension;
			        }
        		
			        // Damper
			        {
				        float projected_rel_vel = wheel_info.m_suspensionRelativeVelocity;
				        {
					        float susp_damping;
					        if ( projected_rel_vel < 0f )
					        {
						        susp_damping = wheel_info.m_wheelsDampingCompression;
					        }
					        else
					        {
						        susp_damping = wheel_info.m_wheelsDampingRelaxation;
					        }
					        force -= susp_damping * projected_rel_vel;
				        }
			        }

			        // RESULT
			        wheel_info.m_wheelsSuspensionForce = force * chassisMass;
			        if (wheel_info.m_wheelsSuspensionForce < 0f)
			        {
				        wheel_info.m_wheelsSuspensionForce = 0f;
			        }
		        }
		        else
		        {
			        wheel_info.m_wheelsSuspensionForce = 0f;
		        }
	        }
        }

        public virtual void UpdateFriction(float timeStep)
        {
	        //calculate the impulse, so that the wheels don't move sidewards
	        int numWheel = GetNumWheels();
	        if (numWheel == 0)
		        return;

            //m_forwardWS.resize(numWheel);
            //m_axle.resize(numWheel);
            //m_forwardImpulse.resize(numWheel);
            //m_sideImpulse.resize(numWheel);
    		
	        int numWheelsOnGround = 0;
    	

	        //collapse all those loops into one!
	        for (int i=0;i<numWheel;i++)
	        {
		        WheelInfo wheelInfo = m_wheelInfo[i];
                RigidBody groundObject = wheelInfo.m_raycastInfo.m_groundObject as RigidBody;
		        if (groundObject != null)
                {
			        numWheelsOnGround++;
                }
		        m_sideImpulse[i] = 0f;
		        m_forwardImpulse[i] = 0f;

	        }

            if (numWheelsOnGround != 4)
            {
                int ibreak = 0;
            }

	        {
    	
                //foreach(WheelInfo wheelInfo in m_wheelInfo)
		        for(int i=0;i<numWheel;++i)
                {
                    WheelInfo wheelInfo = m_wheelInfo[i];
                    RigidBody groundObject = wheelInfo.m_raycastInfo.m_groundObject as RigidBody;

			        if (groundObject != null)
			        {

				        IndexedMatrix wheelTrans = GetWheelTransformWS( i );

				        IndexedBasisMatrix wheelBasis0 = wheelTrans._basis;
                        m_axle[i] = new IndexedVector3(
                        wheelBasis0._el0[m_indexRightAxis],
                        wheelBasis0._el1[m_indexRightAxis],
                        wheelBasis0._el2[m_indexRightAxis]);
    					
				        IndexedVector3 surfNormalWS = wheelInfo.m_raycastInfo.m_contactNormalWS;
				        float proj = IndexedVector3.Dot(m_axle[i],surfNormalWS);
				        m_axle[i] -= surfNormalWS * proj;
				        m_axle[i].Normalize();
    					
				        m_forwardWS[i] = IndexedVector3.Cross(surfNormalWS,m_axle[i]);
				        m_forwardWS[i].Normalize();

                        IndexedVector3 tempAxle = m_axle[i];
                        float tempImpulse = m_sideImpulse[i];
                        ContactConstraint.ResolveSingleBilateral(m_chassisBody, ref wheelInfo.m_raycastInfo.m_contactPointWS,
                                  groundObject, ref wheelInfo.m_raycastInfo.m_contactPointWS,
                                  0f, ref tempAxle, ref tempImpulse, timeStep);
                        m_sideImpulse[i] = (tempImpulse * sideFrictionStiffness2);
			        }
		        }
	        }

            float sideFactor = 1f;
            float fwdFactor = 0.5f;

            bool sliding = false;
            {
	            for (int wheel =0;wheel <numWheel;wheel++)
	            {
		            WheelInfo wheelInfo = m_wheelInfo[wheel];
                    RigidBody groundObject = wheelInfo.m_raycastInfo.m_groundObject as RigidBody;

		            float	rollingFriction = 0f;

		            if(groundObject != null)
		            {
                        if(wheelInfo.m_engineForce != 0.0f)
			            {
				            rollingFriction = wheelInfo.m_engineForce* timeStep;
			            } 
                        else
			            {
				            float defaultRollingFrictionImpulse = 0f;
				            float maxImpulse = (wheelInfo.m_brake != 0f) ? wheelInfo.m_brake : defaultRollingFrictionImpulse;
                            IndexedVector3 tempWheel = m_forwardWS[wheel];
				            WheelContactPoint contactPt = new WheelContactPoint(m_chassisBody,groundObject,ref wheelInfo.m_raycastInfo.m_contactPointWS,ref tempWheel,maxImpulse);
                            m_forwardWS[wheel] = tempWheel;
				            rollingFriction = CalcRollingFriction(contactPt);
			            }
		            }

		            //switch between active rolling (throttle), braking and non-active rolling friction (no throttle/break)

		            m_forwardImpulse[wheel] = 0f;
		            m_wheelInfo[wheel].m_skidInfo= 1f;

		            if (groundObject != null)
		            {
			            m_wheelInfo[wheel].m_skidInfo= 1f;
        				
			            float maximp = wheelInfo.m_wheelsSuspensionForce * timeStep * wheelInfo.m_frictionSlip;
			            float maximpSide = maximp;

			            float maximpSquared = maximp * maximpSide;
			            m_forwardImpulse[wheel] = rollingFriction;//wheelInfo.m_engineForce* timeStep;

			            float x = (m_forwardImpulse[wheel] ) * fwdFactor;
			            float y = (m_sideImpulse[wheel] ) * sideFactor;
        				
			            float impulseSquared = (x*x + y*y);

			            if (impulseSquared > maximpSquared)
			            {
				            sliding = true;
        					
				            float factor = (float)(maximp / Math.Sqrt(impulseSquared));
        					
				            m_wheelInfo[wheel].m_skidInfo *= factor;
			            }
		            } 

	            }
            }

            if (sliding)
            {
                for (int wheel = 0; wheel < numWheel; wheel++)
                {
                    if (m_sideImpulse[wheel] != 0f)
                    {
                        if (m_wheelInfo[wheel].m_skidInfo < 1f)
                        {
                            m_forwardImpulse[wheel] *= m_wheelInfo[wheel].m_skidInfo;
                            m_sideImpulse[wheel] *= m_wheelInfo[wheel].m_skidInfo;
                        }
                    }
                }
            }

            // apply the impulses
            {
	            for (int wheel = 0;wheel<numWheel ; wheel++)
	            {
		            WheelInfo wheelInfo = m_wheelInfo[wheel];

		            IndexedVector3 rel_pos = wheelInfo.m_raycastInfo.m_contactPointWS - 
				            m_chassisBody.GetCenterOfMassPosition();


                    if (m_forwardImpulse[wheel] > 5f || m_sideImpulse[wheel] > 5f)
                    {
                        int ibreak = 0;
                    }

		            if (m_forwardImpulse[wheel] != 0f)
		            {
                        m_chassisBody.ApplyImpulse(m_forwardWS[wheel] * (m_forwardImpulse[wheel]), rel_pos);
		            }
                    if (m_sideImpulse[wheel] != 0f)
                    {
                        RigidBody groundObject = m_wheelInfo[wheel].m_raycastInfo.m_groundObject as RigidBody;

                        IndexedVector3 rel_pos2 = wheelInfo.m_raycastInfo.m_contactPointWS -
                            groundObject.GetCenterOfMassPosition();

                        IndexedVector3 sideImp = m_axle[wheel] * m_sideImpulse[wheel];

#if ROLLING_INFLUENCE_FIX // fix. It only worked if car's up was along Y - VT.
                        IndexedVector3 vChassisWorldUp = GetRigidBody().GetCenterOfMassTransform()._basis.GetColumn(m_indexUpAxis);
                        rel_pos -= vChassisWorldUp * (IndexedVector3.Dot(vChassisWorldUp, rel_pos) * (1.0f - wheelInfo.m_rollInfluence));
#else
					rel_pos[m_indexUpAxis] *= wheelInfo.m_rollInfluence;
#endif


                        m_chassisBody.ApplyImpulse(ref sideImp, ref rel_pos);

                        //apply friction impulse on the ground
                        IndexedVector3 temp = -sideImp;
                        groundObject.ApplyImpulse(ref temp, ref rel_pos2);
                    }
	            }
	        }
        }

	    public RigidBody GetRigidBody()
	    {
		    return m_chassisBody;
	    }

	    public int GetRightAxis()
	    {
		    return m_indexRightAxis;
	    }

        public int GetUpAxis()
	    {
		    return m_indexUpAxis;
	    }

	    public int GetForwardAxis()
	    {
		    return m_indexForwardAxis;
	    }
    	
	    ///Worldspace forward vector
	    public IndexedVector3 GetForwardVector() 
	    {
		    IndexedMatrix chassisTrans = GetChassisWorldTransform();

            IndexedVector3 forwardW = new IndexedVector3( 
                  chassisTrans._basis[0,m_indexForwardAxis], 
                  chassisTrans._basis[1,m_indexForwardAxis], 
                  chassisTrans._basis[2,m_indexForwardAxis]); 

		    return forwardW;
	    }

	    ///Velocity of vehicle (positive if velocity vector has same direction as foward vector)
	    public float GetCurrentSpeedKmHour()
	    {
		    return m_currentVehicleSpeedKmHour;
	    }

	    public virtual void	SetCoordinateSystem(int rightIndex,int upIndex,int forwardIndex)
	    {
		    m_indexRightAxis = rightIndex;
		    m_indexUpAxis = upIndex;
		    m_indexForwardAxis = forwardIndex;
	    }

        ///backwards compatibility
	    public int GetUserConstraintType()
	    {
		    return m_userConstraintType ;
	    }

	    public void SetUserConstraintType(int userConstraintType)
	    {
		    m_userConstraintType = userConstraintType;
	    }

	    public void	SetUserConstraintId(int uid)
	    {
		    m_userConstraintId = uid;
	    }

	    public int GetUserConstraintId()
	    {
		    return m_userConstraintId;
	    }

        public static float CalcRollingFriction(WheelContactPoint contactPoint)
        {
	        float j1=0f;

	        IndexedVector3 contactPosWorld = contactPoint.m_frictionPositionWorld;

	        IndexedVector3 rel_pos1 = contactPosWorld - contactPoint.m_body0.GetCenterOfMassPosition(); 
	        IndexedVector3 rel_pos2 = contactPosWorld - contactPoint.m_body1.GetCenterOfMassPosition();
        	
	        float maxImpulse  = contactPoint.m_maxImpulse;
        	
	        IndexedVector3 vel1 = contactPoint.m_body0.GetVelocityInLocalPoint(ref rel_pos1);
	        IndexedVector3 vel2 = contactPoint.m_body1.GetVelocityInLocalPoint(ref rel_pos2);
	        IndexedVector3 vel = vel1 - vel2;

	        float vrel = IndexedVector3.Dot(contactPoint.m_frictionDirectionWorld,vel);

	        // calculate j that moves us to zero relative velocity
	        j1 = -vrel * contactPoint.m_jacDiagABInv;
	        j1 = Math.Min(j1,maxImpulse);
	        j1 = Math.Max(j1, -maxImpulse);

	        return j1;
        }

        private void DefaultInit(ref VehicleTuning tuning)
        {
            //(void)tuning;
	        m_currentVehicleSpeedKmHour = 0f;
	        m_steeringValue = 0f;

        }

	    private float	m_tau;
	    private float	m_damping;
	    private IVehicleRaycaster m_vehicleRaycaster;
	    private float m_pitchControl;
	    private float m_steeringValue; 
	    private float m_currentVehicleSpeedKmHour;

	    private RigidBody m_chassisBody;

	    private int m_indexRightAxis;
	    private int m_indexUpAxis;
	    private int	m_indexForwardAxis;


        //private IList<IndexedVector3> m_forwardWS = new List<IndexedVector3>();
        //private IList<IndexedVector3> m_axle = new List<IndexedVector3>();
        //private IList<float> m_forwardImpulse = new List<float>();
        //private IList<float> m_sideImpulse = new List<float>();
        private IndexedVector3[] m_axle = new IndexedVector3[4];
        private IndexedVector3[] m_forwardWS = new IndexedVector3[4];

        private float[] m_forwardImpulse = new float[4];
        private float[] m_sideImpulse = new float[4];

        ///backwards compatibility
        private int m_userConstraintType;
        private int m_userConstraintId;

	    private IList<WheelInfo>	m_wheelInfo = new List<WheelInfo>();

        public const float sideFrictionStiffness2 = 1.0f;
        public static RigidBody s_fixedObject = new RigidBody(0f, null, null,IndexedVector3.Zero);
    }

	public class VehicleTuning
	{
		public VehicleTuning()
		{
            m_suspensionStiffness = 5.88f;
            m_suspensionCompression = 0.83f;
            m_suspensionDamping = 0.88f;
            m_maxSuspensionTravelCm = 500f;
            m_frictionSlip = 10.5f;
            m_maxSuspensionForce = 6000f;

		}
		public float m_suspensionStiffness;
        public float m_suspensionCompression;
        public float m_suspensionDamping;
        public float m_maxSuspensionTravelCm;
        public float m_frictionSlip;
        public float m_maxSuspensionForce;

	}

    public class DefaultVehicleRaycaster : IVehicleRaycaster
    {
	    private DynamicsWorld m_dynamicsWorld;

        private struct DataCopy
        {
            IndexedVector3 m_from;
            IndexedVector3 m_to;
            VehicleRaycasterResult m_result;

            public DataCopy(IndexedVector3 from,IndexedVector3 to,VehicleRaycasterResult result)
            {
                m_from = from;
                m_to = to;
                m_result = result;
            }
        }

	    public DefaultVehicleRaycaster(DynamicsWorld world)
	    {
            m_dynamicsWorld = world;
	    }

	    public virtual Object CastRay(ref IndexedVector3 from,ref IndexedVector3 to, ref VehicleRaycasterResult result)
        {
            //	RayResultCallback& resultCallback;
	        ClosestRayResultCallback rayCallback = new ClosestRayResultCallback(ref from,ref to);

	        m_dynamicsWorld.RayTest(ref from, ref to, rayCallback);

            if (rayCallback.HasHit())
            {

                RigidBody body = RigidBody.Upcast(rayCallback.m_collisionObject);
                if (body != null && body.HasContactResponse())
                {
                    result.m_hitPointInWorld = rayCallback.m_hitPointWorld;
                    result.m_hitNormalInWorld = rayCallback.m_hitNormalWorld;
                    result.m_hitNormalInWorld.Normalize();
                    result.m_distFraction = rayCallback.m_closestHitFraction;
                    return body;
                }
            }
            else
            {
                int ibreak = 0;
                ClosestRayResultCallback rayCallback2 = new ClosestRayResultCallback(ref from, ref to);

                m_dynamicsWorld.RayTest(ref from, ref to, rayCallback2);

            }
            rayCallback.Cleanup();
            return null;
        }
    }

    public class WheelContactPoint
    {
	    public RigidBody m_body0;
        public RigidBody m_body1;
        public IndexedVector3 m_frictionPositionWorld;
        public IndexedVector3 m_frictionDirectionWorld;
        public float m_jacDiagABInv;
        public float m_maxImpulse;

	    public WheelContactPoint(RigidBody body0,RigidBody body1,ref IndexedVector3 frictionPosWorld,ref IndexedVector3 frictionDirectionWorld, float maxImpulse)
	    {
            m_body0 = body0;
            m_body1 = body1;
            m_frictionPositionWorld = frictionPosWorld;
            m_frictionDirectionWorld = frictionDirectionWorld;
            m_maxImpulse = maxImpulse;
		    float denom0 = body0.ComputeImpulseDenominator(ref frictionPosWorld,ref frictionDirectionWorld);
		    float denom1 = body1.ComputeImpulseDenominator(ref frictionPosWorld,ref frictionDirectionWorld);
		    float relaxation = 1f;
		    m_jacDiagABInv = relaxation/(denom0+denom1);
	    }
    }
}
