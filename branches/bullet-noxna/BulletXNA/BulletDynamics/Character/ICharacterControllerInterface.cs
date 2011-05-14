﻿/*
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

using BulletXNA.BulletCollision.CollisionDispatch;
using BulletXNA.BulletDynamics.Dynamics;
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletDynamics.Character
{
    public interface ICharacterControllerInterface : IActionInterface
    {
	    void	SetWalkDirection(ref Vector3 walkDirection);
        void    SetVelocityForTimeInterval(ref Vector3 velocity, float timeInterval);
        void	Reset ();
	    void	Warp (ref Vector3 origin);
	    void	PreStep (CollisionWorld collisionWorld);
	    void	PlayerStep (CollisionWorld collisionWorld, float dt);
	    bool	CanJump ();
	    void	Jump ();

	    bool	OnGround ();

    }
}
