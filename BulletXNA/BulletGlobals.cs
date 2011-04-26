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
using System.IO;
using BulletXNA.BulletCollision.CollisionDispatch;
using BulletXNA.LinearMath;
using Microsoft.Xna.Framework;

namespace BulletXNA
{
    public static class BulletGlobals
    {
        public static bool gDisableDeactivation = false;
        public static int gOverlappingPairs = 0;
        public static float gDeactivationTime = 2f;
        public static int BT_BULLET_VERSION = 276;
        public static int RAND_MAX = int.MaxValue;

        public static float gContactBreakingThreshold = .02f;

        public static Random gRandom = new Random();

        public static Matrix IdentityMatrix = Matrix.Identity;

		public static IContactAddedCallback gContactAddedCallback;

        public static IDebugDraw gDebugDraw;

        public static StreamWriter g_streamWriter;

        public static IProfileManager g_profileManager;

        public static void StartProfile(String name)
        {
            if (g_profileManager != null)
            {
                g_profileManager.Start_Profile(name);
            }

        }
        public static void StopProfile()
        {
            if (g_profileManager != null)
            {
                g_profileManager.Stop_Profile();
            }
        }

        public static void ResetProfile()
        {
            if (g_profileManager != null)
            {
                g_profileManager.Reset();
            }
        }
    }
}
