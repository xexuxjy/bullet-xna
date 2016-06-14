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
using BulletXNA.LinearMath;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;

namespace BulletXNADemos.Demos
{
    public class FrameRateCounter : DrawableGameComponent
    {
        ContentManager content;

        int frameRate = 0;
        int frameCounter = 0;
        TimeSpan elapsedTime = TimeSpan.Zero;
        Vector3 m_location;
        XNA_ShapeDrawer m_debugDraw;

        public FrameRateCounter(Game game, Vector3 location, XNA_ShapeDrawer debugDraw)
            : base(game)
        {
            content = new ContentManager(game.Services);
            m_location = location;
            m_debugDraw = debugDraw;
        }

        public void SetLocation(Vector3 location)
        {
            m_location = location;
        }


        //protected override void UnloadGraphicsContent(bool unloadAllContent)
        //{
        //    if (unloadAllContent)
        //        content.Unload();
        //}


        public override void Update(GameTime gameTime)
        {
            elapsedTime += gameTime.ElapsedGameTime;

            if (elapsedTime > TimeSpan.FromSeconds(1))
            {
                elapsedTime -= TimeSpan.FromSeconds(1);
                frameRate = frameCounter;
                frameCounter = 0;
            }
        }


        public override void Draw(GameTime gameTime)
        {
            frameCounter++;

            string fps = string.Format("fps: {0}", frameRate);
            IndexedVector3 colour = new IndexedVector3(1, 1, 1);
            m_debugDraw.DrawText(fps, new IndexedVector3(m_location), colour);
        }
    }
}
