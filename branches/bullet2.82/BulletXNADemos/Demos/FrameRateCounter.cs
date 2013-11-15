
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
