using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BulletXNADemos.Demos
{
    public class BSPDemo : DemoApplication
    {
        public BSPDemo()
        {
        }

        //----------------------------------------------------------------------------------------------------------------

        ~BSPDemo()
        {
        }

        //----------------------------------------------------------------------------------------------------------------

        //----------------------------------------------------------------------------------------------------------------

        static void Main(string[] args)
        {
            using (BSPDemo game = new BSPDemo())
            {
                game.Run();
            }
        }

    }
}
