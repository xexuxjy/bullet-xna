using System;
using System.Windows.Forms;
using BulletXNA.LinearMath;

namespace DemoFramework
{
    public class FreeLook
    {
        public Vector3 Eye { get; private set; }
        public Vector3 Target { get; private set; }
        public Vector3 Up { get; set; }

        Input input;
        MouseController mouseController;
        bool doUpdate;

        public FreeLook(Input input)
        {
            Target = Vector3.UnitX;
            Up = Vector3.UnitY;
            this.input = input;
            mouseController = new MouseController(input);
        }

        public void SetEyeTarget(Vector3 eye, Vector3 target)
        {
            Eye = eye;
            Target = target;

            // Convert direction vector to Y-up for MouseController
            Matrix swapAxis = Matrix.CreateRotationAxis(Vector3.Cross(Up, Vector3.UnitY), Angle(Up, Vector3.UnitY));
            mouseController.Vector = swapAxis * Vector3.Normalize(eye - target);

            doUpdate = true;
        }

        public bool Update(float frameDelta)
        {
            if (mouseController.Update() == false && input.KeysDown.Count == 0)
            {
                if (!doUpdate)
                    return false;
                doUpdate = false;
            }

            // MouseController is Y-up, convert to Up-up
            Matrix swapAxis = Matrix.CreateRotationAxis(Vector3.Cross(Vector3.UnitY, Up), Angle(Vector3.UnitY, Up));
            Vector3 direction = swapAxis * -mouseController.Vector;

            if (input.KeysDown.Count != 0)
            {
                Vector3 relDirection = frameDelta * direction;
                float flySpeed = input.KeysDown.Contains(Keys.ShiftKey) ? 15 : 5;

                if (input.KeysDown.Contains(Keys.W))
                {
                    Eye += flySpeed * relDirection;
                }
                if (input.KeysDown.Contains(Keys.S))
                {
                    Eye -= flySpeed * relDirection;
                }

                if (input.KeysDown.Contains(Keys.A))
                {
                    Eye += Vector3.Cross(relDirection, Up);
                }
                if (input.KeysDown.Contains(Keys.D))
                {
                    Eye -= Vector3.Cross(relDirection, Up);
                }
            }
            Target = Eye + (Eye - Target).Length() * direction;

            return true;
        }

        // vertices must be normalized
        float Angle(Vector3 v1, Vector3 v2)
        {
            return (float)Math.Acos(Vector3.Dot(v1, v2));
        }
    }
}
