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

namespace BulletXNA
{
    public class DefaultMotionState : IMotionState
    {
        public DefaultMotionState()
            : this(Matrix.Identity, Matrix.Identity)
        {
        }

        public DefaultMotionState(Matrix startTrans, Matrix centerOfMassOffset)
        {
            m_graphicsWorldTrans = startTrans;
            m_startWorldTrans = startTrans;
            m_centerOfMassOffset = centerOfMassOffset;
            m_userPointer = null;
        }

        ///synchronizes world transform from user to physics
        public virtual void GetWorldTransform(out Matrix centerOfMassWorldTrans)
        {
            centerOfMassWorldTrans = MathUtil.BulletMatrixMultiply(Matrix.Invert(m_centerOfMassOffset), m_graphicsWorldTrans);
        }

        ///synchronizes world transform from physics to user
        ///Bullet only calls the update of worldtransform for active objects
        public virtual void SetWorldTransform(Matrix centerOfMassWorldTrans)
        {
            SetWorldTransform(ref centerOfMassWorldTrans);
        }

        public virtual void SetWorldTransform(ref Matrix centerOfMassWorldTrans)
        {
            m_graphicsWorldTrans = MathUtil.BulletMatrixMultiply(centerOfMassWorldTrans, m_centerOfMassOffset);
        }
        public Matrix m_graphicsWorldTrans;
        public Matrix m_centerOfMassOffset;
        public Matrix m_startWorldTrans;
        public Object m_userPointer;
    }
}
