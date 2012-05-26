/*
* Adapted from SlimMath
* Copyright (c) 2007-2010 SlimDX Group
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

using System.Runtime.InteropServices;

namespace BulletXNA.LinearMath
{
    /// <summary>
    /// Represents a three dimensional line based on a point in space and a direction.
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct Ray
    {
        /// <summary>
        /// The position in three dimensional space where the ray starts.
        /// </summary>
        public Vector3 Position;

        /// <summary>
        /// The normalized direction in which the ray points.
        /// </summary>
        public Vector3 Direction;

        /// <summary>
        /// Initializes a new instance of the <see cref="SlimMath.Ray"/> struct.
        /// </summary>
        /// <param name="position">The position in three dimensional space of the origin of the ray.</param>
        /// <param name="direction">The normalized direction of the ray.</param>
        public Ray(Vector3 position, Vector3 direction)
        {
            this.Position = position;
            this.Direction = direction;
        }
    }
}
