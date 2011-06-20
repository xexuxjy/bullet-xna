#region License
/*
MIT License
Copyright © 2006 The Mono.Xna Team

All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#endregion License

using System;
using System.Collections;
using System.ComponentModel;
using System.ComponentModel.Design.Serialization;
using System.Globalization;
using System.Reflection;

namespace Microsoft.Xna.Framework.Design
{
    public class Vector3Converter : MathTypeConverter
    {
		#region Constructor
		
        public Vector3Converter()
        {
			Type type = typeof(Vector3);
            base.propertyDescriptions = new PropertyDescriptorCollection(new PropertyDescriptor[] { 
				new FieldDescriptor(type.GetField("X")), new FieldDescriptor(type.GetField("Y")), new FieldDescriptor(type.GetField("Z")) 
			});
        }
		
		#endregion Constructor
		
		#region MathTypeConverter Overrides

        public override object ConvertFrom(ITypeDescriptorContext context, CultureInfo culture, object value)
        {
            if (value.GetType() == typeof(string) && supportStringConvert)
			{
				float[] values = MathTypeConverter.ConvertStringToValues(context, culture, (string)value);
				return new Vector3(values[0], values[1], values[2]);
			}
			return base.ConvertFrom(context, culture, value);
        }

        public override object ConvertTo(ITypeDescriptorContext context, CultureInfo culture, object value, Type destinationType)
        {
			Vector3 vector = (Vector3)value; 
            if (destinationType == typeof(string) && supportStringConvert)
			{
				float[] values = new float[] { vector.X, vector.Y, vector.Z };						
				return MathTypeConverter.ConvertValuesToString(context, culture, values);
			}
			else if (destinationType == typeof(InstanceDescriptor))
			{
				MemberInfo constructorInfo = typeof(Vector3).GetConstructor(new Type[] { typeof(float), typeof(float), typeof(float) });
				object[] parameters = new object[] { vector.X, vector.Y, vector.Z };
				return new InstanceDescriptor(constructorInfo, parameters);	
			}
			
			return base.ConvertTo(context, culture, value, destinationType);
        }

        public override object CreateInstance(ITypeDescriptorContext context, IDictionary propertyValues)
        {
            throw new NotImplementedException();
        }
		
		#endregion MathTypeConverter Overrides
    }
}
