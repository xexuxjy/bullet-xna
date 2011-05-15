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
using System.ComponentModel;
using System.ComponentModel.Design.Serialization;
using System.Globalization;
using System.Text;

namespace BulletXNA.LinearMath.Design
{
    public class MathTypeConverter : ExpandableObjectConverter
    {
		#region Fields
		
        protected PropertyDescriptorCollection propertyDescriptions;
        protected bool supportStringConvert;
       
		#endregion Fields
		
		#region Constructor

        public MathTypeConverter()
        {
           supportStringConvert = true;
        }
		
		#endregion Constructor
		
		#region Methods
		
		internal static string ConvertValuesToString (ITypeDescriptorContext context, CultureInfo culture, float[] values)
		{
			StringBuilder ret = new StringBuilder();
			string delimiter = culture.TextInfo.ListSeparator + " ";
			
			TypeConverter converter = TypeDescriptor.GetConverter(typeof(float));
			for (int i = 0; i < values.Length; i++)
			{
				ret.Append(converter.ConvertTo(context, culture, values[i], typeof(string)));
				if (i < values.Length - 1)
					ret.Append(delimiter);
			}
			return ret.ToString();
		}
		
		internal static float[] ConvertStringToValues (ITypeDescriptorContext context, CultureInfo culture, string valuesString)
		{
			string[] delimiters = new string[] { culture.TextInfo.ListSeparator + " " };
			string[] valueStrings = valuesString.Split(delimiters, StringSplitOptions.None);
			float[] values = new float[valuesString.Length];
			
			TypeConverter converter = TypeDescriptor.GetConverter(typeof(float));
			for (int i = 0; i < values.Length; i++)
				values[i] = (float)converter.ConvertFrom(context, culture, valueStrings[i]);
			
			return values;
		}
		
		#endregion Methods
		
		#region ExpandableObjectConverter Overrides

        public override bool CanConvertFrom (ITypeDescriptorContext context, Type sourceType)
        {
			if (sourceType == typeof(string) && supportStringConvert)
				return true;
			else if (sourceType == typeof(InstanceDescriptor))
				return true;
			
			return base.CanConvertFrom(context,sourceType);
        }

        public override bool CanConvertTo (ITypeDescriptorContext context, Type destinationType)
        {
            if (destinationType == typeof(string) && supportStringConvert)
				return true;
			else if (destinationType == typeof(InstanceDescriptor))
			    return true;
			
			return base.CanConvertTo(context, destinationType);
        }

        public override bool GetCreateInstanceSupported (ITypeDescriptorContext context)
        {
            return true;
        }

        public override PropertyDescriptorCollection GetProperties (ITypeDescriptorContext context, object value, Attribute[] attributes)
        {
           return this.propertyDescriptions;
        }

        public override bool GetPropertiesSupported (ITypeDescriptorContext context)
        {
           return true;
        }
		
		#endregion ExpandableObjectConverter Overrides
		
    }
}
