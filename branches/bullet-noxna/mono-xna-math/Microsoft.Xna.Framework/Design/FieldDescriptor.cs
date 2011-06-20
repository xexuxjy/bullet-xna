
using System;
using System.Reflection;
using System.ComponentModel;

namespace Microsoft.Xna.Framework
{	
	
	internal class FieldDescriptor : PropertyDescriptor
	{
		private FieldInfo FieldInfo_;
		
		public FieldDescriptor(FieldInfo Field) :base(Field.Name, (Attribute[])Field.GetCustomAttributes(typeof(Attribute), true))
		{
			this.FieldInfo_ = Field;
		}
		
		public override bool CanResetValue(object component)
		{
			return false;
		}
		
		public override bool ShouldSerializeValue (object component)
		{
			return true;
		}
		
		public override void ResetValue (object component)
		{
		}

		public override void SetValue (object component, object value)
		{
			this.FieldInfo_.SetValue(component,value);
			this.OnValueChanged(component, EventArgs.Empty);
		}

		public override object GetValue (object component)
		{
			return this.FieldInfo_.GetValue(component);
		}

		public override Type PropertyType {
			get {
				return this.FieldInfo_.FieldType;
			}
		}

		public override bool IsReadOnly {
			get {
				return false;
			}
		}
		
		public override Type ComponentType {
			get {
				return this.FieldInfo_.DeclaringType;
			}
		}
		
		public FieldInfo Field {
			get {
				return this.FieldInfo_;
			}
		}

		
	}
}
