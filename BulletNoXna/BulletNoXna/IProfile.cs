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

namespace BulletXNA
{
    public interface IProfile
    {
         void StartProfile(String name);
         void EndProfile(String name);
    }

    public interface IProfileManager
    {
	     void Start_Profile(String name);
	     void Stop_Profile();
         void CleanupMemory();
         void Reset();
         void Increment_Frame_Counter();
         int Get_Frame_Count_Since_Reset();
         float Get_Time_Since_Reset();

	     void	DumpRecursive(IProfileIterator profileIterator, int spacing);
         void DumpAll();

         IProfileIterator getIterator();

    }
    
    public interface IProfileIterator
    {
	     void First();
	     void Next();
	     bool Is_Done();
	     bool Is_Root();

	     void Enter_Child( int index );		// Make the given child the new parent
	     void Enter_Largest_Child();	// Make the largest child the new parent
	     void Enter_Parent();			// Make the current parent's parent the new parent

	    // Access the current child
	     String Get_Current_Name();
	     int Get_Current_Total_Calls();
	     float Get_Current_Total_Time();

	    // Access the current parent
         String Get_Current_Parent_Name();
         int Get_Current_Parent_Total_Calls();
         float Get_Current_Parent_Total_Time();

    }
}
