/*
  Box2DX Copyright (c) 2008 Ihar Kalasouski http://code.google.com/p/box2dx
  Box2D original C++ version Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

using System;
using System.Collections.Generic;
using System.Text;

using UnityEngine;

namespace Box2DX.Common
{
	/// <summary>
	/// A transform contains translation and rotation.
	/// It is used to represent the position and orientation of rigid frames.
	/// </summary>
	public struct Transform
	{
		public Vector2 position;
		public Quaternion rotation;

		/// <summary>
		/// Initialize using a position vector and a rotation matrix.
		/// </summary>
		/// <param name="position"></param>
		/// <param name="R"></param>
		public Transform(Vector2 position, Quaternion rotation)
		{
			this.position = position;
			this.rotation = rotation;
		}

		/// <summary>
		/// Set this to the identity transform.
		/// </summary>
		public void SetIdentity()
		{
			position = Vector2.zero;
			rotation = Quaternion.identity;
		}
		
		public Vector2 InverseTransformPoint(Vector2 p) 
		{
			return Vector2.zero;
		}
		
		public Vector2 InverseTransformDirection(Vector2 d)
		{
			return Vector2.zero;
		}
		
		public Vector2 TransformPoint(Vector2 p)
		{	
			return rotation * p.ToVector3();
		}
		
		public Vector2 TransformDirection(Vector2 d) 
		{ 
			return Vector2.zero;
		}
	}
}
