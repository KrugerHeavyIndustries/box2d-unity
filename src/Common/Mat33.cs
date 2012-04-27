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

//r175

using System;
using System.Collections.Generic;
using System.Text;

using UnityEngine;

namespace Box2DX.Common
{
	/// <summary>
	/// A 3-by-3 matrix. Stored in column-major order.
	/// </summary>
	public struct Mat33
	{
		/// <summary>
		/// Construct this matrix using columns.
		/// </summary>
		public Mat33(Vector3 c1, Vector3 c2, Vector3 c3)
		{
			Col1 = c1;
			Col2 = c2;
			Col3 = c3;
		}

		/// <summary>
		/// Set this matrix to all zeros.
		/// </summary>
		public void SetZero()
		{
			Col1 = Vector3.zero;
			Col2 = Vector3.zero;
			Col3 = Vector3.zero;
		}

		/// <summary>
		/// Solve A * x = b, where b is a column vector. This is more efficient
		/// than computing the inverse in one-shot cases.
		/// </summary>
		public Vector3 Solve33(Vector3 b)
		{
			float det = Vector3.Dot(Col1, Vector3.Cross(Col2, Col3));
			Box2DXDebug.Assert(det != 0.0f);
			det = 1.0f / det;
			Vector3 x = new Vector3();
			x.x = det * Vector3.Dot(b, Vector3.Cross(Col2, Col3));
			x.y = det * Vector3.Dot(Col1, Vector3.Cross(b, Col3));
			x.z = det * Vector3.Dot(Col1, Vector3.Cross(Col2, b));
			return x;
		}

		/// <summary>
		/// Solve A * x = b, where b is a column vector. This is more efficient
		/// than computing the inverse in one-shot cases. Solve only the upper
		/// 2-by-2 matrix equation.
		/// </summary>
		public Vector2 Solve22(Vector2 b)
		{
			float a11 = Col1.x, a12 = Col2.x, a21 = Col1.y, a22 = Col2.y;
			float det = a11 * a22 - a12 * a21;
			Box2DXDebug.Assert(det != 0.0f);
			det = 1.0f / det;
			Vector2 x = new Vector2();
			x.x = det * (a22 * b.x - a12 * b.y);
			x.y = det * (a11 * b.y - a21 * b.x);
			return x;
		}

		public Vector3 Col1;
		public Vector3 Col2;
		public Vector3 Col3;
	}
}