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
	/// A 2-by-2 matrix. Stored in column-major order.
	/// </summary>
	public struct Mat22
	{
		public Vector2 Col1, Col2;

		/// <summary>
		/// Construct this matrix using columns.
		/// </summary>
		public Mat22(Vector2 c1, Vector2 c2)
		{
			Col1 = c1;
			Col2 = c2;
		}

		/// <summary>
		/// Construct this matrix using scalars.
		/// </summary>
		public Mat22(float a11, float a12, float a21, float a22)
		{
			Col1.x = a11; Col1.y = a21;
			Col2.x = a12; Col2.y = a22;
		}

		/// <summary>
		/// Construct this matrix using an angle. 
		/// This matrix becomes an orthonormal rotation matrix.
		/// </summary>
		public Mat22(float angle)
		{
			float c = (float)Mathf.Cos(angle), s = (float)Mathf.Sin(angle);
			Col1.x = c; Col2.x = -s;
			Col1.y = s; Col2.y = c;
		}

		/// <summary>
		/// Initialize this matrix using columns.
		/// </summary>
		public void Set(Vector2 c1, Vector2 c2)
		{
			Col1 = c1;
			Col2 = c2;
		}

		/// <summary>
		/// Initialize this matrix using an angle.
		/// This matrix becomes an orthonormal rotation matrix.
		/// </summary>
		public void Set(float angle)
		{
			float c = (float)System.Math.Cos(angle), s = (float)System.Math.Sin(angle);
			Col1.x = c; Col2.x = -s;
			Col1.y = s; Col2.y = c;
		}

		/// <summary>
		/// Set this to the identity matrix.
		/// </summary>
		public void SetIdentity()
		{
			Col1.x = 1.0f; Col2.x = 0.0f;
			Col1.y = 0.0f; Col2.y = 1.0f;
		}

		/// <summary>
		/// Set this matrix to all zeros.
		/// </summary>
		public void SetZero()
		{
			Col1.x = 0.0f; Col2.x = 0.0f;
			Col1.y = 0.0f; Col2.y = 0.0f;
		}

		/// <summary>
		/// Extract the angle from this matrix (assumed to be a rotation matrix).
		/// </summary>
		public float GetAngle()
		{
			return (float)System.Math.Atan2(Col1.y, Col1.x);
		}
		
		public Vector2 Multiply(Vector2 vector) { 
			return new Vector2(Col1.x * vector.x + Col2.x * vector.y, Col1.y * vector.x + Col2.y * vector.y);
		}
		
		/// <summary>
		/// Compute the inverse of this matrix, such that inv(A) * A = identity.
		/// </summary>
		public Mat22 GetInverse()
		{
			float a = Col1.x, b = Col2.x, c = Col1.y, d = Col2.y;
			Mat22 B = new Mat22();
			float det = a * d - b * c;
			Box2DXDebug.Assert(det != 0.0f);
			det = 1.0f / det;
			B.Col1.x = det * d; B.Col2.x = -det * b;
			B.Col1.y = -det * c; B.Col2.y = det * a;
			return B;
		}

		/// <summary>
		/// Solve A * x = b, where b is a column vector. This is more efficient
		/// than computing the inverse in one-shot cases.
		/// </summary>
		public Vector2 Solve(Vector2 b)
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

		public static Mat22 Identity { get { return new Mat22(1, 0, 0, 1); } }

		public static Mat22 operator +(Mat22 A, Mat22 B)
		{
			Mat22 C = new Mat22();
			C.Set(A.Col1 + B.Col1, A.Col2 + B.Col2);
			return C;
		}
	}
}
