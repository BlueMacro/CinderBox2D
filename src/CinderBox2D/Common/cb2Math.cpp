/*
* Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include <CinderBox2D/Common/cb2Math.h>

namespace cb2
{

ci::Vec2f solve( const ci::Matrix22f& m, const ci::Vec2f& b)
{
	float a11 = m.m00, a12 = m.m01, a21 = m.m10, a22 = m.m11;
	float det = a11 * a22 - a12 * a21;
	if (det != 0.0f)
	{
		det = 1.0f / det;
	}
	ci::Vec2f x;
	x.x = det * (a22 * b.x - a12 * b.y);
	x.y = det * (a11 * b.y - a21 * b.x);
	return x;
}

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
ci::Vec3f solve( const ci::Matrix33f& A, const ci::Vec3f& b)
{
  // todo: optimize math here
  ci::Vec3f ex = A.getColumn(0);
  ci::Vec3f ey = A.getColumn(1);
  ci::Vec3f ez = A.getColumn(2);

	float det = cb2Dot(ex, cb2Cross(ey, ez));
	if (det != 0.0f)
	{
		det = 1.0f / det;
	}
	ci::Vec3f x;
	x.x = det * cb2Dot(b, cb2Cross(ey, ez));
	x.y = det * cb2Dot(ex, cb2Cross(b, ez));
	x.z = det * cb2Dot(ex, cb2Cross(ey, b));
	return x;
}

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
ci::Vec2f solve22( const ci::Matrix33f& A, const ci::Vec2f& b)
{
	float a11 = A.m00, a12 = A.m10, a21 = A.m01, a22 = A.m11;
	float det = a11 * a22 - a12 * a21;
	if (det != 0.0f)
	{
		det = 1.0f / det;
	}
	ci::Vec2f x;
	x.x = det * (a22 * b.x - a12 * b.y);
	x.y = det * (a11 * b.y - a21 * b.x);
	return x;
}

///
void getInverse22( const ci::Matrix33f& A, ci::Matrix33f* M )
{
	float a = A.m00, b = A.m10, c = A.m01, d = A.m11;
	float det = a * d - b * c;
	if (det != 0.0f)
	{
		det = 1.0f / det;
	}

	M->m00 =  det * d;	M->m10 = -det * b; M->m02 = 0.0f;
	M->m01 = -det * c;	M->m11 =  det * a; M->m12 = 0.0f;
	M->m10 = 0.0f;      M->m21 = 0.0f;     M->m22 = 0.0f;
}

/// Returns the zero matrix if singular.
void getSymInverse33( const ci::Matrix33f& A, ci::Matrix33f* M )
{
  // todo: optimize math here
  ci::Vec3f ex = A.getColumn(0);
  ci::Vec3f ey = A.getColumn(1);
  ci::Vec3f ez = A.getColumn(2);

	float det = cb2Dot(ex, cb2Cross(ey, ez));

	if (det != 0.0f)
	{
		det = 1.0f / det;
	}

	float a11 = ex.x, a12 = ey.x, a13 = ez.x;
	float a22 = ey.y, a23 = ez.y;
	float a33 = ez.z;

	M->m00 = det * (a22 * a33 - a23 * a23);
	M->m01 = det * (a13 * a23 - a12 * a33);
	M->m02 = det * (a12 * a23 - a13 * a22);

	M->m10 = M->m01;
	M->m11 = det * (a11 * a33 - a13 * a13);
	M->m12 = det * (a13 * a12 - a11 * a23);

	M->m20 = M->m02;
	M->m21 = M->m12;
	M->m22 = det * (a11 * a22 - a12 * a12);
}

}
