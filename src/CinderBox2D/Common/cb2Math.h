/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef B2_MATH_H
#define B2_MATH_H
#include <cinder/Vector.h>
#include <cinder/Matrix22.h>
#include <cinder/Matrix33.h>
#include <CinderBox2D/Common/cb2Settings.h>
#include <math.h>

namespace cb2
{
  
/// This function is used to ensure that a floating point number is
/// not a NaN or infinity.
template < typename T > inline bool isValid(const T& x)
{
	if (x != x)
	{
		// NaN.
		return false;
	}

	T infinity = std::numeric_limits<T>::infinity();
	return -infinity < x && x < infinity;
}

template < > inline bool isValid< ci::Vec2f >(const ci::Vec2f& v)
{
	return cb2::isValid(v.x) && cb2::isValid(v.y);
}

template < typename T > void setZero( T& data ) { data = T::zero(); }

template< typename T > inline T  getElement( const ci::Vec2<T> &v, int i ) { return (&v.x)[i];  }
template< typename T > inline T& getElement(       ci::Vec2<T> &v, int i ) { return (&v.x)[i];  }
template< typename T > inline T  getElement( const ci::Vec3<T> &v, int i ) { return (&v.x)[i];  }
template< typename T > inline T& getElement(       ci::Vec3<T> &v, int i ) { return (&v.x)[i];  }
template< typename T > inline T  getElement( const ci::Vec4<T> &v, int i ) { return (&v.x)[i];  }
template< typename T > inline T& getElement(       ci::Vec4<T> &v, int i ) { return (&v.x)[i];  }


template< typename T > inline ci::Vec2<T> skew( const ci::Vec2<T> &v ) { return ci::Vec2<T>( -v.y, v.x ); }

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
ci::Vec2f solve( const ci::Matrix22f& m, const ci::Vec2f& b);

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
ci::Vec3f solve( const ci::Matrix33f&A, const ci::Vec3f& b);

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases. Solve only the upper
/// 2-by-2 matrix equation.
ci::Vec2f solve22( const ci::Matrix33f&A, const ci::Vec2f& b);

void getInverse22( const ci::Matrix33f& A, ci::Matrix33f* M);

void getSymInverse33( const ci::Matrix33f& A, ci::Matrix33f* M );
}

#define	b2Sqrt(x)	sqrtf(x)
#define	b2Atan2(y, x)	atan2f(y, x)


/// Rotation
struct b2Rot
{
	b2Rot() {}

	/// Initialize from an angle in radians
	explicit b2Rot(float angle)
	{
		/// TODO_ERIN optimize
		s = sinf(angle);
		c = cosf(angle);
	}

	/// set using an angle in radians.
	void set(float angle)
	{
		/// TODO_ERIN optimize
		s = sinf(angle);
		c = cosf(angle);
	}

	/// set to the identity rotation
	void SetIdentity()
	{
		s = 0.0f;
		c = 1.0f;
	}

	/// Get the angle in radians
	float GetAngle() const
	{
		return b2Atan2(s, c);
	}

	/// Get the x-axis
	ci::Vec2f GetXAxis() const
	{
		return ci::Vec2f(c, s);
	}

	/// Get the u-axis
	ci::Vec2f GetYAxis() const
	{
		return ci::Vec2f(-s, c);
	}

	/// Sine and cosine
	float s, c;
};

/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
struct b2Transform
{
	/// The default constructor does nothing.
	b2Transform() {}

	/// Initialize using a position vector and a rotation.
	b2Transform(const ci::Vec2f& position, const b2Rot& rotation) : p(position), q(rotation) {}

	/// set this to the identity transform.
	void SetIdentity()
	{
		cb2::setZero(p);
		q.SetIdentity();
	}

	/// set this based on the position and angle.
	void set(const ci::Vec2f& position, float angle)
	{
		p = position;
		q.set(angle);
	}

	ci::Vec2f p;
	b2Rot q;
};

/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
struct b2Sweep
{
	/// Get the interpolated transform at a specific time.
	/// @param beta is a factor in [0,1], where 0 indicates alpha0.
	void GetTransform(b2Transform* xfb, float beta) const;

	/// Advance the sweep forward, yielding a new initial state.
	/// @param alpha the new initial time.
	void Advance(float alpha);

	/// normalize the angles.
	void normalize();

	ci::Vec2f localCenter;	///< local center of mass position
	ci::Vec2f c0, c;		///< center world positions
	float a0, a;		///< world angles

	/// Fraction of the current time step in the range [0,1]
	/// c0 and a0 are the positions at alpha0.
	float alpha0;
};

/// Perform the dot product on two vectors.
inline float b2Dot(const ci::Vec2f& a, const ci::Vec2f& b)
{
	return a.x * b.x + a.y * b.y;
}

/// Perform the cross product on two vectors. In 2D this produces a scalar.
inline float b2Cross(const ci::Vec2f& a, const ci::Vec2f& b)
{
	return a.x * b.y - a.y * b.x;
}

/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
inline ci::Vec2f b2Cross(const ci::Vec2f& a, float s)
{
	return ci::Vec2f(s * a.y, -s * a.x);
}

/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
inline ci::Vec2f b2Cross(float s, const ci::Vec2f& a)
{
	return ci::Vec2f(-s * a.y, s * a.x);
}

/// Multiply a matrix times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another.
inline ci::Vec2f b2Mul(const ci::Matrix22f& A, const ci::Vec2f& v)
{
	return ci::Vec2f(A.m00 * v.x + A.m01 * v.y, A.m10 * v.x + A.m11 * v.y);
}

/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another (inverse transform).
inline ci::Vec2f b2MulT(const ci::Matrix22f& A, const ci::Vec2f& v)
{
  return A.transposed() * v;
}

inline float b2Distance(const ci::Vec2f& a, const ci::Vec2f& b)
{
	ci::Vec2f c = a - b;
	return c.length();
}

inline float b2DistanceSquared(const ci::Vec2f& a, const ci::Vec2f& b)
{
	ci::Vec2f c = a - b;
	return b2Dot(c, c);
}

/// Perform the dot product on two vectors.
inline float b2Dot(const ci::Vec3f& a, const ci::Vec3f& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// Perform the cross product on two vectors.
inline ci::Vec3f b2Cross(const ci::Vec3f& a, const ci::Vec3f& b)
{
  return a.cross( b );
}

// A * B
inline ci::Matrix22f b2Mul(const ci::Matrix22f& A, const ci::Matrix22f& B)
{
	return A * B;
}

// A^T * B
inline ci::Matrix22f b2MulT(const ci::Matrix22f& A, const ci::Matrix22f& B)
{
  return A.transposed() * B;
}

/// Multiply a matrix times a vector.
inline ci::Vec3f b2Mul(const ci::Matrix33f& A, const ci::Vec3f& v)
{
  return A * v;
}

/// Multiply a matrix times a vector.
inline ci::Vec2f b2Mul22(const ci::Matrix33f& A, const ci::Vec2f& v)
{
	return ci::Vec2f(A.m00 * v.x + A.m10 * v.y, A.m01 * v.x + A.m11 * v.y);
}

/// Multiply two rotations: q * r
inline b2Rot b2Mul(const b2Rot& q, const b2Rot& r)
{
	// [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
	// [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
	// s = qs * rc + qc * rs
	// c = qc * rc - qs * rs
	b2Rot qr;
	qr.s = q.s * r.c + q.c * r.s;
	qr.c = q.c * r.c - q.s * r.s;
	return qr;
}

/// Transpose multiply two rotations: qT * r
inline b2Rot b2MulT(const b2Rot& q, const b2Rot& r)
{
	// [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
	// [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
	// s = qc * rs - qs * rc
	// c = qc * rc + qs * rs
	b2Rot qr;
	qr.s = q.c * r.s - q.s * r.c;
	qr.c = q.c * r.c + q.s * r.s;
	return qr;
}

/// Rotate a vector
inline ci::Vec2f b2Mul(const b2Rot& q, const ci::Vec2f& v)
{
	return ci::Vec2f(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y);
}

/// Inverse rotate a vector
inline ci::Vec2f b2MulT(const b2Rot& q, const ci::Vec2f& v)
{
	return ci::Vec2f(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y);
}

inline ci::Vec2f b2Mul(const b2Transform& T, const ci::Vec2f& v)
{
	float x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
	float y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;

	return ci::Vec2f(x, y);
}

inline ci::Vec2f b2MulT(const b2Transform& T, const ci::Vec2f& v)
{
	float px = v.x - T.p.x;
	float py = v.y - T.p.y;
	float x = (T.q.c * px + T.q.s * py);
	float y = (-T.q.s * px + T.q.c * py);

	return ci::Vec2f(x, y);
}

// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
inline b2Transform b2Mul(const b2Transform& A, const b2Transform& B)
{
	b2Transform C;
	C.q = b2Mul(A.q, B.q);
	C.p = b2Mul(A.q, B.p) + A.p;
	return C;
}

// v2 = A.q' * (B.q * v1 + B.p - A.p)
//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
inline b2Transform b2MulT(const b2Transform& A, const b2Transform& B)
{
	b2Transform C;
	C.q = b2MulT(A.q, B.q);
	C.p = b2MulT(A.q, B.p - A.p);
	return C;
}

template <typename T>
inline T b2Abs(T a)
{
	return a > T(0) ? a : -a;
}

inline ci::Vec2f b2Abs(const ci::Vec2f& a)
{
	return ci::Vec2f(b2Abs(a.x), b2Abs(a.y));
}

inline ci::Matrix22f b2Abs(const ci::Matrix22f& A)
{
	return ci::Matrix22f(b2Abs(A.getColumn(0)), b2Abs(A.getColumn(1)));
}

template <typename T>
inline T b2Min(T a, T b)
{
	return a < b ? a : b;
}

inline ci::Vec2f b2Min(const ci::Vec2f& a, const ci::Vec2f& b)
{
	return ci::Vec2f(b2Min(a.x, b.x), b2Min(a.y, b.y));
}

template <typename T>
inline T b2Max(T a, T b)
{
	return a > b ? a : b;
}

inline ci::Vec2f b2Max(const ci::Vec2f& a, const ci::Vec2f& b)
{
	return ci::Vec2f(b2Max(a.x, b.x), b2Max(a.y, b.y));
}

template <typename T>
inline T b2Clamp(T a, T low, T high)
{
	return b2Max(low, b2Min(a, high));
}

inline ci::Vec2f b2Clamp(const ci::Vec2f& a, const ci::Vec2f& low, const ci::Vec2f& high)
{
	return b2Max(low, b2Min(a, high));
}

template<typename T> inline void b2Swap(T& a, T& b)
{
	T tmp = a;
	a = b;
	b = tmp;
}

/// "Next Largest Power of 2
/// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
/// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
/// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
/// largest power of 2. For a 32-bit value:"
inline unsigned int b2NextPowerOfTwo(unsigned int x)
{
	x |= (x >> 1);
	x |= (x >> 2);
	x |= (x >> 4);
	x |= (x >> 8);
	x |= (x >> 16);
	return x + 1;
}

inline bool b2IsPowerOfTwo(unsigned int x)
{
	bool result = x > 0 && (x & (x - 1)) == 0;
	return result;
}

inline void b2Sweep::GetTransform(b2Transform* xf, float beta) const
{
	xf->p = (1.0f - beta) * c0 + beta * c;
	float angle = (1.0f - beta) * a0 + beta * a;
	xf->q.set(angle);

	// Shift to origin
	xf->p -= b2Mul(xf->q, localCenter);
}

inline void b2Sweep::Advance(float alpha)
{
	b2Assert(alpha0 < 1.0f);
	float beta = (alpha - alpha0) / (1.0f - alpha0);
	c0 += beta * (c - c0);
	a0 += beta * (a - a0);
	alpha0 = alpha;
}

/// normalize an angle in radians to be between -pi and pi
inline void b2Sweep::normalize()
{
	float twoPi = 2.0f * b2_pi;
	float d =  twoPi * floorf(a0 / twoPi);
	a0 -= d;
	a -= d;
}

#endif
