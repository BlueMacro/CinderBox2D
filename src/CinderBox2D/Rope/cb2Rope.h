/*
* Copyright (c) 2011 Erin Catto http://www.box2d.org
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

#ifndef CB2_ROPE_H
#define CB2_ROPE_H

#include <CinderBox2D/Common/cb2Math.h>

class cb2Draw;

/// 
struct cb2RopeDef
{
	cb2RopeDef()
	{
		vertices = NULL;
		count = 0;
		masses = NULL;
		cb2::setZero(gravity);
		damping = 0.1f;
		k2 = 0.9f;
		k3 = 0.1f;
	}

	///
	ci::Vec2f* vertices;

	///
	int count;

	///
	float* masses;

	///
	ci::Vec2f gravity;

	///
	float damping;

	/// Stretching stiffness
	float k2;

	/// Bending stiffness. Values above 0.5 can make the simulation blow up.
	float k3;
};

/// 
class cb2Rope
{
public:
	cb2Rope();
	~cb2Rope();

	///
	void Initialize(const cb2RopeDef* def);

	///
	void Step(float timeStep, int iterations);

	///
	int GetVertexCount() const
	{
		return m_count;
	}

	///
	const ci::Vec2f* GetVertices() const
	{
		return m_ps;
	}

	///
	void Draw(cb2Draw* draw) const;

	///
	void SetAngle(float angle);

private:

	void SolveC2();
	void SolveC3();

	int m_count;
	ci::Vec2f* m_ps;
	ci::Vec2f* m_p0s;
	ci::Vec2f* m_vs;

	float* m_ims;

	float* m_Ls;
	float* m_as;

	ci::Vec2f m_gravity;
	float m_damping;

	float m_k2;
	float m_k3;
};

#endif
