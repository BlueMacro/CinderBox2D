/*
* Copyright (c) 2011 Erin Catto http://box2d.org
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

#include <CinderBox2D/Rope/cb2Rope.h>
#include <CinderBox2D/Common/cb2Draw.h>

cb2Rope::cb2Rope()
{
	m_count = 0;
	m_ps = NULL;
	m_p0s = NULL;
	m_vs = NULL;
	m_ims = NULL;
	m_Ls = NULL;
	m_as = NULL;
	m_k2 = 1.0f;
	m_k3 = 0.1f;
}

cb2Rope::~cb2Rope()
{
	cb2Free(m_ps);
	cb2Free(m_p0s);
	cb2Free(m_vs);
	cb2Free(m_ims);
	cb2Free(m_Ls);
	cb2Free(m_as);
}

void cb2Rope::Initialize(const cb2RopeDef* def)
{
	cb2Assert(def->count >= 3);
	m_count = def->count;
	m_ps = (ci::Vec2f*)cb2Alloc(m_count * sizeof(ci::Vec2f));
	m_p0s = (ci::Vec2f*)cb2Alloc(m_count * sizeof(ci::Vec2f));
	m_vs = (ci::Vec2f*)cb2Alloc(m_count * sizeof(ci::Vec2f));
	m_ims = (float*)cb2Alloc(m_count * sizeof(float));

	for (int i = 0; i < m_count; ++i)
	{
		m_ps[i] = def->vertices[i];
		m_p0s[i] = def->vertices[i];
		cb2::setZero(m_vs[i]);

		float m = def->masses[i];
		if (m > 0.0f)
		{
			m_ims[i] = 1.0f / m;
		}
		else
		{
			m_ims[i] = 0.0f;
		}
	}

	int count2 = m_count - 1;
	int count3 = m_count - 2;
	m_Ls = (float*)cb2Alloc(count2 * sizeof(float));
	m_as = (float*)cb2Alloc(count3 * sizeof(float));

	for (int i = 0; i < count2; ++i)
	{
		ci::Vec2f p1 = m_ps[i];
		ci::Vec2f p2 = m_ps[i+1];
		m_Ls[i] = cb2Distance(p1, p2);
	}

	for (int i = 0; i < count3; ++i)
	{
		ci::Vec2f p1 = m_ps[i];
		ci::Vec2f p2 = m_ps[i + 1];
		ci::Vec2f p3 = m_ps[i + 2];

		ci::Vec2f d1 = p2 - p1;
		ci::Vec2f d2 = p3 - p2;

		float a = cb2Cross(d1, d2);
		float b = cb2Dot(d1, d2);

		m_as[i] = cb2Atan2(a, b);
	}

	m_gravity = def->gravity;
	m_damping = def->damping;
	m_k2 = def->k2;
	m_k3 = def->k3;
}

void cb2Rope::Step(float h, int iterations)
{
	if (h == 0.0)
	{
		return;
	}

	float d = expf(- h * m_damping);

	for (int i = 0; i < m_count; ++i)
	{
		m_p0s[i] = m_ps[i];
		if (m_ims[i] > 0.0f)
		{
			m_vs[i] += h * m_gravity;
		}
		m_vs[i] *= d;
		m_ps[i] += h * m_vs[i];

	}

	for (int i = 0; i < iterations; ++i)
	{
		SolveC2();
		SolveC3();
		SolveC2();
	}

	float inv_h = 1.0f / h;
	for (int i = 0; i < m_count; ++i)
	{
		m_vs[i] = inv_h * (m_ps[i] - m_p0s[i]);
	}
}

void cb2Rope::SolveC2()
{
	int count2 = m_count - 1;

	for (int i = 0; i < count2; ++i)
	{
		ci::Vec2f p1 = m_ps[i];
		ci::Vec2f p2 = m_ps[i + 1];

		ci::Vec2f d = p2 - p1;
		float L = d.length();
		d /= L;

		float im1 = m_ims[i];
		float im2 = m_ims[i + 1];

		if (im1 + im2 == 0.0f)
		{
			continue;
		}

		float s1 = im1 / (im1 + im2);
		float s2 = im2 / (im1 + im2);

		p1 -= m_k2 * s1 * (m_Ls[i] - L) * d;
		p2 += m_k2 * s2 * (m_Ls[i] - L) * d;

		m_ps[i] = p1;
		m_ps[i + 1] = p2;
	}
}

void cb2Rope::SetAngle(float angle)
{
	int count3 = m_count - 2;
	for (int i = 0; i < count3; ++i)
	{
		m_as[i] = angle;
	}
}

void cb2Rope::SolveC3()
{
	int count3 = m_count - 2;

	for (int i = 0; i < count3; ++i)
	{
		ci::Vec2f p1 = m_ps[i];
		ci::Vec2f p2 = m_ps[i + 1];
		ci::Vec2f p3 = m_ps[i + 2];

		float m1 = m_ims[i];
		float m2 = m_ims[i + 1];
		float m3 = m_ims[i + 2];

		ci::Vec2f d1 = p2 - p1;
		ci::Vec2f d2 = p3 - p2;

		float L1sqr = d1.lengthSquared();
		float L2sqr = d2.lengthSquared();

		if (L1sqr * L2sqr == 0.0f)
		{
			continue;
		}

		float a = cb2Cross(d1, d2);
		float b = cb2Dot(d1, d2);

		float angle = cb2Atan2(a, b);

		ci::Vec2f Jd1 = (-1.0f / L1sqr) * cb2::skew(d1);
		ci::Vec2f Jd2 = (1.0f / L2sqr) * cb2::skew(d2);

		ci::Vec2f J1 = -Jd1;
		ci::Vec2f J2 = Jd1 - Jd2;
		ci::Vec2f J3 = Jd2;

		float mass = m1 * cb2Dot(J1, J1) + m2 * cb2Dot(J2, J2) + m3 * cb2Dot(J3, J3);
		if (mass == 0.0f)
		{
			continue;
		}

		mass = 1.0f / mass;

		float C = angle - m_as[i];

		while (C > cb2_pi)
		{
			angle -= 2 * cb2_pi;
			C = angle - m_as[i];
		}

		while (C < -cb2_pi)
		{
			angle += 2.0f * cb2_pi;
			C = angle - m_as[i];
		}

		float impulse = - m_k3 * mass * C;

		p1 += (m1 * impulse) * J1;
		p2 += (m2 * impulse) * J2;
		p3 += (m3 * impulse) * J3;

		m_ps[i] = p1;
		m_ps[i + 1] = p2;
		m_ps[i + 2] = p3;
	}
}

void cb2Rope::Draw(cb2Draw* draw) const
{
	cb2Color c(0.4f, 0.5f, 0.7f);

	for (int i = 0; i < m_count - 1; ++i)
	{
		draw->DrawSegment(m_ps[i], m_ps[i+1], c);
	}
}
