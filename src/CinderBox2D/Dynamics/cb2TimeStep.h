/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef CB2_TIME_STEP_H
#define CB2_TIME_STEP_H

#include <CinderBox2D/Common/cb2Math.h>

/// Profiling data. Times are in milliseconds.
struct cb2Profile
{
	float step;
	float collide;
	float solve;
	float solveInit;
	float solveVelocity;
	float solvePosition;
	float broadphase;
	float solveTOI;
};

/// This is an internal structure.
struct cb2TimeStep
{
	float dt;			// time step
	float inv_dt;		// inverse time step (0 if dt == 0).
	float dtRatio;	// dt * inv_dt0
	int velocityIterations;
	int positionIterations;
	bool warmStarting;
};

/// This is an internal structure.
struct cb2Position
{
	ci::Vec2f c;
	float a;
};

/// This is an internal structure.
struct cb2Velocity
{
	ci::Vec2f v;
	float w;
};

/// Solver Data
struct cb2SolverData
{
	cb2TimeStep step;
	cb2Position* positions;
	cb2Velocity* velocities;
};

#endif
