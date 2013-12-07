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

#ifndef CB2_TIME_OF_IMPACT_H
#define CB2_TIME_OF_IMPACT_H

#include <CinderBox2D/Common/cb2Math.h>
#include <CinderBox2D/Collision/cb2Distance.h>

/// Input parameters for cb2TimeOfImpact
struct cb2TOIInput
{
	cb2DistanceProxy proxyA;
	cb2DistanceProxy proxyB;
	cb2Sweep sweepA;
	cb2Sweep sweepB;
	float tMax;		// defines sweep interval [0, tMax]
};

// Output parameters for cb2TimeOfImpact.
struct cb2TOIOutput
{
	enum State
	{
		e_unknown,
		e_failed,
		e_overlapped,
		e_touching,
		e_separated
	};

	State state;
	float t;
};

/// Compute the upper bound on time before two shapes penetrate. Time is represented as
/// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
/// non-tunneling collision. If you change the time interval, you should call this function
/// again.
/// Note: use cb2Distance to compute the contact point and normal at the time of impact.
void cb2TimeOfImpact(cb2TOIOutput* output, const cb2TOIInput* input);

#endif
