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

#ifndef BOX2D_H
#define BOX2D_H

/**
\mainpage Box2D API Documentation

\section intro_sec Getting Started

For documentation please see http://box2d.org/documentation.html

For discussion please visit http://box2d.org/forum
*/

// These include files constitute the main Box2D API

#include <CinderBox2D/Common/cb2Settings.h>
#include <CinderBox2D/Common/cb2Draw.h>
#include <CinderBox2D/Common/cb2Timer.h>

#include <CinderBox2D/Collision/Shapes/cb2CircleShape.h>
#include <CinderBox2D/Collision/Shapes/cb2EdgeShape.h>
#include <CinderBox2D/Collision/Shapes/cb2ChainShape.h>
#include <CinderBox2D/Collision/Shapes/cb2PolygonShape.h>

#include <CinderBox2D/Collision/cb2BroadPhase.h>
#include <CinderBox2D/Collision/cb2Distance.h>
#include <CinderBox2D/Collision/cb2DynamicTree.h>
#include <CinderBox2D/Collision/cb2TimeOfImpact.h>

#include <CinderBox2D/Dynamics/cb2Body.h>
#include <CinderBox2D/Dynamics/cb2Fixture.h>
#include <CinderBox2D/Dynamics/cb2WorldCallbacks.h>
#include <CinderBox2D/Dynamics/cb2TimeStep.h>
#include <CinderBox2D/Dynamics/cb2World.h>

#include <CinderBox2D/Dynamics/Contacts/cb2Contact.h>

#include <CinderBox2D/Dynamics/Joints/cb2DistanceJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2FrictionJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2GearJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2MotorJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2MouseJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2PrismaticJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2PulleyJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2RevoluteJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2RopeJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2WeldJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2WheelJoint.h>

#endif
