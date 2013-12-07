/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include <CinderBox2D/Dynamics/Joints/cb2Joint.h>
#include <CinderBox2D/Dynamics/Joints/cb2DistanceJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2WheelJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2MouseJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2RevoluteJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2PrismaticJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2PulleyJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2GearJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2WeldJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2FrictionJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2RopeJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2MotorJoint.h>
#include <CinderBox2D/Dynamics/cb2Body.h>
#include <CinderBox2D/Dynamics/cb2World.h>
#include <CinderBox2D/Common/cb2BlockAllocator.h>

#include <new>

cb2Joint* cb2Joint::Create(const cb2JointDef* def, cb2BlockAllocator* allocator)
{
	cb2Joint* joint = NULL;

	switch (def->type)
	{
	case e_distanceJoint:
		{
			void* mem = allocator->Allocate(sizeof(cb2DistanceJoint));
			joint = new (mem) cb2DistanceJoint(static_cast<const cb2DistanceJointDef*>(def));
		}
		break;

	case e_mouseJoint:
		{
			void* mem = allocator->Allocate(sizeof(cb2MouseJoint));
			joint = new (mem) cb2MouseJoint(static_cast<const cb2MouseJointDef*>(def));
		}
		break;

	case e_prismaticJoint:
		{
			void* mem = allocator->Allocate(sizeof(cb2PrismaticJoint));
			joint = new (mem) cb2PrismaticJoint(static_cast<const cb2PrismaticJointDef*>(def));
		}
		break;

	case e_revoluteJoint:
		{
			void* mem = allocator->Allocate(sizeof(cb2RevoluteJoint));
			joint = new (mem) cb2RevoluteJoint(static_cast<const cb2RevoluteJointDef*>(def));
		}
		break;

	case e_pulleyJoint:
		{
			void* mem = allocator->Allocate(sizeof(cb2PulleyJoint));
			joint = new (mem) cb2PulleyJoint(static_cast<const cb2PulleyJointDef*>(def));
		}
		break;

	case e_gearJoint:
		{
			void* mem = allocator->Allocate(sizeof(cb2GearJoint));
			joint = new (mem) cb2GearJoint(static_cast<const cb2GearJointDef*>(def));
		}
		break;

	case e_wheelJoint:
		{
			void* mem = allocator->Allocate(sizeof(cb2WheelJoint));
			joint = new (mem) cb2WheelJoint(static_cast<const cb2WheelJointDef*>(def));
		}
		break;

	case e_weldJoint:
		{
			void* mem = allocator->Allocate(sizeof(cb2WeldJoint));
			joint = new (mem) cb2WeldJoint(static_cast<const cb2WeldJointDef*>(def));
		}
		break;
        
	case e_frictionJoint:
		{
			void* mem = allocator->Allocate(sizeof(cb2FrictionJoint));
			joint = new (mem) cb2FrictionJoint(static_cast<const cb2FrictionJointDef*>(def));
		}
		break;

	case e_ropeJoint:
		{
			void* mem = allocator->Allocate(sizeof(cb2RopeJoint));
			joint = new (mem) cb2RopeJoint(static_cast<const cb2RopeJointDef*>(def));
		}
		break;

	case e_motorJoint:
		{
			void* mem = allocator->Allocate(sizeof(cb2MotorJoint));
			joint = new (mem) cb2MotorJoint(static_cast<const cb2MotorJointDef*>(def));
		}
		break;

	default:
		cb2Assert(false);
		break;
	}

	return joint;
}

void cb2Joint::Destroy(cb2Joint* joint, cb2BlockAllocator* allocator)
{
	joint->~cb2Joint();
	switch (joint->m_type)
	{
	case e_distanceJoint:
		allocator->Free(joint, sizeof(cb2DistanceJoint));
		break;

	case e_mouseJoint:
		allocator->Free(joint, sizeof(cb2MouseJoint));
		break;

	case e_prismaticJoint:
		allocator->Free(joint, sizeof(cb2PrismaticJoint));
		break;

	case e_revoluteJoint:
		allocator->Free(joint, sizeof(cb2RevoluteJoint));
		break;

	case e_pulleyJoint:
		allocator->Free(joint, sizeof(cb2PulleyJoint));
		break;

	case e_gearJoint:
		allocator->Free(joint, sizeof(cb2GearJoint));
		break;

	case e_wheelJoint:
		allocator->Free(joint, sizeof(cb2WheelJoint));
		break;
    
	case e_weldJoint:
		allocator->Free(joint, sizeof(cb2WeldJoint));
		break;

	case e_frictionJoint:
		allocator->Free(joint, sizeof(cb2FrictionJoint));
		break;

	case e_ropeJoint:
		allocator->Free(joint, sizeof(cb2RopeJoint));
		break;

	case e_motorJoint:
		allocator->Free(joint, sizeof(cb2MotorJoint));
		break;

	default:
		cb2Assert(false);
		break;
	}
}

cb2Joint::cb2Joint(const cb2JointDef* def)
{
	cb2Assert(def->bodyA != def->bodyB);

	m_type = def->type;
	m_prev = NULL;
	m_next = NULL;
	m_bodyA = def->bodyA;
	m_bodyB = def->bodyB;
	m_index = 0;
	m_collideConnected = def->collideConnected;
	m_islandFlag = false;
	m_userData = def->userData;

	m_edgeA.joint = NULL;
	m_edgeA.other = NULL;
	m_edgeA.prev = NULL;
	m_edgeA.next = NULL;

	m_edgeB.joint = NULL;
	m_edgeB.other = NULL;
	m_edgeB.prev = NULL;
	m_edgeB.next = NULL;
}

bool cb2Joint::IsActive() const
{
	return m_bodyA->IsActive() && m_bodyB->IsActive();
}
