/*
Bullet Continuous Collision Detection and Physics Library
Ragdoll Demo
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#define CONSTRAINT_DEBUG_SIZE 0.2f


#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"
//#include "mex.h"

// added for CTRNN
#include <iomanip>

// old added
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <time.h>
using namespace std;
//end added

#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "PDSTEP_demo.h"


// Enrico: Shouldn't these three variables be real constants and not defines?

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

#define HIP_AP_L -(M_PI_4)*0.5
#define HIP_AP_H (M_PI_4)*2.5
#define HIP_ML_L -(M_PI_4)*0.67
#define HIP_ML_H (M_PI_4)*0.67
#define ANKL_AP_L -(M_PI_4)*0.3
#define ANKL_AP_H (M_PI_4)*1.3
#define ANKL_ML_L -(M_PI_4)*0.62
#define ANKL_ML_H (M_PI_4)*0.62
#define TP_AP_L -(M_PI_4)*0.3
#define TP_AP_H (M_PI_4)*1.3
#define TP_ML_L -(M_PI_4)*0.62
#define TP_ML_H (M_PI_4)*0.62

class RagDoll
{
#ifdef MALE
	double avBH = 181.0;
	double avBM = 78.4;
	// body segments are calculated by the following formula
	// y = b0 + b1 x BM + b2 x BH;

	//MASSES:
	double mass_head = -7.75 + 0.0586*avBM + 0.0497*avBH;
	double mass_torso = 7.57 + 0.295*avBM - 0.0385*avBH; // upper + middle torso
	double mass_pelvis = 13.1 + 0.162*avBM - 0.0873*avBH;
	double mass_thigh = 1.18 + 0.182*avBM - 0.0259*avBH;
	double mass_shank = -3.53 + 0.0306*avBM + 0.0268*avBH;
	double mass_leg = -2.35 + 0.2126*avBM + 0.0009*avBH; // thigh + shank
	double mass_foot = -2.25 + 0.0010*avBM + 0.0182*avBH;
	double mass_UA = -0.896 + 0.0252*avBM + 0.0051*avBH; // upper arm separately
	double mass_FA = -0.731 + 0.0047*avBM + 0.0084*avBH;  // forearm separately
	double mass_arm = -1.627 + 0.0299*avBM + 0.0135*avBH; // UA+FA
	double mass_hand = -0.325 - 0.0016*avBM + 0.0051*avBH; 

	// HEIGHTS:
	double height_head = 1.95 + 0.0535*avBM + 0.105*avBH;
	double height_torso = -32.11 - 0.095*avBM + 0.462*avBH; // upper + middle torso
	double height_pelvis = 26.4 + 0.0473*avBM - 0.0311*avBH;
	double height_thigh = 4.26 - 0.0183*avBM + 0.24*avBH;
	double height_shank = -16.0 + 0.0218*avBM + 0.321*avBH;
	double height_leg = -11.74 + 0.0035*avBM + 0.561*avBH; // thigh + shank
	double length_foot = 3.8 + 0.013*avBM + 0.119*avBH;
	double height_UA = -15.0 + 0.012*avBM + 0.229*avBH; // upper arm separately
	double height_FA = 0.143 - 0.0281*avBM + 0.161*avBH;  // forearm separately
	double height_arm = -14.857 - 0.0161*avBM + 0.39*avBH; // UA+FA
	double height_hand = -3.7 + 0.0036*avBM + 0.131*avBH;

#else //Female:
	double avBH = 169.0;
	double avBM = 75.4;

	//MASSES:
	double mass_head = -2.95 + 0.0359*avBM + 0.0322*avBH;
	double mass_torso = 24.05 + 0.3255*avBM - 0.1424*avBH; // upper + middle torso
	double mass_pelvis = 1.1 + 0.104*avBM - 0.0027*avBH;
	double mass_thigh = -10.9 + 0.213*avBM + 0.038*avBH;
	double mass_shank = -0.563 + 0.0191*avBM + 0.0141*avBH;
	double mass_leg = mass_thigh + mass_shank; // thigh + shank
	double mass_foot = -1.27 + 0.0045*avBM + 0.0104*avBH;
	double mass_UA = 3.05 + 0.0184*avBM - 0.0164*avBH; // upper arm separately
	double mass_FA = -0.481 + 0.0087*avBM + 0.0043*avBH;  // forearm separately
	double mass_arm = mass_UA + mass_FA; // UA+FA
	double mass_hand = -1.13 + 0.0031*avBM + 0.0074*avBH;

	// HEIGHTS:
	double height_head = -8.95 - 0.0057*avBM + 0.202*avBH;
	double height_torso = 10.48 + 0.1291*avBM + 0.147*avBH; // upper + middle torso
	double height_pelvis = 21.4 + 0.0146*avBM - 0.005*avBH;
	double height_thigh = -26.8 - 0.0725*avBM + 0.436*avBH;
	double height_shank = -7.21 - 0.0618*avBM + 0.308*avBH;
	double height_leg = height_thigh + height_shank; // thigh + shank
	double length_foot = 7.39 + 0.0311*avBM + 0.0867*avBH;
	double height_UA = 2.44 - 0.0169*avBM + 0.146*avBH; // upper arm separately
	double height_FA = -8.57 + 0.0494*avBM + 0.18*avBH;  // forearm separately
	double height_arm = height_FA + height_UA; // UA+FA
	double height_hand = -8.96 + 0.0057*avBM + 0.163*avBH;
#endif

#define PELV_HEIGHT 0.3 + length_foot / 60 + height_leg / 30

	// for the case of torso
#ifdef TORSO
		enum
		{
			BODYPART_ABDOMEN = 0, //1
			BODYPART_PELVIS, //2
			BODYPART_LEFT_LEG, //3
			BODYPART_RIGHT_LEG, //4
			BODYPART_LEFT_FOOT, //5
			BODYPART_RIGHT_FOOT, //6
			BODYPART_PLATFORM, //7

			BODYPART_COUNT
		};
#else
	enum
	{
		
		BODYPART_PELVIS = 0, //1 
		BODYPART_LEFT_LEG, //2
		BODYPART_RIGHT_LEG, //3
		BODYPART_LEFT_FOOT, //4
		BODYPART_RIGHT_FOOT, //5
		BODYPART_PLATFORM, //6

		BODYPART_COUNT
	};
#endif

	// in the case of torso - one more joint
#ifdef TORSO
		enum
		{
			JOINT_BODY_PELVIS = 0, //1
			JOINT_LEFT_HIP, //2
			JOINT_RIGHT_HIP, //3
			JOINT_LEFT_ANKLE, //4
			JOINT_RIGHT_ANKLE, //5

			JOINT_COUNT
		};

#else
	enum
	{
		
		JOINT_LEFT_HIP = 0, //1	
		JOINT_RIGHT_HIP, //2
		JOINT_LEFT_ANKLE, //3
		JOINT_RIGHT_ANKLE, //4

		JOINT_COUNT
	};
#endif
	enum legOrient
	{
		X_ORIENT,
		Y_ORIENT,
		Z_ORIENT
	} orient;

	enum jointOrient
	{
		X_ORIENT_P, //positive orientations
		Y_ORIENT_P,
		Z_ORIENT_P,
		X_ORIENT_N, //negative orientations
		Y_ORIENT_N,
		Z_ORIENT_N
	} j_orient;

	int * m_ids;

	btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
	{
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			shape->calculateLocalInertia(mass,localInertia);

		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		m_ownerWorld->addRigidBody(body);

		return body;
	}

public:

	// USED TO BE PRIVATE:
	btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shapes[BODYPART_COUNT];
	btRigidBody* m_bodies[BODYPART_COUNT];
	// btTypedConstraint* m_joints[JOINT_COUNT]; // replaced by line below
	btGeneric6DofConstraint* m_joints[JOINT_COUNT];
	// END USED TO BE PRIVATE

	RagDoll (btDynamicsWorld* ownerWorld, const btVector3& positionOffset, int* IDs)
		: m_ownerWorld(ownerWorld), m_ids(IDs)
	{
		//CREATE BOXES:
		//First three numbers-COM location, second 3 numbers - half-measures (since the origin is inside the body's center)
		//of the body itself

		//in case of the torso
#ifdef TORSO
		CreateBox(BODYPART_PLATFORM, 0, 0.15, 0, 6., 4., 0.15, 200.);
		// mixed up dimensions: width, height, length, now corrected to that described: length, width, height
		// width - mediolateral dir, length into the screen

		//all heights are scaled down by 30, to be comparable with the previous robot. Also units can be thought of as feet
		// since 1 foot = 30.4878 cm. Values are parsed divided by 60 because functions take in half-measures. 
		//CreateBox(BODYPART_ABDOMEN, 0., 4.5, 0., 0.5, 0.75, 0.5, 1.);
		//CreateBox(BODYPART_PELVIS, 0., 3.5, 0., 0.5, 0.5, 0.5, 1.);
		//CreateBox(BODYPART_LEFT_FOOT, 0.5, 0.45, 0., 0.35, 0.15, 0.5, 1.);
		//CreateBox(BODYPART_RIGHT_FOOT, -0.5, 0.45, 0., 0.35, 0.15, 0.5, 1.);
		CreateBox(BODYPART_ABDOMEN, 0., 0.3+length_foot/60+height_leg/30+height_pelvis/60+height_torso/60, 0., height_pelvis/60, height_pelvis/60, height_torso/60, mass_torso);
		CreateBox(BODYPART_PELVIS, 0., 0.3+length_foot/60+height_leg/30, 0., height_pelvis/60, height_pelvis/60, height_pelvis/60, mass_pelvis);
		CreateBox(BODYPART_LEFT_FOOT, height_pelvis/60, 0.3+length_foot/120, 0., length_foot/ 60, length_foot/90, length_foot/120, mass_foot);
		CreateBox(BODYPART_RIGHT_FOOT, -height_pelvis/60, 0.3+length_foot/120, 0., length_foot/60, length_foot/90, length_foot/120, mass_foot);
#else		
		CreateBox(BODYPART_PLATFORM, 0, 0.15, 0, 6., 4., 0.15, 200.);
		CreateBox(BODYPART_PELVIS, 0., 0.3+length_foot/60+height_leg/30, 0., height_pelvis/60, height_pelvis/60, height_pelvis/60, mass_pelvis);
		CreateBox(BODYPART_LEFT_FOOT, height_pelvis/60, 0.3+length_foot/120, 0., length_foot/60, length_foot/90, length_foot/120, mass_foot);
		CreateBox(BODYPART_RIGHT_FOOT, -height_pelvis/60, 0.3+length_foot/120, 0., length_foot/60, length_foot/90, length_foot/120, mass_foot);
#endif		

		//FRICTION CTRL:
		//m_bodies[4]->setFriction(1.3);//Bodies 3,4 are left and right feet respectively, default friction is 0.5. The max friction is 10. 
		//m_bodies[5]->setFriction(1.3);// W/o abdomen it should be 3,4,5 top down.
		//m_bodies[6]->setFriction(1.3);

		//CREATE LEGS:
		//CreateCylinder(BODYPART_LEFT_LEG, Y_ORIENT, 0.5, 2.1, 0., 1.5, 0.15, 1., 1.);
		//CreateCylinder(BODYPART_RIGHT_LEG, Y_ORIENT, -0.5, 2.1, 0., 1.5, 0.15, 1., 1.);
		CreateCylinder(BODYPART_LEFT_LEG, Y_ORIENT, height_pelvis/60, 0.3+length_foot/60+height_leg/60, 0., height_leg/60, 0.15, 1., mass_leg);
		CreateCylinder(BODYPART_RIGHT_LEG, Y_ORIENT, -height_pelvis/60, 0.3+length_foot/60+height_leg/60, 0., height_leg/60, 0.15, 1., mass_leg);

		//CREATE JOINTS:
		//vectors in argument are the joint location in local body part's coordinate system

#ifdef TORSO
		////Flipped the boundary values on 25.02.2014, used to be 2.5 and 0.5 for AL, ML is the same - 0.67 and 0.67. Flipped again on 3.03.2014, researching the bug where the limits on the targetAngle produced a "jumping" bug. 
		//Create6DOF(JOINT_LEFT_HIP, BODYPART_PELVIS, BODYPART_LEFT_LEG, btVector3(0.5, 0., 0.), btVector3(0., 1.4, 0.), M_PI_2, 0, M_PI_2, -(M_PI_4)*2.5, (M_PI_4)*0.5, -(M_PI_4)*0.67, (M_PI_4)*0.67);
		//Create6DOF(JOINT_RIGHT_HIP, BODYPART_PELVIS, BODYPART_RIGHT_LEG, btVector3(-0.5, 0., 0.), btVector3(0., 1.4, 0.), M_PI_2, 0, M_PI_2, -(M_PI_4)*2.5, (M_PI_4)*0.5, -(M_PI_4)*0.67, (M_PI_4)*0.67);
		////Flipped the boundary values on 25.02.2014, used to be 1.3 and 0.3 for AL. Flipped again on 3.03.2014, researching the bug where the limits on the targetAngle produced a "jumping" bug.  
		//Create6DOF(JOINT_LEFT_ANKLE, BODYPART_LEFT_LEG, BODYPART_LEFT_FOOT, btVector3(0., -1.5, 0.), btVector3(0., 0.15, 0.), M_PI_2, 0, M_PI_2, -(M_PI_4)*1.3, (M_PI_4)*0.3, -(M_PI_4)*0.62, (M_PI_4)*0.62);
		////28 degrees (0.62*(pi/4=45degrees)) for ML movement in the ankles 
		//Create6DOF(JOINT_RIGHT_ANKLE, BODYPART_RIGHT_LEG, BODYPART_RIGHT_FOOT, btVector3(0., -1.5, 0.), btVector3(0., 0.15, 0.), M_PI_2, 0, M_PI_2, -(M_PI_4)*1.3, (M_PI_4)*0.3, -(M_PI_4)*0.62, (M_PI_4)*0.62);
		//Create6DOF(JOINT_BODY_PELVIS, BODYPART_ABDOMEN, BODYPART_PELVIS, btVector3(0.25, -0.75, 0), btVector3(0.25, 0.5, 0), M_PI_2, 0, M_PI_2, -(M_PI_4)*1.3, (M_PI_4)*0.3, -(M_PI_4)*0.62, (M_PI_4)*0.62);

		//Flipped the boundary values on 25.02.2014, used to be 2.5 and 0.5 for AL, ML is the same - 0.67 and 0.67. Flipped again on 3.03.2014, researching the bug where the limits on the targetAngle produced a "jumping" bug. 
		Create6DOF(JOINT_LEFT_HIP, BODYPART_PELVIS, BODYPART_LEFT_LEG, btVector3(height_pelvis/60, 0., 0.), btVector3(0., height_leg/60, 0.), M_PI_2, 0, M_PI_2, HIP_AP_L,HIP_AP_H,HIP_ML_L,HIP_ML_H);
		Create6DOF(JOINT_RIGHT_HIP, BODYPART_PELVIS, BODYPART_RIGHT_LEG, btVector3(-height_pelvis/60, 0., 0.), btVector3(0., height_leg/60, 0.), M_PI_2, 0, M_PI_2, HIP_AP_L, HIP_AP_H, HIP_ML_L, HIP_ML_H);
		//Flipped the boundary values on 25.02.2014, used to be 1.3 and 0.3 for AL. Flipped again on 3.03.2014, researching the bug where the limits on the targetAngle produced a "jumping" bug.  
		Create6DOF(JOINT_LEFT_ANKLE, BODYPART_LEFT_LEG, BODYPART_LEFT_FOOT, btVector3(0.,-height_leg/60, 0.), btVector3(0.,length_foot/120, 0.), M_PI_2, 0, M_PI_2, ANKL_AP_L, ANKL_AP_H, ANKL_ML_L, ANKL_ML_H);
		//28 degrees (0.62*(pi/4=45degrees)) for ML movement in the ankles 
		Create6DOF(JOINT_RIGHT_ANKLE, BODYPART_RIGHT_LEG, BODYPART_RIGHT_FOOT, btVector3(0.,-height_leg/60, 0.), btVector3(0.,length_foot/120, 0.), M_PI_2, 0, M_PI_2, ANKL_AP_L, ANKL_AP_H, ANKL_ML_L, ANKL_ML_H);
		Create6DOF(JOINT_BODY_PELVIS, BODYPART_ABDOMEN, BODYPART_PELVIS, btVector3(0., -height_torso/60, height_pelvis/120), btVector3(0., height_pelvis/60, height_pelvis/120), M_PI_2, 0, M_PI_2, TP_AP_L, TP_AP_H, TP_ML_L, TP_ML_H);
#else
		//Flipped the boundary values on 25.02.2014, used to be 2.5 and 0.5 for AL, ML is the same - 0.67 and 0.67. Flipped again on 3.03.2014, researching the bug where the limits on the targetAngle produced a "jumping" bug. 
		Create6DOF(JOINT_LEFT_HIP, BODYPART_PELVIS, BODYPART_LEFT_LEG, btVector3(height_pelvis/60, 0., 0.), btVector3(0., height_leg/60, 0.), M_PI_2, 0, M_PI_2, HIP_AP_L, HIP_AP_H, HIP_ML_L, HIP_ML_H);
		Create6DOF(JOINT_RIGHT_HIP, BODYPART_PELVIS, BODYPART_RIGHT_LEG, btVector3(-height_pelvis/60, 0., 0.), btVector3(0., height_leg/60, 0.), M_PI_2, 0, M_PI_2, HIP_AP_L, HIP_AP_H, HIP_ML_L, HIP_ML_H);
		//Flipped the boundary values on 25.02.2014, used to be 1.3 and 0.3 for AL. Flipped again on 3.03.2014, researching the bug where the limits on the targetAngle produced a "jumping" bug.  
		Create6DOF(JOINT_LEFT_ANKLE, BODYPART_LEFT_LEG, BODYPART_LEFT_FOOT, btVector3(0., -height_leg/60, 0.), btVector3(0., length_foot/120, 0.), M_PI_2, 0, M_PI_2, ANKL_AP_L, ANKL_AP_H, ANKL_ML_L, ANKL_ML_H);
		//28 degrees (0.62*(pi/4=45degrees)) for ML movement in the ankles 
		Create6DOF(JOINT_RIGHT_ANKLE, BODYPART_RIGHT_LEG, BODYPART_RIGHT_FOOT, btVector3(0., -height_leg/60, 0.), btVector3(0., length_foot/120, 0.), M_PI_2, 0, M_PI_2, ANKL_AP_L, ANKL_AP_H, ANKL_ML_L, ANKL_ML_H);
#endif
		return;
	}

	// DESTRUCTOR:
	virtual	~RagDoll ()
	{
		int i;

		// Remove all constraints
		for ( i = 0; i < JOINT_COUNT; ++i)
		{
			m_ownerWorld->removeConstraint(m_joints[i]);
			delete m_joints[i]; m_joints[i] = 0;
		}

		// Remove all bodies and shapes
		for ( i = 0; i < BODYPART_COUNT; ++i)
		{
			m_ownerWorld->removeRigidBody(m_bodies[i]);
			
			delete m_bodies[i]->getMotionState();

			delete m_bodies[i]; m_bodies[i] = 0;
			delete m_shapes[i]; m_shapes[i] = 0;
		}
	}


	// CREATE BOX:
	void CreateBox(int index, double x, double y, double z, double length, double width, double height, double mass)
	{
		btVector3 positionOffset(0., 0., 0.);
		//m_shapes[index] = new btBoxShape(btVector3(length, width, height));
		m_shapes[index] = new btBoxShape(btVector3(width, height, length));
		btTransform offset; offset.setIdentity();
		offset.setOrigin(positionOffset);

		btTransform transform;

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));
		m_bodies[index] = localCreateRigidBody(btScalar(mass), offset*transform, m_shapes[index]);
		m_bodies[index]->setDamping(0.05, 0.85);
		//m_bodies[index]->setDeactivationTime(0.8);
		//m_bodies[index]->setSleepingThresholds(1.6, 2.5);
		(m_bodies[index])->setUserPointer(&(m_ids[index]));
		m_bodies[index]->setActivationState(DISABLE_DEACTIVATION);

	}


	// CREATE CYLINDER:
	void CreateCylinder(int index, legOrient orient, double x, double y, double z, double length, double width, double height, double mass)
	{
		switch (orient)
		{
		case X_ORIENT:
		{btVector3 positionOffset(0., 0., 0.);
		m_shapes[index] = new btCylinderShapeX(btVector3(width, length, height));
		btTransform offset; offset.setIdentity();
		offset.setOrigin(positionOffset);

		btTransform transform;

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));
		m_bodies[index] = localCreateRigidBody(btScalar(mass), offset*transform, m_shapes[index]);
		m_bodies[index]->setActivationState(DISABLE_DEACTIVATION);
		m_bodies[index]->setDamping(0.05, 0.85);
		//m_bodies[index]->setDeactivationTime(0.8);
		//m_bodies[index]->setSleepingThresholds(1.6, 2.5);
		break;
		}
		case Y_ORIENT:
		{btVector3 positionOffset(0., 0., 0.);
		m_shapes[index] = new btCylinderShape(btVector3(width, length, height));
		btTransform offset; offset.setIdentity();
		offset.setOrigin(positionOffset);

		btTransform transform;

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));
		m_bodies[index] = localCreateRigidBody(btScalar(mass), offset*transform, m_shapes[index]);
		m_bodies[index]->setDamping(0.05, 0.85);
		//m_bodies[index]->setDeactivationTime(0.8);
		//m_bodies[index]->setSleepingThresholds(1.6, 2.5);
		break;
		}
		case Z_ORIENT:
		{btVector3 positionOffset(0., 0., 0.);
		m_shapes[index] = new btCylinderShapeZ(btVector3(width, length, height));
		btTransform offset; offset.setIdentity();
		offset.setOrigin(positionOffset);

		btTransform transform;

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));
		m_bodies[index] = localCreateRigidBody(btScalar(mass), offset*transform, m_shapes[index]);
		m_bodies[index]->setDamping(0.05, 0.85);
		//m_bodies[index]->setDeactivationTime(0.8);
		//m_bodies[index]->setSleepingThresholds(1.6, 2.5);
		break;
		}
		default: {break; }

		}
		(m_bodies[index])->setUserPointer(&(m_ids[index]));
	}

	// CREATE 6-DoF JOINT: 
	void Create6DOF(int jointIndex, int index1, int index2, btVector3& origin1, btVector3& origin2, btScalar euler1, btScalar euler2, btScalar euler3, btScalar APLow, btScalar APHigh, btScalar MLLow, btScalar MLHigh)
	{

		btGeneric6DofConstraint * joint;

		btTransform localA, localB;

		localA.setIdentity(); localB.setIdentity();
		//to make the hinge basis oriented in Z direction I used .setEuler rotation method
		localA.getBasis().setEulerZYX(euler1, euler2, euler3); localA.setOrigin(origin1);
		localB.getBasis().setEulerZYX(euler1, euler2, euler3); localB.setOrigin(origin2);

		joint = new btGeneric6DofConstraint(*m_bodies[index1], *m_bodies[index2], localA, localB, FALSE);
		// first 3 - translational DoFs, second 3 - rotational. 4th - rotation around Y-axis (vertical)
		joint->setLimit(0, 0, 0);//the limits of the joint
		joint->setLimit(1, 0, 0);
		joint->setLimit(2, 0, 0);
		//joint->setLimit(3, 0, 0);
		//joint->setLimit(5, btScalar(APLow), btScalar(APHigh));
		//joint->setLimit(4, btScalar(MLLow), btScalar(MLHigh));
		joint->setLimit(3, 0, 0);
		joint->setLimit(4, btScalar(MLLow), btScalar(MLHigh));
		joint->setLimit(5, btScalar(APLow), btScalar(APHigh));
		m_joints[jointIndex] = joint;
		joint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[jointIndex], true);

	}

	// ACTUATE JOINT:
	void ActuateJoint(int joint_index, int motor_index, double desiredAngle, btScalar timeStep)
	{

		double diff, targetVel;
		float MaxForce = 10.0f;
		//cout << "Target is " << desiredAngle << endl;
		//cout << "Joint["<<joint_index<<"], Motor["<<motor_index<<"] current angle is: "<<m_joints[joint_index]->getRotationalLimitMotor(motor_index)->m_currentPosition<< endl;
		diff = desiredAngle - m_joints[joint_index]->getRotationalLimitMotor(motor_index)->m_currentPosition;
		targetVel = diff / timeStep;
		m_joints[joint_index]->getRotationalLimitMotor(motor_index)->m_enableMotor = true;
		m_joints[joint_index]->getRotationalLimitMotor(motor_index)->m_targetVelocity = targetVel;
		m_joints[joint_index]->getRotationalLimitMotor(motor_index)->m_maxMotorForce = MaxForce;
		m_joints[joint_index]->getRotationalLimitMotor(motor_index)->m_maxLimitForce = 25.0f;

	}
	// save one-dimensional vector of neuronal data (requires external counter):
	void save_1dfileN(vector<double> data, string filename, int counter)
	{
		ofstream saveFile;
		saveFile.open(filename, ios_base::app);
		for (unsigned i = 0; i < data.size(); i++)
		{
			saveFile << setprecision(0) << counter << " " << setprecision(6) << data.at(i) << endl;
			counter++;
		}

		saveFile.close();
	}
	// save one-dimensional vector of joint angle data (uses internal counter):
	void save_1dfileJ(vector<double> data, string filename)
	{
		int counter = 1;
		ofstream saveFile;
		saveFile.open(filename, ios_base::app);
		for (unsigned i = 0; i < data.size(); i++)
		{
			saveFile << setprecision(0)<<counter<< " " << setprecision(6) <<data.at(i)<< endl;
			counter++;
		}
		
		saveFile.close();
	}

	// SAVE BODY PART POSITION (at the end of simulation):
	void Save_Position(int index, unsigned char coordinate, string FileName)
	{
		btRigidBody *body = m_bodies[index];
		btVector3 EndPosition = body->getCenterOfMassPosition();
		// path for location of .txt file with coordinate is hardcoded in here:
		//string path = "C:\\Users\\Lunar\\Documents\\[!Library]\\[!!!Phd]\\[!!UVM]\\[!!!!Lab]\\[!!!Robotics]\\[!Human modeling]\\bullet-2.81-rev2613\\" + FileName + ".txt";
		ofstream outputFile;
		outputFile.open(FileName, ios_base::app);
		switch (coordinate)
		{
		case 'x':
		{
			outputFile << EndPosition.x() << endl;
			break;
		}

		case 'y':
		{
			outputFile << EndPosition.y() << endl;
			break;
		}

		case 'z':
		{
			outputFile << EndPosition.z() << endl;
			break;
		}
		default:
			break;
		}
		outputFile.close();
	}

	// GET CoM POSITION
	// have to specify axis:
	double xCOM(int body, unsigned char axis_dir)
	{
		btRigidBody * pointer = m_bodies[body];
		btVector3 position = pointer->getCenterOfMassPosition();
		switch (axis_dir)
		{
		case 'x':
		{
			return position.x(); 
			break;
		}
		case 'y':
		{
			return position.y();
			break;
		}
		case 'z':
		{
			return position.z();
			break;
		}
		default:
			break;
		}
		
	}

	double fitness()
	{
		btRigidBody * leftFoot = m_bodies[BODYPART_LEFT_FOOT];
		btRigidBody * rightFoot = m_bodies[BODYPART_RIGHT_FOOT];
		btRigidBody * pelvis = m_bodies[BODYPART_PELVIS];

		btVector3 lfPos = leftFoot->getCenterOfMassPosition();
		btVector3 rfPos = rightFoot->getCenterOfMassPosition();
		btVector3 pelPos = pelvis->getCenterOfMassPosition();
		//get rotation of pelvis around the Y-axis:
		btScalar pelRot, junk1, junk2;
		pelvis->getCenterOfMassTransform().getBasis().getEulerZYX(junk1, pelRot, junk2);
		// convert rad to deg:
		pelRot = pelRot * 180 / M_PI;

		double initPelvisHeight = 0.3 + length_foot / 60 + height_leg / 30;
		double leftTargZ = initPelvisHeight / 1.5; //step length = 1/3 of body height, just like in humans
		double leftTargX = height_pelvis / 60;// there should be no movement along X-axis, so the foot should maintain its initial pos along x-axis
		double rightTargZ = initPelvisHeight / 1.5;
		double rightTargX = - height_pelvis / 60;
		
		double result = (1-abs(pelRot/90)) + (-0.1*abs(leftTargX - lfPos.x())+1) + (-0.1*abs(leftTargZ - lfPos.z())+1) + (-0.1*abs(rightTargX - rfPos.x())+1) + (-0.1*abs(rightTargZ - rfPos.z())+1) + pelPos.y() / initPelvisHeight;
		// this version doesn't take height into account for scaffolding.
		//double result = (-0.1*abs(leftTargX - lfPos.x()) + 1) + (-0.1*abs(leftTargZ - lfPos.z()) + 1) + (-0.1*abs(rightTargX - rfPos.x()) + 1) + (-0.1*abs(rightTargZ - rfPos.z()) + 1);
		return result / 6;//all 5 members of expression at maximum reach 1 each, this is normalization. 
	}
#ifdef DEBUG
	void printFitness()
	{
		btRigidBody * leftFoot = m_bodies[BODYPART_LEFT_FOOT];
		btRigidBody * rightFoot = m_bodies[BODYPART_RIGHT_FOOT];
		btRigidBody * pelvis = m_bodies[BODYPART_PELVIS];

		btVector3 lfPos = leftFoot->getCenterOfMassPosition();
		btVector3 rfPos = rightFoot->getCenterOfMassPosition();
		btVector3 pelPos = pelvis->getCenterOfMassPosition();

		//get rotation of pelvis around the Y-axis:
		btScalar pelRot, junk1, junk2;
		pelvis->getCenterOfMassTransform().getBasis().getEulerZYX(junk1, pelRot, junk2);
		// convert rad to deg:
		pelRot = pelRot * 180 / M_PI;

		double initPelvisHeight = 0.3 + length_foot / 60 + height_leg / 30;
		double leftTargZ = initPelvisHeight / 1.5; //step length = 1/3 of body height, just like in humans
		double leftTargX = height_pelvis / 60;// there should be no movement along X-axis, so the foot should maintain its initial pos along x-axis
		double rightTargZ = initPelvisHeight / 1.5;
		double rightTargX = -height_pelvis / 60;

		
		// this version doesn't take height into account for scaffolding.
		cout << "===========================" << endl;
		cout << "Left target (X,Z) = (" << leftTargX << ", " << leftTargZ << ")" << endl;
		cout << "Right target (X,Z) = (" << rightTargX << ", " << rightTargZ << ")" << endl;
		cout << "Initial pelvis height = " << initPelvisHeight << endl;
		cout << "Left foot (X,Z) = (" << lfPos.x() << ", " << lfPos.z() << ")" << endl;
		cout << "Right foot (X,Z) = (" << rfPos.x() << ", " << rfPos.z() << ")" << endl;
		cout << "Current pelvis HEIGHT = " << pelPos.y() << endl;
		cout << "Current pelvis ROTATION = " << pelRot << " degrees." << endl;
		double result = (1-abs(pelRot / 90)) + (-0.1*abs(leftTargX - lfPos.x()) + 1) + (-0.1*abs(leftTargZ - lfPos.z()) + 1) + (-0.1*abs(rightTargX - rfPos.x()) + 1) + (-0.1*abs(rightTargZ - rfPos.z()) + 1) + pelPos.y() / initPelvisHeight;
		//double result = (-0.1*abs(leftTargX - lfPos.x()) + 1) + (-0.1*abs(leftTargZ - lfPos.z()) + 1) + (-0.1*abs(rightTargX - rfPos.x()) + 1) + (-0.1*abs(rightTargZ - rfPos.z()) + 1);
		cout << "Current fitness = " << result / 6 << endl; //all 5 members of expression at maximum reach 1 each, this is normalization. 
	}
#endif

	void isUpright()
	{
		btRigidBody * pelvis = m_bodies[BODYPART_PELVIS];
		btVector3 pelPos = pelvis->getCenterOfMassPosition();

		if (pelPos.y() < PELV_HEIGHT * 0.75)
		{
			ofstream outputFile;
			outputFile.open("fit.txt", ios_base::app);
			outputFile << 0. << endl;
			outputFile.close();
			exit(0);
		}
	}

	//void mexFunction(int nlhs, mxArray *plhs[],
	//	int nrhs, const mxArray *prhs[])
	//{
	//	double *y;
	//	double x;

	//	/* Create a 1-by-1 matrix for the return argument. */
	//	plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);

	//	/* Get the scalar value of the input x. */
	//	/* Note: mxGetScalar returns a value, not a pointer. */
	//	x = mxGetScalar(prhs[0]);

	//	/* Assign a pointer to the output. */
	//	y = mxGetPr(plhs[0]);

	//	/* Call the timestwo_alt subroutine. */
	//	timestwo_alt(y, x);
	//}
}; //END OF RagDoll CLASS

static RagdollDemo * ragdollDemo; //for processing touches

// BINARY TOUCH SENSOR SYSTEM:
// detects if two bodies are in contact and assigns 1's to respective values in touches[number of body parts + 1 for ground]
bool myContactProcessedCallback(btManifoldPoint& cp,
	void * body0, void * body1)
{
	int
		* ID1, * ID2;
	btCollisionObject * o1 = static_cast<btCollisionObject * >(body0);
	btCollisionObject * o2 = static_cast<btCollisionObject * >(body1);
#ifdef TORSO
	int groundID = 7;
#else
	int groundID = 6;
#endif
	ID1 = static_cast<int * >(o1->getUserPointer());
	ID2 = static_cast<int * >(o2->getUserPointer());

//	printf("ID1 = %d, ID2 = %d\n",*ID1,*ID2); //Technical line to check if the registration of collisions works

	ragdollDemo->touches[*ID1] = 1;
	ragdollDemo->touches[*ID2] = 1;
	ragdollDemo->touchPoints[*ID1] = cp.m_positionWorldOnB;
	ragdollDemo->touchPoints[*ID2] = cp.m_positionWorldOnB;
	return false;
}

// READING FROM LOCAL DIRECTORY:
//how to read is directed here:
void load_data(const string& file_name, vector<vector<double> >& data) {
	ifstream is(file_name, ios::in | ios::binary);
	if (!is.is_open()) {
		cout << "Failed open " << file_name << endl;
		return;
	}
	//cout << file_name << endl; // To debug which file is being read for input. 
	double i;
	string line;
	while (getline(is, line)) {
		stringstream ss(line);
		data.push_back(vector<double>());
		while (ss >> i)
			data.back().push_back(i);
	}
	is.close();
}

// hardcode the file name here:
RagdollDemo::RagdollDemo() :m_inputFileName("weights.txt")
{}
// execute reading:
void RagdollDemo::initParams(const std::string& inputFileName)
{
	if (!inputFileName.empty())
		m_inputFileName = inputFileName;
}
// END READING WEIGHTS LOCALLY

//CTRNN CODE:
//========================================================================================================

// (update function, initial value, time1, time2, step size, time constant)
vector<vector<double>> RagdollDemo::euler(double neural_step, double h, double tau, vector<vector<double>> w, vector<vector<double>> neuron_vals, vector<vector<double>> bias, vector<int> touches, vector<vector<double>> gain)
{
#ifdef NEURON
	//vector for neuronal values:
	vector<vector<vector<double>>> tempHist;//layer * neuron * time
	//to export neuron vals, initialize a temp 3d vector
	for (int layer = 0; layer < 3; layer++)
	{
		tempHist.push_back(vector<vector<double>>());
		for (unsigned neuron = 0; neuron < neuron_val.at(layer).size(); neuron++)
		{
			tempHist[layer].push_back(vector<double>());
			for (int time = 0; time < (neural_step/h); time++)
			{
				tempHist[layer][neuron].push_back(double());
			}
		}
	}
#endif	
	// Separate weight matrices:
	vector<double> temp_row;
	vector<vector<double>> wIJ, wJK;
	// populate the wIJ matrix
	for (unsigned i = 0; i < w[0].size(); i++)
	{
		for (int j = 0; j < num_input; j++)
		{
			temp_row.push_back(w[j][i]);
		}

		wIJ.push_back(temp_row);
		temp_row.clear();
	}
	//populate the wJK matrix
	for (int j = 0; j < num_output; j++)
	{
		for (int i = num_input; i < num_hidden + num_input; i++)
		{
			temp_row.push_back(w[i][j]);
		}
		wJK.push_back(temp_row);
		temp_row.clear();
	}
	// Separate the array with neuronal values to layers to be processed separately and then stored in the original vector:
	vector<double> input = neuron_vals.at(0);
	vector<double> hidden = neuron_vals.at(1);
	vector<double> output = neuron_vals.at(2);

	// Separate the array with bias values to layers to be processed separately:
	vector<double> input_bias = bias.at(0);
	vector<double> hidden_bias = bias.at(1);
	vector<double> output_bias = bias.at(2);

	// Separate the array with gain values to layers to be processed separately:
	vector<double> input_gain = gain.at(0);
	vector<double> hidden_gain = gain.at(1);
	vector<double> output_gain = gain.at(2);

	// create zero array the size of input layer to substitute for non-existent previous layer, 
	//can be modified to use feedback connections from hidden and output layers:
	vector<double> zeros = {0.,0.};

	//create zero vectors to put for absent sensor values for hidden and output layers calculations:
	vector<double> hidden_sensors; 
	vector<double> output_sensors;

	//populate sensor vectors for hid/out layers with zeros (don't receive sensor inputs):
	for (int i = 0; i < num_hidden; i++) { hidden_sensors.push_back(0); }
	for (int i = 0; i < num_output; i++) { output_sensors.push_back(0); }

#ifdef NEURON
	//internal counter for tempHist:
	int counter = 0;
#endif

	//Main cycle of neuronal values calculation
	// Updates all neuronal values for allotted time "neural step" using step size "h": 
	for (int t = 0; t < (neural_step/h); t++)
	{
		for (unsigned i = 0; i < neuron_vals.at(0).size(); i++)
		{
			// Here, weight matrix wIJ is used (which is wrong, since there are no weights from zero fictional layer)
			// but since the zero layer has only zeros, weights don't matter. Similarly, gains are for calculating the 
			// summed influence of all neurons from previous layer, and thus gains are parsed from the previous layer:
			input.at(i) += h * updateNeuron(tau, zeros, input.at(i), wIJ[i], zeros, touches.at(i), gain.at(0));
		}

		//Cycle to go through all hidden nerons
		for (unsigned i = 0; i < neuron_vals.at(1).size(); i++)
		{
			hidden.at(i) += h * updateNeuron(tau, neuron_vals.at(0), hidden.at(i), wIJ.at(i), input_bias, hidden_sensors.at(i), gain.at(0));
		}
		
		//Cycle to go through all output nerons
		for (unsigned i = 0; i < neuron_vals.at(2).size(); i++)
		{
			output.at(i) += h * updateNeuron(tau, neuron_vals.at(1), output.at(i), wJK.at(i), hidden_bias, output_sensors.at(i), gain.at(1));
		}

#ifdef NEURON
		// store current neuronal values for printing:
		for (int layer = 0; layer < 3; layer++)
		{
			for (unsigned neuron = 0; neuron < neuron_vals.at(layer).size(); neuron++)
			{
				switch (layer) {
				case(0) : {tempHist[layer][neuron][counter] = input[neuron];  break; }
				case(1) : {tempHist[layer][neuron][counter] = hidden[neuron]; break; }
				case(2) : {tempHist[layer][neuron][counter] = output[neuron]; break; }
				}
			}
		}
		//update tempHist counter:
		counter++; tsCounter++;
#endif
	}//End MAIN cycle (through time)

	//update neuronal values:
	neuron_vals.clear(); neuron_vals.push_back(input); neuron_vals.push_back(hidden); neuron_vals.push_back(output);

#ifdef NEURON
	//store neuron vals in a file (VERYYYY SLOW):
	for (int layer = 0; layer < 3; layer++)
	{
		for (unsigned neuron = 0; neuron < neuron_val.at(layer).size(); neuron++)
		{
			string fileName;
			switch (layer)
			{
			case 0:
			{
				fileName = "input" + to_string(neuron + 1) + ".txt";
				break;
			}
			case 1:
			{
				fileName = "hidden" + to_string(neuron + 1) + ".txt";
				break;
			}
			case 2:
			{
				fileName = "output" + to_string(neuron + 1) + ".txt";
				break;
			}
			}
			//cout << "Saving layer " << layer << " and neuron " << neuron << endl;
			m_ragdolls[0]->save_1dfileN(tempHist[layer][neuron], fileName,(tsCounter+1-(neural_step/h))); //-> to be used if only end of simulation fitness is reported
		}
	}
#endif

	return neuron_vals;
}

// function to update a single neuron according to CTRNN formula
double RagdollDemo::updateNeuron(double tau, vector<double> previous_layer, double current_neuron, vector<double> w, vector<double> bias_val, int sensor_val, vector<double> gain)
{
	//var to store all of the summed input to the neuron:	
	double sum_inputs = 0;
	double fcn_val;
	//cycle through all neurons of previous layer:
	for (unsigned j = 0; j < previous_layer.size(); j++) 
		{ 
			sum_inputs += w[j]*tanh((previous_layer.at(j) + bias_val[j]) * gain.at(j));
		}
	//update neuron's state
	fcn_val = (-current_neuron + sensor_val + sum_inputs)*(1 / tau);
	return fcn_val;
}
//END CTRNN CODE:
//========================================================================================================

void RagdollDemo::initPhysics()
{
//ADDED:
	srand(time(NULL));
	ragdollDemo = this;// for processing touches, note spelling "ragdollDemo"
	gContactProcessedCallback = myContactProcessedCallback; //Registers the collision
	SimulationStep = 0; //time step counter to exit after desired # of steps
	tempFitness = 0;
#ifdef NEURON
	tsCounter = 0;
#endif

#ifdef TRAIN
	pause = false;
#else
	pause = true;
#endif
	oneStep = false;

//---- intializing bodypart IDs
#ifdef TORSO
	for (int i = 0; i < 8; i++)
	{
		IDs[i] = i;
	};
#else
	for (int i = 0; i < 7; i++)
	{
		IDs[i] = i;
	};
#endif
	// initalize neuron_val, bias and gains(if inside ClienMoveandDisplay - they are reset to 0's each simulation step): 
	for (int i = 0; i < num_input; i++) { temp_row.push_back(0); temp_bias_row.push_back(bias_val); temp_gain_row.push_back(gain_val); }

	neuron_val.push_back(temp_row); temp_row.clear();
	bias.push_back(temp_bias_row); temp_bias_row.clear();
	gain.push_back(temp_gain_row); temp_gain_row.clear();

	for (int i = 0; i < num_hidden; i++) { temp_row.push_back(0); temp_bias_row.push_back(bias_val); temp_gain_row.push_back(gain_val); }

	neuron_val.push_back(temp_row); temp_row.clear();
	bias.push_back(temp_bias_row); temp_bias_row.clear();
	gain.push_back(temp_gain_row); temp_gain_row.clear();

	for (int i = 0; i < num_output; i++) { temp_row.push_back(0); temp_bias_row.push_back(bias_val); temp_gain_row.push_back(gain_val); }
	neuron_val.push_back(temp_row); temp_row.clear();
	bias.push_back(temp_bias_row); temp_bias_row.clear();
	gain.push_back(temp_gain_row); temp_gain_row.clear();

#ifdef JOINT
	//intialize 2d vector for joint angles:
	for (int i = 0; i < num_output; i++)
	{
		joint_val.push_back(vector<double>());
		for (int j = 0; j < maxStep; j++)
		{
			joint_val[i].push_back(double());		
		}
	}
#endif
	//READ WEIGHTS:
	// from input file into var called 'w': 
	load_data(m_inputFileName, w);

//END ADDED

	// Setup the basic world

	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(9.));// CAMERA DISTANCE in the beginning of the simulation

	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);

	m_solver = new btSequentialImpulseConstraintSolver;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	//m_dynamicsWorld->getDispatchInfo().m_useConvexConservativeDistanceUtil = true;
	//m_dynamicsWorld->getDispatchInfo().m_convexConservativeDistanceThreshold = 0.01f;



	// Setup a big ground box
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-10,0));

#define CREATE_GROUND_COLLISION_OBJECT 1
#ifdef CREATE_GROUND_COLLISION_OBJECT
		btCollisionObject* fixedGround = new btCollisionObject();
		fixedGround->setCollisionShape(groundShape);
		fixedGround->setWorldTransform(groundTransform);
#ifdef TORSO
		fixedGround->setUserPointer(&IDs[7]);
#else
		fixedGround->setUserPointer(&IDs[6]);
#endif
		m_dynamicsWorld->addCollisionObject(fixedGround);
#else
		localCreateRigidBody(btScalar(0.),groundTransform,groundShape);
#endif //CREATE_GROUND_COLLISION_OBJECT

	}

	// Spawn one ragdoll
	btVector3 startOffset(1,0.5,0);
	spawnRagdoll(startOffset);

	// Create the second ragdoll at a different location:
	//startOffset.setValue(-1,0.5,0);
	//spawnRagdoll(startOffset);

	clientResetScene();		
}

void RagdollDemo::spawnRagdoll(const btVector3& startOffset)
{
	RagDoll* ragDoll = new RagDoll (m_dynamicsWorld, startOffset, IDs);
	m_ragdolls.push_back(ragDoll);
}	

// define a const that will store the number of frames after which to display graphics.
// Set > than simulation time to have no video at all 
#ifdef TRAIN
#define TICKS_PER_DISPLAY maxStep // should be bigger than total ticks for the whole simulation
#else
#define TICKS_PER_DISPLAY 1
#endif



void RagdollDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	// vector of target angle values:
	vector<double> targ_angs;

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
	//float printMS = ms;
	//float minFPS = 100000.f/60.f;
	float minFPS = 60.f; // 600 FPS necessary to update sensor values rapidly. 
	float minFPStime = 1000000.f / minFPS; // 1 sec = 10^6 microsec; time for 1 frame, effectively.
	// minFPS is downregulated by a factor of 10 (used to be 10^6/60)
	// to avoid sensors being silent too long.
	// Change this number here and in the code below

	float sensorThreshold = 4000.f; //need to progress simulation at least this many micsec to continuously update sensors
	if (ms < sensorThreshold)
		ms = sensorThreshold;
	
	if (ms > minFPStime)
		ms = minFPStime;

	//!!!!look into best values for these vars:
	neural_step = 1;
	h = 0.1;

	//Add actuation time step: 
	float ActuateTimeStep = 10 * ms / 1000000.f;//10 x more time for joint actuation reduces motor jitter

	if (m_dynamicsWorld)
	{		
		if (!pause || oneStep)
		{
			for (int l = 0; l < TICKS_PER_DISPLAY; l++)
			{
				//Intiation of the touches
#ifdef TORSO
				for (int i = 0; i < 8; i++)
				{
					touches[i] = 0;
				}
				
				//Making sure all the body parts are active every time step:
				//Body parts change color when inactive for sometime:
				for (int k = 0; k<8; k++)
				{
					m_ragdolls[0]->m_bodies[k]->setActivationState(ACTIVE_TAG);
				}
#else
				for (int i = 0; i < 7; i++)
				{
					touches[i] = 0;
				}

				//Making sure all the body parts are active every time step:
				//Body parts change color when inactive for sometime:
				for (int k = 0; k<7; k++)
				{
					m_ragdolls[0]->m_bodies[k]->setActivationState(ACTIVE_TAG);
				}
#endif
				// Populate sensor_val for the first update of CTRNN:
				if (SimulationStep == 0)
				{
					for (int j = 0; j < num_input; j++)
					{
						sensor_val.push_back(0);
					}
				}

				//update neronal states:
				neuron_val = euler(neural_step, h, tau, w, neuron_val, bias, sensor_val, gain);
				
				//extract values from output layer:
				targ_angs = neuron_val.at(2);
	
				// for all motors
				for (int i = 0; i<num_output; i++)
				{
					double targetAngle = tanh(targ_angs.at(i));

					// check which motor is to be actuated (values are scaled to the desired angle)
					switch (i)
					{
					case 0: // Body-pelvis, ML
					{
						targetAngle = 27.9 * targetAngle; //TP_ML [-27.9; 27.9]
						break;
					}
					case 1: // Body-pelvis AP
					{
						targetAngle = 36 * targetAngle + 22.5; //TP_AP [58.5; -13.5]
						//targetAngle = 36 * targetAngle - 22.5; //[-58.5; 13.5] according to constraints on the joints (?)
						break;
					}
					case 2: //Left Hip ML
					{
						targetAngle = 30 * targetAngle; //HIP_ML [-30; 30]
						break;
					}
					case 3: //Left Hip AP
					{
						targetAngle = 67.5 * targetAngle + 45; //HIP_AP [-22.5; 112.5]
						//targetAngle = 67.5 * targetAngle - 45; //[22.5; -112.5] according to constraints on the joints (?)
						break;
					}
					case 4: //Right Hip ML
					{
						targetAngle = 30 * targetAngle; //HIP_ML [-30; 30]
						break;
					}
					case 5: //Right Hip AP
					{
						targetAngle = 67.5 * targetAngle + 45; //HIP_AP [-22.5; 112.5]
						//targetAngle = 67.5 * targetAngle - 45; //[22.5; -112.5] according to constraints on the joints (?)
						break;
					}
					case 6: //Left Ankle ML
					{
						targetAngle = 27.9 * targetAngle; //ANKL_ML [-27.9; 27.9]
						break;
					}
					case 7: //Left Ankle AP
					{
						targetAngle = 36 * targetAngle + 22.5; //ANKL_AP [58.5; -13.5]
						//targetAngle = 36 * targetAngle - 22.5;//[-58.5; 13.5] according to constraints on the joints (?)
						break;
					}
					case 8: //Right Ankle ML
					{
						targetAngle = 27.9 * targetAngle; //ANKL_ML [-27.9; 27.9]
						break;
					}
					case 9: //Right Ankle AP
					{
						targetAngle = 36 * targetAngle + 22.5; //ANKL_AP [58.5; -13.5]
						//targetAngle = 36 * targetAngle - 22.5; //[-58.5; 13.5] according to constraints on the joints (?)
						break;
					}
					default:
						break;
					}
					
					//convert to radians:
					targetAngle = targetAngle*M_PI/180;
					//actuate the i-th motor (indices should be 1 and 2 since in rotational motor there are three motors)
					//motor[0] twists around vertical Y-axis
					m_ragdolls[0]->ActuateJoint(i / 2, abs(sin(i*M_PI / 2))+1, targetAngle, ActuateTimeStep);
					

				}
				// END UPDATE MOTORS

				// Update simulation
				m_dynamicsWorld->stepSimulation(ms / 1000000.f);
#ifdef DEBUG
				//get rotation of pelvis around the Y-axis:
				btScalar pelvisRot, junk10, junk20;
				m_ragdolls[0]->m_bodies[BODYPART_PELVIS]->getCenterOfMassTransform().getBasis().getEulerZYX(junk10, pelvisRot, junk20);
				cout << "Pelvis is initially at " << pelvisRot * 180 / M_PI << " degrees." << endl;
				//Stopping simulation in the end of time for DEMO robots (paused right before the end)
				m_ragdolls[0]->printFitness();
#endif

#ifdef JOINT
				//store joint angles for exporting:
				for (int i = 0; i < num_output; i++)
				{
					joint_val[i][SimulationStep] = m_ragdolls[0]->m_joints[i/2]->getRotationalLimitMotor(abs(sin(i*M_PI / 2)) + 1)->m_currentPosition*180/M_PI;
					//cout << "Joint[" << i + 1 << "] = " << joint_val[i][SimulationStep] << endl;
				}
				//DEBUG when touches are updated:
#endif
				//Get new sensor vals:
				sensor_val.clear();
				for (int j = 0; j < num_input; j++)
				{
#ifdef TORSO
					sensor_val.push_back(touches[j+4]);//the shift is to the fourth value (left foot) and fifth (right foot)
#else 
					sensor_val.push_back(touches[j + 3]);
#endif
				}

				// Increase the simulation time step counter:
				SimulationStep++;

				// make oneStep false, so that the simulation is paused
				// and waiting next press of the button:
				if (oneStep)
				{
					oneStep = false;
					pause = true;
				}
			}// END NO VIDEO LOOP
		}// END if(!pause && oneStep)
		
	
		
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();

#ifdef NEURON
		// Atm there are 10 neural steps per each simStep->takes forever to run 1000 simSteps.
		//stopping simulation
		if (SimulationStep >= maxStep)
		{
			exit(0);
		}
#endif

#ifdef JOINT
		//saving the joint angle values to a file:
		if (SimulationStep >= maxStep)
		{
			for (int i = 0; i < num_output; i++)
			{
			string fileName;
			fileName = "joint" + to_string(i + 1) + ".txt";
			m_ragdolls[0]->save_1dfileJ(joint_val[i], fileName); //-> to be used if only end of simulation fitness is reported
			}
			exit(0);
		}
#endif

#ifndef TRAIN //if DEMO!!!
		//Stopping simulation in the end of time for DEMO robots (paused right before the end)
		if (SimulationStep >= maxStep)
		{
			getchar();
			exit(0);
		}
#endif

		// TIME-CONSTRAINED SIMULATION:
		// !!!!!add ability to exit also by a failure parameter (e.g., robot fell)
#ifdef TRAIN

		// terminate simulation if robot falls and give 0 fitness:
		//m_ragdolls[0]->isUpright();

		/*printf("%d, ", SimulationStep);*/
		if (SimulationStep>=maxStep)
			{
				//this exports only a body-part final position:
				//m_ragdolls[0]->Save_Position(BODYPART_LEFT_FOOT,'x',"fit.txt"); //

				


				//this exports fitness calculated by fitness() fcn:
				ofstream outputFile;
				outputFile.open("fit.txt", ios_base::app);
				outputFile << m_ragdolls[0]->fitness() << endl;
				outputFile.close();
				//m_ragdolls[0]->Save_DEBUG(2); //-> this is simpler function which only reports Z, X positions of the bodiparts parsed in.

				//Saving into a text file part of the intra-simulation fitness process, pls note the normalization on SimStep:
		//		ofstream outputFile;
		//		outputFile.open("APA.txt");
		//        outputFile << SimulationStep << endl;
		//		//outputFile << tempFitness<< " , " << SimulationStep << endl; //Line to debug!
	    //        outputFile.close();
		//		//End of intra-simulation fitness process. 	
		//		getchar(); //to debug
				exit(0);
			}
#endif

	}

	renderme(); 

	glFlush();

	glutSwapBuffers();
}

void RagdollDemo::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}

void RagdollDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
		// ADDING A NEW RAGDOLL:
//	case 'e':
//		{
//		btVector3 startOffset(0,2,0);
//		spawnRagdoll(startOffset);
//		break;
//		}
	case 'p':
	{
		pause = !pause;
		break;
	}
	case 'o':
	{
		oneStep = true;
		break;
	}
	default:
		DemoApplication::keyboardCallback(key, x, y);
	}

	
}



void	RagdollDemo::exitPhysics()
{

	int i;

	for (i=0;i<m_ragdolls.size();i++)
	{
		RagDoll* doll = m_ragdolls[i];
		delete doll;
	}

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;

	
}





