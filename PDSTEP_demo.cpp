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

// These values are obtained from:
// (1) (Sample size = 1892) "Normal Hip and Knee Active Range of Motion: The Relationship to Age", KE ROach, TP Miles, PHYS THER, 1991. Taken from Table 3, All Ages column. 
// (2) (Sample size = 537) "Normal Range of Motion of the Hip, Knee and Ankle Joints in Male Subjects, 30-40 Years of Age", A Roaas & GBJ Andersson, Acta orthop. scand. 1982. Table 1
// (3) (Sample size = 100) "Three-dimensional Lumbar Spinal Kinematics: A Study of Range of Movement in 100 Healthy Subjects Aged 20 to 60+ Years". G van Herp et al. Rheumatology, 2000. Table I, Age 20-29 averaged for F and M
#define HIP_AP_L -(M_PI_4)*0.42222 // -19 (1)
#define HIP_AP_H (M_PI_4)*2.68888 // 121 (1)
#define HIP_ML_L -(M_PI_4)*0.86222 // -38.8 (2) Abduction
#define HIP_ML_H (M_PI_4)*0.677777 // 30.5 (2) Adduction
#define ANKL_AP_L -(M_PI_4)*0.34 // -15.3 (2) Dorsiflexion
#define ANKL_AP_H (M_PI_4)*0.882222 // 39.7 (2) Plantar flexion
#define ANKL_ML_L -(M_PI_4)*0.616666 // -27.75 (2) Eversion
#define ANKL_ML_H (M_PI_4)*0.616666 // 27.75 (2) Inversion
#define TP_AP_L -(M_PI_4)*1.281111 // -57.65 (3) Flexion
#define TP_AP_H (M_PI_4)*0.661111 // 29.75 (3) Extension
#define TP_ML_L -(M_PI_4)*0.565555 // -25.45 (3) Left side bending
#define TP_ML_H (M_PI_4)*0.583333 // 26.25 (3) Left side bending
#define KNEE_AP_L -(M_PI_4)*2.93333 // -132 (1) flexion
#define KNEE_AP_H 0 // extesion was near 0
#define KNEE_ML_L 0
#define KNEE_ML_H 0

#define INPUT_TAU 2.0
#define HIDDEN_TAU 2.0
#define OUTPUT_TAU 2.0

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
#ifndef KNEES
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
#else
	enum
	{
		BODYPART_PELVIS = 0, //1
		BODYPART_LEFT_THIGH, //2
		BODYPART_RIGHT_THIGH, //3
		BODYPART_LEFT_SHANK, //4
		BODYPART_RIGHT_SHANK, //5
		BODYPART_LEFT_FOOT, //6
		BODYPART_RIGHT_FOOT, //7
		BODYPART_PLATFORM, //8

		BODYPART_COUNT
	};
#endif
#endif

	// in the case of torso - one more joint
#ifdef TORSO
		enum
		{
			JOINT_LEFT_HIP=0, //1
			JOINT_RIGHT_HIP, //2
			JOINT_LEFT_ANKLE, //3
			JOINT_RIGHT_ANKLE, //4
			JOINT_BODY_PELVIS, //5

			JOINT_COUNT
		};

#else
#ifndef KNEES 
	enum
	{
		
		JOINT_LEFT_HIP = 0, //1	
		JOINT_RIGHT_HIP, //2
		JOINT_LEFT_ANKLE, //3
		JOINT_RIGHT_ANKLE, //4

		JOINT_COUNT
	};
#else
	enum
	{
		JOINT_LEFT_HIP = 0, //1
		JOINT_RIGHT_HIP, //2
		JOINT_LEFT_ANKLE, //3
		JOINT_RIGHT_ANKLE, //4
		JOINT_LEFT_KNEE, //5
		JOINT_RIGHT_KNEE, //6

		JOINT_COUNT
	};
#endif
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
	btJointFeedback fg;
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
#ifndef KNEES
		CreateBox(BODYPART_PLATFORM, 0, 0.15, 0, 6., 4., 0.15, 200.);
		CreateBox(BODYPART_PELVIS, 0., 0.3+length_foot/60+height_leg/30, 0., height_pelvis/60, height_pelvis/60, height_pelvis/60, mass_pelvis);
		CreateBox(BODYPART_LEFT_FOOT, height_pelvis/60, 0.3+length_foot/120, 0., length_foot/60, length_foot/90, length_foot/120, mass_foot);
		CreateBox(BODYPART_RIGHT_FOOT, -height_pelvis/60, 0.3+length_foot/120, 0., length_foot/60, length_foot/90, length_foot/120, mass_foot);
#else
		CreateBox(BODYPART_PLATFORM, 0, 0.15, 0, 6., 4., 0.15, 200.);
		// mixed up dimensions: width, height, length, now corrected to that described: length, width, height
		// width - mediolateral dir, length into the screen
		CreateBox(BODYPART_PELVIS, 0., 0.3 + length_foot / 60 + height_leg / 30, 0., height_pelvis / 60, height_pelvis / 60, height_pelvis / 60, mass_pelvis);
		CreateBox(BODYPART_LEFT_FOOT, height_pelvis / 60, 0.3 + length_foot / 120, 0., length_foot / 60, length_foot / 90, length_foot / 120, mass_foot);
		CreateBox(BODYPART_RIGHT_FOOT, -height_pelvis / 60, 0.3 + length_foot / 120, 0., length_foot / 60, length_foot / 90, length_foot / 120, mass_foot);
#endif
#endif	


		//CREATE LEGS:
#ifndef KNEES
		CreateCylinder(BODYPART_LEFT_LEG, Y_ORIENT, height_pelvis/60, 0.3+length_foot/60+height_leg/60, 0., height_leg/60, 0.15, 1., mass_leg);
		CreateCylinder(BODYPART_RIGHT_LEG, Y_ORIENT, -height_pelvis/60, 0.3+length_foot/60+height_leg/60, 0., height_leg/60, 0.15, 1., mass_leg);
#else
		CreateCylinder(BODYPART_LEFT_THIGH, Y_ORIENT, height_pelvis / 60, 0.3 + length_foot / 60 + height_shank / 30 + height_thigh/60, 0., height_thigh / 60, 0.15, 1., mass_thigh);
		CreateCylinder(BODYPART_RIGHT_THIGH, Y_ORIENT, -height_pelvis / 60, 0.3 + length_foot / 60 + height_shank / 30 + height_thigh / 60, 0., height_thigh / 60, 0.15, 1., mass_thigh);
		CreateCylinder(BODYPART_LEFT_SHANK, Y_ORIENT, height_pelvis / 60, 0.3 + length_foot / 60 + height_shank / 60, 0., height_shank / 60, 0.15, 1., mass_shank);
		CreateCylinder(BODYPART_RIGHT_SHANK, Y_ORIENT, -height_pelvis / 60, 0.3 + length_foot / 60 + height_shank / 60, 0., height_shank / 60, 0.15, 1., mass_shank);
#endif
		//CREATE JOINTS:
		//vectors in argument are the joint location in local body part's coordinate system

#ifdef TORSO

		//Flipped the boundary values on 25.02.2014, used to be 2.5 and 0.5 for AL, ML is the same - 0.67 and 0.67. Flipped again on 3.03.2014, researching the bug where the limits on the targetAngle produced a "jumping" bug. 
		Create6DOF(JOINT_LEFT_HIP, BODYPART_PELVIS, BODYPART_LEFT_LEG, btVector3(height_pelvis/60, 0., 0.), btVector3(0., height_leg/60, 0.), M_PI_2, 0, M_PI_2, HIP_AP_L,HIP_AP_H,HIP_ML_L,HIP_ML_H);
		Create6DOF(JOINT_RIGHT_HIP, BODYPART_PELVIS, BODYPART_RIGHT_LEG, btVector3(-height_pelvis/60, 0., 0.), btVector3(0., height_leg/60, 0.), M_PI_2, 0, M_PI_2, HIP_AP_L, HIP_AP_H, HIP_ML_L, HIP_ML_H);
		//Flipped the boundary values on 25.02.2014, used to be 1.3 and 0.3 for AL. Flipped again on 3.03.2014, researching the bug where the limits on the targetAngle produced a "jumping" bug.  
		Create6DOF(JOINT_LEFT_ANKLE, BODYPART_LEFT_LEG, BODYPART_LEFT_FOOT, btVector3(0.,-height_leg/60, 0.), btVector3(0.,length_foot/120, 0.), M_PI_2, 0, M_PI_2, ANKL_AP_L, ANKL_AP_H, ANKL_ML_L, ANKL_ML_H);
		//28 degrees (0.62*(pi/4=45degrees)) for ML movement in the ankles 
		Create6DOF(JOINT_RIGHT_ANKLE, BODYPART_RIGHT_LEG, BODYPART_RIGHT_FOOT, btVector3(0.,-height_leg/60, 0.), btVector3(0.,length_foot/120, 0.), M_PI_2, 0, M_PI_2, ANKL_AP_L, ANKL_AP_H, ANKL_ML_L, ANKL_ML_H);
		Create6DOF(JOINT_BODY_PELVIS, BODYPART_ABDOMEN, BODYPART_PELVIS, btVector3(0., -height_torso/60, height_pelvis/120), btVector3(0., height_pelvis/60, height_pelvis/120), M_PI_2, 0, M_PI_2, TP_AP_L, TP_AP_H, TP_ML_L, TP_ML_H);
#else
#ifndef KNEES
		//Flipped the boundary values on 25.02.2014, used to be 2.5 and 0.5 for AL, ML is the same - 0.67 and 0.67. Flipped again on 3.03.2014, researching the bug where the limits on the targetAngle produced a "jumping" bug. 
		Create6DOF(JOINT_LEFT_HIP, BODYPART_PELVIS, BODYPART_LEFT_LEG, btVector3(height_pelvis/60, 0., 0.), btVector3(0., height_leg/60, 0.), M_PI_2, 0, M_PI_2, HIP_AP_L, HIP_AP_H, HIP_ML_L, HIP_ML_H);
		Create6DOF(JOINT_RIGHT_HIP, BODYPART_PELVIS, BODYPART_RIGHT_LEG, btVector3(-height_pelvis/60, 0., 0.), btVector3(0., height_leg/60, 0.), M_PI_2, 0, M_PI_2, HIP_AP_L, HIP_AP_H, HIP_ML_L, HIP_ML_H);
		//Flipped the boundary values on 25.02.2014, used to be 1.3 and 0.3 for AL. Flipped again on 3.03.2014, researching the bug where the limits on the targetAngle produced a "jumping" bug.  
		Create6DOF(JOINT_LEFT_ANKLE, BODYPART_LEFT_LEG, BODYPART_LEFT_FOOT, btVector3(0., -height_leg/60, 0.), btVector3(0., length_foot/120, 0.), M_PI_2, 0, M_PI_2, ANKL_AP_L, ANKL_AP_H, ANKL_ML_L, ANKL_ML_H);
		//28 degrees (0.62*(pi/4=45degrees)) for ML movement in the ankles 
		Create6DOF(JOINT_RIGHT_ANKLE, BODYPART_RIGHT_LEG, BODYPART_RIGHT_FOOT, btVector3(0., -height_leg/60, 0.), btVector3(0., length_foot/120, 0.), M_PI_2, 0, M_PI_2, ANKL_AP_L, ANKL_AP_H, ANKL_ML_L, ANKL_ML_H);
#else // if KNEES:
		Create6DOF(JOINT_LEFT_HIP, BODYPART_PELVIS, BODYPART_LEFT_THIGH, btVector3(height_pelvis / 60, 0., 0.), btVector3(0., height_thigh / 60, 0.), M_PI_2, 0, M_PI_2, HIP_AP_L, HIP_AP_H, HIP_ML_L, HIP_ML_H);
		Create6DOF(JOINT_RIGHT_HIP, BODYPART_PELVIS, BODYPART_RIGHT_THIGH, btVector3(-height_pelvis / 60, 0., 0.), btVector3(0., height_thigh / 60, 0.), M_PI_2, 0, M_PI_2, HIP_AP_L, HIP_AP_H, HIP_ML_L, HIP_ML_H);
		Create6DOF(JOINT_LEFT_KNEE, BODYPART_LEFT_THIGH, BODYPART_LEFT_SHANK, btVector3(0., -height_thigh / 60, 0.), btVector3(0., height_shank / 60, 0.), M_PI_2, 0, M_PI_2, KNEE_AP_L, KNEE_AP_H, KNEE_ML_L, KNEE_ML_H);
		Create6DOF(JOINT_RIGHT_KNEE, BODYPART_RIGHT_THIGH, BODYPART_RIGHT_SHANK, btVector3(0., -height_thigh / 60, 0.), btVector3(0., height_shank / 60, 0.), M_PI_2, 0, M_PI_2, KNEE_AP_L, KNEE_AP_H, KNEE_ML_L, KNEE_ML_H);
		Create6DOF(JOINT_LEFT_ANKLE, BODYPART_LEFT_SHANK, BODYPART_LEFT_FOOT, btVector3(0., -height_shank / 60, 0.), btVector3(0., length_foot / 120, 0.), M_PI_2, 0, M_PI_2, ANKL_AP_L, ANKL_AP_H, ANKL_ML_L, ANKL_ML_H);
		Create6DOF(JOINT_RIGHT_ANKLE, BODYPART_RIGHT_SHANK, BODYPART_RIGHT_FOOT, btVector3(0., -height_shank / 60, 0.), btVector3(0., length_foot / 120, 0.), M_PI_2, 0, M_PI_2, ANKL_AP_L, ANKL_AP_H, ANKL_ML_L, ANKL_ML_H);
		
#endif
#endif
		//FRICTION CTRL:
		m_bodies[BODYPART_LEFT_FOOT]->setFriction(7); 
		m_bodies[BODYPART_RIGHT_FOOT]->setFriction(7);
		m_bodies[BODYPART_PLATFORM]->setFriction(7);
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
		joint->enableFeedback(TRUE);
		joint->setJointFeedback(&fg);
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

	void save_1DbtV3(vector<btVector3> data, string filename)
	{
		ofstream saveFile;
		saveFile.open(filename, ios_base::app);
		saveFile << setprecision(4);
		for (unsigned i = 0; i < data.size(); i++)
		{
			saveFile << data.at(i).x() << " " << data.at(i).y() << " " << data.at(i).z() << " " << endl;
		}

		saveFile.close();
	}

	void save_1DInt(vector<int> data, string filename)
	{
		ofstream saveFile;
		saveFile.open(filename, ios_base::app);
		saveFile << setprecision(4);
		for (unsigned i = 0; i < data.size(); i++)
		{
			saveFile << data.at(i) << endl;
		}

		saveFile.close();
	}
	
	void save_2DDbl(vector<vector<double>> data, string filename)
	{
		ofstream saveFile;
		saveFile.open(filename, ios_base::app);
		saveFile << setprecision(4);
		for (unsigned i = 0; i < data.size(); i++)
		{
			for (unsigned j = 0; j < data.at(i).size(); j++)
			{
				saveFile << data.at(i).at(j) << " ";
			}
			saveFile << endl;
		}

		saveFile.close();
	}

	void save_2DbtV3(vector<vector<btVector3>> data, string filename)
	{
		ofstream saveFile;
		saveFile.open(filename, ios_base::app);
		saveFile << setprecision(4);
		for (unsigned i = 0; i < data.size(); i++)
		{
			for (unsigned j = 0; j < data.at(i).size(); j++)
			{
				saveFile << data.at(i).at(j).x() << " " << data.at(i).at(j).y() << " " << data.at(i).at(j).z() << " ";
			}
			saveFile << endl;
		}

		saveFile.close();
	}

	//// save one-dimensional vector of any type:
	//template <typename T>
	//void save_1DFile(T data, string filename)
	//{
	//	ofstream saveFile;
	//	saveFile.open(filename, ios_base::app);
	//	cout << "Saving into " << filename << endl;
	//	cout << "Vector size = " << data.size() << ", and sizeof(vector) = " << sizeof(data) << endl;
	//	getchar();
	//	saveFile << setprecision(4);
	//	for (unsigned i = 0; i < data.size(); i++)
	//	{
	//		cout << "Step = " << i << ". Value saved = " << data.at(i) << endl;
	//		cout << "Step = " << i << ". Alt value saved = " << (* data.at(i)) << endl;
	//		saveFile << data.at(i) << endl;
	//		getchar();
	//	}
	//	saveFile.close();
	//}
	//// save two-dimensional vector of any type:
	//template <typename T>
	//void save_2DFile(T data, string filename)
	//{
	//	ofstream saveFile;
	//	saveFile.open(filename, ios_base::app);
	//	saveFile << setprecision(4);
	//	for (unsigned i = 0; i < data.size(); i++)
	//	{
	//		for (unsigned j = 0; j < data.at(i).size(); j++)
	//		{
	//			saveFile << data.at(i).at(j) << " ";
	//		}
	//		
	//		saveFile << endl;
	//	}

	//	saveFile.close();
	//}

	//for saving just one double value into a text file
	void save_1by1_file(double data, string fileName)
	{
		ofstream outputFile;
		outputFile.open(fileName, ios_base::app);
		outputFile << data << endl;
		outputFile.close();
	}

	// GET CoM POSITION of a body part
	btVector3 getCOMposition(int bodyIndex)
	{
		btRigidBody * pointer = m_bodies[bodyIndex];
		btVector3 position = pointer->getCenterOfMassPosition();
		return position;
		
	}
	// Get COM position of the whole body
	btVector3 wholeBodyCOM()
	{
		double thisMass = 0;
		double sumMass = 0;
		btScalar COMcoordX = 0;
		btScalar COMcoordY = 0;
		btScalar COMcoordZ = 0;
		//cout << "Foot length = " << length_foot / 30 << ", width = " << length_foot / 45 << ", height = " << length_foot / 60 << endl;
		//last "body part" is the platform, which should be discounted
		for (int i = 0; i < BODYPART_COUNT-1; i++)
		{
			btRigidBody * pointer = m_bodies[i];
			btVector3 bodySegCOM = pointer->getCenterOfMassPosition();

			thisMass =  1 / (pointer->getInvMass());
			sumMass += 1 / (pointer->getInvMass());
			//cout << "Body = " << i << ", x = " << bodySegCOM.x() << ", y = " << bodySegCOM.y() << ", z = " << bodySegCOM.z() << ", mass = " << thisMass << endl;
			//cout << "Running summed mass = " << sumMass << endl;
			COMcoordX += bodySegCOM.x() * thisMass;
			COMcoordY += bodySegCOM.y() * thisMass;
			COMcoordZ += bodySegCOM.z() * thisMass;
		}
		//cout << "SUM.x = " << COMcoordX << ", SUM.y = " << COMcoordY << ", SUM.z = " << COMcoordZ << ", Summed mass = " << sumMass << endl;
		COMcoordX = COMcoordX / sumMass;
		COMcoordY = COMcoordY / sumMass;
		COMcoordZ = COMcoordZ / sumMass;
		btVector3 wholeCOM = btVector3(COMcoordX, COMcoordY, COMcoordZ);
		return wholeCOM;
	}

	vector<btVector3> getTargPos()
	{
		double initPelvisHeight = 0.3 + length_foot / 60 + height_leg / 30;
		btScalar leftTargZ = initPelvisHeight / 1.5; //step length = 1/3 of body height, just like in humans
		btScalar leftTargY = 0.3 + length_foot / 120;
		btScalar leftTargX = height_pelvis / 60;//

		btScalar rightTargZ = 0; //step length = 1/3 of body height, just like in humans
		btScalar rightTargY = 0.3 + length_foot / 120;
		btScalar rightTargX = -height_pelvis / 60;//
		vector<btVector3> targPos;
		targPos.push_back(btVector3(leftTargX, leftTargY, leftTargZ));
		targPos.push_back(btVector3(rightTargX, rightTargY, rightTargZ));
		return targPos;
	}

	double onlineFitness(int SimulationStep, int maxStep)
	{
		btRigidBody * p_swingFoot;
		btRigidBody * p_stanceFoot;
		btGeneric6DofConstraint * p_stanceFootAnkleJoint;

		double initPelvisHeight = 0.3 + length_foot / 60 + height_leg / 30;
		double swingTargX;
		double swingTargY = 0.3 + length_foot / 120; // initial feet COM height
		double swingTargZ = initPelvisHeight / 1.5;
		double swingFootInitZ = 0;
		double stanceTargX;
		double stanceTargY = 0.3 + length_foot / 120;
		double stanceTargZ;

		if (SimulationStep < maxStep / 2)
		{
			//cout << "Since SimStep " << SimulationStep << " < maxStep/2 " << maxStep / 2 << ", I am using FitFcn 1 (first half of simulation)." << endl;
			p_swingFoot = m_bodies[BODYPART_LEFT_FOOT];
			p_stanceFoot = m_bodies[BODYPART_RIGHT_FOOT];
			p_stanceFootAnkleJoint = m_joints[JOINT_RIGHT_ANKLE];
			swingTargX = height_pelvis / 60;
			stanceTargX = - height_pelvis / 60;
			stanceTargZ = 0;
		}
		else
		{
			//cout << "Since SimStep " << SimulationStep << " > maxStep/2 " << maxStep / 2 << ", I am using FitFcn 2 (second half of simulation)." << endl;
			p_swingFoot = m_bodies[BODYPART_RIGHT_FOOT];
			p_stanceFoot = m_bodies[BODYPART_LEFT_FOOT];
			p_stanceFootAnkleJoint = m_joints[JOINT_LEFT_ANKLE];
			swingTargX =  - height_pelvis / 60;
			stanceTargX = height_pelvis / 60;
			stanceTargZ = initPelvisHeight / 1.5;
		}

		btRigidBody * p_pelvis = m_bodies[BODYPART_PELVIS];
		
		// Get COM positions:
		btVector3 swingFootPos = p_swingFoot->getCenterOfMassPosition();
		btVector3 stanceFootPos = p_stanceFoot->getCenterOfMassPosition();
		btVector3 pelPos = p_pelvis->getCenterOfMassPosition();

		//get rotation of pelvis around the Y-axis:
		btScalar pelRot, junk1, junk2;
		p_pelvis->getCenterOfMassTransform().getBasis().getEulerZYX(junk1, pelRot, junk2);
		//get joint angles for ankles
		double stanceFootAnkleAPRot = p_stanceFootAnkleJoint->getRotationalLimitMotor(2)->m_currentPosition;
		double stanceFootAnkleMLRot = p_stanceFootAnkleJoint->getRotationalLimitMotor(1)->m_currentPosition;

		//values to compare online values of rotation:
		double initPelvRot = 0;
		double initAnklRotAP = 0;
		double initAnklRotML = 0;
		
		//Ensure that only forward swing foot movement is rewarded:
		if (swingFootPos.z() < 0) swingFootPos = btVector3(swingFootPos.x(), swingFootPos.y(), btScalar(0));
		if (pelPos.y() > initPelvisHeight) pelPos = btVector3(pelPos.x(), initPelvisHeight, pelPos.z());
		// Fitness members:
		double pelvRotMember = 1 / (1 + abs(initPelvRot - pelRot)); //cout << "pelvRotMember = " << pelvRotMember << endl;
		//double stanceAnklRotMemberAP = 1 / (1 + abs(initAnklRotAP - stanceFootAnkleAPRot)); //cout << "stanceAnklRotMemberAP = " << stanceAnklRotMemberAP << endl;
		//double stanceAnklRotMemberML = 1 / (1 + abs(initAnklRotML - stanceFootAnkleMLRot)); //cout << "stanceAnklRotMemberML = " << stanceAnklRotMemberML << endl;
		double targetMemberX = 1 / (1 + abs(swingTargX - swingFootPos.x()));	//cout << "targetMemberX = " << targetMemberX << endl;
		double targetMemberY = 1 / (1 + abs(swingTargY - swingFootPos.y()));
		double targetMemberZ = 1 / (1 + abs( (swingTargZ - swingFootPos.z()) / (swingFootPos.z() - swingFootInitZ) )); //cout << "targetMemberZ = " << targetMemberZ << endl;
		double stanceMemberX = 1 / (1 + abs(stanceTargX - stanceFootPos.x())); //cout << "stanceMemberX = " << stanceMemberX << endl;
		double stanceMemberY = 1 / (1 + abs(stanceTargY - stanceFootPos.y()));
		double stanceMemberZ = 1 / (1 + abs(stanceTargZ - stanceFootPos.z())); //cout << "stanceMemberZ = " << stanceMemberZ << endl;
		double pelvHeightMember = 1 / (1 + abs( (initPelvisHeight - pelPos.y())  )); //cout << "pelvHeightMember = " << pelvHeightMember << endl;
		//double result = pelvRotMember * pelvHeightMember * stanceAnklRotMemberAP * stanceAnklRotMemberML * targetMemberX * targetMemberY * targetMemberZ * stanceMemberX * stanceMemberY * stanceMemberZ;
		double result = pelvRotMember * pelvHeightMember * targetMemberX * targetMemberY * targetMemberZ * stanceMemberX * stanceMemberY * stanceMemberZ;
		//cout << "swingTargZ = " << swingTargZ << ", swingFootPos.z() = " << swingFootPos.z() << ", swingFootInitZ = " << swingFootInitZ << endl;
		//cout << " Fitness of this SimStep is = " << result << endl;
		//cout << "---------------------------------" << endl;
		return result;

	}
	
	// detects if the pelvis is fallign below certain height, exits simulation, and saves accumulated fitness up to that simulation step
	void isUpright(double tempFitness, int maxStep, int SimulationStep)
	{
		btRigidBody * pelvis = m_bodies[BODYPART_PELVIS];
		btVector3 pelPos = pelvis->getCenterOfMassPosition();
		double avgTempFitness;

		if (pelPos.y() < PELV_HEIGHT * 0.75)
		{
			//avgTempFitness = tempFitness / maxStep;
			avgTempFitness = tempFitness;
			//save_1by1_file<double>(avgTempFitness, "fit.txt");
			save_1by1_file(avgTempFitness,"fit.txt");
#ifndef TRAIN
			cout << "Exit early due to fall. Curr height: " << pelPos.y() << " < 75% of Init height " << PELV_HEIGHT << " = " << PELV_HEIGHT*0.75 << endl;
			cout << "Sim.Step: " << SimulationStep << " out of " << maxStep << ". tempFitness = " << tempFitness << ", avgTempFitness = " << avgTempFitness << endl;
			getchar();
#endif
			exit(0);
		}
		return;
	}

	
}; //END OF RagDoll CLASS

// Alternative version of collision detection from:
// http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=2568&hilit=contact+info
// after experimenting figuren out - it does the same.
//static RagdollDemo * ragdollDemo; //for processing touches
//void myTickCallback(btDynamicsWorld *world, btScalar timeStep) {
//	int
//		* ID1, *ID2;
//
//	if (world)
//		world->performDiscreteCollisionDetection();
//	int i;
//	///one way to draw all the contact points is iterating over contact manifolds / points:
//	int numManifolds = world->getDispatcher()->getNumManifolds();
//	//cout << "CP# = " << numManifolds << endl;
//	for (i = 0; i<numManifolds; i++)
//	{
//		btPersistentManifold* contactManifold = world->getDispatcher()->getManifoldByIndexInternal(i);
//		btCollisionObject* obA = static_cast<btCollisionObject*>((void*)contactManifold->getBody0());
//		btCollisionObject* obB = static_cast<btCollisionObject*>((void*)contactManifold->getBody1());
//
//		ID1 = static_cast<int * >(obA->getUserPointer());
//		ID2 = static_cast<int * >(obB->getUserPointer());
//
//		int numContacts = contactManifold->getNumContacts();
//		for (int j = 0; j<numContacts; j++)
//		{
//			btManifoldPoint& contactPoint = contactManifold->getContactPoint(j);
//			btVector3 normal = contactPoint.m_normalWorldOnB;
//			btScalar angleX = normal.angle(btVector3(1, 0, 0));
//			btScalar angleY = normal.angle(btVector3(0, 1, 0));
//			btScalar angleZ = normal.angle(btVector3(0, 0, 1));
//			btScalar impulseX = contactPoint.m_appliedImpulse*cos(angleX);
//			btScalar impulseY = contactPoint.m_appliedImpulse*cos(angleY);
//			btScalar impulseZ = contactPoint.m_appliedImpulse*cos(angleZ);
//			btScalar forceX = impulseX / (timeStep);
//			btScalar forceY = impulseY / (timeStep);
//			btScalar forceZ = impulseZ / (timeStep);
//
//			ragdollDemo->touches[*ID1] = 1;
//			ragdollDemo->touches[*ID2] = 1;
//			ragdollDemo->touchPoints[*ID1] = contactPoint.m_positionWorldOnB;
//			ragdollDemo->touchPoints[*ID2] = contactPoint.m_positionWorldOnB;
//
//			ragdollDemo->forces[*ID1] = btVector3(impulseX, impulseY, impulseZ);
//			ragdollDemo->forces[*ID2] = btVector3(impulseX, impulseY, impulseZ);
//
//			//if (!(*ID1 == 8 && *ID2 == 7))
//			//{
//			//	printf("ID1 = %d, ID2 = %d\n", *ID1, *ID2);
//			//	printf("Force: %8.10f %8.10f %8.10f \n", impulseX, impulseY, impulseZ);
//			//}
//		}
//	}
//}
//
//
//// BINARY TOUCH SENSOR SYSTEM:
static RagdollDemo * ragdollDemo; //for processing touches
//// detects if two bodies are in contact and assigns 1's to respective values in touches[number of body parts + 1 for ground]
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
#ifndef KNEES
	int groundID = 6;
#else
	int groundID = 8;
#endif
#endif
	ID1 = static_cast<int * >(o1->getUserPointer());
	ID2 = static_cast<int * >(o2->getUserPointer());

	//printf("ID1 = %d, ID2 = %d\n",*ID1,*ID2); //Technical line to check if the registration of collisions works

	ragdollDemo->touches[*ID1] = 1;
	ragdollDemo->touches[*ID2] = 1;
	ragdollDemo->touchPoints[*ID1] = cp.m_positionWorldOnB;
	ragdollDemo->touchPoints[*ID2] = cp.m_positionWorldOnB;
	
	btVector3 normal = cp.m_normalWorldOnB;

	btScalar angleX = normal.angle(btVector3(1, 0, 0));
	btScalar angleY = normal.angle(btVector3(0, 1, 0));
	btScalar angleZ = normal.angle(btVector3(0, 0, 1));

	btScalar impulseX = cp.m_appliedImpulse*cos(angleX);
	btScalar impulseY = cp.m_appliedImpulse*cos(angleY);
	btScalar impulseZ = cp.m_appliedImpulse*cos(angleZ);

	btScalar timeStep = 1.f / 60.f;

	btScalar forceX = impulseX / timeStep;
	btScalar forceY = impulseY / timeStep;
	btScalar forceZ = impulseZ / timeStep;

	ragdollDemo->forces[*ID1] = btVector3(forceX,forceY,forceZ);
	ragdollDemo->forces[*ID2] = btVector3(forceX, forceY, forceZ);
	//cout << "Appl. Force on body " << *ID1 << " = " << ragdollDemo->forces[*ID1].x()<< " , " << ragdollDemo->forces[*ID1].y() << " , " << ragdollDemo->forces[*ID1].z() << " ) " << endl;
	//cout << "Appl. Force on body " << *ID2 << " = " << ragdollDemo->forces[*ID2].x() << " , " << ragdollDemo->forces[*ID2].y() << " , " << ragdollDemo->forces[*ID2].z() << " ) " << endl;
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
#ifdef EV_TAU
, m_inputTauFileName("tau.txt")
//{}
#endif
#ifdef TAU_SWITCH
, m_inputTauFileName("tau.txt")
#endif

{}
// execute reading:
void RagdollDemo::initParams(const std::string& inputFileName)
{
	if (!inputFileName.empty())
		m_inputFileName = inputFileName;
}

//READ TAU
#ifdef EV_TAU
void RagdollDemo::initParams1(const std::string& inputFileName)
{
	if (!inputFileName.empty())
		m_inputTauFileName = inputFileName;
}
#endif

#ifdef TAU_SWITCH
void RagdollDemo::initParams1(const std::string& inputFileName)
{
	if (!inputFileName.empty())
		m_inputTauFileName = inputFileName;
}
#endif

#ifdef TAU_SWITCH
vector<vector<double>> RagdollDemo::flipTau(vector<vector<double>> tau)
{
	vector<vector<double>> tempTau;
	//initialize tempTau structure:
	for (unsigned i = 0; i < tau.size(); i++)
	{
		tempTau.push_back(vector<double>());
		for (unsigned j = 0; j < tau[i].size(); j++)
		{
			tempTau[i].push_back(double());
		}
	}
	//populate tempTau with reversed data:
	//// input layer remains the same:
	//tempTau[0][0] = tau[0][0];
	//tempTau[0][1] = tau[0][1];
	// input layer reverses too:
	tempTau[0][0] = tau[0][1];
	tempTau[0][1] = tau[0][0];
	// flip hidden neurons' tau:
	tempTau[1][0]=tau[1][1];
	tempTau[1][1]=tau[1][0];
	// flip output neurons
	//cout << "tau[2].size is " << tau[2].size() << endl;
	for (unsigned k = 0; k < tau[2].size(); k=k+4)
	{
		//cout << "k = " << k << endl;
		tempTau[2][k] = tau[2][k + 2];
		tempTau[2][k+1] = tau[2][(k + 1) + 2];
		
	}

	for (unsigned m = 2; m < tau[2].size(); m = m + 4)
	{
		//cout << "m = " << m << endl;
		tempTau[2][m] = tau[2][m - 2];
		tempTau[2][m + 1] = tau[2][(m + 1) - 2];

	}

//#ifndef TRAIN
//	//to debug the flip:
//	for (unsigned i = 0; i < tau.size(); i++)
//	{
//		cout << "Layer " << i << " of original tau: ";
//		for (unsigned j = 0; j < tau[i].size(); j++)
//		{
//			cout << tau[i][j] << " ";
//		}
//		cout << endl;
//
//		cout << "Layer " << i << " of flipped tau: ";
//		for (unsigned j = 0; j < tau[i].size(); j++)
//		{
//			cout << tempTau[i][j] << " ";
//		}
//		cout << endl;
//	}
//#endif

	return tempTau;

}
#endif

//CTRNN CODE:
//========================================================================================================
#ifdef EV_TAU
vector<vector<double>> RagdollDemo::euler(double neural_step, double h, vector<vector<double>> tau, vector<vector<double>> w, vector<vector<double>> neuron_vals, vector<vector<double>> bias, vector<int> touches, vector<vector<double>> gain)
#else
#ifdef TAU_SWITCH
// (update function, initial value, time1, time2, step size, time constant)
//vector<vector<double>> RagdollDemo::euler(double neural_step, double h, double tau, vector<vector<double>> w, vector<vector<double>> neuron_vals, vector<vector<double>> bias, vector<int> touches, vector<vector<double>> gain)
vector<vector<double>> RagdollDemo::euler(double neural_step, double h, vector<vector<double>> tau, vector<vector<double>> w, vector<vector<double>> neuron_vals, vector<vector<double>> bias, vector<int> touches, vector<vector<double>> gain)
#else
vector<vector<double>> RagdollDemo::euler(double neural_step, double h, vector<double> tau, vector<vector<double>> w, vector<vector<double>> neuron_vals, vector<vector<double>> bias, vector<int> touches, vector<vector<double>> gain)
#endif
#endif
{
	// Separate weight matrices:
	vector<double> temp_row;
	vector<vector<double>> wIJ, wJK;

	//find the max positive value and min negative value to cale weight matrix to [-1; 1]:
	double maxPos = w[0][0];
	double minNeg = w[0][0];

	for (unsigned inx1 = 0; inx1 < w.size(); inx1++)
	{
		for (unsigned inx2 = 0; inx2 < w[inx1].size(); inx2++)
		{
			//cout << "Accessing w[" << inx1 << "," << inx2 << "] = " << w[inx1][inx2] << endl;
			if (w[inx1][inx2] < minNeg) { minNeg = w[inx1][inx2]; }//cout << "Saving minNeg = " << w[inx1][inx2] << endl; }
			if (w[inx1][inx2] > maxPos) { maxPos = w[inx1][inx2]; }//cout << "Saving maxPos = " << w[inx1][inx2] << endl;
		
		}
		//cout << "maxPos = " << maxPos << ", minNeg = " << minNeg << endl;
	}
	double absMax;
	if (maxPos > abs(minNeg)) { absMax = maxPos; }
	else { absMax = abs(minNeg); }


	// populate the wIJ matrix
	//cout << "wIJ = " << endl;
	for (unsigned i = 0; i < w[0].size(); i++)
	{
		for (int j = 0; j < num_input; j++)
		{
#ifdef WTS_NORM
			double normVal;
			if (w[j][i]>0) { normVal = w[j][i] / maxPos; }
			else { normVal = w[j][i] / abs(minNeg); }
			//cout << " w[" << j << ", " << i << "] = " << w[j][i] << ", norm_w[" << j << ", " << i << "] = " << normVal << "; ";
			//t
			temp_row.push_back(normVal);
#else
			temp_row.push_back(w[j][i]);
#endif
		}
		//cout << endl;
		wIJ.push_back(temp_row);
		temp_row.clear();
	}
	
	//populate the wJK matrix
	//cout << "wJK = " << endl;
	for (int j = 0; j < num_output; j++)
	{
		for (int i = num_input; i < num_hidden + num_input; i++)
		{
#ifdef WTS_NORM
			double normVal;
			if (w[i][j]>0) { normVal = w[i][j] / maxPos; }
			else { normVal = w[i][j] / abs(minNeg); }
			//cout << " w[" << i << ", " << j << "] = " << w[i][j] << ", norm_w[" << i << ", " << j << "] = " << normVal << "; ";
			//temp_row.push_back(w[i][j]);
			temp_row.push_back(normVal);
#else
			temp_row.push_back(w[i][j]);
#endif
		}
		//cout << endl;
		wJK.push_back(temp_row);
		temp_row.clear();
		//getchar();
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

	// pad misc values due to use of CPG:
	input_bias.push_back(bias_val);
	hidden_sensors.push_back(0.);
	gain[0].push_back(gain_val);

	//Main cycle of neuronal values calculation
	// Updates all neuronal values for allotted time "neural step" using step size "h": 
	for (int t = 0; t < (neural_step/h); t++)
	{
		for (unsigned i = 0; i < neuron_vals.at(0).size(); i++)
		{
			// Here, weight matrix wIJ is used (which is wrong, since there are no weights from zero fictional layer)
			// but since the zero layer has only zeros, weights don't matter. Similarly, gains are for calculating the 
			// summed influence of all neurons from previous layer, and thus gains are parsed from the previous layer:
#ifdef EV_TAU
			input.at(i) = input.at(i) + h * updateNeuron(tau[0][i], zeros, input.at(i), wIJ[i], zeros, touches.at(i), gain.at(0));
#else
#ifdef TAU_SWITCH
			//cout << "Input neuron #" << i << ", using tau[0][" << i << "] = " << tau[0][i] << endl;
			input.at(i) = input.at(i) + h * updateNeuron(tau[0][i], zeros, input.at(i), wIJ[i], zeros, touches.at(i), gain.at(0));
#else
			input.at(i) = input.at(i) + h * updateNeuron(tau[0], zeros, input.at(i), wIJ[i], zeros, touches.at(i), gain.at(0));
#endif
#endif

		}
		vector<double> currHidden = hidden;
		//Cycle to go through all hidden nerons
		for (unsigned i = 0; i < neuron_vals.at(1).size(); i++)
		{
			//pad input layer with current states of hidden neurons:
			vector<double> inputLayer = input;
			// pad weight matrix with -1 weights:
			vector<double> wIJpadded = wIJ[i];
#ifdef CPG
			inputLayer.push_back(currHidden[(int)cos(M_PI_2*i)]);// add current state of the other hidden neuron
			
#ifdef WTS_NORM
			wIJpadded.push_back(-1.0);	
#else
			wIJpadded.push_back(-absMax);
#endif
#else
			wIJpadded.push_back(0.0);
#endif
			//cout << "EULER. Int. tick #" << t << ", hidden neur[" << i << "] = " << hidden.at(i) << " BEFORE" << endl;
#ifdef EV_TAU
			hidden.at(i) = hidden.at(i) + h * updateNeuron(tau[1][i], inputLayer, hidden.at(i), wIJpadded, input_bias, hidden_sensors.at(i), gain.at(0));
#else
#ifdef TAU_SWITCH
			//cout << "Hidden neuron #" << i << ", using tau[1][" << i << "] = " << tau[1][i] << endl;
			hidden.at(i) = hidden.at(i) + h * updateNeuron(tau[1][i], inputLayer, hidden.at(i), wIJpadded, input_bias, hidden_sensors.at(i), gain.at(0));
#else
			//hidden.at(i) += h * updateNeuron(tau, neuron_vals.at(0), hidden.at(i), wIJ.at(i), input_bias, hidden_sensors.at(i), gain.at(0));
			hidden.at(i) = hidden.at(i) + h * updateNeuron(tau[1], inputLayer, hidden.at(i), wIJpadded, input_bias, hidden_sensors.at(i), gain.at(0));
#endif
#endif
			//cout << "EULER. Int. tick #" << t << ", hidden neur[" << i << "] = " << hidden.at(i) << " AFTER" << endl;
		}
		//Cycle to go through all output nerons
		for (unsigned i = 0; i < neuron_vals.at(2).size(); i++)
		{
#ifdef EV_TAU
			output.at(i) += h * updateNeuron(tau[2][i], hidden, output.at(i), wJK.at(i), hidden_bias, output_sensors.at(i), gain.at(1));
#else
#ifdef TAU_SWITCH
			//cout << "Output neuron #" << i << ", using tau[2][" << i << "] = " << tau[2][i] << endl;
			output.at(i) = output.at(i) + h * updateNeuron(tau[2][i], hidden, output.at(i), wJK.at(i), hidden_bias, output_sensors.at(i), gain.at(1));
#else
			output.at(i) += h * updateNeuron(tau[2], hidden, output.at(i), wJK.at(i), hidden_bias, output_sensors.at(i), gain.at(1));
#endif
#endif
		}
	}//End MAIN cycle (through time)

	//update neuronal values:
	neuron_vals.clear(); neuron_vals.push_back(input); neuron_vals.push_back(hidden); neuron_vals.push_back(output);
	return neuron_vals;
}

#ifdef EV_TAU
vector<vector<vector<double>>> RagdollDemo::eulerEXPORT(double neural_step, double h, vector<vector<double>> tau, vector<vector<double>> w, vector<vector<double>> neuron_vals, vector<vector<double>> bias, vector<int> touches, vector<vector<double>> gain)
#else
#ifdef TAU_SWITCH
// (update function, initial value, time1, time2, step size, time constant)
//vector<vector<double>> RagdollDemo::euler(double neural_step, double h, double tau, vector<vector<double>> w, vector<vector<double>> neuron_vals, vector<vector<double>> bias, vector<int> touches, vector<vector<double>> gain)
vector<vector<vector<double>>> RagdollDemo::eulerEXPORT(double neural_step, double h, vector<vector<double>> tau, vector<vector<double>> w, vector<vector<double>> neuron_vals, vector<vector<double>> bias, vector<int> touches, vector<vector<double>> gain)
#else
vector<vector<vector<double>>> RagdollDemo::eulerEXPORT(double neural_step, double h, vector<double> tau, vector<vector<double>> w, vector<vector<double>> neuron_vals, vector<vector<double>> bias, vector<int> touches, vector<vector<double>> gain)
#endif
#endif
{
	// create an inner 3d vector for internal states:
	vector<vector<vector<double>>> intNeuronVals;
	for (int i = 0; i < (neural_step / h + 1); i++)
	{
		intNeuronVals.push_back(vector<vector<double>>());
		for (int j = 0; j < 3; j++)
		{
			intNeuronVals[i].push_back(vector<double>());
			for (int k = 0; k < neuron_val[j].size(); k++)
			{
				intNeuronVals[i][j].push_back(double());
			}
		}
	}
	//copy the inputted neuronal states into the first time step:
	for (int j = 0; j < 3; j++)
	{
		for (int k = 0; k < neuron_val[j].size(); k++)
		{
			intNeuronVals[0][j][k] = neuron_val[j][k];
			//cout << "Using " << neuron_val[j][k] << " as 0-th value for intNeuronVals[" << j << "][" << k << "]" << endl;
		}
	}

	// Separate weight matrices:
	vector<double> temp_row;
	vector<vector<double>> wIJ, wJK;

	//find the max positive value and min negative value to cale weight matrix to [-1; 1]:
	double maxPos = w[0][0];
	double minNeg = w[0][0];

	for (unsigned inx1 = 0; inx1 < w.size(); inx1++)
	{
		for (unsigned inx2 = 0; inx2 < w[inx1].size(); inx2++)
		{
			//cout << "Accessing w[" << inx1 << "," << inx2 << "] = " << w[inx1][inx2] << endl;
			if (w[inx1][inx2] < minNeg) { minNeg = w[inx1][inx2]; }//cout << "Saving minNeg = " << w[inx1][inx2] << endl; }
			if (w[inx1][inx2] > maxPos) { maxPos = w[inx1][inx2]; }//cout << "Saving maxPos = " << w[inx1][inx2] << endl;

		}
	}
	double absMax;
	if (maxPos > abs(minNeg)) { absMax = maxPos; }
	else { absMax = abs(minNeg); }


	// populate the wIJ matrix
	for (unsigned i = 0; i < w[0].size(); i++)
	{
		for (int j = 0; j < num_input; j++)
		{
#ifdef WTS_NORM
			double normVal;
			if (w[j][i] > 0) { normVal = w[j][i] / maxPos; }
			else { normVal = w[j][i] / abs(minNeg); }
			//cout << " w[" << j << ", " << i << "] = " << w[j][i] << ", norm_w[" << j << ", " << i << "] = " << normVal << "; ";
			//t
			temp_row.push_back(normVal);
#else
			temp_row.push_back(w[j][i]);
#endif
		}
		//cout << endl;
		wIJ.push_back(temp_row);
		temp_row.clear();
	}

	//populate the wJK matrix
	for (int j = 0; j < num_output; j++)
	{
		for (int i = num_input; i < num_hidden + num_input; i++)
		{
#ifdef WTS_NORM
			double normVal;
			if (w[i][j] > 0) { normVal = w[i][j] / maxPos; }
			else { normVal = w[i][j] / abs(minNeg); }
			//cout << " w[" << i << ", " << j << "] = " << w[i][j] << ", norm_w[" << i << ", " << j << "] = " << normVal << "; ";
			//temp_row.push_back(w[i][j]);
			temp_row.push_back(normVal);
#else
			temp_row.push_back(w[i][j]);
#endif
		}
		wJK.push_back(temp_row);
		temp_row.clear();
	}

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
	vector<double> zeros = { 0.,0. };

	//create zero vectors to put for absent sensor values for hidden and output layers calculations:
	vector<double> hidden_sensors;
	vector<double> output_sensors;

	//populate sensor vectors for hid/out layers with zeros (don't receive sensor inputs):
	for (int i = 0; i < num_hidden; i++) { hidden_sensors.push_back(0); }
	for (int i = 0; i < num_output; i++) { output_sensors.push_back(0); }

	//Main cycle of neuronal values calculation
	// Updates all neuronal values for allotted time "neural step" using step size "h": 

	// pad misc values due to use of CPG connections:
	input_bias.push_back(bias_val);
	hidden_sensors.push_back(0.);
	for (int t = 1; t < (neural_step / h + 1); t++)
	{
		for (unsigned i = 0; i < neuron_vals.at(0).size(); i++)
		{
			// Here, weight matrix wIJ is used (which is wrong, since there are no weights from zero fictional layer)
			// but since the zero layer has only zeros, weights don't matter. Similarly, gains are for calculating the 
			// summed influence of all neurons from previous layer, and thus gains are parsed from the previous layer:
#ifdef EV_TAU
			intNeuronVals[t][0][i] = intNeuronVals[t - 1][0][i] + h * updateNeuron(tau[0][i], zeros, intNeuronVals[t - 1][0][i], wIJ[i], zeros, touches.at(i), gain.at(0));
#else
#ifdef TAU_SWITCH
			intNeuronVals[t][0][i] = intNeuronVals[t - 1][0][i] + h * updateNeuron(tau[0][i], zeros, intNeuronVals[t - 1][0][i], wIJ[i], zeros, touches.at(i), gain.at(0));
#else
			intNeuronVals[t][0][i] = intNeuronVals[t - 1][0][i] + h * updateNeuron(tau[0], zeros, intNeuronVals[t - 1][0][i], wIJ[i], zeros, touches.at(i), gain.at(0));
#endif
#endif
		}
		vector<double> currHidden = intNeuronVals[t-1][1];
		//Cycle to go through all hidden nerons
		for (unsigned i = 0; i < neuron_vals.at(1).size(); i++)
		{
			//pad input layer with current states of hidden neurons:
			vector<double> inputLayer = intNeuronVals[t][0];
			vector<double> wIJpadded = wIJ[i];
#ifdef CPG
			inputLayer.push_back(currHidden[(int)cos(M_PI_2*i)]);// add current state of the other hidden neuron
#ifdef WTS_NORM
			wIJpadded.push_back(-1.0);// pad weight matrix with -1 weights:
#else
			wIJpadded.push_back(-absMax);
#endif
#else
			wIJpadded.push_back(0.0);
#endif
			gain[0].push_back(gain_val);
#ifdef EV_TAU
			intNeuronVals[t][1][i] = intNeuronVals[t - 1][1][i] + h * updateNeuron(tau[1][i], inputLayer, intNeuronVals[t - 1][1][i], wIJpadded, input_bias, hidden_sensors.at(i), gain.at(0));
#else
#ifdef TAU_SWITCH
			intNeuronVals[t][1][i] = intNeuronVals[t - 1][1][i] + h * updateNeuron(tau[1][i], inputLayer, intNeuronVals[t - 1][1][i], wIJpadded, input_bias, hidden_sensors.at(i), gain.at(0));
#else
			//hidden.at(i) += h * updateNeuron(tau, neuron_vals.at(0), hidden.at(i), wIJ.at(i), input_bias, hidden_sensors.at(i), gain.at(0));
			intNeuronVals[t][1][i] = intNeuronVals[t - 1][1][i] + h * updateNeuron(tau[1], inputLayer, intNeuronVals[t - 1][1][i], wIJpadded, input_bias, hidden_sensors.at(i), gain.at(0));
#endif
#endif
		}
		//Cycle to go through all output nerons
		for (unsigned i = 0; i < neuron_vals.at(2).size(); i++)
		{
#ifdef EV_TAU
			intNeuronVals[t][2][i] = intNeuronVals[t - 1][2][i] + h * updateNeuron(tau[2][i], intNeuronVals[t][1], intNeuronVals[t - 1][2][i], wJK.at(i), hidden_bias, output_sensors.at(i), gain.at(1));
#else
#ifdef TAU_SWITCH
			intNeuronVals[t][2][i] = intNeuronVals[t - 1][2][i] + h * updateNeuron(tau[2][i], intNeuronVals[t][1], intNeuronVals[t - 1][2][i], wJK.at(i), hidden_bias, output_sensors.at(i), gain.at(1));
#else
			intNeuronVals[t][2][i] = intNeuronVals[t - 1][2][i] + h * updateNeuron(tau[2], intNeuronVals[t][1], intNeuronVals[t - 1][2][i], wJK.at(i), hidden_bias, output_sensors.at(i), gain.at(1));
#endif
#endif
		}
		//		cout << "=========END OUTPUT================" << endl;
		//		cout << "Finished int.tick #" << t << endl;
		//		getchar();
	}//End MAIN cycle (through time)
	
	//erase the 1st member of the vector (previous neuronal values)
	intNeuronVals.erase(intNeuronVals.begin());
	return intNeuronVals;
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

double RagdollDemo::updateNeuronDEBUG(double tau, vector<double> previous_layer, double current_neuron, vector<double> w, vector<double> bias_val, int sensor_val, vector<double> gain)
{
	//var to store all of the summed input to the neuron:	
	double sum_inputs = 0;
	double fcn_val;
	cout << "---------INSIDE UPDATE NEURON------------" << endl;
	cout << "tau = " << tau << ", previous layer size " << previous_layer.size();
	//cycle through all neurons of previous layer:
	for (unsigned j = 0; j < previous_layer.size(); j++)
	{
		cout << "w[" << j << "] = " << w[j] << " * tanh[prev.layer[" << j << "] = " << previous_layer.at(j) << " + bias_val[" << j << "] = " << bias_val[j] << ")*gain[" << j << "] = " << gain[j] << "]) = " << tanh((previous_layer.at(j) + bias_val[j]) * gain.at(j)) * w[j] << endl;
		sum_inputs += w[j] * tanh((previous_layer.at(j) + bias_val[j]) * gain.at(j));
		cout << "sum_inputs = " << sum_inputs << endl;
	}
	//update neuron's state
	fcn_val = (-current_neuron + sensor_val + sum_inputs)*(1 / tau);
	cout << "fcn_val = [( -current_neuron = " << -current_neuron << " + sensor val = " << sensor_val << " + sum_inputs = " << sum_inputs << ")*(1/tau) = " << (1 / tau) << "] = " << fcn_val << endl;
	//	cout << "Result = " << fcn_val << endl;
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
#ifndef EV_TAU
#ifndef TAU_SWITCH
	tau.push_back(INPUT_TAU); 
	tau.push_back(HIDDEN_TAU);
	tau.push_back(OUTPUT_TAU);
#endif
#endif

#ifdef TRAIN
	pause = false;// should be false
#else
	pause = true;//should be true
#endif
	oneStep = false;

//Override pause setting if it's a COM file:
#ifdef COM
	pause = false;
#endif

//---- intializing bodypart IDs
	for (int i = 0; i < BODYPART_COUNT+1; i++)
	{
		IDs[i] = i;
	};

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

//if there is need for extracting neuronal states:
#ifdef NEURON

	//intialize 3d [timeStep X layer X neuron] vector for neuron states:
	for (int i = 0; i < (maxStep * (neural_step / h)); i++)
	{
		neuronHist.push_back(vector<vector<double>>());
		for (int j = 0; j < 3; j++)
		{
			neuronHist[i].push_back(vector<double>());
			for (int k = 0; k < neuron_val[j].size(); k++)
			{
				neuronHist[i][j].push_back(double());
			}
		}
	}
	cout << "neuronHist [ " << neuronHist.size() << " , " << neuronHist[0].size() << " , " << neuronHist[0][0].size() << " - " << neuronHist[0][1].size() << " - " << neuronHist[0][2].size() << "]" << endl;
#endif

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

#ifdef COM
	//intialize 2d vector for joint angls and forces [number of joints X number of time steps]:
	for (int i = 0; i < num_output; i++)
	{
		jointAngs.push_back(vector<double>());
		jointForces.push_back(vector<double>());
		for (int j = 0; j < maxStep; j++)
		{
			jointAngs[i].push_back(double());
			jointForces[i].push_back(double());
		}
	}
	//cout << "Sizeof(jointForces) = " << sizeof(jointForces) << ", sizeof(jointForces[0]) = " << sizeof(jointForces[0]) << endl;
	//init COM, feet forces, swing foot touch sensor, and swing foot COM position vectors:
	for (int j = 0; j < maxStep; j++)
	{
		COMpath.push_back(btVector3());
		leftFootForce.push_back(btVector3());
		rightFootForce.push_back(btVector3());
		swingFootTouch.push_back(int());
		swingFootCOMtrace.push_back(btVector3());
	}

#endif


	//READ WEIGHTS:
	// from input file into var called 'w': 
	load_data(m_inputFileName, w);
#ifdef EV_TAU
	load_data(m_inputTauFileName, tau);
#endif

#ifdef TAU_SWITCH
	load_data(m_inputTauFileName, tau);
#endif

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
#ifndef KNEES
		fixedGround->setUserPointer(&IDs[6]);
#else
		fixedGround->setUserPointer(&IDs[8]);
#endif
#endif
		m_dynamicsWorld->addCollisionObject(fixedGround);
#else
		localCreateRigidBody(btScalar(0.),groundTransform,groundShape);
#endif //CREATE_GROUND_COLLISION_OBJECT

	}
	//Correct default gravity setting to more correct 9.81 value
	m_dynamicsWorld->setGravity(btVector3(0, -9.81, 0));
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
#define TICKS_PER_DISPLAY maxStep // maxStep be bigger than total ticks for the whole simulation
#else
#define TICKS_PER_DISPLAY 1
#endif



void RagdollDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//SOCKET CREATION:
	//SOCKET ConnectSocket = m_ragdolls[0]->CreateSocket();

	// vector of target angle values:
	vector<double> targ_angs;

	//BULLET note: simple dynamics world doesn't handle fixed-time-stepping
	//Roman Popov note: timestep is set to bullet's internal tick = 1/60 of a second. This is done to create constancy between
	//graphics ON/OFF versions of the robot. Actuation timestep is 5 * 1/60 of a second, as it removes movement jitter.

	//!!!!look into best values for these vars:
	//neural_step = 1;
	//h = 0.05;

	//Time steps:
	float timeStep = 1.0f / 60.0f;
	float ActuateTimeStep = 5*timeStep;

	if (m_dynamicsWorld)
	{		
		if (!pause || oneStep)
		{
			for (int l = 0; l < TICKS_PER_DISPLAY; l++)
			{
				//cout << "Step: " << SimulationStep << endl;
				//Intiation of the touches
				for (int i = 0; i < bodyCount; i++)
				{
					touches[i] = 0;
					forces[i] = btVector3(0, 0, 0);
				}
				//Making sure all the body parts are active every time step:
				//Body parts change color when inactive for sometime:
				for (int k = 0; k < bodyCount; k++)
				{
					m_ragdolls[0]->m_bodies[k]->setActivationState(ACTIVE_TAG);
				}
				// Populate sensor_val for the first update of CTRNN:
				if (SimulationStep == 0)
				{
					for (int j = 0; j < num_input; j++)
					{
						sensor_val.push_back(0);
					}
				}

#ifdef NEURON
				vector<vector<vector<double>>> temp3Darray;
				temp3Darray = eulerEXPORT(neural_step, h, tau, w, neuron_val, bias, sensor_val, gain);

				for (int step = 0; step < (neural_step / h); step++)
				{
					//cout << "Sim. step = " << SimulationStep << ", int.neuronal tick = " << step << endl;
					for (int layr = 0; layr < neuronHist[0].size(); layr++)
					{
						//cout << "neuronHist. Layer " << layr << ": ";
						for (int nrn = 0; nrn < neuronHist[0][layr].size(); nrn++)
						{
							neuronHist[SimulationStep*(neural_step / h) + step][layr][nrn] = temp3Darray[step][layr][nrn];
							//cout << "neuronHist["<<(SimulationStep*(neural_step / h) + step)<<"]["<<layr<<"]["<<nrn<<"] = "<< (neuronHist[SimulationStep*(neural_step / h) + step][layr][nrn]) <<"  = temp3Darray["<<step<<"]["<<layr<<"]["<<nrn<<"] = " << temp3Darray[step][layr][nrn] << endl;
						}
						//cout << endl;
					}
				}
#endif
				//update neronal states:
				neuron_val = euler(neural_step, h, tau, w, neuron_val, bias, sensor_val, gain);

				//extract values from output layer:
				targ_angs = neuron_val.at(2);

				// for all motors
				for (int i = 0; i < num_output; i++)
				{
					double targetAngle = tanh((targ_angs.at(i) + bias[2][i]) * gain[2][i]);
					//activityIndex += abs(targetAngle); // <- simplified version. Should be: targetAngle / [range of all possible values for targetAngle]
					// but since [range] = 2, or 1 on each side (it's symmetrical), then it is just targetAngle. The larger the angle, the higher the activity index. 
					// check which motor is to be actuated (values are scaled to the desired angle)
					//cout << "SimStep: " << SimulationStep << ". targetAngle(unscaled) = " << targetAngle << ". Accumulated activityIndex = " << activityIndex << endl;
					switch (i)
					{
					case 0: //Left Hip ML
					{
						targetAngle = ((HIP_ML_H - HIP_ML_L) / 2) * targetAngle + ((HIP_ML_H + HIP_ML_L) / 2); //HIP_ML [-38.8; 30.5]
						//cout << "Sending target angle = " << targetAngle * 180 / M_PI << " to " << i << "-th motor that has limits [" << HIP_ML_L * 180 / M_PI << "," << HIP_ML_H * 180 / M_PI << "]" << endl;

						break;
					}
					case 1: //Left Hip AP
					{
						targetAngle = ((HIP_AP_H - HIP_AP_L) / 2) * targetAngle + ((HIP_AP_H + HIP_AP_L) / 2); //HIP_AP [-19; 121]
						//targetAngle = 67.5 * targetAngle - 45; //[22.5; -112.5] according to constraints on the joints (?)
						break;
					}
					case 2: //Right Hip ML
					{
						targetAngle = ((HIP_ML_H - HIP_ML_L) / 2) * targetAngle + ((HIP_ML_H + HIP_ML_L) / 2); //HIP_ML [-38.8; 30.5]
						break;
					}
					case 3: //Right Hip AP
					{
						targetAngle = ((HIP_AP_H - HIP_AP_L) / 2) * targetAngle + ((HIP_AP_H + HIP_AP_L) / 2); //HIP_AP [-19; 121]
						//targetAngle = 67.5 * targetAngle - 45; //[22.5; -112.5] according to constraints on the joints (?)
						break;
					}
					case 4: //Left Ankle ML
					{
						targetAngle = ((ANKL_ML_H - ANKL_ML_L) / 2) * targetAngle + ((ANKL_ML_H + ANKL_ML_L) / 2); //ANKL_ML [-27.75; 27.75]
						break;
					}
					case 5: //Left Ankle AP
					{
						targetAngle = ((ANKL_AP_H - ANKL_AP_L) / 2) * targetAngle + ((ANKL_AP_H + ANKL_AP_L) / 2); //ANKL_AP [39.7; -15.3]
						//targetAngle = 36 * targetAngle - 22.5;//[-58.5; 13.5] according to constraints on the joints (?)
						break;
					}
					case 6: //Right Ankle ML
					{
						targetAngle = ((ANKL_ML_H - ANKL_ML_L) / 2) * targetAngle + ((ANKL_ML_H + ANKL_ML_L) / 2); //ANKL_ML [-27.75; 27.75]
						break;
					}
					case 7: //Right Ankle AP
					{
						targetAngle = ((ANKL_AP_H - ANKL_AP_L) / 2) * targetAngle + ((ANKL_AP_H + ANKL_AP_L) / 2); //ANKL_AP [39.7; -15.3]
						break;
					}
#ifdef TORSO
					case 8: // Body-pelvis, ML
					{
						targetAngle = ((TP_ML_H - TP_ML_L) / 2) * targetAngle + ((TP_ML_H + TP_ML_L) / 2); //TP_ML [-25.45; 26.25]
						break;
					}
					case 9: // Body-pelvis AP
					{
						targetAngle = ((TP_AP_H - TP_AP_L) / 2) * targetAngle + ((TP_AP_H + TP_AP_L) / 2); //TP_AP [-57.65; 29.75]															  
						break;
					}
#endif
#ifdef KNEES
					case 8: // Left Knee, ML
					{
						targetAngle = ((KNEE_ML_H - KNEE_ML_L) / 2) * targetAngle + ((KNEE_ML_H + KNEE_ML_L) / 2); //KNEE_ML [0; 0]
						break;
					}
					case 9: // Left Knee AP
					{
						targetAngle = ((KNEE_AP_H - KNEE_AP_L) / 2) * targetAngle + ((KNEE_AP_H + KNEE_AP_L) / 2); //KNEE_AP [-132; 0]
						break;
					}
					case 10: // Right Knee, ML
					{
						targetAngle = ((KNEE_ML_H - KNEE_ML_L) / 2) * targetAngle + ((KNEE_ML_H + KNEE_ML_L) / 2); //KNEE_ML [0; 0]
						break;
					}
					case 11: // Right Knee, AP
					{
						targetAngle = ((KNEE_AP_H - KNEE_AP_L) / 2) * targetAngle + ((KNEE_AP_H + KNEE_AP_L) / 2); //KNEE_AP [-132; 0]
						break;
					}
#endif
					default:
						break;
					}

					m_ragdolls[0]->ActuateJoint(i / 2, abs(sin(i*M_PI / 2)) + 1, targetAngle, ActuateTimeStep);
					//double currAng = m_ragdolls[0]->m_joints[i / 2]->getRotationalLimitMotor(abs(sin(i*M_PI / 2)) + 1)->m_currentPosition;
					//cout << "SS: " << SimulationStep << ", J# " << (int)i / 2 << ", M#" << abs(sin(i*M_PI / 2)) + 1 << ". Current pos(deg) = " << currAng * 180 / M_PI << ". Target given = " << targetAngle * 180 / M_PI << endl;

				}
				// END UPDATE MOTORS

				m_dynamicsWorld->stepSimulation(timeStep, 0);

				//cout << "World was updated. New joint values!" << endl;
#ifdef COM
				btVector3 axisInA(0, 1, 0);
				btScalar appliedImpulse;
				for (int i = 0; i < num_output; i++)
				{
					jointAngs[i][SimulationStep] = m_ragdolls[0]->m_joints[i / 2]->getRotationalLimitMotor(abs(sin(i*M_PI / 2)) + 1)->m_currentPosition * 180 / M_PI;
					//jointForces[i][SimulationStep] = m_ragdolls[0]->m_joints[i / 2]->getRotationalLimitMotor(abs(sin(i*M_PI / 2)) + 1)->m_accumulatedImpulse / (1.f / 60.f);
					//jointForces[i][SimulationStep] = m_ragdolls[0]->m_joints[i / 2]->getAppliedImpulse() / (1.f / 60.f);


					appliedImpulse = m_ragdolls[0]->m_joints[i / 2]->getJointFeedback()->m_appliedTorqueBodyA.dot(axisInA);
					jointForces[i][SimulationStep] = appliedImpulse / (1.f / 60.f);
					double currAng = m_ragdolls[0]->m_joints[i / 2]->getRotationalLimitMotor(abs(sin(i*M_PI / 2)) + 1)->m_currentPosition;
					//cout << "SimStep: " << SimulationStep << ", joint num " << (int)i / 2 << ", motor num " << abs(sin(i*M_PI / 2)) + 1 << ". Current pos(deg) = " << currAng * 180 / M_PI << endl;
					COMpath[SimulationStep] = m_ragdolls[0]->wholeBodyCOM();
					leftFootForce[SimulationStep] = forces[BODYPART_LEFT_FOOT];
					rightFootForce[SimulationStep] = forces[BODYPART_RIGHT_FOOT];
					swingFootTouch[SimulationStep] = touches[BODYPART_LEFT_FOOT];
					swingFootCOMtrace[SimulationStep] = m_ragdolls[0]->getCOMposition(BODYPART_LEFT_FOOT);
			    }
#endif 

				//cout << "Back to inside values" << endl;

#ifndef COM
#ifndef NEURON
				// Check if robot is still upright:
				m_ragdolls[0]->isUpright(tempFitness, maxStep, SimulationStep);
#endif
#endif
				//if check is passed, continue the simulation and fitness calculation:;
				tempFitness += m_ragdolls[0]->onlineFitness(SimulationStep, maxStep);


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
					sensor_val.push_back(touches[(BODYPART_LEFT_FOOT + j)]);
					//cout << "Sensor val " << j << " from body #" << BODYPART_LEFT_FOOT + j << " = " << touches[(BODYPART_LEFT_FOOT + j)] << endl;
				}
				
				// Increase the simulation time step counter:
				
				SimulationStep++;
#ifdef TAU_SWITCH
				if (SimulationStep == (maxStep / 2))
				{
					tau = flipTau(tau);
				}
#endif

#ifndef TRAIN //if DEMO!!!

				//Stopping simulation in the end of time for DEMO robots (paused right before the end)
				if (SimulationStep >= maxStep)
				{
#ifdef COM
					cout << "Simstep = " << SimulationStep << endl;
					m_ragdolls[0]->save_1DbtV3(leftFootForce, "leftFootForce.txt");
					m_ragdolls[0]->save_1DbtV3(rightFootForce, "rightFootForce.txt");
					m_ragdolls[0]->save_1DbtV3(COMpath, "com.txt");
					m_ragdolls[0]->save_1DInt(swingFootTouch, "swingFootTouch.txt");
					m_ragdolls[0]->save_1DbtV3(swingFootCOMtrace, "swingFootCOMtrace.txt");
					m_ragdolls[0]->save_2DDbl(jointAngs, "jointAngs.txt");
					m_ragdolls[0]->save_2DDbl(jointForces, "jointForces.txt");

					vector<btVector3> targsPos;
					targsPos.push_back(btVector3()); targsPos.push_back(btVector3());
					targsPos = m_ragdolls[0]->getTargPos();
					m_ragdolls[0]->save_1DbtV3(targsPos, "targets.txt");

#else

					//double fitval = tempFitness / SimulationStep;
					double fitval = tempFitness;
					cout << "SimStep: " << SimulationStep << ", C++ fitness: " << fitval << endl;
					getchar();
					
#endif
#ifdef NEURON
					//transform 3D to 2D neuronal states vector [time X layer X neuron] -> [time X neuron]
					vector<vector<double>> outputNeurStates;
					int layr, neuronCount;
					for (int t = 0; t < neuronHist.size(); t++)
					{
						outputNeurStates.push_back(vector<double>());
						for (int nrn = 0; nrn < (num_input + num_hidden + num_output); nrn++)
						{
							outputNeurStates[t].push_back(double());
							if (nrn < num_input) { layr = 0; neuronCount = nrn; }
							if ((nrn < (num_input + num_hidden)) && (nrn >= num_input)) { layr = 1; neuronCount = nrn - num_input; }
							if (nrn >= (num_input + num_hidden)) { layr = 2; neuronCount = nrn - num_input - num_hidden; }
							outputNeurStates[t][nrn] = neuronHist[t][layr][neuronCount];
						}
					}
					//export 2d vector:
					m_ragdolls[0]->save_2DDbl(outputNeurStates, "neuron.txt");
#endif
					exit(0);
				}
#else // IF TRAIN:
				if (SimulationStep >= maxStep)
				{
					//double fitval = tempFitness / SimulationStep;
					double fitval = tempFitness;
					ofstream outputFile;
					outputFile.open("fit.txt", ios_base::app);
					outputFile << fitval << endl;
					outputFile.close();
					exit(0);
				}
#endif
				
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





