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
//#define WIN32_LEAN_AND_MEAN
////socket info:
#define _WINSOCKAPI_
#include <windows.h>

#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>
#include <stdio.h>
//
//
//// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")
//

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "27015"
////end socket info

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

SOCKET ConnectSocket;

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

	btVector3 getLeftTargPos()
	{
		double initPelvisHeight = 0.3 + length_foot / 60 + height_leg / 30;
		btScalar TargZ = initPelvisHeight / 1.5; //step length = 1/3 of body height, just like in humans
		btScalar TargY = 0.3 + length_foot / 120;
		btScalar TargX = height_pelvis / 60;//
		return btVector3(TargX, TargY, TargZ);
	}

	btVector3 getRightTargPos()
	{
		double initPelvisHeight = 0.3 + length_foot / 60 + height_leg / 30;
		btScalar TargZ = 0; //step length = 1/3 of body height, just like in humans
		btScalar TargY = 0.3 + length_foot / 120;
		btScalar TargX = - height_pelvis / 60;//
		return btVector3(TargX, TargY, TargZ);
	}


	// use 0 if robot fell, 1 - if it didn't fall
	double fitness(bool isUprightFlag)
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
		//double rightTargZ = initPelvisHeight / 1.5;
		double rightTargZ = 0; //don't move the right leg, stay at the initial position. 
		double rightTargX = - height_pelvis / 60;

		if (isUprightFlag == 1)
		{
			double result = 0.33334*(1 - abs(pelRot / 90)) + (-0.1*abs(leftTargX - lfPos.x()) + 1) + (-0.1*abs(leftTargZ - lfPos.z()) + 1) + 0.33333*(-0.1*abs(rightTargX - rfPos.x()) + 1) + 0.33333*(-0.1*abs(rightTargZ - rfPos.z()) + 1);
			return result / 3;//all members of expression at maximum reach 1 each, this is normalization.
		}
		else // if robot fell - lower the fitness:
		{
			double result = 0.33334*(1 - abs(pelRot / 90)) + (-0.1*abs(leftTargX - lfPos.x()) + 1) + (-0.1*abs(leftTargZ - lfPos.z()) + 1) + 0.33333*(-0.1*abs(rightTargX - rfPos.x()) + 1) + 0.33333*(-0.1*abs(rightTargZ - rfPos.z()) + 1);
			return result / 9;
		}

		//double result = (1 - abs(pelRot / 90)) + (-0.1*abs(leftTargX - lfPos.x()) + 1) + (-0.1*abs(leftTargZ - lfPos.z()) + 1) + (-0.1*abs(rightTargX - rfPos.x()) + 1) + (-0.1*abs(rightTargZ - rfPos.z()) + 1);
		//return result / 5;
	}

	double onlineFitness()
	{
		btRigidBody * leftFoot = m_bodies[BODYPART_LEFT_FOOT];
		btRigidBody * rightFoot = m_bodies[BODYPART_RIGHT_FOOT];
		btRigidBody * pelvis = m_bodies[BODYPART_PELVIS];
		btGeneric6DofConstraint * rightAnkleJoint = m_joints[JOINT_RIGHT_ANKLE];

		btVector3 lfPos = leftFoot->getCenterOfMassPosition();
		btVector3 rfPos = rightFoot->getCenterOfMassPosition();
		btVector3 pelPos = pelvis->getCenterOfMassPosition();
		//get rotation of pelvis around the Y-axis:
		btScalar pelRot, junk1, junk2;
		pelvis->getCenterOfMassTransform().getBasis().getEulerZYX(junk1, pelRot, junk2);
		//double pelRot = rightAnkleJoint->getRotationalLimitMotor(2)->m_currentPosition;
		double rightAnkleAPRot = rightAnkleJoint->getRotationalLimitMotor(2)->m_currentPosition;
		double rightAnkleMLRot = rightAnkleJoint->getRotationalLimitMotor(1)->m_currentPosition;

		double initPelvisHeight = 0.3 + length_foot / 60 + height_leg / 30;
		double leftTargZ = initPelvisHeight / 1.5; //step length = 1/3 of body height, just like in humans
		double leftTargX = height_pelvis / 60;// there should be no movement along X-axis, so the foot should maintain its initial pos along x-axis
											  //double rightTargZ = initPelvisHeight / 1.5;
		double rightTargZ = 0; //don't move the right leg, stay at the initial position. 
		double rightTargX = -height_pelvis / 60;
		double initPelvRot = 0;
		double initAnklRotAP = 0;
		double initAnklRotML = 0;
		double scalingConst = 5;

		// Fitness members:
		double pelvRotMember = 1 / (1 + abs(initPelvRot - pelRot));
		double stanceAnklRotMemberAP = 1 / (1 + abs(initAnklRotAP - rightAnkleAPRot));
		double stanceAnklRotMemberML = 1 / (1 + abs(initAnklRotML - rightAnkleAPRot));
		double targetMemberX = 1 / (1 + scalingConst * abs(leftTargX - lfPos.x()));
		double targetMemberZ = 1 / (1 + scalingConst * abs(leftTargZ - lfPos.z()));
		double stanceMemberX = 1 / (1 + abs(rightTargX - rfPos.x()));
		double stanceMemberZ = 1 / (1 + abs(rightTargZ - rfPos.z()));
		double pelvHeightMember = 1 / (1 + abs(initPelvisHeight - pelPos.y()));
		double result = pelvRotMember * pelvHeightMember * stanceAnklRotMemberAP * stanceAnklRotMemberML * targetMemberX * targetMemberZ * stanceMemberX*stanceMemberZ;
		return result;

	}
#ifndef DIRECT
	void isUpright()
	{
		btRigidBody * pelvis = m_bodies[BODYPART_PELVIS];
		btVector3 pelPos = pelvis->getCenterOfMassPosition();
		
		if (pelPos.y() < PELV_HEIGHT * 0.75)
		{
			double fitval = fitness(0);
			ofstream outputFile;
			outputFile.open("fit.txt", ios_base::app);
			outputFile << fitval << endl;
			outputFile.close();
			//cout << "Exit early due to fall. Curr height: " << pelPos.y() << " < 85% of Init height " << PELV_HEIGHT << " = " << PELV_HEIGHT*0.85 <<". Sim Step: " << SimulationStep << ". Fitness: " << fitval << endl;
			exit(0);
		}
	}
#endif


//NOt optimized, not completely debugged socket connection to MATLAB
#ifdef DIRECT
	void isUpright(SOCKET ConnectSocket, bool socketFlag)
	{
		btRigidBody * pelvis = m_bodies[BODYPART_PELVIS];
		btVector3 pelPos = pelvis->getCenterOfMassPosition();

		if (pelPos.y() < PELV_HEIGHT * 0.85)
		{
			if (socketFlag == 0)//0 flag = don't use socket
			{
				ofstream outputFile;
				outputFile.open("fit.txt", ios_base::app);
				outputFile << fitness() << endl;
				outputFile.close();
			}
			else
			{
				storeFitness(ConnectSocket, 0);
			}
			exit(0);
		}
	}


	SOCKET CreateClientSocket()
	{
		// socket info
		WSADATA wsaData;
		SOCKET ConnectSocket = INVALID_SOCKET;
		// Initialize Winsock
		struct addrinfo *result = NULL,
						*ptr = NULL,
						hints;
		//make a test cal, check for errors:
		int iResult;

		//Initialize Winsock:
		iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
		if (iResult != 0) {
			//printf("WSAStartup failed with error: %d\n", iResult);
		}
		ZeroMemory(&hints, sizeof(hints));
		hints.ai_family = AF_UNSPEC;
		hints.ai_socktype = SOCK_STREAM;
		hints.ai_protocol = IPPROTO_TCP;

		// Resolve the server address and port
		iResult = getaddrinfo("localhost", DEFAULT_PORT, &hints, &result);
		if (iResult != 0) {
			//printf("getaddrinfo failed with error: %d\n", iResult);
			WSACleanup();
			//return 1;
		}

		// Attempt to connect to an address until one succeeds
		for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

			// Create a SOCKET for connecting to server
			ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
			if (ConnectSocket == INVALID_SOCKET) {
				//printf("socket failed with error: %ld\n", WSAGetLastError());
				WSACleanup();
				//return 1;
			}

			// Connect to server.
			iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
			//if (iResult == 0) cout << "C++ Client is connected to the server!\n";
			if (iResult == SOCKET_ERROR) {
				closesocket(ConnectSocket);
				ConnectSocket = INVALID_SOCKET;
				continue;
			}
			break;
		}
		freeaddrinfo(result);
		if (ConnectSocket == INVALID_SOCKET) {
			//printf("C++ Client is unable to connect to server!\n");
			WSACleanup();
			//return 1;
		}
		return ConnectSocket;
	}

	void CloseSocket(SOCKET ConnectSocket)
	{
		int iResult;
		// shutdown the send half of the connection since no more data will be sent
		//iResult = shutdown(ClientSocket, SD_SEND);
		iResult = shutdown(ConnectSocket, SD_BOTH);
		//cout << "Shutdown socket result (0 - receive, 1 - send, 2 - both): " << iResult << endl;
		if (iResult == SOCKET_ERROR) {
			//printf("shutdown failed: %d\n", WSAGetLastError());
			//closesocket(ClientSocket);
			closesocket(ConnectSocket);
			WSACleanup();
		}
		// cleanup
		closesocket(ConnectSocket);
		WSACleanup();
	}
	//If robot fell - pass 0, if not - 1
	void storeFitness(SOCKET ConnectSocket)
	{
		double fitVal;
		int iSendResult;

		fitVal = fitness();
		string fitValStr = to_string(fitVal);
		//iSendResult = send(ClientSocket, fitValStr.c_str(), fitValStr.size(), 0);
		//cout << "Attempting to send fitness value of " << fitValStr.c_str() << endl;
		//cout << "Message length: " << fitValStr.size() << endl;
		iSendResult = send(ConnectSocket, fitValStr.c_str(), fitValStr.size(), 0);
		//cout << "Send result (number of bytes sent, or -1 = Error): " << iSendResult << endl;
		if (iSendResult == SOCKET_ERROR) {
			//printf("send failed: %d\n", WSAGetLastError());
			//closesocket(ClientSocket);
			closesocket(ConnectSocket);
			WSACleanup();
		}	
	}

	//vector<vector<double>> receiveWeights(SOCKET ConnectSocket)

	//void receiveWeights(SOCKET ConnectSocket)
	//{
	//	int iResult;
	//	//int numWts = 5;
	//	char recvbuf[DEFAULT_BUFLEN];
	//	int recvbuflen = DEFAULT_BUFLEN;
	//	
	//	double tempValue;
	//	//for (int i = 0; i < numWts; i++)
	//	//{
	//	iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);
	//		//cout << "Read Result: " << iResult << endl;
	//	if (iResult > 0)
	//	{
	//		printf("Bytes received: %d\n", iResult);
	//		tempValue = atof(recvbuf);
	//		cout << "Value obtained: " << tempValue << endl;
	//	}
	//	else if (iResult == 0)
	//		printf("Connection closed\n");
	//	else
	//		printf("recv failed with error: %d\n", WSAGetLastError());
	//	//}
	//}
#endif

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
#ifndef KNEES
	int groundID = 6;
#else
	int groundID = 8;
#endif
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
	pause = false;// should be false
#else
	pause = true;//should be true
#endif
	oneStep = false;

//---- intializing bodypart IDs
#ifdef TORSO
	for (int i = 0; i < 8; i++)
	{
		IDs[i] = i;
	};
#else
#ifndef KNEES
	for (int i = 0; i < 7; i++)
	{
		IDs[i] = i;
	};
#else
	for (int i = 0; i < 9; i++)
	{
		IDs[i] = i;
	};
#endif
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

#ifdef COM
	//intialize 2d vector for COM coordinates:
	for (int i = 0; i < 3; i++)
	{
		COMpath.push_back(vector<double>());
		for (int j = 0; j < maxStep; j++)
		{
			COMpath[i].push_back(double());
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
#ifdef DIRECT
	ConnectSocket = m_ragdolls[0]->CreateClientSocket();
	//m_ragdolls[0]->receiveWeights(ConnectSocket);
#endif
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
	neural_step = 1;
	h = 0.1;

	//Add actuation time step: 
	float ActuateTimeStep = 5*(1.f/60.f);

	if (m_dynamicsWorld)
	{		
		if (!pause || oneStep)
		{
			for (int l = 0; l < TICKS_PER_DISPLAY; l++)
			{
				//Intiation of the touches
				for (int i = 0; i < bodyCount; i++)
				{
					touches[i] = 0;
				}
				//Making sure all the body parts are active every time step:
				//Body parts change color when inactive for sometime:
				for (int k = 0; k< bodyCount; k++)
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

				//update neronal states:
				neuron_val = euler(neural_step, h, tau, w, neuron_val, bias, sensor_val, gain);
				
				//extract values from output layer:
				targ_angs = neuron_val.at(2);

				// for all motors
				for (int i = 0; i<num_output; i++)
				{
					double targetAngle = tanh(targ_angs.at(i));
					//activityIndex += abs(targetAngle); // <- simplified version. Should be: targetAngle / [range of all possible values for targetAngle]
					// but since [range] = 2, or 1 on each side (it's symmetrical), then it is just targetAngle. The larger the angle, the higher the activity index. 
					// check which motor is to be actuated (values are scaled to the desired angle)
					//cout << "SimStep: " << SimulationStep << ". targetAngle(unscaled) = " << targetAngle << ". Accumulated activityIndex = " << activityIndex << endl;
					switch (i)
					{
					case 0: //Left Hip ML
					{
						targetAngle = ((HIP_ML_H - HIP_ML_L)/2) * targetAngle + ((HIP_ML_H + HIP_ML_L) / 2); //HIP_ML [-38.8; 30.5]
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
					
					m_ragdolls[0]->ActuateJoint(i / 2, abs(sin(i*M_PI / 2))+1, targetAngle, ActuateTimeStep);


					
				}
				// END UPDATE MOTORS

				m_dynamicsWorld->stepSimulation(1.f / 60.f,0);
				tempFitness += m_ragdolls[0]->onlineFitness();
#ifdef COM
				btVector3 thisCOM = m_ragdolls[0]->wholeBodyCOM();
				COMpath[0][SimulationStep] = thisCOM.x();
				COMpath[1][SimulationStep] = thisCOM.y();
				COMpath[2][SimulationStep] = thisCOM.z();
				//cout << "COM.x = " << thisCOM.x() << ", COM.y = " << thisCOM.y() << ", COM.z = " << thisCOM.z() << endl;
				// An attempt to also draw a sphere where COM belongs
				//virtual void renderCOM()
				//{
				//	extern GLDebugDrawer gDebugDrawer;
				//	GlutDemoApplication::renderCOM();
				//	gDebugDrawer.drawSphere(thisCOM, 10.0, btVector3(0.75, 0.75, 0.));
				//}
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
					sensor_val.push_back(touches[(BODYPART_LEFT_FOOT + j)]);
				}

				// Increase the simulation time step counter:
				SimulationStep++;

#ifndef TRAIN //if DEMO!!!

				//Stopping simulation in the end of time for DEMO robots (paused right before the end)
				if (SimulationStep >= maxStep)
				{
					double fitval = tempFitness/SimulationStep;
					cout << "SimStep: " << SimulationStep << ", C++ fitness: " << fitval << endl;
#ifdef COM
					ofstream COMoutputFile;
					COMoutputFile.open("com.txt", ios_base::app);
					for (int m = 0; m < SimulationStep; m++)
					{
						COMoutputFile << COMpath[0][m]<< " " << COMpath[1][m] << " " << COMpath[2][m] << endl;
					}
					COMoutputFile.close();

					// Store target locations for plotting
					ofstream TargOutputFile;
					TargOutputFile.open("targets.txt", ios_base::app);

					btVector3 leftTargPos = m_ragdolls[0]->getLeftTargPos();
					btVector3 rightTargPos = m_ragdolls[0]->getRightTargPos();

					TargOutputFile << leftTargPos.x() << " " << leftTargPos.y() << " " << leftTargPos.z() << endl;
					TargOutputFile << rightTargPos.x() << " " << rightTargPos.y() << " " << rightTargPos.z() << endl;

					TargOutputFile.close();

#endif
					getchar();
					exit(0);
				}
#else // IF TRAIN:
				if (SimulationStep >= maxStep)
				{
#ifdef DIRECT
					//socket export:
					m_ragdolls[0]->storeFitness(ConnectSocket, 1);
					m_ragdolls[0]->CloseSocket(ConnectSocket);
#else
					double fitval = tempFitness / SimulationStep;
					ofstream outputFile;
					outputFile.open("fit.txt", ios_base::app);
					outputFile << fitval << endl;
					outputFile.close();

#endif
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





