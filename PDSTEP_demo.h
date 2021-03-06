/*
Bullet Continuous Collision Detection and Physics Library
RagdollDemo
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

#ifndef RAGDOLLDEMO_H
#define RAGDOLLDEMO_H

#include <string> //added
#include <vector> //added
using namespace std; //added
#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "GLDebugDrawer.h" //added
#include <time.h>
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;



class RagdollDemo : public GlutDemoApplication
{

	btAlignedObjectArray<class RagDoll*> m_ragdolls;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;



	bool pause, oneStep; //bools additional control 

public:
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

	// "added to the demo":
#ifdef TRAIN
	int maxStep = 120;
#else
	int maxStep = 120;//this is to debug only, remove in final version.
#endif

#ifdef TORSO
	int IDs[8]; //CHANGE ACCORDING TO BODY PLAN; pointers to the body parts in contact
	int touches[8]; //record if corresponding body part is in contact, +1 to touches bcuz ground is an object too
	btVector3 touchPoints[8];//locations of contact points
#else
#ifndef KNEES
	int IDs[7]; //CHANGE ACCORDING TO BODY PLAN; pointers to the body parts in contact
	int touches[7]; //record if corresponding body part is in contact, +1 to touches bcuz ground is an object too
	btVector3 touchPoints[7];//locations of contact points
#else // KNEES
	int IDs[9]; //CHANGE ACCORDING TO BODY PLAN; pointers to the body parts in contact
	int touches[9]; //record if corresponding body part is in contact, +1 to touches bcuz ground is an object too
	btVector3 touchPoints[9];//locations of contact points
	int bodyCount = sizeof(IDs) / sizeof(IDs[0]);
	btVector3 forces[9];
#endif
#endif

	//intra-simulation fitness counter
	double tempFitness; 
	//counter
	int SimulationStep;
	// weights passed from evolutionary algorithm
	vector<vector<double>> w;
	// name of the file with weights
	string m_inputFileName; 
#ifdef EV_TAU
	// name of the file with taus:
	string m_inputTauFileName;
#endif
#ifdef TAU_SWITCH
	// name of the file with taus:
	string m_inputTauFileName;
	vector<vector<double>> flipTau(vector<vector<double>> tau);
#endif

	//CTRNN params:
	// number of input neurons (= number of sensors):
	int num_input = 2;
	// number of hidden neurons:
	int num_hidden = 2;
	//number of output neurons (= number of joint motors):
#ifdef TORSO
	int num_output = 10;
	int circleCount = 9;//for drawing red circles at contact points
#else // NO TORSO
#ifndef KNEES
	int num_output = 8;
	int circleCount = 8;
#else
	int num_output = 12;
	int circleCount = 12;//for drawing red circles at contact points
#endif
#endif

	// all neuronal states or values are held in this vector of vectors:
	vector<vector<double>> neuron_val;//number of layers X number of neurons in this layer

#ifdef JOINT
	// all neuronal states or values are held in this vector of vectors:
	vector<vector<double>> joint_val;//number of joints x time
#endif

	// biases for each layer:
	vector<vector<double>> bias;
	// gains for each layer:
	vector<vector<double>> gain;
	//vector of sensor values:
	vector<int> sensor_val;
	// time constant (the same for all neurons, so far; not optimized)
	//double tau = 2.0;
#ifdef EV_TAU
	vector<vector<double>> tau;
#else
#ifdef TAU_SWITCH
	vector<vector<double>> tau;
#else
	vector<double> tau;
#endif
#endif
	// bias value (the same for all neurons, so far; not optimized)
	double bias_val = 0.001;
	// gain value (the same for all neurons, so far; not optimized)
	double gain_val = 1.0;
	// step size for CTRNN update (should be larger? than the simulation step size, 
	//so that CTRNN could catch up with environment):
	double neural_step = 1.0;
	// integration step size for updating the neuronal states:
	double h = 0.1;
	// END CTRNN params

	// neuron_val, bias and gain vectors: 
	vector<double> temp_row, temp_bias_row, temp_gain_row;


	// Declare function that calculates the neuronal state according to the equation within:
	double updateNeuron(double tau, vector<double> previous_layer, double current_neuron, vector<double> w, vector<double> bias_val, int sensor_val, vector<double> gain);
	double updateNeuronDEBUG(double tau, vector<double> previous_layer, double current_neuron, vector<double> w, vector<double> bias_val, int sensor_val, vector<double> gain);
	// (update function, initial value, time1, time2, step size, time constant)
#ifdef EV_TAU
	vector<vector<double>> RagdollDemo::euler(double neural_step, double h, vector<vector<double>> tau, vector<vector<double>> w, vector<vector<double>> neuron_vals, vector<vector<double>> bias, vector<int> touches, vector<vector<double>> gain);
#else
#ifdef TAU_SWITCH
	//vector<vector<double>> euler(double neural_step, double h, double tau, vector<vector<double>> w, vector<vector<double>> neuron_val, vector<vector<double>> bias, vector<int> touches, vector<vector<double>> gain);
	vector<vector<double>> euler(double neural_step, double h, vector<vector<double>> tau, vector<vector<double>> w, vector<vector<double>> neuron_vals, vector<vector<double>> bias, vector<int> touches, vector<vector<double>> gain);
#else
	vector<vector<double>> RagdollDemo::euler(double neural_step, double h, vector<double> tau, vector<vector<double>> w, vector<vector<double>> neuron_val, vector<vector<double>> bias, vector<int> touches, vector<vector<double>> gain);
#endif
#endif

#ifdef EV_TAU
	vector<vector<vector<double>>> RagdollDemo::eulerEXPORT(double neural_step, double h, vector<vector<double>> tau, vector<vector<double>> w, vector<vector<double>> neuron_vals, vector<vector<double>> bias, vector<int> touches, vector<vector<double>> gain);
#else
#ifdef TAU_SWITCH
	//vector<vector<double>> euler(double neural_step, double h, double tau, vector<vector<double>> w, vector<vector<double>> neuron_val, vector<vector<double>> bias, vector<int> touches, vector<vector<double>> gain);
	vector<vector<vector<double>>> eulerEXPORT(double neural_step, double h, vector<vector<double>> tau, vector<vector<double>> w, vector<vector<double>> neuron_vals, vector<vector<double>> bias, vector<int> touches, vector<vector<double>> gain);
#else
	vector<vector<vector<double>>> eulerEXPORT(double neural_step, double h, vector<double> tau, vector<vector<double>> w, vector<vector<double>> neuron_val, vector<vector<double>> bias, vector<int> touches, vector<vector<double>> gain);
#endif
#endif
	// function that reads wts from a file
	void initParams(const std::string& inputFileName); 
	void initParams1(const std::string& inputTauFileName);

#ifdef NEURON
	//Time Step counter for global tracking of neuronal time 
	//(euler fcn only updates values for "neural_step", export is accumulated internally in euler):
	int tsCounter;
	vector<vector<vector<double>>> neuronHist;
#endif
	// if COM needs to be extracted:
#ifdef COM
	vector<btVector3> COMpath; //rows x, y, z; columns - simulation steps
	vector<btVector3> leftFootForce;
	vector<btVector3> rightFootForce;
	vector<btVector3> swingFootCOMtrace;
	vector<int> swingFootTouch;
	vector<vector<double>> jointAngs;
	vector<vector<double>> jointForces;
	
#endif


	RagdollDemo();
    // end "added to demo"

	void initPhysics();

	void exitPhysics();

	virtual ~RagdollDemo()
	{
		exitPhysics();
	}

	void spawnRagdoll(const btVector3& startOffset);

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	// "added to the demo"
	// code draws small red spheres at contact points
	virtual void renderme()
	{
		extern GLDebugDrawer gDebugDrawer;
		// Call the parent method.
		GlutDemoApplication::renderme();

		//Makes a sphere with 0.2 radius around every contact with the ground, third argument is color in RGB - grey
		for (int i = 0; i < circleCount; i++)
		{
			if (touches[i])
				gDebugDrawer.drawSphere(touchPoints[i], 0.2, btVector3(1., 0., 0.));
			// example:
			/*Make a circle with a 0.9 radius at (0,0,0) with RGB color (1,0,0).*/
			/*gDebugDrawer.drawSphere(btVector3(0.,0.,0.), 0.9, btVector3(1., 0., 0.));*/
		}
// DRAW TARGETS:
#ifdef MALE
		double avBH = 181.0;
		double avBM = 78.4;
		double height_pelvis = 26.4 + 0.0473*avBM - 0.0311*avBH;
		double length_foot = 3.8 + 0.013*avBM + 0.119*avBH;
		double height_thigh = 4.26 - 0.0183*avBM + 0.24*avBH;
		double height_shank = -16.0 + 0.0218*avBM + 0.321*avBH;
		double height_leg = -11.74 + 0.0035*avBM + 0.561*avBH; // thigh + shank
#else //Female:
		double avBH = 169.0;
		double avBM = 75.4;
		double height_pelvis = 21.4 + 0.0146*avBM - 0.005*avBH;
		double length_foot = 7.39 + 0.0311*avBM + 0.0867*avBH;
		double height_thigh = -26.8 - 0.0725*avBM + 0.436*avBH;
		double height_shank = -7.21 - 0.0618*avBM + 0.308*avBH;
		double height_leg = height_thigh + height_shank; // thigh + shank
#endif

#ifndef TRAIN
		double initPelvisHeight = 0.3 + length_foot / 60 + height_leg / 30;
		double leftTargZ = initPelvisHeight / 1.5; //step length = 1/3 of body height, just like in humans
		double leftTargX = height_pelvis / 60;// there should be no movement along X-axis, so the foot should maintain its initial pos along x-axis
		double rightTargZ = initPelvisHeight / 1.5; // now have both feet reach out for those targets.
		double rightTargX = -height_pelvis / 60;
		double leftTargY = 0.15;
		double rightTargY = 0.15;

		btVector3 left = btVector3(btScalar(leftTargX), btScalar(leftTargY), btScalar(leftTargZ));
		btVector3 right = btVector3(btScalar(rightTargX), btScalar(rightTargY), btScalar(rightTargZ));
		btVector3 leftInitPos = btVector3(btScalar(height_pelvis / 60), btScalar(0.15), btScalar(0));
		btVector3 rightInitPos = btVector3(btScalar(-height_pelvis / 60), btScalar(0.15), btScalar(0));

		gDebugDrawer.drawSphere(left, 0.4, btVector3(0., 1., 1.));
		gDebugDrawer.drawSphere(right, 0.4, btVector3(1., 1., 0.));
		gDebugDrawer.drawSphere(leftInitPos, 0.4, btVector3(0.7, 1., 1.));
		gDebugDrawer.drawSphere(rightInitPos, 0.4, btVector3(1., 1., 0.7));
#endif

	}
	// end "added to the demo"

	static DemoApplication* Create()
	{
		RagdollDemo* demo = new RagdollDemo();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
	
};


#endif

