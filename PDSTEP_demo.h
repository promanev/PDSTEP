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

	// "added to the demo":
#ifdef TRAIN
	int maxStep = 50;
#else
	int maxStep = 50;//this is to debug only, remove in final version.
#endif

#ifdef TORSO
	int IDs[8]; //CHANGE ACCORDING TO BODY PLAN; pointers to the body parts in contact
	int touches[8]; //record if corresponding body part is in contact, +1 to touches bcuz ground is an object too
	btVector3 touchPoints[8];//locations of contact points
#else
	int IDs[7]; //CHANGE ACCORDING TO BODY PLAN; pointers to the body parts in contact
	int touches[7]; //record if corresponding body part is in contact, +1 to touches bcuz ground is an object too
	btVector3 touchPoints[7];//locations of contact points
#endif
	//intra-simulation fitness counter
	double tempFitness; 
	//counter
	int SimulationStep;
	// ANN weights passed from evolutionary algorithm
	vector<vector<double>> w;
	// name of the file with weights
	string m_inputFileName; 

	//CTRNN params:
	// step size for CTRNN update (should be larger? than the simulation step size, 
	//so that CTRNN could catch up with environment):
	double neural_step;
	// number of input neurons (= number of sensors):
	int num_input = 2;
	// number of hidden neurons:
	int num_hidden = 2;
	//number of output neurons (= number of joint motors):
#ifdef TORSO
	int num_output = 10;
	int circleCount = 9;
#else
	int num_output = 8;
	int circleCount = 8;
#endif
	// all neuronal states or values are held in this vector of vectors:
	vector<vector<double>> neuron_val;
	// biases for each layer:
	vector<vector<double>> bias;
	// gains for each layer:
	vector<vector<double>> gain;
	//vector of sensor values:
	vector<int> sensor_val;
	// time constant (the same for all neurons, so far; not optimized)
	double tau = 10;
	// bias value (the same for all neurons, so far; not optimized)
	double bias_val = 0.0;
	// integration step size for updating the neuronal states:
	double h;
	// END CTRNN params

	// neuron_val, bias and gain vectors: 
	vector<double> temp_row, temp_bias_row, temp_gain_row;


	// Declare function that calculates the neuronal state according to the equation within:
	double updateNeuron(double tau, vector<double> previous_layer, double current_neuron, vector<double> w, double bias_val, int sensor_val, vector<double> gain);
	// (update function, initial value, time1, time2, step size, time constant)
	vector<vector<double>> euler(double neural_step, double h, double tau, vector<vector<double>> w, vector<vector<double>> neuron_val, vector<vector<double>> bias, vector<int> touches, vector<vector<double>> gain);
	// function that reads wts from a file
	void initParams(const std::string& inputFileName); 

	//vector for neuronal values:
	vector<vector<vector<double>>> history;//layer * neuron * time
	
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

		//Makes a sphere with 0.2 radius around every contact with the ground, third argument is color in RGB (1,0,0)
		for (int i = 0; i < circleCount; i++)
		{
			if (touches[i])
				gDebugDrawer.drawSphere(touchPoints[i], 0.2, btVector3(1., 0., 0.));
			// example:
			/*Make a circle with a 0.9 radius at (0,0,0) with RGB color (1,0,0).*/
			/*gDebugDrawer.drawSphere(btVector3(0.,0.,0.), 0.9, btVector3(1., 0., 0.));*/
		}
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
