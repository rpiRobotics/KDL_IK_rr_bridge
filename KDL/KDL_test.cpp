#include "stdafx.h"


// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

#include <chain.hpp>
#include <chainfksolver.hpp>
#include <chainfksolverpos_recursive.hpp>
#include <chainiksolvervel_pinv.hpp>
#include <chainiksolverpos_nr.hpp>
#include <frames_io.hpp>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <mex.h>

using namespace KDL;

int Test_Fwd(KDL::Chain chain, KDL::Frame &cartpos) {

	// Create solver based on kinematic chain
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

	// Create joint array
	unsigned int nj = chain.getNrOfJoints();
	KDL::JntArray jointpositions = JntArray(nj);

	// Assign some values to the joint positions
	for (unsigned int i = 0; i<nj; i++) {
		float myinput;
		printf("Enter the position of joint %i: ", i);
		scanf("%e", &myinput);
		jointpositions(i) = (double)myinput;
	}

	// Sawyer special 0 config
	KDL::Frame f_EE = Frame(Rotation(Vector(1.0000, 0.0000, 0.0000), Vector(0.0000, 0.984815757178173, -0.173602777666664), Vector(0.0000, 0.173602777666664, 0.984815757178173)));
	//std::cout << f_EE << std::endl;

	// Calculate forward position kinematics
	bool kinematics_status;
	kinematics_status = fksolver.JntToCart(jointpositions, cartpos);

	std::cout << "End Effector:" << std::endl;
	std::cout << cartpos << std::endl;

	// Count in Sawyer R_EE
	cartpos = cartpos*f_EE;

	std::cout << "End Effector (Counting R_EE):" << std::endl;

	if (kinematics_status >= 0) {
		std::cout << cartpos << std::endl;
		printf("%s \n", "Success, thanks KDL!");
		return EXIT_SUCCESS;
	}
	else {
		printf("%s \n", "Error: could not calculate forward kinematics :(");
		return EXIT_FAILURE;
	}
}

int Test_Fwd(KDL::Chain chain, KDL::Frame &cartpos, JntArray jointpositions) {

	// Create solver based on kinematic chain
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

	// Create joint array
	unsigned int nj = chain.getNrOfJoints();
	
	// Sawyer special 0 config
	KDL::Frame f_EE = Frame(Rotation(Vector(1.0000, 0.0000, 0.0000), Vector(0.0000, 0.984815757178173, -0.173602777666664), Vector(0.0000, 0.173602777666664, 0.984815757178173)));
	//std::cout << f_EE << std::endl;

	// Calculate forward position kinematics
	bool kinematics_status;
	kinematics_status = fksolver.JntToCart(jointpositions, cartpos);

	std::cout << "End Effector (Result):" << std::endl;
	std::cout << cartpos << std::endl;

	// Count in Sawyer R_EE
	cartpos = cartpos*f_EE;

	//std::cout << "End Effector (Counting R_EE):" << std::endl;

	if (kinematics_status >= 0) {
		//std::cout << cartpos << std::endl;
		//printf("%s \n", "Success, thanks KDL!");
		return EXIT_SUCCESS;
	}
	else {
		printf("%s \n", "Error: could not calculate forward kinematics :(");
		return EXIT_FAILURE;
	}
}

void wrap2pi(JntArray &q) {
	double PI = 3.141592653589793;
	for (int i = 0; i < q.data.size(); i++){
		while (abs(q(i)) > PI) {
			if (q(i) > PI) {
				q(i) = q(i) - 2 * PI;
			}
			else if (q(i) < -PI) {
				q(i) = q(i) + 2 * PI;
			}
		}
	}
}


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	//Definition of a kinematic chain & add segments to the chain
	KDL::Chain chain;

	chain.addSegment(Segment(Joint(Joint::None), Frame(Vector(0, 0, 0.0800))));
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0810, 0.0500, 0.2370))));
	chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(0.1400, 0.1425, 0))));
	chain.addSegment(Segment(Joint(Joint::RotX), Frame(Vector(0.2600, -0.0420, 0)))); //RotX
	chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(0.125, -0.1265, 0))));
	chain.addSegment(Segment(Joint(Joint::RotX), Frame(Vector(0.2750, 0.0310, 0))));
	chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(0.1100, 0.1053, 0))));
	chain.addSegment(Segment(Joint(Joint::RotX), Frame(Vector(0.13, 0, 0))));

	
	/*chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 1.020))));
	chain.addSegment(Segment(Joint(Joint::RotX), Frame(Vector(0.0, 0.0, 0.480))));
	chain.addSegment(Segment(Joint(Joint::RotX), Frame(Vector(0.0, 0.0, 0.645))));
	chain.addSegment(Segment(Joint(Joint::RotZ)));
	chain.addSegment(Segment(Joint(Joint::RotX), Frame(Vector(0.0, 0.0, 0.120))));
	chain.addSegment(Segment(Joint(Joint::RotZ)));*/

	//Get Inputs
	double *qin = mxGetPr(prhs[0]);
	//Creation of jntarrays:
	JntArray q(chain.getNrOfJoints());
	for (int i = 0; i<chain.getNrOfJoints(); i++)
		q(i) = qin[i];
	JntArray q_init(chain.getNrOfJoints());
	/*q_init(0) = 1.5708 + 0.1;
	q_init(1) = -0.3000;
	q_init(2) = 0.0;
	q_init(3) = 0.58;
	q_init(4) = 0.0;
	q_init(5) = -0.31;
	q_init(6) = 0.15;*/

	/*q_init(0) = 0.99;
	q_init(1) = 0.99;
	q_init(2) = 0.99;
	q_init(3) = 0.99;
	q_init(4) = 0.99;
	q_init(5) = 0.99;
	q_init(6) = 0.99;*/
	
	//// Forward Kinematics
	KDL::Frame cartpos;
	Test_Fwd(chain, cartpos, q);


	//Creation of the solvers:
	ChainFkSolverPos_recursive fksolver(chain);//Forward position solver
	ChainIkSolverVel_pinv iksolverv(chain);//Inverse velocity solver
	ChainIkSolverPos_NR iksolverpos(chain, fksolver, iksolverv,200, 1e-8);//Maximum 100 iterations, stop at accuracy 1e-6

	
	

	// Sawyer special 0 config
	KDL::Frame f_EE = Frame(Rotation(Vector(1.0000, 0.0000, 0.0000), Vector(0.0000, 0.984815757178173, -0.173602777666664), Vector(0.0000, 0.173602777666664, 0.984815757178173)));
	f_EE.M = f_EE.M.Inverse();


	cartpos = cartpos*f_EE;

	std::cout << "End Effector (Calc IK):" << std::endl;
	std::cout << cartpos << std::endl;

	//Set destination frame
	Frame F_dest = cartpos;

	int ret = iksolverpos.CartToJnt(q_init, F_dest, q);


	Test_Fwd(chain, cartpos, q);
	
	/*for (int i = 0; i<q.data.size(); i++)
		std::cout << q(i) << std::endl;*/
	if (ret >= 0) {
		std::cout << "q:" << std::endl;
		wrap2pi(q);
		for(int i = 0; i<q.data.size(); i++)
			std::cout << q(i) << std::endl;
		//mexErrMsgIdAndTxt("mex:error", "Success, thanks KDL!");
		plhs[0] = mxCreateDoubleMatrix(chain.getNrOfJoints(),1,mxREAL);
		double * ptr = (double *)mxGetData(plhs[0]);
		for (int i = 0; i < chain.getNrOfJoints(); i++)
			ptr[i] = q(i);
		
		printf("%s \n", "Success, thanks KDL!");
		//return EXIT_SUCCESS;
	}
	else {
		mexErrMsgIdAndTxt("mex:error", "Error: could not calculate inverse kinematics :(");
		//printf("%s \n", "Error: could not calculate inverse kinematics :(");
		//return EXIT_FAILURE;
	}
	
}

