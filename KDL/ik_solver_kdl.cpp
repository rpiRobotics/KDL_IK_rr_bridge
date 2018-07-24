#include "stdafx.h"


// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// 2018 Modification by JDC. Newton method IK solver for MATLAB

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
	// Handle inputs
	// ikTgt to cartpos
	KDL::Rotation rot_tgt;
	KDL::Vector trans_tgt;
	double *pr0;
	pr0 = mxGetPr(prhs[0]);
	for(int i = 0; i<3; i++)
		for(int j = 0; j<3; j++)
			rot_tgt.data[3*i+j] = pr0[4*j+i];
	for(int j = 0; j<3; j++)
		trans_tgt(j) = pr0[4*j+3];
	KDL::Frame cartpos = Frame(rot_tgt,trans_tgt);//Target frame	
	
	//P and H to chain
	mwSize ii, jj, mxszNumJoints;
	double *pr_P = mxGetPr(prhs[1]);
	double *pr_H = mxGetPr(prhs[2]);
	mxszNumJoints = mxGetN(prhs[2]);
	
	KDL::Chain chain;
	KDL::Vector trans_jt;
	for(int j = 0; j<3; j++)
		trans_jt(j) = pr_P[j];
	chain.addSegment(Segment(Joint(Joint::None), Frame(trans_jt)));
    
    double *pr4 = mxGetPr(prhs[4]);//Fixed Joints
	JntArray q_fixed(mxszNumJoints);
    JntArray flag_fixed(mxszNumJoints);
    int nFixed = 0;
    for (int i = 0; i<mxszNumJoints; i++){
		flag_fixed(i) = pr4[i];
        q_fixed(i) = pr4[mxszNumJoints+i];
    }
    
	for (ii = 0; ii<mxszNumJoints; ii++){
		for (jj = 0; jj<3; jj++){
			trans_jt(jj) = pr_P[3*(ii+1) + jj];
		}
        if(flag_fixed(ii) == 1){
            nFixed++;
//             mexPrintf("\nfixed is %d\n", ii+1);
            chain.addSegment(Segment(Joint(Joint::None), Frame(trans_jt)));
        }else{
//             Joint jt = Joint(Joint::RotAxis);
//             jt.JointAxis() = Joint("Joint",trans_jt,Vector(pr_H[3*ii],pr_H[3*ii+1],pr_H[3*ii+2]),Joint::RotAxis);
            chain.addSegment(Segment(Joint("Joint",Vector(0,0,0),Vector(pr_H[3*ii],pr_H[3*ii+1],pr_H[3*ii+2]),Joint::RotAxis), Frame(trans_jt)));
//             Vector(pr_H[3*ii],pr_H[3*ii+1],pr_H[3*ii+2]);
        }
//             if(pr_H[3*ii] == 1){
// 			chain.addSegment(Segment(Joint(Joint::RotX), Frame(trans_jt)));
// 		}else if(pr_H[3*ii + 1] == 1){
// 			chain.addSegment(Segment(Joint(Joint::RotY), Frame(trans_jt)));
// 		}else{
// 			chain.addSegment(Segment(Joint(Joint::RotZ), Frame(trans_jt)));
// 		}
	}
    
    //q_rslt = ik_solver_kdl(ikTgt,P,H,R_EE,q_fixed,q_init_guess);
    KDL::Rotation rot_EE;
	//KDL::Vector trans_EE; //Right now only rotation
	double *pr3;
	pr3 = mxGetPr(prhs[3]);
	for(int i = 0; i<3; i++)
		for(int j = 0; j<3; j++)
			rot_EE.data[3*i+j] = pr3[3*j+i];	
	// Special 0 config
	KDL::Frame f_EE = Frame(rot_EE);
	f_EE.M = f_EE.M.Inverse();
    
    //Joint Angle Initial Guess
    double *pr5 = mxGetPr(prhs[5]);
//     mexPrintf("\nnumber of joints %d\n", chain.getNrOfJoints());
	JntArray q_init(chain.getNrOfJoints());
    for (int i = 0; i<chain.getNrOfJoints(); i++)
		q_init(i) = pr5[i]; 

	//Creation of the solvers:
	ChainFkSolverPos_recursive fksolver(chain);//Forward position solver
	ChainIkSolverVel_pinv iksolverv(chain);//Inverse velocity solver
	ChainIkSolverPos_NR iksolverpos(chain, fksolver, iksolverv,200, 1e-8);//Maximum 100 iterations, stop at accuracy 1e-6

	//Creation of jntarrays:
	JntArray q(chain.getNrOfJoints());    
	
	cartpos = cartpos*f_EE;

	//Set destination frame
	Frame F_dest = cartpos;
	
	//Compute IK
	int ret = iksolverpos.CartToJnt(q_init, F_dest, q);
	
	if (ret >= 0) {
		//wrap2pi(q);
		plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
		double * ptr0 = (double *)mxGetData(plhs[0]);
		ptr0[0] = 1;
		for(int i = 0; i<q.data.size(); i++)
			std::cout << q(i) << std::endl;
		plhs[1] = mxCreateDoubleMatrix(chain.getNrOfJoints(),1,mxREAL);
		double * ptr = (double *)mxGetData(plhs[1]);
		for (int i = 0; i < chain.getNrOfJoints(); i++)
			ptr[i] = q(i);
	}
	else {
		plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
		double * ptr0 = (double *)mxGetData(plhs[0]);
		ptr0[0] = 0;
		plhs[1] = mxCreateDoubleMatrix(chain.getNrOfJoints(),1,mxREAL);
		double * ptr = (double *)mxGetData(plhs[1]);
		for (int i = 0; i < chain.getNrOfJoints(); i++)
			ptr[i] = 0;
	}
	
}

