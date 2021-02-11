/*
 * cost_structure.h
 *
 *  Created on: Jan 21, 2021
 *      Author: talhakavuncu
 */

#ifndef COST_STRUCTURE_H_
#define COST_STRUCTURE_H_

//#include <iostream>
#include <unsupported/Eigen/AdolcForward>
#include <adolc/adolc.h>
#include <Eigen/Dense>
//#include "cost.h"
//typedef Eigen::Matrix<adouble,Eigen::Dynamic,1> vec_type;
//template<size_t state_size,size_t input_size>


namespace Unicycle_Cost{
	typedef Eigen::Matrix<adouble,4,1> state_type_unicycle;
	typedef Eigen::Matrix<adouble,2,1> input_type_unicycle;
	adouble running_cost(const state_type_unicycle & states,const input_type_unicycle & inputs);
	adouble terminal_cost(const state_type_unicycle & states,const input_type_unicycle & inputs);
}



namespace Drone_Cost{
//	adouble C_T,C_D,g,d,I_xx,I_yy,I_zz,mass;
	const double C_T=3.1582*1e-10;
	const double C_D=7.9379*1e-12;
	const double g=9.80665;
	const double d=39.73*1e-3;
	const double I_xx=1.395*1e-5;
	const double I_yy=1.436*1e-5;
	const double I_zz=2.173*1e-5;
	const double mass=0.033;
	typedef Eigen::Matrix<adouble,12,1> state_type_drone;
//	typedef Eigen::Matrix<double,12,1> goal_type_drone;
	typedef Eigen::Matrix<adouble,4,1> input_type_drone;
	typedef Eigen::Matrix<adouble,12*2,1> state_type_drone2;
//	typedef Eigen::Matrix<double,12*2,1> goal_type_drone2;
	typedef Eigen::Matrix<adouble,4*2,1> input_type_drone2;
	adouble running_cost(const state_type_drone & states,const input_type_drone & inputs,const state_type_drone & X_goal);
	adouble terminal_cost(const state_type_drone & states,const input_type_drone & inputs,const state_type_drone & X_goal);

	adouble running_cost2(const state_type_drone2 & states,const input_type_drone2 & inputs,const state_type_drone2 & X_goal);
	adouble terminal_cost2(const state_type_drone2 & states,const input_type_drone2 & inputs,const state_type_drone2 & X_goal);

};

namespace Single_Integrator_Cost
{
	typedef Eigen::Matrix<adouble,3,1> state_tensor;
	typedef Eigen::Matrix<adouble,3,1> input_tensor;
	adouble running_cost(const state_tensor & x, const input_tensor & u,const state_tensor & x_goal);
	adouble terminal_cost(const state_tensor & x, const input_tensor & u,const state_tensor & x_goal);

	typedef Eigen::Matrix<adouble,6,1> state_tensor2;
	typedef Eigen::Matrix<adouble,6,1> input_tensor2;
	adouble running_cost2(const state_tensor2 & x, const input_tensor2 & u,const state_tensor2 & x_goal);
	adouble terminal_cost2(const state_tensor2 & x, const input_tensor2 & u,const state_tensor2 & x_goal);
}



#endif /* COST_STRUCTURE_H_ */
