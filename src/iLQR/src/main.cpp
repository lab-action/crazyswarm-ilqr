#include <math.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <iterator>
#include <vector>
#include <thread>

#include <adolc/adolc.h>
#include <adolc/adouble.h>
#include <Eigen/Dense>
//#include <unsupported/Eigen/AutoDiff>
#include <unsupported/Eigen/AdolcForward>
//#include <unsupported/Eigen/AdolcSupport>

#include "cost.h"
#include "dynamics.h"
#include "iLQR.h"
#include "utils.h"
// #include "DataStreamClient.h"

namespace fs = std::filesystem;

unsigned int tag1(1), tag2(2), tag3(3), tag4(4), tag5(5), tag6(6), tag7(7), tag8(8), tag9(9);

constexpr size_t horizon = 10;
constexpr int total_steps = 10;
constexpr int total_subs_steps = 1;
const fs::path src_dir = fs::current_path();

double time_step = 0.1;

void single_unicycle()
{
	cost<4, 2> running_cost_uni(Unicycle_Cost::running_cost, tag1);
	cost<4, 2> terminal_cost_uni(Unicycle_Cost::terminal_cost, tag2);
	dynamics<4, 2> dynamics_uni(Unicycle_Dynamics::dynamics, tag3, time_step);

	iLQR<4, 2, horizon>::input_trajectory u_init_uni;
	iLQR<4, 2, horizon> uni_solver(running_cost_uni, terminal_cost_uni, dynamics_uni);

	clock_t t_start, t_end;
	double seconds;
	int execution_steps = 1;

	Eigen::Matrix<double, 2, 1> u_uni = Eigen::Matrix<double, 2, 1>::Zero();
	Eigen::Matrix<double, 4, 1> x0_uni;
	Eigen::Matrix<double, 4, 1> x_goal_uni;

	x0_uni << -5, -5, 0, 0;
	x_goal_uni << 5, 5, 0, 0;
	std::fill(std::begin(u_init_uni), std::end(u_init_uni), u_uni);

	iLQR<4, 2, horizon>::state_input_trajectory soln_uni;
	iLQR<4, 2, total_steps>::state_input_trajectory soln_uni_rhc;

	uni_solver.set_MPC(u_init_uni);
	soln_uni_rhc.first[0] = x0_uni;

	fs::path state_path = fs::current_path() / "states.txt";
	fs::path input_path = fs::current_path() / "inputs.txt";

	t_start = clock();
	for (int i = 0; i < total_steps; ++i)
	{
		std::cout << "iteration " << i << std::endl;

		soln_uni = uni_solver.run_MPC(soln_uni_rhc.first[i], x_goal_uni, total_steps,
									  total_subs_steps, execution_steps);
		soln_uni_rhc.first[i + 1] = soln_uni.first[1];
		soln_uni_rhc.second[i] = soln_uni.second[0];
	}
	t_end = clock();
	seconds = 1 / ((double)(t_end - t_start) / CLOCKS_PER_SEC / total_steps);
	printf("Time: %f s\n", seconds);

	write_file<4, 2, total_steps>(state_path, input_path, soln_uni_rhc);
	std::cout << "finished" << std::endl;
}

void two_unicycles()
{
	cost<8, 4> running_cost_uni2(Unicycle_Cost::running_cost2, tag1);
	cost<8, 4> terminal_cost_uni2(Unicycle_Cost::terminal_cost2, tag2);
	dynamics<8, 4> dynamics_uni2(Unicycle_Dynamics::dynamics_2, tag3, time_step);

	iLQR<8, 4, horizon> uni2_solver(running_cost_uni2, terminal_cost_uni2, dynamics_uni2);
	iLQR<8, 4, horizon>::input_trajectory u_init_uni2;

	clock_t t_start, t_end;
	double seconds;
	int execution_steps = 1;

	Eigen::Matrix<double, 4, 1> u_uni2 = Eigen::Matrix<double, 4, 1>::Random() * 0.1;
	Eigen::Matrix<double, 8, 1> x0_uni2;
	Eigen::Matrix<double, 8, 1> x_goal_uni2;
	x0_uni2 << 0, 0.1, 0, 0, 10, 0, M_PI, 0;
	x_goal_uni2 << 10, 0, 0, 0, 0, 0, M_PI, 0;

	for (auto &item : u_init_uni2)
		item = Eigen::Matrix<double, 4, 1>::Random() * 0.1;

	std::fill(std::begin(u_init_uni2), std::end(u_init_uni2), u_uni2);
	iLQR<8, 4, horizon>::state_input_trajectory soln_uni2;
	iLQR<8, 4, total_steps>::state_input_trajectory soln_uni2_rhc;

	uni2_solver.set_MPC(u_init_uni2);
	soln_uni2_rhc.first[0] = x0_uni2;

	fs::path state_path = fs::current_path() / "states.txt";
	fs::path input_path = fs::current_path() / "inputs.txt";

	t_start = clock();
	for (int i = 0; i < total_steps; ++i)
	{
		std::cout << "iteration " << i << std::endl;
		soln_uni2 = uni2_solver.run_MPC(soln_uni2_rhc.first[i], x_goal_uni2,
										total_steps, total_subs_steps, execution_steps);
		soln_uni2_rhc.first[i + 1] = soln_uni2.first[1];
		soln_uni2_rhc.second[i] = soln_uni2.second[0];
	}
	t_end = clock();
	seconds = 1 / ((double)(t_end - t_start) / CLOCKS_PER_SEC / total_steps);
	printf("Time: %f s\n", seconds);

	write_file<8, 4, total_steps>(state_path, input_path, soln_uni2_rhc);
	std::cout << "finished" << std::endl;
}

void single_drone()
{
	cost<12, 4> running_cost_dr(Drone_Cost::running_cost, tag1);
	cost<12, 4> terminal_cost_dr(Drone_Cost::terminal_cost, tag2);
	dynamics<12, 4> dynamics_dr(Drone_Dynamics::dynamics, tag3, time_step);

	iLQR<12, 4, horizon> drone_solver(running_cost_dr, terminal_cost_dr, dynamics_dr);
	Eigen::Matrix<double, 4, 1> u_drone = Eigen::Matrix<double, 4, 1>::Ones();
	Eigen::Matrix<double, 12, 1> X0_drone;
	Eigen::Matrix<double, 12, 1> X_goal_drone;

	iLQR<12, 4, horizon>::input_trajectory u_init_drone;
	iLQR<12, 4, horizon>::state_input_trajectory soln_drone;
	iLQR<12, 4, total_steps>::state_input_trajectory soln_drone_rhc;

	X0_drone << 0, -50, 10, M_PI / 20, 0, 0, 0, 0, 0, 0, 0, 0;
	X_goal_drone << 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	u_drone = u_drone * sqrt(Drone_Dynamics::mass * Drone_Dynamics::g / (4 * Drone_Dynamics::C_T));
	std::fill(std::begin(u_init_drone), std::end(u_init_drone), u_drone);

	clock_t t_start, t_end;
	double seconds;
	int execution_steps = 1;

	drone_solver.set_MPC(u_init_drone);
	soln_drone_rhc.first[0] = X0_drone;

	fs::path state_path = fs::current_path() / "states.txt";
	fs::path input_path = fs::current_path() / "inputs.txt";

	t_start = clock();
	for (int i = 0; i < total_steps; ++i)
	{
		std::cout << "iteration " << i << std::endl;
		soln_drone = drone_solver.run_MPC(soln_drone_rhc.first[i], X_goal_drone,
										  total_steps, total_subs_steps, execution_steps);
		soln_drone_rhc.first[i + 1] = soln_drone.first[1];
		soln_drone_rhc.second[i] = soln_drone.second[0];
	}
	t_end = clock();
	seconds = 1 / ((double)(t_end - t_start) / CLOCKS_PER_SEC / total_steps);
	printf("Time: %f s\n", seconds);

	write_file<12, 4, total_steps>(state_path, input_path, soln_drone_rhc);
	std::cout << "finished" << std::endl;
}

void two_drones()
{
	cost<12 * 2, 4 * 2> running_cost_dr2(Drone_Cost::running_cost2, tag4);
	cost<12 * 2, 4 * 2> terminal_cost_dr2(Drone_Cost::terminal_cost2, tag5);
	dynamics<12 * 2, 4 * 2> dynamics_dr2(Drone_Dynamics::dynamics_2, tag6, time_step);
	iLQR<12 * 2, 4 * 2, horizon>::input_trajectory u_init_drone2;

	iLQR<12 * 2, 4 * 2, horizon>::state_input_trajectory soln_drone2;
	iLQR<12 * 2, 4 * 2, total_steps>::state_input_trajectory soln_drone2_rhc;
	iLQR<12 * 2, 4 * 2, horizon> drone2_solver(running_cost_dr2, terminal_cost_dr2, dynamics_dr2);

	Eigen::Matrix<double, 4 * 2, 1> u_drone2 = Eigen::Matrix<double, 4 * 2, 1>::Ones();
	Eigen::Matrix<double, 12 * 2, 1> X0_drone2;
	Eigen::Matrix<double, 12 * 2, 1> X_goal_drone2;

	X0_drone2 << -2.5, -2.5, -2.5, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		2.5, -2.5, -2.5, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	// X0_drone2<<-1,0,2, 0,0,0, 0,0,0, 0,0,0,
	// 1,0,2, 0,0,0, 0,0,0, 0,0,0;

	X_goal_drone2 << 2.5, 2.5, 2.5, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		-2.5, 2.5, 2.5, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	// X_goal_drone2<<1,0,2, 0,0,0, 0,0,0, 0,0,0,
	// 		-1,0,2, 0,0,0, 0,0,0, 0,0,0;

	u_drone2 = u_drone2 * sqrt(Drone_Dynamics::mass * Drone_Dynamics::g / (4 * Drone_Dynamics::C_T));
	std::fill(std::begin(u_init_drone2), std::end(u_init_drone2), u_drone2);

	clock_t t_start, t_end;
	double seconds;
	int execution_steps = 1;

	drone2_solver.set_MPC(u_init_drone2);
	soln_drone2_rhc.first[0] = X0_drone2;

	fs::path state_path = fs::current_path() / "states.txt";
	fs::path input_path = fs::current_path() / "inputs.txt";

	t_start = clock();
	for (int i = 0; i < total_steps; ++i)
	{
		std::cout << "iteration " << i << std::endl;
		soln_drone2 = drone2_solver.run_MPC(soln_drone2_rhc.first[i], X_goal_drone2,
											total_steps, total_subs_steps, execution_steps);
		soln_drone2_rhc.first[i + 1] = soln_drone2.first[1];
		soln_drone2_rhc.second[i] = soln_drone2.second[0];
	}
	t_end = clock();
	seconds = 1 / ((double)(t_end - t_start) / CLOCKS_PER_SEC / total_steps);
	printf("Time: %f s\n", seconds);

	write_file<12 * 2, 4 * 2, total_steps>(state_path, input_path, soln_drone2_rhc);
	std::cout << "finished" << std::endl;
}

void single_integrator()
{

	cost<3, 3> integrator_running_cost(Single_Integrator_Cost::running_cost, tag7);
	cost<3, 3> integrator_terminal_cost(Single_Integrator_Cost::terminal_cost, tag8);
	dynamics<3, 3> integrator_dynamics(Single_Integrator_3D::dynamics, tag9, time_step);
	iLQR<3, 3, horizon> integrator_solver(integrator_running_cost, integrator_terminal_cost, integrator_dynamics);

	iLQR<3, 3, horizon>::state_input_trajectory soln_integrator;
	iLQR<3, 3, horizon>::input_trajectory u_init_integrator;
	iLQR<3, 3, total_steps>::state_input_trajectory soln_integrator_rhc;

	Eigen::Matrix<double, 3, 1> u_integrator = Eigen::Matrix<double, 3, 1>::Zero();
	Eigen::Matrix<double, 3, 1> x0_integrator = Eigen::Matrix<double, 3, 1>::Zero();
	Eigen::Matrix<double, 3, 1> x_goal_integrator;

	x0_integrator << 0, 0, 0.5;
	x_goal_integrator << 2, 2, 2;
	// x0_integrator+=0.1*Eigen::Matrix<double,3,1>::Random();

	std::fill(std::begin(u_init_integrator), std::end(u_init_integrator), u_integrator);
	integrator_solver.set_MPC(u_init_integrator);
	soln_integrator_rhc.first[0] = x0_integrator;

	clock_t t_start, t_end;
	double seconds;
	int execution_steps = 1;

	fs::path state_path = fs::current_path() / "states.txt";
	fs::path input_path = fs::current_path() / "inputs.txt";

	t_start = clock();
	for (int i = 0; i < total_steps; ++i)
	{
		std::cout << "iteration " << i << std::endl;
		soln_integrator = integrator_solver.run_MPC(soln_integrator_rhc.first[i],
													x_goal_integrator, total_steps,
													total_subs_steps, execution_steps);
		soln_integrator_rhc.first[i + 1] = soln_integrator.first[1];
		soln_integrator_rhc.second[i] = soln_integrator.second[0];
	}
	t_end = clock();
	seconds = 1 / ((double)(t_end - t_start) / CLOCKS_PER_SEC / total_steps);
	printf("Time: %f s\n", seconds);

	write_file<3, 3, total_steps>(state_path, input_path, soln_integrator_rhc);
}

void two_single_integrators()
{
	cost<6, 6> integrator2_running_cost(Single_Integrator_Cost::running_cost2, tag7);
	cost<6, 6> integrator2_terminal_cost(Single_Integrator_Cost::terminal_cost2, tag8);
	dynamics<6, 6> integrator2_dynamics(Single_Integrator_3D::dynamics2, tag9, time_step);

	iLQR<6, 6, horizon>::input_trajectory u_init_integrator2;
	iLQR<6, 6, horizon> integrator2_solver(integrator2_running_cost, integrator2_terminal_cost, integrator2_dynamics);

	Eigen::Matrix<double, 6, 1> u_integrator2 = Eigen::Matrix<double, 6, 1>::Zero();
	Eigen::Matrix<double, 6, 1> x0_integrator2;
	Eigen::Matrix<double, 6, 1> x_goal_integrator2;
	x0_integrator2 << -1, 0, 2.001, 1, 0, 2;
	x_goal_integrator2 << 1, 0, 2, -1, 0, 2;

	iLQR<6, 6, horizon>::state_input_trajectory soln_integrator2;
	iLQR<6, 6, total_steps>::state_input_trajectory soln_integrator2_rhc;

	std::fill(std::begin(u_init_integrator2), std::end(u_init_integrator2), u_integrator2);
	integrator2_solver.set_MPC(u_init_integrator2);
	soln_integrator2_rhc.first[0] = x0_integrator2;

	clock_t t_start, t_end;
	double seconds;
	int execution_steps = 1;

	fs::path state_path = fs::current_path() / "states.txt";
	fs::path input_path = fs::current_path() / "inputs.txt";

	t_start = clock();
	for (int i = 0; i < total_steps; ++i)
	{
		std::cout << "iteration " << i << std::endl;
		soln_integrator2 = integrator2_solver.run_MPC(soln_integrator2_rhc.first[i],
													  x_goal_integrator2, total_steps, total_subs_steps, execution_steps);
		soln_integrator2_rhc.first[i + 1] = soln_integrator2.first[1];
		soln_integrator2_rhc.second[i] = soln_integrator2.second[0];
	}
	t_end = clock();
	seconds = 1 / ((double)(t_end - t_start) / CLOCKS_PER_SEC / total_steps);
	printf("Time: %f s\n", seconds);

	write_file<6, 6, total_steps>(state_path, input_path, soln_integrator2_rhc);
}

void double_integrator()
{
	cost<6, 3> integrator_running_cost(Double_Integrator_Cost::running_cost, tag7);
	cost<6, 3> integrator_terminal_cost(Double_Integrator_Cost::terminal_cost, tag8);
	dynamics<6, 3> integrator_dynamics(Double_Integrator_3D::dynamics, tag9, time_step);

	iLQR<6, 3, horizon>::input_trajectory u_init_integrator;
	iLQR<6, 3, horizon> integrator_solver(integrator_running_cost, integrator_terminal_cost, integrator_dynamics);

	Eigen::Matrix<double, 3, 1> u_integrator = Eigen::Matrix<double, 3, 1>::Zero();
	Eigen::Matrix<double, 6, 1> x0_integrator;
	Eigen::Matrix<double, 6, 1> x_goal_integrator;

	x0_integrator << 0, 0, 0.5, 0, 0, 0;
	// x0_integrator += 0.1 * Eigen::Matrix<double, 3, 1>::Random();
	x_goal_integrator << 2, 2, 2, 0, 0, 0;

	std::fill(std::begin(u_init_integrator), std::end(u_init_integrator), u_integrator);
	iLQR<6, 3, horizon>::state_input_trajectory soln_integrator;
	iLQR<6, 3, total_steps>::state_input_trajectory soln_integrator_rhc;

	soln_integrator_rhc.first[0] = x0_integrator;
	integrator_solver.set_MPC(u_init_integrator);

	clock_t t_start, t_end;
	double seconds;
	int execution_steps = 1;

	fs::path state_path = fs::current_path() / "states.txt";
	fs::path input_path = fs::current_path() / "inputs.txt";

	t_start = clock();
	for (int i = 0; i < total_steps; ++i)
	{
		std::cout << "iteration " << i << std::endl;
		soln_integrator = integrator_solver.run_MPC(soln_integrator_rhc.first[i],
													x_goal_integrator, total_steps,
													total_subs_steps, execution_steps);
		soln_integrator_rhc.first[i + 1] = soln_integrator.first[1];
		soln_integrator_rhc.second[i] = soln_integrator.second[0];
	}
	t_end = clock();
	seconds = 1 / ((double)(t_end - t_start) / CLOCKS_PER_SEC / total_steps);
	printf("Time: %f s\n", seconds);

	write_file<6, 3, total_steps>(state_path, input_path, soln_integrator_rhc);
}

void first_order_drone()
{
	cost<6, 6> drone_running_cost(Drone_First_Order_Cost::running_cost, tag7);
	cost<6, 6> drone_terminal_cost(Drone_First_Order_Cost::terminal_cost, tag8);
	dynamics<6, 6> drone_dynamics(Drone_First_Order_Dynamics::dynamics, tag9, time_step);

	iLQR<6, 6, horizon>::input_trajectory u_init_drone;
	iLQR<6, 6, horizon> drone_solver(drone_running_cost, drone_terminal_cost, drone_dynamics);

	Eigen::Matrix<double, 6, 1> u_drone = Eigen::Matrix<double, 6, 1>::Zero();
	Eigen::Matrix<double, 6, 1> x0_drone;
	Eigen::Matrix<double, 6, 1> x_goal_drone;

	x0_drone << 0, 0, 0.5, 0, 0, 0;
	x_goal_drone << 2, 2, 2, 0, 0, 0;

	iLQR<6, 6, horizon>::state_input_trajectory soln_drone;
	iLQR<6, 6, total_steps>::state_input_trajectory soln_drone_rhc;

	std::fill(std::begin(u_init_drone), std::end(u_init_drone), u_drone);
	drone_solver.set_MPC(u_init_drone);

	clock_t t_start, t_end;
	double seconds;
	int execution_steps = 1;

	soln_drone_rhc.first[0] = x0_drone;

	fs::path state_path = fs::current_path() / "states.txt";
	fs::path input_path = fs::current_path() / "inputs.txt";

	t_start = clock();
	for (int i = 0; i < total_steps; ++i)
	{
		double scale = 0.01;
		auto term1 = soln_drone_rhc.first[i] + scale * Eigen::Matrix<double, 6, 1>::Random();
		soln_drone = drone_solver.run_MPC(term1,
										  x_goal_drone, total_steps, total_subs_steps, execution_steps);
		soln_drone_rhc.first[i + 1] = soln_drone.first[1];
		soln_drone_rhc.second[i] = soln_drone.second[0];
	}
	t_end = clock();
	seconds = 1 / ((double)(t_end - t_start) / CLOCKS_PER_SEC / total_steps);
	printf("Time: %f s\n", seconds);

	write_file<6, 6, total_steps>(state_path, input_path, soln_drone_rhc);
}

void two_first_order_drones()
{
	cost<12, 12> drone2_running_cost(Drone_First_Order_Cost::running_cost2, tag7);
	cost<12, 12> drone2_terminal_cost(Drone_First_Order_Cost::terminal_cost2, tag8);
	dynamics<12, 12> drone2_dynamics(Drone_First_Order_Dynamics::dynamics_2, tag9, time_step);

	iLQR<12, 12, horizon>::input_trajectory u_init_drone2;
	iLQR<12, 12, horizon> drone2_solver(drone2_running_cost, drone2_terminal_cost, drone2_dynamics);

	Eigen::Matrix<double, 12, 1> u_drone2 = Eigen::Matrix<double, 12, 1>::Zero();
	Eigen::Matrix<double, 12, 1> x0_drone2;
	Eigen::Matrix<double, 12, 1> x_goal_drone2;

	iLQR<12, 12, horizon>::state_input_trajectory soln_drone2;
	iLQR<12, 12, total_steps>::state_input_trajectory soln_drone2_rhc;

	u_drone2 = u_drone2 * sqrt(Drone_Dynamics::mass * Drone_Dynamics::g / (4 * Drone_Dynamics::C_T));
	std::fill(std::begin(u_init_drone2), std::end(u_init_drone2), u_drone2);

	x0_drone2 << -3, 0, 2.1, 0, 0, 0, 3, 0, 2, 0, 0, 0;
	x_goal_drone2 << 3, 0, 2, 0, 0, 0, -3, 0, 2, 0, 0, 0;

	drone2_solver.set_MPC(u_init_drone2);
	soln_drone2_rhc.first[0] = x0_drone2;

	clock_t t_start, t_end;
	double seconds;
	int execution_steps = 1;

	fs::path state_path = fs::current_path() / "states.txt";
	fs::path input_path = fs::current_path() / "inputs.txt";

	t_start = clock();
	for (int i = 0; i < total_steps; ++i)
	{
		soln_drone2 = drone2_solver.run_MPC(soln_drone2_rhc.first[i], x_goal_drone2,
											total_steps, total_subs_steps,
											execution_steps);
		soln_drone2_rhc.first[i + 1] = soln_drone2.first[1];
		soln_drone2_rhc.second[i] = soln_drone2.second[0];
	}
	t_end = clock();
	seconds = 1 / ((double)(t_end - t_start) / CLOCKS_PER_SEC / total_steps);
	printf("Time: %f s\n", seconds);

	write_file<12, 12, total_steps>(state_path, input_path, soln_drone2_rhc);
}

int main()
{
	// Set a pseudorandom seed for reproducibility of results.
	srand((unsigned int)time(0));

	// single_unicycle();
	// two_unicycles();
	// single_drone();
	// two_drones();
	// single_integrator();
	// two_single_integrators();
	// first_order_drone();
	two_first_order_drones();

	return 0;
};
