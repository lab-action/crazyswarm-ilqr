/*
 * utils.h
 *
 *  Created on: Jan 26, 2021
 *      Author: talhakavuncu
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>
#include "iLQR.h"
#include <fstream>

struct stateVector
{
	float x, y, z;
	float roll, pitch, yaw;
	float x_d, y_d, z_d;
	float roll_d, pitch_d, yaw_d;
};

struct pose
{
	float x, y, z;
	float roll, pitch, yaw;
	unsigned long timeStamp;
};

template <size_t st_sz, size_t ac_sz, size_t horizon>
void write_file(std::string &state_path, std::string &input_path,
				typename iLQR<st_sz, ac_sz, horizon>::state_input_trajectory soln)
{
	// iLQR<12,4,horizon>::state_input_trajectory
	std::ofstream state_file(state_path);
	std::ofstream input_file(input_path);
	Eigen::IOFormat CommaInitFmt(Eigen::FullPrecision, Eigen::DontAlignCols, ",", ", ", "", "", "", "");
	for (auto i : soln.first)
	{
		// std::cout << i.transpose().format(CommaInitFmt) << std::endl;
		if (state_file.is_open())
		{
			// Eigen::MatrixXf m = Eigen::MatrixXf::Random(30, 3);
			state_file << i.transpose().format(CommaInitFmt) << std::endl;
			// file << "m" << '\n' << colm(m) << '\n';
		}
	}
	for (auto i : soln.second)
	{
		// std::cout << i.transpose().format(CommaInitFmt) << std::endl;
		if (input_file.is_open())
		{
			// Eigen::MatrixXf m = Eigen::MatrixXf::Random(30, 3);
			input_file << i.transpose().format(CommaInitFmt) << std::endl;
			// file << "m" << '\n' << colm(m) << '\n';
		}
	}
}

void init_datalog(std::ofstream &file, std::string path, std::string name);

void logDroneStates(std::ofstream &file, std::vector<stateVector> states);

// void logOptimizer(std::ofstream& file, std::vector<)

// #include "utils.cpp"

#endif /* UTILS_H_ */
