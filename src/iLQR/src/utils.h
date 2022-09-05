#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "iLQR.h"

namespace fs = std::filesystem;

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
void write_file(const fs::path state_path, const fs::path input_path,
				typename iLQR<st_sz, ac_sz, horizon>::state_input_trajectory soln)
{
	std::ofstream state_file(state_path);
	std::ofstream input_file(input_path);
	Eigen::IOFormat CommaInitFmt(Eigen::FullPrecision, Eigen::DontAlignCols, ",", ", ", "", "", "", "");

	if (!state_file.is_open())
		throw std::invalid_argument("Unable to open file");
	if (!input_file.is_open())
		throw std::invalid_argument("Unable to open file");

	for (auto i : soln.first)
	{
		// std::cout << i.transpose().format(CommaInitFmt) << std::endl;
		// Eigen::MatrixXf m = Eigen::MatrixXf::Random(30, 3);
		state_file << i.transpose().format(CommaInitFmt) << std::endl;
		// file << "m" << '\n' << colm(m) << '\n';
	}
	for (auto i : soln.second)
	{
		// std::cout << i.transpose().format(CommaInitFmt) << std::endl;
		// Eigen::MatrixXf m = Eigen::MatrixXf::Random(30, 3);
		input_file << i.transpose().format(CommaInitFmt) << std::endl;
		// file << "m" << '\n' << colm(m) << '\n';
	}
}

void init_datalog(std::ofstream &file, std::string path, std::string name);

void logDroneStates(std::ofstream &file, std::vector<stateVector> states);

const std::string currentDateTime();

#endif /* UTILS_H_ */
