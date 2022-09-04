/*
 * utils.cpp
 *
 *  Created on: Jan 26, 2021
 *      Author: talhakavuncu
 */
#ifndef UTILS_CPP_
#define UTILS_CPP_

#include <string>
#include <time.h>
#include <chrono>

#include "utils.h"

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime()
{
	time_t now = time(0);
	struct tm tstruct;
	char buf[80];
	tstruct = *localtime(&now);
	// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
	// for more information about date/time format
	strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

	return buf;
}

void init_datalog(std::ofstream &file, std::string path, std::string name)
{

	std::string fileName;

	fileName = path + currentDateTime() + "_" + name + ".csv";

	file.open(fileName);
}

void logDroneStates(std::ofstream &file, std::vector<stateVector> states)
{

	std::string data;

	for (int i = 0; i < states.size(); ++i)
	{
		data += std::to_string(states[i].x) + "," +
				std::to_string(states[i].y) + "," +
				std::to_string(states[i].z) + "," +
				std::to_string(states[i].roll) + "," +
				std::to_string(states[i].pitch) + "," +
				std::to_string(states[i].yaw) + ",";
	}

	const auto p1 = std::chrono::system_clock::now();
	data += std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count());

	data += "\n";

	file << data;
}

#endif
