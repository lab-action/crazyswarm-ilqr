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

template<size_t st_sz,size_t ac_sz,size_t horizon>
void write_file(std::string& state_path,std::string& input_path,
		typename iLQR<st_sz,ac_sz,horizon>::state_input_trajectory soln);

#include "utils.cpp"

#endif /* UTILS_H_ */
