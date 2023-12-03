/*
 * time.hpp
 *
 *  Created on: 26 Nov 2023
 *      Author: Alexander L.
 */

#ifndef UTIL_TIME_HPP_
#define UTIL_TIME_HPP_

#include "common.hpp"
#include "tim.h"

namespace util {

void initTimebase();
MicroSeconds getTime();
uint32_t getTimestamp();

}  // namespace util

#endif /* UTIL_TIME_HPP_ */
