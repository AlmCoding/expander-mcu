/*
 * time.cpp
 *
 *  Created on: 26 Nov 2023
 *      Author: Alexander L.
 */

#include "util/time.hpp"

namespace util {

void initTimebase() {
  HAL_TIM_Base_Start(&htim2);
}

MicroSeconds getTime() {
  return htim2.Instance->CNT;
}

uint32_t getTimestamp() {
  return htim2.Instance->CNT;
}

}  // namespace util
