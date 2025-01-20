/*
 * boot.hpp
 *
 *  Created on: Jan 20, 2025
 *      Author: Alexander L.
 */

#ifndef UTIL_BOOT_HPP_
#define UTIL_BOOT_HPP_

#include "common.hpp"

namespace util {

bool isBootloaderRequested();
void requestBootloader();
void clearBootloaderRequest();
void startBootloader();

}  // namespace util

#endif /* UTIL_BOOT_HPP_ */
