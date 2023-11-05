/*
 * c2cpp.hpp
 *
 *  Created on: 18 May 2023
 *      Author: Alexander L.
 */

#ifndef UTIL_C2CPP_HPP_
#define UTIL_C2CPP_HPP_

#ifdef __cplusplus
namespace util {
extern "C" {
#endif

void notifyUsbDeviceActivate(void* cdc_acm);
void notifyUsbDeviceDeactivate();

#ifdef __cplusplus
}  // extern "C"
}  // namespace util
#endif

#endif /* UTIL_C2CPP_HPP_ */
