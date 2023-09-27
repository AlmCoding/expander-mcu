/*
 * builder.hpp
 *
 *  Created on: May 4, 2023
 *      Author: Alexander L.
 */

#ifndef OS_BUILDER_HPP_
#define OS_BUILDER_HPP_

#include "tx_api.h"

#ifdef __cplusplus
namespace os {

void enterOs();

extern "C" {
#endif

UINT buildOs(VOID* memory_ptr);

#ifdef __cplusplus
}  // extern "C"
}  // namespace os
#endif

#endif /* OS_BUILDER_HPP_ */
