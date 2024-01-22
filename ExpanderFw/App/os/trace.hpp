/*
 * trace.hpp
 *
 *  Created on: Jan 21, 2024
 *      Author: Alexander L.
 */

#ifndef OS_TRACE_HPP_
#define OS_TRACE_HPP_

#include "common.hpp"
#include "tx_api.h"

namespace os {

constexpr size_t Trace_BufferSize = 64000;
constexpr size_t Trace_MaxThreadCount = 12;

UINT enableTracing();

}  // namespace os

#endif /* OS_TRACE_HPP_ */
