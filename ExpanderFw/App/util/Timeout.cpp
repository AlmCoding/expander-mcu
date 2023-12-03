/*
 * Timeout.cpp
 *
 *  Created on: 24 Jun 2023
 *      Author: Alexander L.
 */

#include "util/Timeout.hpp"
#include "util/time.hpp"

namespace util {

void Timeout::start(MicroSeconds timeout) {
  timeout_ = timeout;
  start_ = getTime();
}

bool Timeout::isExpired() {
  MicroSeconds time = getTime() - start_;
  if (time >= timeout_) {
    return true;
  }
  return false;
}

MicroSeconds Timeout::remaining() {
  return timeout_ - (getTime() - start_);
}

}  // namespace util
