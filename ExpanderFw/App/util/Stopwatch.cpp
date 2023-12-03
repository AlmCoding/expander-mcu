/*
 * Stopwatch.cpp
 *
 *  Created on: May 31, 2023
 *      Author: Alexander L.
 */

#include "util/Stopwatch.hpp"
#include "util/time.hpp"

namespace util {

void Stopwatch::start() {
  running_ = true;
  start_ = getTime();
}

MicroSeconds Stopwatch::time() {
  if (running_ == true) {
    return getTime() - start_;
  }
  return time_;
}

MicroSeconds Stopwatch::stop() {
  time_ = getTime() - start_;
  running_ = false;
  return time_;
}

}  // namespace util
