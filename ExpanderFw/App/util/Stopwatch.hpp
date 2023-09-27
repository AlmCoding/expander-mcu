/*
 * Stopwatch.hpp
 *
 *  Created on: May 31, 2023
 *      Author: Alexander L.
 */

#ifndef UTIL_STOPWATCH_HPP_
#define UTIL_STOPWATCH_HPP_

#include "common.hpp"

namespace util {

class Stopwatch {
 public:
  Stopwatch();
  virtual ~Stopwatch();

  void start();
  MicroSeconds time();
  MicroSeconds stop();

 private:
  bool running_ = false;
  MicroSeconds start_;
  MicroSeconds time_;
};

}  // namespace util

#endif /* UTIL_STOPWATCH_HPP_ */
