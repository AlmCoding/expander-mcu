/*
 * Timeout.hpp
 *
 *  Created on: 24 Jun 2023
 *      Author: Alexander L.
 */

#ifndef UTIL_TIMEOUT_HPP_
#define UTIL_TIMEOUT_HPP_

#include "common.hpp"

namespace util {

class Timeout {
 public:
  Timeout() = default;
  virtual ~Timeout() = default;

  void start(MicroSeconds timeout);
  bool isExpired();
  MicroSeconds remaining();

 private:
  MicroSeconds timeout_;
  MicroSeconds start_;
};

}  // namespace util

#endif /* UTIL_TIMEOUT_HPP_ */
