/*
 * ctrlTypes.hpp
 *
 *  Created on: Jul 9, 2023
 *      Author: Alexander L.
 */

#ifndef APP_CTRL_CTRLTYPES_HPP_
#define APP_CTRL_CTRLTYPES_HPP_

#include "common.hpp"

namespace app::ctrl {

typedef void (*RequestSrvCallback)(uint32_t cnt);

}  // namespace app::ctrl

#endif /* APP_CTRL_CTRLTYPES_HPP_ */
