/*
 * DacIrq.hpp
 *
 *  Created on: Aug 15, 2025
 *      Author: Alexander L.
 */

#ifndef HAL_DAC_DACIRQ_HPP_
#define HAL_DAC_DACIRQ_HPP_

#include "common.hpp"
#include "enum/magic_enum.hpp"
#include "hal/dac/DacConfig.hpp"
#include "hal/dac/DacController.hpp"

namespace hal::dac {

class DacIrq {
 private:
  constexpr static size_t DacCount = magic_enum::enum_count<DacId>();
  constexpr static uint32_t TimerClk = 240000000;  // 240 MHz

 public:
  // Deleted copy constructor and assignment operator to enforce singleton
  DacIrq(const DacIrq&) = delete;
  DacIrq& operator=(const DacIrq&) = delete;

  static DacIrq& getInstance() {
    static DacIrq instance;
    return instance;
  }

  void setTimerHandle(TIM_HandleTypeDef* timer_handle);
  void setDacCtrlHandle(DacController* dac_ctrl_handle);
  Status_t config(bool config_ch0 = true, uint32_t sampling_rate_ch0 = 1,  //
                  bool config_ch1 = true, uint32_t sampling_rate_ch1 = 1);
  void enableDacChannel(DacId dac_id);
  void disableDacChannel(DacId dac_id);
  void timerExpired();

 private:
  DacIrq() = default;

  TIM_HandleTypeDef* timer_handle_ = nullptr;
  DacController* dac_ctrl_handle_ = nullptr;

  uint32_t sampling_rate_ch0_ = 1;
  uint32_t sampling_rate_ch1_ = 1;
  bool enable_ch0_ = false;
  bool enable_ch1_ = false;
  bool* enable_ratio_channel_ = nullptr;

  DacId base_channel_id_ = DacId::Dac0;
  DacId ratio_channel_id_ = DacId::Dac1;
  uint32_t sampling_ratio_ = 1;
  uint32_t ratio_counter_ = 1;
  bool timer_running_ = false;
};

}  // namespace hal::dac

#endif /* HAL_DAC_DACIRQ_HPP_ */
