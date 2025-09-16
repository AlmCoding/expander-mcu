/*
 * DacIrq.cpp
 *
 *  Created on: Aug 15, 2025
 *      Author: Alexander L.
 */

#include "hal/dac/DacIrq.hpp"
#include "etl/algorithm.h"      // etl::max
#include "etl/error_handler.h"  // etl::ETL_ASSERT()
#include "util/debug.hpp"

#define DEBUG_ENABLE_DAC_IRQ 1
#if ((DEBUG_ENABLE_DAC_IRQ == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][DacIrq]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][DacIrq]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][DacIrq]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace hal::dac {

void DacIrq::setTimerHandle(TIM_HandleTypeDef* timer_handle) {
  ETL_ASSERT(timer_handle != nullptr, ETL_ERROR(0));
  timer_handle_ = timer_handle;
}

void DacIrq::setDacCtrlHandle(DacController* dac_ctrl_handle) {
  ETL_ASSERT(dac_ctrl_handle != nullptr, ETL_ERROR(0));
  dac_ctrl_handle_ = dac_ctrl_handle;
}

Status_t DacIrq::config(bool config_ch0, uint32_t sampling_rate_ch0,  //
                        bool config_ch1, uint32_t sampling_rate_ch1) {
  Status_t status = Status_t::Ok;
  ETL_ASSERT(timer_handle_ != nullptr, ETL_ERROR(0));

  if (config_ch0 == false && config_ch1 == false) {
    DEBUG_ERROR("No channel to configure!");
    return Status_t::Error;
  }

  if (config_ch0 == true) {
    DEBUG_INFO("Config (ch0) sampling_rate: %d", sampling_rate_ch0);
    sampling_rate_ch0_ = sampling_rate_ch0;
    disableDacChannel(DacId::Dac0);
  }

  if (config_ch1 == true) {
    DEBUG_INFO("Config (ch1) sampling_rate: %d", sampling_rate_ch1);
    sampling_rate_ch1_ = sampling_rate_ch1;
    disableDacChannel(DacId::Dac1);
  }

  DEBUG_INFO("Config DacIrq [OK]");
  return status;
}

void DacIrq::enableDacChannel(DacId dac_id) {
  ETL_ASSERT(timer_handle_ != nullptr, ETL_ERROR(0));

  if (dac_id == DacId::Dac0) {
    if (enable_ch0_ == true) {
      DEBUG_WARN("DAC ch0 already enabled!");
      return;
    }
    DEBUG_INFO("Enable DAC ch0 [OK]");
    enable_ch0_ = true;

  } else if (dac_id == DacId::Dac1) {
    if (enable_ch1_ == true) {
      DEBUG_WARN("DAC ch1 already enabled!");
      return;
    }
    DEBUG_INFO("Enable DAC ch1 [OK]");
    enable_ch1_ = true;

  } else {
    ETL_ASSERT(false, ETL_ERROR(0));
  }

  if (timer_running_ == true) {
    return;  // Timer already running
  }

  // Configure timer for the higher sampling rate
  uint32_t sampling_rate = 0;
  if (sampling_rate_ch0_ >= sampling_rate_ch1_) {
    sampling_rate = sampling_rate_ch0_;
    sampling_ratio_ = sampling_rate_ch0_ / sampling_rate_ch1_;
    base_channel_id_ = DacId::Dac0;
    ratio_channel_id_ = DacId::Dac1;
    enable_base_channel_ = &enable_ch0_;
    enable_ratio_channel_ = &enable_ch1_;
  } else {
    sampling_rate = sampling_rate_ch1_;
    sampling_ratio_ = sampling_rate_ch1_ / sampling_rate_ch0_;
    base_channel_id_ = DacId::Dac1;
    ratio_channel_id_ = DacId::Dac0;
    enable_base_channel_ = &enable_ch1_;
    enable_ratio_channel_ = &enable_ch0_;
  }
  ratio_counter_ = sampling_ratio_;

  uint32_t timer_period = TimerClk / sampling_rate;  // - 1;

  if (HAL_TIM_Base_Stop(timer_handle_) != HAL_OK) {
    DEBUG_ERROR("Failed to stop timer for DacIrq!");
    return;
  }

  // Update the period
  timer_handle_->Init.Period = timer_period;

  if (HAL_TIM_Base_Init(timer_handle_) != HAL_OK) {
    DEBUG_ERROR("Failed to reinitialize timer for DacIrq!");
    return;
  }

  timer_running_ = true;
  if (HAL_TIM_Base_Start_IT(timer_handle_) != HAL_OK) {
    DEBUG_ERROR("Failed to start timer for DacIrq!");
    return;
  }
  DEBUG_INFO("Start timer for DacIrq [OK]");
}

void DacIrq::disableDacChannel(DacId dac_id) {
  ETL_ASSERT(timer_handle_ != nullptr, ETL_ERROR(0));

  if (dac_id == DacId::Dac0) {
    DEBUG_INFO("Disable DAC channel 0");
    enable_ch0_ = false;
  } else if (dac_id == DacId::Dac1) {
    DEBUG_INFO("Disable DAC channel 1");
    enable_ch1_ = false;
  } else {
    ETL_ASSERT(false, ETL_ERROR(0));
  }

  // Stop timer when both channels are disabled
  if (enable_ch0_ == false && enable_ch1_ == false && timer_running_ == true) {
    DEBUG_INFO("Both DAC channels disabled, stopping timer");
    HAL_TIM_Base_Stop_IT(timer_handle_);
    timer_running_ = false;
  }
}

void DacIrq::timerExpired() {
  ETL_ASSERT(dac_ctrl_handle_ != nullptr, ETL_ERROR(0));
  ETL_ASSERT(enable_ratio_channel_ != nullptr, ETL_ERROR(0));
  ETL_ASSERT(timer_running_ == true, ETL_ERROR(0));
  ratio_counter_--;

  if (ratio_counter_ == 0) {
    ratio_counter_ = sampling_ratio_;
    if (*enable_ratio_channel_ == true) {
      dac_ctrl_handle_->updateSample(ratio_channel_id_, DacUpdate::No);
    }
  }

  if (*enable_base_channel_ == true) {
    dac_ctrl_handle_->updateSample(base_channel_id_, DacUpdate::All);
  }
}

}  // namespace hal::dac
