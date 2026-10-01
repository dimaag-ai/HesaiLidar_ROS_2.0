/************************************************************************************************
  Copyright(C)2023 Hesai Technology Co., Ltd.
  All code in this repository is released under the terms of the following [Modified BSD License.]
  Modified BSD License:
  Redistribution and use in source and binary forms,with or without modification,are permitted 
  provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice,this list of conditions 
   and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice,this list of conditions and 
   the following disclaimer in the documentation and/or other materials provided with the distribution.
  *Neither the names of the University of Texas at Austin,nor Austin Robot Technology,nor the names of 
   other contributors maybe used to endorse or promote products derived from this software without 
   specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGH THOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
  WARRANTIES,INCLUDING,BUT NOT LIMITED TO,THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
  PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
  ANY DIRECT,INDIRECT,INCIDENTAL,SPECIAL,EXEMPLARY,OR CONSEQUENTIAL DAMAGES(INCLUDING,BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;LOSS OF USE,DATA,OR PROFITS;OR BUSINESS INTERRUPTION)HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY,WHETHER IN CONTRACT,STRICT LIABILITY,OR TORT(INCLUDING NEGLIGENCE 
  OR OTHERWISE)ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,EVEN IF ADVISED OF THE POSSIBILITY OF 
  SUCHDAMAGE.
************************************************************************************************/

/*
 * File: composable_hesai_driver.hpp
 * Description: ROS 2 Composable node for Hesai LiDAR driver.
 *
 * Modified by DIMAAG-AI.
 * Copyright (c) 2025, DIMAAG-AI, Inc.
 */

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include "manager/node_manager.h"

class ComposableHesaiDriver : public rclcpp::Node {
 public:
  explicit ComposableHesaiDriver(const rclcpp::NodeOptions& options);
  ~ComposableHesaiDriver() override;

  // Delete copy constructors
  ComposableHesaiDriver(const ComposableHesaiDriver&) = delete;
  ComposableHesaiDriver& operator=(const ComposableHesaiDriver&) = delete;

 private:
  // One-shot timer callback
  rclcpp::TimerBase::SharedPtr init_timer_;

  // Initialize the driver
  void initialize_driver();

  // Background thread: calls NodeManager::Start()
  void driver_loop();

  // Periodic timer callback
  void shutdown_check_callback();

  // Stop the node manager and cancel timers.
  void stop_driver();

  std::shared_ptr<NodeManager> node_manager_;
  rclcpp::TimerBase::SharedPtr shutdown_check_timer_;

  // Background thread running the SDK start/loop
  std::thread driver_thread_;

  bool is_started_;
  YAML::Node config_;

  // Flag to signal the driver loop and shutdown check to stop
  std::atomic<bool> shutdown_requested_{false};
};
