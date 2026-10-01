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
 * File: composable_hesai_driver.cpp
 * Description: ROS 2 Composable node implementation for Hesai LiDAR driver.
 *
 * Modified by DIMAAG-AI.
 * Copyright (c) 2025, DIMAAG-AI, Inc.
 */

#include "composable_hesai_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <yaml-cpp/yaml.h>
#include "Version.h"

ComposableHesaiDriver::ComposableHesaiDriver(const rclcpp::NodeOptions& options)
  : rclcpp::Node("hesai_ros_driver_node", options),
    is_started_(false) {
  std::cout << "-------- Hesai Lidar ROS 2 Composable Node V"
            << VERSION_MAJOR << "." << VERSION_MINOR << "." << VERSION_TINY
            << " --------" << std::endl;

  // Initialize and start the driver.
  init_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(0),
    [this]() {
      init_timer_->cancel();  // one-shot
      initialize_driver();
    });
}

ComposableHesaiDriver::~ComposableHesaiDriver() {
  stop_driver();
}

void ComposableHesaiDriver::stop_driver() {
  shutdown_requested_ = true;

  // Cancel timers
  if (init_timer_) {
    init_timer_->cancel();
  }
  if (shutdown_check_timer_) {
    shutdown_check_timer_->cancel();
  }

  // Stop the node manager (wakes up the driver_loop)
  if (node_manager_ && is_started_) {
    node_manager_->Stop();
    is_started_ = false;
  }

  // Join the background driver thread
  if (driver_thread_.joinable()) {
    driver_thread_.join();
  }

  RCLCPP_INFO(get_logger(), "Driver stopped.");
}

void ComposableHesaiDriver::initialize_driver() {
  // config_path must be provided via the launch file parameter
  declare_parameter<std::string>("config_path", "");
  std::string config_path = get_parameter("config_path").as_string();

  if (config_path.empty()) {
    RCLCPP_FATAL(get_logger(),
      "Required parameter 'config_path' is not set. "
      "Pass it from the launch file");
    throw std::runtime_error("Missing required parameter: config_path");
  }

  // Load configuration
  try {
    config_ = YAML::LoadFile(config_path);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to load config file: %s - %s",
                 config_path.c_str(), e.what());
    throw std::runtime_error("Failed to load config file");
  }

  RCLCPP_INFO(get_logger(), "Loaded configuration from: %s",
    config_path.c_str());

  node_manager_ = std::make_shared<NodeManager>();
  node_manager_->Init(config_, shared_from_this());

  RCLCPP_INFO(get_logger(), "Driver initialized successfully.");

  // Start the SDK in a background thread so the component container's spin
  // thread is never blocked.
  driver_thread_ = std::thread(&ComposableHesaiDriver::driver_loop, this);

  // Periodic shutdown-check timer
  shutdown_check_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ComposableHesaiDriver::shutdown_check_callback, this));
}

void ComposableHesaiDriver::driver_loop() {
  if (!node_manager_) {
    RCLCPP_ERROR(get_logger(), "NodeManager not initialized.");
    return;
  }

  node_manager_->Start();
  is_started_ = true;

  RCLCPP_INFO(get_logger(), "Driver started. Waiting for data...");

  while (rclcpp::ok() && !shutdown_requested_.load()) {
    if (node_manager_->IsPlayEnded()) {
      RCLCPP_INFO(get_logger(), "Data playback ended.");
      break;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  if (node_manager_ && is_started_) {
    node_manager_->Stop();
    is_started_ = false;
  }

  RCLCPP_INFO(get_logger(), "Driver loop exited.");
}

void ComposableHesaiDriver::shutdown_check_callback() {
  if (!rclcpp::ok() || shutdown_requested_.load()) {
    RCLCPP_INFO(get_logger(), "Shutdown detected. Stopping driver...");
    stop_driver();
  }
}

// Register as a composable component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ComposableHesaiDriver)
