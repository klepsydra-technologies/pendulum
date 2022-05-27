// Copyright 2020 Carlos San Vicente
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <utility>
#include <string>

#include <spdlog/spdlog.h>

#include "rclcpp/rclcpp.hpp"
#include "pendulum_controller/pendulum_controller_node.hpp"
#include "pendulum_utils/process_settings.hpp"
#include "pendulum_utils/lifecycle_autostart.hpp"

#include <kpsr_ros2_executor/executor_factory.hpp>

#include <spdlog/sinks/stdout_color_sinks.h>

int main(int argc, char * argv[])
{
    auto kpsrLogger = spdlog::stdout_color_mt("kpsr");
    kpsrLogger->set_pattern("[%c] [%H:%M:%S %f] [%n] [%l] [%t] %v");
    kpsrLogger->set_level(spdlog::level::debug); // Set global log level to info
    spdlog::set_default_logger(kpsrLogger);

  pendulum::utils::ProcessSettings settings;
  if (!settings.init(argc, argv)) {
    return EXIT_FAILURE;
  }

  int32_t ret = 0;
  try {
    // configure process real-time settings
    if (settings.configure_child_threads) {
      // process child threads created by ROS nodes will inherit the settings
      settings.configure_process();
    }
    rclcpp::init(argc, argv);

    // Create a static executor
//    rclcpp::executors::StaticSingleThreadedExecutor exec;

    rclcpp::Executor::SharedPtr exec = kpsr::ros2::ExecutorFactory::createExecutor(kpsr::ros2::QueueSize::_256, false);

    // Create pendulum controller node
    using pendulum::pendulum_controller::PendulumControllerNode;
    const auto controller_node_ptr =
      std::make_shared<PendulumControllerNode>("pendulum_controller");

    exec->add_node(controller_node_ptr->get_node_base_interface());

    // configure process real-time settings
    if (!settings.configure_child_threads) {
      // process child threads created by ROS nodes will NOT inherit the settings
      settings.configure_process();
    }

    if (settings.auto_start_nodes) {
      pendulum::utils::autostart(*controller_node_ptr);
    }

    exec->spin();

    kpsr::ros2::ExecutorFactory::deleteAllExecutors();

    rclcpp::shutdown();

  } catch (const std::exception & e) {
    RCLCPP_INFO(rclcpp::get_logger("pendulum_demo"), e.what());
    ret = 2;
  } catch (...) {
    RCLCPP_INFO(
      rclcpp::get_logger("pendulum_demo"), "Unknown exception caught. "
      "Exiting...");
    ret = -1;
  }
  return ret;
}
