
#pragma once

// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/common/command.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

// flightros
#include "flightros/Cmd.h"
#include "flightros/State.h"
#include "flightros/ResetSim.h"

using namespace flightlib;

namespace flightros {
class Simulator {
 public:
  Simulator();
  ~Simulator();

  // callbacks
  void cmdCallback(const Cmd::ConstPtr& msg);
  bool resetCallback(ResetSim::Request  &req, ResetSim::Response &res);

  void run();
  State genState(const QuadState& qs);


 private:
  // unity quadrotor
  std::shared_ptr<Quadrotor> quad_ptr_;
  Command cmd_;
  float time_;
};
}  // namespace flightros