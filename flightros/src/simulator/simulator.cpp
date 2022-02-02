
// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/common/command.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

using namespace flightlib;

int main(int argc, char *argv[]) {
  // initialize ROS
  ros::init(argc, argv, "simulator");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");
  ros::Rate(50.0);

  // publisher
  image_transport::Publisher rgb_pub;

  // unity quadrotor
  std::shared_ptr<Quadrotor> quad_ptr = std::make_shared<Quadrotor>();
  // define quadsize scale (for unity visualization only)
  Vector<3> quad_size(0.5, 0.5, 0.5);
  quad_ptr->setSize(quad_size);
  QuadState quad_state;

  //
  std::shared_ptr<RGBCamera> rgb_camera = std::make_shared<RGBCamera>();

  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr = UnityBridge::getInstance();
  SceneID scene_id{UnityScene::WAREHOUSE};
  bool unity_ready{false};

  // initialize publishers
  image_transport::ImageTransport it(pnh);
  rgb_pub = it.advertise("/rgb", 1);

  // Flightmare
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC << std::endl;
  rgb_camera->setFOV(90);
  rgb_camera->setWidth(640);
  rgb_camera->setHeight(360);
  rgb_camera->setRelPose(B_r_BC, R_BC);
  quad_ptr->addRGBCamera(rgb_camera);

  // initialization
  quad_state.setZero();
  quad_ptr->reset(quad_state);

  // connect unity
  unity_bridge_ptr->addQuadrotor(quad_ptr);
  unity_ready = unity_bridge_ptr->connectUnity(scene_id);

  FrameID frame_id = 0;
  Scalar dt{.2};
  Command cmd{.0, {3., 3., 3., 3.}};
  quad_ptr->setCommand(cmd);

  while (ros::ok() && unity_ready) {
    // quad_state.x[QS::POSZ] += 0.1;

    // quad_ptr->setState(quad_state);
    quad_ptr->run(dt);

    unity_bridge_ptr->getRender(frame_id);
    unity_bridge_ptr->handleOutput();

    cv::Mat img;

    ros::Time timestamp = ros::Time::now();

    rgb_camera->getRGBImage(img);
    sensor_msgs::ImagePtr rgb_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    rgb_msg->header.stamp = timestamp;
    rgb_pub.publish(rgb_msg);

    frame_id += 1;
  }

  return 0;
}
