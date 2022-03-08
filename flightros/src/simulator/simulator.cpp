#include "flightros/simulator/simulator.hpp"

using namespace flightlib;

namespace flightros {

Simulator::Simulator()
: cmd_{.0, {-Gz / 4, -Gz / 4, -Gz / 4, -Gz / 4}}, time_{.0} {
  // quad initialization
  quad_ptr_ = std::make_shared<Quadrotor>();
}

Simulator::~Simulator() {}

void Simulator::cmdCallback(const Cmd::ConstPtr& msg) {
  cmd_.t = msg->time.toSec();
  cmd_.thrusts << msg->thrusts[0], msg->thrusts[1], msg->thrusts[2], msg->thrusts[3];
  quad_ptr_->setCommand(cmd_);
}

// service callback
bool Simulator::resetCallback(ResetSim::Request  &req,
             ResetSim::Response &res)
{
  time_ = 0;
  return true;
}

State Simulator::genState(const QuadState& qs) {
  State st;
  st.time = ros::Time(time_);
  st.pose.position.x = qs.x[QS::POSX];
  st.pose.position.y = qs.x[QS::POSY];
  st.pose.position.z = qs.x[QS::POSZ];
  st.pose.orientation.w = qs.x[QS::ATTW];
  st.pose.orientation.x = qs.x[QS::ATTX];
  st.pose.orientation.y = qs.x[QS::ATTY];
  st.pose.orientation.z = qs.x[QS::ATTZ];
  st.twist.linear.x = qs.x[QS::VELX];
  st.twist.linear.y = qs.x[QS::VELY];
  st.twist.linear.z = qs.x[QS::VELZ];
  st.twist.angular.x = qs.x[QS::OMEX];
  st.twist.angular.y = qs.x[QS::OMEY];
  st.twist.angular.z = qs.x[QS::OMEZ];

  return st;
}

void Simulator::run() {
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");

  ros::Rate rate(2.0);

  // service
  ros::ServiceServer service = nh.advertiseService("reset_sim", &Simulator::resetCallback, this);

  // publisher
  image_transport::Publisher rgb_pub;
  ros::Publisher state_pub;

  // define quadsize scale (for unity visualization only)
  Vector<3> quad_size(0.5, 0.5, 0.5);
  quad_ptr_->setSize(quad_size);
  QuadState quad_state;

  //
  std::shared_ptr<RGBCamera> rgb_camera = std::make_shared<RGBCamera>();

  // Flightmare(Unity3D)
  // std::shared_ptr<UnityBridge> unity_bridge_ptr = UnityBridge::getInstance();
  // SceneID scene_id{UnityScene::WAREHOUSE};
  bool unity_ready{false};

  // initialize publishers
  image_transport::ImageTransport it(pnh);
  rgb_pub = it.advertise("/rgb", 1);
  state_pub = nh.advertise<State>("state", 1);

  // subscriber
  ros::Subscriber sub = nh.subscribe("cmd", 1, &Simulator::cmdCallback, this);

  // Flightmare
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC << std::endl;
  rgb_camera->setFOV(90);
  rgb_camera->setWidth(640);
  rgb_camera->setHeight(360);
  rgb_camera->setRelPose(B_r_BC, R_BC);
  quad_ptr_->addRGBCamera(rgb_camera);

  // initialization
  quad_state.setZero();
  quad_ptr_->reset(quad_state);

  // connect unity
  // unity_bridge_ptr->addQuadrotor(quad_ptr_);
  // unity_ready = unity_bridge_ptr->connectUnity(scene_id);

  FrameID frame_id = 0;
  Scalar dt{.5};
  quad_ptr_->setCommand(cmd_);

  // while (ros::ok() && unity_ready) {
  while (ros::ok()) {
    quad_ptr_->run(dt);

    // unity_bridge_ptr->getRender(frame_id);
    // unity_bridge_ptr->handleOutput();
    quad_ptr_->getState(&quad_state);
    state_pub.publish(genState(quad_state));

    // ros::Time timestamp = ros::Time::now();
    // cv::Mat img;
    // rgb_camera->getRGBImage(img);
    // sensor_msgs::ImagePtr rgb_msg =
    //   cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    // rgb_msg->header.stamp = timestamp;
    // rgb_pub.publish(rgb_msg);

    ros::spinOnce();

    frame_id += 1;
    time_ += dt;
    rate.sleep();
  }
}
}  // namespace flightros

int main(int argc, char** argv) {
  ros::init(argc, argv, "flight_pilot");

  flightros::Simulator sim;
  sim.run();

  return 0;
}
