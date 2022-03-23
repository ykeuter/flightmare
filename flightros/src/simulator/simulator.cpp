#include "flightros/simulator/simulator.hpp"

using namespace flightlib;

namespace flightros {

Simulator::Simulator()
: cmd_{.0, {.0, .0, .0, .0}} {
  // quad initialization
  quad_ptr_ = std::make_shared<Quadrotor>();
  YAML::Node cfg_ = YAML::LoadFile(getenv("FLIGHTMARE_PATH") +
    std::string("/flightlib/configs/quadrotor_env.yaml"));
  QuadrotorDynamics dynamics;
  dynamics.updateParams(cfg_);
  quadrotor_ptr_->updateDynamics(dynamics);
  Scalar mass = quadrotor_ptr_->getMass();
  thrust_mean_ = (-mass * Gz) / 4;
  thrust_std_ = (-mass * 2 * Gz) / 4;
}

Simulator::~Simulator() {}

void Simulator::cmdCallback(const Cmd::ConstPtr& msg) {
  cmd_.t = msg->time.toSec();
  cmd_.thrusts << msg->thrusts[0], msg->thrusts[1], msg->thrusts[2], msg->thrusts[3];
  cmd_.thrusts *= thrust_std_;
  cmd_.thrusts += thrust_mean_;
  quad_ptr_->setCommand(cmd_);
}

// service callback
bool Simulator::resetCallback(ResetSim::Request  &req,
             ResetSim::Response &res)
{
  quad_state_.setZero();
  quad_state_.x[QS::POSX] = st.pose.position.x;
  quad_state_.x[QS::POSY] = st.pose.position.y;
  quad_state_.x[QS::POSZ] = st.pose.position.z;
  quad_state_.x[QS::ATTW] = st.pose.orientation.w;
  quad_state_.x[QS::ATTX] = st.pose.orientation.x;
  quad_state_.x[QS::ATTY] = st.pose.orientation.y;
  quad_state_.x[QS::ATTZ] = st.pose.orientation.z;
  quad_state_.x[QS::VELX] = st.twist.linear.x;
  quad_state_.x[QS::VELY] = st.twist.linear.y;
  quad_state_.x[QS::VELZ] = st.twist.linear.z;
  quad_state_.x[QS::OMEX] = st.twist.angular.x;
  quad_state_.x[QS::OMEY] = st.twist.angular.y;
  quad_state_.x[QS::OMEZ] = st.twist.angular.z;
  quadrotor_ptr_->reset(quad_state_);
  cmd_.t = 0.0;
  cmd_.thrusts.setZero();
  quad_ptr_->setCommand(cmd_);
  return true;
}

State Simulator::genState() {
  State st;
  st.time = ros::Time(quad_state_.t);
  st.pose.position.x = quad_state_.x[QS::POSX];
  st.pose.position.y = quad_state_.x[QS::POSY];
  st.pose.position.z = quad_state_.x[QS::POSZ];
  st.pose.orientation.w = quad_state_.x[QS::ATTW];
  st.pose.orientation.x = quad_state_.x[QS::ATTX];
  st.pose.orientation.y = quad_state_.x[QS::ATTY];
  st.pose.orientation.z = quad_state_.x[QS::ATTZ];
  st.twist.linear.x = quad_state_.x[QS::VELX];
  st.twist.linear.y = quad_state_.x[QS::VELY];
  st.twist.linear.z = quad_state_.x[QS::VELZ];
  st.twist.angular.x = quad_state_.x[QS::OMEX];
  st.twist.angular.y = quad_state_.x[QS::OMEY];
  st.twist.angular.z = quad_state_.x[QS::OMEZ];

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
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);

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
    quad_ptr_->getState(&quad_state_);
    state_pub.publish(genState());

    // ros::Time timestamp = ros::Time::now();
    // cv::Mat img;
    // rgb_camera->getRGBImage(img);
    // sensor_msgs::ImagePtr rgb_msg =
    //   cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    // rgb_msg->header.stamp = timestamp;
    // rgb_pub.publish(rgb_msg);

    ros::spinOnce();

    frame_id += 1;
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
