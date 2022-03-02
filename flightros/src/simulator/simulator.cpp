#include "flightros/simulator/simulator.hpp"

using namespace flightlib;

namespace flightros {

Simulator::Simulator()
: cmd_{.0, {-Gz / 4, -Gz / 4, -Gz / 4, -Gz / 4}} {
  // quad initialization
  quad_ptr_ = std::make_shared<Quadrotor>();
}

Simulator::~Simulator() {}

void Simulator::cmdCallback(const Cmd::ConstPtr& msg) {
  cmd_.t = msg->time;
  cmd_.thrusts << msg->thrusts[0], msg->thrusts[1], msg->thrusts[2], msg->thrusts[3];
  quad_ptr_->setCommand(cmd_);
}

QuadObs Simulator::genObs(const QuadState& qs) {
  QuadObs qo;
  // qo.position = geometry_msgs::Vector3(quad_state.x[QS::POSX], quad_state.x[QS::POSY], quad_state.x[QS::POSZ]);
  qo.position.x = .5;
  qo.position.y = .5;
  qo.position.z = .5;
  qo.velocity.x = quad_state.x[QS::VELX];
  qo.velocity.y = quad_state.x[QS::VELY];
  qo.velocity.z = quad_state.x[QS::VELZ];
  qo.angular_velocity.x = quad_state.x[QS::OMEX];
  qo.angular_velocity.y = quad_state.x[QS::OMEY];
  qo.angular_velocity.z = quad_state.x[QS::OMEZ];
  
  Vector<3> euler = quad_state.q().toRotationMatrix().eulerAngles(2, 1, 0);
  qo.euler_zyx.x = euler.x;
  qo.euler_zyx.y = euler.y;
  qo.euler_zyx.z = euler.z;
  Vector<3> euler = quad_state.q().toRotationMatrix().eulerAngles(2, 1, 0);
  qo.euler_zyx = geometry_msgs::Vector3(euler.x, euler.y, euler.z);
  return qo;
}

void Simulator::run() {
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");

  ros::Rate(50.0);

  // publisher
  image_transport::Publisher rgb_pub;
  ros::Publisher obs_pub;

  // define quadsize scale (for unity visualization only)
  Vector<3> quad_size(0.5, 0.5, 0.5);
  quad_ptr_->setSize(quad_size);
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
  obs_pub = nh.advertise<QuadObs>("quad_obs", 1);

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
  unity_bridge_ptr->addQuadrotor(quad_ptr_);
  unity_ready = unity_bridge_ptr->connectUnity(scene_id);

  FrameID frame_id = 0;
  Scalar dt{.02};
  quad_ptr_->setCommand(cmd_);

  while (ros::ok() && unity_ready) {
    // quad_state.x[QS::POSZ] += 0.1;

    // quad_ptr->setState(quad_state);
    quad_ptr_->run(dt);

    unity_bridge_ptr->getRender(frame_id);
    unity_bridge_ptr->handleOutput();
    
    obs_pub.publish(genObs(quad_state));

    ros::Time timestamp = ros::Time::now();
    cv::Mat img;
    rgb_camera->getRGBImage(img);
    sensor_msgs::ImagePtr rgb_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    rgb_msg->header.stamp = timestamp;
    rgb_pub.publish(rgb_msg);

    ros::spinOnce();

    frame_id += 1;
  }
}
}  // namespace flightros

int main(int argc, char** argv) {
  ros::init(argc, argv, "flight_pilot");

  flightros::Simulator sim;
  sim.run();

  return 0;
}
