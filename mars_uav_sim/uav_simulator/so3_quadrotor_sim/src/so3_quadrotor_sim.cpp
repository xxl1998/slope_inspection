#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "so3_controller/so3_controller.hpp"
#include "so3_quadrotor/quadrotor_dynamics.hpp"

std::shared_ptr<so3_quadrotor::Quadrotor> quadrotorPtr_;
so3_quadrotor::Control control_;
so3_quadrotor::Cmd cmd_;
ros::Publisher odom_pub_, imu_pub_;
ros::Timer simulation_timer;

// parameters
int simulation_rate_ = 1e3;
int odom_rate_ = 400;
// msgs
nav_msgs::Odometry odom_msg_;
sensor_msgs::Imu imu_msg_;

// controller global variable
std::shared_ptr<so3_controller::SO3Controller> so3ControlPtr_;
ros::Subscriber position_cmd_sub_;
ros::Timer state_timer_;
quadrotor_msgs::SO3Command so3cmd_;

bool position_cmd_received_flag_ = false;
Eigen::Vector3d des_pos_;
double des_yaw_;

void so3cmd_callback(const quadrotor_msgs::SO3Command& cmd_msg) {
  cmd_.force[0] = cmd_msg.force.x;
  cmd_.force[1] = cmd_msg.force.y;
  cmd_.force[2] = cmd_msg.force.z;
  cmd_.qx = cmd_msg.orientation.x;
  cmd_.qy = cmd_msg.orientation.y;
  cmd_.qz = cmd_msg.orientation.z;
  cmd_.qw = cmd_msg.orientation.w;
  cmd_.kR[0] = cmd_msg.kR[0];
  cmd_.kR[1] = cmd_msg.kR[1];
  cmd_.kR[2] = cmd_msg.kR[2];
  cmd_.kOm[0] = cmd_msg.kOm[0];
  cmd_.kOm[1] = cmd_msg.kOm[1];
  cmd_.kOm[2] = cmd_msg.kOm[2];
  cmd_.corrections[0] = cmd_msg.aux.kf_correction;
  cmd_.corrections[1] = cmd_msg.aux.angle_corrections[0];
  cmd_.corrections[2] = cmd_msg.aux.angle_corrections[1];
  cmd_.current_yaw = cmd_msg.aux.current_yaw;
  cmd_.use_external_yaw = cmd_msg.aux.use_external_yaw;
}

void controller_timer_callback(const ros::TimerEvent& event) {
  if (position_cmd_received_flag_) {
    position_cmd_received_flag_ = false;
  } else {
    Eigen::Vector3d des_vel(0, 0, 0);
    Eigen::Vector3d des_acc(0, 0, 0);
    Eigen::Vector3d kx(5.7, 5.7, 6.2);
    Eigen::Vector3d kv(3.4, 3.4, 4.0);
    so3ControlPtr_->calculateControl(des_pos_, des_vel, des_acc, des_yaw_, 0,
                                     kx, kv);
    const Eigen::Vector3d& f = so3ControlPtr_->getF();
    const Eigen::Quaterniond& q = so3ControlPtr_->getQ();
    so3cmd_.force.x = f(0);
    so3cmd_.force.y = f(1);
    so3cmd_.force.z = f(2);
    so3cmd_.orientation.x = q.x();
    so3cmd_.orientation.y = q.y();
    so3cmd_.orientation.z = q.z();
    so3cmd_.orientation.w = q.w();
    so3cmd_callback(so3cmd_);
  }
}

void position_cmd_callback(
    const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
  position_cmd_received_flag_ = true;
  Eigen::Vector3d des_pos(msg->position.x, msg->position.y, msg->position.z);
  Eigen::Vector3d des_vel(msg->velocity.x, msg->velocity.y, msg->velocity.z);
  Eigen::Vector3d des_acc(msg->acceleration.x, msg->acceleration.y,
                          msg->acceleration.z);
  Eigen::Vector3d kx(msg->kx[0], msg->kx[1], msg->kx[2]);
  Eigen::Vector3d kv(msg->kv[0], msg->kv[1], msg->kv[2]);
  if (msg->kx[0] == 0) {
    kx(0) = 5.7;
    kx(1) = 5.7;
    kx(2) = 6.2;
    kv(0) = 3.4;
    kv(1) = 3.4;
    kv(2) = 4.0;
  }
  double des_yaw = msg->yaw;
  double des_yaw_dot = msg->yaw_dot;
  so3ControlPtr_->calculateControl(des_pos, des_vel, des_acc, des_yaw,
                                   des_yaw_dot, kx, kv);
  const Eigen::Vector3d& f = so3ControlPtr_->getF();
  const Eigen::Quaterniond& q = so3ControlPtr_->getQ();
  so3cmd_.force.x = f(0);
  so3cmd_.force.y = f(1);
  so3cmd_.force.z = f(2);
  so3cmd_.orientation.x = q.x();
  so3cmd_.orientation.y = q.y();
  so3cmd_.orientation.z = q.z();
  so3cmd_.orientation.w = q.w();
  so3cmd_callback(so3cmd_);
  // store last des_pos and des_yaw
  des_pos_ = des_pos;
  des_yaw_ = des_yaw;
}

void quadrotor_timer_callback(const ros::TimerEvent& event) {
  static tf::TransformBroadcaster tf_br;
  auto last_control = control_;
  control_ = quadrotorPtr_->getControl(cmd_);
  for (size_t i = 0; i < 4; ++i) {
    if (std::isnan(control_.rpm[i])) control_.rpm[i] = last_control.rpm[i];
  }
  quadrotorPtr_->setInput(control_.rpm[0], control_.rpm[1], control_.rpm[2],
                          control_.rpm[3]);
  quadrotorPtr_->step(1.0 / simulation_rate_);
  static ros::Time next_odom_pub_time = ros::Time::now();
  ros::Time tnow = ros::Time::now();
  if (tnow >= next_odom_pub_time) {
    next_odom_pub_time += ros::Duration(1.0 / odom_rate_);
    const Eigen::Vector3d& pos = quadrotorPtr_->getPos();
    const Eigen::Vector3d& vel = quadrotorPtr_->getVel();
    const Eigen::Vector3d& acc = quadrotorPtr_->getAcc();
    const Eigen::Quaterniond& quat = quadrotorPtr_->getQuat();
    const Eigen::Vector3d& omega = quadrotorPtr_->getOmega();

    // tf
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pos.x(), pos.y(), pos.z()));
    transform.setRotation(
        tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
    tf_br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "body"));
    // odom
    odom_msg_.header.stamp = tnow;
    odom_msg_.pose.pose.position.x = pos(0);
    odom_msg_.pose.pose.position.y = pos(1);
    odom_msg_.pose.pose.position.z = pos(2);
    odom_msg_.pose.pose.orientation.x = quat.x();
    odom_msg_.pose.pose.orientation.y = quat.y();
    odom_msg_.pose.pose.orientation.z = quat.z();
    odom_msg_.pose.pose.orientation.w = quat.w();

    odom_msg_.twist.twist.linear.x = vel(0);
    odom_msg_.twist.twist.linear.y = vel(1);
    odom_msg_.twist.twist.linear.z = vel(2);

    odom_msg_.twist.twist.angular.x = omega(0);
    odom_msg_.twist.twist.angular.y = omega(1);
    odom_msg_.twist.twist.angular.z = omega(2);

    so3ControlPtr_->setPos(pos);
    so3ControlPtr_->setVel(vel);
    so3cmd_.aux.current_yaw = tf::getYaw(odom_msg_.pose.pose.orientation);

    odom_pub_.publish(odom_msg_);

    // imu
    imu_msg_.header.stamp = tnow;
    imu_msg_.orientation.x = quat.x();
    imu_msg_.orientation.y = quat.y();
    imu_msg_.orientation.z = quat.z();
    imu_msg_.orientation.w = quat.w();

    imu_msg_.angular_velocity.x = omega(0);
    imu_msg_.angular_velocity.y = omega(1);
    imu_msg_.angular_velocity.z = omega(2);

    imu_msg_.linear_acceleration.x = acc[0];
    imu_msg_.linear_acceleration.y = acc[1];
    imu_msg_.linear_acceleration.z = acc[2];

    so3ControlPtr_->setAcc(acc);

    imu_pub_.publish(imu_msg_);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "so3_quadrotor_sim");
  ros::NodeHandle nh("~");

  // quadrotor parameters
  double init_x, init_y, init_z, init_yaw;
  nh.getParam("init_x", init_x);
  nh.getParam("init_y", init_y);
  nh.getParam("init_z", init_z);
  nh.getParam("init_yaw", init_yaw);
  // config of quadrotor
  so3_quadrotor::Config config;
  nh.getParam("g", config.g);
  nh.getParam("mass", config.mass);
  double Ixx, Iyy, Izz;
  nh.getParam("Ixx", Ixx);
  nh.getParam("Iyy", Iyy);
  nh.getParam("Izz", Izz);
  config.J = Eigen::Vector3d(Ixx, Iyy, Izz).asDiagonal();
  nh.getParam("kf", config.kf);
  double prop_radius;
  nh.getParam("prop_radius", prop_radius);
  config.km = 0.07 * (3 * prop_radius) * config.kf;
  nh.getParam("arm_length", config.arm_length);
  nh.getParam("motor_time_constant", config.motor_time_constant);
  nh.getParam("max_rpm", config.max_rpm);
  nh.getParam("min_rpm", config.min_rpm);
  nh.getParam("simulation_rate", simulation_rate_);
  nh.getParam("odom_rate", odom_rate_);

  // controller parameters
  double mass, g;
  nh.getParam("mass", mass);
  nh.getParam("g", g);
  so3ControlPtr_ = std::make_shared<so3_controller::SO3Controller>(mass, g);
  so3cmd_.header.frame_id = "world";
  nh.getParam("gains/rot/x", so3cmd_.kR[0]);
  nh.getParam("gains/rot/y", so3cmd_.kR[1]);
  nh.getParam("gains/rot/z", so3cmd_.kR[2]);
  nh.getParam("gains/ang/x", so3cmd_.kOm[0]);
  nh.getParam("gains/ang/y", so3cmd_.kOm[1]);
  nh.getParam("gains/ang/z", so3cmd_.kOm[2]);
  nh.getParam("corrections/z", so3cmd_.aux.kf_correction);
  nh.getParam("corrections/r", so3cmd_.aux.angle_corrections[0]);
  nh.getParam("corrections/p", so3cmd_.aux.angle_corrections[1]);

  // quadrotor init
  quadrotorPtr_ = std::make_shared<so3_quadrotor::Quadrotor>(config);
  quadrotorPtr_->setPos(Eigen::Vector3d(init_x, init_y, init_z));
  quadrotorPtr_->setYpr(Eigen::Vector3d(init_yaw, 0, 0));
  double rpm = sqrt(config.mass * config.g / 4 / config.kf);
  quadrotorPtr_->setRpm(Eigen::Vector4d(rpm, rpm, rpm, rpm));
  Eigen::Quaterniond quat =
      uav_utils::ypr_to_quaternion(Eigen::Vector3d(init_yaw, 0, 0));
  cmd_.force[2] = config.mass * config.g;
  cmd_.qw = quat.w();
  cmd_.qx = quat.x();
  cmd_.qy = quat.y();
  cmd_.qz = quat.z();

  // controller init
  so3cmd_.aux.enable_motors = true;
  so3cmd_.aux.use_external_yaw = false;

  // quadrotor ROS
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 100);
  imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu", 10);
  simulation_timer = nh.createTimer(ros::Duration(1.0 / simulation_rate_),
                                    &quadrotor_timer_callback);

  // controller ROS
  position_cmd_sub_ = nh.subscribe("position_cmd", 10, &position_cmd_callback,
                                   ros::TransportHints().tcpNoDelay());
  state_timer_ = nh.createTimer(ros::Duration(0.1), &controller_timer_callback);

  odom_msg_.header.frame_id = "world";
  imu_msg_.header.frame_id = "world";

  ros::AsyncSpinner spinner(0);
  spinner.start();

  ros::Rate r(10);
  while (ros::ok()) {
    r.sleep();
  }

  ros::waitForShutdown();
  return 0;
}
