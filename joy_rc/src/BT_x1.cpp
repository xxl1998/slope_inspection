#include <mavros_msgs/RCIn.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

ros::Publisher rc_pub;
mavros_msgs::RCIn rc_msg;
bool rc_flag = false;
bool main_channel_reverse = true;

std::vector<double> joy_axes;
std::vector<int> joy_buttoms;

double Clamp(double x, double low, double high) {
  if (x < low) return low;
  if (x > high) return high;
  return x;
}

void ChannelReverseRemap(const int joy_id, const int rc_id) {
  const double rc = 1500.0 - joy_axes[joy_id] * 500.0;
  rc_msg.channels[rc_id] = static_cast<uint16_t>(Clamp(rc, 1000.0, 2000.0));
}

void ChannelRemap(const int joy_id, const int rc_id) {
  const double rc = 1500.0 + joy_axes[joy_id] * 500.0;
  rc_msg.channels[rc_id] = static_cast<uint16_t>(Clamp(rc, 1000.0, 2000.0));
}

void JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
  rc_flag = true;
  joy_axes.resize(msg->axes.size());
  joy_buttoms.resize(msg->buttons.size());

  for (int i = 0; i < msg->axes.size(); i++) {
    joy_axes[i] = msg->axes[i];
  }
  for (int i = 0; i < msg->buttons.size(); i++) {
    joy_buttoms[i] = msg->buttons[i];
  }

  // Direct remap:
  // joy axis 0-5 -> rc channel 0-5, joy axis 6 -> rc channel 10.
  for (int i = 0; i <= 5 && i < static_cast<int>(joy_axes.size()); i++) {
    if (i < 4 && main_channel_reverse) {
      ChannelReverseRemap(i, i);
    } else {
      ChannelRemap(i, i);
    }
  }
  if (joy_axes.size() > 6) {
    ChannelRemap(6, 10);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "BT_X1_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param("main_channel_reverse", main_channel_reverse, true);

  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>(
      "joy", 1, &JoyCallback, ros::TransportHints().tcpNoDelay());
  rc_pub = nh.advertise<mavros_msgs::RCIn>("rc", 1);

  rc_msg.channels.resize(12);
  for (int i = 0; i < rc_msg.channels.size(); i++) {
    rc_msg.channels[i] = 1050;
  }

  ros::Rate rate(50);
  while (ros::ok()) {
    ros::spinOnce();
    if (rc_flag) {
      rc_msg.header.stamp = ros::Time::now();
      rc_pub.publish(rc_msg);
    }
    rate.sleep();
  }

  return 0;
}
