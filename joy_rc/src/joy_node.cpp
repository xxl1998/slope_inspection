#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace {
constexpr double kAxisMax = 32767.0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "joy_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string dev = "/dev/input/js0";
  double publish_rate_hz = 100.0;
  pnh.param<std::string>("dev", dev, dev);
  pnh.param<double>("publish_rate", publish_rate_hz, publish_rate_hz);

  int fd = -1;
  uint8_t axis_count = 8;
  uint8_t button_count = 12;

  ros::Publisher joy_pub = nh.advertise<sensor_msgs::Joy>("joy", 10);
  std::vector<float> axes(axis_count, 0.0f);
  std::vector<int32_t> buttons(button_count, 0);

  ros::Rate rate(std::max(1.0, publish_rate_hz));
  while (ros::ok()) {
    if (fd < 0) {
      fd = open(dev.c_str(), O_RDONLY | O_NONBLOCK);
      if (fd < 0) {
        ROS_WARN_THROTTLE(2.0, "Waiting for joystick device %s: %s",
                          dev.c_str(), std::strerror(errno));
        ros::spinOnce();
        rate.sleep();
        continue;
      }

      if (ioctl(fd, JSIOCGAXES, &axis_count) < 0) {
        ROS_WARN("Cannot read axis count from %s, using default 8", dev.c_str());
        axis_count = 8;
      }
      if (ioctl(fd, JSIOCGBUTTONS, &button_count) < 0) {
        ROS_WARN("Cannot read button count from %s, using default 12", dev.c_str());
        button_count = 12;
      }
      axes.assign(axis_count, 0.0f);
      buttons.assign(button_count, 0);
      ROS_INFO("joy_node opened %s with %u axes and %u buttons",
               dev.c_str(), axis_count, button_count);
    }

    js_event e{};
    bool updated = false;
    while (read(fd, &e, sizeof(e)) == sizeof(e)) {
      const uint8_t type = e.type & ~JS_EVENT_INIT;
      if (type == JS_EVENT_AXIS && e.number < axes.size()) {
        axes[e.number] = static_cast<float>(e.value / kAxisMax);
        updated = true;
      } else if (type == JS_EVENT_BUTTON && e.number < buttons.size()) {
        buttons[e.number] = (e.value != 0) ? 1 : 0;
        updated = true;
      }
    }

    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      ROS_WARN_THROTTLE(2.0, "Joystick read error on %s: %s. Reconnecting...",
                        dev.c_str(), std::strerror(errno));
      close(fd);
      fd = -1;
    }

    if (updated || joy_pub.getNumSubscribers() > 0) {
      sensor_msgs::Joy msg;
      msg.header.stamp = ros::Time::now();
      msg.axes = axes;
      msg.buttons = buttons;
      joy_pub.publish(msg);
    }

    ros::spinOnce();
    rate.sleep();
  }

  if (fd >= 0) {
    close(fd);
  }
  return 0;
}
