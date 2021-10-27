#include <serial_motor_controller.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <sstream>

namespace serial_motor_controller
{
class RoboteqHDC2460 : public serial_motor_controller
{
public:
  RoboteqHDC2460();

  ~RoboteqHDC2460();

  bool startup();

  void shutdown();

  bool send(const std::string& command);

  void control_cb(const geometry_msgs::Twist::ConstPtr& command);

  void receive(const std::string& response);

  void get_encoder_count(const ros::TimerEvent&);

private:
  std::string device_name;
  double gear_reduction;
  bool has_encoders;
  bool left_encoder_value_recieved;

  std::stringstream msg_builder;

  ros::Publisher encoder_output;
  ros::Subscriber control_input;
};
}  // namespace serial_motor_controller