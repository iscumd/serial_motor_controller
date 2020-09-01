#include "serial_motor_controller.h"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <sstream>
#include <isc_shared_msgs/EncoderCounts.h>

namespace serial_motor_controller
{
class roboteq_mdc2460 : public serial_motor_controller
{
public:
  roboteq_mdc2460();

  ~roboteq_mdc2460();

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
  isc_shared_msgs::EncoderCounts counts;

  std::stringstream msg_builder;

  ros::Publisher encoder_output;
  ros::Subscriber control_input;
};
}  // namespace serial_motor_controller
