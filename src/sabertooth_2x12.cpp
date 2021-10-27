#include "sabertooth_2x12.hpp"

namespace serial_motor_controller
{
Sabertooth2x12::Sabertooth2x12()
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  nh_private.param("device", device_name, std::string("/dev/ttyUSB0"));
  nh_private.param("gear_reduction", gear_reduction, 1.0);
  nh_private.param("has_encoders", has_encoders, false);
  left_encoder_value_recieved = false;  //?

  control_input = nh.subscribe("control_vel", 1, &Sabertooth2x12::control_cb, this);

  if (has_encoders)
  {
    ros::Timer encoder_timer = nh.createTimer(ros::Duration(0.1), &Sabertooth2x12::get_encoder_count, this);
    encoder_output = nh.advertise<isc_shared_msgs::EncoderCounts>("encoder_counts", 1000);
  }
}

bool Sabertooth2x12::startup()
{
  if (serial_port.isOpen())
  {
    shutdown();
  }

  // configure the serial port
  serial_port.setPort(device_name);
  serial_port.setBaudrate(115200);
  serial_port.setBytesize(serial::eightbits);
  serial_port.setParity(serial::parity_none);
  serial_port.setStopbits(serial::stopbits_one);

  serial::Timeout t = serial::Timeout::simpleTimeout(10);
  serial_port.setTimeout(t);



  //we really don't be listening to the sabertooth though.
  //Can I remove all of this?

  // configure the serial listener
  serial_listener.setChunkSize(64);
  serial_listener.setTokenizer(serial::utils::SerialListener::delimeter_tokenizer("\r\n"));
  serial_listener.setDefaultHandler(boost::bind(&Sabertooth2x12::receive, this, _1));

  // open port and start listening
  try
  {
    serial_port.open();
    serial_listener.startListening(serial_port);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Serial exception!: %s", e.what());
    return false;
  }

  ROS_INFO("Connected to NextGen Roboteq!");

  return true;
}

void Sabertooth2x12::shutdown()
{
  if (serial_port.isOpen())
  {
    serial_listener.stopListening();
    serial_port.close();
  }
}

bool Sabertooth2x12::send(const std::string& command)
{
  return serial_motor_controller::send(command + "\r");
};

void Sabertooth2x12::control_cb(const geometry_msgs::Twist::ConstPtr& command)
{
  std::pair<int, int> wheel_speeds = twist_to_wheel_speeds(command);

  std::string left_move_cmd = "!G 1 ";
  std::string right_move_cmd = "!G 2 ";

  left_move_cmd += boost::lexical_cast<std::string>(serial_motor_controller::constrain_speed(wheel_speeds.first, 10000));
  right_move_cmd += boost::lexical_cast<std::string>(serial_motor_controller::constrain_speed(wheel_speeds.second, 10000));

  send(left_move_cmd);
  send(right_move_cmd);
}

void Sabertooth2x12::receive(const std::string& response)
{
  ROS_INFO("Received from Roboteq: %s", response.c_str());
  if (has_encoders)
  {
    if (response.substr(0, 3) == "CR=" && !left_encoder_value_recieved)
    {
      counts.left_count = std::stoi(response.substr(3)) / gear_reduction;
      left_encoder_value_recieved = true;
    }
    else
    {
      counts.right_count = std::stoi(response.substr(3)) / gear_reduction;
      encoder_output.publish(counts);
      left_encoder_value_recieved = false;
    }
  }
}

void Sabertooth2x12::get_encoder_count(const ros::TimerEvent&)
{
  //now we have to capture them from the arduino.


  send("?CR 1");
  send("?CR 2");
}

Sabertooth2x12::~Sabertooth2x12()
{
  shutdown();
}
}  // namespace serial_motor_controller
