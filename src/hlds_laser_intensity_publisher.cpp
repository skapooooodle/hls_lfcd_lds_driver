/*******************************************************************************
* Copyright (c) 2016, Hitachi-LG Data Storage
* Copyright (c) 2017, ROBOTIS
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

 /* Authors: Pyo, Darby Lim, SP Kong, JH Yang */
 /* maintainer: Pyo */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include<unistd.h>

#include <rclcpp/rclcpp.hpp>
//#include <sensor_msgs/msg/laser_scan.hpp>
#include <boost/asio.hpp>
#include <hls_lfcd_lds_driver/lfcd_laser_inten.hpp>

#include "std_msgs/msg/string.hpp"

namespace hls_lfcd_lds
{
LFCDLaser::LFCDLaser(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io)
  : port_(port), baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_)
{
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));

  boost::asio::write(serial_, boost::asio::buffer("b", 1));  // start motor
}

LFCDLaser::~LFCDLaser()
{
  boost::asio::write(serial_, boost::asio::buffer("e", 1));  // stop motor
}

//void LFCDLaser::poll(sensor_msgs::msg::LaserScan::SharedPtr scan)
void LFCDLaser::poll(std_msgs::msg::String::SharedPtr message)
{
  uint8_t start_count = 0;
  bool got_scan = false;
  boost::array<uint8_t, 2520> raw_bytes;
  uint8_t good_sets = 0;
  uint32_t motor_speed = 0;
  rpms=0;
  int index;

  //unsigned int microsecond = 1000;

  auto node = rclcpp::Node::make_shared("hlds_laser_intensity_publisher");
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  publisher_ = node->create_publisher<std_msgs::msg::String>("scan_inten", rclcpp::QoS(rclcpp::SensorDataQoS()));

  while (!shutting_down_ && !got_scan)
  {
    // Wait until first data sync of frame: 0xFA, 0xA0
    boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[start_count],1));

    if(start_count == 0)
    {
      if(raw_bytes[start_count] == 0xFA)
      {
        start_count = 1;
      }
    }
    else if(start_count == 1)
    {
      if(raw_bytes[start_count] == 0xA0)
      {
        start_count = 0;

        // Now that entire start sequence has been found, read in the rest of the message
        got_scan = true;

        boost::asio::read(serial_,boost::asio::buffer(&raw_bytes[2], 2518));
        
        for(uint16_t i = 0; i < raw_bytes.size(); i=i+42)
        {
          if(raw_bytes[i] == 0xFA && raw_bytes[i+1] == (0xA0 + i / 42)) //&& CRC check
          {
            good_sets++;
            motor_speed += (raw_bytes[i+3] << 8) + raw_bytes[i+2]; //accumulate count for avg. time increment
            rpms=(raw_bytes[i+3]<<8|raw_bytes[i+2])/10;

            for(uint16_t j = i+4; j < i+40; j=j+6)
            {
              index = 6*(i/42) + (j-4-i)/6;

              uint8_t byte0 = raw_bytes[j];
              uint8_t byte1 = raw_bytes[j+1];
              uint16_t intensity = (byte1 << 8) + byte0;
              if ( (359-index <= 50 ) || (359-index >= 309))
              {
                message -> data = "r[" + std::to_string(359-index) + "] = " + std::to_string(intensity);
                //usleep(1 * microsecond);//sleep for debugging
                publisher_->publish(*message);
                //printf ("r[%d]=%d,",359-index, intensity);
              }

            }
          }
        }
      }
      else
      {
        start_count = 0;
      }
    }
  }
}
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("hlds_laser_intensity_publisher");
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  boost::asio::io_service io;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr message;

  std::string port;
  std::string frame_id;
  int baud_rate;

  const char* topic = "scan_inten"; //name of topic
  
  node->declare_parameter<std::string>("port");
  node->declare_parameter<std::string>("frame_id");

  node->get_parameter_or<std::string>("port", port, "/dev/ttyUSB0");
  node->get_parameter_or<std::string>("frame_id", frame_id, "laser");

  baud_rate = 230400;

  RCLCPP_INFO(node->get_logger(), "Publishing topic: '%s'", topic);

  try
  {
    hls_lfcd_lds::LFCDLaser laser(port, baud_rate, io);
    publisher_ = node->create_publisher<std_msgs::msg::String>("topic", rclcpp::QoS(rclcpp::SensorDataQoS())); 
    auto message = std_msgs::msg::String();
    
    while (rclcpp::ok())
    { 
      auto message = std::make_shared<std_msgs::msg::String>();
      laser.poll(message);
    }
    laser.close();

    return 0;
  }
  catch (boost::system::system_error& ex)
  {
    //ROS_ERROR("An exception was thrown: %s", ex.what());
    return -1;
  }
}