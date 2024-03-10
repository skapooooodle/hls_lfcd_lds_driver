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
#include "std_msgs/msg/float32.hpp"

#include <algorithm>
#include <iterator>
#include <iostream>
#include <cmath>

/*********** range of intensity of white line ***********/
#define min_white_line_intensity 900                  //test for correct numbers required
#define max_white_line_intensity 3079

/*********** field of view (LiDAR) ***********/
#define min_angle_range 50                            //equals to 400 mm Spurbreite.eng(xd)
#define max_angle_range 310

uint16_t set_counter = 0;

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
//void LFCDLaser::poll(std_msgs::msg::String::SharedPtr message)
void LFCDLaser::poll(std_msgs::msg::Float32::SharedPtr message)
{
  uint8_t start_count = 0;
  bool got_scan = false;
  boost::array<uint8_t, 2520> raw_bytes;
  uint8_t good_sets = 0;
  uint32_t motor_speed = 0;
  rpms=0;
  int index;

  uint16_t intensities[360];

  //uint16_t dan_counter;
  //uint16_t mid_point_line[2];
  //uint16_t angles[min_angle_range];

  //unsigned int microsecond = 1000;
/*
  auto node = rclcpp::Node::make_shared("hlds_laser_intensity_publisher");
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  publisher_ = node->create_publisher<std_msgs::msg::Float32>("scan_inten", rclcpp::QoS(rclcpp::SensorDataQoS()));
*/
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
              intensities[359-index] = intensity;
              //message -> data = intensity; 
              //dan_counter++;
              //printf("dan_counter: %d ", dan_counter);
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
        /********************************* HERE WE GO mit dem latest shit *********************************/
        
        /*** coole variablen ***/
        uint16_t line_angles_b[min_angle_range];
        uint16_t line_angles_t[360];
        uint16_t mid_point_line_b;
        uint16_t mid_point_line_t;

        uint8_t min_degree_b = 0;
        uint16_t min_degree_t = 0;

        /*** get midpoint of white line from 0-50° ***/
        for (int i = 0; i < min_angle_range; i++){
          if ( (intensities[i] >= min_white_line_intensity) && (intensities[i] <= max_white_line_intensity) ){
            line_angles_b[i] = i;                                                                                      //append angles with "white" intensity
            //printf("line_angles_b[%d]= %d \n", i, line_angles_b[i]);
            if ( (line_angles_b[i-1] == 0) && (line_angles_b[i-2] == 0) ){
              min_degree_b = i;                                                                                          //get first degree with "white" intensity
            }
          } 
          else {
            line_angles_b[i] = 0;
          }
        }

        /*** get midpoint of white line from 310-359° ***/
        for (int i = 359; i > max_angle_range; i--){
          if ( (intensities[i] >= min_white_line_intensity) && (intensities[i] <= max_white_line_intensity) ){
            line_angles_t[i] = i;                                                                                      //append angles with "white" intensity
            min_degree_t = i;
            //printf("line_angles_t[%d]= %d \n", i, line_angles_t[i]);
          }
          else {
            line_angles_t[i] = 0;
            //printf("zero out line_angles_t[%d]= %d \n", i, line_angles_t[i]);
          }
        }

        for (int i = 0; i <= max_angle_range; i++){
          line_angles_t[i] = 0;
        }
        
        /*** Get max/min value of array ***/
        //uint16_t *min_degree_b = std::min_element(std::begin(line_angles_b), std::end(line_angles_b));  
        //uint16_t *min_degree_t = std::min_element(std::begin(line_angles_t), std::end(line_angles_t));  

        uint16_t *max_degree_b = std::max_element(std::begin(line_angles_b), std::end(line_angles_b));   
        uint16_t *max_degree_t = std::max_element(std::begin(line_angles_t), std::end(line_angles_t)); 

        //uint16_t max_degree_b  = std::max_element(line_angles_b.begin(), line_angles_b.end());
        /*
        printf("Minimum degree b : %d \n", min_degree_b);
        std::cout << "Maximum degree b: " << *max_degree_b << std::endl;

        printf("Minimum degree t : %d \n", min_degree_t);
        std::cout << "Maximum degree t: " << *max_degree_t << std::endl; */

        //printf("min_degree_b = %u \n", min_degree_b); 

        /*** get mid points of white lines ***/
        mid_point_line_b = round( (min_degree_b + *max_degree_b)/2 ); //int??!!!
        mid_point_line_t = round( (min_degree_t + *max_degree_t)/2 ); //int??!!!

        printf("mid_point_line_b = %u \n", mid_point_line_b);
        printf("mid_point_line_t = %u \n", mid_point_line_t); 

        /******* Calculate mid point ********/
        //uint16_t mid_point_line = ((mid_point_line_b + (360 - mid_point_line_t)) / 2)-(360 - mid_point_line_t);
        message->data = ((mid_point_line_b + (360 - mid_point_line_t)) / 2)-(360 - mid_point_line_t) + 4; 

        //set_counter++;
        //printf("   set_counter: %d    ", set_counter); 
}
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("hlds_laser_intensity_publisher");
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  boost::asio::io_service io;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr message;

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
    publisher_ = node->create_publisher<std_msgs::msg::Float32>(topic, rclcpp::QoS(rclcpp::SensorDataQoS())); 
    //auto message = std_msgs::msg::Float();
    
    while (rclcpp::ok())
    { 
      auto message = std::make_shared<std_msgs::msg::Float32>();
      laser.poll(message);
      publisher_ -> publish(*message);
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