/* Copyright (C) 2020 Prof. Dr. Christian Pfitzner - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license, which unfortunately won't be
 * written for another century.
 */



// ros includes
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>


// std includes
#include <array>



// global variables
sensor_msgs::Joy g_joy_msg; 
bool             g_init = false; 

void joyCallback(const sensor_msgs::Joy  msg)
{
   std::cout << __PRETTY_FUNCTION__ << std::endl; 
   g_joy_msg = msg; 

   g_init = true; 
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "drives_control");
   ros::NodeHandle n;



   ros::Subscriber joy_sub = n.subscribe("joy", 1, joyCallback);
   ros::Publisher cmd_pub  = n.advertise<std_msgs::Int16MultiArray>("cmd_pwm", 1);
   

   ros::Rate loop_rate(10);



   while(ros::ok())
   {
      std::cout << g_init << std::endl; 
      ros::spinOnce();          


      if(g_init == false)
         continue; 

      const auto forward = g_joy_msg.axes[1]; 
      const auto left    = g_joy_msg.axes[0]; 
      const auto rot     = g_joy_msg.axes[3]; 


      auto front_left  = forward - left - rot; 
      auto front_right = forward + left + rot; 
      auto rear_left   = forward + left - rot; 
      auto rear_right  = forward - left + rot;    




      // calculate the kinematic based on matrix
      std::array<unsigned int, 4> pwm_values;  
      pwm_values[0] = front_left * 255; 
      pwm_values[1] = front_right* 255;
      pwm_values[2] = rear_left  * 255; 
      pwm_values[3] = rear_right * 255; 


      // publish the calculated results via rosserial
      std_msgs::Int16MultiArray cmd_pwm;
      cmd_pwm.layout.dim.resize(4);  
      std::copy(pwm_values.begin(), pwm_values.end(), std::back_inserter(cmd_pwm.data));

      // cmd_pwm.data       = pwm_values.data;  

      cmd_pub.publish(cmd_pwm); 
      


      loop_rate.sleep(); 
   }

   return 0;
}