#include <arpa/inet.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <algorithm>
#include <boost/thread.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#include <ros/package.h>
#include <ros/ros.h>
#include "core_msgs/ball_position.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"

#include "opencv2/opencv.hpp"
#include "simple_lidar_subscriber.h"

int ball_number;
float ball_X[20];
float ball_Y[20];
float ball_distance[20];
int near_ball;

int action;

int len;
int n;

void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{
  int count = position->size;
  ball_number = count;
  for (int i = 0; i < count; i++)
  {
    ball_X[i] = position->img_x[i];
    ball_Y[i] = position->img_y[i];
    // std::cout << "degree : "<< ball_degree[i];
    // std::cout << "   distance : "<< ball_distance[i]<<std::endl;
    ball_distance[i] = ball_X[i] * ball_X[i] + ball_Y[i] * ball_X[i];
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_integation");
  ros::NodeHandle n;

  SimpleLidarSubscriber lidar_subscriber;
  ros::Subscriber sub =
      n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &SimpleLidarSubscriber::callback, &lidar_subscriber);
  ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);
  ros::Publisher pub_left_wheel =
      n.advertise<std_msgs::Float64>("/turtlebot3_waffle_sim/left_wheel_velocity_controller/command", 10);
  ros::Publisher pub_right_wheel =
      n.advertise<std_msgs::Float64>("/turtlebot3_waffle_sim/right_wheel_velocity_controller/command", 10);

  while (ros::ok())
  {
    std_msgs::Float64 left_wheel_msg;
    std_msgs::Float64 right_wheel_msg;

    left_wheel_msg.data = 1;   // set left_wheel velocity
    right_wheel_msg.data = 1;  // set right_wheel velocity
    /////////////////////////////////////////////////////////////////////////////////////////////////
    // // 각노드에서 받아오는 센서 테이터가 잘 받아 왔는지 확인하는 코드 (ctrl + /)을 눌러 주석을 추가/제거할수 있다.///
    /////////////////////////////////////////////////////////////////////////////////////////////////

    //  for(int i = 0; i < lidar_size; i++
    // {
    //    std::cout << "degree : "<< lidar_degree[i];
    //    std::cout << "   distance : "<< lidar_distance[i]<<std::endl;
    //  }
    // for(int i = 0; i < ball_number; i++)
    // {
    // 	std::cout << "ball_X : "<< ball_X[i];
    // 	std::cout << "ball_Y : "<< ball_Y[i]<<std::endl;
    //
    // }

    pub_left_wheel.publish(left_wheel_msg);    // publish left_wheel velocity
    pub_right_wheel.publish(right_wheel_msg);  // publish right_wheel velocity

    ros::Duration(0.025).sleep();
    ros::spinOnce();
  }

  return 0;
}
