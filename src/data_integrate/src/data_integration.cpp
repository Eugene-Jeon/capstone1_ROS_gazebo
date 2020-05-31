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

#define RAD2DEG(x) ((x)*180. / M_PI)

boost::mutex map_mutex;

int lidar_size;
float lidar_degree[400];
float lidar_distance[400];
float lidar_obs;

int ball_number;
float ball_X[20];
float ball_Y[20];
float ball_distance[20];
int near_ball;

int action;

int len;
int n;

#define RAD2DEG(x) ((x)*180. / M_PI)

void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  map_mutex.lock();

  int count = scan->angle_max / scan->angle_increment;
  lidar_size = count;
  for (int i = 0; i < count; i++)
  {
    lidar_degree[i] = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    lidar_distance[i] = scan->ranges[i];
  }
  map_mutex.unlock();
}
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

void lidarToHoughLines(cv::Mat& image, int threshold)
{
  const int IMAGE_WIDTH = 400;
  const int IMAGE_HEIGHT = 400;
  const double MAP_RESOL = 0.015;

  // Fill the matrix with zeros
  image = cv::Mat::zeros(IMAGE_WIDTH, IMAGE_HEIGHT, CV_8UC1);

  // Create a rect for checking points
  const cv::Rect imageRect(cv::Point(), image.size());

  // Build a binary (black-and-white) image from LIDAR data
  int cx, cy;
  int cx1, cx2, cy1, cy2;
  for (int i = 0; i < lidar_size; i++)
  {
    float obstacle_x = lidar_distance[i] * cos(lidar_degree[i]);
    float obstacle_y = lidar_distance[i] * sin(lidar_degree[i]);
    int cx = IMAGE_WIDTH / 2 + static_cast<int>(obstacle_y / MAP_RESOL);
    int cy = IMAGE_HEIGHT / 2 + static_cast<int>(obstacle_x / MAP_RESOL);

    if (imageRect.contains({ cx, cy }))
    {
      image.at<int>(cx, cy) = 255;
    }
  }

  // Standard Hough Line Transform
  std::vector<cv::Vec2f> lines;
  cv::HoughLines(image, lines, 1, CV_PI / 180, threshold, 0, 0);

  for (size_t i = 0; i < lines.size(); i++)
  {
    float rho = lines[i][0], theta = lines[i][1];
    std::cout << "Line #" << (i + 1) << ": "
              << "rho = " << rho << "; theta" << theta << std::endl;
  }

  std::cout << "lidarToHoughLines completed, " << lines.size() << " line(s) found.\n";
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_integation");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
  ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);
  ros::Publisher pub_left_wheel =
      n.advertise<std_msgs::Float64>("/turtlebot3_waffle_sim/left_wheel_velocity_controller/command", 10);
  ros::Publisher pub_right_wheel =
      n.advertise<std_msgs::Float64>("/turtlebot3_waffle_sim/right_wheel_velocity_controller/command", 10);

  cv::Mat lidarImage;

  int houghThreshold = n.param("/data_integation/hough_threshold", 5);
  std::cout << "Using threshold = " << houghThreshold << "...\n";

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

    lidarToHoughLines(lidarImage, houghThreshold);

    ros::Duration(0.025).sleep();
    ros::spinOnce();
  }

  return 0;
}
