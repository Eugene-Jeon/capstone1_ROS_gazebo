#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#include <ros/package.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

#include "core_msgs/ball_position.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "opencv2/opencv.hpp"

float MAP_CX = 200.5;
float MAP_CY = 200.5;
float MAP_RESOL = 0.015;  // Map resoultion [cm]
int MAP_WIDTH = 400;
int MAP_HEIGHT = 400;
int MAP_CENTER = 50;
int OBSTACLE_PADDING = 2;       // Obstacle Size
int OBSTACLE_CONNECT_MAX = 15;  // Range to connect obstacles

int init_ball;
int init_lidar;

int lidar_size;
float lidar_degree[400];
float lidar_distance[400];

int ball_number;
float ball_X[20];
float ball_Y[20];

boost::mutex map_mutex;

#define RAD2DEG(x) ((x)*180. / M_PI)

bool check_point_range(int cx, int cy)
{
  return (cx < MAP_WIDTH - 1) && (cx > 0) && (cy < MAP_HEIGHT - 1) && (cy > 0);
}

void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{
  map_mutex.lock();
  int count = position->size;
  ball_number = count;
  for (int i = 0; i < count; i++)
  {
    ball_X[i] = position->img_x[i];
    ball_Y[i] = position->img_y[i];
    std::cout << "ball_X : " << ball_X[i];
    std::cout << "ball_Y : " << ball_Y[i] << std::endl;
  }
  map_mutex.unlock();
}
void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  map_mutex.lock();
  int count = scan->angle_max / scan->angle_increment;
  lidar_size = count;
  for (int i = 0; i < count; i++)
  {
    lidar_degree[i] = scan->angle_min + scan->angle_increment * i;
    lidar_distance[i] = scan->ranges[i];
    // std::cout << "degree : "<< lidar_degree[i];
    // std::cout << "   distance : "<< lidar_distance[i]<<std::endl;
  }
  map_mutex.unlock();
}

/** Class for detecting lines in LIDAR data using Hough Transform. */
class HoughLinesFinder
{
public:
  /**
   * Applies Standard Hough Line Transform to an image to detect lines.
   * @param lines   Output vector of lines. Each element is a pair of (rho, theta) of the line.
   * @param image   Image of points to analyze.
   * @param threshold   Accumulator threshold (minimum vote).
   * @param rho     Distance resolution of the accumulator in pixels.
   * @param theta   Distance resolution of the accumulator in radians.
   */
  void findLines(std::vector<cv::Vec2d>& lines, int threshold, double rho, double theta)
  {
    const int IMAGE_WIDTH = 400;
    const int IMAGE_HEIGHT = 400;
    const float MAP_RESOLUTION = 0.015;  // Map resolution (unit: meters/pixel)

    // Fill the matrix with zeros
    image_ = cv::Mat::zeros(IMAGE_WIDTH, IMAGE_HEIGHT, CV_8UC1);

    // Create a rect for checking if a point lies within the image bounds
    const cv::Rect imageRect(cv::Point(), image_.size());

    // Build a binary (black-and-white) image from LIDAR data
    for (int i = 0; i < lidar_size; i++)
    {
      float obstacle_x = lidar_distance[i] * cos(lidar_degree[i]);
      float obstacle_y = lidar_distance[i] * sin(lidar_degree[i]);
      int cx = IMAGE_WIDTH / 2 + static_cast<int>(obstacle_y / MAP_RESOLUTION);
      int cy = IMAGE_HEIGHT / 2 + static_cast<int>(obstacle_x / MAP_RESOLUTION);

      if (imageRect.contains({ cx, cy }))
        image_.at<int>(cx, cy) = 255;
    }

    // Standard Hough Line Transform
    cv::HoughLines(image_, lines, rho, theta, threshold, 0, 0);
  }

private:
  /** Matrix to use when converting LIDAR data to a 2D image, which is needed for Hough Transform. */
  cv::Mat image_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_show_node");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
  ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);

  std::vector<cv::Vec2d> lines;
  HoughLinesFinder houghLinesFinder;

  // Hough Transform parameters, which will be read from the ROS Parameter Server
  int threshold;
  double rho;
  double theta_degree;
  ros::param::param("~threshold", threshold, 10);
  ros::param::param("~rho", rho, 1.0);
  ros::param::param("~theta_degree", theta_degree, 1.0);
  ROS_INFO("Using threshold = %d, rho = %f (pixels), theta = %f (degrees)...\n", threshold, rho, theta_degree);

  while (ros::ok())
  {
    cv::Mat map = cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8UC3);
    // Drawing Lidar data
    float obstacle_x, obstacle_y;
    int cx, cy;
    int cx1, cx2, cy1, cy2;
    for (int i = 0; i < lidar_size; i++)
    {
      obstacle_x = lidar_distance[i] * cos(lidar_degree[i]);
      obstacle_y = lidar_distance[i] * sin(lidar_degree[i]);
      cx = MAP_WIDTH / 2 + (int)(obstacle_y / MAP_RESOL);
      cy = MAP_HEIGHT / 2 + (int)(obstacle_x / MAP_RESOL);
      cx1 = cx - OBSTACLE_PADDING;
      cy1 = cy - OBSTACLE_PADDING;
      cx2 = cx + OBSTACLE_PADDING;
      cy2 = cy + OBSTACLE_PADDING;

      if (check_point_range(cx, cy) && check_point_range(cx1, cy1) && check_point_range(cx2, cy2))
      {
        cv::line(map, cv::Point(MAP_WIDTH / 2, MAP_HEIGHT / 2), cv::Point(cx, cy), cv::Scalar(63, 63, 0), 1);
        cv::rectangle(map, cv::Point(cx1, cy1), cv::Point(cx2, cy2), cv::Scalar(255, 255, 0), -1);
      }
    }
    // Drawing ball
    for (int i = 0; i < ball_number; i++)
    {
      cx = (int)(ball_X[i] / 4);
      cy = (int)(ball_Y[i] / 4);
      cx1 = cx - OBSTACLE_PADDING * 2;
      cy1 = cy - OBSTACLE_PADDING * 2;
      cx2 = cx + OBSTACLE_PADDING * 2;
      cy2 = cy + OBSTACLE_PADDING * 2;

      if (check_point_range(cx, cy) && check_point_range(cx1, cy1) && check_point_range(cx2, cy2))
      {
        cv::rectangle(map, cv::Point(cx1, cy1), cv::Point(cx2, cy2), cv::Scalar(0, 0, 255), -1);
      }
    }
    // Drawing ROBOT
    cv::circle(map, cv::Point(MAP_WIDTH / 2, MAP_HEIGHT / 2), 3, cv::Scalar(255, 0, 0), -1);

    // Find lines using Hough Transform
    houghLinesFinder.findLines(lines, threshold, rho, theta_degree * CV_PI / 180);

    // Draw the lines found with Hough Transform
    for (size_t i = 0; i < lines.size(); i++)
    {
      auto rho = lines[i][0];
      auto theta = lines[i][1];
      double a = cos(theta), b = sin(theta);
      double x0 = a * rho, y0 = b * rho;
      cv::Point pt1(cvRound(x0 - 1000 * b), cvRound(y0 + 1000 * a));
      cv::Point pt2(cvRound(x0 + 1000 * b), cvRound(y0 - 1000 * a));
      cv::line(map, pt1, pt2, cv::Scalar(255, 255, 0));
    }

    cv::imshow("Frame", map);
    cv::waitKey(50);

    if (cv::waitKey(50) == 113)
    {  // wait for a key command. if 'q' is pressed, then program will be terminated.
      return 0;
    }
    ros::spinOnce();
  }

  return 0;
}
