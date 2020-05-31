#include "angles/angles.h"
#include "simple_lidar_subscriber.h"

void SimpleLidarSubscriber::callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  size_t count = scan->ranges.size();
  data_.resize(count);
  for (size_t i = 0; i < count; i++)
  {
    auto angle = scan->angle_min + scan->angle_increment * i;
    data_[i].angle = angle;
    data_[i].angle_deg = angles::to_degrees(angle);
    data_[i].range = scan->ranges[i];
  }
}