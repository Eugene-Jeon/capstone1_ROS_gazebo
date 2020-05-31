#include <utility>
#include <vector>
#include "sensor_msgs/LaserScan.h"

struct SimpleLidarPoint
{
  float angle;      ///< Angle to point (unit: radians).
  float angle_deg;  ///< Angle to point (unit: degrees).
  float range;      ///< Distance from origin to point (unit: meter).
};

/**
 * A simple subscriber that receives and stores data from a LaserScan (LIDAR) data topic (e.g. "/scan").
 * This class stores the most recently received data only.
 *
 * Note: This class is not thread-safe.
 */
class SimpleLidarSubscriber
{
public:
  /**
   * Callback to use when subscribing to a LaserScan (LIDAR) data topic.
   */
  void callback(const sensor_msgs::LaserScan::ConstPtr& scan);

  /**
   * Retrieve the vector containing the latest LIDAR data.
   */
  const std::vector<SimpleLidarPoint>& data() const
  {
    return data_;
  }

private:
  std::vector<SimpleLidarPoint> data_;
};