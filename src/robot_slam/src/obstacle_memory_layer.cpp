#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <boost/thread/mutex.hpp>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layer.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

namespace robot_slam
{

class ObstacleMemoryLayer : public costmap_2d::Layer
{
public:
  ObstacleMemoryLayer()
    : memory_resolution_(0.02)
    , decay_time_(180.0)
    , min_hits_(2)
    , max_range_(3.0)
    , clear_robot_radius_(0.28)
    , bounds_padding_(0.35)
    , max_cells_(8000)
    , publish_cloud_(true)
    , transform_timeout_(0.08)
  {
  }

  virtual void onInitialize()
  {
    ros::NodeHandle private_nh("~/" + name_);
    ros::NodeHandle nh;

    enabled_ = true;
    current_ = true;
    global_frame_ = layered_costmap_->getGlobalFrameID();
    cloud_topic_ = "/scan_obstacle_memory";

    private_nh.param("enabled", enabled_, enabled_);
    private_nh.param("scan_topic", scan_topic_, std::string("/scan_filtered"));
    private_nh.param("global_frame", global_frame_, global_frame_);
    private_nh.param("memory_resolution", memory_resolution_, memory_resolution_);
    private_nh.param("decay_time", decay_time_, decay_time_);
    private_nh.param("min_hits", min_hits_, min_hits_);
    private_nh.param("max_range", max_range_, max_range_);
    private_nh.param("clear_robot_radius", clear_robot_radius_, clear_robot_radius_);
    private_nh.param("bounds_padding", bounds_padding_, bounds_padding_);
    private_nh.param("max_cells", max_cells_, max_cells_);
    private_nh.param("publish_cloud", publish_cloud_, publish_cloud_);
    private_nh.param("cloud_topic", cloud_topic_, cloud_topic_);
    private_nh.param("transform_timeout", transform_timeout_, transform_timeout_);

    if (memory_resolution_ <= 0.0)
    {
      ROS_WARN("[%s] memory_resolution %.3f invalid, fallback to 0.02", name_.c_str(), memory_resolution_);
      memory_resolution_ = 0.02;
    }
    if (decay_time_ < 0.1)
    {
      ROS_WARN("[%s] decay_time %.3f too small, fallback to 180.0", name_.c_str(), decay_time_);
      decay_time_ = 180.0;
    }
    if (min_hits_ < 1)
    {
      min_hits_ = 1;
    }
    if (max_cells_ < 100)
    {
      max_cells_ = 100;
    }

    scan_nh_ = nh;
    if (enabled_)
    {
      startScanSubscriber();
    }
    if (publish_cloud_)
    {
      cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic_, 1, true);
    }
    clear_srv_ = private_nh.advertiseService("clear", &ObstacleMemoryLayer::clearService, this);
    set_enabled_srv_ = private_nh.advertiseService("set_enabled", &ObstacleMemoryLayer::setEnabledService, this);

    ROS_INFO(
      "[%s] initialized scan=%s frame=%s decay=%.1fs res=%.3f min_hits=%d max_range=%.2f clear_robot_radius=%.2f max_cells=%d enabled=%s",
      name_.c_str(), scan_topic_.c_str(), global_frame_.c_str(), decay_time_, memory_resolution_, min_hits_,
      max_range_, clear_robot_radius_, max_cells_, enabled_ ? "true" : "false");
  }

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x, double* max_y)
  {
    boost::mutex::scoped_lock lock(mutex_);
    const ros::Time now = ros::Time::now();

    removeExpiredCells(now);
    removeCellsNearRobot(robot_x, robot_y);
    pruneOverflow();

    addBoundsForCells(removed_cells_, min_x, min_y, max_x, max_y);
    removed_cells_.clear();

    active_cells_.clear();
    if (!enabled_)
    {
      return;
    }

    for (CellMap::const_iterator it = cells_.begin(); it != cells_.end(); ++it)
    {
      if (it->second.hits < min_hits_)
      {
        continue;
      }
      active_cells_.push_back(it->second);
    }

    addBoundsForCells(active_cells_, min_x, min_y, max_x, max_y);
  }

  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!enabled_)
    {
      return;
    }

    for (std::vector<MemoryCell>::const_iterator it = active_cells_.begin(); it != active_cells_.end(); ++it)
    {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!master_grid.worldToMap(it->x, it->y, mx, my))
      {
        continue;
      }
      if (static_cast<int>(mx) < min_i || static_cast<int>(mx) >= max_i ||
          static_cast<int>(my) < min_j || static_cast<int>(my) >= max_j)
      {
        continue;
      }
      master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
    }
  }

  virtual void reset()
  {
    clearMemory("reset");
  }

private:
  struct MemoryCell
  {
    double x;
    double y;
    ros::Time last_seen;
    int hits;
  };

  typedef std::pair<int, int> CellKey;
  typedef std::map<CellKey, MemoryCell> CellMap;

  CellKey cellKey(double x, double y) const
  {
    return CellKey(static_cast<int>(std::floor(x / memory_resolution_)),
                   static_cast<int>(std::floor(y / memory_resolution_)));
  }

  void scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
  {
    {
      boost::mutex::scoped_lock lock(mutex_);
      if (!enabled_)
      {
        return;
      }
    }
    if (scan->header.frame_id.empty())
    {
      ROS_WARN_THROTTLE(2.0, "[%s] scan frame_id is empty", name_.c_str());
      return;
    }

    geometry_msgs::TransformStamped transform_msg;
    try
    {
      transform_msg = tf_->lookupTransform(
        global_frame_, scan->header.frame_id, scan->header.stamp, ros::Duration(transform_timeout_));
    }
    catch (const tf2::TransformException& ex)
    {
      try
      {
        transform_msg = tf_->lookupTransform(global_frame_, scan->header.frame_id, ros::Time(0),
                                             ros::Duration(transform_timeout_));
      }
      catch (const tf2::TransformException& fallback_ex)
      {
        ROS_WARN_THROTTLE(2.0, "[%s] TF %s -> %s failed: %s",
                          name_.c_str(), scan->header.frame_id.c_str(), global_frame_.c_str(),
                          fallback_ex.what());
        return;
      }
    }

    tf2::Transform transform;
    tf2::fromMsg(transform_msg.transform, transform);

    const double range_limit = std::min(max_range_, static_cast<double>(scan->range_max));
    const ros::Time stamp = scan->header.stamp.isZero() ? ros::Time::now() : scan->header.stamp;
    std::vector<MemoryCell> new_cells;
    new_cells.reserve(scan->ranges.size());

    double angle = scan->angle_min;
    for (std::size_t i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment)
    {
      const double range = scan->ranges[i];
      if (!std::isfinite(range) || range < scan->range_min || range > range_limit)
      {
        continue;
      }

      const tf2::Vector3 local_point(range * std::cos(angle), range * std::sin(angle), 0.0);
      const tf2::Vector3 map_point = transform * local_point;
      MemoryCell cell;
      cell.x = map_point.x();
      cell.y = map_point.y();
      cell.last_seen = stamp;
      cell.hits = 1;
      new_cells.push_back(cell);
    }

    if (new_cells.empty())
    {
      return;
    }

    {
      boost::mutex::scoped_lock lock(mutex_);
      if (!enabled_)
      {
        return;
      }
      for (std::vector<MemoryCell>::const_iterator it = new_cells.begin(); it != new_cells.end(); ++it)
      {
        const CellKey key = cellKey(it->x, it->y);
        MemoryCell& cell = cells_[key];
        if (cell.hits <= 0)
        {
          cell = *it;
        }
        else
        {
          cell.x = it->x;
          cell.y = it->y;
          cell.last_seen = it->last_seen;
          if (cell.hits < 1000000)
          {
            ++cell.hits;
          }
        }
      }
      pruneOverflow();
    }

    publishMemoryCloud();
  }

  bool clearService(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
  {
    (void)request;
    const std::size_t count = clearMemory("service_clear");
    response.success = true;
    response.message = "cleared " + toString(count) + " obstacle memory cells";
    return true;
  }

  bool setEnabledService(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
  {
    const bool target_enabled = request.data;
    {
      boost::mutex::scoped_lock lock(mutex_);
      if (!target_enabled)
      {
        enabled_ = false;
        const std::size_t count = clearMemoryLocked("service_disable");
        response.success = true;
        response.message = "disabled and cleared " + toString(count) + " obstacle memory cells";
      }
      else
      {
        enabled_ = true;
        response.success = true;
        response.message = "enabled";
      }
      publishMemoryCloudLocked();
    }
    if (target_enabled)
    {
      startScanSubscriber();
    }
    else
    {
      stopScanSubscriber();
    }
    ROS_INFO("[%s] set_enabled=%s: %s", name_.c_str(), enabled_ ? "true" : "false", response.message.c_str());
    return true;
  }

  std::size_t clearMemory(const std::string& reason)
  {
    boost::mutex::scoped_lock lock(mutex_);
    const std::size_t count = clearMemoryLocked(reason);
    publishMemoryCloudLocked();
    return count;
  }

  std::size_t clearMemoryLocked(const std::string& reason)
  {
    const std::size_t count = cells_.size();
    for (CellMap::const_iterator it = cells_.begin(); it != cells_.end(); ++it)
    {
      removed_cells_.push_back(it->second);
    }
    cells_.clear();
    active_cells_.clear();
    ROS_INFO("[%s] clear memory reason=%s cells=%lu", name_.c_str(), reason.c_str(),
             static_cast<unsigned long>(count));
    return count;
  }

  void removeExpiredCells(const ros::Time& now)
  {
    for (CellMap::iterator it = cells_.begin(); it != cells_.end();)
    {
      if ((now - it->second.last_seen).toSec() > decay_time_)
      {
        removed_cells_.push_back(it->second);
        cells_.erase(it++);
      }
      else
      {
        ++it;
      }
    }
  }

  void removeCellsNearRobot(double robot_x, double robot_y)
  {
    if (clear_robot_radius_ <= 0.0)
    {
      return;
    }

    const double radius_sq = clear_robot_radius_ * clear_robot_radius_;
    for (CellMap::iterator it = cells_.begin(); it != cells_.end();)
    {
      const double dx = it->second.x - robot_x;
      const double dy = it->second.y - robot_y;
      if (dx * dx + dy * dy <= radius_sq)
      {
        removed_cells_.push_back(it->second);
        cells_.erase(it++);
      }
      else
      {
        ++it;
      }
    }
  }

  void pruneOverflow()
  {
    while (cells_.size() > static_cast<std::size_t>(max_cells_))
    {
      CellMap::iterator oldest = cells_.begin();
      for (CellMap::iterator it = cells_.begin(); it != cells_.end(); ++it)
      {
        if (it->second.last_seen < oldest->second.last_seen)
        {
          oldest = it;
        }
      }
      removed_cells_.push_back(oldest->second);
      cells_.erase(oldest);
    }
  }

  void addBoundsForCells(const std::vector<MemoryCell>& cells,
                         double* min_x, double* min_y, double* max_x, double* max_y) const
  {
    for (std::vector<MemoryCell>::const_iterator it = cells.begin(); it != cells.end(); ++it)
    {
      const double pad = std::max(memory_resolution_, bounds_padding_);
      *min_x = std::min(*min_x, it->x - pad);
      *min_y = std::min(*min_y, it->y - pad);
      *max_x = std::max(*max_x, it->x + pad);
      *max_y = std::max(*max_y, it->y + pad);
    }
  }

  void publishMemoryCloud()
  {
    if (!publish_cloud_ || cloud_pub_.getNumSubscribers() <= 0)
    {
      return;
    }
    boost::mutex::scoped_lock lock(mutex_);
    publishMemoryCloudLocked();
  }

  void publishMemoryCloudLocked()
  {
    if (!publish_cloud_)
    {
      return;
    }

    std::vector<MemoryCell> trusted_cells;
    trusted_cells.reserve(cells_.size());
    for (CellMap::const_iterator it = cells_.begin(); it != cells_.end(); ++it)
    {
      if (enabled_ && it->second.hits >= min_hits_)
      {
        trusted_cells.push_back(it->second);
      }
    }

    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = global_frame_;
    cloud.height = 1;
    cloud.width = trusted_cells.size();

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(trusted_cells.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (std::vector<MemoryCell>::const_iterator it = trusted_cells.begin(); it != trusted_cells.end();
         ++it, ++iter_x, ++iter_y, ++iter_z)
    {
      *iter_x = static_cast<float>(it->x);
      *iter_y = static_cast<float>(it->y);
      *iter_z = 0.0f;
    }

    cloud_pub_.publish(cloud);
  }

  std::string toString(std::size_t value) const
  {
    std::ostringstream stream;
    stream << value;
    return stream.str();
  }

  void startScanSubscriber()
  {
    if (scan_sub_)
    {
      return;
    }
    scan_sub_ = scan_nh_.subscribe(scan_topic_, 1, &ObstacleMemoryLayer::scanCallback, this);
    ROS_INFO("[%s] subscribed scan topic %s", name_.c_str(), scan_topic_.c_str());
  }

  void stopScanSubscriber()
  {
    if (!scan_sub_)
    {
      return;
    }
    scan_sub_.shutdown();
    ROS_INFO("[%s] unsubscribed scan topic %s", name_.c_str(), scan_topic_.c_str());
  }

  boost::mutex mutex_;
  CellMap cells_;
  std::vector<MemoryCell> active_cells_;
  std::vector<MemoryCell> removed_cells_;

  ros::NodeHandle scan_nh_;
  ros::Subscriber scan_sub_;
  ros::Publisher cloud_pub_;
  ros::ServiceServer clear_srv_;
  ros::ServiceServer set_enabled_srv_;

  std::string scan_topic_;
  std::string global_frame_;
  std::string cloud_topic_;
  double memory_resolution_;
  double decay_time_;
  int min_hits_;
  double max_range_;
  double clear_robot_radius_;
  double bounds_padding_;
  int max_cells_;
  bool publish_cloud_;
  double transform_timeout_;
};

}  // namespace robot_slam

PLUGINLIB_EXPORT_CLASS(robot_slam::ObstacleMemoryLayer, costmap_2d::Layer)
