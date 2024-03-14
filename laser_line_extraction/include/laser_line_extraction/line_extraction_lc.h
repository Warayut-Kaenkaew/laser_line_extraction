#ifndef LINE_EXTRACTION_ROS_H
#define LINE_EXTRACTION_ROS_H

#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "laser_line_extraction_interfaces/msg/line_segment.hpp"
#include "laser_line_extraction_interfaces/msg/line_segment_list.hpp"
#include "laser_line_extraction/line_extraction.h"
#include "laser_line_extraction/line.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace line_extraction
{

  class LifecycleLineExtractionROS : public rclcpp_lifecycle::LifecycleNode
  {

  public:
    // Constructor / destructor
    LifecycleLineExtractionROS(const std::string & node_name, bool intra_process_comms = false);
    ~LifecycleLineExtractionROS();

    // Running
    void run();
    void publish();
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) ;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);
  
  private:
    // ROS
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<laser_line_extraction_interfaces::msg::LineSegmentList>> line_publisher_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>> marker_publisher_;
    // Parameters
    size_t count;
    std::string frame_id_;
    std::string scan_topic_;
    bool pub_markers_;
    double frequency_;
    double bearing_std_dev, range_std_dev, least_sq_angle_thresh, least_sq_radius_thresh,
      max_line_gap, min_line_length, min_range, max_range, min_split_dist, outlier_dist;
    int min_line_points;
    // Line extraction
    rclcpp::TimerBase::SharedPtr timer_;
    LineExtraction line_extraction_;
    bool data_cached_; // true after first scan used to cache data
    // Members
    void loadParameters();
    void populateLineSegListMsg(const std::vector<Line> &, laser_line_extraction_interfaces::msg::LineSegmentList &);
    void populateMarkerMsg(const std::vector<Line> &, visualization_msgs::msg::Marker &);
    void cacheData(const sensor_msgs::msg::LaserScan::SharedPtr &);
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr);

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
    std::shared_ptr<rclcpp::TimerBase> count_timer_;
  };

} // namespace line_extraction

#endif
