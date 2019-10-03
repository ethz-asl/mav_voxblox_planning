#include <ros/ros.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_planning_common/color_utils.h>
#include <mav_planning_common/path_visualization.h>
#include <visualization_msgs/MarkerArray.h>

namespace mav_planning {

class TrajectoryRecolor {
 public:
  TrajectoryRecolor(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private)
      : nh_(nh), nh_private_(nh_private), counter_(0), max_plans_(50) {
    path_marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
        "recolored_path", 1, true);
    marker_sub_ = nh_.subscribe("mav_local_planner/local_path", 1,
                                &TrajectoryRecolor::markerCallback, this);
  }

  void markerCallback(const visualization_msgs::MarkerArray& marker_msg);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher path_marker_pub_;
  ros::Subscriber marker_sub_;

  int counter_;
  int max_plans_;

  visualization_msgs::MarkerArray marker_cache_;
};

void TrajectoryRecolor::markerCallback(
    const visualization_msgs::MarkerArray& marker_msg) {
  // Makes a copy, since we take a const ref.
  for (visualization_msgs::Marker marker : marker_msg.markers) {
    std_msgs::ColorRGBA color =
        percentToRainbowColor(static_cast<double>(counter_) / max_plans_);
    color.a = 0.5;
    marker.scale.x = 0.025;
    marker.scale.y = 0.025;
    marker.scale.z = 0.025;
    marker.color = color;
    marker.id = counter_++;
    marker_cache_.markers.push_back(marker);
  }

  path_marker_pub_.publish(marker_cache_);

  ROS_INFO("Counter: %d", counter_);
  if (counter_ > max_plans_) {
    counter_ = counter_ % max_plans_;
  }
}

}  // namespace mav_planning

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_recolor");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  mav_planning::TrajectoryRecolor node(nh, nh_private);

  ROS_INFO("[Trajectory Recolor] Started node.");

  ros::spin();
}
