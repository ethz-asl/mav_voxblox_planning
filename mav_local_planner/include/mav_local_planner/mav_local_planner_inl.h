#ifndef MAV_LOCAL_PLANNER_MAV_LOCAL_PLANNER_INL_H_
#define MAV_LOCAL_PLANNER_MAV_LOCAL_PLANNER_INL_H_

namespace mav_planning {

template <typename MsgType>
bool MavLocalPlanner::checkFrame(const MsgType& msg) {
  if (msg.header.frame_id == global_frame_id_) {
    ROS_INFO_THROTTLE(
        10, "Waypoints in map frame, will be transformed at publish time.");
    is_trajectory_in_global_frame_ = true;
  } else if (msg.header.frame_id == local_frame_id_) {
    is_trajectory_in_global_frame_ = false;
  } else {
    // Passed plan is in neither the global nor local frames. Check fails.
    return false;
  }
  return true;
}

}  // namespace mav_planning

#endif  // MAV_LOCAL_PLANNER_MAV_LOCAL_PLANNER_INL_H_
