#ifndef MAV_PLANNING_COMMON_COLOR_UTILS_H_
#define MAV_PLANNING_COMMON_COLOR_UTILS_H_

#include <std_msgs/ColorRGBA.h>

namespace mav_planning {

// Maps from a percent to an RGB value on a rainbow scale using math magic.
std_msgs::ColorRGBA percentToRainbowColor(double h);

}  // namespace mav_planning

#endif  // MAV_PLANNING_COMMON_COLOR_UTILS_H_
