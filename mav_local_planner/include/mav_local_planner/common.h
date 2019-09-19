#ifndef MAV_LOCAL_PLANNER_COMMON_H_
#define MAV_LOCAL_PLANNER_COMMON_H_

#include <kindr/minimal/quat-transformation.h>

namespace mav_planning {

using Transformation = kindr::minimal::QuatTransformationTemplate<double>;

}  // namespace mav_planning

#endif  // MAV_LOCAL_PLANNER_COMMON_H_
