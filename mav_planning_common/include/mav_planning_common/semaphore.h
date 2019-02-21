#ifndef MAV_PLANNING_COMMON_SEMAPHORE_H_
#define MAV_PLANNING_COMMON_SEMAPHORE_H_

#include <ros/ros.h>
#include <condition_variable>
#include <mutex>

namespace mav_planning {

// A simple semaphore, taken from StackOverflow:
// https://stackoverflow.com/questions/4792449/c0x-has-no-semaphores-how-to-synchronize-threads
// All credit to Tsuneo Yoshioka.
class RosSemaphore {
 public:
  RosSemaphore(int count = 0) : count_(count) {}

  inline void notify() {
    std::unique_lock<std::mutex> lock(mutex_);
    count_++;
    cv_.notify_one();
  }

  inline bool wait_for(double sec) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!cv_.wait_for(lock, std::chrono::duration<double>(sec),
                      [this]() { return count_ > 0 || !ros::ok(); })) {
      return false;
    }
    if (count_ > 0) {
      count_--;
      return true;
    }
    return false;
  }

 private:
  std::mutex mutex_;
  std::condition_variable cv_;
  int count_;
};

}  // namespace mav_planning

#endif  // MAV_PLANNING_COMMON_SEMAPHORE_H_
