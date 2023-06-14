//
// Created by amanda on 6/14/23.
//

#ifndef UT_VSLAM_CUMULATIVE_TIMER_FACTORY_H
#define UT_VSLAM_CUMULATIVE_TIMER_FACTORY_H

#include <util/timer.h>

#include <memory>
#include <unordered_map>

namespace vslam_types_refactor {

class CumulativeTimerFactory {
 protected:
  CumulativeTimerFactory() = default;

 public:
  // NOTE: Make sure to keep variables returned by this function passed by
  // reference so that the singleton pattern holds.
  static CumulativeTimerFactory& getInstance() {
    static CumulativeTimerFactory factory_instance;
    return factory_instance;
  }

  std::shared_ptr<CumulativeFunctionTimer> getOrCreateFunctionTimer(
      const std::string& timer_name) {
    if (timers_map_.find(timer_name) == timers_map_.end()) {
      timers_map_[timer_name] =
          std::make_shared<CumulativeFunctionTimer>(timer_name.c_str());
    }
    return timers_map_.at(timer_name);
  }

 private:
  std::unordered_map<std::string, std::shared_ptr<CumulativeFunctionTimer>>
      timers_map_;
};
}  // namespace vslam_types_refactor

#endif  // UT_VSLAM_CUMULATIVE_TIMER_FACTORY_H
