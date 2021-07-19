/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include "modules/planning/lattice/trajectory_generation/piecewise_braking_trajectory_generator.h"

#include <cmath>

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

std::shared_ptr<Curve1d> PiecewiseBrakingTrajectoryGenerator::Generate(
    const double s_target, const double s_curr, const double v_target,
    const double v_curr, const double a_comfort, const double d_comfort,
    const double max_time) {
  std::shared_ptr<PiecewiseAccelerationTrajectory1d> ptr_trajectory =
      std::make_shared<PiecewiseAccelerationTrajectory1d>(s_curr, v_curr);

  double s_dist = s_target - s_curr;

  double comfort_stop_dist = ComputeStopDistance(v_curr, d_comfort);

  // if cannot stop using comfort deceleration, then brake in the beginning.
  if (comfort_stop_dist > s_dist) {
    double stop_d = ComputeStopDeceleration(s_dist, v_curr);
    double stop_t = v_curr / stop_d;
    // 第一段的加速度为-stop_d，持续时间stop_t
    ptr_trajectory->AppendSegment(-stop_d, stop_t);
    // 在规划时间内可以刹停，剩下的时间加速度为0，即静止不动
    if (ptr_trajectory->ParamLength() < max_time) {
      ptr_trajectory->AppendSegment(0.0,
                                    max_time - ptr_trajectory->ParamLength());
    }
    return ptr_trajectory;
  }

  // otherwise, the vehicle can stop from current speed with comfort brake.
  if (v_curr > v_target) {
    double t_cruise = (s_dist - comfort_stop_dist) / v_target;
    double t_rampdown = (v_curr - v_target) / d_comfort;
    double t_dec = v_target / d_comfort;
    // 先由当前速度减速到目标速度
    ptr_trajectory->AppendSegment(-d_comfort, t_rampdown);
    // 再以目标速度巡航
    ptr_trajectory->AppendSegment(0.0, t_cruise);
    // 最后减速到0
    ptr_trajectory->AppendSegment(-d_comfort, t_dec);
    // 在规划时间内可以刹停，剩下的时间加速度为0，即静止不动
    if (ptr_trajectory->ParamLength() < max_time) {
      ptr_trajectory->AppendSegment(0.0,
                                    max_time - ptr_trajectory->ParamLength());
    }
    return ptr_trajectory;

  } else {
    double t_rampup = (v_target - v_curr) / a_comfort;
    double t_rampdown = (v_target - v_curr) / d_comfort;
    double s_ramp = (v_curr + v_target) * (t_rampup + t_rampdown) * 0.5;

    double s_rest = s_dist - s_ramp - comfort_stop_dist;
    if (s_rest > 0) {
      double t_cruise = s_rest / v_target;
      double t_dec = v_target / d_comfort;

      // construct the trajectory
      // 先加速到目标车速
      ptr_trajectory->AppendSegment(a_comfort, t_rampup);
      // 再以目标车速运行行驶
      ptr_trajectory->AppendSegment(0.0, t_cruise);
      // 从目标车速减速至0
      ptr_trajectory->AppendSegment(-d_comfort, t_dec);
      // 在规划时间内可以刹停，剩下的时间加速度为0，即静止不动
      if (ptr_trajectory->ParamLength() < max_time) {
        ptr_trajectory->AppendSegment(0.0,
                                      max_time - ptr_trajectory->ParamLength());
      }
      return ptr_trajectory;
    } else {
      double s_rampup_rampdown = s_dist - comfort_stop_dist;
      double v_max = std::sqrt(v_curr * v_curr + 2.0 * a_comfort * d_comfort *
                                                     s_rampup_rampdown /
                                                     (a_comfort + d_comfort));

      double t_acc = (v_max - v_curr) / a_comfort;
      double t_dec = v_max / d_comfort;

      // construct the trajectory
      // 先加速到最大允许速度
      ptr_trajectory->AppendSegment(a_comfort, t_acc);
      // 从最大速度加速至0
      ptr_trajectory->AppendSegment(-d_comfort, t_dec);
      // 在规划时间内可以刹停，剩下的时间加速度为0，即静止不动
      if (ptr_trajectory->ParamLength() < max_time) {
        ptr_trajectory->AppendSegment(0.0,
                                      max_time - ptr_trajectory->ParamLength());
      }
      return ptr_trajectory;
    }
  }
}

double PiecewiseBrakingTrajectoryGenerator::ComputeStopDistance(
    const double v, const double dec) {
  ACHECK(dec > 0.0);
  return v * v / dec * 0.5;
}

double PiecewiseBrakingTrajectoryGenerator::ComputeStopDeceleration(
    const double dist, const double v) {
  return v * v / dist * 0.5;
}

}  // namespace planning
}  // namespace apollo
