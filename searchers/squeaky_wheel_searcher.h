#pragma once

#include "../planners/prioritized_planner.h"
#include "planners/plan.h"
#include "search_util.h"
#include "sequencing.h"

#include "common/config.h"

Plan plan_multiple_arms_squeaky_wheel(
    rai::Configuration &C, const RobotTaskPoseMap &rtpm,
    const std::unordered_map<Robot, arr> &home_poses) {
  return {};
}