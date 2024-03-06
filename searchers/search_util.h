#pragma once
#include <Core/array.h>
#include "plan.h"
#include "../planners/prioritized_planner.h"

double estimate_task_duration(const arr &start_pose, const arr &goal_pose,
                              const double max_vel, const double max_acc) {
  const double max_dist = absMax(goal_pose - start_pose);
  const double time_to_accelerate = max_vel / max_acc;
  const double acc_dist = 0.5 * max_vel * max_vel / max_acc * 2;

  double dt = 0;
  if (acc_dist > max_dist) {
    // this is wrong
    std::cout << "using acc. timing only" << std::endl;
    dt = 2 * time_to_accelerate;
  } else {
    std::cout << "using acc. timing and max-vel" << std::endl;
    dt = (max_dist - acc_dist) / max_vel + 2 * time_to_accelerate;
  }

  return dt;
}

double compute_lb_for_sequence(const OrderedTaskSequence &seq,
                               const RobotTaskPoseMap &rtpm,
                               const std::unordered_map<Robot, arr> &start_poses,
                               const uint start_index = 0,
                               const std::unordered_map<Robot, double> start_times = {}) {
  // the lower bound can be computed by finding the minimum time
  // of the current task, and using the precedence constraints as well.
  std::unordered_map<Robot, double> robot_time = start_times;
  std::unordered_map<Robot, arr> robot_pos = start_poses;

  for (uint i = start_index; i < seq.size(); ++i) {
    const auto task_tuple = seq[i];
    const auto robot = task_tuple.robots[0];
    const auto task_index = task_tuple.task.object;

    // std::cout << robot << std::endl;

    if (!robot_time.count(robot)) {
      robot_time[robot] = 0.;
    }

    std::cout << robot << " " << task_index << std::endl;

    const arr start_pose = robot_pos[robot];
    const arr goal_pose = rtpm.at(task_tuple)[0][0];

    const double max_acc = 0.1;

    const double dt =
        estimate_task_duration(start_pose, goal_pose, robot.vmax, max_acc);
    robot_time[robot] += dt;

    // since we know that there is a precendence constraint on the
    // task-completion-order we set the time to the highest current time if it
    // was lower than that
    for (const auto &rt : robot_time) {
      if (robot_time[robot] < rt.second) {
        robot_time[robot] = rt.second;
      }
    }

    robot_pos[robot] = goal_pose;
  }

  // extract the maximum time of all the robot times
  double max = 0;
  for (const auto &rt : robot_time) {
    if (max < rt.second) {
      max = rt.second;
    }
  }

  return max;
}


bool sequence_is_feasible(const OrderedTaskSequence &seq,
                          const RobotTaskPoseMap &rtpm) {
  for (const auto &s : seq) {
    const Robot r = s.robots[0];
    const auto task_index = s.task.object;

    if (rtpm.at(s).size() == 0) {
      return false;
    }
  }

  return true;
}