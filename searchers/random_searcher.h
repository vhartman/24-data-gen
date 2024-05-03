#pragma once

#include "../planners/prioritized_planner.h"
#include "planners/plan.h"
#include "search_util.h"
#include "sequencing.h"

#include "common/config.h"

Plan plan_multiple_arms_random_search(
    rai::Configuration &C, const RobotTaskPoseMap &rtpm,
    const std::unordered_map<Robot, arr> &home_poses,
    const uint max_attempts = 1000) {
  // make foldername for current run
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);

  std::stringstream buffer;
  buffer << "random_search_" << std::put_time(&tm, "%Y%m%d_%H%M%S");

  // generate random sequence of robot/pt pairs
  std::vector<Robot> robots;
  for (const auto &element : home_poses) {
    robots.push_back(element.first);
  }
  int num_tasks = 0;
  for (auto f : C.frames) {
    if (f->name.contains("obj")) {
      num_tasks += 1;
    }
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  OrderedTaskSequence best_seq;
  Plan best_plan;
  double best_makespan = 1e6;

  std::unordered_set<OrderedTaskSequence> all_sequences;

  for (uint i = 0; i < max_attempts; ++i) {
    // const auto seq = generate_random_sequence(robots, num_tasks);

    // if (sequence_is_feasible(seq, rtpm)) {
    //   spdlog::info("Generated sequence not feasible.");
    //   continue;
    // }

    const auto seq = generate_random_valid_sequence(robots, num_tasks, rtpm);

    if (seq.size() == 0) {
      return Plan();
    }

    // check if the sequence was already evaluated at some point
    if (all_sequences.count(seq) > 0){
      continue;
    }
    all_sequences.insert(seq);

    // const double lb = compute_lb_for_sequence(seq, rtpm, home_poses);
    // std::cout << "LB for sequence " << lb << std::endl;
    // for (auto s : seq) {
    //   std::cout << "(" << s.robots[0] << " " << s.task.object << ")";
    // }
    // std::cout << std::endl;

    // if (lb > best_makespan) {
    //   continue;
    // }

    // plan for it
    const auto plan_result = plan_multiple_arms_given_sequence(
        C, rtpm, seq, home_poses, best_makespan);
    if (plan_result.status == PlanStatus::success) {
      const Plan plan = plan_result.plan;
      const double makespan = get_makespan_from_plan(plan);

      spdlog::info("Current MAKESPAN {}, best so far: {}", makespan,
                   best_makespan);
      std::stringstream ss;
      for (const auto &s : seq) {
        ss << "(" << s.serialize() << ")";
      }
      spdlog::info(ss.str());

      const auto end_time = std::chrono::high_resolution_clock::now();
      const auto duration =
          std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
                                                                start_time)
              .count();

      export_plan(C, robots, home_poses, plan, seq, buffer.str(), i, duration);

      if (makespan < best_makespan) {
        best_makespan = makespan;
        best_plan = plan;

        if (global_params.export_images) {
          const std::string image_path = global_params.output_path +
                                         buffer.str() + "/" +
                                         std::to_string(i) + "/img/";
          visualize_plan(C, best_plan, global_params.allow_display, image_path);
        } else {
          visualize_plan(C, best_plan, global_params.allow_display);
        }
      }
    }
  }
  return best_plan;
}