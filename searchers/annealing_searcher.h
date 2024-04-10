#pragma once

#include "plan.h"
#include "planners/prioritized_planner.h"
#include "search_util.h"

Plan plan_multiple_arms_simulated_annealing(
    rai::Configuration C, const RobotTaskPoseMap &rtpm,
    const std::unordered_map<Robot, arr> &home_poses) {
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);
  std::stringstream buffer;
  buffer << "simulated_annealing_" << std::put_time(&tm, "%Y%m%d_%H%M%S");

  auto start_time = std::chrono::high_resolution_clock::now();

  // generate random sequence of robot/pt pairs
  std::vector<Robot> robots;
  for (const auto &element : home_poses) {
    robots.push_back(element.first);
  }
  // const uint num_tasks = rtpm.begin()->second.size();
  int num_tasks = 0;
  for (auto f : C.frames) {
    if (f->name.contains("obj")) {
      num_tasks += 1;
    }
  }
  auto seq = generate_random_sequence(robots, num_tasks);

  // plan for it
  const auto plan_result =
      plan_multiple_arms_given_sequence(C, rtpm, seq, home_poses);

  auto best_plan = plan_result.plan;
  uint best_makespan = get_makespan_from_plan(plan_result.plan);

  uint curr_makespan = best_makespan;

  std::cout << "Initial path with makespan " << best_makespan << std::endl
            << std::endl;

  {
    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time)
            .count();
    export_plan(C, robots, home_poses, best_plan, seq, buffer.str(), 0, duration);
  }

  auto p = [](const double e, const double eprime, const double temperature) {
    if (eprime < e) {
      return 1.;
    }

    return exp(-(eprime - e) / temperature);
  };

  const uint max_iter = 1000;
  const double T0 = 1e6;

  double T = T0;
  double cooling_factor = 0.999;

  std::vector<uint> best_makespan_at_iteration;
  std::vector<double> computation_time_at_iteration;

  for (uint i = 0; i < max_iter; ++i) {
    // T = T0 * (1 - (i+1.)/nmax);
    T = T * cooling_factor; // temp(i);

    // modify sequence
    const OrderedTaskSequence seq_new = neighbour(seq, robots);

    // compute lower bound
    const double lb_makespan =
        compute_lb_for_sequence(seq_new, rtpm, home_poses);

    arr rnd(1);
    rndUniform(rnd);

    if (p(curr_makespan, lb_makespan, T) > rnd(0)) {
      const auto new_plan_result =
          plan_multiple_arms_given_sequence(C, rtpm, seq_new, home_poses);

      if (new_plan_result.status == PlanStatus::success) {
        const auto end_time = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  end_time - start_time)
                                  .count();

        const Plan new_plan = new_plan_result.plan;
        const double makespan = get_makespan_from_plan(new_plan);

        export_plan(C, robots, home_poses, new_plan, seq_new, buffer.str(), i + 1,
                    duration);

        std::cout << "\n\n\nMAKESPAN " << makespan << " best so far "
                  << best_makespan << std::endl;
        for (const auto &s : seq_new) {
          std::cout << "(" << s.robots[0] << " " << s.task.object << ")";
        }

        if (p(curr_makespan, makespan, T) > rnd(0)) {
          curr_makespan = makespan;
          seq = seq_new;
        }

        if (makespan < best_makespan) {
          best_makespan = makespan;
          best_plan = new_plan;

          const std::string image_path =
              global_params.output_path + buffer.str() + "/" + std::to_string(i) + "/img/";
          visualize_plan(C, best_plan, true, image_path);
        }
      }
    }

    best_makespan_at_iteration.push_back(best_makespan);
    computation_time_at_iteration.push_back(i);
  }

  return best_plan;
}