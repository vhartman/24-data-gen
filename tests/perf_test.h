#pragma once

#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

#include "../samplers/pick_and_place_sampler.h"

void export_performance_data(){}

void vacuum_gripper_keyframe_perf_test(const uint N=10){
  spdlog::set_level(spdlog::level::err);

  const double vmax = 0.05;

  std::vector<Robot> robots;
  robots.push_back(Robot("a0_", RobotType::ur5, vmax));

  rai::Configuration C;
  single_robot_configuration(C, false);

  const uint num_objects = 2;
  shuffled_line(C, num_objects, 0.3, false);

  // time this
  std::vector<uint> durations;
  for (uint i = 0; i < N; ++i) {
    const auto start_time = std::chrono::high_resolution_clock::now();

    // run
    const auto rtpm = compute_pick_and_place_positions(C, robots);

    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                              end_time - start_time)
                              .count();

    durations.push_back(duration);
  }

  // export data to ./out/performance/vacuum_gripper_keyframes/[date_time]/...  
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);

  std::stringstream buffer;
  buffer << std::put_time(&tm, "%Y%m%d_%H%M%S");

  const std::string folder =
      "./out/performance/vacuum_gripper_keyframes/" + buffer.str() + "/";
  const int res = system(STRING("mkdir -p " << folder).p);
  (void)res;

  {
    std::ofstream f;
    f.open(folder + "durations_in_us.txt", std::ios_base::trunc);
    for (auto d: durations){
      f << d << std::endl;
    }
  }

}

void two_finger_gripper_keyframe_perf_test(){}

void vacuum_gripper_planning_perf_test(){}

void vacuum_gripper_handover_planning_perf_test(){}

void two_finger_gripper_planning_perf_test(){}

void two_finger_gripper_handover_planning_perf_test(){}