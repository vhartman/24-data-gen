#pragma once

#include "spdlog/spdlog.h"

#include "../samplers/sampler.h"

#include "common/env_util.h"
#include "common/types.h"

#include <experimental/filesystem>

void report_test_result(const std::string& text, const bool success){
  if (success){
     std::cout << "\033[1;32m" << text << "\033[0m" << std::endl;
  }
  else{
     std::cout << "\033[1;31m" << text << "\033[0m" << std::endl;
  }
}

bool check_plan_validity(rai::Configuration C, const std::vector<Robot> robots,
                         const Plan &plan,
                         const std::unordered_map<Robot, arr> &home_poses) {
  const uint makespan = get_makespan_from_plan(plan);
  spdlog::info("Makespan is {}", makespan);

  for (uint t = 0; t < makespan; ++t) {
    for (const auto &r : robots) {
      setActive(C, r);
      arr pose = get_robot_pose_at_time(t, r, home_poses, plan);
      C.setJointState(pose);
    }

    ConfigurationProblem cp(C);
    const auto res = cp.query({}, false);

    if (!res->isFeasible) {
      spdlog::error("Path is not feasible. Collision at time {}", t);
      return false;
    }
  }

  return true;
}

bool run_test_problem_from_files(const std::string &env_path,
                                 const std::string &obj_path,
                                 const std::string seq_path = "") {
  // setup_problem_from_files();
  rai::Configuration C;
  auto robots = make_robot_environment_from_config(C, env_path);
  add_objects_from_config(C, obj_path);

  int num_tasks = 0;
  for (auto f : C.frames) {
    if (f->name.contains("obj")) {
      num_tasks += 1;
    }
  }

  RobotTaskPoseMap robot_task_pose_mapping;
  RobotTaskPoseMap pick_rtpm = compute_all_pick_and_place_positions(C, robots);
  robot_task_pose_mapping.insert(pick_rtpm.begin(), pick_rtpm.end());

  RobotTaskPoseMap handover_rtpm = compute_all_handover_poses(C, robots);
  robot_task_pose_mapping.insert(handover_rtpm.begin(), handover_rtpm.end());

  // TODO: add expected number of keyframes
  std::cout << robot_task_pose_mapping.size() << " poses computed." << std::endl;

  OrderedTaskSequence seq;
  if (seq_path.length() > 0) {
    seq = load_sequence_from_json(seq_path, robots);
  } else {
    // generate a sequence randomly
    seq = generate_random_valid_sequence(robots, num_tasks,
                                         robot_task_pose_mapping);
  }

  if (seq.size() == 0){
    return false;
  }

  const std::unordered_map<Robot, arr> home_poses =
      get_robot_home_poses(robots);
  // run_planning_problem();
  const auto plan_result = plan_multiple_arms_given_sequence(
      C, robot_task_pose_mapping, seq, home_poses);

  return plan_result.status == PlanStatus::success;
}

void run_all_test_problems_from_folder(const std::string &path) {
  std::vector<std::string> folders;
  // collect all folders
  for (const auto &entry :
       std::experimental::filesystem::directory_iterator(path)) {
    if (std::experimental::filesystem::is_directory(entry.path())) {
      folders.push_back(entry.path().string());
    }
  }

  for (const auto &folder : folders) {
    std::cout << "Running " << folder << std::endl;

    // get env file and obj file
    std::string obj_file;
    std::string env_file;
    std::string seq_file;
    for (const auto &entry :
         std::experimental::filesystem::directory_iterator(folder)) {
      if (std::experimental::filesystem::is_regular_file(entry.path())) {
        const std::string filename = entry.path().string();

        if (filename.find("obj") != std::string::npos) {
          obj_file = filename;
        }
        if (filename.find("env") != std::string::npos) {
          env_file = filename;
        }
        if (filename.find("seq") != std::string::npos) {
          seq_file = filename;
        }
      }
    }

    if (obj_file.length() == 0) {
      spdlog::error("Did not find obj-file in '{}'", folder);
      continue;
    }
    if (env_file.length() == 0) {
      spdlog::error("Did not find env-file in '{}'", folder);
      continue;
    }

    const bool success =
        run_test_problem_from_files(env_file, obj_file, seq_file);
    if (!success) {
      spdlog::error("Was not able to find a plan for the problem in {}", folder);
      report_test_result("Failure", false);
    }
    else{
      report_test_result("Success", true);
    }
  }
}

void single_arm_two_finger_keyframe_test(const bool show = false,
                                         const bool export_images = false) {
  spdlog::info("Running single arm keyframe test");

  rai::Configuration C;
  const auto robots = single_robot_configuration(C, true);

  const uint num_objects = 2;
  shuffled_line(C, num_objects, 0.3, false);

  // C.watch(true);

  const auto rtpm = compute_all_pick_and_place_positions(C, robots);

  assert(rtpm.size() == num_objects);
  spdlog::info("Found {} of {} possible solutions", rtpm.size(), num_objects);

  for (const auto &r : rtpm) {
    // we should get feasible poses for all the robots in this setting
    for (const arr &pose : r.second[0]) {
      setActive(C, r.first.robots);
      const arr initial_pose = C.getJointState();

      if (show) {
        // std::cout << pose << std::endl;
        C.setJointState(pose);
        C.watch(true);
      }

      ConfigurationProblem cp(C);
      auto res = cp.query(pose);
      assert(res->isFeasible);

      // set back to home pose
      C.setJointState(initial_pose);
    }
  }

  report_test_result("Success", true);
}

void two_arms_two_finger_keyframe_test(const bool show = false, const bool export_images = false) {
  spdlog::info("Running dual arm keyframe test");

  rai::Configuration C;
  const auto robots = two_robot_configuration(C, true);

  const uint num_objects = 2;
  shuffled_line(C, num_objects, 0.3, false);

  // C.watch(true);

  const auto rtpm = compute_all_pick_and_place_positions(C, robots);

  spdlog::info("Found {} of {} possible solutions", rtpm.size(), num_objects * 2);
  assert(rtpm.size() == num_objects * 2);

  for (const auto &r : rtpm) {
    // we should get feasible poses for all the robots in this setting
    for (const arr &pose : r.second[0]) {
      setActive(C, r.first.robots);
      const arr initial_pose = C.getJointState();

      if (show) {
        // std::cout << pose << std::endl;
        C.setJointState(pose);
        C.watch(true);
      }

      ConfigurationProblem cp(C);
      auto res = cp.query(pose);
      assert(res->isFeasible);

      // set back to home pose
      C.setJointState(initial_pose);
    }
  }

  report_test_result("Success", true);
}

void three_arms_two_finger_keyframe_test(const bool show = false, const bool export_images = false) {
  spdlog::info("Running triple arm keyframe test");

  rai::Configuration C;
  const auto robots = opposite_three_robot_configuration(C, true);
  const uint num_objects = 2;
  shuffled_line(C, 2, 0.3, false);

  // C.watch(true);

  const auto rtpm = compute_all_pick_and_place_positions(C, robots);

  spdlog::info("Found {} of {} possible solutions", rtpm.size(), num_objects * 3);
  assert(rtpm.size() == num_objects * 3);

  for (const auto &r : rtpm) {
    // we should get feasible poses for all the robots in this setting
    for (const arr &pose : r.second[0]) {
      setActive(C, r.first.robots);
      const arr initial_pose = C.getJointState();

      if (show) {
        // std::cout << pose << std::endl;
        C.setJointState(pose);
        C.watch(true);
      }

      ConfigurationProblem cp(C);
      auto res = cp.query(pose);
      assert(res->isFeasible);

      // set back to home pose
      C.setJointState(initial_pose);
    }
  }
  report_test_result("Success", true);
}

void two_arm_two_finger_handover_keyframe_test(const bool show = false, const bool export_images = false) {
  spdlog::info("Running dual arm handover keyframe test");

  rai::Configuration C;
  const auto robots = two_robot_configuration(C, true);

  const uint num_objects = 2;
  shuffled_line(C, num_objects, 0.3, false);

  // C.watch(true);

  const auto rtpm = compute_all_handover_poses(C, robots);
  spdlog::info("Found {} of {} possible solutions", rtpm.size(), num_objects * 2);

  for (const auto &r : rtpm) {
    // we should get feasible poses for all the robots in this setting
    for (uint i = 0; i < 3; ++i) {
      const arr pose = r.second[0][i];
      if (i == 0) {
        setActive(C, r.first.robots[0]);
      } else if (i == 1) {
        setActive(C, r.first.robots);
      } else if (i == 2) {
        setActive(C, r.first.robots[1]);
      }
      const arr initial_pose = C.getJointState();

      if (show) {
        // std::cout << pose << std::endl;
        C.setJointState(pose);
        C.watch(true);
      }

      ConfigurationProblem cp(C);
      auto res = cp.query(pose);
      assert(res->isFeasible);

      // set back to home pose
      C.setJointState(initial_pose);
    }
  }

  assert(rtpm.size() == 4);
  report_test_result("Success", true);
}

void three_arm_two_finger_handover_keyframe_test(const bool show = false, const bool export_images = false) {
  spdlog::info("Running triple arm handover keyframe test");

  rai::Configuration C;
  const auto robots = opposite_three_robot_configuration(C, true);

  const uint num_objects = 2;
  shuffled_line(C, num_objects, 0.3, false);

  // C.watch(true);

  const auto rtpm = compute_all_handover_poses(C, robots);
  spdlog::info("Found {} of {} possible solutions", rtpm.size(), num_objects * 2 * 3);

  for (const auto &r : rtpm) {
    // we should get feasible poses for all the robots in this setting
    for (uint i = 0; i < 3; ++i) {
      const arr pose = r.second[0][i];
      if (i == 0) {
        setActive(C, r.first.robots[0]);
      } else if (i == 1) {
        setActive(C, r.first.robots);
      } else if (i == 2) {
        setActive(C, r.first.robots[1]);
      }
      const arr initial_pose = C.getJointState();

      if (show) {
        // std::cout << pose << std::endl;
        C.setJointState(pose);
        C.watch(true);
      }

      ConfigurationProblem cp(C);
      auto res = cp.query(pose);
      assert(res->isFeasible);

      // set back to home pose
      C.setJointState(initial_pose);
    }
  }
}

void single_arm_two_finger_planning_test(const bool show = false) {
  spdlog::info("Running single arm planning test");

  rai::Configuration C;
  const auto robots = single_robot_configuration(C, true);
  shuffled_line(C, 2, 0.3, false);

  // C.watch(true);

  const auto home_poses = get_robot_home_poses(robots);
  const auto rtpm = compute_all_pick_and_place_positions(C, robots);

  assert(rtpm.size() == 2);

  for (const auto &r : rtpm) {
    // we should get feasible poses for all the robots in this setting
    for (const arr &pose : r.second[0]) {
      setActive(C, r.first.robots);
      const arr initial_pose = C.getJointState();

      if (show) {
        // std::cout << pose << std::endl;
        C.setJointState(pose);
        C.watch(true);
      }

      ConfigurationProblem cp(C);
      auto res = cp.query(pose);
      assert(res->isFeasible);

      // set back to home pose
      C.setJointState(initial_pose);
    }
  }

  const auto sequence = generate_random_sequence(robots, 2);
  const auto plan_result =
      plan_multiple_arms_given_sequence(C, rtpm, sequence, home_poses);

  check_plan_validity(C, robots, plan_result.plan, home_poses);

  if (show){
    visualize_plan(C, plan_result.plan);
  }
}

void two_arm_two_finger_planning_test(const bool show = false) {
  spdlog::info("Running dual arm planning test");

  for (uint i = 0; i < 3; ++i) {
    rai::Configuration C;
    const auto robots = two_robot_configuration(C, true);

    const uint num_objects = i + 2;
    shuffled_line(C, num_objects, 0.17 * num_objects, false);

    C.watch(true);

    const auto home_poses = get_robot_home_poses(robots);
    const auto rtpm = compute_all_pick_and_place_positions(C, robots);

    assert(rtpm.size() == num_objects * 2);

    for (const auto &r : rtpm) {
      // we should get feasible poses for all the robots in this setting
      for (const arr &pose : r.second[0]) {
        setActive(C, r.first.robots);
        const arr initial_pose = C.getJointState();

        if (show) {
          //   std::cout << pose << std::endl;
          C.setJointState(pose);
          C.watch(true);
        }

        ConfigurationProblem cp(C);
        auto res = cp.query(pose);
        assert(res->isFeasible);

        // set back to home pose
        C.setJointState(initial_pose);
      }
    }

    const auto sequence = generate_random_sequence(robots, num_objects);
    const auto plan_result =
        plan_multiple_arms_given_sequence(C, rtpm, sequence, home_poses);
    
    check_plan_validity(C, robots, plan_result.plan, home_poses);

    if (show){
      visualize_plan(C, plan_result.plan);
    }
  }
}

void three_arm_two_finger_planning_test(const bool show = false) {
  spdlog::info("Running triple arm planning test");

  for (uint i = 0; i < 3; ++i) {
    rai::Configuration C;
    const auto robots = opposite_three_robot_configuration(C, true);

    const uint num_objects = i + 2;
    shuffled_line(C, num_objects, 0.16 * num_objects, false);

    // C.watch(true);

    const auto home_poses = get_robot_home_poses(robots);
    const auto rtpm = compute_all_pick_and_place_positions(C, robots);

    assert(rtpm.size() == num_objects * 3);

    for (const auto &r : rtpm) {
      // we should get feasible poses for all the robots in this setting
      for (const arr &pose : r.second[0]) {
        setActive(C, r.first.robots);
        const arr initial_pose = C.getJointState();

        if (show) {
          //   std::cout << pose << std::endl;
          C.setJointState(pose);
          C.watch(true);
        }

        ConfigurationProblem cp(C);
        auto res = cp.query(pose);
        assert(res->isFeasible);

        // set back to home pose
        C.setJointState(initial_pose);
      }
    }

    const auto sequence = generate_random_sequence(robots, num_objects);
    const auto plan_result =
        plan_multiple_arms_given_sequence(C, rtpm, sequence, home_poses);

    if (show) {
      visualize_plan(C, plan_result.plan);
    }
  }
}

void two_arm_two_finger_handover_planning_test(const bool show = false) {
  spdlog::info("Running dual arm handover planning test");

  for (uint i = 0; i < 2; ++i) {
    rai::Configuration C;
    const auto robots = two_robot_configuration(C, true);
    const auto home_poses = get_robot_home_poses(robots);

    const uint num_objects = i + 2;
    shuffled_line(C, num_objects, num_objects * 0.15, false);

    // C.watch(true);

    const auto rtpm = compute_all_handover_poses(C, robots);

    for (const auto &r : rtpm) {
      // we should get feasible poses for all the robots in this setting
      for (uint i = 0; i < 3; ++i) {
        const arr pose = r.second[0][i];
        if (i == 0) {
          setActive(C, r.first.robots[0]);
        } else if (i == 1) {
          setActive(C, r.first.robots);
        } else if (i == 2) {
          setActive(C, r.first.robots[1]);
        }
        const arr initial_pose = C.getJointState();

        if (show) {
          //   std::cout << pose << std::endl;
          C.setJointState(pose);
          C.watch(true);
        }

        ConfigurationProblem cp(C);
        auto res = cp.query(pose);
        assert(res->isFeasible);

        // set back to home pose
        C.setJointState(initial_pose);
      }
    }

    // assert(rtpm.size() == num_objects * 2);

    const auto sequence = make_handover_sequence(robots, num_objects, rtpm);
    const auto plan_result =
        plan_multiple_arms_given_sequence(C, rtpm, sequence, home_poses);

    check_plan_validity(C, robots, plan_result.plan, home_poses);

    if (show){
      visualize_plan(C, plan_result.plan);
    }
  }
}

void three_arm_two_finger_handover_planning_test(const bool show = false) {
  spdlog::info("Running triple arm handover planning test");

  for (uint i = 0; i < 2; ++i) {
    rai::Configuration C;
    const auto robots = opposite_three_robot_configuration(C, true);
    const auto home_poses = get_robot_home_poses(robots);

    const uint num_objects = i + 2;
    shuffled_line(C, num_objects, num_objects * 0.15, false);

    // C.watch(true);

    const auto rtpm = compute_all_handover_poses(C, robots);

    for (const auto &r : rtpm) {
      // we should get feasible poses for all the robots in this setting
      for (uint i = 0; i < 3; ++i) {
        const arr pose = r.second[0][i];
        if (i == 0) {
          setActive(C, r.first.robots[0]);
        } else if (i == 1) {
          setActive(C, r.first.robots);
        } else if (i == 2) {
          setActive(C, r.first.robots[1]);
        }
        const arr initial_pose = C.getJointState();

        if (show) {
          // std::cout << pose << std::endl;
          C.setJointState(pose);
          C.watch(true);
        }

        ConfigurationProblem cp(C);
        auto res = cp.query(pose);
        assert(res->isFeasible);

        // set back to home pose
        C.setJointState(initial_pose);
      }
    }

    // assert(rtpm.size() == num_objects * 2);

    const auto sequence = make_handover_sequence(robots, num_objects, rtpm);
    const auto plan_result =
        plan_multiple_arms_given_sequence(C, rtpm, sequence, home_poses);

    check_plan_validity(C, robots, plan_result.plan, home_poses);

    if (show){
      visualize_plan(C, plan_result.plan);
    }
  }
}