#pragma once

#include "spdlog/spdlog.h"

#include "../samplers/sampler.h"
#include "env_util.h"
#include "types.h"

void single_arm_two_finger_keyframe_test(const bool show = false, const bool export_images = false) {
  spdlog::info("Running single arm keyframe test");

  const double vmax = 0.05;

  std::vector<Robot> robots;
  robots.push_back(Robot("a0_", RobotType::ur5, vmax));

  rai::Configuration C;
  single_robot_configuration(C, true);

  const uint num_objects = 2;
  shuffled_line(C, num_objects, 0.3, false);

  // C.watch(true);

  const auto rtpm = compute_pick_and_place_positions(C, robots);

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
}

void two_arms_two_finger_keyframe_test(const bool show = false, const bool export_images = false) {
  spdlog::info("Running dual arm keyframe test");

  const double vmax = 0.05;

  std::vector<Robot> robots;
  robots.push_back(Robot("a0_", RobotType::ur5, vmax));
  robots.push_back(Robot("a1_", RobotType::ur5, vmax));

  rai::Configuration C;
  two_robot_configuration(C, true);

  const uint num_objects = 2;
  shuffled_line(C, num_objects, 0.3, false);

  // C.watch(true);

  const auto rtpm = compute_pick_and_place_positions(C, robots);

  assert(rtpm.size() == num_objects * 2);
  spdlog::info("Found {} of {} possible solutions", rtpm.size(), num_objects * 2);

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
}

void three_arms_two_finger_keyframe_test(const bool show = false, const bool export_images = false) {
  spdlog::info("Running triple arm keyframe test");

  const double vmax = 0.05;

  std::vector<Robot> robots;
  robots.push_back(Robot("a0_", RobotType::ur5, vmax));
  robots.push_back(Robot("a1_", RobotType::ur5, vmax));
  robots.push_back(Robot("a2_", RobotType::ur5, vmax));

  rai::Configuration C;
  opposite_three_robot_configuration(C, true);
  const uint num_objects = 2;
  shuffled_line(C, 2, 0.3);

  // C.watch(true);

  const auto rtpm = compute_pick_and_place_positions(C, robots);

  assert(rtpm.size() == num_objects * 3);
  spdlog::info("Found {} of {} possible solutions", rtpm.size(), num_objects * 3);

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
}

void two_arm_two_finger_handover_keyframe_test(const bool show = false, const bool export_images = false) {
  spdlog::info("Running dual arm handover keyframe test");

  const double vmax = 0.05;

  std::vector<Robot> robots;
  robots.push_back(Robot("a0_", RobotType::ur5, vmax));
  robots.push_back(Robot("a1_", RobotType::ur5, vmax));

  rai::Configuration C;
  two_robot_configuration(C, true);

  const uint num_objects = 2;
  shuffled_line(C, num_objects, 0.3, false);

  // C.watch(true);

  const auto rtpm = compute_handover_poses(C, robots);
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
}

void three_arm_two_finger_handover_keyframe_test(const bool show = false, const bool export_images = false) {
  spdlog::info("Running triple arm handover keyframe test");

  const double vmax = 0.05;

  std::vector<Robot> robots;
  robots.push_back(Robot("a0_", RobotType::ur5, vmax));
  robots.push_back(Robot("a1_", RobotType::ur5, vmax));
  robots.push_back(Robot("a2_", RobotType::ur5, vmax));

  rai::Configuration C;
  opposite_three_robot_configuration(C, true);

  const uint num_objects = 2;
  shuffled_line(C, num_objects, 0.3, false);

  // C.watch(true);

  const auto rtpm = compute_handover_poses(C, robots);
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

// TODO: test if the path is actually feasible

void single_arm_two_finger_planning_test(const bool show = false, const bool export_images = false) {
  spdlog::info("Running single arm planning test");

  const double vmax = 0.05;

  std::vector<Robot> robots;
  robots.push_back(Robot("a0_", RobotType::ur5, vmax));

  rai::Configuration C;
  single_robot_configuration(C, true);
  shuffled_line(C, 2, 0.3, false);

  // C.watch(true);

  const auto home_poses = get_robot_home_poses(C, robots);
  const auto rtpm = compute_pick_and_place_positions(C, robots);

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

  visualize_plan(C, plan_result.plan, export_images, show);
}

void two_arm_two_finger_planning_test(const bool show = false, const bool export_images = false) {
  spdlog::info("Running dual arm planning test");

  const double vmax = 0.05;

  std::vector<Robot> robots;
  robots.push_back(Robot("a0_", RobotType::ur5, vmax));
  robots.push_back(Robot("a1_", RobotType::ur5, vmax));

  for (uint i = 0; i < 3; ++i) {

    rai::Configuration C;
    two_robot_configuration(C, true);

    const uint num_objects = i + 2;
    shuffled_line(C, num_objects, 0.15 * num_objects, false);

    // C.watch(true);

    const auto home_poses = get_robot_home_poses(C, robots);
    const auto rtpm = compute_pick_and_place_positions(C, robots);

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

    visualize_plan(C, plan_result.plan, export_images, show);
  }
}

void three_arm_two_finger_planning_test(const bool show = false, const bool export_images = false) {
  spdlog::info("Running triple arm planning test");

  const double vmax = 0.05;

  std::vector<Robot> robots;
  robots.push_back(Robot("a0_", RobotType::ur5, vmax));
  robots.push_back(Robot("a1_", RobotType::ur5, vmax));
  robots.push_back(Robot("a2_", RobotType::ur5, vmax));

  for (uint i = 0; i < 3; ++i) {
    rai::Configuration C;
    opposite_three_robot_configuration(C, true);

    const uint num_objects = i + 2;
    shuffled_line(C, num_objects, 0.15 * num_objects, false);

    // C.watch(true);

    const auto home_poses = get_robot_home_poses(C, robots);
    const auto rtpm = compute_pick_and_place_positions(C, robots);

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

    visualize_plan(C, plan_result.plan, export_images, show);
  }
}

void two_arm_two_finger_handover_planning_test(const bool show = false, const bool export_images = false) {
  spdlog::info("Running dual arm handover planning test");

  const double vmax = 0.05;

  std::vector<Robot> robots;
  robots.push_back(Robot("a0_", RobotType::ur5, vmax));
  robots.push_back(Robot("a1_", RobotType::ur5, vmax));

  for (uint i = 0; i < 2; ++i) {
    rai::Configuration C;
    two_robot_configuration(C, true);
    const auto home_poses = get_robot_home_poses(C, robots);

    const uint num_objects = i + 2;
    shuffled_line(C, num_objects, num_objects * 0.15, false);

    // C.watch(true);

    const auto rtpm = compute_handover_poses(C, robots);

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

    visualize_plan(C, plan_result.plan, export_images, show);
  }
}

void three_arm_two_finger_handover_planning_test(const bool show = false, const bool export_images = false) {
  spdlog::info("Running triple arm handover planning test");

  const double vmax = 0.05;

  std::vector<Robot> robots;
  robots.push_back(Robot("a0_", RobotType::ur5, vmax));
  robots.push_back(Robot("a1_", RobotType::ur5, vmax));
  robots.push_back(Robot("a2_", RobotType::ur5, vmax));

  for (uint i = 0; i < 2; ++i) {
    rai::Configuration C;
    opposite_three_robot_configuration(C, true);
    const auto home_poses = get_robot_home_poses(C, robots);

    const uint num_objects = i + 2;
    shuffled_line(C, num_objects, num_objects * 0.15, false);

    // C.watch(true);

    const auto rtpm = compute_handover_poses(C, robots);

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

    visualize_plan(C, plan_result.plan, export_images, show);
    
    // ensure that the path is valid
  }
}