#define GTEST_DONT_DEFINE_TEST 1
#include <gtest/gtest.h>

#include "spdlog/spdlog.h"

#include "samplers/sampler.h"
#include <Kin/featureSymbols.h>

#include "searchers/sequencing.h"

#include "common/config.h"
#include "common/env_util.h"
#include "common/types.h"
#include "tests/test_util.h"

#include <experimental/filesystem>

manip::Parameters global_params;

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
      // cp.C.watch(true);
      return false;
    }
  }

  return true;
}

GTEST_TEST(KEYFRAME_TEST, SingleArmRepeatedPickPlaceTest_Vacuum_Reorientation) {
  bool show = false;
  spdlog::set_level(spdlog::level::off);

  rai::Configuration C;
  const auto robots = single_robot_configuration(C, false);
  cubes_with_random_rotation(C, 1);

  const auto rtpm =
      compute_all_pick_and_place_with_intermediate_pose(C, robots, true);

  ASSERT_GE(rtpm.size(), 1);
}

GTEST_TEST(KEYFRAME_TEST, DualArmRepeatedPickPlaceTest_Vacuum_Reorientation) {
  bool show = false;
  spdlog::set_level(spdlog::level::off);

  rai::Configuration C;
  const auto robots = two_robot_configuration(C, false);
  cubes_with_random_rotation(C, 1);

  const auto rtpm =
      compute_all_pick_and_place_with_intermediate_pose(C, robots, true);

  ASSERT_GE(rtpm.size(), 1);

  for (const auto &r : rtpm) {
    if (r.first.task.type != PrimitiveType::pick_pick_1) {
      continue;
    }

    // find the corresponding second task
    TaskPoses tp_second_part;
    for (const auto &r_inner : rtpm) {
      if (r_inner.first.task.type == PrimitiveType::pick_pick_2 &&
          r.first.robots[0] == r_inner.first.robots[0] &&
          r.first.robots[1] == r_inner.first.robots[1]) {
        tp_second_part = r_inner.second[0];

        break;
      }
    }

    for (int i = 0; i < 4; ++i) {
      arr pose;
      if (i < 2) {
        pose = r.second[0][i];
      } else {
        pose = tp_second_part[i - 2];
      }
      if (i == 0) {
        setActive(C, r.first.robots[0]);
      } else if (i == 1) {
        setActive(C, r.first.robots[0]);
      } else if (i == 2) {
        setActive(C, r.first.robots[1]);
      } else if (i == 3) {
        setActive(C, r.first.robots[1]);
      }
      // std::cout << pose << std::endl;
      // std::cout << r.first.robots[0] << std::endl;
      // std::cout << r.first.robots[1] << std::endl;
      const arr initial_pose = C.getJointState();

      if (show) {
        // std::cout << pose << std::endl;
        C.setJointState(pose);
        C.watch(true);
      }

      ConfigurationProblem cp(C);
      auto res = cp.query(pose);
      ASSERT_TRUE(res->isFeasible);

      // set back to home pose
      C.setJointState(initial_pose);
    }
  }
}

GTEST_TEST(KEYFRAME_TEST, SingleArmPickPlaceTest) {
  bool show = false;
  spdlog::set_level(spdlog::level::off);

  rai::Configuration C;
  const auto robots = single_robot_configuration(C, true);

  const uint num_objects = 2;
  shuffled_line(C, num_objects, 0.3, false);

  // C.watch(true);

  const auto rtpm = compute_all_pick_and_place_positions(C, robots);

  ASSERT_EQ(rtpm.size(), num_objects);
  // spdlog::info("Found {} of {} possible solutions", rtpm.size(),
  // num_objects);

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
      ASSERT_TRUE(res->isFeasible);

      // set back to home pose
      C.setJointState(initial_pose);
    }
  }
}

GTEST_TEST(KEYFRAME_TEST, DualArmPickPlaceTest) {
  bool show = false;
  spdlog::set_level(spdlog::level::off);

  rai::Configuration C;
  const auto robots = two_robot_configuration(C, true);

  const uint num_objects = 2;
  shuffled_line(C, num_objects, 0.3, false);

  // C.watch(true);

  const auto rtpm = compute_all_pick_and_place_positions(C, robots);

  spdlog::info("Found {} of {} possible solutions", rtpm.size(),
               num_objects * 2);
  ASSERT_EQ(rtpm.size(), num_objects * 2);

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
      ASSERT_TRUE(res->isFeasible);

      // set back to home pose
      C.setJointState(initial_pose);
    }
  }
}

GTEST_TEST(KEYFRAME_TEST, TripleArmPickPlaceTest) {
  bool show = false;
  spdlog::set_level(spdlog::level::off);

  rai::Configuration C;
  const auto robots = opposite_three_robot_configuration(C, true);
  const uint num_objects = 2;
  shuffled_line(C, 2, 0.3, false);

  // C.watch(true);

  const auto rtpm = compute_all_pick_and_place_positions(C, robots);

  spdlog::info("Found {} of {} possible solutions", rtpm.size(),
               num_objects * 3);
  ASSERT_EQ(rtpm.size(), num_objects * 3);

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
      ASSERT_TRUE(res->isFeasible);

      // set back to home pose
      C.setJointState(initial_pose);
    }
  }
}

GTEST_TEST(KEYFRAME_TEST, DualArmHandoverTest) {
  bool show = false;
  spdlog::set_level(spdlog::level::off);

  rai::Configuration C;
  const auto robots = two_robot_configuration(C, true);

  const uint num_objects = 2;
  shuffled_line(C, num_objects, 0.3, false);

  // C.watch(true);

  const auto rtpm = compute_all_handover_poses(C, robots);
  spdlog::info("Found {} of {} possible solutions", rtpm.size(),
               num_objects * 2);
  ASSERT_EQ(rtpm.size(), 4);

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
      ASSERT_TRUE(res->isFeasible);

      // set back to home pose
      C.setJointState(initial_pose);
    }
  }
}

GTEST_TEST(KEYFRAME_TEST, TripleArmHandoverTest) {
  bool show = false;
  spdlog::set_level(spdlog::level::off);

  rai::Configuration C;
  const auto robots = opposite_three_robot_configuration(C, true);

  const uint num_objects = 2;
  shuffled_line(C, num_objects, 0.3, false);

  // C.watch(true);

  const auto rtpm = compute_all_handover_poses(C, robots);
  spdlog::info("Found {} of {} possible solutions", rtpm.size(),
               num_objects * 2 * 3);
  ASSERT_EQ(rtpm.size(), 12);

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
      ASSERT_TRUE(res->isFeasible);

      // set back to home pose
      C.setJointState(initial_pose);
    }
  }
}

GTEST_TEST(PLANNING_TEST, SingleArmTest) {
  bool show = false;
  spdlog::set_level(spdlog::level::off);

  spdlog::info("Running single arm planning test");

  rai::Configuration C;
  const auto robots = single_robot_configuration(C, true);
  shuffled_line(C, 2, 0.3, false);

  // C.watch(true);

  const auto home_poses = get_robot_home_poses(robots);
  const auto rtpm = compute_all_pick_and_place_positions(C, robots);

  ASSERT_EQ(rtpm.size(), 2);

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
      ASSERT_TRUE(res->isFeasible);

      // set back to home pose
      C.setJointState(initial_pose);
    }
  }

  const auto sequence = generate_random_sequence(robots, 2);
  const auto plan_result =
      plan_multiple_arms_given_sequence(C, rtpm, sequence, home_poses);

  if (show) {
    visualize_plan(C, plan_result.plan);
  }

  ASSERT_TRUE(check_plan_validity(C, robots, plan_result.plan, home_poses));
}

GTEST_TEST(UTIL_TEST, SetAndLinkToPhaseTest) {
  // TODO
}