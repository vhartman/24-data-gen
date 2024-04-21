#pragma once

#include "spdlog/spdlog.h"

#include "../samplers/sampler.h"
#include "env_util.h"
#include "types.h"

#include <experimental/filesystem>

void benchmark_single_arm_pick_and_place_success_rate(
    const bool show = false, const bool export_images = false) {
  spdlog::info("Running single arm success rate test");

  const uint N = 100;
  uint cnt_success = 0;

  rai::Configuration C;
  const auto robots = single_robot_configuration(C, true);

  const arr base_pos = C["a0_base"]->getPosition();

  // add obstacles
  for (uint j = 0; j < 5; ++j) {
    double width = rnd.uni(0.05, 0.2);
    double depth = rnd.uni(0.05, 0.2);

    auto *obj = C.addFrame(STRING("obs" << j), "table");

    obj->setShape(rai::ST_box, {depth, width, 0.07, 0.01});
    obj->setContact(-1);
    obj->setJoint(rai::JT_rigid);
    obj->setColor({0, 0.6, 0.2, 0.9});

    while (true) {
      const double x = rnd.uni(-1, 1);
      const double y = rnd.uni(-1, 1);

      if (length(ARR(x, y, 0)) < 0.3) {
        continue;
      }

      obj->setRelativePosition({base_pos(0) + x, base_pos(1) + y, 0.07});
      // obj->setRelativeQuaternion(base_quat);

      // check if something is in collision
      ConfigurationProblem cp(C);
      cp.activeOnly = false;
      const auto res = cp.query({}, false);
      if (res->isFeasible) {
        break;
      }
    }
  }

  auto *obj = C.addFrame("obj1", "table");

  auto *marker = C.addFrame("marker", obj->name);
  marker->setShape(rai::ST_marker, {0.1});
  marker->setContact(0.);

  auto *goal = C.addFrame("goal1", "table");

  double total_duration = 0.;

  for (uint i = 0; i < N; ++i) {
    // add object to move
    double width = rnd.uni(0.02, 0.04);
    double depth = rnd.uni(0.04, 0.08);

    obj->setShape(rai::ST_box, {depth, width, 0.05, 0.01});
    obj->setContact(1);
    obj->setJoint(rai::JT_rigid);
    // obj->setColor({col(0), col(1), col(2)});

    while (true) {
      const double x = rnd.uni(-0.8, 0.8);
      const double y = rnd.uni(-0.8, 0.8);

      const double alpha = rnd.uni(0, 2 * 3.1415);

      if (length(ARR(x, y, 0)) > 0.9 || length(ARR(x, y, 0)) < 0.3) {
        continue;
      }

      obj->setPosition({base_pos(0) + x, base_pos(1) + y, 0.61});
      obj->setRelativeQuaternion({cos(alpha / 2), 0, 0, sin(alpha / 2)});

      // check if something is in collision
      ConfigurationProblem cp(C);
      const auto res = cp.query({}, false);
      if (res->isFeasible) {
        // std::cout << "goal rot: " << alpha << std::endl;

        break;
      }
    }

    goal->setShape(rai::ST_box, {depth, width, 0.05, 0.03});
    goal->setContact(1);
    goal->setJoint(rai::JT_rigid);
    goal->setColor({0, 0, 0, 0.5});

    while (true) {
      const double x = rnd.uni(-0.8, 0.8);
      const double y = rnd.uni(-0.8, 0.8);
      const double alpha = rnd.uni(0, 2 * 3.1415);

      // ensure that it is not too close to the base
      if (length(ARR(x, y, 0)) < 0.3 || length(ARR(x, y, 0)) > 0.85) {
        continue;
      }

      goal->setPosition({base_pos(0) + x, base_pos(1) + y, 0.61});
      goal->setRelativeQuaternion({cos(alpha / 2), 0, 0, sin(alpha / 2)});

      // goal->setColor({col(0), col(1), col(2), 0.5});

      // check if something is in collision
      ConfigurationProblem cp(C);
      const auto res = cp.query({}, false);
      if (res->isFeasible) {

        // std::cout << "goal rot: " << alpha << std::endl;
        break;
      }
    }

    goal->setContact(0);

    // C.watch(true);

    const auto start_time = std::chrono::high_resolution_clock::now();

    const auto rtpm = compute_all_pick_and_place_positions(C, robots);

    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                              end_time - start_time)
                              .count();

    total_duration += duration;

    spdlog::info("Found {} of {} possible solutions", rtpm.size(), 1);

    if (rtpm.size() == 1) {
      cnt_success += 1;
      report_test_result("Success", true);
    } else {
      report_test_result("Failure", false);
      std::cout << "goal: " << length(C["goal1"]->getPosition() - base_pos)
                << std::endl;
      std::cout << "obj " << length(C["obj1"]->getPosition() - base_pos)
                << std::endl;
      // C.watch(true);
    }
  }

  std::cout << cnt_success << " out of " << N << " solutions found"
            << std::endl;
  std::cout << "Took " << total_duration / 1000 << "ms" << std::endl;

  spdlog::info("Found {} of {} solutions.", cnt_success, N);
}

void benchmark_dual_arm_handover_success_rate(
    const bool show = false, const bool export_images = false) {
  spdlog::info("Running dual arm handover success rate test");

  const uint N = 100;
  uint cnt_success = 0;

  rai::Configuration C;
  const auto robots = two_robot_configuration(C, false);

  const arr r1_base_pos = C["a0_base"]->getPosition();
  const arr r2_base_pos = C["a1_base"]->getPosition();

  auto *obj = C.addFrame("obj1", "table");

  auto *marker = C.addFrame("marker", obj->name);
  marker->setShape(rai::ST_marker, {0.1});
  marker->setContact(0.);

  auto *goal = C.addFrame("goal1", "table");

  double total_duration = 0.;

  for (uint i = 0; i < N; ++i) {
    // add obstacles
    // for (uint j=0; j<5; ++j){
    //   double width = rnd.uni(0.05, 0.2);
    //   double depth = rnd.uni(0.05, 0.2);

    //   auto *obj = C.addFrame(STRING("obs"<<j), "table");

    //   obj->setShape(rai::ST_box, {depth, width, 0.03, 0.01});
    //   obj->setContact(1);
    //   obj->setJoint(rai::JT_rigid);
    //   obj->setColor({0, 0.6, 0.2, 0.9});

    //   while(true){
    //     const double x = rnd.uni(-2, 2);
    //     const double y = rnd.uni(-1, 1);

    //     obj->setRelativePosition({x, y, 0.05});
    //     // obj->setRelativeQuaternion(base_quat);

    //     // check if something is in collision
    //     ConfigurationProblem cp(C);
    //     const auto res = cp.query({}, false);
    //     if (res->isFeasible) {
    //       break;
    //     }
    //   }
    // }

    // add object to move
    double width = rnd.uni(0.03, 0.04);
    double depth = rnd.uni(0.08, 0.1);

    obj->setShape(rai::ST_box, {depth, width, 0.05, 0.01});
    obj->setContact(1);
    obj->setJoint(rai::JT_rigid);
    // obj->setColor({col(0), col(1), col(2)});

    while (true) {
      const double x = rnd.uni(-0.8, 0.8);
      const double y = rnd.uni(-0.8, 0.8);

      const double alpha = rnd.uni(0, 2 * 3.1415);

      if (length(ARR(x, y, 0)) > 0.85 || length(ARR(x, y, 0)) < 0.3) {
        continue;
      }

      obj->setPosition({r1_base_pos(0) + x, r1_base_pos(1) + y, 0.61});
      obj->setRelativeQuaternion({cos(alpha / 2), 0, 0, sin(alpha / 2)});

      // check if something is in collision
      ConfigurationProblem cp(C);
      const auto res = cp.query({}, false);
      if (res->isFeasible) {
        // std::cout << "goal rot: " << alpha << std::endl;

        break;
      }
    }

    goal->setShape(rai::ST_box, {depth, width, 0.05, 0.03});
    goal->setContact(1);
    goal->setJoint(rai::JT_rigid);
    goal->setColor({0, 0, 0, 0.5});

    while (true) {
      const double x = rnd.uni(-0.8, 0.8);
      const double y = rnd.uni(-0.8, 0.8);
      const double alpha = rnd.uni(0, 2 * 3.1415);

      // ensure that it is not too close to the base
      if (length(ARR(x, y, 0)) < 0.3 || length(ARR(x, y, 0)) > 0.9) {
        continue;
      }

      goal->setPosition({r2_base_pos(0) + x, r2_base_pos(1) + y, 0.61});
      goal->setRelativeQuaternion({cos(alpha / 2), 0, 0, sin(alpha / 2)});

      // goal->setColor({col(0), col(1), col(2), 0.5});

      // check if something is in collision
      ConfigurationProblem cp(C);
      const auto res = cp.query({}, false);
      if (res->isFeasible) {

        // std::cout << "goal rot: " << alpha << std::endl;
        break;
      }
    }

    goal->setContact(0);

    // C.watch(true);

    const auto start_time = std::chrono::high_resolution_clock::now();

    // const auto rtpm = compute_all_handover_poses(C, robots);
    const auto sol = compute_handover_pose(C, robots[0], robots[1],
                                           STRING("obj1"), STRING("goal1"));

    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                              end_time - start_time)
                              .count();

    total_duration += duration;

    spdlog::info("Found the solution.");

    if (sol.size() >= 1) {
      cnt_success += 1;
      report_test_result("Success", true);
    } else {
      report_test_result("Failure", false);
      // std::cout << "goal: " << length(C["goal1"]->getPosition() - base_pos)
      // << std::endl; std::cout << "obj " << length(C["obj1"]->getPosition() -
      // base_pos) << std::endl; C.watch(true);
    }

    // for (const auto &r : rtpm) {
    //   // we should get feasible poses for all the robots in this setting
    //   for (const arr &pose : r.second[0]) {
    //     setActive(C, r.first.robots);
    //     const arr initial_pose = C.getJointState();

    //     if (show) {
    //       // std::cout << pose << std::endl;
    //       C.setJointState(pose);
    //       C.watch(true);
    //     }

    //     ConfigurationProblem cp(C);
    //     auto res = cp.query(pose);
    //     if (!res->isFeasible) {
    //       res->writeDetails(cout, cp.C);
    //       cp.C.watch(true);
    //     }

    //     assert(res->isFeasible);

    //     // set back to home pose
    //     C.setJointState(initial_pose);
    //   }
    // }
  }

  std::cout << cnt_success << " out of " << N << " solutions found"
            << std::endl;
  std::cout << "Took " << total_duration / 1000 << "ms" << std::endl;

  spdlog::info("Found {} of {} solutions.", cnt_success, N);
}

void benchmark_dual_arm_pick_pick_success_rate(
    const bool show = false, const bool export_images = false) {
  spdlog::info("Running dual arm repeated pick success rate test");

  const uint N = 100;
  uint cnt_success = 0;

  rai::Configuration C;
  const auto robots = two_robot_configuration(C, false);

  const arr r1_base_pos = C["a0_base"]->getPosition();
  const arr r2_base_pos = C["a1_base"]->getPosition();

  auto *obj = C.addFrame("obj1", "table");
  auto *marker = C.addFrame("marker", obj->name);
  marker->setShape(rai::ST_marker, {0.1});
  marker->setContact(0.);

  auto *goal = C.addFrame("goal1", "table");

  double total_duration = 0.;

  for (uint i = 0; i < N; ++i) {
    // add object to move
    double width = rnd.uni(0.03, 0.04);
    double depth = rnd.uni(0.08, 0.1);

    obj->setShape(rai::ST_box, {depth, width, 0.05, 0.01});
    obj->setContact(1);
    obj->setJoint(rai::JT_rigid);
    // obj->setColor({col(0), col(1), col(2)});

    while (true) {
      const double x = rnd.uni(-0.8, 0.8);
      const double y = rnd.uni(-0.8, 0.8);

      const double alpha = rnd.uni(0, 2 * 3.1415);

      if (length(ARR(x, y, 0)) > 0.85 || length(ARR(x, y, 0)) < 0.3) {
        continue;
      }

      obj->setPosition({r1_base_pos(0) + x, r1_base_pos(1) + y, 0.61});
      obj->setRelativeQuaternion({cos(alpha / 2), 0, 0, sin(alpha / 2)});

      // check if something is in collision
      ConfigurationProblem cp(C);
      const auto res = cp.query({}, false);
      if (res->isFeasible) {
        // std::cout << "goal rot: " << alpha << std::endl;

        break;
      }
    }

    goal->setShape(rai::ST_box, {depth, width, 0.05, 0.03});
    goal->setContact(1);
    goal->setJoint(rai::JT_rigid);
    goal->setColor({0, 0, 0, 0.5});

    while (true) {
      const double x = rnd.uni(-0.8, 0.8);
      const double y = rnd.uni(-0.8, 0.8);
      const double alpha = rnd.uni(0, 2 * 3.1415);

      // ensure that it is not too close to the base
      if (length(ARR(x, y, 0)) < 0.3 || length(ARR(x, y, 0)) > 0.9) {
        continue;
      }

      goal->setPosition({r2_base_pos(0) + x, r2_base_pos(1) + y, 0.61});
      goal->setRelativeQuaternion({cos(alpha / 2), 0, 0, sin(alpha / 2)});

      // goal->setColor({col(0), col(1), col(2), 0.5});

      // check if something is in collision
      ConfigurationProblem cp(C);
      const auto res = cp.query({}, false);
      if (res->isFeasible) {

        // std::cout << "goal rot: " << alpha << std::endl;
        break;
      }
    }

    goal->setContact(0);

    // C.watch(true);

    const auto start_time = std::chrono::high_resolution_clock::now();

    // const auto rtpm = compute_all_pick_and_place_with_intermediate_pose(C,
    // robots);
    const auto sol = compute_pick_and_place_with_intermediate_pose(
        C, robots[0], robots[1], STRING("obj1"), STRING("goal1"));

    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                              end_time - start_time)
                              .count();

    total_duration += duration;

    // ensure that the keyframe we found is the one we are looking for
    if (sol.size() >= 1) {
      cnt_success += 1;
      report_test_result("Success", true);
    } else {
      report_test_result("Failure", false);
      // std::cout << "goal: " << length(C["goal1"]->getPosition() - base_pos)
      // << std::endl; std::cout << "obj " << length(C["obj1"]->getPosition() -
      // base_pos) << std::endl; C.watch(true);
    }

    // for (const auto &r : rtpm) {
    //   // we should get feasible poses for all the robots in this setting
    //   for (const arr &pose : r.second[0]) {
    //     setActive(C, r.first.robots);
    //     const arr initial_pose = C.getJointState();

    //     if (show) {
    //       // std::cout << pose << std::endl;
    //       C.setJointState(pose);
    //       C.watch(true);
    //     }

    //     ConfigurationProblem cp(C);
    //     auto res = cp.query(pose);
    //     if (!res->isFeasible) {
    //       res->writeDetails(cout, cp.C);
    //       cp.C.watch(true);
    //     }

    //     assert(res->isFeasible);

    //     // set back to home pose
    //     C.setJointState(initial_pose);
    //   }
    // }
  }

  std::cout << cnt_success << " out of " << N << " solutions found"
            << std::endl;
  std::cout << "Took " << total_duration / 1000 << "ms" << std::endl;
  spdlog::info("Found {} of {} solutions.", cnt_success, N);
}

void benchmark_dual_arm_multi_pick_planning() {
  spdlog::info("Running dual arm planning success rate test");

  const uint N = 100;
  uint cnt_success = 0;

  rai::Configuration C;
  const auto robots = two_robot_configuration(C, false);
  const auto home_poses = get_robot_home_poses(robots);

  const arr r1_base_pos = C["a0_base"]->getPosition();
  const arr r2_base_pos = C["a1_base"]->getPosition();

  for (uint i = 0; i < 2; ++i) {
    auto *obj = C.addFrame(STRING("obj" << i + 1), "table");
    arr col(3);
    rndUniform(col, 0, 1);
    obj->setColor(col);

    auto *marker = C.addFrame("marker", obj->name);
    marker->setShape(rai::ST_marker, {0.1});
    marker->setContact(0.);

    auto *goal = C.addFrame(STRING("goal" << i + 1), "table");
    goal->setColor({col(0), col(1), col(2), 0.2});
  }

  double total_duration = 0.;

  for (uint i = 0; i < N; ++i) {
    // add object to move

    for (uint j = 0; j < 2; ++j) {
      double width = rnd.uni(0.03, 0.04);
      double depth = rnd.uni(0.08, 0.1);

      auto obj = C[STRING("obj" << j + 1)];
      obj->setShape(rai::ST_box, {depth, width, 0.05, 0.01});
      obj->setContact(1);
      obj->setJoint(rai::JT_rigid);
      // obj->setColor({col(0), col(1), col(2)});

      while (true) {
        const double x = rnd.uni(-0.8, 0.8);
        const double y = rnd.uni(-0.8, 0.8);

        const double alpha = rnd.uni(0, 2 * 3.1415);

        if (length(ARR(x, y, 0)) > 0.85 || length(ARR(x, y, 0)) < 0.3 ||
            x > -0.1) {
          continue;
        }

        obj->setPosition({r1_base_pos(0) + x, r1_base_pos(1) + y, 0.61});
        obj->setRelativeQuaternion({cos(alpha / 2), 0, 0, sin(alpha / 2)});

        // check if something is in collision
        ConfigurationProblem cp(C);
        const auto res = cp.query({}, false);
        if (res->isFeasible) {
          std::cout << "goal rot: " << alpha << std::endl;

          break;
        }
      }

      auto goal = C[STRING("goal" << j + 1)];
      goal->setShape(rai::ST_box, {depth, width, 0.05, 0.03});
      goal->setContact(1);
      goal->setJoint(rai::JT_rigid);

      while (true) {
        const double x = rnd.uni(-0.8, 0.8);
        const double y = rnd.uni(-0.8, 0.8);
        const double alpha = rnd.uni(0, 2 * 3.1415);

        // ensure that it is not too close to the base
        if (length(ARR(x, y, 0)) < 0.3 || length(ARR(x, y, 0)) > 0.9 ||
            x < 0.1) {
          continue;
        }

        goal->setPosition({r2_base_pos(0) + x, r2_base_pos(1) + y, 0.61});
        goal->setRelativeQuaternion({cos(alpha / 2), 0, 0, sin(alpha / 2)});

        // goal->setColor({col(0), col(1), col(2), 0.5});

        // check if something is in collision
        ConfigurationProblem cp(C);
        const auto res = cp.query({}, false);
        if (res->isFeasible) {

          std::cout << "goal rot: " << alpha << std::endl;
          break;
        }
      }
    }

    for (uint j = 0; j < 2; ++j) {
      auto goal = C[STRING("goal" << j + 1)];
      goal->setContact(0);
    }

    // C.watch(true);

    const auto start_time = std::chrono::high_resolution_clock::now();

    const auto rtpm =
        // compute_all_handover_poses(C, robots);
        compute_all_pick_and_place_with_intermediate_pose(C, robots);

    const auto seq = generate_random_valid_sequence(robots, 2, rtpm);

    if (seq.size() == 0){
      report_test_result("Failure", false);
      continue;
    }

    const auto plan_result =
        plan_multiple_arms_given_sequence(C, rtpm, seq, home_poses);

    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                              end_time - start_time)
                              .count();

    total_duration += duration;

    spdlog::info("Found solution");

    // ensure that the keyframe we found is the one we are looking for
    if (plan_result.status == PlanStatus::success) {
      cnt_success += 1;
      report_test_result("Success", true);
    } else {
      report_test_result("Failure", false);
      // std::cout << "goal: " << length(C["goal1"]->getPosition() - base_pos)
      // << std::endl; std::cout << "obj " << length(C["obj1"]->getPosition() -
      // base_pos) << std::endl; C.watch(true);
    }
  }

  std::cout << cnt_success << " out of " << N << " solutions found"
            << std::endl;
  std::cout << "Took " << total_duration / 1000 << "ms" << std::endl;
  spdlog::info("Found {} of {} solutions.", cnt_success, N);
}

void benchmark_dual_arm_planning(const uint N = 50) {
  spdlog::info("Running dual arm planning success rate test");

  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);

  std::stringstream buffer;
  buffer << "dual_arm_planning_benchmark"
         << std::put_time(&tm, "%Y%m%d_%H%M%S");

  uint cnt_success = 0;

  rai::Configuration C;
  const auto robots = two_robot_configuration(C, false);
  const auto home_poses = get_robot_home_poses(robots);

  const arr r1_base_pos = C["a0_base"]->getPosition();
  const arr r2_base_pos = C["a1_base"]->getPosition();

  std::vector<arr> base_poses{r1_base_pos, r2_base_pos};

  const uint num_objs = 4;

  for (uint i = 0; i < num_objs; ++i) {
    auto *obj = C.addFrame(STRING("obj" << i + 1), "table");
    arr col(3);
    rndUniform(col, 0, 1);
    obj->setColor(col);

    auto *marker = C.addFrame("marker", obj->name);
    marker->setShape(rai::ST_marker, {0.1});
    marker->setContact(0.);

    auto *goal = C.addFrame(STRING("goal" << i + 1), "table");
    goal->setColor({col(0), col(1), col(2), 0.2});
  }

  double total_duration = 0.;

  for (uint i = 0; i < N; ++i) {
    spdlog::info("Running scenario {}", i);

    for (uint j = 0; j < num_objs; ++j) {
      double width = rnd.uni(0.03, 0.04);
      double depth = rnd.uni(0.08, 0.1);

      auto obj = C[STRING("obj" << j + 1)];
      obj->setShape(rai::ST_box, {depth, width, 0.05, 0.01});
      obj->setContact(1);
      obj->setJoint(rai::JT_rigid);
      // obj->setColor({col(0), col(1), col(2)});

      while (true) {
        const double x = rnd.uni(-2, 2);
        const double y = rnd.uni(-2, 2);

        const double alpha = rnd.uni(0, 2 * 3.1415);

        bool too_close = false;
        bool too_far_away = true;

        for (const arr& p : base_poses) {
          if (length(ARR(x - p(0), y - p(1), 0)) < 0.3) {
            too_close = true;
          }

          if (length(ARR(x - p(0), y - p(1), 0)) < 0.8) {
            too_far_away = false;
          }
        }

        if (too_close || too_far_away) {
          continue;
        }

        obj->setPosition({x, y, 0.61});
        obj->setRelativeQuaternion({cos(alpha / 2), 0, 0, sin(alpha / 2)});

        // check if something is in collision
        ConfigurationProblem cp(C);
        const auto res = cp.query({}, false);
        if (res->isFeasible) {
          std::cout << "goal rot: " << alpha << std::endl;

          break;
        }
      }

      auto goal = C[STRING("goal" << j + 1)];
      goal->setShape(rai::ST_box, {depth, width, 0.05, 0.03});
      goal->setContact(1);
      goal->setJoint(rai::JT_rigid);

      while (true) {
        const double x = rnd.uni(-2, 2);
        const double y = rnd.uni(-2, 2);

        const double alpha = rnd.uni(0, 2 * 3.1415);

        bool too_close = false;
        bool too_far_away = true;

        for (const arr& p : base_poses) {
          if (length(ARR(x - p(0), y - p(1), 0)) < 0.3) {
            too_close = true;
          }

          if (length(ARR(x - p(0), y - p(1), 0)) < 0.8) {
            too_far_away = false;
          }
        }

        if (too_close || too_far_away) {
          continue;
        }
        goal->setPosition({x, y, 0.61});
        goal->setRelativeQuaternion({cos(alpha / 2), 0, 0, sin(alpha / 2)});

        // goal->setColor({col(0), col(1), col(2), 0.5});

        // check if something is in collision
        ConfigurationProblem cp(C);
        const auto res = cp.query({}, false);
        if (res->isFeasible) {

          std::cout << "goal rot: " << alpha << std::endl;
          break;
        }
      }
    }

    for (uint j = 0; j < num_objs; ++j) {
      auto goal = C[STRING("goal" << j + 1)];
      goal->setContact(0);
    }

    // C.watch(true);

    const auto start_time = std::chrono::high_resolution_clock::now();

    RobotTaskPoseMap rtpm;

    RobotTaskPoseMap pick_rtpm =
        compute_all_pick_and_place_positions(C, robots);
    rtpm.insert(pick_rtpm.begin(), pick_rtpm.end());

    RobotTaskPoseMap handover_rtpm = compute_all_handover_poses(C, robots);
    rtpm.insert(handover_rtpm.begin(), handover_rtpm.end());

    RobotTaskPoseMap pick_pick_rtpm =
        compute_all_pick_and_place_with_intermediate_pose(C, robots);
    rtpm.insert(pick_pick_rtpm.begin(), pick_pick_rtpm.end());

    const auto seq = generate_random_valid_sequence(robots, num_objs, rtpm);

    if (seq.size() == 0){
      // C.watch(true);

      report_test_result("Failure", false);
      continue;
    }

    const auto plan_result =
        plan_multiple_arms_given_sequence(C, rtpm, seq, home_poses);

    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                              end_time - start_time)
                              .count();

    total_duration += duration;

    // ensure that the keyframe we found is the one we are looking for
    if (plan_result.status == PlanStatus::success) {
      cnt_success += 1;
      report_test_result("Success", true);

      const Plan plan = plan_result.plan;

      export_plan(C, robots, home_poses, plan, seq, buffer.str(), i, duration);
    } else {
      report_test_result("Failure", false);
      // std::cout << "goal: " << length(C["goal1"]->getPosition() - base_pos)
      // << std::endl; std::cout << "obj " << length(C["obj1"]->getPosition() -
      // base_pos) << std::endl; C.watch(true);
    }
  }

  std::cout << cnt_success << " out of " << N << " solutions found"
            << std::endl;
  std::cout << "Took " << total_duration / 1000 << "ms" << std::endl;
  spdlog::info("Found {} of {} solutions.", cnt_success, N);
}