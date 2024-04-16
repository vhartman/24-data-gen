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
  const auto robots = single_robot_configuration(C, false);

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

  for (uint i=0; i<N; ++i){
    // add object to move
    double width = rnd.uni(0.02, 0.04);
    double depth = rnd.uni(0.04, 0.08);


    obj->setShape(rai::ST_box, {depth, width, 0.05, 0.01});
    obj->setContact(1);
    obj->setJoint(rai::JT_rigid);
    // obj->setColor({col(0), col(1), col(2)});

    while (true){
      const double x = rnd.uni(-0.8, 0.8);
      const double y = rnd.uni(-0.8, 0.8);

      const double alpha = rnd.uni(0, 2*3.1415);

      if (length(ARR(x, y, 0)) > 0.9 || length(ARR(x, y, 0)) < 0.3) {
        continue;
      }

      obj->setPosition({base_pos(0) + x, base_pos(1) + y, 0.61});
      obj->setRelativeQuaternion({cos(alpha/2), 0, 0, sin(alpha/2)});
      
      // check if something is in collision
      ConfigurationProblem cp(C);
      const auto res = cp.query({}, false);
      if (res->isFeasible){
                std::cout << "goal rot: " << alpha << std::endl;

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
      const double alpha = rnd.uni(0, 2*3.1415);

      // ensure that it is not too close to the base
      if (length(ARR(x, y, 0)) < 0.3 ||
          length(ARR(x, y, 0)) > 0.85){
        continue;
      }

      goal->setPosition({base_pos(0) + x, base_pos(1) + y, 0.61});
      goal->setRelativeQuaternion({cos(alpha/2), 0, 0, sin(alpha/2)});

      // goal->setColor({col(0), col(1), col(2), 0.5});
      
      // check if something is in collision
      ConfigurationProblem cp(C);
      const auto res = cp.query({}, false);
      if (res->isFeasible) {

        std::cout << "goal rot: " << alpha << std::endl;
        break;
      }
    }

    goal->setContact(0);

    // C.watch(true);

    const auto start_time = std::chrono::high_resolution_clock::now();

    const auto rtpm = compute_pick_and_place_positions(C, robots);

    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                              end_time - start_time)
                              .count();
    
    total_duration += duration;

    spdlog::info("Found {} of {} possible solutions", rtpm.size(), 1);

    if (rtpm.size() == 1){
      cnt_success += 1;
      report_test_result("Success", true);
    }
    else{
      report_test_result("Failure", false);
      std::cout << "goal: " << length(C["goal1"]->getPosition() - base_pos) << std::endl;
      std::cout << "obj " << length(C["obj1"]->getPosition() - base_pos) << std::endl;
      C.watch(true);
    }
  }

  std::cout << cnt_success << " out of " << N << " solutions found" << std::endl;
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

  for (uint i=0; i<N; ++i){
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

    while (true){
      const double x = rnd.uni(-0.8, 0.8);
      const double y = rnd.uni(-0.8, 0.8);

      const double alpha = rnd.uni(0, 2*3.1415);

      if (length(ARR(x, y, 0)) > 0.85 || length(ARR(x, y, 0)) < 0.3) {
        continue;
      }

      obj->setPosition({r1_base_pos(0) + x, r1_base_pos(1) + y, 0.61});
      obj->setRelativeQuaternion({cos(alpha/2), 0, 0, sin(alpha/2)});
      
      // check if something is in collision
      ConfigurationProblem cp(C);
      const auto res = cp.query({}, false);
      if (res->isFeasible){
                std::cout << "goal rot: " << alpha << std::endl;

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
      const double alpha = rnd.uni(0, 2*3.1415);

      // ensure that it is not too close to the base
      if (length(ARR(x, y, 0)) < 0.3 ||
          length(ARR(x, y, 0)) > 0.9){
        continue;
      }

      goal->setPosition({r2_base_pos(0) + x, r2_base_pos(1) + y, 0.61});
      goal->setRelativeQuaternion({cos(alpha/2), 0, 0, sin(alpha/2)});

      // goal->setColor({col(0), col(1), col(2), 0.5});
      
      // check if something is in collision
      ConfigurationProblem cp(C);
      const auto res = cp.query({}, false);
      if (res->isFeasible) {

        std::cout << "goal rot: " << alpha << std::endl;
        break;
      }
    }

    goal->setContact(0);

    // C.watch(true);

    const auto start_time = std::chrono::high_resolution_clock::now();

    const auto rtpm = compute_handover_poses(C, robots);

    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                              end_time - start_time)
                              .count();

    total_duration += duration;

    spdlog::info("Found {} of {} possible solutions", rtpm.size(), 1);

    if (rtpm.size() >= 1){
      cnt_success += 1;
      report_test_result("Success", true);
    }
    else{
      report_test_result("Failure", false);
      // std::cout << "goal: " << length(C["goal1"]->getPosition() - base_pos) << std::endl;
      // std::cout << "obj " << length(C["obj1"]->getPosition() - base_pos) << std::endl;
      // C.watch(true);
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

  std::cout << cnt_success << " out of " << N << " solutions found" << std::endl;
  std::cout << "Took " << total_duration / 1000 << "ms" << std::endl;

  spdlog::info("Found {} of {} solutions.", cnt_success, N);
}

void benchmark_dual_arm_pick_pick_success_rate(
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

  for (uint i=0; i<N; ++i){
    // add object to move
    double width = rnd.uni(0.03, 0.04);
    double depth = rnd.uni(0.08, 0.1);


    obj->setShape(rai::ST_box, {depth, width, 0.05, 0.01});
    obj->setContact(1);
    obj->setJoint(rai::JT_rigid);
    // obj->setColor({col(0), col(1), col(2)});

    while (true){
      const double x = rnd.uni(-0.8, 0.8);
      const double y = rnd.uni(-0.8, 0.8);

      const double alpha = rnd.uni(0, 2*3.1415);

      if (length(ARR(x, y, 0)) > 0.85 || length(ARR(x, y, 0)) < 0.3) {
        continue;
      }

      obj->setPosition({r1_base_pos(0) + x, r1_base_pos(1) + y, 0.61});
      obj->setRelativeQuaternion({cos(alpha/2), 0, 0, sin(alpha/2)});
      
      // check if something is in collision
      ConfigurationProblem cp(C);
      const auto res = cp.query({}, false);
      if (res->isFeasible){
                std::cout << "goal rot: " << alpha << std::endl;

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
      const double alpha = rnd.uni(0, 2*3.1415);

      // ensure that it is not too close to the base
      if (length(ARR(x, y, 0)) < 0.3 ||
          length(ARR(x, y, 0)) > 0.9){
        continue;
      }

      goal->setPosition({r2_base_pos(0) + x, r2_base_pos(1) + y, 0.61});
      goal->setRelativeQuaternion({cos(alpha/2), 0, 0, sin(alpha/2)});

      // goal->setColor({col(0), col(1), col(2), 0.5});
      
      // check if something is in collision
      ConfigurationProblem cp(C);
      const auto res = cp.query({}, false);
      if (res->isFeasible) {

        std::cout << "goal rot: " << alpha << std::endl;
        break;
      }
    }

    goal->setContact(0);

    // C.watch(true);

    const auto start_time = std::chrono::high_resolution_clock::now();

    const auto rtpm = compute_pick_and_place_with_intermediate_pose(C, robots);

    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end_time -
                                                              start_time)
            .count();

    total_duration += duration;

    spdlog::info("Found {} of {} possible solutions", rtpm.size(), 1);

    // ensure that the keyframe we found is the one we are looking for
    if (rtpm.size() >= 1){
      cnt_success += 1;
      report_test_result("Success", true);
    }
    else{
      report_test_result("Failure", false);
      // std::cout << "goal: " << length(C["goal1"]->getPosition() - base_pos) << std::endl;
      // std::cout << "obj " << length(C["obj1"]->getPosition() - base_pos) << std::endl;
      // C.watch(true);
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

  std::cout << cnt_success << " out of " << N << " solutions found" << std::endl;
  std::cout << "Took " << total_duration / 1000 << "ms" << std::endl;
  spdlog::info("Found {} of {} solutions.", cnt_success, N);
}