#pragma once

#include "spdlog/spdlog.h"

#include <Kin/featureSymbols.h>

#include "../plan.h"
#include "../planners/prioritized_planner.h"
#include "../util.h"

RobotTaskPoseMap
compute_handover_poses(rai::Configuration C,
                       const std::vector<Robot> &robots) {
  int num_objects = 0;
  for (auto f : C.frames) {
    if (f->name.contains("obj")) {
      num_objects += 1;
    }
  }

  delete_unnecessary_frames(C);
  const auto pairs = get_cant_collide_pairs(C);
  C.fcl()->deactivatePairs(pairs);

  std::unordered_map<Robot, FrameL> robot_frames;
  for (const auto &r: robots){
    robot_frames[r] = get_robot_joints(C, r);
  }

  // C.watch(true);
  OptOptions options;
  options.allowOverstep = true;

  RobotTaskPoseMap rtpm;

  ConfigurationProblem cp(C);
  for (const auto &r1 : robots) {
    for (const auto &r2 : robots) {

      if (r1 == r2){continue;}

      setActive(C, std::vector<Robot>{r1, r2});
      setActive(cp.C, std::vector<Robot>{r1, r2});

      cp.limits = cp.C.getLimits();

      for (uint i = 0; i < num_objects; ++i) {
        spdlog::info("computing handover for {0}, {1}, obj {2}", r1.prefix,
                     r2.prefix, i + 1);
                     
        RobotTaskPair rtp;
        rtp.robots = {r1, r2};
        rtp.task = Task{.object=i, .type=TaskType::handover};

        KOMO komo;
        komo.verbose = 0;
        komo.setModel(C, true);
        // komo.animateOptimization = 5;

        komo.setDiscreteOpt(3);

        komo.add_collision(true, .1, 1e1);
        komo.add_jointLimits(true, 0., 1e1);

        const auto r1_pen_tip = STRING(r1 << "pen_tip");
        const auto r2_pen_tip = STRING(r2 << "pen_tip");

        const auto obj = STRING("obj" << i + 1);
        const auto goal = STRING("goal" << i + 1);

        Skeleton S = {
            // {1., 2., SY_touch, {r1_pen_tip, obj}},
            {1., 2, SY_stable, {r1_pen_tip, obj}},
            // {2., -1., SY_touch, {r2_pen_tip, obj}},
            {2., 3., SY_stable, {r2_pen_tip, obj}},
            {3., -1, SY_poseEq, {obj, goal}},
            // {3., -1, SY_positionEq, {obj, goal}}
            
        };

        komo.setSkeleton(S);

        const double offset = 0.1;
        komo.addObjective({2., 2.}, FS_distance, {"table", obj}, OT_ineq, {1e0},
                          {-offset});

        // komo.addObjective({1., 1.}, FS_positionDiff, {r1_pen_tip, STRING(obj)},
        //                   OT_sos, {1e1});

        // komo.addObjective({2., 2.}, FS_positionDiff, {r2_pen_tip, STRING(obj)},
        //                   OT_sos, {1e1});

        komo.addObjective({1., 2.}, FS_insideBox, {r1_pen_tip, STRING(obj)},
                          OT_ineq, {1e1});
        komo.addObjective({2., 3.}, FS_insideBox, {r2_pen_tip, STRING(obj)},
                          OT_ineq, {1e1});

        // const double margin = 0.1;
        // komo.addObjective({1., 1.}, FS_positionDiff, {r1_pen_tip, STRING(obj)},
        //                   OT_ineq, {-1e1}, {-margin, -margin, -margin});

        // komo.addObjective({1., 1.}, FS_positionDiff, {r1_pen_tip, STRING(obj)},
        //             OT_ineq, {1e1}, {margin, margin, margin});

        // komo.addObjective({2., 2.}, FS_positionDiff, {r2_pen_tip, STRING(obj)},
        //                   OT_ineq, {-1e1}, {-margin, -margin, -margin});

        // komo.addObjective({2., 2.}, FS_positionDiff, {r2_pen_tip, STRING(obj)},
        //                   OT_ineq, {1e1}, {margin, margin, margin});

        komo.addObjective({1., 1.}, FS_scalarProductZZ, {obj, r1_pen_tip},
                          OT_sos, {1e0}, {-1.});

        komo.addObjective({2., 2.}, FS_scalarProductZZ, {obj, r2_pen_tip},
                          OT_sos, {1e0}, {-1.});

        // komo.addObjective({1.}, FS_scalarProductYX, {obj, r1_pen_tip},
        //                   OT_sos, {1e0}, {1.});

        // komo.addObjective({2.}, FS_scalarProductYX, {obj, r2_pen_tip},
        //                   OT_sos, {1e0}, {1.});

        komo.addObjective({1., 2.}, FS_scalarProductXY, {obj, r1_pen_tip}, OT_eq,
                          {1e0}, {1.});

        komo.addObjective({2., 3.}, FS_scalarProductXY, {obj, r2_pen_tip}, OT_eq,
                          {1e0}, {1.});

        // homing
        if (true) {
          for (const auto &base_name : {r1.prefix, r2.prefix}) {
            uintA bodies;
            rai::Joint *j;
            for (rai::Frame *f : komo.world.frames) {
              if ((j = f->joint) && j->qDim() > 0 &&
                  (f->name.contains(base_name.c_str()))) {
                bodies.append(f->ID);
              }
            }
            komo.addObjective({0, 3}, make_shared<F_qItself>(bodies, true), {},
                              OT_sos, {1e-1}, NoArr); // world.q, prec);
          }
        }

        bool found_solution = false;
        for (uint j = 0; j < 3; ++j) {
          if (j==0){
            komo.run_prepare(0.0, false);
          }
          else{
            komo.run_prepare(0.00001, false);
          }
          komo.run(options);

          const arr q0 = komo.getPath()[0]();
          const arr q1 = komo.getPath()[1]();
          const arr q2 = komo.getPath()[2]();

        //   komo.pathConfig.watch(true);

          // ensure via sampling as well
          // std::cout << q0 << std::endl;
          // std::cout << q1 << std::endl;
          // std::cout << q2 << std::endl;

          const auto initial_pose = cp.C.getJointState();

          ConfigurationProblem cp2(C);

          const auto res1 = cp2.query(q0);

          {
            // link object to other robot
            auto from = cp2.C[r1_pen_tip];
            auto to = cp2.C[obj];
            to->unLink();
            // create a new joint
            to->linkFrom(from, true);
          }

          // cp.C.watch(true);
          cp2.C.calc_indexedActiveJoints();

          const auto res2 = cp2.query(q1);
          // cp2.C.watch(true);

          {
            auto from = cp2.C[r2_pen_tip];
            auto to = cp2.C[obj];

            to->unLink();

            // create a new joint
            to->linkFrom(from, true);
          }
          cp2.C.calc_indexedActiveJoints();

          const auto res3 = cp2.query(q2);
          // cp2.C.watch(true);

          cp.C.setJointState(initial_pose);

          const double ineq = komo.getReport(false).get<double>("ineq");
          const double eq = komo.getReport(false).get<double>("eq");

          if (res1->isFeasible && res2->isFeasible && res3->isFeasible && ineq < 1. && eq < 1.) {
            const auto home = C.getJointState();

            C.setJointState(q0);
            const arr pick_pose = C.getJointState(robot_frames[r1]);

            C.setJointState(q2);
            const arr place_pose = C.getJointState(robot_frames[r2]);

            rtpm[rtp].push_back({pick_pose, q1, place_pose});
            // komo.pathConfig.watch(true);

            found_solution = true;

            C.setJointState(home);

            break;
          } else {
            spdlog::debug(
                "pick/place failed for robot {} and {}, obj {} ineq: "
                "{:03.2f} eq: {:03.2f}",
                r1.prefix, r2.prefix, obj, ineq, eq);
            spdlog::debug("Collisions: pose 1 coll: {0}, pose 2 coll: {1}, pose "
                         "3 coll: {2}",
                         res1->isFeasible, res2->isFeasible, res3->isFeasible);

            if (!res1->isFeasible) {
              std::stringstream ss;
              res1->writeDetails(ss, cp.C);
              spdlog::debug(ss.str());
            }
            if (!res2->isFeasible) {
              std::stringstream ss;
              res2->writeDetails(ss, cp.C);
              spdlog::debug(ss.str());
            }
            if (!res3->isFeasible) {
              std::stringstream ss;
              res3->writeDetails(ss, cp.C);
              spdlog::debug(ss.str());
            }
            // komo.pathConfig.watch(true);
          }
        }

        if (!found_solution){
          spdlog::info("Could not find a solution.");
        }
      }
    }
  }

  return rtpm;
}