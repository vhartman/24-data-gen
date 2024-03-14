#pragma once

#include "spdlog/spdlog.h"

#include <Kin/featureSymbols.h>

#include "../plan.h"
#include "../planners/prioritized_planner.h"
#include "../util.h"

class PickAndPlaceSampler {
public:
  rai::Configuration C;
  PickAndPlaceSampler(const rai::Configuration &_C) : C(_C) {
    delete_unnecessary_frames(C);
    const auto pairs = get_cant_collide_pairs(C);
    C.fcl()->deactivatePairs(pairs);
  }

  TaskPoses sample_at_times(std::vector<Robot> robots, std::string obj, rai::Animation A){
    // TODO: set things to state.
    return sample(robots, obj);
  }

  TaskPoses sample(std::vector<Robot> robots, std::string obj){
    KOMO komo;
    komo.setModel(C, true);
    komo.setDiscreteOpt(2);

    std::vector<arr> infeasible;
    return infeasible;
  }
};

RobotTaskPoseMap
compute_pick_and_place_positions(rai::Configuration C,
                                 const std::vector<Robot> &robots) {
  RobotTaskPoseMap rtpm;

  int num_objects = 0;
  for (auto f: C.frames){
    if (f->name.contains("obj")){
      num_objects += 1;
    }
  }

  delete_unnecessary_frames(C);

  const auto pairs = get_cant_collide_pairs(C);
  C.fcl()->deactivatePairs(pairs);

  // C.watch(true);

  ConfigurationProblem cp(C);
  OptOptions options;
  // options.stopIters = 100;
  // options.damping = 1e-3;

  options.allowOverstep = true;
  options.maxStep = 5;

  for (const Robot &r : robots) {
    setActive(C, r);
    setActive(cp.C, r.prefix);
    
    for (uint i = 0; i < num_objects; ++i) {
      // C.watch(true);
      spdlog::info("Attempting to compute keyframes for robot {} and object {}", r.prefix, i);
      RobotTaskPair rtp;
      rtp.robots = {r};
      rtp.task = Task{.object=i, .type=TaskType::pick};

      KOMO komo;
      komo.verbose = 0;
      komo.setModel(C, true);
      komo.world.fcl()->deactivatePairs(pairs);

      komo.setDiscreteOpt(2);
      // komo.animateOptimization = 2;

      // komo.world.stepSwift();

      komo.add_collision(true, 0.1, 1e1);
      komo.add_jointLimits(true, 0., 1e1);

      const auto pen_tip = STRING(r.prefix << "pen_tip");
      const auto obj = STRING("obj" << i + 1);
      const auto goal = STRING("goal" << i + 1);

      Skeleton S = {
        //   {1., 1., SY_touch, {pen_tip, obj}},
          {1., -1, SY_stable, {pen_tip, obj}},
          {2., -1, SY_poseEq, {obj, goal}},
          // {2., -1, SY_positionEq, {obj, goal}},
      };

      komo.setSkeleton(S);

    //   komo.addObjective({1.}, FS_position, {STRING(prefix << "pen_tip")}, OT_eq,
    //                     {1e2}, point);
    //   komo.addObjective({1., 1.}, FS_distance,
    //                     {STRING(prefix << "pen_tip"), STRING(obj)}, OT_sos,
    //                     {1e1});
      // komo.addObjective({1., 1.}, FS_positionDiff, {pen_tip, STRING(obj)},
      //                   OT_sos, {1e-1});

      komo.addObjective({1., 1.}, FS_insideBox, {pen_tip, STRING(obj)}, OT_ineq,
                        {1e1});

      komo.addObjective({1., 1.}, FS_scalarProductXY, {obj, pen_tip}, OT_eq,
                        {1e1}, {1.});

      komo.addObjective({1., 1.}, FS_scalarProductZZ, {obj, pen_tip}, OT_sos,
                        {1e0}, {-1.});

      //   const double margin = 0.05;
      //   komo.addObjective({1., 1.}, FS_positionDiff, {pen_tip, STRING(obj)},
      //                     OT_ineq, {-1e1}, {-margin, -margin, -margin});

      //   komo.addObjective({1., 1.}, FS_positionDiff, {pen_tip, STRING(obj)},
      //                     OT_ineq, {1e1}, {margin, margin, margin});

    //   komo.addObjective({1.}, FS_vectorZ, {pen_tip},
    //                     OT_sos, {1e0}, {0., 0., -1.});

      // komo.addObjective({1.}, FS_position, {STRING(prefix << "pen_tip")},
      // OT_sos, {1e0}, C[obj]->getPosition());

      // komo.addObjective({1.}, FS_vectorZ, {STRING(prefix << "pen")}, OT_sos,
      // {1e1}, {0., 0., -1.}); komo.addObjective({1.}, FS_vectorZDiff,
      // {STRING(prefix << "pen"), "world"}, OT_ineq, {1e1}, {0., 0., -0.9});

      if (true) {
          for (const auto &base_name : {r.prefix}) {
            uintA bodies;
            rai::Joint *j;
            for (rai::Frame *f : komo.world.frames) {
              if ((j = f->joint) && j->qDim() > 0 &&
                  (f->name.contains(base_name.c_str()))) {
                bodies.append(f->ID);
              }
            }
            komo.addObjective({0, 3}, make_shared<F_qItself>(bodies, true), {},
                              OT_sos, {1e0}, NoArr); // world.q, prec);
          }
        }

      komo.run_prepare(0.0, false);
      const auto inital_state = komo.pathConfig.getJointState();

      bool found_solution = false;
      for (uint j = 0; j < 10; ++j) {
        // reset komo to initial state
        komo.pathConfig.setJointState(inital_state);
        komo.reset();

        if (j == 0) {
          komo.run_prepare(0.0, false);
        } else {
          komo.run_prepare(0.0001, false);
        }
        komo.run(options);

        const arr q0 = komo.getPath()[0]();
        const arr q1 = komo.getPath()[1]();
        // komo.pathConfig.watch(true);

        // ensure via sampling as well
        const auto initial_state = cp.C.getJointState();

        const auto res1 = cp.query(q0);
        const auto res2 = cp.query(q1);

        cp.C.setJointState(initial_state);

        const double ineq = komo.getReport(false).get<double>("ineq");
        const double eq = komo.getReport(false).get<double>("eq");

        if (res1->isFeasible && res2->isFeasible && ineq < 1. &&
            eq < 1.) {
          rtpm[rtp].push_back({q0, q1});

          // komo.pathConfig.watch(true);

          found_solution = true;

          break;
        } else {
          spdlog::info("pick/place failed for robot {} and {} ineq: {:03.2f} eq: {:03.2f}", r.prefix, obj, ineq, eq);
          spdlog::info("Collisions: pose 1 coll: {0} pose 2 coll: {1}", res1->isFeasible, res2->isFeasible);

          if (!res1->isFeasible) {
            std::stringstream ss;
            res1->writeDetails(ss, cp.C);
            spdlog::debug(ss.str());
          }
          if (!res2->isFeasible) {
            std::stringstream ss;
            res2->writeDetails(std::cout, cp.C);
            spdlog::debug(ss.str());
          }
          // komo.pathConfig.watch(true);
        }
      }

      if (!found_solution){
        spdlog::info("Did not find a solution");
      }
    }
  }

  return rtpm;
}