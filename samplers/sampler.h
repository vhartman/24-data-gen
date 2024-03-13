#pragma once

#include "spdlog/spdlog.h"

#include <Kin/featureSymbols.h>

#include "../plan.h"
#include "../planners/prioritized_planner.h"
#include "../util.h"

#include "pick_and_place_sampler.h"
#include "handover_sampler.h"

class RobotTaskPoseSampler{
  public:
    // this implements caching of results
    TaskPoses sample(const rai::Configuration &C, const RobotTaskPair &rtp){
      if (cache.count(rtp) > 0){
        return cache.at(rtp)[0];
      }
      else{
        const auto res = generate(C, rtp);
        cache[rtp].push_back(res);

        return res;
      }
    };

  private:
    virtual TaskPoses generate(const rai::Configuration &C, const RobotTaskPair &rtp) = 0;

    // infeasible is represented as empty vector
    std::unordered_map<RobotTaskPair, std::vector<TaskPoses>> cache;
};

// class PickAndPlaceSampler: public RobotTaskPoseSampler{
//   public:
//     rai::Configuration C;
//     PickAndPlaceSampler(rai::Configuration &_C) : C(_C) {
//       int num_objects = 0;
//       for (auto f : C.frames) {
//         if (f->name.contains("obj")) {
//           num_objects += 1;
//         }
//       }

//       // deleteUnnecessaryFrames(C);
//       for (const auto f : C.frames) {
//         if (f->name.contains("goal")) {
//           f->setContact(1);
//         }
//       }

//       deleteUnnecessaryFrames(C);

//       for (const auto f : C.frames) {
//         if (f->name.contains("goal")) {
//           f->setContact(0);
//         }
//       }

//       const auto pairs = get_cant_collide_pairs(C);
//       C.fcl()->deactivatePairs(pairs);
//     };

//   private:
//     TaskPoses generate(const RobotTaskPair &rtp) {
//       if (rtp.task.type == TaskType::pick) {
//         return generate_pick_place(rtp);
//       } else if (rtp.task.type == TaskType::handover) {
//         return generate_handover_place(rtp);
//       }
//     };

//     TaskPoses generate_pick_place(const RobotTaskPair &rtp);
//     TaskPoses generate_handover_place(const RobotTaskPair &rtp);
//     // TaskPoses generate_handover();
// };


RobotTaskPoseMap compute_pick_and_place_with_intermediate_pose(
    rai::Configuration C, const std::vector<Robot> &robots) {
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
  for (const auto &r : robots) {
    robot_frames[r] = get_robot_joints(C, r);
  }

  OptOptions options;
  options.allowOverstep = true;
  options.maxStep = 5;

  RobotTaskPoseMap rtpm;

  ConfigurationProblem cp(C);
  for (const auto &r1: robots){
    for (const auto &r2: robots){
      if (r1 == r2){continue;}

      setActive(C, std::vector<Robot>{r1, r2});
      setActive(cp.C, std::vector<Robot>{r1, r2});

      cp.limits = cp.C.getLimits();

      for (uint i = 0; i < num_objects; ++i) {
        spdlog::info("computing pick and repick for {0}, {1}, obj {2}", r1.prefix,
                     r2.prefix, i + 1);

        KOMO komo;
        komo.verbose = 0;
        komo.setModel(C, true);
        // komo.animateOptimization = 5;

        komo.setDiscreteOpt(4);

        komo.add_collision(true, .00, 1e1);
        komo.add_jointLimits(true, 0., 1e1);

        const auto r1_pen_tip = STRING(r1 << "pen_tip");
        const auto r2_pen_tip = STRING(r2 << "pen_tip");

        const auto obj = STRING("obj" << i + 1);
        const auto goal = STRING("goal" << i + 1);

        Skeleton S = {
            {1., 2., SY_touch, {r1_pen_tip, obj}},
            {1., 2, SY_stable, {r1_pen_tip, obj}},
            // {2., 3., SY_stable, {r2_pen_tip, obj}},

            // {2., 3., SY_touch, {obj, "table"}},
            {2., 3., SY_stable, {"table", obj}},

            {3., 4., SY_stable, {r2_pen_tip, obj}},
            
            {4., -1, SY_poseEq, {obj, goal}},
            // {3., -1, SY_positionEq, {obj, goal}}
            
        };

        komo.setSkeleton(S);

        const double offset = 0.1;

        komo.addObjective({2., 3.}, FS_distance, {"table", obj}, OT_ineq, {-1e0},
                          {-0.01});
        komo.addObjective({2., 3.}, FS_distance, {"table", obj}, OT_ineq, {1e0},
                          {0.5});
 
        // komo.addObjective({2., 2.}, FS_distance, {"table", obj}, OT_ineq, {1e0},
        //                   {-offset});

        // komo.addObjective({1., 1.}, FS_positionDiff, {r1_pen_tip, STRING(obj)},
        //                   OT_sos, {1e1});

        // komo.addObjective({2., 2.}, FS_positionDiff, {r2_pen_tip, STRING(obj)},
        //                   OT_sos, {1e1});

        komo.addObjective({1., 2.}, FS_insideBox, {r1_pen_tip, STRING(obj)},
                          OT_ineq, {1e1});
        komo.addObjective({3., 4.}, FS_insideBox, {r2_pen_tip, STRING(obj)},
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

        komo.addObjective({3., 3.}, FS_scalarProductZZ, {obj, r2_pen_tip},
                          OT_sos, {1e0}, {-1.});

        komo.addObjective({2., 2.}, FS_scalarProductZZ, {obj, "table"},
                          OT_eq, {1e0}, {1.});

        // komo.addObjective({1.}, FS_scalarProductYX, {obj, r1_pen_tip},
        //                   OT_sos, {1e0}, {1.});

        // komo.addObjective({2.}, FS_scalarProductYX, {obj, r2_pen_tip},
        //                   OT_sos, {1e0}, {1.});

        komo.addObjective({1., 2.}, FS_scalarProductXY, {obj, r1_pen_tip}, OT_eq,
                          {1e1}, {1.});

        komo.addObjective({3., 4.}, FS_scalarProductXY, {obj, r2_pen_tip}, OT_eq,
                          {1e1}, {1.});

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
            komo.addObjective({0, 5}, make_shared<F_qItself>(bodies, true), {},
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

          komo.pathConfig.watch(true);

          const arr q0 = komo.getPath()[0]();
          const arr q1 = komo.getPath()[1]();
          const arr q2 = komo.getPath()[2]();
          const arr q3 = komo.getPath()[3]();
          // const arr q4 = komo.getPath()[4]();

        //   komo.pathConfig.watch(true);

          // ensure via sampling as well
          // std::cout << q0 << std::endl;
          // std::cout << q1 << std::endl;
          // std::cout << q2 << std::endl;

          const auto initial_pose = cp.C.getJointState();

          const auto res1 = cp.query(q0);
          const auto res2 = cp.query(q1);
          const auto res3 = cp.query(q2);
          const auto res4 = cp.query(q3);
          // const auto res5 = cp.query(q4);

          cp.C.setJointState(initial_pose);

          const double ineq = komo.getReport(false).get<double>("ineq");
          const double eq = komo.getReport(false).get<double>("eq");

          if (res1->isFeasible && res2->isFeasible && res3->isFeasible && res4->isFeasible && ineq < 1. && eq < 1.) {
            const auto home = C.getJointState();

            C.setJointState(q0);
            const arr pick_pose = C.getJointState(robot_frames[r1]);

            C.setJointState(q1);
            const arr place_pose = C.getJointState(robot_frames[r1]);

            C.setJointState(q2);
            const arr pick_2_pose = C.getJointState(robot_frames[r2]);

            C.setJointState(q3);
            const arr place_2_pose = C.getJointState(robot_frames[r2]);
            
            RobotTaskPair rtp_1;
            rtp_1.robots = {r1, r2};
            rtp_1.task = Task{.object = i, .type = TaskType::pick_pick_1};
            rtpm[rtp_1].push_back(
                {pick_pose, place_pose});

            RobotTaskPair rtp_2;
            rtp_2.robots = {r1, r2};
            rtp_2.task = Task{.object = i, .type = TaskType::pick_pick_2};
            rtpm[rtp_2].push_back(
                {pick_2_pose, place_2_pose});


            found_solution = true;

            C.setJointState(home);

            break;
          } else {
            spdlog::info("not found sol yet");
          }
        }
      }
    }
  }

  return rtpm;
}