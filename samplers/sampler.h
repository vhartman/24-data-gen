#pragma once

#include "../plan.h"
#include "../planners/prioritized_planner.h"
#include "../util.h"
#include "util.h"
#include <Kin/featureSymbols.h>

// class RobotTaskPoseSampler{
//   public:
//     // this implements caching of results
//     TaskPoses sample(const RobotTaskPair &rtp){
//       if (cache.count(rtp) > 0){
//         return cache.at(rtp)[0];
//       }
//       else{
//         const auto res = generate(rtp);
//         cache[rtp].push_back(res);

//         return res;
//       }
//     };

//   private:
//     virtual TaskPoses generate(const RobotTaskPair &rtp) = 0;

//     // infeasible is represented as empty vector
//     std::unordered_map<RobotTaskPair, std::vector<TaskPoses>> cache;
// };

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

  // deleteUnnecessaryFrames(C);
  for (const auto f: C.frames){
    if (f->name.contains("goal")){
      f->setContact(1);
    }
  }

  delete_unnecessary_frames(C);

  for (const auto f: C.frames){
    if (f->name.contains("goal")){
      f->setContact(0);
    }
  }

  const auto pairs = get_cant_collide_pairs(C);
  C.fcl()->deactivatePairs(pairs);

  // C.watch(true);

  ConfigurationProblem cp(C);

  for (const Robot &r : robots) {
    setActive(C, r);
    
    for (uint i = 0; i < num_objects; ++i) {
      RobotTaskPair rtp;
      rtp.robots = {r};
      rtp.task = Task{.object=i, .type=TaskType::pick};

      OptOptions options;
      // options.stopIters = 100;
      // options.damping = 1e-3;

      KOMO komo;
      komo.verbose = 0;
      komo.setModel(C, true);

      komo.setDiscreteOpt(2);

      // komo.world.stepSwift();

      komo.add_collision(true, .00, 1e2);
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

      // komo.addObjective({1.}, FS_position, {STRING(prefix << "pen_tip")},
      // OT_eq,
      //                  {1e2}, point);
      //komo.addObjective({1., 1.}, FS_distance, {STRING(prefix << "pen_tip"), STRING(obj)}, OT_sos, {1e1});
      komo.addObjective({1., 2.}, FS_positionDiff,
                        {pen_tip, STRING(obj)}, OT_sos,
                        {1e1});

      komo.addObjective({1., 2.}, FS_insideBox, {pen_tip, STRING(obj)}, OT_ineq,
                        {1e1});


        komo.addObjective({1., 1.}, FS_scalarProductXY, {obj, pen_tip}, OT_eq,
                          {1e1}, {1.});

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
      setActive(cp.C, r.prefix);

      bool found_solution = false;
      for (uint j = 0; j < 10; ++j) {
        komo.run_prepare(0.0, true);
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
          std::cout << "pick/place failed for robot " << r << " and obj " << obj
                    << " ineq: " << ineq << " eq: " << eq << std::endl;
          std::cout << "\t pose 1 coll: " << res1->isFeasible << " pose 2 coll: " << res2->isFeasible
                    << std::endl;
          if (!res1->isFeasible){
            res1->writeDetails(std::cout, cp.C);}
          if (!res2->isFeasible){
            res2->writeDetails(std::cout, cp.C);}
        //   komo.pathConfig.watch(true);
        }
      }
    }
  }

  return rtpm;
}

RobotTaskPoseMap compute_pick_and_place_with_intermediate_pose(
    rai::Configuration C, const std::vector<std::string> &robots) {
  int num_objects = 0;
  for (auto f : C.frames) {
    if (f->name.contains("obj")) {
      num_objects += 1;
    }
  }

  // deleteUnnecessaryFrames(C);
  for (const auto f : C.frames) {
    if (f->name.contains("goal")) {
      f->setContact(1);
    }
  }

  delete_unnecessary_frames(C);

  for (const auto f : C.frames) {
    if (f->name.contains("goal")) {
      f->setContact(0);
    }
  }

  const auto pairs = get_cant_collide_pairs(C);
  C.fcl()->deactivatePairs(pairs);

  std::unordered_map<std::string, FrameL> robot_frames;
  for (const auto &r : robots) {
    robot_frames[r] = get_robot_joints(C, r);
  }

  OptOptions options;
  RobotTaskPoseMap rtpm;

  ConfigurationProblem cp(C);

  return rtpm;
}

RobotTaskPoseMap
compute_handover_poses(rai::Configuration C,
                       const std::vector<Robot> &robots) {
  int num_objects = 0;
  for (auto f : C.frames) {
    if (f->name.contains("obj")) {
      num_objects += 1;
    }
  }

  // deleteUnnecessaryFrames(C);
  for (const auto f : C.frames) {
    if (f->name.contains("goal")) {
      f->setContact(1);
    }
  }

  delete_unnecessary_frames(C);

  for (const auto f : C.frames) {
    if (f->name.contains("goal")) {
      f->setContact(0);
    }
  }

  const auto pairs = get_cant_collide_pairs(C);
  C.fcl()->deactivatePairs(pairs);

  std::unordered_map<Robot, FrameL> robot_frames;
  for (const auto &r: robots){
    robot_frames[r] = get_robot_joints(C, r);
  }

  // C.watch(true);
  OptOptions options;
  RobotTaskPoseMap rtpm;

  ConfigurationProblem cp(C);
  for (const auto &r1 : robots) {
    for (const auto &r2 : robots) {

      if (r1 == r2){continue;}

      setActive(C, std::vector<Robot>{r1, r2});
      setActive(cp.C, std::vector<Robot>{r1, r2});

      cp.limits = cp.C.getLimits();

      for (uint i = 0; i < num_objects; ++i) {
        std::cout << "computing handover for " << r1 << " " << r2 << " and " << i << std::endl;
        RobotTaskPair rtp;
        rtp.robots = {r1, r2};
        rtp.task = Task{.object=i, .type=TaskType::handover};

        KOMO komo;
        komo.verbose = 0;
        komo.setModel(C, true);
        // komo.animateOptimization = 5;

        komo.setDiscreteOpt(3);

        komo.add_collision(true, .00, 1e2);
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

        komo.addObjective({1., 1.}, FS_scalarProductXY, {obj, r1_pen_tip}, OT_eq,
                          {1e1}, {1.});

        komo.addObjective({2., 2.}, FS_scalarProductXY, {obj, r2_pen_tip}, OT_eq,
                          {1e1}, {1.});

        // homing
        if (true) {
          for (const auto base_name : {r1.prefix, r2.prefix}) {
            uintA bodies;
            rai::Joint *j;
            for (rai::Frame *f : komo.world.frames)
              if ((j = f->joint) && j->qDim() > 0 &&
                  (f->name.contains(base_name.c_str()))){
                bodies.append(f->ID);}
            komo.addObjective({0, 3}, make_shared<F_qItself>(bodies, true), {},
                              OT_sos, {1e-1}, NoArr); // world.q, prec);
          }
        }

        bool found_solution = false;
        for (uint j = 0; j < 3; ++j) {
          komo.run_prepare(0.0, false);
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

          const auto res1 = cp.query(q0);
          const auto res2 = cp.query(q1);
          const auto res3 = cp.query(q2);

          cp.C.setJointState(initial_pose);

          const double ineq = komo.getReport(false).get<double>("ineq");
          const double eq = komo.getReport(false).get<double>("eq");

          if (res1->isFeasible && res2->isFeasible && res3->isFeasible && ineq < 1. && eq < 1.) {
            C.setJointState(q0);
            const arr pick_pose = C.getJointState(robot_frames[r1]);

            C.setJointState(q2);
            const arr place_pose = C.getJointState(robot_frames[r2]);

            rtpm[rtp].push_back({pick_pose, q1, place_pose});
            // komo.pathConfig.watch(true);

            found_solution = true;

            break;
          } else {
            std::cout << "handover failed for robot " << r1 << " " << r2 << " and obj " << obj << " with ineq " << ineq << " eq " << eq 
                      << std::endl;

            std::cout << "\t pose 1 coll: " << res1->isFeasible
                      << " pose 2 coll: " << res2->isFeasible
                      << " pose 3 coll: " << res3->isFeasible << std::endl;
            if (!res1->isFeasible) {
              res1->writeDetails(std::cout, cp.C);
            }
            if (!res2->isFeasible) {
              res2->writeDetails(std::cout, cp.C);
            }
            if (!res3->isFeasible) {
              res3->writeDetails(std::cout, cp.C);
            }
            // komo.pathConfig.watch(true);
          }
        }
      }
    }
  }

  return rtpm;
}