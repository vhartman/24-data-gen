#pragma once

#include "spdlog/spdlog.h"

#include <Kin/featureSymbols.h>

#include "common/util.h"
#include "planners/plan.h"
#include "planners/prioritized_planner.h"

bool solve_problem_without_collision() {}

class RepeatedPickSampler {
public:
  RepeatedPickSampler(rai::Configuration &_C) : C(_C) {
    delete_unnecessary_frames(C);
    const auto pairs = get_cant_collide_pairs(C);
    C.fcl()->deactivatePairs(pairs);

    // options.allowOverstep = true;
    options.nonStrictSteps = 50;
    options.damping = 10;
    options.wolfe = 0.01;
  };

  rai::Configuration C;
  OptOptions options;

  // TODO: copied from pick_and_place sampler
  // TODO: should likely make this shared
  enum class PickDirection {PosX, NegX, PosY, NegY, PosZ, NegZ};

  void setup_problem(KOMO &komo, const Robot &r1, const Robot &r2,
                     const rai::String &obj, const rai::String &goal) {

    const auto r1_pen_tip = STRING(r1 << "pen_tip");
    const auto r2_pen_tip = STRING(r2 << "pen_tip");

    const auto link_to_frame = STRING("table");

    Skeleton S = {
        {1., 2., SY_touch, {r1_pen_tip, obj}},
        {1., 2, SY_stable, {r1_pen_tip, obj}},

        // {2., 3., SY_touch, {obj, "table"}},
        {2., 3., SY_stable, {link_to_frame, obj}},

        {3., 4., SY_stable, {r2_pen_tip, obj}},

        {4., -1, SY_poseEq, {obj, goal}},
        // {3., -1, SY_positionEq, {obj, goal}}
    };

    komo.setSkeleton(S);

    // constraints for placing the object
    komo.addObjective({2., 3.}, FS_distance, {link_to_frame, obj}, OT_ineq,
                      {-1e1}, {-0.04});
    komo.addObjective({2., 3.}, FS_distance, {link_to_frame, obj}, OT_ineq,
                      {1e1}, {0.05});

    // constraints for picking the object
    komo.addObjective({1., 2.}, FS_insideBox, {r1_pen_tip, obj}, OT_ineq,
                      {5e1});
    komo.addObjective({3., 4.}, FS_insideBox, {r2_pen_tip, obj}, OT_ineq,
                      {5e1});

    // komo.addObjective({3., 3.}, FS_positionDiff, {STRING(r1<<"base"),
    // STRING(obj)},
    //                   OT_sos, {1e0});
    // komo.addObjective({3., 3.}, FS_positionDiff,
    //                   {STRING(r2 << "base"), STRING(obj)}, OT_sos, {1e0});

    // komo.addObjective({2., 2.}, FS_positionDiff, {r2_pen_tip, STRING(obj)},
    //                   OT_sos, {1e1});

    // const double margin = 0.1;
    // komo.addObjective({1., 1.}, FS_positionDiff, {r1_pen_tip, STRING(obj)},
    //                   OT_ineq, {-1e1}, {-margin, -margin, -margin});

    // komo.addObjective({1., 1.}, FS_positionDiff, {r1_pen_tip, STRING(obj)},
    //             OT_ineq, {1e1}, {margin, margin, margin});

    // komo.addObjective({2., 2.}, FS_positionDiff, {r2_pen_tip, STRING(obj)},
    //                   OT_ineq, {-1e1}, {-margin, -margin, -margin});

    // komo.addObjective({2., 2.}, FS_positionDiff, {r2_pen_tip, STRING(obj)},
    //                   OT_ineq, {1e1}, {margin, margin, margin});

    komo.addObjective({1., 2.}, FS_scalarProductZZ, {obj, r1_pen_tip}, OT_sos,
                      {1e1}, {-1.});

    komo.addObjective({3., 4.}, FS_scalarProductZZ, {obj, r2_pen_tip}, OT_sos,
                      {1e1}, {-1.});

    komo.addObjective({2., 2.}, FS_scalarProductZZ, {obj, link_to_frame}, OT_eq,
                      {1e1}, {1.});

    // komo.addObjective({1.}, FS_scalarProductYX, {obj, r1_pen_tip},
    //                   OT_sos, {1e0}, {1.});

    // komo.addObjective({2.}, FS_scalarProductYX, {obj, r2_pen_tip},
    //                   OT_sos, {1e0}, {1.});

    if (C[obj]->shape->size(0) > C[obj]->shape->size(1)) {
      // x longer than y
      spdlog::info("Trying to grab along x-axis");
      if (r1.ee_type == EndEffectorType::two_finger) {
        komo.addObjective({1., 2.}, FS_scalarProductXY, {obj, r1_pen_tip},
                          OT_eq, {1e1}, {0.});
      }

      if (r2.ee_type == EndEffectorType::two_finger) {
        komo.addObjective({3., 4.}, FS_scalarProductXY, {obj, r2_pen_tip},
                          OT_eq, {1e1}, {0.});
      }
    } else {
      spdlog::info("Trying to grab along y-axis");
      if (r1.ee_type == EndEffectorType::two_finger) {
        komo.addObjective({1., 2.}, FS_scalarProductXX, {obj, r1_pen_tip},
                          OT_eq, {1e1}, {0.});
      }

      if (r2.ee_type == EndEffectorType::two_finger) {
        komo.addObjective({3., 4.}, FS_scalarProductXX, {obj, r2_pen_tip},
                          OT_eq, {1e1}, {0.});
      }
    }

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
                          OT_sos, {1e1}, NoArr); // world.q, prec);
      }
    }
  }

  std::vector<arr> sample(const Robot &r1, const Robot &r2,
                          const rai::String &obj, const rai::String &goal, const PickDirection pd1=PickDirection::NegZ, const PickDirection intermediate_direction=PickDirection::NegZ, const PickDirection pd2=PickDirection::NegZ) {
    setActive(C, std::vector<Robot>{r1, r2});
    std::unordered_map<Robot, FrameL> robot_frames;
    for (const auto &r : {r1, r2}) {
      robot_frames[r] = get_robot_joints(C, r);
    }

    ConfigurationProblem cp(C);

    const arr obj_pos = C[obj]->getPosition();
    const arr goal_pos = C[goal]->getPosition();

    const arr r1_pos = C[STRING(r1 << "base")]->getPosition();
    const arr r2_pos = C[STRING(r2 << "base")]->getPosition();

    // For now: hardcode the radius of the ur5
    const double ws_1 = get_workspace_from_robot_type(r1.type);
    const double ws_2 = get_workspace_from_robot_type(r2.type);

    if (euclideanDistance(obj_pos, r1_pos) > ws_1 ||
        euclideanDistance(goal_pos, r2_pos) > ws_2 ||
        euclideanDistance(r1_pos, r2_pos) > ws_1 + ws_2) {
      spdlog::info("Skipping pickpick keyframe computation for obj {} and "
                   "robots {}, {} due to worspace limits",
                   obj.p, r1.prefix, r2.prefix);
      return {};
    }

    // TODO: solve subproblems to check for feasibility.
    // {
    //   KOMO komo;
    //   komo.verbose = 0;
    //   komo.setModel(C, true);
    //   // komo.animateOptimization = 5;

    //   komo.setDiscreteOpt(4);

    //   setup_problem(komo, r1, r2, obj, goal);
    //   komo.run_prepare(0.00001, false);
    //   komo.run(options);

    //   // komo.pathConfig.watch(true);

    //   const double ineq = komo.getReport(false).get<double>("ineq");
    //   const double eq = komo.getReport(false).get<double>("eq");

    //   if (ineq > 1 || eq > 1) {
    //     // std::cout << ineq << " " << eq << std::endl;
    //     spdlog::info("Skipping pickpick keyframe copmutation for obj {} and "
    //                  "robots {}, {} since subproblem is infeasible",
    //                  obj.p, r1.prefix, r2.prefix);
    //     // komo.pathConfig.watch(true);
    //     return {};
    //   }
    // }

    KOMO komo;
    komo.verbose = 0;
    komo.setModel(C, true);
    // komo.animateOptimization = 5;

    komo.setDiscreteOpt(4);

    komo.add_collision(true, .05, 1e1);
    komo.add_jointLimits(true, 0., 1e1);

    // add constraints and costs for alignment
    setup_problem(komo, r1, r2, obj, goal);

    const auto r1_pen_tip = STRING(r1 << "pen_tip");
    const auto r2_pen_tip = STRING(r2 << "pen_tip");

    const auto link_to_frame = STRING("table");

    const double r1_z_rot =
        C[STRING(r1 << "base")]->get_Q().rot.getEulerRPY()(2);
    const double r2_z_rot =
        C[STRING(r2 << "base")]->get_Q().rot.getEulerRPY()(2);

    const double r1_obj_angle =
        std::atan2(obj_pos(1) - r1_pos(1), obj_pos(0) - r1_pos(0)) - r1_z_rot;
    const double r1_r2_angle =
        std::atan2(r2_pos(1) - r1_pos(1), r2_pos(0) - r1_pos(0)) - r1_z_rot;
    const double r2_r1_angle =
        std::atan2(r1_pos(1) - r2_pos(1), r1_pos(0) - r2_pos(0)) - r2_z_rot;
    const double r2_goal_angle =
        std::atan2(goal_pos(1) - r2_pos(1), goal_pos(0) - r2_pos(0)) - r2_z_rot;

    const uint max_attempts = 5;
    for (uint j = 0; j < max_attempts; ++j) {
      komo.run_prepare(0.00001, false);

      const std::string r1_base_joint_name = get_base_joint_name(r1.type);
      const std::string r2_base_joint_name = get_base_joint_name(r2.type);

      uint r1_cnt = 0;
      uint r2_cnt = 0;
      for (const auto aj : komo.pathConfig.activeJoints) {
        const uint ind = aj->qIndex;
        if (aj->frame->name.contains(r1_base_joint_name.c_str()) &&
            aj->frame->name.contains(r1.prefix.c_str())) {
          // komo.x(ind) = cnt + j;
          if (r1_cnt == 0) {
            // compute orientation for robot to face towards box
            komo.x(ind) = r1_obj_angle + (rnd.uni(-1, 1) * j) / max_attempts;
          }
          if (r1_cnt == 1) {
            // compute orientation for robot to face towards other robot
            komo.x(ind) = r1_r2_angle + (rnd.uni(-1, 1) * j) / max_attempts;
          }
          ++r1_cnt;
        }

        if (aj->frame->name.contains(r2_base_joint_name.c_str()) &&
            aj->frame->name.contains(r2.prefix.c_str())) {
          // komo.x(ind) = cnt + j;
          if (r2_cnt == 2) {
            // compute orientation for robot to face towards box
            komo.x(ind) = r2_r1_angle + (rnd.uni(-1, 1) * j) / max_attempts;
          }
          if (r2_cnt == 3) {
            // compute orientation for robot to face towards other robot
            komo.x(ind) = r2_goal_angle + (rnd.uni(-1, 1) * j) / max_attempts;
          }
          ++r2_cnt;
        }
      }

      komo.pathConfig.setJointState(komo.x);
      for (const auto f : komo.pathConfig.frames) {
        if (f->name == obj) {
          f->setPose(C[obj]->getPose());
        }
      }

      uintA objID;
      objID.append(C[obj]->ID);
      // rai::Frame *obj2 =
      //     komo.pathConfig.getFrames(komo.pathConfig.frames.d1 * 3 +
      //     objID)(0);
      // arr rndpos(2);
      // rndUniform(rndpos, -1, 1);
      // obj2->setRelativePosition({rndpos(0), rndpos(1),
      // C[obj]->getRelativePosition()(2)});

      rai::Frame *obj3 =
          komo.pathConfig.getFrames(komo.pathConfig.frames.d1 * 4 + objID)(0);
      obj3->setPose(C[goal]->getPose());

      komo.run_prepare(0.0, false);

      komo.run(options);

      // komo.pathConfig.watch(true);

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

      if (res1->isFeasible && res2->isFeasible && res3->isFeasible &&
          res4->isFeasible && ineq < 1. && eq < 1.) {
        // komo.pathConfig.watch(true);

        const auto home = C.getJointState();

        C.setJointState(q0);
        const arr pick_pose = C.getJointState(robot_frames[r1]);

        C.setJointState(q1);
        const arr place_pose = C.getJointState(robot_frames[r1]);

        C.setJointState(q2);
        const arr pick_2_pose = C.getJointState(robot_frames[r2]);

        C.setJointState(q3);
        const arr place_2_pose = C.getJointState(robot_frames[r2]);

        C.setJointState(home);

        return {pick_pose, place_pose, pick_2_pose, place_2_pose};
      }
    }

    return {};
  }
};

std::vector<arr>
compute_pick_and_place_with_intermediate_pose(rai::Configuration C, Robot r1,
                                              Robot r2, rai::String obj,
                                              rai::String goal) {
  RepeatedPickSampler sampler(C);
  return sampler.sample(r1, r2, obj, goal);
}

RobotTaskPoseMap compute_all_pick_and_place_with_intermediate_pose(
    rai::Configuration C, const std::vector<Robot> &robots) {
  uint num_objects = 0;
  for (auto f : C.frames) {
    if (f->name.contains("obj")) {
      num_objects += 1;
    }
  }

  delete_unnecessary_frames(C);
  const auto pairs = get_cant_collide_pairs(C);
  C.fcl()->deactivatePairs(pairs);

  RobotTaskPoseMap rtpm;

  RepeatedPickSampler sampler(C);

  for (const auto &r1 : robots) {
    for (const auto &r2 : robots) {
      if (r1 == r2) {
        continue;
      }

      for (uint i = 0; i < num_objects; ++i) {
        const auto obj = STRING("obj" << i + 1);
        const auto goal = STRING("goal" << i + 1);
        const auto sol = sampler.sample(r1, r2, obj, goal);

        if (sol.size() > 0) {
          RobotTaskPair rtp_1;
          rtp_1.robots = {r1, r2};
          rtp_1.task = Task{.object = i, .type = PrimitiveType::pick_pick_1};
          rtpm[rtp_1].push_back({sol[0], sol[1]});

          RobotTaskPair rtp_2;
          rtp_2.robots = {r1, r2};
          rtp_2.task = Task{.object = i, .type = PrimitiveType::pick_pick_2};
          rtpm[rtp_2].push_back({sol[2], sol[3]});
        }
      }
    }
  }

  return rtpm;
}