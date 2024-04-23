#pragma once

#include "spdlog/spdlog.h"

#include <Kin/featureSymbols.h>

#include "planners/plan.h"
#include "planners/prioritized_planner.h"
#include "common/util.h"

class GoToSampler {
public:
  GoToSampler(rai::Configuration &_C) : C(_C) {
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

  std::vector<arr> sample(const Robot r, const rai::String goal) {
    setActive(C, std::vector<Robot>{r});
    std::unordered_map<Robot, FrameL> robot_frames;
    for (const auto &r : {r}) {
      robot_frames[r] = get_robot_joints(C, r);
    }

    ConfigurationProblem cp(C);

    const arr goal_pos = C[goal]->getPosition();
    const arr r1_pos = C[STRING(r << "base")]->getPosition();

    // TODO: solve subproblems to check for feasibility.
    // For now: hardcode the radius of the ur5
    if (euclideanDistance(goal_pos, r1_pos) > 1.) {
      spdlog::info("Skipping goto keyframe copmutation for obj {} and "
                   "robots {}",
                   goal.p, r.prefix);
      return {};
    }

    KOMO komo;
    komo.verbose = 0;
    komo.setModel(C, true);
    // komo.animateOptimization = 5;

    komo.setDiscreteOpt(1);

    komo.add_collision(true, .05, 1e1);
    komo.add_jointLimits(true, 0., 1e1);

    const auto r1_pen_tip = STRING(r << "pen_tip");

    const double r1_z_rot =
        C[STRING(r << "base")]->get_Q().rot.getEulerRPY()(2);
    const double r1_goal_angle =
        std::atan2(goal_pos(1) - r1_pos(1), goal_pos(0) - r1_pos(0)) - r1_z_rot;

    Skeleton S = {{1., 1., SY_touch, {r1_pen_tip, goal}}};

    komo.setSkeleton(S);

    // homing
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
        komo.addObjective({0, 1}, make_shared<F_qItself>(bodies, true), {},
                          OT_sos, {1e-1}, NoArr); // world.q, prec);
      }
    }

    // initialization
    komo.run_prepare(0.0, false);
    const auto inital_state = komo.pathConfig.getJointState();
    const uint max_attempts = 5;
    for (uint j = 0; j < max_attempts; ++j) {
      komo.pathConfig.setJointState(inital_state);
      komo.run_prepare(0.00001, false);

      uint r1_cnt = 0;
      for (const auto aj : komo.pathConfig.activeJoints) {
        const uint ind = aj->qIndex;
        if (aj->frame->name.contains("shoulder_pan_joint") &&
            aj->frame->name.contains(r.prefix.c_str())) {
          // komo.x(ind) = cnt + j;
          if (r1_cnt == 0) {
            // compute orientation for robot to face towards box
            komo.x(ind) = r1_goal_angle + (rnd.uni(-1, 1) * j) / max_attempts;
          }
          ++r1_cnt;
        }
      }

      komo.run_prepare(0.0, false);

      komo.run(options);

      // komo.pathConfig.watch(true);

      const arr q0 = komo.getPath()[0]();

      //   komo.pathConfig.watch(true);

      const auto initial_pose = cp.C.getJointState();

      const auto res1 = cp.query(q0);
      // const auto res5 = cp.query(q4);

      cp.C.setJointState(initial_pose);

      const double ineq = komo.getReport(false).get<double>("ineq");
      const double eq = komo.getReport(false).get<double>("eq");

      if (res1->isFeasible && ineq < 1. && eq < 1.) {
        return {q0};
      }
    }

    return {};
  };
};