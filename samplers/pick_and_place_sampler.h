#pragma once

#include "spdlog/spdlog.h"

#include <Kin/featureSymbols.h>

#include "common/util.h"
#include "planners/plan.h"
#include "planners/prioritized_planner.h"

#include "samplers/pick_constraints.h"

class PickAndPlaceSampler {
public:
  rai::Configuration C;
  PickAndPlaceSampler(const rai::Configuration &_C) : C(_C) {
    delete_unnecessary_frames(C);
    const auto pairs = get_cant_collide_pairs(C);
    C.fcl()->deactivatePairs(pairs);

    // options.allowOverstep = true;
    options.nonStrictSteps = 50;
    options.damping = 10;

    // options.stopIters = 500;
    options.wolfe = 0.01;
    // options.maxStep = 0.5;

    // options.stopIters = 100;
    // options.damping = 1e-3;
  }

  OptOptions options;

  // TaskPoses sample_at_times(std::vector<Robot> robots, std::string obj,
  //                           rai::Animation A) {
  //   // TODO: set things to state.
  //   return sample(robots, obj);
  // }

  void add_place_contraint_and_objectives() {}

  // enum class PickDirection {PosX, NegX, PosY, NegY, PosZ, NegZ};

  TaskPoses sample(const Robot r, const rai::String obj, const rai::String goal,
                   const PickDirection pick_direction = PickDirection::NegZ,
                   const bool sample_only_place = false) {
    spdlog::info("Attempting to compute keyframes for robot {} and object {}",
                 r.prefix, obj.p);

    const arr obj_pos = C[obj]->getPosition();
    const arr goal_pos = C[goal]->getPosition();

    const arr r1_pos = C[STRING(r << "base")]->getPosition();

    setActive(C, r);

    // TODO: solve subproblems to check for feasibility.
    // For now: hardcode the radius of the ur5
    if (euclideanDistance(obj_pos, r1_pos) > 1. ||
        euclideanDistance(goal_pos, r1_pos) > 1.) {
      spdlog::info("Skipping pick keyframe copmutation for obj {} and "
                   "robot {}",
                   obj, r.prefix);
      return {};
    }

    KOMO komo;
    komo.verbose = 0;
    komo.setModel(C, true);
    // komo.pathConfig.fcl()->deactivatePairs(pairs);

    uint total_phases = 2;
    if (sample_only_place) {
      total_phases = 1;
    }

    komo.setDiscreteOpt(total_phases);
    // komo.animateOptimization = 3;

    // komo.world.stepSwift();

    komo.add_collision(true, 0.1, 1e1);
    komo.add_jointLimits(true, 0., 1e1);

    const auto pen_tip = STRING(r.prefix << "pen_tip");

    const double r1_z_rot =
        C[STRING(r << "base")]->get_X().rot.getEulerRPY()(2);

    const double r1_obj_angle =
        std::atan2(obj_pos(1) - r1_pos(1), obj_pos(0) - r1_pos(0)) - r1_z_rot;
    const double r1_goal_angle =
        std::atan2(goal_pos(1) - r1_pos(1), goal_pos(0) - r1_pos(0)) - r1_z_rot;

    double pick_phase = 1;
    double place_phase = 2;

    if (sample_only_place) {
      place_phase = 1;
    }

    Skeleton S;
    if (!sample_only_place)
      S.append({pick_phase, place_phase, SY_stable, {pen_tip, obj}});
    S.append({place_phase, place_phase, SY_poseEq, {obj, goal}});
    //   {1., 1., SY_touch, {pen_tip, obj}},
    // {2., -1, SY_positionEq, {obj, goal}},

    komo.setSkeleton(S);

    if (!sample_only_place) {
      add_pick_constraints(komo, pick_phase, pick_phase, pen_tip, r.ee_type,
                           obj, pick_direction, C[obj]->shape->size);
    }

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

    const uint max_attempts = 10;
    for (uint j = 0; j < max_attempts; ++j) {
      // reset komo to initial state
      komo.pathConfig.setJointState(inital_state);
      komo.reset();

      komo.run_prepare(0.0001, false);

      // set orientation to the direction of the object and the goal
      // respectively

      uint r1_cnt = 0;
      const std::string base_joint_name = get_base_joint_name(r.type);

      for (const auto aj : komo.pathConfig.activeJoints) {
        const uint ind = aj->qIndex;
        if (aj->frame->name.contains(base_joint_name.c_str()) &&
            aj->frame->name.contains(r.prefix.c_str())) {
          // komo.x(ind) = cnt + j;
          if (r1_cnt == 0) {
            // compute orientation for robot to face towards box
            komo.x(ind) = r1_obj_angle + rnd.uni(-1, 1) * j / max_attempts;
          }
          if (r1_cnt == 1) {
            // compute orientation for robot to face towards other robot
            komo.x(ind) = r1_goal_angle + rnd.uni(-1, 1) * j / max_attempts;
          }
          ++r1_cnt;
        }
      }

      komo.pathConfig.setJointState(komo.x);

      uintA objID;
      objID.append(C[obj]->ID);
      rai::Frame *obj1 =
          komo.pathConfig.getFrames(komo.pathConfig.frames.d1 * 1 + objID)(0);
      obj1->setPose(C[obj]->getPose());

      rai::Frame *obj2 =
          komo.pathConfig.getFrames(komo.pathConfig.frames.d1 * 2 + objID)(0);
      obj2->setPose(C[goal]->getPose());

      komo.run_prepare(0.);
      // std::cout << "init confg" << std::endl;
      // komo.pathConfig.watch(true);
      komo.run(options);

      const arr q0 = komo.getPath()[0]();
      const arr q1 = komo.getPath()[1]();
      // komo.pathConfig.watch(true);

      // ensure via sampling as well
      ConfigurationProblem cp2(C);
      const auto res1 = cp2.query(q0);
      {
        // link object to robot
        auto from = cp2.C[pen_tip];
        auto to = cp2.C[obj];
        to->unLink();
        // create a new joint
        to->linkFrom(from, true);
      }
      cp2.C.calc_indexedActiveJoints();

      const auto res2 = cp2.query(q1);

      const double ineq = komo.getReport(false).get<double>("ineq");
      const double eq = komo.getReport(false).get<double>("eq");

      if (res1->isFeasible && res2->isFeasible && ineq < 1. && eq < 1.) {
        return {q0, q1};

        // std::cout << q0 << std::endl;
        // std::cout << q1 << std::endl;

        // komo.pathConfig.watch(true);
      } else {
        spdlog::info("pick/place failed for robot {} and {} ineq: {:03.2f} "
                     "eq: {:03.2f}",
                     r.prefix, obj, ineq, eq);
        spdlog::info("Collisions: pose 1 coll: {0} pose 2 coll: {1}",
                     res1->isFeasible, res2->isFeasible);

        if (!res1->isFeasible) {
          std::stringstream ss;
          res1->writeDetails(ss, cp2.C);
          spdlog::debug(ss.str());
        }
        if (!res2->isFeasible) {
          std::stringstream ss;
          res2->writeDetails(ss, cp2.C);
          spdlog::debug(ss.str());
        }
        // komo.pathConfig.watch(true);
      }
    }

    std::vector<arr> infeasible;
    return infeasible;
  }
};

RobotTaskPoseMap compute_all_pick_and_place_positions(
    rai::Configuration C, const std::vector<Robot> &robots,
    const bool attempt_all_directions = false) {
  RobotTaskPoseMap rtpm;

  uint num_objects = 0;
  for (auto f : C.frames) {
    if (f->name.contains("obj")) {
      num_objects += 1;
    }
  }

  delete_unnecessary_frames(C);

  std::vector<PickDirection> directions = {PickDirection::NegZ};
  if (attempt_all_directions) {
    directions = {PickDirection::NegZ, PickDirection::NegX,
                  PickDirection::NegY, PickDirection::PosZ,
                  PickDirection::PosX, PickDirection::PosY};
  }

  const auto pairs = get_cant_collide_pairs(C);
  C.fcl()->deactivatePairs(pairs);

  PickAndPlaceSampler sampler(C);

  for (const Robot &r : robots) {
    setActive(C, r);

    for (uint i = 0; i < num_objects; ++i) {
      const auto obj = STRING("obj" << i + 1);
      const auto goal = STRING("goal" << i + 1);

      for (const auto dir: directions){
        const auto sol = sampler.sample(r, obj, goal, dir);

        if (sol.size() > 0) {
          RobotTaskPair rtp;
          rtp.robots = {r};
          rtp.task = Task{.object = i, .type = PrimitiveType::pick};
          rtpm[rtp].push_back({sol[0], sol[1]});
          break;
        } else {
          spdlog::info("Did not find a solution");
        }
      }
    }
  }

  return rtpm;
}