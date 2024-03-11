#pragma once

#include "spdlog/spdlog.h"

#include <Kin/featureSymbols.h>

#include "../plan.h"
#include "../planners/prioritized_planner.h"
#include "../util.h"

#include "pick_and_place_sampler.h"
#include "handover_sampler.h"

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


RobotTaskPoseMap compute_pick_and_place_with_intermediate_pose(
    rai::Configuration C, const std::vector<std::string> &robots) {
  int num_objects = 0;
  for (auto f : C.frames) {
    if (f->name.contains("obj")) {
      num_objects += 1;
    }
  }

  delete_unnecessary_frames(C);
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