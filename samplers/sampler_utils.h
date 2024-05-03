#pragma once

#include "spdlog/spdlog.h"

#include <Core/util.h>
#include <KOMO/komo.h>

#include "common/types.h"

// enum class GraspType {top_grasp, front_grasp, side_grasp};

void add_grasping_constraint(KOMO &komo, const double start, const double end,
                             const EndEffectorType ee_type,
                             const rai::String ee_frame_name,
                             const rai::String obj_frame_name) {
  // add box-touching
  komo.addObjective({start, end}, FS_insideBox, {ee_frame_name, obj_frame_name},
                    OT_ineq, {5e1});

  // add orientation -> top-grasp always at the moment
  komo.addObjective({start, start}, FS_scalarProductZZ,
                    {obj_frame_name, ee_frame_name}, OT_sos, {1e1}, {-1.});

  // add alignment
  if (ee_type == EndEffectorType::two_finger) {
    FeatureSymbol fs;
    if (komo.world[obj_frame_name]->shape->size(0) >
        komo.world[obj_frame_name]->shape->size(1)) {
      // x longer than y
      spdlog::info("Trying to grab along x-axis");
      fs = FS_scalarProductXY;

    } else {
      spdlog::info("Trying to grab along y-axis");
      fs = FS_scalarProductXX;
    }

    komo.addObjective({start, end}, fs, {obj_frame_name, ee_frame_name}, OT_eq,
                      {1e1}, {0.});
  }
}

void add_homing_constraint() {}