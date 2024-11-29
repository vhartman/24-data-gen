#pragma once

#include "spdlog/spdlog.h"

#include <KOMO/komo.h>
#include <Kin/featureSymbols.h>

#include "../common/types.h"

// TODO: copied from pick_and_place sampler
// TODO: should likely make this shared
enum class PickDirection { PosX, NegX, PosY, NegY, PosZ, NegZ };

std::string to_string(PickDirection direction) {
  switch (direction) {
  case PickDirection::PosX:
    return "PosX";
  case PickDirection::NegX:
    return "NegX";
  case PickDirection::PosY:
    return "PosY";
  case PickDirection::NegY:
    return "NegY";
  case PickDirection::PosZ:
    return "PosZ";
  case PickDirection::NegZ:
    return "NegZ";
  default:
    throw std::invalid_argument("Unknown enum value");
  }
}

arr dir_to_vec(const PickDirection dir) {
  switch (dir) {
  case PickDirection::PosX:
    return {1., 0., 0.};
  case PickDirection::NegX:
    return {-1., 0., 0.};
  case PickDirection::PosY:
    return {0., 1., 0.};
  case PickDirection::NegY:
    return {0., -1., 0.};
  case PickDirection::PosZ:
    return {0.0, 0.0, 1.0};
  case PickDirection::NegZ:
    return {0.0, 0.0, -1.0};
  default:
    throw std::invalid_argument("Unknown enum value");
  }
}

// We add alignment constraints between the dir-axis of the object, and the
// z-axis of the robot
void add_pick_constraints(KOMO &komo, const double start, const int end,
                          const rai::String &ee, const EndEffectorType ee_type,
                          const rai::String &obj, const PickDirection dir,
                          const arr obj_size) {
  komo.addObjective({start, start}, FS_positionDiff, {ee, obj}, OT_sos, {1e0});

  komo.addObjective({start, start}, FS_insideBox, {ee, obj}, OT_ineq, {5e1});

  if (dir == PickDirection::NegZ) {
    komo.addObjective({start, start}, FS_scalarProductZZ, {obj, ee}, OT_sos,
                      {1e1}, {-1.});
  } else if (dir == PickDirection::PosZ) {
    komo.addObjective({start, start}, FS_scalarProductZZ, {obj, ee}, OT_sos,
                      {1e1}, {1.});
  } else if (dir == PickDirection::NegX) {
    komo.addObjective({start, start}, FS_scalarProductXZ, {obj, ee}, OT_sos,
                      {1e1}, {-1.});
  } else if (dir == PickDirection::PosX) {
    komo.addObjective({start, start}, FS_scalarProductXZ, {obj, ee}, OT_sos,
                      {1e1}, {1.});
  } else if (dir == PickDirection::NegY) {
    komo.addObjective({start, start}, FS_scalarProductYZ, {obj, ee}, OT_sos,
                      {1e1}, {-1.});
  } else if (dir == PickDirection::PosY) {
    komo.addObjective({start, start}, FS_scalarProductYZ, {obj, ee}, OT_sos,
                      {1e1}, {1.});
  }

  // only add the 'alignment' constaint if the end effector is a
  // two-finger-gripper
  if (ee_type == EndEffectorType::two_finger) {
    if (obj_size(0) > obj_size(1)) {
      // x longer than y
      spdlog::info("Trying to grab along x-axis");
      komo.addObjective({start, start}, FS_scalarProductXY, {obj, ee}, OT_eq,
                        {1e1}, {0.});
    } else {
      spdlog::info("Trying to grab along y-axis");
      komo.addObjective({start, start}, FS_scalarProductXX, {obj, ee}, OT_eq,
                        {1e1}, {0.});
    }
  }
}