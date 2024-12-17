#pragma once

#include "spdlog/spdlog.h"

#include <KOMO/komo.h>
#include <Kin/featureSymbols.h>

#include "../common/types.h"

// TODO: copied from pick_and_place sampler
// TODO: should likely make this shared
enum class PickDirection { PosX, NegX, PosY, NegY, PosZ, NegZ };

constexpr PickDirection operator!(PickDirection dir) {
  switch (dir) {
  case PickDirection::PosX:
    return PickDirection::NegX;
  case PickDirection::NegX:
    return PickDirection::PosX;
  case PickDirection::PosY:
    return PickDirection::NegY;
  case PickDirection::NegY:
    return PickDirection::PosY;
  case PickDirection::PosZ:
    return PickDirection::NegZ;
  case PickDirection::NegZ:
    return PickDirection::PosZ;
  }
  // Optional: Add a default return or error handling
  throw std::invalid_argument("Invalid PickDirection");
}

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

arr get_pos_z_axis_dir(const arr &quaternion) {
  // Extract components of the quaternion
  double w = quaternion(0);
  double x = quaternion(1);
  double y = quaternion(2);
  double z = quaternion(3);

  // Normalize the quaternion to avoid numerical inaccuracies
  double norm = std::sqrt(w * w + x * x + y * y + z * z);
  w /= norm;
  x /= norm;
  y /= norm;
  z /= norm;

  // Compute the Z-axis in world coordinates
  double zX = 2.0 * (x * z + w * y);
  double zY = 2.0 * (y * z - w * x);
  double zZ = 1.0 - 2.0 * (x * x + y * y);

  return {zX, zY, zZ};
}

// We add alignment constraints between the dir-axis of the object, and the
// z-axis of the robot
void add_pick_constraints(KOMO &komo, const double start, const int end,
                          const rai::String &ee, const EndEffectorType ee_type,
                          const rai::String &obj, const PickDirection dir,
                          const arr obj_size) {
  komo.addObjective({start, start}, FS_positionDiff, {ee, obj}, OT_sos, {1e0});

  komo.addObjective({start, start}, FS_insideBox, {ee, obj}, OT_ineq, {5e1});

  spdlog::info("Grasping from " + to_string(dir) + " direction");

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
  const double max_grabbable_width = 0.1;
  if (ee_type == EndEffectorType::two_finger) {
    // if we grasp from one of the z-directions, we ty to align the y-axis of
    // the gripper with the x (or y) axis of the object
    if (dir == PickDirection::NegZ || dir == PickDirection::PosZ) {
      // obj_size(0) is x, (1) is y, (2) is z
      if (obj_size(0) < obj_size(1)) {
        // x shorter than y
        spdlog::info("Trying to grab along y-axis");
        komo.addObjective({start, start}, FS_scalarProductYY, {obj, ee}, OT_eq,
                          {1e1}, {0.});

        if (obj_size(1) > max_grabbable_width){
          spdlog::warn("Unlikely to be able to grab with these constraints, since the obejct is too big.");
        }
      } else {
        spdlog::info("Trying to grab along x-axis");
        komo.addObjective({start, start}, FS_scalarProductXY, {obj, ee}, OT_eq,
                          {1e1}, {0.});
        if (obj_size(0) > max_grabbable_width){
          spdlog::warn("Unlikely to be able to grab with these constraints, since the obejct is too big.");
        }
      }
    }
    // if we grasp from the y direction, we want to align with the x or the z
    // direction
    else if (dir == PickDirection::NegY || dir == PickDirection::PosY) {
      if (obj_size(0) < obj_size(2)) {
        // x shorter than z
        spdlog::info("Trying to grab along z-axis");
        // should be ZY, is not defined, XX is equivalent
        komo.addObjective({start, start}, FS_scalarProductXX, {obj, ee}, OT_eq,
                          {1e1}, {0.});
        if (obj_size(2) > max_grabbable_width){
          spdlog::warn("Unlikely to be able to grab with these constraints, since the obejct is too big.");
        }
      } else {
        spdlog::info("Trying to grab along x-axis");
        // should be FS_scalarProduct ZY, but that is undefined. this here is equivalent
        komo.addObjective({start, start}, FS_scalarProductXY, {obj, ee}, OT_eq,
                          {1e1}, {0.});
        if (obj_size(0) > max_grabbable_width){
          spdlog::warn("Unlikely to be able to grab with these constraints, since the obejct is too big.");
        }
      }
    }
    else if (dir == PickDirection::NegX || dir == PickDirection::PosX) {
      if (obj_size(1) < obj_size(2)) {
        // y shorter than z
        spdlog::info("Trying to grab along z-axis");
        komo.addObjective({start, start}, FS_scalarProductYX, {obj, ee}, OT_eq,
                          {1e1}, {0.});
        if (obj_size(2) > max_grabbable_width){
          spdlog::warn("Unlikely to be able to grab with these constraints, since the obejct is too big.");
        }
      } else {
        spdlog::info("Trying to grab along z-axis");
        // should be FS_scalarProduct ZX, but that is undefined. this here is equivalent
        komo.addObjective({start, start}, FS_scalarProductYY, {obj, ee}, OT_eq,
                          {1e1}, {0.});

        if (obj_size(1) > max_grabbable_width){
          spdlog::warn("Unlikely to be able to grab with these constraints, since the obejct is too big.");
        }
      }
    }
  }
}