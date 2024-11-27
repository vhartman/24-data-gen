#pragma once

#include <string>
#include <vector>
#include <Core/array.h>

enum class PrimitiveType { pick, handover, go_to, joint_pick, pick_pick_1, pick_pick_2};

// enum class ActionType { none, pick, place, handover, go_to };
// class ActionSequence {
//   // TODO: this doe currently not work, since we need a dependency graph to
//   // represent the action sequence of multiple robots in one primitve
//   std::vector<std::vector<ActionType>> sequence;
//   std::vector<Robot> robots;
//   std::vector<rai::String>
//       frames; // the involved frames (usually the object name)
// };

// std::vector<std::vector<Action>> primitive_to_sequence() {
//   return {{Action::pick, Action::place}};
//   return {{Action::pick, Action::handover}, {Action::none, Action::handover, Action::place}};
//   // return {{Action::pick, Action::handover}, {Action::handover, Action::place}};
// }

std::string primitive_type_to_string(PrimitiveType t){
  if (t == PrimitiveType::go_to){
    return "go_to";
  } else if (t == PrimitiveType::handover) {
    return "handover";
  } else if (t == PrimitiveType::pick) {
    return "pick";
  } else if (t == PrimitiveType::pick_pick_1) {
    return "pickpick1";
  } else if (t == PrimitiveType::pick_pick_2) {
    return "pickpick2";
  }

  return "TYPE NOT DEFINED";
}

PrimitiveType string_to_primitive_type(const std::string &str){
  if (str == "handover") {
    return PrimitiveType::handover;
  } else if (str == "pick") {
    return PrimitiveType::pick;
  } else if (str == "pickpick1") {
    return PrimitiveType::pick_pick_1;
  } else if (str == "pickpick2") {
    return PrimitiveType::pick_pick_2;
  } else {
    spdlog::error("No task specified.");
  }

  // how to handle this? no idea.
  return PrimitiveType::handover;
}

struct Task {
  unsigned int object;
  PrimitiveType type;
};

enum class RobotType { ur5, kuka, panda };

// we return a number that is higher than the indicated one to stay on the conservative side with our
// early stopping.
double get_workspace_from_robot_type(const RobotType t){
if (t == RobotType::kuka){
    return 1.; // from datasheet: 800mm
  }
  else if (t == RobotType::ur5){
    return 1.; // from datasheet: 850mm
  }
  else if (t == RobotType::panda){
    return 1.; // from datasheet: 855mm
  }

  return 0.;
}

std::string robot_type_to_string(RobotType t){
  if (t == RobotType::kuka){
    return "kuka";
  }
  else if (t == RobotType::ur5){
    return "ur5";
  }
  else if (t == RobotType::panda){
    return "panda";
  }

  return "NOT SPECIFIED ROBOT TYPE";
}

std::string get_base_joint_name(const RobotType rt){
  if (rt == RobotType::ur5) {
    return "shoulder_pan_joint";
  } else if (rt == RobotType::panda) {
    return "panda_joint1";
  }
  // kuka
  return "iiwa_joint_1";
}

enum class EndEffectorType { two_finger, vacuum, pen };

std::string ee_type_to_string(EndEffectorType t){
  if (t == EndEffectorType::two_finger){
    return "two_finger_gripper";
  }
  else if (t == EndEffectorType::vacuum){
    return "vacuum";
  }
  else if (t == EndEffectorType::pen){
    return "pen";
  }

  return "UNSPECIFIED EE";
}

class Robot {
public:
  Robot(){};
  Robot(const std::string &_prefix, const RobotType _type = RobotType::ur5, const double _vmax=0.05)
      : prefix(_prefix), type(_type), vmax(_vmax) {}

  bool operator==(const Robot &o) const {
    return prefix == o.prefix && type == o.type;
  }

  bool operator!=(const Robot &o) const { return !(*this == o); }

  std::string prefix;
  RobotType type;
  EndEffectorType ee_type;

  arr home_pose;
  double vmax = 0.05;
};

std::ostream &operator<<(std::ostream &os, Robot const &r) { 
    return os << r.prefix;
}

template <> struct std::hash<Robot> {
  std::size_t operator()(Robot const &r) const {
    std::hash<std::string> hasher;
    return hasher(r.prefix);
  }
};

struct RobotTaskPair {
  std::vector<Robot> robots;
  Task task;

  std::string serialize() const {
    std::stringstream ss;
    ss << "robots: ";
    for (const auto &r : robots) {
      ss << r << ", ";
    }
    ss << "; obj: " << task.object
       << ";  primitive: " << primitive_type_to_string(task.type);
    return ss.str();
  }

  bool operator==(const RobotTaskPair &other) const {
    if (robots.size() != other.robots.size()) {
      return false;
    }
    if (task.type != other.task.type || task.object != other.task.object) {
      return false;
    }

    for (uint i = 0; i < robots.size(); ++i) {
      if (robots[i] != other.robots[i]) {
        return false;
      }
    }

    return true;
  }
};

template <> struct std::hash<RobotTaskPair> {
  std::size_t operator()(RobotTaskPair const &rtp) const {
    std::size_t seed = rtp.robots.size();
    std::hash<Robot> hasher;
    for (auto &i : rtp.robots) {
      seed ^= hasher(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    seed ^= std::size_t(rtp.task.type) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= rtp.task.object + 0x9e3779b9 + (seed << 6) + (seed >> 2);

    return seed;
  }
};