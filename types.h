#pragma once
#include <string>
#include <vector>

enum class TaskType { pick, handover, go_to, joint_pick };
struct Task {
  unsigned int object;
  TaskType type;
};

enum class RobotType { ur5, kuka, pandas };
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

// typedef std::string Robot; // prefix of the robot -> fully identifies the
// robot

struct RobotTaskPair {
  std::vector<Robot> robots;
  Task task;

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