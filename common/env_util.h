#pragma once

#include "json/json.h"
#include <Kin/kin.h>

#include "types.h"
#include <PlanningSubroutines/ConfigurationProblem.h>

using json = nlohmann::ordered_json;

void setActive(rai::Configuration &C,
               const std::vector<std::string> &prefixes) {
  FrameL roots;
  for (const auto &prefix : prefixes) {
    roots.append(C.getFrame(STRING(prefix << "base")));
  }
  C.selectJointsBySubtrees(roots);
}

void setActive(rai::Configuration &C, const std::string &prefix) {
  setActive(C, std::vector<std::string>{prefix});
}

void setActive(rai::Configuration &C, const Robot &r) {
  setActive(C, r.prefix);
}

void setActive(rai::Configuration &C, const std::vector<Robot> &robots) {
  std::vector<std::string> robot_prefixes;
  for (auto r : robots) {
    robot_prefixes.push_back(r.prefix);
  }
  setActive(C, robot_prefixes);
}

void setRobotJointState() {}

std::vector<Robot> make_robot_environment_from_json(
    rai::Configuration &C, json jf,
    const std::string &base_scene_path = "./in/scenes/floor.g") {
  if (base_scene_path.size() == 0) {
    C.addFile("./in/scenes/floor.g");
  } else {
    C.addFile(base_scene_path.c_str());
  }

  uint cnt = 0;
  std::stringstream ss;

  std::vector<Robot> robots;

  for (const auto &item : jf["robots"].items()) {
    ss.clear();
    ss << item.value()["base_pos"];
    arr base_pos;
    base_pos.read(ss);

    ss.clear();
    arr base_quat;
    ss << item.value()["base_quat"];
    base_quat.read(ss);

    std::string parent;
    if (item.value().contains("parent")) {
      parent = item.value()["parent"];
    } else {
      parent = "table";
    }

    std::string robot = item.value()["type"];
    rai::Frame *a;
    EndEffectorType ee;
    RobotType robot_type;
    if (robot == "ur5_gripper") {
      a = C.addFile("./in/robots/ur5.g");
      ee = EndEffectorType::two_finger;
      robot_type = RobotType::ur5;
    } else if (robot == "ur5_vacuum") {
      a = C.addFile("./in/robots/ur5_vacuum.g");
      ee = EndEffectorType::vacuum;
      robot_type = RobotType::ur5;} 
    else if (robot == "inflated_ur5_vacuum") {
      a = C.addFile("./in/robots/inflated_ur5_vacuum.g");
      ee = EndEffectorType::vacuum;
      robot_type = RobotType::ur5;
    } else if (robot == "ur5_vacuum") {
      a = C.addFile("./in/robots/ur5_pen.g");
      ee = EndEffectorType::vacuum;
      robot_type = RobotType::ur5;
    } else if (robot == "franka") {
      a = C.addFile("./in/robots/franka.g");
      ee = EndEffectorType::two_finger;
      robot_type = RobotType::panda;
    } else if (robot == "kuka") {
      a = C.addFile("./in/robots/kuka.g");
      ee = EndEffectorType::two_finger;
      robot_type = RobotType::kuka;
    } else {
      spdlog::error("Invalid robot type");
      return {};
    }

    C.reconfigureRoot(a, true);
    a->linkFrom(C[parent.c_str()]);

    const rai::String prefix = STRING('a' << cnt << '_');
    a->prefixSubtree(prefix);

    const rai::String agentBase = STRING(prefix << "base");
    C[agentBase]->setRelativePosition(base_pos);
    C[agentBase]->setRelativeQuaternion(base_quat);

    // check if home_pose is set, otherwise use default
    setActive(C, std::string(prefix.p));
    arr home_pose = C.getJointState();

    if (item.value().contains("home_pose")) {
      ss.clear();
      ss << item.value()["home_pose"];
      arr state;
      state.read(ss);

      assert(state.d0 == C.getJointState().N);
      home_pose = state;
    }

    if (item.value().contains("start_pose")) {
      ss.clear();
      ss << item.value()["start_pose"];
      arr state;
      state.read(ss);

      // std::cout << "Given start pose: " << state << std::endl;
      // std::cout << "Default start pose: " << C.getJointState() << std::endl;

      assert(state.d0 == C.getJointState().N);
      C.setJointState(state);
    }

    arr start_pose = C.getJointState();

    // check if speed is set, otherwise use default
    double vmax = 0.05;
    if (item.value().contains("vmax")) {
      vmax = item.value()["vmax"];
    }

    ConfigurationProblem cp(C);
    cp.activeOnly = true;
    const auto res = cp.query({}, false);

    const auto feasible = res->isFeasible;

    if (!feasible) {
      spdlog::error("Setting up configuration: Robot {} is in collision.", cnt);
      // res->write(std::cout);
      res->writeDetails(std::cout, cp, 0);
    }

    robots.push_back(Robot(prefix.p, robot_type, vmax));
    robots.back().home_pose = home_pose;
    robots.back().start_pose = start_pose;
    robots.back().ee_type = ee;

    ++cnt;
  }

  return robots;
}

std::vector<Robot> make_robot_environment_from_config(
    rai::Configuration &C, const std::string &config_file_path,
    const std::string &base_scene_path = "./in/scenes/floor.g") {

  std::ifstream ifs(config_file_path);
  json jf = json::parse(ifs);

  return make_robot_environment_from_json(C, jf, base_scene_path);
}

void add_objects_from_json(rai::Configuration &C, json jf) {
  uint cnt = 0;
  std::stringstream ss;

  for (const auto &item : jf["objects"].items()) {
    ss.clear();
    ss << item.value()["shape"];
    arr shape;
    shape.read(ss);

    ss.clear();
    ss << item.value()["start_pos"];
    arr base_pos;
    base_pos.read(ss);

    ss.clear();
    ss << item.value()["start_quat"];
    arr base_quat;
    base_quat.read(ss);

    ss.clear();
    ss << item.value()["goal_pos"];
    arr goal_pos;
    goal_pos.read(ss);

    ss.clear();
    ss << item.value()["goal_quat"];
    arr goal_quat;
    goal_quat.read(ss);

    std::string parent;
    if (item.value().contains("parent")) {
      parent = item.value()["parent"];
    } else {
      parent = "table";
    }

    auto *obj = C.addFrame(STRING("obj" << cnt + 1), parent.c_str());

    // randomize color
    arr col(3);
    rndUniform(col, 0, 1);

    obj->setShape(rai::ST_box, {shape(0), shape(1), shape(2), 0.01});
    obj->setContact(1);
    obj->setJoint(rai::JT_rigid);
    obj->setRelativePosition(base_pos);
    obj->setRelativeQuaternion(base_quat);
    obj->setColor({col(0), col(1), col(2)});

    auto *marker = C.addFrame("goal_marker", obj->name);
    marker->setShape(rai::ST_marker, {0.1});
    marker->setContact(0.);

    auto *goal = C.addFrame(STRING("goal" << cnt + 1), "table");

    goal->setShape(rai::ST_box, {shape(0), shape(1), shape(2), 0.01});
    goal->setContact(0);
    goal->setJoint(rai::JT_rigid);
    goal->setRelativePosition(goal_pos);
    goal->setRelativeQuaternion(goal_quat);
    // goal->setColor({0, 0, 0, 0.5});
    goal->setColor({col(0), col(1), col(2), 0.5});

    // check if something is in collision
    ConfigurationProblem cp(C);
    const auto res = cp.query({}, false);
    if (!res->isFeasible) {
      spdlog::error(
          "Initial configuration not feasible. Obj {} is in collision.", cnt);
    }

    ++cnt;
  }

  C.stepFcl();
}

void add_objects_from_config(rai::Configuration &C,
                             const std::string &config_file_path) {
  std::ifstream ifs(config_file_path);
  json jf = json::parse(ifs);
  add_objects_from_json(C, jf);
}

void add_obstacles_from_json(rai::Configuration &C, json jf) {
  std::stringstream ss;

  uint cnt = 0;
  for (const auto &item : jf["obstacles"].items()) {
    ss.clear();
    ss << item.value()["size"];
    arr size;
    size.read(ss);

    std::string shape;
    shape = item.value()["shape"];

    ss.clear();
    ss << item.value()["pos"];
    arr base_pos;
    base_pos.read(ss);

    ss.clear();
    ss << item.value()["quat"];
    arr base_quat;
    base_quat.read(ss);

    std::string name;
    if (item.value().contains("name")) {
      name = item.value()["name"];
    } else {
      name = "obs_" + std::to_string(cnt);
    }

    std::string parent;
    if (item.value().contains("parent")) {
      parent = item.value()["parent"];
    } else {
      parent = "table";
    }

    auto *obj = C.addFrame(name.c_str(), parent.c_str());

    rai::ShapeType shape_type;
    if (shape == "box") {
      shape_type = rai::ST_box;
    } else if (shape == "sphere") {
      shape_type = rai::ST_sphere;
    } else if (shape == "capsule") {
      shape_type = rai::ST_capsule;
    } else {
      std::cout << "SHAPE NOT DEFINED" << std::endl;
    }

    obj->setShape(shape_type, size);
    obj->setContact(1);
    obj->setRelativePosition(base_pos);
    obj->setRelativeQuaternion(base_quat);
  }

  // C.watch(true);
}

void add_obstacles_from_config(rai::Configuration &C,
                               const std::string &config_file_path) {
  if (config_file_path.size() == 0) {
    return;
  }

  std::ifstream ifs(config_file_path);
  json jf = json::parse(ifs);
  add_obstacles_from_json(C, jf);
}

// TODO: fill this in
bool check_configuration_feasibility(const rai::Configuration &C) {
  return true;
}

std::vector<Robot> tub_lab_setting(rai::Configuration &C) {
  auto *base = C.addFrame("world", "");
  base->setShape(rai::ST_marker, {0.001});
  base->setPosition({0., 0., .5});
  base->setContact(0.);

  C.addFile("./in/scenes/table.g");

  const arrA basePos = {{-.4, -.3, 0.00}, {.4, -.3, 0.0}, {.0, .6, 0.15}};

  std::vector<Robot> robots;

  for (uint i = 0; i < 2; i++) {
    auto *a = C.addFile("./in/robots/franka.g");
    C.reconfigureRoot(a, true);
    a->linkFrom(C["table"]);

    const rai::String prefix = STRING('a' << i << '_');
    a->prefixSubtree(prefix);

    const rai::String agentBase = STRING(prefix << "base");
    C[agentBase]->setRelativePosition(basePos(i));

    robots.push_back(Robot(prefix.p, RobotType::panda, 0.05));
    robots.back().home_pose = C.getJointState();
    robots.back().start_pose = C.getJointState();
    robots.back().ee_type = EndEffectorType::two_finger;
  }

  return robots;
}

std::vector<Robot> more_robots(rai::Configuration &C, const uint n = 2) {
  auto *base = C.addFrame("world", "");
  base->setShape(rai::ST_marker, {0.001});
  base->setPosition({0., 0., .5});
  base->setContact(0.);

  C.addFile("./in/scenes/table.g");

  arrA basePos = {
      {-.5, -.35, 0.00}, {.5, -.35, 0.0}, {-.5, .55, 0.}, {.5, .55, 0.0}};
  arrA baseQuat = {
      {0.924, 0, 0, 0.383},
      {0.383, 0, 0, 0.924},
      {0.924, 0, 0, -0.383},
      {-0.383, 0, 0, 0.924},
  };

  std::vector<Robot> robots;
  for (uint i = 0; i < n; i++) {
    auto *a = C.addFile("./in/robots/franka.g");
    C.reconfigureRoot(a, true);
    a->linkFrom(C["table"]);

    const rai::String prefix = STRING('a' << i << '_');
    a->prefixSubtree(prefix);

    const rai::String agentBase = STRING(prefix << "base");
    // const rai::String agentBase = STRING(prefix<<"base");
    // C[agentBase]->setPosition(basePos(i));
    C[agentBase]->setRelativePosition(basePos(i));
    C[agentBase]->setQuaternion(baseQuat(i));

    setActive(C, std::string(prefix.p));
    arr state = C.getJointState();
    state(1) -= 0.25;
    state(3) += 0.25;
    C.setJointState(state);

    robots.push_back(Robot(prefix.p, RobotType::panda, 0.05));
    robots.back().home_pose = C.getJointState();
    robots.back().ee_type = EndEffectorType::two_finger;
  }

  return robots;
}

std::vector<Robot> opposite_robot_configuration(rai::Configuration &C) {
  C.addFile("./in/scenes/floor.g");

  const arrA basePos = {{-.5, -.1, 0.00}, {.5, .1, 0.0}, {.0, .6, 0.15}};

  const arrA baseQuat = {
      {1, 0, 0, 0},
      {0, 0, 0, 1},
      {0.924, 0, 0, -0.383},
      {-0.383, 0, 0, 0.924},
  };

  std::vector<Robot> robots;
  for (uint i = 0; i < 2; i++) {
    auto *a = C.addFile("./in/robots/franka.g");
    C.reconfigureRoot(a, true);
    a->linkFrom(C["table"]);

    const rai::String prefix = STRING('a' << i << '_');
    a->prefixSubtree(prefix);

    const rai::String agentBase = STRING(prefix << "base");
    C[agentBase]->setRelativePosition(basePos(i));
    C[agentBase]->setQuaternion(baseQuat(i));

    setActive(C, std::string(prefix.p));
    arr state = C.getJointState();
    state(1) -= 0.25;
    // state(3) += 0.25;
    C.setJointState(state);

    robots.push_back(Robot(prefix.p, RobotType::panda, 0.05));
    robots.back().home_pose = C.getJointState();
    robots.back().ee_type = EndEffectorType::two_finger;
  }

  return robots;
}

std::vector<Robot> side_by_side_robot_configuration(rai::Configuration &C) {
  C.addFile("./in/scenes/floor.g");

  const arrA basePos = {{-.4, -.3, 0.00}, {.4, -.3, 0.0}, {.0, .6, 0.15}};

  std::vector<Robot> robots;
  for (uint i = 0; i < 2; i++) {
    auto *a = C.addFile("./in/robots/franka.g");
    C.reconfigureRoot(a, true);
    a->linkFrom(C["table"]);

    const rai::String prefix = STRING('a' << i << '_');
    a->prefixSubtree(prefix);

    const rai::String agentBase = STRING(prefix << "base");
    C[agentBase]->setRelativePosition(basePos(i));

    robots.push_back(Robot(prefix.p, RobotType::panda, 0.05));
    robots.back().home_pose = C.getJointState();
    robots.back().ee_type = EndEffectorType::two_finger;
  }

  return robots;
}

std::vector<Robot> make_configuration_from_base_pose_and_quat(
    rai::Configuration &C, const arrA base_pos, const arrA base_quat,
    const bool two_finger_gripper) {
  assert(base_pos.d0 == base_quat.d0);
  const uint N = base_pos.d0;

  std::vector<Robot> robots;

  for (uint i = 0; i < N; ++i) {
    rai::Frame *a;
    EndEffectorType ee;
    if (two_finger_gripper) {
      a = C.addFile("./in/robots/ur5.g");
      ee = EndEffectorType::two_finger;
    } else {
      a = C.addFile("./in/robots/ur5_vacuum.g");
      ee = EndEffectorType::vacuum;
    }
    // auto *a = C.addFile("./in/franka.g");
    C.reconfigureRoot(a, true);
    a->linkFrom(C["table"]);

    const rai::String prefix = STRING('a' << i << '_');
    a->prefixSubtree(prefix);

    const rai::String agentBase = STRING(prefix << "base");
    C[agentBase]->setRelativePosition(base_pos(i));
    C[agentBase]->setQuaternion(base_quat(i));

    setActive(C, std::string(prefix.p));
    arr state = C.getJointState();
    // state(1) -= .5;
    // state(3) += 0.25;
    C.setJointState(state);

    robots.push_back(Robot(prefix.p, RobotType::ur5, 0.05));
    robots.back().home_pose = C.getJointState();
    robots.back().start_pose = C.getJointState();
    robots.back().ee_type = ee;
  }

  return robots;
}

std::vector<Robot>
single_robot_configuration(rai::Configuration &C,
                           const bool two_finger_gripper = true) {
  C.addFile("./in/scenes/floor.g");

  const arrA basePos = {{-.4, -.3, 0.00}};

  const arrA baseQuat = {{1, 0, 0, 1}};

  return make_configuration_from_base_pose_and_quat(C, basePos, baseQuat,
                                                    two_finger_gripper);
}

std::vector<Robot>
two_robot_configuration(rai::Configuration &C,
                        const bool two_finger_gripper = true) {
  C.addFile("./in/scenes/floor.g");

  const arrA basePos = {{-.4, -.3, 0.00}, {.4, -.3, 0.0}};

  const arrA baseQuat = {{1, 0, 0, 1}, {1, 0, 0, 1}};

  return make_configuration_from_base_pose_and_quat(C, basePos, baseQuat,
                                                    two_finger_gripper);
}

std::vector<Robot>
opposite_three_robot_configuration(rai::Configuration &C,
                                   const bool two_finger_gripper = true) {
  C.addFile("./in/scenes/floor.g");

  const arrA basePos = {{-.4, -.3, 0.00}, {.4, -.3, 0.0}, {.0, .6, 0.0}};

  const arrA baseQuat = {
      {1, 0, 0, 1},
      {1, 0, 0, 1},
      {-1, 0, 0, 1},
  };

  return make_configuration_from_base_pose_and_quat(C, basePos, baseQuat,
                                                    two_finger_gripper);
}

void random_objects(rai::Configuration &C, const uint N,
                    const double width = .5) {
  for (uint i = 0; i < N; ++i) {
    auto *obj = C.addFrame(STRING("obj" << i + 1), "table");

    arr shape(2);
    rndUniform(shape, 0.02, 0.04);
    shape(0) *= 2;

    obj->setShape(rai::ST_box, {shape(0), shape(1), 0.06, 0.01});
    obj->setContact(1.);
    obj->setJoint(rai::JT_rigid);

    while (true) {
      arr rnd(2);
      rndUniform(rnd, 0, 1);
      rnd(0) = (rnd(0) - 0.5) * width;
      rnd(1) = (rnd(1) - 0.3) * 0.7;

      obj->setPosition({rnd(0), rnd(1), 0.66});

      ConfigurationProblem cp(C);
      if (cp.query({}, false)->isFeasible) {
        break;
      }
    }

    auto *goal = C.addFrame(STRING("goal" << i + 1), "table");

    goal->setShape(rai::ST_box, {shape(0), shape(1), 0.06, 0.01});
    goal->setContact(1.);
    goal->setColor({0, 0, 0, 0.5});
    goal->setJoint(rai::JT_rigid);

    while (true) {
      arr rnd(2);
      rndUniform(rnd, 0, 1);
      // rnd(0) = (rnd(0) - 0.5) * 0.5;
      // rnd(1) = (rnd(1) - 0.3) * 0.5;

      rnd(0) = (rnd(0) - 0.5) * width;
      rnd(1) = (rnd(1) - 0.3) * 1;

      goal->setPosition({rnd(0), rnd(1), 0.66});

      ConfigurationProblem cp(C);
      if (cp.query({}, false)->isFeasible) {
        break;
      }
    }

    goal->setContact(1.);
  }

  for (auto f : C.frames) {
    if (f->name.contains("goal")) {
      f->setContact(0);
    }
  }
}

arr get_random_axis_aligned_orientation() {
  // Generate a random integer between 0 and 5
  int rnd_orientation = std::rand() % 6;

  arr orientation(4);

  switch (rnd_orientation) {
  case 0:
    orientation = {1, 0, 0, 0};
    break; // Identity rotation (z -> +z)
  case 1:
    orientation = {0, 1, 0, 0};
    break; // 180° rotation about x-axis (z -> -z)
  case 2:
    orientation = {0, 0, 0.7071, 0.7071};
    break; // 90° rotation about y-axis (z -> +x)
  case 3:
    orientation = {0, 0, -0.7071, 0.7071};
    break; // -90° rotation about y-axis (z -> -x)
  case 4:
    orientation = {0.7071, 0.7071, 0, 0};
    break; // 90° rotation about x-axis (z -> +y)
  case 5:
    orientation = {0.7071, -0.7071, 0, 0};
    break; // -90° rotation about x-axis (z -> -y)
  default:
    orientation = {1, 0, 0, 0}; // Fallback (identity)
  }

  // std::cout << orientation << std::endl;

  return orientation;
}

void cubes_with_random_rotation(rai::Configuration &C, const uint N,
                                const double width = .5) {
  for (uint i = 0; i < N; ++i) {
    auto *obj = C.addFrame(STRING("obj" << i + 1), "table");

    const arr shape = {0.05, 0.05, 0.05};

    obj->setShape(rai::ST_box, {shape(0), shape(1), shape(2), 0.01});
    obj->setContact(1.);
    obj->setJoint(rai::JT_rigid);
    obj->setColor({0.5, 0.5, 0.5, 0.5});

    while (true) {
      arr rnd(2);
      rndUniform(rnd, 0, 1);
      rnd(0) = (rnd(0) - 0.5) * width;
      rnd(1) = (rnd(1) - 0.3) * 0.7;

      obj->setPosition({rnd(0), rnd(1), 0.66});

      obj->setRelativeQuaternion(get_random_axis_aligned_orientation());

      ConfigurationProblem cp(C);
      if (cp.query({}, false)->isFeasible) {
        break;
      }
    }

    auto *marker = C.addFrame("goal_tmp_1", obj->name);
    marker->setShape(rai::ST_marker, {0.1});
    marker->setContact(0.);

    auto *goal = C.addFrame(STRING("goal" << i + 1), "table");

    goal->setShape(rai::ST_box, {shape(0), shape(1), 0.06, 0.01});
    goal->setContact(1.);
    goal->setColor({0, 0, 0, 0.5});
    goal->setJoint(rai::JT_rigid);

    auto *marker2 = C.addFrame("goal_tmp_2", goal->name);
    marker2->setShape(rai::ST_marker, {0.1});
    marker2->setContact(0.);


    while (true) {
      arr rnd(2);
      rndUniform(rnd, 0, 1);
      // rnd(0) = (rnd(0) - 0.5) * 0.5;
      // rnd(1) = (rnd(1) - 0.3) * 0.5;

      rnd(0) = (rnd(0) - 0.5) * width;
      rnd(1) = (rnd(1) - 0.3) * 1;

      goal->setPosition({rnd(0), rnd(1), 0.66});
      goal->setRelativeQuaternion(get_random_axis_aligned_orientation());

      ConfigurationProblem cp(C);
      if (cp.query({}, false)->isFeasible) {
        break;
      }
    }

    goal->setContact(1.);
  }

  for (auto f : C.frames) {
    if (f->name.contains("goal")) {
      f->setContact(0);
    }
  }
}

void line(rai::Configuration &C, const uint N, const double width = 2.) {
  for (uint i = 0; i < N; ++i) {
    auto *obj = C.addFrame(STRING("obj" << i + 1), "table");

    arr shape(2);
    rndUniform(shape, 0.02, 0.04);
    shape(0) *= 2;

    obj->setShape(rai::ST_box, {shape(0), shape(1), 0.06, 0.01});
    obj->setContact(1.);
    obj->setJoint(rai::JT_rigid);
    obj->setPosition({width / (N - 1) * i - width / 2, 0.3, 0.66});

    auto *goal = C.addFrame(STRING("goal" << i + 1), "table");

    goal->setShape(rai::ST_box, {shape(0), shape(1), 0.06, 0.01});
    goal->setContact(1.);
    goal->setColor({0, 0, 0, 0.5});
    goal->setJoint(rai::JT_rigid);
    goal->setPosition({width / (N - 1) * i - width / 2, 0., 0.66});

    goal->setContact(1.);
  }

  for (auto f : C.frames) {
    if (f->name.contains("goal")) {
      f->setContact(0);
    }
  }
}

void shuffled_line(rai::Configuration &C, const uint N, const double width = 2.,
                   const bool random_size = true) {
  const double heigth_offset = 0.6 - 0.05 + 0.05 / 2 + 0.001;
  const double height = 0.06;

  std::vector<uint> tmp;
  for (uint q = 0; q < N; ++q) {
    tmp.push_back(q);
  }

  // Shuffle the vector
  std::random_shuffle(tmp.begin(), tmp.end());

  for (uint i = 0; i < N; ++i) {
    auto *obj = C.addFrame(STRING("obj" << i + 1), "table");

    arr shape(2);
    rndUniform(shape, 0.02, 0.06);
    shape(1) *= 2;

    if (!random_size) {
      shape(0) = 0.04;
      shape(1) = 0.2;
    }

    obj->setShape(rai::ST_box, {shape(0), shape(1), height, 0.01});
    obj->setContact(1.);
    obj->setJoint(rai::JT_rigid);
    obj->setPosition(
        {width / (N - 1) * i - width / 2, 0.3, height / 2 + heigth_offset});
    obj->setQuaternion({1, 0, 0, 1});

    auto *marker = C.addFrame("goal_marker", obj->name);
    marker->setShape(rai::ST_marker, {0.1});
    marker->setContact(0.);

    auto *goal = C.addFrame(STRING("goal" << i + 1), "table");

    goal->setShape(rai::ST_box, {shape(0), shape(1), height, 0.01});
    goal->setContact(1.);
    goal->setColor({0, 0, 0, 0.5});
    goal->setJoint(rai::JT_rigid);
    goal->setPosition(
        {width / (N - 1) * tmp[i] - width / 2, 0., height / 2 + heigth_offset});

    goal->setContact(1.);
  }

  for (auto f : C.frames) {
    if (f->name.contains("goal")) {
      f->setContact(0);
    }
  }
}

void big_objs(rai::Configuration &C, const uint N) {
  const double width = 2.;
  const double heigth_offset = 0.6 - 0.05 + 0.05 / 2;

  std::vector<uint> tmp;
  for (uint q = 0; q < N; ++q) {
    tmp.push_back(q);
  }

  // Shuffle the vector
  std::random_shuffle(tmp.begin(), tmp.end());

  for (uint i = 0; i < N; ++i) {
    auto *obj = C.addFrame(STRING("obj" << i + 1), "table");

    arr shape(2);
    rndUniform(shape, 0.04, 0.1);
    shape(0) = shape(0) * 1.5;

    const double height = 0.06;

    std::cout << shape << std::endl;

    obj->setShape(rai::ST_box, {shape(0), shape(1), height, 0.01});
    obj->setContact(1.);
    obj->setJoint(rai::JT_rigid);
    obj->setPosition(
        {width / (N - 1) * i - 1, 0.3, height / 2 + heigth_offset});

    auto *goal = C.addFrame(STRING("goal" << i + 1), "table");

    goal->setShape(rai::ST_box, {shape(0), shape(1), 0.06, 0.01});
    goal->setContact(1.);
    goal->setColor({0, 0, 0, 0.5});
    goal->setJoint(rai::JT_rigid);
    goal->setPosition(
        {width / (N - 1) * tmp[i] - 1, 0., height / 2 + heigth_offset});

    goal->setContact(1.);
  }

  for (auto f : C.frames) {
    if (f->name.contains("goal")) {
      f->setContact(0);
    }
  }
}

void pick_and_place(rai::Configuration &C) {
  auto *base = C.addFrame("world", "");
  base->setShape(rai::ST_marker, {0.001});
  base->setPosition({0., 0., .5});
  base->setContact(0.);

  C.addFile("./in/robots/table_pick_place.g");

  const arrA basePos = {{-.5, -.1, 0.00}, {.5, .1, 0.0}, {.0, .6, 0.15}};

  const arrA baseQuat = {
      {1, 0, 0, 0},
      {0, 0, 0, 1},
      {0.924, 0, 0, -0.383},
      {-0.383, 0, 0, 0.924},
  };

  for (uint i = 0; i < 2; i++) {
    auto *a = C.addFile("./in/robots/franka.g");
    C.reconfigureRoot(a, true);
    a->linkFrom(C["table"]);

    const rai::String prefix = STRING('a' << i << '_');
    a->prefixSubtree(prefix);

    const rai::String agentBase = STRING(prefix << "base");
    C[agentBase]->setRelativePosition(basePos(i));
    C[agentBase]->setQuaternion(baseQuat(i));

    setActive(C, std::string(prefix.p));
    arr state = C.getJointState();
    state(1) -= 0.25;
    // state(3) += 0.25;
    C.setJointState(state);
  }

  // C.watch(true);
}

std::unordered_map<Robot, arr>
get_robot_home_poses(const std::vector<Robot> &robots) {
  std::unordered_map<Robot, arr> poses;
  for (const auto &r : robots) {
    poses[r] = r.home_pose;
  }

  return poses;
}