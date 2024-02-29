#pragma once

#include <Kin/kin.h>

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

void setRobotJointState(){}

void labSetting(rai::Configuration &C) {
  auto *base = C.addFrame("world", "");
  base->setShape(rai::ST_marker, {0.001});
  base->setPosition({0., 0., .5});
  base->setContact(0.);

  C.addFile("./in/table.g");

  const arrA basePos = {{-.4, -.3, 0.00}, {.4, -.3, 0.0}, {.0, .6, 0.15}};

  for (uint i = 0; i < 2; i++) {
    auto *a = C.addFile("./in/franka.g");
    C.reconfigureRoot(a, true);
    a->linkFrom(C["table"]);

    const rai::String prefix = STRING('a' << i << '_');
    a->prefixSubtree(prefix);

    const rai::String agentBase = STRING(prefix << "base");
    C[agentBase]->setRelativePosition(basePos(i));
  }
}

void more_robots(rai::Configuration &C, const uint n = 2) {
  auto *base = C.addFrame("world", "");
  base->setShape(rai::ST_marker, {0.001});
  base->setPosition({0., 0., .5});
  base->setContact(0.);

  C.addFile("./in/table.g");

  arrA basePos = {
      {-.5, -.35, 0.00}, {.5, -.35, 0.0}, {-.5, .55, 0.}, {.5, .55, 0.0}};
  arrA baseQuat = {
      {0.924, 0, 0, 0.383},
      {0.383, 0, 0, 0.924},
      {0.924, 0, 0, -0.383},
      {-0.383, 0, 0, 0.924},
  };

  for (uint i = 0; i < n; i++) {
    auto *a = C.addFile("./in/franka.g");
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
  }
}

void opposite_robot_configuration(rai::Configuration &C){
  C.addFile("./in/floor.g");

  const arrA basePos = {{-.5, -.1, 0.00}, {.5, .1, 0.0}, {.0, .6, 0.15}};

  const arrA baseQuat = {
      {1, 0, 0, 0},
      {0, 0, 0, 1},
      {0.924, 0, 0, -0.383},
      {-0.383, 0, 0, 0.924},
  };

  for (uint i = 0; i < 2; i++) {
    auto *a = C.addFile("./in/franka.g");
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

  C.watch(true);
}

void side_by_side_robot_configuration(rai::Configuration &C){
  C.addFile("./in/floor.g");

  const arrA basePos = {{-.4, -.3, 0.00}, {.4, -.3, 0.0}, {.0, .6, 0.15}};

  for (uint i = 0; i < 2; i++) {
    auto *a = C.addFile("./in/franka.g");
    C.reconfigureRoot(a, true);
    a->linkFrom(C["table"]);

    const rai::String prefix = STRING('a' << i << '_');
    a->prefixSubtree(prefix);

    const rai::String agentBase = STRING(prefix << "base");
    C[agentBase]->setRelativePosition(basePos(i));
  }

  // C.watch(true);
}

void opposite_three_robot_configuration(rai::Configuration &C, const bool two_finger_gripper=true){
  C.addFile("./in/floor.g");

  const arrA basePos = {{-.4, -.3, 0.00}, {.4, -.3, 0.0}, {.0, .6, 0.0}};

    const arrA baseQuat = {
      {1, 0, 0, 1},
      {1, 0, 0, 1},
      {-1, 0, 0, 1},
      {-0.383, 0, 0, 0.924},
  };

  for (uint i = 0; i < 3; i++) {
    rai::Frame *a;
    if (two_finger_gripper){
      a = C.addFile("./in/ur5.g");
    }
    else{
      a = C.addFile("./in/ur5_vacuum.g");
    }
    // auto *a = C.addFile("./in/franka.g");
    C.reconfigureRoot(a, true);
    a->linkFrom(C["table"]);

    const rai::String prefix = STRING('a' << i << '_');
    a->prefixSubtree(prefix);

    const rai::String agentBase = STRING(prefix << "base");
    C[agentBase]->setRelativePosition(basePos(i));
    C[agentBase]->setQuaternion(baseQuat(i));

    setActive(C, std::string(prefix.p));
    arr state = C.getJointState();
    // state(1) -= .5;
    // state(3) += 0.25;
    C.setJointState(state);
  }

  // C.watch(true);
}

void random_objects(rai::Configuration &C, const uint N){  
  for (uint i=0; i<N; ++i){
    auto *obj = C.addFrame(STRING("obj"<<i+1), "table");

    arr shape(2);
    rndUniform(shape, 0.02, 0.04);
    shape(0) *= 2;

    obj->setShape(rai::ST_ssBox, {shape(0), shape(1), 0.06, 0.01});
    obj->setContact(1.);
    obj->setJoint(rai::JT_rigid);

    while(true){
      arr rnd(2);
      rndUniform(rnd, 0, 1);
      rnd(0) = (rnd(0) - 0.5) * 2;
      rnd(1) = (rnd(1) - 0.3) * 0.7;

      obj->setPosition({rnd(0), rnd(1), 0.66});

      ConfigurationProblem cp(C);
      if (cp.query({}, false)->isFeasible){
        break;
      } 
    }

    auto *goal = C.addFrame(STRING("goal"<<i+1), "table");

    goal->setShape(rai::ST_ssBox, {shape(0), shape(1), 0.06, 0.01});
    goal->setContact(1.);
    goal->setColor({0, 0, 0, 0.5});
    goal->setJoint(rai::JT_rigid);
    
    while(true){
      arr rnd(2);
      rndUniform(rnd, 0, 1);
      // rnd(0) = (rnd(0) - 0.5) * 0.5;
      // rnd(1) = (rnd(1) - 0.3) * 0.5;

      rnd(0) = (rnd(0) - 0.5) * 2;
      rnd(1) = (rnd(1) - 0.3) * 1;

      goal->setPosition({rnd(0), rnd(1), 0.66});

      ConfigurationProblem cp(C);
      if (cp.query({}, false)->isFeasible){
        break;
      } 
    }

    goal->setContact(1.);
  }

  for (auto f: C.frames){
    if (f->name.contains("goal")){
      f->setContact(0);
    }
  }
}

void line(rai::Configuration &C, const uint N){  
  const double width = 2.;

  for (uint i=0; i<N; ++i){
    auto *obj = C.addFrame(STRING("obj"<<i+1), "table");

    arr shape(2);
    rndUniform(shape, 0.02, 0.04);
    shape(0) *= 2;

    obj->setShape(rai::ST_ssBox, {shape(0), shape(1), 0.06, 0.01});
    obj->setContact(1.);
    obj->setJoint(rai::JT_rigid);
    obj->setPosition({width / (N-1) * i - 1, 0.3, 0.66});

    auto *goal = C.addFrame(STRING("goal"<<i+1), "table");

    goal->setShape(rai::ST_ssBox, {shape(0), shape(1), 0.06, 0.01});
    goal->setContact(1.);
    goal->setColor({0, 0, 0, 0.5});
    goal->setJoint(rai::JT_rigid);
    goal->setPosition({width / (N-1) * i - 1, 0., 0.66});

    goal->setContact(1.);
  }

  for (auto f: C.frames){
    if (f->name.contains("goal")){
      f->setContact(0);
    }
  }
}

void shuffled_line(rai::Configuration &C, const uint N){  
  const double width = 2.;

  std::vector<uint> tmp;
  for (uint q = 0; q < N; ++q) {
    tmp.push_back(q);
  }

  // Shuffle the vector
  std::random_shuffle(tmp.begin(), tmp.end());

  for (uint i=0; i<N; ++i){
    auto *obj = C.addFrame(STRING("obj"<<i+1), "table");

    arr shape(2);
    rndUniform(shape, 0.02, 0.04);
    shape(0) *= 2;

    obj->setShape(rai::ST_ssBox, {shape(0), shape(1), 0.06, 0.01});
    obj->setContact(1.);
    obj->setJoint(rai::JT_rigid);
    obj->setPosition({width / (N-1) * i - 1, 0.3, 0.66});

    auto *goal = C.addFrame(STRING("goal"<<i+1), "table");

    goal->setShape(rai::ST_ssBox, {shape(0), shape(1), 0.06, 0.01});
    goal->setContact(1.);
    goal->setColor({0, 0, 0, 0.5});
    goal->setJoint(rai::JT_rigid);
    goal->setPosition({width / (N-1) * tmp[i] - 1, 0., 0.66});

    goal->setContact(1.);
  }

  for (auto f: C.frames){
    if (f->name.contains("goal")){
      f->setContact(0);
    }
  }
}

void pick_and_place(rai::Configuration &C) {
  auto *base = C.addFrame("world", "");
  base->setShape(rai::ST_marker, {0.001});
  base->setPosition({0., 0., .5});
  base->setContact(0.);

  C.addFile("./in/table_pick_place.g");

  const arrA basePos = {{-.5, -.1, 0.00}, {.5, .1, 0.0}, {.0, .6, 0.15}};

  const arrA baseQuat = {
      {1, 0, 0, 0},
      {0, 0, 0, 1},
      {0.924, 0, 0, -0.383},
      {-0.383, 0, 0, 0.924},
  };

  for (uint i = 0; i < 2; i++) {
    auto *a = C.addFile("./in/franka.g");
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

