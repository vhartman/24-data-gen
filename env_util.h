#pragma once

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

