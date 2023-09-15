#pragma once

arr constructShortcutPath(const rai::Configuration &C, const arr &path,
                          const uint i, const uint j,
                          const std::vector<uint> short_ind) {
  auto periodicDimensions = std::vector<bool>(C.getJointState().N, false);

  for (auto *j : C.activeJoints) {
    if (j->type == rai::JT_hingeX || j->type == rai::JT_hingeY ||
        j->type == rai::JT_hingeZ) {
      periodicDimensions[j->qIndex] = true;
    }
  }

  const arr p1 = path[i]();
  const arr p2 = path[j]();
  arr delta = p2 - p1;
  for (uint l = 0; l < delta.N; ++l) {
    if (periodicDimensions[l]) {
      // this is an angular joint -> we need to check the other direction
      const double start = p2(l);
      const double end = p1(l);
      delta(l) = std::fmod(start - end + 3. * RAI_PI, 2 * RAI_PI) - RAI_PI;
    }
  }

  arr p(j - i, path.d1);

  for (uint l = 0; l < j - i; ++l) {
    for (uint k = 0; k < path.d1; ++k) {
      if (std::find(short_ind.begin(), short_ind.end(), k) != short_ind.end()) {
        const double a = 1. * l / (j - i);
        // p(l, k) = path(i, k) + a * (path(j, k) - path(i, k));
        p(l, k) = path(i, k) + a * (delta(k));
      } else {
        p(l, k) = path(l + i, k);
      }
    }
  }

  return p;
}

double corput(int n, const int base = 2) {
  double q = 0, bk = (double)1 / base;

  while (n > 0) {
    q += (n % base) * bk;
    n /= base;
    bk /= base;
  }

  return q;
}

const double pathLength(const arr &path) {
  double cost = 0.;
  for (uint i = 0; i < path.d0 - 1; ++i) {
    cost += length(path[i] - path[i + 1]);
  }

  return cost;
}

arr partial_shortcut(TimedConfigurationProblem &TP, const arr &initialPath,
                     const uint t0) {
  if (TP.A.prePlannedFrames.N != 0) {
    return initialPath;
  }

  arr smoothedPath = initialPath;
  /*for (uint i=0; i<smoothedPath.d0; i+=4){
    TP.query(smoothedPath[i], t0 + i);
    TP.C.watch(true);
  }*/

  std::vector<double> costs;
  costs.push_back(pathLength(initialPath));

  const uint max_iter = 100;
  const uint resolution = 1;
  // const uint max_iter = 100;
  // const uint resolution = 5;
  for (uint k = 0; k < max_iter; ++k) {
    // choose random indices
    int i, j;
    do {
      i = rand() % initialPath.d0;
      j = rand() % initialPath.d0;
    } while (abs(j - i) <= 1);

    if (i > j) {
      std::swap(i, j);
    }

    // choose, which indices to shortcut
    std::vector<uint> ind;
    for (uint q = 0; q < smoothedPath.d1; ++q) {
      const double r = 1. * rand() / RAND_MAX;
      if (r > 1. / smoothedPath.d1) {
        ind.push_back(q);
      }
    }

    // construct the new path
    auto p = constructShortcutPath(TP.C, smoothedPath, i, j, {});
    auto ps = constructShortcutPath(TP.C, smoothedPath, i, j, ind);

    bool shortcutFeasible = true;

    // hack, since I didnt wanna move my projection method
    PathFinder_RRT_Time planner(TP);

    const double len = pathLength(ps);

    // check if the new path is feasible (interpolate)
    uintA q;
    q.setStraightPerm(j - i - 1);
    q.permuteRandomly();

    for (uint n : q) {
      const arr dir = ps[n + 1] - ps[n];
      for (uint l = 0; l < resolution; ++l) {
        const double interp = corput(l);
        const arr point = ps[n] + dir * interp;
        const double t = t0 + i + n + interp;

        const auto qr = planner.TP.query(point, t);
        if (!qr->isFeasible) {
          shortcutFeasible = false;
          break;
        }
      }
    }

    // check if the new path is shorter
    if (shortcutFeasible && pathLength(p) > len) {
      for (uint n = 1; n < j - i; ++n) {
        smoothedPath[i + n] = ps[n];
      }
    }

    auto c = pathLength(smoothedPath);
    costs.push_back(c);

    // proxy measure for convergence
    const uint conv = 100;
    if (k > conv && abs(costs.back() - costs[costs.size() - conv - 1]) < 1e-6) {
      std::cout << "converged after " << k << " iterations " << costs[0] << " "
                << costs.back() << std::endl;
      break;
    }
  }

  std::cout << '\t' << "Change in path costs: " << costs[0] << " "
            << costs.back() << std::endl;
  return smoothedPath;
}

arr smoothing(const rai::Animation &A, rai::Configuration &C, const arr &ts,
              const arr &path, const std::string prefix) {
  OptOptions options;
  options.stopIters = 100;
  options.damping = 1e-3;

  std::cout << "setting up komo" << std::endl;
  KOMO komo;
  komo.setModel(C, true);
  komo.setTiming(1., ts.N, 5, 2);
  komo.verbose = 0;
  komo.solver = rai::KS_sparse;

  komo.add_collision(true, .001, 1e1);
  komo.add_qControlObjective({}, 2, 1e1);
  komo.add_qControlObjective({}, 1, 1e1);

  komo.setConfiguration(-2, path[0]);
  komo.setConfiguration(-1, path[0]);
  komo.setConfiguration(0, path[0]);

  // make pen tip go a way from the table
  const double offset = 0.06;
  komo.addObjective({0.1, 0.9}, FS_distance,
                    {"table", STRING(prefix << "pen_tip")}, OT_ineq, {1e1},
                    {-offset});
  // komo.addObjective({0.1, 0.8}, FS_distance,
  //                  {"table", STRING(prefix << "pen_tip")}, OT_sos, {1e1});

  // position
  // komo.addObjective({0}, FS_qItself, {}, OT_eq, {1e1}, path[0]);
  komo.addObjective({1}, FS_qItself, {}, OT_eq, {1e1}, path[-1]);

  // speed
  // komo.addObjective({0.0, 0.05}, FS_qItself, {}, OT_eq, {1e1}, {},
  //                  1); // slow at beginning
  komo.addObjective({0.95, 1.0}, FS_qItself, {}, OT_eq, {1e1}, {},
                    1); // slow at end

  // acceleration
  // komo.addObjective({0.0, 0.05}, FS_qItself, {}, OT_eq, {1e1}, {},
  //                  2); // slow at beginning
  komo.addObjective({0.95, 1.0}, FS_qItself, {}, OT_eq, {1e1}, {},
                    2); // slow at end

  setKomoToAnimation(komo, C, A, ts);

  for (uint j = 0; j < ts.d0; ++j) {
    komo.setConfiguration(j, path[j]);
  }

  std::cout << "running komo" << std::endl;
  komo.run_prepare(0.0);
  komo.run(options);
  std::cout << "done komo" << std::endl;

  const double ineq = komo.getReport(false).get<double>("ineq");
  const double eq = komo.getReport(false).get<double>("eq");

  std::cout << "smoothing komo ineq: " << ineq << " eq: " << eq << std::endl;

  // if (eq > 2 || ineq > 2){
  komo.getReport(true);
  // komo.pathConfig.watch(true);
  //}

  // komo.pathConfig.watch(true);

  // check if the path is actually feasible
  arr smooth(ts.N, path[0].N);
  for (uint j = 0; j < ts.N; ++j) {
    smooth[j] = komo.getPath_q(j);
  }

  // force boundary conditions to be true
  smooth[0] = path[0];
  smooth[-1] = path[-1];

  return smooth;
}

