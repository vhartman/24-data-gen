#pragma once

#include "spdlog/spdlog.h"

#include <Manip/timedPath.h>
#include <KOMO/komo.h>
#include <Geo/fclInterface.h>

#include <PlanningSubroutines/ConfigurationProblem.h>

#include "util.h"

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

arr partial_spacetime_shortcut(TimedConfigurationProblem &TP, const arr &initialPath,
                     const uint t0) {
  spdlog::info("Starting shortcutting");
  // We do not currently support preplaned frames here
  if (TP.A.prePlannedFrames.N != 0) {
    return initialPath;
  }

  // TP.C.fcl()->stopEarly = false;
  TP.C.fcl()->stopEarly = true;

  arr smoothedPath = initialPath;
  /*for (uint i=0; i<smoothedPath.d0; i+=4){
    TP.query(smoothedPath[i], t0 + i);
    TP.C.watch(true);
  }*/

  std::vector<double> costs;
  costs.push_back(pathLength(initialPath));

  const uint max_iter = 100;
  // const uint resolution = 2;
  const double resolution = 0.1;
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

    // choose which indices to shortcut
    std::vector<uint> ind;
    for (uint q = 0; q < smoothedPath.d1; ++q) {
      const double r = 1. * rand() / RAND_MAX;
      //if (r > 1. / smoothedPath.d1) {
        ind.push_back(q);
      //}
    }

    // construct the new path
    auto p = constructShortcutPath(TP.C, smoothedPath, i, j, {});
    auto ps = constructShortcutPath(TP.C, smoothedPath, i, j, ind);

    const double len = pathLength(ps);

    // if the path length of the shortcut path is not shorter than the original one, don't consider it
    if (pathLength(p) <= len){
      continue;
    }

    // check if the new path is feasible (interpolate)
    // permute the indices that we check
    uintA q;
    q.setStraightPerm(j - i - 1);
    q.permuteRandomly();

    // enable not checking everything here
    bool shortcutFeasible = true;
    for (const uint n : q) {
      const arr dir = ps[n + 1] - ps[n];
      const double dist = length(dir);
      const uint num_pts = uint(std::max(dist / resolution, 1.));
      for (uint l = 0; l < num_pts; ++l) {
        const double interp = corput(l);
        const arr point = ps[n] + dir * interp;
        const double t = t0 + i + n + interp;

        // std::cout << t << " " << point << std::endl;

        const auto qr = TP.query(point, t);

        // ConfigurationProblem cp(TP.C);
        // auto tmp = cp.query({}, false);

        // std::cout << qr->isFeasible << std::endl;
        // std::cout << tmp->isFeasible << std::endl;

        // if (tmp->isFeasible != qr->isFeasible){
        //   TP.C.watch();
        //   cp.C.watch(true);
        // }

        if (!qr->isFeasible) {
          // std::cout << "A" << std::endl;
          shortcutFeasible = false;
          break;
        }
      }
      if (!shortcutFeasible){break;}
    }

    // if path is valid, copy it over
    // we already know that it is shorter (from the check above)
    if (shortcutFeasible) {
      for (uint n = 1; n < j - i; ++n) {
        smoothedPath[i + n] = ps[n];
      }
    }

    const auto c = pathLength(smoothedPath);
    costs.push_back(c);

    // proxy measure for convergence
    const uint conv = 100;
    if (k > conv && abs(costs.back() - costs[costs.size() - conv - 1]) < 1e-6) {
      spdlog::info("Converged after {}, iterations", k);
      break;
    }
  }

  spdlog::info("Change in path cost {}, {}", costs[0], costs.back());
  return smoothedPath;
}

// this currently happens in full resolution, this tends to be slow
// mostly due to the necessary collision checks that are done
arr smoothing(const rai::Animation &A, rai::Configuration &C, const arr &ts,
              const arr &path, const std::string prefix) {
  const double skip = 1;
  const uint num_timesteps = ts.N / skip;

  arr scaled_ts(num_timesteps);
  for (uint i=0; i<num_timesteps; ++i){
    scaled_ts(i) = ts(0) + (ts(-1) - ts(0)) / (num_timesteps-1) * i;
  }

  TimedPath tp(path, ts);
  const arr scaled_path = tp.resample(scaled_ts, C);

  OptOptions options;
  options.stopIters = 10;
  options.damping = 1e-3;
  options.stopTolerance = 0.1;
  options.allowOverstep = false;
  options.maxStep = 1;

  auto pairs = get_cant_collide_pairs(C);
  C.fcl()->deactivatePairs(pairs);

  std::cout << "setting up komo for smoothing" << std::endl;
  KOMO komo;
  komo.setModel(C, true);
  komo.setTiming(1., num_timesteps, 5, 2);
  komo.world.fcl()->stopEarly = true;

  komo.verbose = 0;
  komo.solver = rai::KS_sparse;

  komo.add_collision(true, .00, 1e0);
  komo.add_qControlObjective({}, 2, 1e-1);
  komo.add_qControlObjective({}, 1, 1e-2);

  komo.setConfiguration(-2, path[0]);
  komo.setConfiguration(-1, path[0]);
  komo.setConfiguration(0, path[0]);

  // make pen tip go a way from the table
  // const double offset = 0.06;
  // komo.addObjective({0.1, 0.9}, FS_distance,
  //                   {"table", STRING(prefix << "pen_tip")}, OT_ineq, {1e1},
  //                   {-offset});
  // komo.addObjective({0.1, 0.8}, FS_distance,
  //                  {"table", STRING(prefix << "pen_tip")}, OT_sos, {1e1});

  // position
  // komo.addObjective({0}, FS_qItself, {}, OT_eq, {1e1}, path[0]);
  komo.addObjective({1., 1.}, FS_qItself, {}, OT_eq, {1e1}, path[-1]);

  // speed
  komo.addObjective({0.0, 0.05}, FS_qItself, {}, OT_eq, {1e1}, {},
                   1); // slow at beginning
  komo.addObjective({.97, 1.}, FS_qItself, {}, OT_eq, {1e1}, {},
                    1); // slow at end

  // acceleration
  // komo.addObjective({0.0, 0.05}, FS_qItself, {}, OT_eq, {1e1}, {},
  //                  2); // slow at beginning
  komo.addObjective({1.0, 1.}, FS_qItself, {}, OT_eq, {1e1}, {},
                    2); // slow at end

  setKomoToAnimation(komo, C, A, scaled_ts);

  for (uint j = 0; j < num_timesteps; ++j) {
    // interpolate q
    //arr q = path[next_idx] - path[prev_idx];
    //const arr q = path[j*skip];
    const arr q = scaled_path[j];
    komo.setConfiguration(j, q);
  }

  std::cout << "running komo" << std::endl;
  komo.run_prepare(0.0);
  komo.run(options);
  std::cout << "done komo" << std::endl;

  const double ineq = komo.getReport(false).get<double>("ineq");
  const double eq = komo.getReport(false).get<double>("eq");

  std::cout << "smoothing komo ineq: " << ineq << " eq: " << eq << std::endl;

  // if (eq > 2 || ineq > 2){
  // komo.getReport(true);
  // komo.pathConfig.watch(true);
  //}

  // komo.pathConfig.watch(true);

  // check if the path is actually feasible
  arr smooth(scaled_ts.N, path[0].N);
  for (uint j = 0; j < scaled_ts.N; ++j) {
    smooth[j] = komo.getPath_q(j);
  }
  
  // force boundary conditions to be true
  smooth[0] = path[0];
  smooth[-1] = path[-1];

  TimedPath tp_smooth(smooth, scaled_ts);
  arr unscaled_path = tp_smooth.resample(ts, C);

  spdlog::info("Done with smoothing");
  return unscaled_path;
}

