#pragma once

#include "spdlog/spdlog.h"

#include <Manip/timedPath.h>
#include <KOMO/komo.h>
#include <Geo/fclInterface.h>

#include <PlanningSubroutines/ConfigurationProblem.h>

#include "util.h"
#include "config.h"

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

// compute path length while considering periodic dimensions
const double path_length(const rai::Configuration &C, const arr &path, const bool inf_norm=true) {
  double cost = 0.;
  auto periodicDimensions = std::vector<bool>(C.getJointState().N, false);

  for (auto *j : C.activeJoints) {
    if (j->type == rai::JT_hingeX || j->type == rai::JT_hingeY ||
        j->type == rai::JT_hingeZ) {
      periodicDimensions[j->qIndex] = true;
    }
  }

  for (uint i = 0; i < path.d0 - 1; ++i) {
    arr delta = path[i] - path[i+1];

    for (uint l = 0; l < delta.N; ++l) {
      if (periodicDimensions[l]) {
        // this is an angular joint -> we need to check the other direction
        const double start = path[i](l);
        const double end = path[i+1](l);
        delta(l) = std::fmod(start - end + 3. * RAI_PI, 2 * RAI_PI) - RAI_PI;
      }
    }

    if (!inf_norm) {
      cost += length(delta);
    } else {
      cost += absMax(delta);
    }
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

  TP.C.fcl()->stopEarly = global_params.use_early_coll_check_stopping;

  arr smoothedPath = initialPath;
  /*for (uint i=0; i<smoothedPath.d0; i+=4){
    TP.query(smoothedPath[i], t0 + i);
    TP.C.watch(true);
  }*/

  std::vector<double> costs;
  costs.push_back(path_length(TP.C, initialPath));

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

    const double len = path_length(TP.C, ps);

    // if the path length of the shortcut path is not shorter than the original one, don't consider it
    if (path_length(TP.C, p) <= len){
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

    const auto c = path_length(TP.C, smoothedPath);
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
  arr scaled_path = tp.resample(scaled_ts, C);
  // const arr scaled_path = tmp;

   // correct path for periodic stuff - komo does not deal well with transition from -pi to pi
  auto periodicDimensions = std::vector<bool>(C.getJointState().N, false);

  for (auto *j: C.activeJoints){
    if (j->type == rai::JT_hingeX || j->type == rai::JT_hingeY|| j->type == rai::JT_hingeZ){
      periodicDimensions[j->qIndex] = true;
    }
  }

  arr tmp(scaled_path.d0, scaled_path.d1);
  tmp = 0;
  tmp[0] = scaled_path[0]();

  for (uint i=0; i<scaled_path.d0-1; ++i){
    const arr p1 = scaled_path[i]();
    const arr p2 = scaled_path[i+1]();
    arr delta = p2 - p1;
    for (uint j=0; j<delta.N; ++j){
      if (periodicDimensions[j]){
        // this is an angular joint -> we need to check the other direction
        const double start = p2(j);
        const double end = p1(j);
        delta(j) = std::fmod(start - end + 3.*RAI_PI, 2*RAI_PI) - RAI_PI;
      }
    }
    
    tmp[i+1] = tmp[i] + delta;
    // std::cout << path[i] << std::endl;
    // std::cout << tmp[i] << std::endl;
  }

  scaled_path = tmp;

  OptOptions options;
  options.stopIters = 50;
  options.damping = 10;
  // options.stopTolerance = 0.1;
  options.allowOverstep = false;
  options.nonStrictSteps = 5;
  options.wolfe = 0.001;
  // options.maxStep = 0.01;

  auto pairs = get_cant_collide_pairs(C);
  C.fcl()->deactivatePairs(pairs);

  spdlog::debug("setting up komo for smoothing");

  KOMO komo;
  komo.setModel(C, true);
  // komo.setTiming(1., num_timesteps, 5, 2);
  komo.setTiming(1., num_timesteps, 5, 2);
  // komo.world.fcl()->stopEarly = global_params.use_early_coll_check_stopping;

  // komo.animateOptimization = 3;

  // auto komo_pairs = get_cant_collide_pairs(komo.world);
  // komo.world.fcl()->deactivatePairs(komo_pairs);

  komo.verbose = 0;
  komo.solver = rai::KS_sparse;

  komo.add_collision(true, 0.1, 1e2);
  komo.add_jointLimits(true, 0., 1e1);

  komo.add_qControlObjective({}, 2, 1e0);
  komo.add_qControlObjective({}, 1, 1e0);

  // komo.add_qControlObjective({}, 3, 1e-1);

  setKomoToAnimation(komo, C, A, scaled_ts);

  komo.setConfiguration(-2, scaled_path[0]);
  komo.setConfiguration(-1, scaled_path[0]);
  komo.setConfiguration(0, scaled_path[0]);

  // make pen tip go a way from the table
  const double offset = 0.1;
  komo.addObjective({0.2, 0.8}, FS_distance,
                    {"table", STRING(prefix << "pen_tip")}, OT_ineq, {1e0},
                    {-offset});

  // komo.addObjective({0.1, 0.8}, FS_distance,
  //                  {"table", STRING(prefix << "pen_tip")}, OT_sos, {1e1});

  // position
  // komo.addObjective({0}, FS_qItself, {}, OT_eq, {1e1}, path[0]);
  komo.addObjective({1., 1.}, FS_qItself, {}, OT_eq, {1e2}, scaled_path[-1]);
  komo.addObjective({0., 0.}, FS_qItself, {}, OT_eq, {1e2}, scaled_path[0]);

  // speed
  komo.addObjective({0.0, 0.01}, FS_qItself, {}, OT_eq, {1e0}, {},
                   1); // slow at beginning
  komo.addObjective({0.99, 1.}, FS_qItself, {}, OT_eq, {1e0}, {},
                    1); // slow at end

  // // acceleration
  komo.addObjective({0.0, 0.01}, FS_qItself, {}, OT_eq, {1e0}, {},
                   2); // slow at beginning
  komo.addObjective({0.99, 1.}, FS_qItself, {}, OT_eq, {1e0}, {},
                    2); // slow at end

  for (uint j = 0; j < num_timesteps; ++j) {
    // interpolate q
    //arr q = path[next_idx] - path[prev_idx];
    //const arr q = path[j*skip];
    const arr q = scaled_path[j];
    komo.setConfiguration(j, q);

    // komo.addObjective({1. * j/num_timesteps}, FS_qItself, {}, OT_sos, {1e1}, q);
  }

  // komo.addObjective({0.5}, FS_qItself, {}, OT_eq, {1e2}, scaled_path[int(num_timesteps / 2)]);

  spdlog::debug("running komo");
  komo.run_prepare(0.0);

  // std::cout << "A" <<std::endl;
  // std::cout << komo.getPath_q(-1) << std::endl;
  // std::cout << komo.getPath_q(-2) << std::endl;
  // std::cout << komo.getPath_q(0) << std::endl;

  // std::cout << "B" <<std::endl;
  // std::cout << komo.pathConfig.getJointStateSlice(2) << std::endl;
  // std::cout << scaled_path[0] << std::endl;

  // std::cout << "A" <<std::endl;

  // std::cout << komo.pathConfig.getJointStateSlice(num_timesteps) << std::endl;
  // std::cout << scaled_path[-1] << std::endl;

  // komo.pathConfig.watch(true);

  komo.run(options);
  spdlog::debug("done komo");

  const double ineq = komo.getReport(false).get<double>("ineq");
  const double eq = komo.getReport(false).get<double>("eq");

  spdlog::info("smoothing komo ineq: {} eq: {}", ineq, eq);

  if (eq > 2 || ineq > 2){
    // komo.getReport(true);
    // komo.pathConfig.watch(true);
    return {};
  }

  // komo.pathConfig.watch(true);

  // check if the path is actually feasible
  arr smooth(scaled_ts.N, path[0].N);
  for (uint j = 0; j < scaled_ts.N; ++j) {
    smooth[j] = komo.getPath_q(j);
  }
  
  // force boundary conditions to be true
  smooth[0] = scaled_path[0];
  smooth[-1] = scaled_path[-1];

  TimedPath tp_smooth(smooth, scaled_ts);
  arr unscaled_path = tp_smooth.resample(ts, C);

  spdlog::info("Done with smoothing");
  return unscaled_path;
}

