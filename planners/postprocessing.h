#pragma once

#include "spdlog/spdlog.h"

#include <Manip/timedPath.h>
#include <KOMO/komo.h>
#include <Geo/fclInterface.h>

#include <PlanningSubroutines/ConfigurationProblem.h>
#include <Kin/F_qFeatures.h>

#include "common/util.h"
#include "common/config.h"

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
      // delta(l) = start - end;
    }
  }

  arr p(j - i + 1, path.d1);

  for (uint l = 0; l < p.d0; ++l) {
    for (uint k = 0; k < path.d1; ++k) {
      if (std::find(short_ind.begin(), short_ind.end(), k) != short_ind.end()) {
        const double a = 1. * l / (j - i);
        p(l, k) = path(i, k) + a * (delta(k));

        if (periodicDimensions[k]){
          p(l, k) = std::fmod(p(l, k) + 3. * RAI_PI, 2 * RAI_PI) - RAI_PI;
        }

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
  // if (TP.A.prePlannedFrames.N != 0) {
  //   return initialPath;
  // }

  TP.C.fcl()->stopEarly = global_params.use_early_coll_check_stopping;

  arr smoothedPath = initialPath;
  /*for (uint i=0; i<smoothedPath.d0; i+=4){
    TP.query(smoothedPath[i], t0 + i);
    TP.C.watch(true);
  }*/

  // hack, since I didnt wanna move my projection method
  PathFinder_RRT_Time planner(TP);

  std::vector<double> costs;
  costs.push_back(path_length(TP.C, initialPath));

  const uint max_iter = 100;
  // const uint resolution = 2;
  const double resolution = 0.1;
  // const uint max_iter = 100;
  // const uint resolution = 5;
  for (uint k = 0; k < max_iter; ++k) {
    // choose random indices
    uint i, j;
    while (true) {
      i = rand() % initialPath.d0;
      j = rand() % initialPath.d0; 

      if (i > j) {
        std::swap(i, j);
      }
     
      if (j-i > 1 &&
          (TP.A.prePlannedFrames.N == 0 ||
           (TP.A.prePlannedFrames.N > 0 &&
            ((i >= TP.A.tPrePlanned - t0 && j >= TP.A.tPrePlanned - t0) ||
             (i <= TP.A.tPrePlanned - t0 && j <= TP.A.tPrePlanned - t0))))) {
        break;
      }
    }

    // choose which indices to shortcut
    std::vector<uint> ind;
    for (uint q = 0; q < smoothedPath.d1; ++q) {
      ind.push_back(q);

      // const double r = 1. * rand() / RAND_MAX;
      //if (r > 1. / smoothedPath.d1) {
        // ind.push_back(q);
      //}
    }

    // construct the new path
    auto p = constructShortcutPath(TP.C, smoothedPath, i, j, {});
    auto ps = constructShortcutPath(TP.C, smoothedPath, i, j, ind);
    
    if (TP.A.prePlannedFrames.N > 0){
      for (uint n=0; n<ps.d0; ++n){
        // project path
        if (t0 + i + n <= TP.A.tPrePlanned){
          const arr pProjected = planner.projectToManifold(ps[n], t0 + i + n);
          ps[n] = pProjected*1.;
        }

        // planner.TP.C.setJointState(ps[n]);
        // TP.query(ps[n], t0 + i + n);
        // if (TP.A.prePlannedFrames.N != 0) planner.TP.C.watch(true);
      }
    }

    const double len = path_length(TP.C, ps);

    // if the path length of the shortcut path is not shorter than the original one, don't consider it
    if (path_length(TP.C, p) <= len){
      continue;
    }

    // check if the new path is feasible (interpolate)
    // permute the indices that we check
    uintA q;
    q.setStraightPerm(j - i);
    q.permuteRandomly();

    // enable not checking everything here
    bool shortcutFeasible = true;
    for (const uint n : q) {
      const arr dir = ps[n + 1] - ps[n];
      const double dist = length(dir);
      const uint num_pts = uint(std::max(dist / resolution, 1.));
      for (uint l = 0; l < num_pts; ++l) {
        const double interp = corput(l);
        const arr point = ps[n] + interp * 1. * dir;
        const double t = t0 + i + n + 1. * interp;

        // std::cout << t << " " << point << std::endl;

        const auto qr = TP.query(point, t);

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
      for (uint n = 1; n < ps.d0; ++n) {
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
  if (A.prePlannedFrames.N != 0) {
    return path;
  }

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
  // options.stopIters = 50;
  options.stopIters = 10;
  options.damping = 10;
  // options.stopTolerance = 0.1;
  options.allowOverstep = false;
  // options.nonStrictSteps = 5;
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

  komo.add_qControlObjective({}, 2, 1e1);
  komo.add_qControlObjective({}, 1, 1e1);

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
  komo.addObjective({0.0, 0.00}, FS_qItself, {}, OT_eq, {1e0}, {},
                   1); // slow at beginning
  komo.addObjective({1., 1.}, FS_qItself, {}, OT_eq, {1e0}, {},
                    1); // slow at end

  // acceleration
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


Plan reoptimize_plan(rai::Configuration C,
                const Plan &unscaled_plan,
                const std::unordered_map<Robot, arr> &home_poses) {
  // formulate KOMO problem for multiple robots
  std::vector<Robot> all_robots;
  for (const auto &per_robot_plan : unscaled_plan) {
    all_robots.push_back(per_robot_plan.first);
  }

  for (const auto &element : all_robots) {
    std::cout << element << std::endl;
  }

  setActive(C, all_robots);

  rai::Animation A = make_animation_from_plan(unscaled_plan);

  setActive(C, all_robots);

  // extract complete trajectory
  std::cout << "extracting traj" << std::endl;
  arr smoothed_path(A.getT(), (unscaled_plan.begin()->second)[0].path.d1 *
                                  all_robots.size());
  for (uint i = 0; i < A.getT(); ++i) {
    uint offset = 0;
    for (uint j = 0; j < all_robots.size(); ++j) {
      const arr pose =
          get_robot_pose_at_time(i, all_robots[j], home_poses, unscaled_plan);
      for (uint k = 0; k < pose.N; ++k) {
        smoothed_path[i](k + offset) = pose(k);
      }
      offset += pose.N;
    }
  }

  // get joints per robot
  std::unordered_map<Robot, StringA> per_robot_joints;
  for (const auto &f : C.frames) {
    for (const auto &r : all_robots) {
      if (f->name.contains(STRING(r)) && f->joint) {
        per_robot_joints[r].append(f->name);
      }
    }
  }

  const uint horizon_length = 50;
  const uint total_length = A.getT();
  const uint step_size = 20;

  OptOptions options;
  options.stopIters = 10;
  // options.damping = 1e-3;
  // options.stopLineSteps = 5;

  KOMO komo;
  komo.setModel(C, true);
  komo.setTiming(1., horizon_length, 1, 2);
  komo.verbose = 0;
  komo.solver = rai::KS_sparse;

  for (uint i = 0; i < total_length - horizon_length; i += step_size) {
    std::cout << "iteration " << i << std::endl;
    komo.add_collision(true, .001, 1e1);
    komo.add_qControlObjective({}, 2, 1e1);
    komo.add_qControlObjective({}, 1, 1e1);

    // start constraint
    if (i > 0) {
      komo.setConfiguration(-2, smoothed_path[i - 2]);
      komo.setConfiguration(-1, smoothed_path[i - 1]);
    }

    const arr q0 = smoothed_path[i];
    komo.setConfiguration(0, q0);

    for (uint j = 0; j < horizon_length; ++j) {
      komo.setConfiguration(j, smoothed_path[i + j]);
    }

    // goal constraint
    const arr q_final = smoothed_path[i + horizon_length];

    komo.addObjective({1}, FS_qItself, {}, OT_eq, {1e1}, q_final);

    // add constraints for positions of actions
    // TODO: check if this is actually correct
    for (const auto &robot_tasks : unscaled_plan) {
      const auto r = robot_tasks.first;
      const auto tasks = robot_tasks.second;

      for (const auto &task : tasks) {
        const double task_end_time = task.t(0) + task.t.d0;
        if (task_end_time >= i && task_end_time < i + horizon_length) {
          const double scaled_time = 1. * (task_end_time - i) / horizon_length;
          const double constr_start_time =
              std::max(0., scaled_time - 0.5 / horizon_length);
          const double constr_end_time =
              std::min(1., scaled_time + 0.5 / horizon_length);

          std::cout << "Adding constraint at time " << task_end_time
                    << std::endl;
          std::cout << "for robot " << r << std::endl;
          std::cout << scaled_time << std::endl;
          std::cout << constr_start_time << " " << constr_end_time << std::endl;

          // position
          komo.addObjective({constr_start_time, constr_end_time},
                            make_shared<F_qItself>(F_qItself::byJointNames,
                                                   per_robot_joints[r],
                                                   komo.world),
                            {}, OT_eq, {1e1}, task.path[-1]);

          // velocity
          komo.addObjective({constr_start_time, constr_end_time},
                            make_shared<F_qItself>(F_qItself::byJointNames,
                                                   per_robot_joints[r],
                                                   komo.world),
                            {}, OT_eq, {1e1}, {}, 1);
        }
      }
    }

    // optimize
    komo.run_prepare(0.0, true);
    std::cout << "running" << std::endl;
    komo.run(options);
    std::cout << "done" << std::endl;

    // TODO: ensure that everything is collision free

    // get results from komo
    for (uint j = 0; j < horizon_length; ++j) {
      smoothed_path[i + j] = komo.getPath_q(j);
    }

    const double ineq = komo.getReport(false).get<double>("ineq");
    const double eq = komo.getReport(false).get<double>("eq");

    std::cout << ineq << " " << eq << std::endl;

    if (false) {
      std::cout << komo.getReport(true, 0) << std::endl;
      komo.pathConfig.watch(true);
    }

    komo.clearObjectives();
  }

  std::unordered_map<Robot, arr> per_robot_paths;
  for (const auto &r : all_robots) {
    per_robot_paths[r].resize(A.getT(), home_poses.at(r).d0);
  }

  for (uint i = 0; i < A.getT(); ++i) {
    C.setJointState(smoothed_path[i]);
    C.watch(false);
    rai::wait(0.01);

    for (const auto &r : all_robots) {
      setActive(C, r);
      const arr pose = C.getJointState();
      per_robot_paths[r][i] = pose;
    }

    setActive(C, all_robots);
  }

  Plan optimized_plan;

  for (const auto &per_robot_plan : unscaled_plan) {
    const auto robot = per_robot_plan.first;
    const auto tasks = per_robot_plan.second;

    for (const auto &task : tasks) {
      TaskPart new_task_part;

      new_task_part.t = task.t;
      new_task_part.r = task.r;
      new_task_part.task_index = task.task_index;
      new_task_part.algorithm = task.algorithm;
      new_task_part.name = task.name;
      new_task_part.is_exit = task.is_exit;

      new_task_part.path =
          per_robot_paths[task.r]({task.t(0), task.t(0) + task.t.d0 - 1});

      FrameL robot_frames;
      for (const auto &frame : task.anim.frameNames) {
        robot_frames.append(C[frame]);
      }
      setActive(C, robot);
      const auto anim_part =
          make_animation_part(C, new_task_part.path, robot_frames, task.t(0));
      new_task_part.anim = anim_part;

      optimized_plan[robot].push_back(new_task_part);
    }
  }

  return optimized_plan;
}
