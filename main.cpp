#include <KOMO/komo.h>
#include <Kin/F_operators.h>
#include <Kin/F_pose.h>
#include <Kin/F_qFeatures.h>
#include <Kin/featureSymbols.h>
#include <Kin/kin.h>

#include <Kin/kinViewer.h>
#include <Kin/viewer.h>

#include <Manip/rrt-time.h>
#include <PlanningSubroutines/ConfigurationProblem.h>

#include <iomanip>
#include <numeric>

#include <algorithm>
#include <chrono>
#include <random>

#include <math.h>

#include <GL/gl.h>
#include <Gui/opengl.h>

#include <PlanningSubroutines/Animation.h>
#include <PlanningSubroutines/ConfigurationProblem.h>

#include "line.h"

#include "util.h"
#include "env_util.h"
#include "stippling_util.h"
#include "path_util.h"

// TODO:
// - fix loading and visualization of previously computed paths
// - time-rescale path
// - split main planning subroutine
// - squeaky wheel planner
// - enable things that are not only 'go to point', e.g. drawing a line
// - enable multi-arm cooperation
// - enable search over sequences with precendence constraints
// - look into more complex motion planning:
// -- joint optimization
// -- constrained sampling based planning

arr sampleConfigurationForRobot(KOMO &komo, const arr &point,
                                const std::string &prefix, bool rnd = false,
                                bool ineq = false) {
  OptOptions options;
  options.stopIters = 100;
  options.damping = 1e-3;

  komo.setDiscreteOpt(1);

  // komo.world.stepSwift();

  komo.add_collision(true, .01, 1e1);
  komo.add_jointLimits(true, 0., 1e1);

  komo.addObjective({1.}, FS_position, {STRING(prefix << "pen_tip")}, OT_eq,
                    {1e2}, point);

  if (!ineq) {
    komo.addObjective({1.}, FS_vectorZ, {STRING(prefix << "pen")}, OT_sos,
                      {1e1}, {0., 0., -1.});
  } else {
    komo.addObjective({1.}, FS_scalarProductZZ,
                      {STRING(prefix << "pen"), "world"}, OT_ineq, {1e1},
                      {-cos(15 * 3.1415 / 180.)});
  }
  // komo.addObjective({1.}, FS_vectorZ, {STRING(prefix << "pen")}, OT_sos,
  // {1e1}, {0., 0., -1.}); komo.addObjective({1.}, FS_vectorZDiff,
  // {STRING(prefix << "pen"), "world"}, OT_ineq, {1e1}, {0., 0., -0.9});
  ConfigurationProblem cp(komo.world);
  setActive(cp.C, prefix);

  for (uint i = 0; i < 10; ++i) {
    if (rnd) {
      komo.run_prepare(0.1, false);
    } else {
      komo.run_prepare(0.0, true);
    }
    komo.run(options);

    const arr q = komo.getPath()[0]();
    // komo.pathConfig.watch(true);

    // ensure via sampling as well
    const bool res = cp.query(q)->isFeasible;

    if (res && komo.getReport(false).get<double>("ineq") < 1. &&
        // komo.getReport(false).get<double>("sos") < 0.5 &&
        komo.getReport(false).get<double>("eq") < 1.) {
      // komo.pathConfig.watch(true);
      return q;
    }
  }

  std::cout << "failed for pt " << point << std::endl;
  return {};
}

arr sampleConfigurationForRobot(rai::Configuration &C, const arr &point,
                                const std::string &prefix) {
  // activate agents
  setActive(C, prefix);

  OptOptions options;
  options.stopIters = 100;
  options.damping = 1e-3;

  KOMO komo;

  komo.verbose = 0;

  // set up komo problem
  komo.setModel(C, true);

  return sampleConfigurationForRobot(komo, point, prefix);
}

std::vector<arr> computeConfigurationsForPoints(const arr &pts,
                                                rai::Configuration &C,
                                                const std::string prefix) {
  std::vector<arr> configurations;

  const auto start = std::chrono::high_resolution_clock::now();

  // activate agents
  setActive(C, prefix);

  const auto home = C.getJointState();

  KOMO komo;
  komo.verbose = 0;
  komo.setModel(C, true);

  for (uint i = 0; i < pts.d0; ++i) {
    const arr pt = {pts[i](0), pts[i](1), 0.075};
    const arr q = sampleConfigurationForRobot(
        komo, C["table"]->getPosition() + pt, prefix);
    // const arr q = sampleConfigurationForRobot(C, C["table"]->getPosition() +
    // pt, prefix);
    configurations.push_back(q);

    // C.setJointState(q);
    //
    komo.clearObjectives();
    komo.world.setJointState(home);
  }
  C.setJointState(home);

  const auto stop = std::chrono::high_resolution_clock::now();
  const auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  std::cout << "Duration of IK pose computation: " << duration.count() / 1000.
            << " ms (per point: " << duration.count() / 1000. / pts.d0 << "ms )"
            << std::endl;

  return configurations;
}

double cost(const arr &p0, const arr &p1) {
  // return length(p0 - p1);
  return absMax(p0 - p1);
}

double fitness(const std::vector<uint> &order, const std::vector<arr> &qs) {
  double s = 0.;

  for (uint i = 0; i < order.size() - 1; ++i) {
    s += cost(qs[order[i]], qs[order[i + 1]]);
  }

  return s;
}

std::vector<uint> rndPerm(const std::vector<arr> &qs) {
  std::vector<uint> indices(qs.size());
  std::iota(std::begin(indices), std::end(indices), 0);

  double bestYet = fitness(indices, qs);
  auto bestOrderYet = indices;

  for (uint i = 0; i < 10000; ++i) {
    auto rng = std::default_random_engine{};
    std::shuffle(std::begin(indices), std::end(indices), rng);

    double c = fitness(indices, qs);

    if (c < bestYet) {
      bestYet = c;
      bestOrderYet = indices;
    }
  }

  return bestOrderYet;
}


std::vector<uint> reverseSubtour(const std::vector<uint> &indices,
                                 const uint start, const uint end) {
  std::vector<uint> tour = indices;

  for (uint i = 0; i <= end - start; ++i) {
    tour[start + i] = indices[end - i];
  }

  return tour;
}

std::vector<std::pair<uint, uint>>
reverseSubtour(const std::vector<std::pair<uint, uint>> &indices,
               const uint start, const uint end) {
  std::vector<std::pair<uint, uint>> tour = indices;

  for (uint i = 0; i <= end - start; ++i) {
    tour[start + i] = indices[end - i];
  }

  return tour;
}

std::vector<uint> alterTour(const std::vector<uint> &indices,
                            const std::vector<arr> qs,
                            const uint nmax = 10000) {
  std::vector<uint> bestTourYet = indices;
  double bestCostYet = fitness(indices, qs);

  for (uint i = 0; i < nmax; ++i) {
    uint j = rand() % indices.size();
    uint k = rand() % indices.size();
    if (j > k) {
      std::swap(j, k);
    }

    auto tmp = reverseSubtour(bestTourYet, j, k);
    double c = fitness(tmp, qs);

    if (c < bestCostYet) {
      bestCostYet = c;
      bestTourYet = tmp;
    }
  }

  return bestTourYet;
}

void drawPts(rai::Configuration C, arr pts, arr color = {0., 0., 0., 1.}) {
  for (uint i = 0; i < pts.d0; ++i) {
    const arr pt = {pts[i](0), pts[i](1), 0.075};

    auto *dot = C.addFrame("goal", "table");
    dot->setShape(rai::ST_sphere, {0.005});
    dot->setRelativePosition({pts[i](0), pts[i](1), 0.05});
    dot->setContact(0.);
    dot->setColor(color);
  }

  C.gl()->displayCamera().setPosition(0, 0., 3);
  C.gl()->displayCamera().focusOrigin();

  C.watch(true);
}

// overloaded for both
void drawPts(rai::Configuration C, std::map<uint, arr> tmp,
             arr color = {0., 0., 0., 1.}) {
  for (auto element : tmp) {
    uint j = element.first;
    arr pts = element.second;

    for (uint i = 0; i < pts.d0; ++i) {
      const arr pt = {pts[i](0), pts[i](1), 0.075};

      auto *dot = C.addFrame("goal", "table");
      dot->setShape(rai::ST_sphere, {0.01});
      dot->setRelativePosition({pts[i](0), pts[i](1), 0.05});
      dot->setContact(0.);
      dot->setColor({1. * j, 1. * j, 1. * j, 1.});
    }
  }

  C.watch(true);
}

typedef std::vector<arr> TaskPoses;
typedef std::map<std::string, std::vector<TaskPoses>> RobotTaskPoseMap;

typedef std::string Robot;
typedef std::pair<Robot, int> robot_task_pair;
typedef std::vector<robot_task_pair> TaskSequence;

TaskSequence generate_random_sequence(const std::vector<Robot> &robots,
                                      const uint num_tasks) {
  TaskSequence seq;

  for (uint i = 0; i < num_tasks; ++i) {
    // sample robot
    const uint r = rand() % robots.size();

    // make pair
    seq.push_back(std::make_pair(robots[r], i));
  }

  auto rng = std::default_random_engine{
      std::chrono::system_clock::now().time_since_epoch().count()};
  std::shuffle(std::begin(seq), std::end(seq), rng);

  return seq;
}

TaskSequence generate_single_arm_sequence(const std::vector<Robot> &robots,
                                          const uint num_tasks) {
  // sample robot _once_.
  const uint r = rand() % robots.size();

  TaskSequence seq;
  for (uint i = 0; i < num_tasks; ++i) {
    // make pair
    seq.push_back(std::make_pair(robots[r], i));
  }

  auto rng = std::default_random_engine{
      std::chrono::system_clock::now().time_since_epoch().count()};
  std::shuffle(std::begin(seq), std::end(seq), rng);

  return seq;
}

TaskSequence
generate_alternating_random_sequence(const std::vector<Robot> &robots,
                                     const uint num_tasks,
                                     const RobotTaskPoseMap &rtpm) {
  uint r = rand() % robots.size();

  auto available_tasks = straightPerm(num_tasks);
  TaskSequence seq;
  while (available_tasks.size() > 0) {
    for (uint j = 0; j < 10; ++j) {
      const uint task_index = available_tasks[rand() % available_tasks.size()];

      // check if the task is feasible with the chosen robot
      if (rtpm.at(robots[r])[task_index].size() != 0) {
        seq.push_back(std::make_pair(robots[r], task_index));

        available_tasks.erase(std::remove(available_tasks.begin(),
                                          available_tasks.end(), task_index),
                              available_tasks.end());

        break;
      }
    }
    r = (r + 1) % robots.size();
  }

  return seq;
}

TaskSequence generate_alternating_greedy_sequence(
    const std::vector<Robot> &robots, const uint num_tasks,
    const RobotTaskPoseMap &rtpm, const std::map<Robot, arr> &home_poses) {
  std::cout << "Generating alternating greedy" << std::endl;
  auto available_tasks = straightPerm(num_tasks);

  // sample starting_ robot.
  uint r = rand() % robots.size();
  std::map<Robot, arr> poses = home_poses;

  TaskSequence seq;
  while (available_tasks.size() > 0) {
    // find minimum dist pose to current robot
    auto min_dist = 1e6;
    uint task_index = 0;
    bool assigned_task = false;
    for (auto t : available_tasks) {
      if (rtpm.at(robots[r])[t].size() != 0) {
        const auto dist = absMax(poses[robots[r]] - rtpm.at(robots[r])[t][0]);
        if (dist < min_dist) {
          task_index = t;
          min_dist = dist;
          assigned_task = true;
        }
      }
    }

    if (assigned_task) {
      // remove task from available tasks
      available_tasks.erase(std::remove(available_tasks.begin(),
                                        available_tasks.end(), task_index),
                            available_tasks.end());

      // make pair
      std::cout << "adding " << robots[r] << " with index " << r << std::endl;
      seq.push_back(std::make_pair(robots[r], task_index));
    }

    r = (r + 1) % robots.size();
  }

  return seq;
}

// this is the solution of one task
struct TaskPart {
  bool has_solution = false;

  TaskPart(const arr &_t, const arr &_path)
      : has_solution(true), t(_t), path(_path){};
  TaskPart(){};

  rai::Animation::AnimationPart anim;

  arr t;
  arr path;

  Robot r;
  uint task_index;

  std::string name;
  std::string algorithm;

  bool is_exit = false;
};

//#define VMAX 0.1;
const double VMAX = 0.05;

arr plan_with_komo_given_horizon(const rai::Animation &A, rai::Configuration &C,
                                 const arr &q0, const arr &q1, const arr &ts,
                                 const std::string prefix, double &ineq,
                                 double &eq) {
  OptOptions options;
  options.stopIters = 100;
  options.damping = 1e-3;
  options.stopLineSteps = 5;

  std::cout << "setting up komo" << std::endl;
  KOMO komo;
  komo.setModel(C, true);
  komo.setTiming(1., ts.N, 5, 2);
  komo.verbose = 0;
  komo.solver = rai::KS_sparse;

  komo.add_collision(true, .001, 1e1);
  komo.add_qControlObjective({}, 2, 1e1);
  komo.add_qControlObjective({}, 1, 1e1);

  komo.setConfiguration(-2, q0);
  komo.setConfiguration(-1, q0);
  komo.setConfiguration(0, q0);

  // make pen tip go a way from the table
  const double offset = 0.06;
  komo.addObjective({0.1, 0.9}, FS_distance,
                    {"table", STRING(prefix << "pen_tip")}, OT_ineq, {1e1},
                    {-offset});
  // komo.addObjective({0.1, 0.8}, FS_distance,
  //                  {"table", STRING(prefix << "pen_tip")}, OT_sos, {1e1});

  // position
  // komo.addObjective({0}, FS_qItself, {}, OT_eq, {1e1}, q0);
  komo.addObjective({1}, FS_qItself, {}, OT_eq, {1e2}, q1);

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

  if (false) {
    const arr v_constr = q0 * 0. + VMAX;
    komo.addObjective({0.0, 1.}, FS_qItself, {}, OT_ineq, {1e0}, {v_constr},
                      1); // slow at beginning
    komo.addObjective({0.0, 1.}, FS_qItself, {}, OT_ineq, {-1e0}, {-v_constr},
                      1); // slow at beginning
  }

  setKomoToAnimation(komo, C, A, ts);

  std::cout << "running komo" << std::endl;
  komo.run_prepare(0.01);
  komo.run(options);
  std::cout << "done komo" << std::endl;

  /*if (komo.getReport(false).get<double>("ineq") > 1. ||
  komo.getReport(false).get<double>("eq") > 1.){ std::cout << "infeasible komo
  sol, ineq: " << komo.getReport(false).get<double>("ineq")
      << " eq. " << komo.getReport(false).get<double>("eq") << std::endl;
    return {};
  }*/

  ineq = komo.getReport(false).get<double>("ineq");
  eq = komo.getReport(false).get<double>("eq");

  // if (eq > 2 || ineq > 2){
  komo.getReport(true);
  // komo.pathConfig.watch(true);
  //}

  // komo.pathConfig.watch(true);

  // check if the path is actually feasible
  arr path(ts.N, q0.N);
  for (uint j = 0; j < ts.N; ++j) {
    path[j] = komo.getPath_q(j);
  }

  if (length(path[0] - q0) > 1e-3) {
    std::cout << length(path[0] - q0) << std::endl;
  }

  if (length(path[-1] - q1) > 1e-3) {
    std::cout << length(path[-1] - q1) << std::endl;
  }

  path[0] = q0;
  path[-1] = q1;

  return path;
}

double get_max_speed(const arr &path) {
  double max_speed = 0;
  for (uint i = 0; i < path.d0 - 1; ++i) {
    max_speed = std::max({max_speed, absMax(path[i] - path[i + 1])});
  }

  return max_speed;
}

double get_earliest_feasible_time(TimedConfigurationProblem &TP, const arr &q,
                                  const uint t_max, const uint t_min) {
  uint t_earliest_feas = t_max;
  while (t_earliest_feas > t_min) {
    const auto res = TP.query(q, t_earliest_feas);
    if (!res->isFeasible) {
      t_earliest_feas += 2;
      break;
    }
    --t_earliest_feas;
  }

  if (t_max == t_earliest_feas + 2) {
    std::cout << "C" << std::endl;
    std::cout << "C" << std::endl;
    std::cout << "C" << std::endl;
  }

  return t_earliest_feas;
}

TaskPart plan_in_animation_komo(const rai::Animation &A, rai::Configuration &C,
                                const uint t0, const arr &q0, const arr &q1,
                                const uint time_lb, const std::string prefix,
                                const int time_ub_prev_found = -1) {
  TimedConfigurationProblem TP(C, A);

  // Check if start q is feasible
  const auto start_res = TP.query(q0, t0);
  if (!start_res->isFeasible && min(start_res->coll_y) < -0.1) {
    std::cout << t0 << std::endl;
    LOG(-1) << "q_start is not feasible! This should not happen ";
    start_res->writeDetails(cout, C);
    std::cout << "colliding by " << min(start_res->coll_y) << std::endl;

    // TP.A.setToTime(TP.C, t0);
    // TP.C.setJointState(q0);
    // TP.C.watch(true);

    return TaskPart();
  }

  const uint dt_max_vel = uint(std::ceil(absMax(q0 - q1) / VMAX));

  // the goal should always be free at the end of the animation, as we always
  // plan an exit path but it can be the case that we are currently planning an
  // exit path, thus we include the others
  const uint t_max_to_check = std::max({time_lb, t0 + dt_max_vel, A.getT()});
  // establish time at which the goal is free, and stays free
  const double t_earliest_feas = get_earliest_feasible_time(
      TP, q1, t_max_to_check, std::max({time_lb, t0 + dt_max_vel}));

  std::cout << "Final time for komo: " << t_earliest_feas
            << " dt: " << t_earliest_feas - t0 << std::endl;

  // the minimum horizon length is mainly given by the earliest possible finish
  // time
  const uint min_horizon_length =
      std::max({5u, dt_max_vel, uint(t_earliest_feas - t0)});
  const uint max_horizon_length =
      std::max({25u, time_lb - t0, (A.getT() > t0) ? A.getT() - t0 : 0u});

  // uint horizon = std::ceil((min_horizon_length + max_horizon_length)/2.);
  uint horizon = std::ceil(min_horizon_length);

  const uint max_komo_run_attempts = 5;
  uint iters = 0;
  while (true) {
    std::cout << "komo horizon: " << horizon << std::endl;
    arr ts(horizon);
    for (uint j = 0; j < horizon; ++j) {
      ts(j) = t0 + j;
    }

    if (time_ub_prev_found > 0 && time_ub_prev_found < ts(-1)) {
      std::cout << "found cheaper path before" << std::endl;
      return TaskPart();
    }

    // ensure that the goal is truly free. Sanity check.
    const auto res = TP.query(q1, t0 + horizon);
    if (!res->isFeasible) {
      std::cout << "Q" << std::endl;
      std::cout << "Q" << std::endl;
      std::cout << "Q" << std::endl;
      std::cout << "Q" << std::endl;
      std::cout << "Q" << std::endl;
      res->writeDetails(cout, C);
      // TP.A.setToTime(TP.C, t0 + horizon);
      // TP.C.setJointState(q1);
      // TP.C.watch(true);

      return TaskPart();
    }

    if (false) {
      TP.A.setToTime(TP.C, t0);
      TP.C.setJointState(q0);
      TP.C.watch(true);

      TP.A.setToTime(TP.C, t0 + horizon);
      TP.C.setJointState(q1);
      TP.C.watch(true);
    }

    // set up komo problem
    double ineq = 0;
    double eq = 0;
    const arr path =
        plan_with_komo_given_horizon(A, C, q0, q1, ts, prefix, ineq, eq);

    if (false) {
      for (uint i = 0; i < ts.N; ++i) {
        TP.A.setToTime(TP.C, ts(i));
        TP.C.setJointState(path[i]);
        TP.C.watch(true);
      }
    }

    if (false) {
      for (uint i = 0; i < ts.N; ++i) {
        const auto res = TP.query(path[i], ts(i));
        if (!res->isFeasible) {
          std::cout << "R " << ts(i) << std::endl;
          TP.A.setToTime(TP.C, ts(i));
          TP.C.setJointState(path[i]);
          TP.C.watch(true);
        }
      }
    }

    // check max. speed
    const double max_speed = get_max_speed(path);
    std::cout << "max speed: " << max_speed << std::endl;

    if (eq > 15 && ineq > 15) {
      return TaskPart();
    }

    if (max_speed < VMAX && eq < 1.5 && ineq < 1.5) {
      std::cout << "done, accepting komo. ineq: " << ineq << " eq. " << eq
                << std::endl;
      return TaskPart(ts, path);
    }

    if (iters >= max_komo_run_attempts) {
      std::cout << "too many reruns. ineq: " << ineq << " eq. " << eq
                << std::endl;
      return TaskPart();
    }

    // const uint num_add_timesteps = std::max({uint((iters+1)*2), uint(req_t -
    // ts.N)}); const uint num_add_timesteps = std::ceil(1. *
    // (max_horizon_length
    // - min_horizon_length) / max_komo_run_attempts);
    const int add_timesteps_for_speed =
        std::max({0, int(std::ceil(horizon * ((max_speed / VMAX) - 1)))});
    const uint num_add_timesteps =
        std::max({(iters + 1) * 3u, uint(add_timesteps_for_speed)});
    std::cout << "adding " << num_add_timesteps << " steps" << std::endl;

    horizon += num_add_timesteps;

    std::cout << "rerunning komo; ineq: " << ineq << " eq: " << eq << std::endl;

    ++iters;
  }
}

TaskPart plan_in_animation_rrt(const rai::Animation &A, rai::Configuration &C,
                               const uint t0, const arr &q0, const arr &q1,
                               const uint time_lb, const std::string prefix,
                               int time_ub_prev_found = -1) {
  TimedConfigurationProblem TP(C, A);
  TP.activeOnly = true;

  // Check if start q is feasible
  const auto start_res = TP.query(q0, t0);
  if (!start_res->isFeasible) {
    std::cout << t0 << std::endl;
    LOG(-1) << "q_start is not feasible! This should not happen ";
    start_res->writeDetails(cout, C);
    std::cout << "colliding by " << min(start_res->coll_y) << std::endl;

    // TP.A.setToTime(TP.C, t0);
    // TP.C.setJointState(q0);
    // TP.C.watch(true);

    return TaskPart();
  }

  PathFinder_RRT_Time planner(TP);
  planner.vmax = VMAX;
  planner.lambda = 0.5;
  // planner.disp = true;
  // planner.optimize = optimize;
  // planner.step_time = 5;
  planner.maxIter = 500;
  planner.goalSampleProbability = 0.9; // 0.9

  const uint dt_max_vel = uint(std::ceil(absMax(q0 - q1) / VMAX));

  // the goal should always be free at the end of the animation, as we always
  // plan an exit path but it can be the case that we are currently planning an
  // exit path, thus we include the others
  const uint t_max_to_check = std::max({time_lb, t0 + dt_max_vel, A.getT()});
  // establish time at which the goal is free, and stays free
  const uint t_earliest_feas = get_earliest_feasible_time(
      TP, q1, t_max_to_check, std::max({time_lb, t0 + dt_max_vel}));

  std::cout << "t_earliest_feas " << t_earliest_feas << std::endl;
  std::cout << "last anim time " << A.getT() << std::endl;

  if (false) {
    TP.A.setToTime(TP.C, A.getT());
    TP.C.setJointState(q1);
    TP.C.watch(true);
  }

  // run once without upper bound
  // auto res_check_feasibility = planner.plan(q0, t0, q1, t_earliest_feas);

  const uint max_delta = 7;
  const uint max_iter = 12;
  TimedPath timedPath({}, {});
  for (uint i = 0; i < max_iter; ++i) {
    const uint time_ub = t_earliest_feas + max_delta * (i);

    std::cout << i << " " << time_ub << std::endl;
    if (time_ub > time_ub_prev_found && time_ub_prev_found > 0) {
      std::cout << "Aborting bc. faster path found" << std::endl;
      break;
    }

    auto res = planner.plan(q0, t0, q1, t_earliest_feas, time_ub);

    if (res.time.N != 0) {
      timedPath = res;
      break;
    }
  }

  if (timedPath.time.N == 0) {
    /*TP.A.setToTime(TP.C, t0);
    TP.C.setJointState(q0);
    TP.C.watch(true);

    TP.A.setToTime(TP.C, t_earliest_feas);
    TP.C.setJointState(q1);
    TP.C.watch(true);

    for (uint i=0; i<A.getT(); ++i){
      std::cout << i << std::endl;
      TP.A.setToTime(TP.C, i);
      TP.C.setJointState(q1);
      TP.C.watch(true);
    }*/
    return TaskPart();
  }

  // resample
  const uint N = std::ceil(timedPath.time(timedPath.time.N - 1) - t0) + 1;
  arr t(N);
  for (uint i = 0; i < t.N; ++i) {
    t(i) = i + t0;
  }

  const arr path = timedPath.resample(t, TP.C);

  // check if resampled path is still fine
  for (uint i = 0; i < t.N; ++i) {
    const auto res = TP.query(path[i], t(i));
    if (!res->isFeasible) {
      LOG(-1) << "resampled path is not feasible! This should not happen.";
      start_res->writeDetails(cout, C);

      // TP.A.setToTime(TP.C, t0);
      // TP.C.setJointState(q0);
      // TP.C.watch(true);

      // return TaskPart();
    }
  }

  // shortcutting
  const arr new_path = partial_shortcut(TP, path, t0);

  // std::cout << new_path[-1] << "\n" << q1 << std::endl;

  // smoothing and imposing bunch of constraints
  std::cout << "smoothing rrt path using komo" << std::endl;
  arr smooth_path = smoothing(A, C, t, new_path, prefix);

  for (uint i = 0; i < smooth_path.d0; ++i) {
    const auto res = TP.query(smooth_path[i], t(i));
    if (!res->isFeasible && res->coll_y.N > 0 && min(res->coll_y) < -0.01) {
      std::cout << "smoothed path infeasible" << std::endl;
      smooth_path = new_path;
      break;
    }
  }

  // const arr smooth_path = new_path;

  const double max_speed = get_max_speed(smooth_path);
  std::cout << "rrt final time " << t(-1) << std::endl;
  std::cout << "rrt max_speed " << max_speed << std::endl;

  return TaskPart(t, smooth_path);
}

// robust 'time-optimal' planning method
TaskPart plan_in_animation(const rai::Animation &A, rai::Configuration &C,
                           const uint t0, const arr &q0, const arr &q1,
                           const uint time_lb, const std::string prefix,
                           const bool exit_path) {
  // run rrt
  TaskPart rrt_path = plan_in_animation_rrt(A, C, t0, q0, q1, time_lb, prefix);
  rrt_path.algorithm = "rrt";

  /*if(rrt_path.has_solution){
    TimedConfigurationProblem TP(C, A);
    for (uint i = 0; i < rrt_path.t.N; ++i) {
      const auto res = TP.query(rrt_path.path[i], rrt_path.t(i));
      if(res->coll_y.N > 0) std::cout <<min(res->coll_y) << std::endl;
      if (!res->isFeasible){
        std::cout << "B" << std::endl;
        std::cout << "B" << std::endl;
        std::cout << "B" << std::endl;
      }
      if (!res->isFeasible && min(res->coll_y) < -0.01) {
        std::cout << "rrt actually infeasible" << std::endl;
        rrt_path.has_solution = false;
        break;
      }
    }
  }*/

  int time_ub = -1;
  if (rrt_path.has_solution) {
    time_ub = rrt_path.t(-1);
  }

  // attempt komo
  TaskPart komo_path =
      plan_in_animation_komo(A, C, t0, q0, q1, time_lb, prefix, time_ub);
  komo_path.algorithm = "komo";

  /*if(komo_path.has_solution){
    return komo_path;
  }*/

  // ensure that the komo-path does not run into start configurations of future
  // tasks
  if (komo_path.has_solution) {
    TimedConfigurationProblem TP(C, A);
    for (uint i = 0; i < komo_path.t.N; ++i) {
      const auto res = TP.query(komo_path.path[i], komo_path.t(i));
      if (res->coll_y.N > 0)
        std::cout << min(res->coll_y) << std::endl;
      if (!res->isFeasible && min(res->coll_y) < -0.01) {
        std::cout << "komo actually infeasible" << std::endl;
        komo_path.has_solution = false;
        break;
      }
    }
  }

  if (komo_path.has_solution && !rrt_path.has_solution) {
    std::cout << "using komo" << std::endl;
    return komo_path;
  }
  if (!komo_path.has_solution && rrt_path.has_solution) {
    std::cout << "using rrt" << std::endl;
    return rrt_path;
  }

  if (komo_path.has_solution && rrt_path.has_solution &&
      komo_path.t(-1) < rrt_path.t(-1)) {
    std::cout << "using komo" << std::endl;
    return komo_path;
  }

  std::cout << "using rrt" << std::endl;
  return rrt_path;
}

rai::Animation::AnimationPart make_animation_part(rai::Configuration &C,
                                                  const arr &path,
                                                  const FrameL &frames,
                                                  const uint t_start) {
  rai::Animation::AnimationPart anim;

  StringA frameNames;
  for (auto f : frames) {
    frameNames.append(f->name);
  }

  anim.start = t_start;
  anim.frameIDs = framesToIndices(frames);
  anim.frameNames = frameNames;

  const uint dt = path.d0;
  anim.X.resize(dt, frames.N, 7);

  arr q;
  for (uint i = 0; i < path.d0; ++i) {
    q = path[i];
    C.setJointState(q);
    // C.watch(true);
    anim.X[i] = C.getFrameState(frames);
  }
  return anim;
}

FrameL get_robot_frames(rai::Configuration &C, const Robot &robot) {
  FrameL frames;

  const rai::String base STRING(robot << "base");
  FrameL roots = C.getFrames(C.getFrameIDs({base}));
  for (auto *f : roots) {
    f->getSubtree(frames);
  }
  frames.append(roots);

  return frames;
}

typedef std::map<Robot, std::vector<TaskPart>> Plan;

enum class PlanStatus { failed, aborted, success, unplanned };

struct PlanResult {
  PlanResult() : status(PlanStatus::unplanned) {}
  PlanResult(PlanStatus _status) : status(_status){};
  PlanResult(PlanStatus _status, const Plan &_plan)
      : status(_status), plan(_plan){};

  PlanStatus status;
  Plan plan;
};

/*PlanStatus plan_task(rai::Configuration &CPlanner, const robot_task_pair &rtp,
    const RobotTaskPoseMap &rtpm,
    const std::map<Robot, FrameL> robot_frames, const uint best_makespan_so_far,
    const std::map<Robot, arr> &home_poses, const uint prev_finishing_time, Plan
&paths){
  // set robots to home pose
  for (const auto &r : home_poses) {
    setActive(CPlanner, r.first);
    CPlanner.setJointState(r.second);
  }

  // plan for current goal
  const Robot robot = rtp.first;
  const uint task = rtp.second;

  // remove exit path
  if (paths[robot].size() > 0 &&
      prev_finishing_time < paths[robot].back().t(-1) + 1 + 25) {
    paths[robot].pop_back();
  }

  arr start_pose;
  uint start_time;

  if (paths[robot].size() > 0) {
    start_pose = paths[robot].back().path[-1];
    start_time = paths[robot].back().t(-1) + 1;
  } else {
    start_pose = home_poses.at(robot);
    start_time = 0;
  }

  std::cout << "start time " << start_time << std::endl;

  const arr goal_pose = rtpm.at(robot)[task][0];

  const uint time_lb = std::max({prev_finishing_time, start_time});

  std::cout << "lb " << time_lb << std::endl;

  if (time_lb > start_time && time_lb - start_time > 25) {
    start_time = time_lb - 25;
  }

  // make animation from path-parts
  rai::Animation A;
  for (const auto &p : paths) {
    for (const auto path : p.second) {
      A.A.append(path.anim);
    }
  }

  if (false) {
    for (uint i = 0; i < prev_finishing_time; ++i) {
      A.setToTime(CPlanner, i);
      CPlanner.watch(false);
      rai::wait(0.1);
    }
  }

  // set configuration to plannable for current robot
  std::cout << "setting up C" << std::endl;
  setActive(CPlanner, robot);

  auto path = plan_in_animation(A, CPlanner, start_time, start_pose,
                                goal_pose, time_lb, robot, true);

  path.r = robot;
  path.task_index = task;
  path.name = "task";

  if (path.has_solution) {
    if (false) {
      for (uint i = 0; i < path.path.d0; ++i) {
        auto q = path.path[i];
        CPlanner.setJointState(q);
        CPlanner.watch(false);
        rai::wait(0.01);
      }
      CPlanner.setJointState(home_poses.at(robot));
    }
    // make animation part
    const auto anim_part = make_animation_part(
        CPlanner, path.path, robot_frames.at(robot), start_time);
    path.anim = anim_part;

    if (prev_finishing_time > best_makespan_so_far) {
      std::cout << "Stopping early due to better prev. path. ("
                << best_makespan_so_far << ")" << std::endl;
      return PlanStatus::aborted;
    }

    paths[robot].push_back(path);
  } else {
    std::cout << "Was not able to find a path" << std::endl;
    return PlanStatus::failed;
  }

  const uint exit_start_time = path.t(-1) + 1;

  auto exit_path =
      plan_in_animation(A, CPlanner, exit_start_time, goal_pose,
                        home_poses.at(robot), exit_start_time, robot, true);
  exit_path.r = robot;
  exit_path.task_index = task;
  exit_path.is_exit = true;
  exit_path.name = "exit";

  if (exit_path.has_solution) {
    const auto exit_anim_part = make_animation_part(
        CPlanner, exit_path.path, robot_frames.at(robot), exit_start_time);
    exit_path.anim = exit_anim_part;
    paths[robot].push_back(exit_path);
  } else {
    std::cout << "Was not able to find an exit path" << std::endl;
    return PlanStatus::failed;
  }

  return PlanStatus::success;
}*/

PlanResult plan_multiple_arms_given_subsequence_and_prev_plan(
    rai::Configuration C, const RobotTaskPoseMap &rtpm,
    const TaskSequence &sequence, const uint start_index, const Plan prev_paths,
    const std::map<Robot, arr> &home_poses,
    const uint best_makespan_so_far = 1e6) {
  rai::Configuration CPlanner = C;
  // C.watch(true);

  // prepare planning-configuration
  delete_unnecessary_frames(CPlanner);

  // CPlanner.watch(true);

  std::map<Robot, FrameL> robot_frames;
  for (auto r : home_poses) {
    const auto robot = r.first;
    robot_frames[robot] = get_robot_frames(CPlanner, robot);
  }

  // remove things from paths
  std::vector<uint> unplanned_tasks;
  for (uint i = start_index; i < sequence.size(); ++i) {
    unplanned_tasks.push_back(sequence[i].second);
  }

  std::map<Robot, std::vector<TaskPart>> paths;

  for (const auto p : prev_paths) {
    const auto r = p.first;
    for (auto plan : p.second) {
      if (std::find(unplanned_tasks.begin(), unplanned_tasks.end(),
                    plan.task_index) == unplanned_tasks.end()) {
        paths[r].push_back(plan);
        std::cout << "adding (" << r << " " << plan.task_index << ")"
                  << std::endl;
      }
    }
  }

  // figure out which robots need an exit path
  std::vector<std::pair<Robot, uint>> robot_exit_paths;
  for (const auto p : paths) {
    const auto robot = p.first;
    // do not plan an exit path if
    // -- ther is already one
    // -- there is no other path
    // -- we are planning for this robot next
    if (p.second.size() > 0 && !paths[robot].back().is_exit) {
      robot_exit_paths.push_back({robot, paths[robot].back().t(-1)});
    }
  }

  std::sort(robot_exit_paths.begin(), robot_exit_paths.end(),
            [](auto &left, auto &right) { return left.second < right.second; });

  for (const auto p : robot_exit_paths) {
    const auto robot = p.first;
    // do not plan an exit path if
    // -- ther is already one
    // -- there is no other path
    // -- we are planning for this robot next

    std::cout << "adding exit path for " << robot << std::endl;
    // plan exit path for robot
    rai::Animation A;
    for (const auto &p2 : paths) {
      for (const auto path2 : p2.second) {
        A.A.append(path2.anim);
      }
    }

    std::cout << "start time:" << p.second << std::endl;
    setActive(CPlanner, robot);
    auto exit_path =
        plan_in_animation(A, CPlanner, p.second, paths[robot].back().path[-1],
                          home_poses.at(robot), p.second + 5, robot, true);
    exit_path.r = robot;
    exit_path.task_index = paths[robot].back().task_index;
    exit_path.is_exit = true;
    exit_path.name = "exit";

    /*for (uint i=0; i<exit_path.t.N; ++i){
      std::cout << exit_path.path[i] << std::endl;
      CPlanner.setJointState(exit_path.path[i]);
      CPlanner.watch(true);
    }*/

    if (exit_path.has_solution) {
      std::cout << "adding path for " << robot << " at time " << p.second
                << std::endl;
      const auto exit_anim_part = make_animation_part(
          CPlanner, exit_path.path, robot_frames[robot], p.second);
      exit_path.anim = exit_anim_part;
      paths[robot].push_back(exit_path);
    } else {
      std::cout << "Was not able to find an exit path" << std::endl;
      return PlanResult(PlanStatus::failed);
    }
    /*{
      rai::Animation An;
      for (const auto &p2 : paths) {
        for (const auto path2 : p2.second) {
          An.A.append(path2.anim);
        }
      }
      An.play(CPlanner, false);
    }*/
  }

  // actually plan
  for (uint i = start_index; i < sequence.size(); ++i) {
    uint prev_finishing_time = 0;

    for (const auto &p : paths) {
      const auto plans = p.second;
      if (plans.size() > 0) {
        if (plans.back().is_exit) {
          // if a path is an exit, we are looking at the maximum non-exit time
          prev_finishing_time = std::max(
              {uint(plans[plans.size() - 2].t(-1)), prev_finishing_time});
        } else {
          // else, we are looking at the final current time
          prev_finishing_time = std::max(
              {uint(plans[plans.size() - 1].t(-1)), prev_finishing_time});
        }
      }
    }

    /*
    const auto res = plan_task(CPlanner, sequence[i], rtpm, robot_frames,
    best_makespan_so_far, home_poses, prev_finishing_time, paths);

    if (res != PlanStatus::success){
      return PlanResult(res);
    }*/

    // set robots to home pose
    for (const auto &r : home_poses) {
      setActive(CPlanner, r.first);
      CPlanner.setJointState(r.second);
    }

    // plan for current goal
    const Robot robot = sequence[i].first;
    const uint task = sequence[i].second;
    std::cout << "planning task " << task << " for robot " << robot << " as "
              << i << " th task" << std::endl;

    bool is_bin_picking = false;
    if (rtpm.at(robot)[task].size() > 1) {
      is_bin_picking = true;
    }

    // remove exit path
    bool removed_exit_path = false;
    const uint max_start_time_shift = 35 * rtpm.at(robot)[task].size();
    if (paths[robot].size() > 0 && paths[robot].back().is_exit &&
        prev_finishing_time <
            paths[robot].back().t(-1) + 1 + max_start_time_shift) {
#if 0
      // partial removal of the exit path
      const uint max_time_diff = max_start_time_shift;
      if (prev_finishing_time > max_time_diff && prev_finishing_time - paths[robot].back().t(0) + 1 > max_time_diff){
        uint del_index = 0;
        for (uint i=0; i<paths[robot].back().t.N; ++i){
          if (paths[robot].back().t(i) == prev_finishing_time - max_time_diff){
            del_index = i;
            break;
          }
        }
        std::cout << del_index << std::endl;
        if (del_index > 0){
          std::cout << "deleting parts of the prev. exit path" << std::endl;
          std::cout << paths[robot].back().t << std::endl;

          const uint n = paths[robot].back().t.N-del_index;
          {
            arr tmp;
            tmp.resize(del_index,paths[robot].back().anim.X.d1, 7);
            for (uint i=0; i<del_index; ++i){
              tmp[i] = paths[robot].back().anim.X[i];
            }
            paths[robot].back().anim.X = tmp;
          }
          {
            arr tmp;
            tmp.resize(del_index);
            for (uint i=0; i<del_index; ++i){
              tmp(i) = paths[robot].back().t(i);
            }
            paths[robot].back().t = tmp;
          }
          {
            arr tmp;
            tmp.resize(del_index, paths[robot].back().path.d1);
            for (uint i=0; i<del_index; ++i){
              tmp[i] = paths[robot].back().path[i];
            }
            paths[robot].back().path = tmp;
          }

          std::cout << paths[robot].back().t << std::endl;
        }
      }
      else{
        std::cout << "removing exit path of " << robot << std::endl;
        std::cout << "exit path end time: " << paths[robot].back().t(-1) << std::endl;
        paths[robot].pop_back();
      }

      removed_exit_path = true;
#else
      std::cout << "removing exit path of " << robot << std::endl;
      std::cout << "exit path end time: " << paths[robot].back().t(-1)
                << std::endl;
      paths[robot].pop_back();

      removed_exit_path = true;
#endif
    }

    for (uint j = 0; j < rtpm.at(robot)[task].size(); ++j) {
      arr start_pose;
      uint start_time;

      if (paths[robot].size() > 0) {
        start_pose = paths[robot].back().path[-1];
        start_time = paths[robot].back().t(-1);
      } else {
        start_pose = home_poses.at(robot);
        start_time = 0;

        if (prev_finishing_time > max_start_time_shift + 1) {
          start_time = prev_finishing_time - max_start_time_shift + 1;
        }
      }

      std::cout << "start time " << start_time << std::endl;

      if (!removed_exit_path &&
          prev_finishing_time > start_time + max_start_time_shift + 1) {
        start_time = prev_finishing_time - max_start_time_shift + 1;
      }

      const arr goal_pose = rtpm.at(robot)[task][j];
      const uint time_lb = std::max(
          {(j == rtpm.at(robot)[task].size() - 1) ? prev_finishing_time : 0,
           start_time});

      std::cout << "prev finishing time " << prev_finishing_time << std::endl;
      std::cout << "new start time " << start_time << std::endl;
      std::cout << "lower bound time " << time_lb << std::endl;

      // make animation from path-parts
      rai::Animation A;
      for (const auto &p : paths) {
        for (const auto path : p.second) {
          A.A.append(path.anim);
        }
      }

      if (false) {
        for (uint i = 0; i < A.getT(); ++i) {
          A.setToTime(CPlanner, i);
          CPlanner.watch(false);
          rai::wait(0.1);
        }
      }

      // set configuration to plannable for current robot
      std::cout << "setting up C" << std::endl;
      setActive(CPlanner, robot);

      auto path = plan_in_animation(A, CPlanner, start_time, start_pose,
                                    goal_pose, time_lb, robot, false);

      path.r = robot;
      path.task_index = task;
      path.name = "task";

      if (path.has_solution) {
        if (false) {
          for (uint i = 0; i < path.path.d0; ++i) {
            auto q = path.path[i];
            CPlanner.setJointState(q);
            CPlanner.watch(false);
            rai::wait(0.01);
          }
          CPlanner.setJointState(home_poses.at(robot));
        }
        // make animation part
        auto tmp_frames = robot_frames[robot];
        // add obj. frame to the anim-part.
        if (is_bin_picking) {
          const auto obj = STRING("obj" << task + 1);
          auto to = CPlanner[obj];
          tmp_frames.append(to);
        }
        const auto anim_part =
            make_animation_part(CPlanner, path.path, tmp_frames, start_time);
        path.anim = anim_part;

        if (path.t(-1) > best_makespan_so_far) {
          std::cout << "Stopping early due to better prev. path. ("
                    << best_makespan_so_far << ")" << std::endl;
          return PlanResult(PlanStatus::aborted);
        }

        paths[robot].push_back(path);
      } else {
        std::cout << "Was not able to find a path" << std::endl;
        return PlanResult(PlanStatus::failed);
      }

      // re-link things if we are doing bin-picking
      if (is_bin_picking) {
        CPlanner.setJointState(path.path[-1]);
        // CPlanner.watch(true);
        const auto pen_tip = STRING(robot << "pen_tip");
        const auto obj = STRING("obj" << task + 1);

        if (j == 0) {
          auto from = CPlanner[pen_tip];
          auto to = CPlanner[obj];

          to->unLink();

          // create a new joint
          to->linkFrom(from, true);
          // to->joint->makeRigid();
        }

        if (j == 1) {
          auto to = CPlanner[obj];
          auto from = CPlanner["table_base"];

          to->unLink();

          // create a new joint
          to->linkFrom(from, true);
          // to->joint->makeRigid();
        }

        // CPlanner.watch(true);
      }
    }

    std::cout << "planning exit path" << std::endl;

    const uint exit_start_time = paths[robot].back().t(-1);
    const arr exit_path_start_pose = paths[robot].back().path[-1];

    rai::Animation A;
    for (const auto &p : paths) {
      for (const auto path : p.second) {
        A.A.append(path.anim);
      }
    }

    auto exit_path =
        plan_in_animation(A, CPlanner, exit_start_time, exit_path_start_pose,
                          home_poses.at(robot), exit_start_time, robot, true);
    exit_path.r = robot;
    exit_path.task_index = task;
    exit_path.is_exit = true;
    exit_path.name = "exit";

    if (exit_path.has_solution) {
      const auto exit_anim_part = make_animation_part(
          CPlanner, exit_path.path, robot_frames[robot], exit_start_time);
      exit_path.anim = exit_anim_part;
      paths[robot].push_back(exit_path);
    } else {
      std::cout << "Was not able to find an exit path" << std::endl;
      return PlanResult(PlanStatus::failed);
    }
  }

  if (false) {
    rai::Animation A;
    for (const auto &p : paths) {
      for (const auto path : p.second) {
        A.A.append(path.anim);
      }
    }

    for (uint i = 0; i < A.getT(); ++i) {
      A.setToTime(CPlanner, i);
      CPlanner.watch(false);
      rai::wait(0.1);
    }
  }

  return PlanResult(PlanStatus::success, paths);
}

// overload (not in the literal or in the c++ sense) of the above
PlanResult plan_multiple_arms_given_sequence(
    rai::Configuration C, const RobotTaskPoseMap &rtpm,
    const TaskSequence &sequence, const std::map<Robot, arr> &home_poses,
    const uint best_makespan_so_far = 1e6) {

  Plan paths;
  return plan_multiple_arms_given_subsequence_and_prev_plan(
      C, rtpm, sequence, 0, paths, home_poses, best_makespan_so_far);
}

Plan plan_multiple_arms_unsynchronized(rai::Configuration &C,
                                       const RobotTaskPoseMap &rtpm,
                                       const std::map<Robot, arr> &home_poses) {
  // generate random sequence of robot/pt pairs
  std::vector<Robot> robots;
  for (auto element : home_poses) {
    robots.push_back(element.first);
  }
  const uint num_tasks = rtpm.begin()->second.size();

  const auto seq = generate_random_sequence(robots, num_tasks);

  // plan for it
  const auto plan_result =
      plan_multiple_arms_given_sequence(C, rtpm, seq, home_poses);
  return plan_result.plan;
}

double
get_makespan_from_plan(const std::map<Robot, std::vector<TaskPart>> &plan) {
  double max_time = 0.;
  for (const auto &robot_plan : plan) {
    const auto last_subpath = robot_plan.second.back();
    max_time = std::max({last_subpath.t(-1), max_time});
  }

  return max_time;
}

// TODO
std::map<Robot, std::vector<TaskPart>> reoptimize_plan(const std::map<Robot, std::vector<TaskPart>> &unscaled_plan){
  std::map<Robot, std::vector<TaskPart>> optimized_plan;
  return optimized_plan;
}

// TODO
/*std::map<Robot, std::vector<TaskPart>> rescale_plan(const std::map<Robot, std::vector<TaskPart>> &unscaled_plan){

  std::vector<double> scaling_factors;

  // go over all the robots, by assembling the complete animation, extract the position at times,
  // and make up a timing schedule that respects the velocity and acceleration limits.
  rai::Animation A;
  for (const auto &p : paths) {
    for (const auto path : p.second) {
      A.A.append(path.anim);
    }
  }

  const uint makespan = 0;
  for (uint t=0; t<makespan; ++t){
    const double per_robot_v = 0;
    const double per_robot_a = 0;

    const double max_v = 0;
    const double max_a = 0;

    const double scaling_factor_at_t = 0;


    scaling_factors.push_back(scaling_factor_at_t);
  }

  std::map<Robot, std::vector<TaskPart>> scaled_plan;
  for (const auto &robot_plan : unscaled_plan) {
    const auto robot = robot_plan.first;
    const auto task_parts = robot_plan.second;

    // resample t
    arr t;
    arr scaled_t;

    // rescale path
    arr path = timed_path.resample(scaled_t);
    
    // rebuild animation
    //rai::Animation::AnimationPart anim;
  }


  return scaled_plan;
}*/

double compute_lb_for_sequence(const TaskSequence &seq,
                               const RobotTaskPoseMap &rtpm,
                               const std::map<Robot, arr> &start_poses,
                               const uint start_index = 0,
                               const std::map<Robot, double> start_times = {}) {
  // the lower bound can be computed by finding the minimum time
  // of the current task, and using the precedence constraints as well.
  std::map<Robot, double> robot_time = start_times;
  std::map<Robot, arr> robot_pos = start_poses;

  for (uint i = start_index; i < seq.size(); ++i) {
    const auto task_tuple = seq[i];
    const auto robot = task_tuple.first;
    const auto task_index = task_tuple.second;

    // std::cout << robot << std::endl;

    if (!robot_time.count(robot)) {
      robot_time[robot] = 0.;
    }

    std::cout << robot << " " << task_index << std::endl;

    const arr start_pose = robot_pos[robot];
    const arr goal_pose = rtpm.at(robot)[task_index][0];

    // const arr dist = goal_pose - start_pose;
    // std::cout << goal_pose - start_pose << std::endl;
    const double max_dist = absMax(goal_pose - start_pose);
    const double max_acc = 0.1;
    const double time_to_accelerate = VMAX / max_acc;
    const double acc_dist = 0.5 * VMAX * VMAX / max_acc * 2;

    double dt = 0;
    if (acc_dist > max_dist) {
      // this is wrong
      std::cout << "using acc. timing only" << std::endl;
      dt = 2 * time_to_accelerate;
    } else {
      std::cout << "using acc. timing and max-vel" << std::endl;
      dt = (max_dist - acc_dist) / VMAX + 2 * time_to_accelerate;
    }

    robot_time[robot] += dt;

    for (const auto &rt : robot_time) {
      if (robot_time[robot] < rt.second) {
        robot_time[robot] = rt.second;
      }
    }

    robot_pos[robot] = goal_pose;
  }

  double max = 0;
  for (const auto rt : robot_time) {
    if (max < rt.second) {
      max = rt.second;
    }
  }

  return max;
}

arr get_robot_pose_at_time(const uint t, const Robot r,
                           const std::map<Robot, arr> &home_poses,
                           const std::map<Robot, std::vector<TaskPart>> &plan) {
  if (plan.count(r) > 0) {
    for (const auto &part : plan.at(r)) {
      // std::cout <<part.t(0) << " " << part.t(-1) << std::endl;
      if (part.t(0) > t || part.t(-1) < t) {
        continue;
      }

      for (uint i = 0; i < part.t.N; ++i) {
        if (part.t(i) == t) {
          return part.path[i];
          // std::cout <<part.path[i] << std::endl;
        }
      }
    }
  }

  return home_poses.at(r);
}

void visualize_plan(rai::Configuration C, const Plan &plan,
                    const bool save = false) {
  rai::Animation A;
  for (const auto &p : plan) {
    for (const auto path : p.second) {
      A.A.append(path.anim);
    }
  }

  rai::ConfigurationViewer Vf;
  // Vf.setConfiguration(C, "\"Real World\"", true);
  Vf.setConfiguration(C, "\"Real World\"", false);

  const double makespan = get_makespan_from_plan(plan);

  for (uint t = 0; t < makespan; ++t) {
    // A.setToTime(C, t);

    // std::cout << t << std::endl;
    for (const auto tp : plan) {
      const auto r = tp.first;
      const auto parts = tp.second;

      bool done = false;
      for (auto part : parts) {
        // std::cout <<part.t(0) << " " << part.t(-1) << std::endl;
        if (part.t(0) > t || part.t(-1) < t) {
          continue;
        }

        for (uint i = 0; i < part.t.N - 1; ++i) {
          if (part.t(i) <= t && part.t(i + 1) > t) {
            setActive(C, r);
            C.setJointState(part.path[i]);
            // std::cout <<part.path[i] << std::endl;
            done = true;

            // set bin picking things
            const auto task_index = part.task_index;
            const auto obj_name = STRING("obj" << task_index + 1);

            if (part.anim.frameNames.contains(obj_name)) {
              const auto pose =
                  part.anim.X[uint(std::floor(t - part.anim.start))];
              arr tmp(1, 7);
              tmp[0] = pose[-1];
              C.setFrameState(tmp, {C[obj_name]});
            }
            break;
          }
        }

        if (done) {
          break;
        }
      }
    }

    // C.watch(false);
    Vf.setConfiguration(C, ".", false);
    rai::wait(0.01);

    if (save) {
      Vf.savePng();
    }
  }
}

void export_plan(const std::vector<Robot> &robots,
                 const std::map<Robot, arr> &home_poses,
                 const std::map<Robot, std::vector<TaskPart>> &plan,
                 const TaskSequence &seq, const std::string base_folder,
                 const uint iteration, const uint computation_time) {
  std::cout << "exporting plan" << std::endl;
  // make folder
  const std::string folder =
      "./out/" + base_folder + "/" + std::to_string(iteration) + "/";
  const int res = system(STRING("mkdir -p " << folder).p);
  (void)res;

  rai::Animation A;
  for (const auto &p : plan) {
    for (const auto path : p.second) {
      A.A.append(path.anim);
    }
  }

  // - add info
  // -- comp. time
  {
    std::ofstream f;
    f.open(folder + "comptime.txt", std::ios_base::trunc);
    f << computation_time;
  }

  // -- makespan
  {
    std::ofstream f;
    f.open(folder + "makespan.txt", std::ios_base::trunc);
    f << A.getT();
  }

  // -- sequence
  {
    std::ofstream f;
    f.open(folder + "sequence.txt", std::ios_base::trunc);
    for (const auto &s : seq) {
      f << "(" << s.first << " " << s.second << ")";
    }
  }

  // -- plan
  {
    std::ofstream f;
    f.open(folder + "plan.txt", std::ios_base::trunc);
    for (const auto per_robot_plan : plan) {
      const auto robot = per_robot_plan.first;
      const auto tasks = per_robot_plan.second;

      f << robot << ": ";
      for (const auto task : tasks) {
        f << task.name << "(" << task.algorithm << ")"
          << " " << task.task_index << ", " << task.t(0) << ", " << task.t(-1)
          << "; ";
      }
      f << std::endl;
    }
  }

  // -- actual path
  {
    std::ofstream f;
    f.open(folder + "robot_controls.txt", std::ios_base::trunc);
    arr path(A.getT(), home_poses.at(robots[0]).d0 * robots.size());
    for (uint i = 0; i < A.getT(); ++i) {
      uint offset = 0;
      for (uint j = 0; j < robots.size(); ++j) {
        const arr pose = get_robot_pose_at_time(i, robots[j], home_poses, plan);
        for (uint k = 0; k < pose.N; ++k) {
          path[i](k + offset) = pose(k);
        }
        offset += pose.N;
      }
    }

    f << path;
  }
}

Plan plan_multiple_arms_random_search(rai::Configuration &C,
                                      const RobotTaskPoseMap &rtpm,
                                      const std::map<Robot, arr> &home_poses) {
  // generate random sequence of robot/pt pairs
  std::vector<Robot> robots;
  for (auto element : home_poses) {
    robots.push_back(element.first);
  }
  const uint num_tasks = rtpm.begin()->second.size();

  TaskSequence best_seq;
  Plan best_plan;
  double best_makespan = 1e6;

  for (uint i = 0; i < 100; ++i) {
    const auto seq = generate_random_sequence(robots, num_tasks);

    const double lb = compute_lb_for_sequence(seq, rtpm, home_poses);
    std::cout << "LB for sequence " << lb << std::endl;
    for (auto s : seq) {
      std::cout << "(" << s.first << " " << s.second << ")";
    }
    std::cout << std::endl;

    if (lb > best_makespan) {
      continue;
    }

    // plan for it
    const auto plan_result = plan_multiple_arms_given_sequence(
        C, rtpm, seq, home_poses, best_makespan);
    if (plan_result.status == PlanStatus::success) {
      const Plan plan = plan_result.plan;
      const double makespan = get_makespan_from_plan(plan);

      std::cout << "\n\n\nMAKESPAN " << makespan << " best so far "
                << best_makespan << std::endl;
      for (auto s : seq) {
        std::cout << "(" << s.first << " " << s.second << ")";
      }
      std::cout << "\n\n\n" << std::endl;

      if (makespan < best_makespan) {
        best_makespan = makespan;
        best_plan = plan;

        visualize_plan(C, best_plan);
      }
    }
  }
  return best_plan;
}

TaskSequence swap_robot(const TaskSequence &seq,
                        const std::vector<Robot> &robots) {
  const uint task_index = rand() % seq.size();
  while (true) {
    const uint r = rand() % robots.size();

    TaskSequence seq_new = seq;
    if (seq_new[task_index].first != robots[r]) {
      seq_new[task_index].first = robots[r];
      return seq_new;
    }
  }
}

TaskSequence swap_tasks(const TaskSequence &seq) {
  while (true) {
    const uint t1_index = rand() % seq.size();
    const uint t2_index = rand() % seq.size();

    if (t1_index != t2_index) {
      TaskSequence seq_new = seq;
      auto tmp = seq_new[t1_index];
      seq_new[t1_index] = seq_new[t2_index];
      seq_new[t2_index] = tmp;

      return seq_new;
    }
  }
}

TaskSequence reverse_subtour(const TaskSequence &seq) {
  uint start = rand() % seq.size();
  uint end = rand() % seq.size();

  while (start == end) {
    start = rand() % seq.size();
    end = rand() % seq.size();
  }

  if (start > end) {
    std::swap(start, end);
  }

  TaskSequence seq_new = seq;
  for (uint i = 0; i <= end - start; ++i) {
    seq_new[start + i] = seq[end - i];
  }

  return seq_new;
}

TaskSequence neighbour(const TaskSequence &seq,
                       const std::vector<Robot> &robots) {
  arr rnd(1);
  rndUniform(rnd, 0, 1);

  if (rnd(0) < 1. / 3.) {
    std::cout << "Swapping robots" << std::endl;
    return swap_robot(seq, robots);
  } else if (rnd(0) < 2. / 3.) {
    std::cout << "Swapping tasks" << std::endl;
    return swap_tasks(seq);
  } else {
    std::cout << "Reversing subtour" << std::endl;
    return reverse_subtour(seq);
  }
}

bool sequence_is_feasible(const TaskSequence &seq,
                          const RobotTaskPoseMap &rtpm) {
  for (const auto &s : seq) {
    const Robot r = s.first;
    const auto task_index = s.second;

    if (rtpm.at(r)[task_index].size() == 0) {
      return false;
    }
  }

  return true;
}

void get_plan_from_cache() {}

Plan plan_multiple_arms_squeaky_wheel(
    rai::Configuration &C, const RobotTaskPoseMap &rtpm,
    const std::map<Robot, arr> &home_poses) {
}

Plan plan_multiple_arms_greedy_random_search(
    rai::Configuration &C, const RobotTaskPoseMap &rtpm,
    const std::map<Robot, arr> &home_poses) {
  // make foldername for current run
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);

  std::stringstream buffer;
  buffer << "greedy_" << std::put_time(&tm, "%Y%m%d_%H%M%S");

  // generate random sequence of robot/pt pairs
  std::vector<Robot> robots;
  for (auto element : home_poses) {
    robots.push_back(element.first);
  }
  const uint num_tasks = rtpm.begin()->second.size();

  TaskSequence best_seq;
  Plan best_plan;
  double best_makespan = 1e6;

  auto start_time = std::chrono::high_resolution_clock::now();

  std::vector<std::pair<TaskSequence, Plan>> cache;

  uint iter = 0;
  for (uint i = 0; i < 20000; ++i) {
    std::cout << "Generating completely new seq. " << i << std::endl;
    TaskSequence seq;
    // seq = generate_alternating_random_sequence(robots, num_tasks, rtpm);
    // seq = generate_single_arm_sequence(robots, num_tasks);
    seq = generate_alternating_greedy_sequence(robots, num_tasks, rtpm,
                                               home_poses);
    /*if (true || i == 0) {
      // seq = generate_single_arm_sequence(robots, num_tasks);
      //seq = generate_random_sequence(robots, num_tasks);
      // seq = generate_alternating_greedy_sequence(robots, num_tasks, rtpm,
    home_poses); } else if (i == 1) { seq =
    generate_alternating_random_sequence(robots, num_tasks, rtpm); } else if (i
    == 2) { seq = generate_single_arm_sequence(robots, num_tasks); } else { seq
    = generate_random_sequence(robots, num_tasks);
    }*/

    if (!sequence_is_feasible(seq, rtpm)) {
      std::cout << "Generated sequence no feasible" << std::endl;
      continue;
    }

    Plan plan;
    double prev_makespan = 1e6;
    for (uint j = 0; j < 30; ++j) {
      ++iter;
      TaskSequence new_seq = seq;
      while (true) {
        if (j > 0) {
          new_seq = neighbour(seq, robots);
        }

        // ensure that sequence is actually feasible, i.e. robots can do the
        // assigned tasks
        if (sequence_is_feasible(new_seq, rtpm)) {
          break;
        }
      }

      const double lb = compute_lb_for_sequence(new_seq, rtpm, home_poses);
      std::cout << "LB for sequence " << lb << std::endl;
      for (const auto &s : new_seq) {
        std::cout << "(" << s.first << " " << s.second << ")";
      }
      std::cout << std::endl;

      if (lb > best_makespan) {
        std::cout << "skipping planning, since lb is larger than best plan"
                  << std::endl;
        continue;
      }

      // plan for it
      PlanResult new_plan_result;
      if (plan.empty()) {
        new_plan_result = plan_multiple_arms_given_sequence(
            C, rtpm, new_seq, home_poses, prev_makespan);
      } else {
        // compute index where the new sequence starts
        uint change_in_sequence = 0;
        for (uint k = 0; k < seq.size(); ++k) {
          if (seq[k].first != new_seq[k].first ||
              seq[k].second != new_seq[k].second) {
            change_in_sequence = k;
            break;
          }
        }
        std::cout << "planning only subsequence " << change_in_sequence
                  << std::endl;
        new_plan_result = plan_multiple_arms_given_subsequence_and_prev_plan(
            C, rtpm, new_seq, change_in_sequence, plan, home_poses,
            prev_makespan);
      }

      if (new_plan_result.status == PlanStatus::success) {
        const Plan new_plan = new_plan_result.plan;
        const double makespan = get_makespan_from_plan(new_plan);

        const auto end_time = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                                  end_time - start_time)
                                  .count();

        // cache.push_back(std::make_pair(new_seq, new_plan));

        export_plan(robots, home_poses, new_plan, new_seq, buffer.str(), iter,
                    duration);

        std::cout << "\n\n\nMAKESPAN " << makespan << " best so far "
                  << best_makespan << " (" << prev_makespan << ")" << std::endl;
        for (const auto &s : new_seq) {
          std::cout << "(" << s.first << " " << s.second << ")";
        }
        std::cout << "\n\n\n" << std::endl;

        if (makespan < prev_makespan) {
          seq = new_seq;
          plan = new_plan;
          prev_makespan = makespan;

          visualize_plan(C, plan);
        }

        if (makespan < best_makespan) {
          best_makespan = makespan;
          best_plan = plan;
          best_seq = new_seq;

          // visualize_plan(C, best_plan);
        }
      } else {
        const std::string folder =
            "./out/" + buffer.str() + "/" + std::to_string(iter) + "/";
        const int res = system(STRING("mkdir -p " << folder).p);
        (void)res;

        {
          std::ofstream f;
          f.open(folder + "comptime.txt", std::ios_base::trunc);
          const auto end_time = std::chrono::high_resolution_clock::now();
          const auto duration =
              std::chrono::duration_cast<std::chrono::seconds>(end_time -
                                                               start_time)
                  .count();
          f << duration;
        }
        {
          std::ofstream f;
          if (new_plan_result.status == PlanStatus::failed) {
            f.open(folder + "failed.txt", std::ios_base::trunc);
          } else if (new_plan_result.status == PlanStatus::aborted) {
            f.open(folder + "aborted.txt", std::ios_base::trunc);
          }
        }
      }
      if (new_plan_result.status == PlanStatus::failed) {
        break;
      }
    }
  }
  return best_plan;
}

void plan_multiple_arms_simulated_annealing(
    rai::Configuration C, const RobotTaskPoseMap &rtpm,
    const std::map<Robot, arr> &home_poses) {
  // generate random sequence of robot/pt pairs
  std::vector<Robot> robots;
  for (const auto element : home_poses) {
    robots.push_back(element.first);
  }
  const uint num_tasks = rtpm.begin()->second.size();
  const auto seq = generate_random_sequence(robots, num_tasks);

  // plan for it
  const auto plan_result =
      plan_multiple_arms_given_sequence(C, rtpm, seq, home_poses);

  auto best_plan = plan_result.plan;
  uint best_makespan = get_makespan_from_plan(plan_result.plan);

  uint curr_makespan = best_makespan;

  auto p = [](const double e, const double eprime, const double temperature) {
    if (eprime < e) {
      return 1.;
    }

    return exp(-(eprime - e) / temperature);
  };

  const uint max_iter = 1000;
  const double T0 = 1e6;

  double T = T0;
  double cooling_factor = 0.999;

  std::vector<uint> best_makespan_at_iteration;
  std::vector<double> computation_time_at_iteration;

  for (uint i = 0; i < max_iter; ++i) {
    // T = T0 * (1 - (i+1.)/nmax);
    T = T * cooling_factor; // temp(i);

    // modify sequence
    const TaskSequence seq_new = neighbour(seq, robots);

    // compute lower bound
    double lb_makespan = compute_lb_for_sequence(seq_new, rtpm, home_poses);

    arr rnd(1);
    rndUniform(rnd);

    if (p(curr_makespan, lb_makespan, T) > rnd(0)) {
      const auto new_plan =
          plan_multiple_arms_given_sequence(C, rtpm, seq_new, home_poses).plan;

      uint makespan = get_makespan_from_plan(new_plan);

      if (p(curr_makespan, lb_makespan, T) > rnd(0)) {
        curr_makespan = makespan;
      }

      if (makespan < best_makespan) {
        best_makespan = makespan;
      }
    }

    best_makespan_at_iteration.push_back(best_makespan);
    computation_time_at_iteration.push_back(i);
  }
}

RobotTaskPoseMap
compute_pick_and_place_positions(rai::Configuration &C,
                                 const std::vector<std::string> &robots,
                                 const uint n = 6) {
  RobotTaskPoseMap rtpm;

  for (const auto prefix : robots) {
    for (uint i = 0; i < n; ++i) {
      setActive(C, prefix);

      const auto home = C.getJointState();

      OptOptions options;
      options.stopIters = 100;
      options.damping = 1e-3;

      KOMO komo;
      komo.verbose = 0;
      komo.setModel(C, true);

      komo.setDiscreteOpt(2);

      // komo.world.stepSwift();

      komo.add_collision(true, .01, 1e1);
      komo.add_jointLimits(true, 0., 1e1);

      auto pen_tip = STRING(prefix << "pen_tip");
      auto obj = STRING("obj" << i + 1);
      auto goal = STRING("goal" << i + 1);

      Skeleton S = {
          {1., 1., SY_touch, {pen_tip, obj}},
          {1., 2., SY_stable, {pen_tip, obj}},
          {2., 2., SY_poseEq, {obj, goal}},
      };

      komo.setSkeleton(S);

      // komo.addObjective({1.}, FS_position, {STRING(prefix << "pen_tip")},
      // OT_eq,
      //                  {1e2}, point);
      // komo.addObjective({1., 1.}, FS_distance, {STRING(prefix << "pen_tip"),
      // STRING(obj << i + 1)}, OT_eq, {1e1});

      komo.addObjective({1.}, FS_vectorZ, {STRING(prefix << "pen")}, OT_sos,
                        {1e1}, {0., 0., -1.});
      // komo.addObjective({1.}, FS_position, {STRING(prefix << "pen_tip")},
      // OT_sos, {1e0}, C[obj]->getPosition());

      // komo.addObjective({1.}, FS_vectorZ, {STRING(prefix << "pen")}, OT_sos,
      // {1e1}, {0., 0., -1.}); komo.addObjective({1.}, FS_vectorZDiff,
      // {STRING(prefix << "pen"), "world"}, OT_ineq, {1e1}, {0., 0., -0.9});
      ConfigurationProblem cp(komo.world);
      setActive(cp.C, prefix);

      for (uint i = 0; i < 10; ++i) {
        komo.run_prepare(0.0, true);
        komo.run(options);

        const arr q0 = komo.getPath()[0]();
        const arr q1 = komo.getPath()[1]();
        // komo.pathConfig.watch(true);

        // ensure via sampling as well
        const bool res1 = cp.query(q0)->isFeasible;
        const bool res2 = cp.query(q1)->isFeasible;

        if (res1 && res2 && komo.getReport(false).get<double>("ineq") < 1. &&
            komo.getReport(false).get<double>("eq") < 1.) {
          rtpm[prefix].push_back({q0, q1});
          break;
        } else {
          std::cout << "failed for a bit" << std::endl;
        }
      }
    }
  }

  return rtpm;
}

RobotTaskPoseMap
compute_stippling_poses_for_arms(rai::Configuration &C, const arr &pts,
                                 const std::vector<Robot> &robots) {
  RobotTaskPoseMap rtpm;
  for (const Robot &r : robots) {
    const TaskPoses poses = computeConfigurationsForPoints(pts, C, r);
    std::vector<TaskPoses> tp;
    for (const arr &p : poses) {
      if (p.N > 0) {
        tp.push_back({p});
      } else {
        tp.push_back({});
      }
    }
    rtpm[r] = tp;
  }

  return rtpm;
}

std::map<Robot, arr> get_robot_home_poses(rai::Configuration &C,
                                          const std::vector<Robot> &robots) {
  std::map<Robot, arr> poses;
  for (auto r : robots) {
    setActive(C, r);
    poses[r] = C.getJointState();

    // std::cout << poses[r] << std::endl;
  }

  return poses;
}

void load_and_viz(rai::Configuration C, const bool pick_and_place) {
  arr path;
  // const std::string filepath =
  // "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/grid/greedy_20230328_000824/139/robot_controls.txt";
  // const std::string filepath =
  // "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/lis/greedy_20230329_000242/86/robot_controls.txt";
  // const std::string filepath =
  // "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/lis/greedy_20230329_103317/1/robot_controls.txt";

  // for bin picking:
  // robot, obj, start, end
  // for points
  // robot end
  // const std::string filepath =
  // "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/bin/greedy_20230328_211219/29/robot_controls.txt";
  /*std::vector<std::vector<uint>> timings;
  timings.push_back({0, 0, 22, 53});
  timings.push_back({0, 5, 92, 137});
  timings.push_back({0, 3, 151, 214});
  timings.push_back({0, 2, 233, 263});

  timings.push_back({1, 1, 110, 179});
  timings.push_back({1, 4, 218, 277});*/

  /*const std::string filepath =
  "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/bin/greedy_20230330_005636/19/robot_controls.txt";
  std::vector<std::vector<uint>> timings;
  timings.push_back({0, 5, 24, 69});
  timings.push_back({0, 0, 107, 138});
  timings.push_back({0, 3, 159, 220});
  timings.push_back({0, 2, 239, 269});

  timings.push_back({1, 1, 111, 170});
  timings.push_back({1, 4, 221, 269});*/

  /*
  const std::string filepath =
  "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/grid_four/opt/greedy_20230331_020353/3/robot_controls.txt";
  std::vector<std::vector<uint>> timings;
  timings.push_back({0, 39});
  timings.push_back({0, 101});
  timings.push_back({0, 184});
  timings.push_back({0, 252});
  timings.push_back({0, 295});

  timings.push_back({1, 56});
  timings.push_back({1, 143});
  timings.push_back({1, 207});
  timings.push_back({1, 260});

  timings.push_back({2, 76});
  timings.push_back({2, 157});
  timings.push_back({2, 221});

  timings.push_back({3, 32});
  timings.push_back({3, 89});
  timings.push_back({3, 172});
  timings.push_back({3, 221});
  */

  // greedy
  /*const std::string filepath =
  "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/grid_four/opt/greedy_20230331_165530/1/robot_controls.txt";
  std::vector<std::vector<uint>> timings;
  timings.push_back({0, 25});
  timings.push_back({0, 64});
  timings.push_back({0, 123});
  timings.push_back({0, 214});

  timings.push_back({1, 46});
  timings.push_back({1, 68});
  timings.push_back({1, 142});
  timings.push_back({1, 225});

  timings.push_back({2, 48});
  timings.push_back({2, 97});
  timings.push_back({2, 156});

  timings.push_back({3, 26});
  timings.push_back({3, 65});
  timings.push_back({3, 96});
  timings.push_back({3, 182});
  timings.push_back({3, 235});*/

  // opt
  const std::string filepath =
      "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/"
      "greedy_20230401_010325/17/robot_controls.txt";
  std::vector<std::vector<uint>> timings;
  timings.push_back({0, 25});
  timings.push_back({0, 52});
  timings.push_back({0, 66});
  timings.push_back({0, 94});
  timings.push_back({0, 105});
  timings.push_back({0, 172});

  timings.push_back({1, 116});
  timings.push_back({1, 171});

  timings.push_back({2, 34});
  timings.push_back({2, 60});
  timings.push_back({2, 141});

  timings.push_back({3, 26});
  timings.push_back({3, 35});
  timings.push_back({3, 53});
  timings.push_back({3, 59});
  timings.push_back({3, 153});
  timings.push_back({3, 194});

  FILE(filepath.c_str()) >> path;

  const std::vector<Robot> robots{"a0_", "a1_", "a2_", "a3_"};
  // const std::vector<Robot> robots{"a0_", "a1_"};

  setActive(C, robots);

  rai::ConfigurationViewer Vf;
  Vf.setConfiguration(C, "\"Real World\"", true);

  for (uint i = 0; i < path.d0; ++i) {
    C.setJointState(path[i]);
    // C.watch(false);
    // rai::wait(0.01);

    if (pick_and_place) {
      for (uint j = 0; j < timings.size(); ++j) {
        const auto pen_tip = STRING("a" << timings[j][0] << "_pen_tip");
        const auto obj = STRING("obj" << timings[j][1] + 1);

        if (i == timings[j][2]) {
          auto from = C[pen_tip];
          auto to = C[obj];

          to->unLink();

          // create a new joint
          to->linkFrom(from, true);
          // to->joint->makeRigid();
        }

        if (i == timings[j][3]) {
          auto to = C[obj];
          auto from = C["table_base"];

          to->unLink();

          // create a new joint
          to->linkFrom(from, true);
          // to->joint->makeRigid();
        }
      }
    } else {
      // draw dots
      for (uint j = 0; j < timings.size(); ++j) {
        const auto pen_tip = STRING("a" << timings[j][0] << "_pen_tip");
        const arr pos = C[pen_tip]->getPosition();

        if (timings[j][1] == i) {
          // add small sphere
          auto *dot = C.addFrame("goal", "table");
          dot->setShape(rai::ST_sphere, {0.01});
          dot->setPosition(pos);
          dot->setContact(0.);

          if (timings[j][0] == 0) {
            dot->setColor({0, 0., 0., 0.5});
          } else if (timings[j][0] == 1) {
            dot->setColor({1., 0., 0., 0.5});
          } else if (timings[j][0] == 2) {
            dot->setColor({1., 0., 1., 0.5});
          } else if (timings[j][0] == 3) {
            dot->setColor({1., 1., 0., 0.5});
          }
        }
      }
    }
    Vf.setConfiguration(C, ".", false);
    rai::wait(0.01);
    Vf.savePng();
  }
}

arr get_scenario(const rai::String &str) {
  // const arr pts = grid(2, 2, 0.4, 0.1);
  // const arr pts = grid(2, 3, 0.4, 0.1);

  arr pts;
  if (str == "default_grid") {
    pts = grid();
  } else if (str == "four_by_four_grid") {
    pts = grid(4, 4);
  } else if (str == "three_by_three_grid") {
    pts = grid(3, 3);
  } else if (str == "three_by_two_grid") {
    pts = grid(3, 2);
  } else if (str == "spiral") {
    pts = spiral();
  } else if (str == "random") {
    pts = randomPts();
  } else if (str == "cube") {
    pts = cube(200);
  } else if (str == "circles") {
    pts = circles(0.3, 7);
  } else if (str == "lis_default") {
    pts = LISlogo(false);
  } else if (str == "lis_large") {
    pts = LISlogo(true);
  } else if (str == "greedy_counterexample") {
    pts = greedy_counterexample();
  } else if (str == "four_robot_test_1") {
    pts = grid(2, 2, 0.05, 0.05);
  } else if (str == "four_robot_test_2") {
    pts = grid(2, 2, 0.7, 0.05);
  } else {
    std::cout << "Scenario not found" << std::endl;
  }

  return pts;
}

int main(int argc, char **argv) {
  rai::initCmdLine(argc, argv);
  const uint seed = rai::getParameter<double>("seed", 42); // seed
  rnd.seed(seed);

  const uint verbosity = rai::getParameter<double>(
      "verbosity", 0); // verbosity, does not do anything atm

  const bool plan_pick_and_place =
      rai::getParameter<bool>("pnp", false); // pick and place yes/no

  // possible modes:
  // - test
  // - optimize
  // - show scenario
  // - show saved path
  const rai::String mode =
      rai::getParameter<rai::String>("mode", "test"); // scenario
  const rai::String stippling_scenario =
      rai::getParameter<rai::String>("stippling_pts", ""); // scenario

  const rai::String env =
      rai::getParameter<rai::String>("env", ""); // environment

  std::vector<std::string> robots; // string-prefix for robots

  rai::Configuration C;
  if (plan_pick_and_place) {
    pick_and_place(C);
    robots = {"a0_", "a1_"};
  } else {
    if (env == "lab") {
      labSetting(C);
      robots = {"a0_", "a1_"};
    } else {
      more_robots(C, 4);
      robots = {"a0_", "a1_", "a2_", "a3_"};
    }
  }

  // maps [robot] to home_pose
  const std::map<Robot, arr> home_poses = get_robot_home_poses(C, robots);

  // show prev path
  if (mode == "show_plan") {
    load_and_viz(C, plan_pick_and_place);
    return 0;
  }

  // stippling
  RobotTaskPoseMap robot_task_pose_mapping;
  if (!plan_pick_and_place) {
    const arr pts = get_scenario(stippling_scenario);
    if (pts.N == 0) {
      return 0;
    }

    if (verbosity > 0) {
      drawPts(C, pts);
    }

    // maps [robot] to [index, pose]
    std::cout << "Computing stippling poses" << std::endl;
    robot_task_pose_mapping = compute_stippling_poses_for_arms(C, pts, robots);
  } else {
    // bin picking
    std::cout << "Computing pick and place poses" << std::endl;
    robot_task_pose_mapping = compute_pick_and_place_positions(C, robots);
  }

  // initial test
  if (mode == "test") {
    const auto plan = plan_multiple_arms_unsynchronized(
        C, robot_task_pose_mapping, home_poses);
    std::cout << "Makespan: " << get_makespan_from_plan(plan) << std::endl;

    visualize_plan(C, plan);
  } else if (mode == "random_search") {
    // random search
    const auto plan = plan_multiple_arms_random_search(
        C, robot_task_pose_mapping, home_poses);
  } else if (mode == "greedy_random_search") {
    // greedy random search
    const auto plan = plan_multiple_arms_greedy_random_search(
        C, robot_task_pose_mapping, home_poses);
  } else if (mode == "simulated_annealing") {
    plan_multiple_arms_simulated_annealing(C, robot_task_pose_mapping,
                                           home_poses);
  }

  return 0;
}
