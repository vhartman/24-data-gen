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

#include "json.h"

#include <PlanningSubroutines/Animation.h>
#include <PlanningSubroutines/ConfigurationProblem.h>

#include "plan.h"

#include "env_util.h"
#include "path_util.h"
#include "stippling.h"
#include "util.h"

#include "planners/optimal_planner.h"
#include "planners/prioritized_planner.h"
#include "planners/planner.h"

#include "samplers/sampler.h"

// TODO:
// Code
// - split main planning subroutine
// - fix loading and visualization of previously computed paths
//
// Speed improvements
// - squeaky wheel planner
// - simulated annealing
// - speed up komo runs
//
// Capabilities
// - enable things that are not only 'go to point', e.g. drawing a line
// - enable search over sequences with precendence constraints
// - time-rescale path
// - enable multi-arm cooperation
// - look into more complex motion planning:
// -- joint optimization
// -- constrained sampling based planning
// - more statistics
// - joint optimization over the whole path

using json = nlohmann::json;

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
  for (const auto &element : tmp) {
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

OrderedTaskSequence generate_random_sequence(const std::vector<Robot> &robots,
                                             const uint num_tasks) {
  OrderedTaskSequence seq;

  for (uint i = 0; i < num_tasks; ++i) {
    // sample robot
    const uint r = rand() % robots.size();

    // make pair
    seq.push_back(RobotTaskPair{.robots={robots[r]}, .task = Task{.object=i}});
  }

  auto seed = static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count());
  auto rng = std::default_random_engine{seed};
  std::shuffle(std::begin(seq), std::end(seq), rng);

  return seq;
}

OrderedTaskSequence
generate_single_arm_sequence(const std::vector<Robot> &robots,
                             const uint num_tasks) {
  // sample robot _once_.
  const uint r = rand() % robots.size();

  OrderedTaskSequence seq;
  for (uint i = 0; i < num_tasks; ++i) {
    // make pair
    seq.push_back(RobotTaskPair{.robots = {robots[r]}, .task=Task{.object=i}});
  }

  auto seed = static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count());
  auto rng = std::default_random_engine{seed};
  std::shuffle(std::begin(seq), std::end(seq), rng);

  return seq;
}

OrderedTaskSequence
generate_alternating_random_sequence(const std::vector<Robot> &robots,
                                     const uint num_tasks,
                                     const RobotTaskPoseMap &rtpm) {
  uint r = rand() % robots.size();

  auto available_tasks = straightPerm(num_tasks);
  OrderedTaskSequence seq;
  while (available_tasks.size() > 0) {
    for (uint j = 0; j < 10; ++j) {
      const uint task_index = available_tasks[rand() % available_tasks.size()];
      RobotTaskPair rtp;
      rtp.robots = {robots[r]};
      rtp.task = Task{.object=task_index, .type=TaskType::pick};

      // check if the task is feasible with the chosen robot
      if (rtpm.count(rtp) != 0) {
        seq.push_back(RobotTaskPair{.robots={robots[r]}, .task=Task{.object=task_index}});

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

OrderedTaskSequence generate_alternating_greedy_sequence(
    const std::vector<Robot> &robots, const uint num_tasks,
    const RobotTaskPoseMap &rtpm, const std::map<Robot, arr> &home_poses) {
  std::cout << "Generating alternating greedy" << std::endl;
  auto available_tasks = straightPerm(num_tasks);

  // sample starting_ robot.
  uint r = rand() % robots.size();
  std::map<Robot, arr> poses = home_poses;

  OrderedTaskSequence seq;
  while (available_tasks.size() > 0) {
    // find minimum dist pose to current robot
    auto min_dist = 1e6;
    uint task_index = 0;
    bool assigned_task = false;
    for (auto t : available_tasks) {
      RobotTaskPair rtp;
      rtp.robots = {robots[r]};
      rtp.task = Task{.object=t, .type=TaskType::pick};

      // check if a valid pose exists for the object/action pair
      if (rtpm.count(rtp) != 0) {
        // estimate pose-distance
        const auto dist = absMax(poses[robots[r]] - rtpm.at(rtp)[0][0]);
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
      seq.push_back(RobotTaskPair{.robots={robots[r]}, .task=Task{.object=task_index, .type=TaskType::pick}});
    }
    else{
      std::cout << "not asigned task" << std::endl;
    }

    r = (r + 1) % robots.size();
  }

  return seq;
}

PlanResult plan_multiple_arms_jointly(rai::Configuration C, const RobotTaskPoseMap &rtpm,
    const OrderedTaskSequence &sequence, const std::map<Robot, arr> &home_poses){
  CompoundTreePlanner ctp(C, sequence);
  auto plan = ctp.plan();
  return PlanResult(PlanStatus::failed);
}

Plan plan_multiple_arms_unsynchronized(rai::Configuration &C,
                                       const RobotTaskPoseMap &rtpm,
                                       const std::map<Robot, arr> &home_poses) {
  // generate random sequence of robot/pt pairs
  std::vector<Robot> robots;
  for (const auto &element : home_poses) {
    robots.push_back(element.first);
  }

  int num_tasks = 0;
  for (auto f : C.frames) {
    if (f->name.contains("obj")) {
      num_tasks += 1;
    }
  }

  const auto seq = generate_random_sequence(robots, num_tasks);

  // plan for it
  const auto plan_result =
      plan_multiple_arms_given_sequence(C, rtpm, seq, home_poses);
  return plan_result.plan;
}

// TODO
/*std::map<Robot, std::vector<TaskPart>> rescale_plan(const std::map<Robot,
std::vector<TaskPart>> &unscaled_plan){

  std::vector<double> scaling_factors;

  // go over all the robots, by assembling the complete animation, extract the
position at times,
  // and make up a timing schedule that respects the velocity and acceleration
limits. rai::Animation A; for (const auto &p : paths) { for (const auto path :
p.second) { A.A.append(path.anim);
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

double estimate_task_duration(const arr &start_pose, const arr &goal_pose,
                              const double max_vel, const double max_acc) {
  const double max_dist = absMax(goal_pose - start_pose);
  const double time_to_accelerate = max_vel / max_acc;
  const double acc_dist = 0.5 * max_vel * max_vel / max_acc * 2;

  double dt = 0;
  if (acc_dist > max_dist) {
    // this is wrong
    std::cout << "using acc. timing only" << std::endl;
    dt = 2 * time_to_accelerate;
  } else {
    std::cout << "using acc. timing and max-vel" << std::endl;
    dt = (max_dist - acc_dist) / max_vel + 2 * time_to_accelerate;
  }

  return dt;
}

double compute_lb_for_sequence(const OrderedTaskSequence &seq,
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
    const auto robot = task_tuple.robots[0];
    const auto task_index = task_tuple.task.object;

    // std::cout << robot << std::endl;

    if (!robot_time.count(robot)) {
      robot_time[robot] = 0.;
    }

    std::cout << robot << " " << task_index << std::endl;

    const arr start_pose = robot_pos[robot];
    const arr goal_pose = rtpm.at(task_tuple)[0][0];

    const double max_acc = 0.1;

    const double dt =
        estimate_task_duration(start_pose, goal_pose, VMAX, max_acc);
    robot_time[robot] += dt;

    // since we know that there is a precendence constraint on the
    // task-completion-order we set the time to the highest current time if it
    // was lower than that
    for (const auto &rt : robot_time) {
      if (robot_time[robot] < rt.second) {
        robot_time[robot] = rt.second;
      }
    }

    robot_pos[robot] = goal_pose;
  }

  // extract the maximum time of all the robot times
  double max = 0;
  for (const auto &rt : robot_time) {
    if (max < rt.second) {
      max = rt.second;
    }
  }

  return max;
}

void visualize_plan(rai::Configuration C, const Plan &plan,
                    const bool save = false) {
  rai::ConfigurationViewer Vf;
  // Vf.setConfiguration(C, "\"Real World\"", true);
  Vf.setConfiguration(C, "\"Real World\"", false);

  const double makespan = get_makespan_from_plan(plan);

  for (uint t = 0; t < makespan; ++t) {
    // A.setToTime(C, t);

    // std::cout << t << std::endl;
    for (const auto &tp : plan) {
      const auto r = tp.first;
      const auto parts = tp.second;

      bool done = false;
      for (const auto &part : parts) {
        // std::cout <<part.t(0) << " " << part.t(-1) << std::endl;
        if (part.t(0) > t || part.t(-1) < t) {
          continue;
        }

        for (uint i = 0; i < part.t.N; ++i) {
          if ((i == part.t.N-1 && t == part.t(-1)) ||
              (i < part.t.N-1 && (part.t(i) <= t && part.t(i + 1) > t))) {
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
  rai::wait(0.01);
  std::cout << "B" << std::endl;
}

void export_plan(rai::Configuration C, const std::vector<Robot> &robots,
                 const std::map<Robot, arr> &home_poses,
                 const Plan &plan,
                 const OrderedTaskSequence &seq, const std::string base_folder,
                 const uint iteration, const uint computation_time) {
  std::cout << "exporting plan" << std::endl;
  // make folder
  const std::string folder =
      "./out/" + base_folder + "/" + std::to_string(iteration) + "/";
  const int res = system(STRING("mkdir -p " << folder).p);
  (void)res;

  rai::Animation A;
  for (const auto &p : plan) {
    for (const auto &path : p.second) {
      A.A.append(path.anim);
    }
  }

  // - add info
  // -- comp. time
  {
    std::ofstream f;
    f.open(folder + "comptime.txt", std::ios_base::trunc);
    f << computation_time / 1000.;
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
      f << "(" << s.robots[0] << " " << s.task.object << ")";
    }
  }

  {
    std::ofstream f;
    f.open(folder + "robot_pose.json", std::ios_base::trunc);
    
    // robots
    json data;
    for (const auto &e: home_poses){
      const auto r = e.first;
      const rai::String robot_base STRING(r << "base");

      data[robot_base.p] = C[robot_base]->getPose();
    }

    f << data;
  }

  {
    std::ofstream f;
    f.open(folder + "obj_goals.json", std::ios_base::trunc);
    
    json data;
    // goals
    for (const auto frame: C.frames){
      if (frame->name.contains("goal")){
        data[frame->name.p] = frame->getPose().vec();
      }
    }

    f << data;
  }

  {
    std::map<rai::String, arr> objs;
    for (const auto frame: C.frames){
      if (frame->name.contains("obj")){
        objs[frame->name] = arr(0, 7);
      }
    }

    const double makespan = get_makespan_from_plan(plan);
    for (uint t = 0; t < makespan; ++t) {
      // std::cout << t << std::endl;
      for (const auto &tp : plan) {
        const auto r = tp.first;
        const auto parts = tp.second;

        bool done = false;
        for (const auto &part : parts) {
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

      for (const auto &obj: objs){
        objs[obj.first].append(C[obj.first]->getPose());
      }
    }

    std::ofstream f;
    f.open(folder + "obj_poses.json", std::ios_base::trunc);
    
    json data;
    for (const auto &obj: objs){
      std::vector<arr> tmp;
      for (uint i=0; i<obj.second.d0; ++i) tmp.push_back(obj.second[i]);
      data[obj.first.p] = tmp;
    }

    f << data;
  }

  // -- plan
  {
    std::ofstream f;
    f.open(folder + "plan.txt", std::ios_base::trunc);

    for (const auto &per_robot_plan : plan) {
      const auto robot = per_robot_plan.first;
      const auto tasks = per_robot_plan.second;

      f << robot << ": ";
      for (const auto &task : tasks) {
        f << task.name << "(" << task.algorithm << ")"
          << " " << task.task_index << ", " << task.t(0) << ", " << task.t(-1)
          << "; ";
      }
      f << std::endl;
    }
  }

  {
    std::ofstream f;
    f.open(folder + "plan.json", std::ios_base::trunc);
    json data;

    for (const auto &per_robot_plan : plan) {
      const auto robot = per_robot_plan.first;
      const auto tasks = per_robot_plan.second;

      json tmp;
      tmp["robot"] = robot;

      for (const auto &task : tasks) {
        json task_description;
        task_description["name"] = task.name;
        task_description["algorithm"]= task.algorithm;
        task_description["index"] = task.task_index;
        task_description["start"] = task.t(0);
        task_description["end"] = task.t(-1);

        tmp["tasks"].push_back(task_description);
      }

      data.push_back(tmp);
    }

    f << data;
  }

  // -- compute times
  {
    std::ofstream f;
    f.open(folder + "computation_times.txt", std::ios_base::trunc);
    for (const auto &per_robot_plan : plan) {
      const auto robot = per_robot_plan.first;
      const auto tasks = per_robot_plan.second;

      f << robot << ": ";
      for (const auto &task : tasks) {
        f << task.name << "(" << task.algorithm << ")"
          << " " << task.task_index << ", " << task.stats.total_compute_time
          << ", " << task.stats.rrt_compute_time << ", "
          << task.stats.rrt_plan_time << ", "
          << task.stats.rrt_coll_time << ", "
          << task.stats.rrt_nn_time << ", "
          << task.stats.rrt_smoothing_time << ", "
          << task.stats.rrt_shortcut_time << ", "
          << task.stats.komo_compute_time << "; ";
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

  {
    std::ofstream f;
    f.open(folder + "robot_controls.json", std::ios_base::trunc);
    json data;

    // arr path(A.getT(), home_poses.at(robots[0]).d0 * robots.size());
    std::map<Robot, std::vector<arr>> paths;
    for (uint i = 0; i < A.getT(); ++i) {
      uint offset = 0;
      for (uint j = 0; j < robots.size(); ++j) {
        const arr pose = get_robot_pose_at_time(i, robots[j], home_poses, plan);
        paths[robots[j]].push_back(pose);
      }
    }
    data = paths;

    f << data;
  }
}

Plan plan_multiple_arms_random_search(rai::Configuration &C,
                                      const RobotTaskPoseMap &rtpm,
                                      const std::map<Robot, arr> &home_poses) {
  // generate random sequence of robot/pt pairs
  std::vector<Robot> robots;
  for (const auto &element : home_poses) {
    robots.push_back(element.first);
  }
  int num_tasks = 0;
  for (auto f : C.frames) {
    if (f->name.contains("obj")) {
      num_tasks += 1;
    }
  }

  OrderedTaskSequence best_seq;
  Plan best_plan;
  double best_makespan = 1e6;

  for (uint i = 0; i < 100; ++i) {
    const auto seq = generate_random_sequence(robots, num_tasks);

    const double lb = compute_lb_for_sequence(seq, rtpm, home_poses);
    std::cout << "LB for sequence " << lb << std::endl;
    for (auto s : seq) {
      std::cout << "(" << s.robots[0] << " " << s.task.object << ")";
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
        std::cout << "(" << s.robots[0] << " " << s.task.object << ")";
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

OrderedTaskSequence swap_robot(const OrderedTaskSequence &seq,
                               const std::vector<Robot> &robots) {
  // randomly choose what to swap in the sequence
  const uint task_index = rand() % seq.size();
  while (true) {
    const uint r = rand() % robots.size();

    OrderedTaskSequence seq_new = seq;
    // check if the robots in the chosen task are different from the randomly sampled one
    if (seq_new[task_index].robots[0] != robots[r]) {
      seq_new[task_index].robots[0] = robots[r];
      return seq_new;
    }
  }
}

OrderedTaskSequence swap_tasks(const OrderedTaskSequence &seq) {
  while (true) {
    // randomly sample task-indices
    const uint t1_index = rand() % seq.size();
    const uint t2_index = rand() % seq.size();

    // enusre that they are different
    if (t1_index != t2_index) {
      OrderedTaskSequence seq_new = seq;
      auto tmp = seq_new[t1_index];
      seq_new[t1_index] = seq_new[t2_index];
      seq_new[t2_index] = tmp;

      return seq_new;
    }
  }
}

OrderedTaskSequence reverse_subtour(const OrderedTaskSequence &seq) {
  uint start = rand() % seq.size();
  uint end = rand() % seq.size();

  while (start == end) {
    start = rand() % seq.size();
    end = rand() % seq.size();
  }

  if (start > end) {
    std::swap(start, end);
  }

  OrderedTaskSequence seq_new = seq;
  for (uint i = 0; i <= end - start; ++i) {
    seq_new[start + i] = seq[end - i];
  }

  return seq_new;
}

OrderedTaskSequence neighbour(const OrderedTaskSequence &seq,
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

bool sequence_is_feasible(const OrderedTaskSequence &seq,
                          const RobotTaskPoseMap &rtpm) {
  for (const auto &s : seq) {
    const Robot r = s.robots[0];
    const auto task_index = s.task.object;

    if (rtpm.at(s).size() == 0) {
      return false;
    }
  }

  return true;
}

void get_plan_from_cache() {}

Plan plan_multiple_arms_squeaky_wheel(rai::Configuration &C,
                                      const RobotTaskPoseMap &rtpm,
                                      const std::map<Robot, arr> &home_poses) {
  return {};
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
  for (const auto &element : home_poses) {
    robots.push_back(element.first);
  }
  // const uint num_tasks = rtpm.begin()->second.size();
  int num_tasks = 0;
  for (auto f : C.frames) {
    if (f->name.contains("obj")) {
      num_tasks += 1;
    }
  }

  OrderedTaskSequence best_seq;
  Plan best_plan;
  double best_makespan = 1e6;

  auto start_time = std::chrono::high_resolution_clock::now();

  std::vector<std::pair<OrderedTaskSequence, Plan>> cache;

  uint iter = 0;
  for (uint i = 0; i < 20000; ++i) {
    std::cout << "Generating completely new seq. " << i << std::endl;
    OrderedTaskSequence seq;
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
      OrderedTaskSequence new_seq = seq;
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
        std::cout << "(" << s.robots[0] << " " << s.task.object << ")";
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
          if (seq[k].robots[0] != new_seq[k].robots[0] ||
              seq[k].task.object != new_seq[k].task.object) {
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
        const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  end_time - start_time)
                                  .count();

        // cache.push_back(std::make_pair(new_seq, new_plan));

        export_plan(C, robots, home_poses, new_plan, new_seq, buffer.str(), iter,
                    duration);

        std::cout << "\n\n\nMAKESPAN " << makespan << " best so far "
                  << best_makespan << " (" << prev_makespan << ")" << std::endl;
        for (const auto &s : new_seq) {
          std::cout << "(" << s.robots[0] << " " << s.task.object << ")";
        }
        std::cout << "\n\n\n" << std::endl;

        if (makespan < prev_makespan) {
          seq = new_seq;
          plan = new_plan;
          prev_makespan = makespan;

          visualize_plan(C, plan);
          std::cout << "A" << std::endl;
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

Plan plan_multiple_arms_simulated_annealing(
    rai::Configuration C, const RobotTaskPoseMap &rtpm,
    const std::map<Robot, arr> &home_poses) {
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);
  std::stringstream buffer;
  buffer << "simulated_annealing_" << std::put_time(&tm, "%Y%m%d_%H%M%S");

  auto start_time = std::chrono::high_resolution_clock::now();

  // generate random sequence of robot/pt pairs
  std::vector<Robot> robots;
  for (const auto &element : home_poses) {
    robots.push_back(element.first);
  }
  // const uint num_tasks = rtpm.begin()->second.size();
  int num_tasks = 0;
  for (auto f : C.frames) {
    if (f->name.contains("obj")) {
      num_tasks += 1;
    }
  }
  auto seq = generate_random_sequence(robots, num_tasks);

  // plan for it
  const auto plan_result =
      plan_multiple_arms_given_sequence(C, rtpm, seq, home_poses);

  auto best_plan = plan_result.plan;
  uint best_makespan = get_makespan_from_plan(plan_result.plan);

  uint curr_makespan = best_makespan;

  std::cout << "Initial path with makespan " << best_makespan << std::endl
            << std::endl;

  {
    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time)
            .count();
    export_plan(C, robots, home_poses, best_plan, seq, buffer.str(), 0, duration);
  }

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
    const OrderedTaskSequence seq_new = neighbour(seq, robots);

    // compute lower bound
    const double lb_makespan =
        compute_lb_for_sequence(seq_new, rtpm, home_poses);

    arr rnd(1);
    rndUniform(rnd);

    if (p(curr_makespan, lb_makespan, T) > rnd(0)) {
      const auto new_plan_result =
          plan_multiple_arms_given_sequence(C, rtpm, seq_new, home_poses);

      if (new_plan_result.status == PlanStatus::success) {
        const auto end_time = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  end_time - start_time)
                                  .count();

        const Plan new_plan = new_plan_result.plan;
        const double makespan = get_makespan_from_plan(new_plan);

        export_plan(C, robots, home_poses, new_plan, seq_new, buffer.str(), i + 1,
                    duration);

        std::cout << "\n\n\nMAKESPAN " << makespan << " best so far "
                  << best_makespan << std::endl;
        for (const auto &s : seq_new) {
          std::cout << "(" << s.robots[0] << " " << s.task.object << ")";
        }

        if (p(curr_makespan, makespan, T) > rnd(0)) {
          curr_makespan = makespan;
          seq = seq_new;
        }

        if (makespan < best_makespan) {
          best_makespan = makespan;
          best_plan = new_plan;

          visualize_plan(C, best_plan);
        }
      }
    }

    best_makespan_at_iteration.push_back(best_makespan);
    computation_time_at_iteration.push_back(i);
  }

  return best_plan;
}

OrderedTaskSequence make_handover_sequence(const std::vector<Robot> &robots,
                                           const uint num_tasks,
                                           const RobotTaskPoseMap &rtpm) {
  OrderedTaskSequence seq;
  for (uint i = 0; i < num_tasks; ++i) {
    while (true) {
      // choose random action
      const uint a = rand() % 2;

      // choose random robot
      uint r1 = rand() % robots.size();
      uint r2 = rand() % robots.size();

      RobotTaskPair rtp;

      if (a == 0) {
        rtp.robots = {robots[r1]};
        rtp.task = Task{.object = i, .type = TaskType::pick};
      } else {
        rtp.robots = {robots[r1], robots[r2]};
        rtp.task = Task{.object = i, .type = TaskType::handover};
      }

      // check if the task is feasible with the chosen robot(s)
      if (rtpm.count(rtp) != 0) {
        seq.push_back(rtp);
        break;
      }
    }
  }

  return seq;
}

std::map<Robot, arr> get_robot_home_poses(rai::Configuration &C,
                                          const std::vector<Robot> &robots) {
  std::map<Robot, arr> poses;
  for (auto r : robots) {
    setActive(C, r);
    poses[r] = C.getJointState();
  }

  return poses;
}

void load_and_viz(rai::Configuration C, const bool pick_and_place) {
  arr path;

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

arr plan_line(rai::Configuration C, const rai::String &robot_prefix, const std::vector<arr> &pts){
  OptOptions options;
  options.stopIters = 100;
  options.damping = 1e-3;
  options.stopLineSteps = 5;

  const uint horizon_length = pts.size();

  KOMO komo;
  komo.setModel(C, true);
  komo.setTiming(1., horizon_length, 1, 2);
  komo.verbose = 0;
  komo.solver = rai::KS_sparse;

  komo.add_collision(true, .001, 1e1);
  komo.add_qControlObjective({}, 1, 1e1);

  // velocity 0
  komo.addObjective({0.0, 0.0}, FS_qItself, {}, OT_eq, {1e1}, {}, 1);

  komo.addObjective({1.0}, FS_qItself, {}, OT_eq, {1e1}, {}, 1);

  for (uint i = 0; i < pts.size(); ++i) {
    const arr pt =
        C["table"]->getPosition() + ARR(pts[i](0), pts[i](1), 0.075);
    double constr_time = 1. * i / horizon_length;

    komo.addObjective({constr_time - 0.01, constr_time + 0.01}, FS_position,
                      {STRING(robot_prefix << "pen_tip")}, OT_eq, {1e1}, pt);

    //komo.addObjective({constr_time - 0.01, constr_time + 0.01}, FS_vectorZ,
    //                  {STRING(robot_prefix << "pen")}, OT_sos, {1e0}, {0., 0., -1.});
  }

  komo.run_prepare(0.0);
  komo.run(options);
  komo.pathConfig.watch(true);

  arr path(horizon_length, C.getJointState().d0);
  for (uint j = 0; j < horizon_length; ++j) {
    path[j] = komo.getPath_q(j);
  }

  return path;
}

void line_test() {
  rai::Configuration C;
  labSetting(C);

  std::vector<arr> line;
  for (uint i = 0; i < 20; ++i) {
    line.push_back({-0.2 + 0.4 * i / 20., 0.1});
  }

  rai::String prefix = "a1_";

  const arr path = plan_line(C, prefix, line);

  for (uint i = 0; i < path.d0; ++i) {
    C.setJointState(path[i]);
    C.watch(true);
    rai::wait(0.01);
  }
}

// TODO:
// - improve handover sampler
// - constrained motion planning 

int main(int argc, char **argv) {
  rai::initCmdLine(argc, argv);
  const uint seed = rai::getParameter<double>("seed", 42); // seed
  rnd.seed(seed);

  const uint verbosity = rai::getParameter<double>(
      "verbosity", 0); // verbosity, does not do anything atm

  const bool plan_pick_and_place =
      rai::getParameter<bool>("pnp", false); // pick and place yes/no

  const uint num_objects =
      rai::getParameter<double>("objects", 5); // number of objects

  const double vmax =
      rai::getParameter<double>("vmax", 0.1); // does not do anything atm.

  // possible modes:
  // - benchmark
  // - test
  // - optimize
  // - show scenario
  // - show saved path
  const rai::String mode =
      rai::getParameter<rai::String>("mode", "test"); // scenario
  const rai::String stippling_scenario =
      rai::getParameter<rai::String>("stippling_pts", "random"); // scenario

  const rai::String env =
      rai::getParameter<rai::String>("env", ""); // environment

  std::vector<std::string> robots; // string-prefix for robots

  rai::Configuration C;
  if (plan_pick_and_place) {
    // pick_and_place(C);
    // robots = {"a0_", "a1_"};

    random_objects(C, num_objects);
    robots = {"a0_", "a1_", "a2_"};
  } else {
    if (env == "lab") {
      labSetting(C);
      robots = {"a0_", "a1_"};
    } else {
      more_robots(C, 4);
      robots = {"a0_", "a1_", "a2_", "a3_"};
    }
  }

  // {
  //   rai::Configuration tmp;
  //   random_objects(tmp, 10);
  //   tmp.watch(true);
  // }

  // {
  //   rai::Configuration tmp;
  //   side_by_side_robot_configuration(tmp);
  // }

  // maps [robot] to home_pose
  const std::map<Robot, arr> home_poses = get_robot_home_poses(C, robots);

  // show prev path
  if (mode == "show_plan") {
    load_and_viz(C, plan_pick_and_place);
    return 0;
  }

  if (mode == "line_test") {
    line_test();
    return 0;
  }

  if (mode == "handover_test"){
    RobotTaskPoseMap handover_rtpm = compute_handover_poses(C, robots);
    RobotTaskPoseMap pick_rtpm = compute_pick_and_place_positions(C, robots);

    // merge both maps
    RobotTaskPoseMap rtpm;
    rtpm.insert(pick_rtpm.begin(), pick_rtpm.end());
    rtpm.insert(handover_rtpm.begin(), handover_rtpm.end());

    const auto test_sequence_for_handover = make_handover_sequence(robots, num_objects, rtpm);
    auto plan = plan_multiple_arms_given_sequence(C, rtpm, test_sequence_for_handover, home_poses);
    visualize_plan(C, plan.plan, true);

  }

  // stippling
  RobotTaskPoseMap robot_task_pose_mapping;
  if (!plan_pick_and_place) {
    const arr pts = get_stippling_scenario(stippling_scenario);
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

    // rerun optimizer
    const auto optimized_plan = reoptimize_plan(C, plan, home_poses);
    visualize_plan(C, optimized_plan);

    // export plan
    // export_plan(robots, home_poses, new_plan, new_seq, buffer.str(), iter,
    //                duration);
  } else if (mode == "performance_benchmark") {
  } else if (mode == "optimization_benchmark") {
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
