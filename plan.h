#pragma once

#include <string>
#include <vector>

#include "json.h"

#include <Core/array.h>

#include "util.h"
#include "env_util.h"

using json = nlohmann::json;

enum class TaskType {pick, handover, go_to, joint_pick};
struct Task{
  unsigned int object;
  TaskType type;
};

typedef std::string Robot; // prefix of the robot -> fully identifies the robot

struct RobotTaskPair{
  std::vector<Robot> robots;
  Task task;

  bool operator==(const RobotTaskPair &other)const{
    if (robots.size() != other.robots.size()){
      return false;
    }
    if (task.type != other.task.type || task.object != other.task.object){
      return false;
    }

    for (uint i=0; i<robots.size(); ++i){
      if (robots[i] != other.robots[i]){
        return false;
      }
    }

    return true;
  }
};

template <>
struct std::hash<RobotTaskPair> {
  std::size_t operator()(RobotTaskPair const& rtp) const {
    std::size_t seed = rtp.robots.size();
    std::hash<std::string> hasher;
    for(auto& i : rtp.robots) {
      seed ^= hasher(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    seed ^= std::size_t(rtp.task.type) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= rtp.task.object + 0x9e3779b9 + (seed << 6) + (seed >> 2);

    return seed;
  }
};

typedef std::vector<arr> TaskPoses;
typedef std::unordered_map<RobotTaskPair, std::vector<TaskPoses>> RobotTaskPoseMap;

// typedef std::pair<Robot, Task> robot_task_pair;
typedef std::vector<RobotTaskPair> OrderedTaskSequence;
typedef std::map<Robot, std::vector<Task>> UnorderedTaskSequence;

struct ComputeStatistics {
  double total_compute_time;

  double rrt_compute_time;
  double rrt_plan_time;
  double rrt_nn_time;
  double rrt_coll_time;
  double rrt_shortcut_time;
  double rrt_smoothing_time;

  double komo_compute_time;
};

// this is the solution of one task
// TODO: add constraints that this solution needs to fulfill
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

  ComputeStatistics stats;
};

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

double
get_makespan_from_plan(const std::map<Robot, std::vector<TaskPart>> &plan) {
  double max_time = 0.;
  for (const auto &robot_plan : plan) {
    const auto last_subpath = robot_plan.second.back();
    max_time = std::max({last_subpath.t(-1), max_time});
  }

  return max_time;
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

std::map<Robot, std::vector<TaskPart>>
reoptimize_plan(rai::Configuration C,
                const std::map<Robot, std::vector<TaskPart>> &unscaled_plan,
                const std::map<Robot, arr> &home_poses) {
  // formulate KOMO problem for multiple robots
  std::vector<std::string> all_robots;
  for (const auto &per_robot_plan : unscaled_plan) {
    all_robots.push_back(per_robot_plan.first);
  }

  for (const auto &element : all_robots) {
    std::cout << element << std::endl;
  }

  setActive(C, all_robots);

  rai::Animation A;
  for (const auto &p : unscaled_plan) {
    for (const auto &path : p.second) {
      A.A.append(path.anim);
    }
  }

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
  std::map<Robot, StringA> per_robot_joints;
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

  std::map<Robot, arr> per_robot_paths;
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

  std::map<Robot, std::vector<TaskPart>> optimized_plan;

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

void export_plan(rai::Configuration C, const std::vector<Robot> &robots,
                 const std::map<Robot, arr> &home_poses, const Plan &plan,
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