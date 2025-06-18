#include <deque>
#include <fstream>
#include <iomanip>
#include <numeric>

#include <algorithm>
#include <chrono>
#include <random>

#include <zmq.h>

#include <math.h>

#include <KOMO/komo.h>
#include <Kin/F_operators.h>
#include <Kin/F_pose.h>
#include <Kin/featureSymbols.h>
#include <Kin/kin.h>

#include <Kin/kinViewer.h>
#include <Kin/viewer.h>

#include <Manip/rrt-time.h>
#include <PlanningSubroutines/ConfigurationProblem.h>

#include <GL/gl.h>
#include <Gui/opengl.h>

#include <PlanningSubroutines/Animation.h>
#include <PlanningSubroutines/ConfigurationProblem.h>

// #include "stippling.h"

#include "searchers/annealing_searcher.h"
#include "searchers/greedy_random_searcher.h"
#include "searchers/random_searcher.h"
#include "searchers/sequencing.h"
#include "searchers/squeaky_wheel_searcher.h"

#include "planners/optimal_planner.h"
#include "planners/plan.h"
#include "planners/planner.h"
#include "planners/postprocessing.h"
#include "planners/prioritized_planner.h"

#include "samplers/sampler.h"

#include "tests/benchmark.h"
#include "tests/perf_test.h"
#include "tests/test.h"

#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

#include "common/config.h"
#include "common/env_util.h"
#include "common/util.h"

manip::Parameters global_params;

// PlanResult plan_multiple_arms_jointly(rai::Configuration C, const
// RobotTaskPoseMap &rtpm,
//     const OrderedTaskSequence &sequence, const std::unordered_map<Robot, arr>
//     &home_poses){
//   CompoundTreePlanner ctp(C, sequence);
//   auto plan = ctp.plan();
//   return PlanResult(PlanStatus::failed);
// }

Plan plan_multiple_arms_unsynchronized(
    rai::Configuration &C, const RobotTaskPoseMap &rtpm,
    const std::unordered_map<Robot, arr> &home_poses) {
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

  const auto seq =
      generate_alternating_random_sequence(robots, num_tasks, rtpm);

  // plan for it
  const auto plan_result =
      plan_multiple_arms_given_sequence(C, rtpm, seq, home_poses);
  return plan_result.plan;
}

arr plan_line(rai::Configuration C, const rai::String &robot_prefix,
              const std::vector<arr> &pts) {
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
    const arr pt = C["table"]->getPosition() + ARR(pts[i](0), pts[i](1), 0.075);
    double constr_time = 1. * i / horizon_length;

    komo.addObjective({constr_time - 0.01, constr_time + 0.01}, FS_position,
                      {STRING(robot_prefix << "pen_tip")}, OT_eq, {1e1}, pt);

    // komo.addObjective({constr_time - 0.01, constr_time + 0.01}, FS_vectorZ,
    //                   {STRING(robot_prefix << "pen")}, OT_sos, {1e0}, {0.,
    //                   0., -1.});
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
  tub_lab_setting(C);

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

OrderedTaskSequence make_pick_pick_seq(const std::vector<Robot> &robots,
                                       const uint num_objects,
                                       const RobotTaskPoseMap &rtpm) {

  OrderedTaskSequence seq;

  std::vector<std::deque<RobotTaskPair>> rtps;

  for (uint i = 0; i < num_objects; ++i) {
    // check which robot combinations are available for a given object
    std::vector<std::pair<Robot, Robot>> available_robots;
    for (auto rtp : rtpm) {
      auto robots = rtp.first.robots;
      if (rtp.first.task.object == i) {
        available_robots.push_back(std::make_pair(robots[0], robots[1]));
      }
    }

    if (available_robots.size() == 0) {
      spdlog::error("No robot available for manipulating object {}", i);
      continue;
    }

    const uint ind = std::rand() % available_robots.size();
    spdlog::info("Using the {} available robot for pickpick", ind);

    // choose a combo randomly
    for (auto &rtp : rtpm) {
      auto robots = rtp.first.robots;
      if (rtp.first.task.object == i &&
          robots[0] == available_robots[ind].first &&
          robots[1] == available_robots[ind].second &&
          rtp.first.task.type == PrimitiveType::pick_pick_1) {
        rtps.push_back({});
        rtps.back().push_back(rtp.first);
        // seq.push_back(rtp.first);
      }
    }

    for (auto &rtp : rtpm) {
      auto robots = rtp.first.robots;
      if (rtp.first.task.object == i &&
          robots[0] == available_robots[ind].first &&
          robots[1] == available_robots[ind].second &&
          rtp.first.task.type == PrimitiveType::pick_pick_2) {
        // seq.push_back(rtp.first);
        rtps.back().push_back(rtp.first);
      }
    }
  }

  // more efficient: create vector with each obj-index twice, shuffle, done
  while (true) {
    const uint obj = std::rand() % rtps.size();

    if (rtps[obj].size() > 0) {
      seq.push_back(rtps[obj].front());
      rtps[obj].pop_front();

      std::cout << obj << std::endl;
    }

    bool done = true;
    for (uint i = 0; i < rtps.size(); ++i) {
      if (rtps[i].size() != 0) {
        done = false;
      }
    }

    if (done) {
      break;
    }
  }

  return seq;
}

RobotTaskPoseMap
compute_keyframes(rai::Configuration &C, const std::vector<Robot> &robots,
                  const bool use_picks = true, const bool use_handovers = true,
                  const bool use_repeated_picks = true,
                  const bool attempt_all_grasp_directions = false) {
  RobotTaskPoseMap robot_task_pose_mapping;

  if (use_picks) {
    RobotTaskPoseMap pick_rtpm = compute_all_pick_and_place_positions(
        C, robots, attempt_all_grasp_directions);
    robot_task_pose_mapping.insert(pick_rtpm.begin(), pick_rtpm.end());
  }
  if (use_handovers) {
    RobotTaskPoseMap handover_rtpm =
        compute_all_handover_poses(C, robots, attempt_all_grasp_directions);
    robot_task_pose_mapping.insert(handover_rtpm.begin(), handover_rtpm.end());
  }
  if (use_repeated_picks) {
    RobotTaskPoseMap pick_pick_rtpm =
        compute_all_pick_and_place_with_intermediate_pose(
            C, robots, attempt_all_grasp_directions);
    robot_task_pose_mapping.insert(pick_pick_rtpm.begin(),
                                   pick_pick_rtpm.end());
  }

  return robot_task_pose_mapping;
}

void export_keyframes() {}

void set_to_mode_for_primitive(rai::Configuration &C, RobotTaskPair rtp,
                               TaskPoses poses, const uint phase) {
  if (rtp.task.type == PrimitiveType::handover) {
    for (uint i = 0; i < phase; ++i) {
      const auto pose = poses[i];

      // set all robots to their home-pose
      for (const auto &r : rtp.robots) {
        setActive(C, r);
        C.setJointState(r.home_pose);
      }

      if (i == 0) {
        setActive(C, rtp.robots[0]);
        C.setJointState(pose);

        const auto ee_frame_name =
            STRING(rtp.robots[0] << rtp.robots[0].ee_frame_name);
        const auto obj = STRING("obj" << rtp.task.object + 1);

        auto from = C[ee_frame_name];
        auto to = C[obj];

        to->unLink();

        // create a new joint
        to->linkFrom(from, true);
      } else if (i == 1) {
        setActive(C, rtp.robots);
        C.setJointState(pose);

        const auto ee_frame_name =
            STRING(rtp.robots[1] << rtp.robots[0].ee_frame_name);
        const auto obj = STRING("obj" << rtp.task.object + 1);

        auto from = C[ee_frame_name];
        auto to = C[obj];

        to->unLink();

        // create a new joint
        to->linkFrom(from, true);
      } else if (i == 2) {
        setActive(C, rtp.robots[1]);
        C.setJointState(pose);

        const auto obj = STRING("obj" << rtp.task.object + 1);

        auto from = C["table_base"];
        auto to = C[obj];

        to->unLink();

        // create a new joint
        to->linkFrom(from, true);
      }

      // C.watch(true);
    }
  } else if (rtp.task.type == PrimitiveType::pick) {
    for (uint i = 0; i < phase; ++i) {
      const auto pose = poses[i];

      // set all robots to their home-pose
      for (const auto &r : rtp.robots) {
        setActive(C, r);
        C.setJointState(r.home_pose);
      }

      if (i == 0) {
        setActive(C, rtp.robots[0]);
        C.setJointState(pose);

        const auto ee_frame_name =
            STRING(rtp.robots[0] << rtp.robots[0].ee_frame_name);
        const auto obj = STRING("obj" << rtp.task.object + 1);

        auto from = C[ee_frame_name];
        auto to = C[obj];

        to->unLink();

        // create a new joint
        to->linkFrom(from, true);
      } else if (i == 1) {
        setActive(C, rtp.robots[0]);
        C.setJointState(pose);

        const auto obj = STRING("obj" << rtp.task.object + 1);

        auto from = C["table_base"];
        auto to = C[obj];

        to->unLink();

        // create a new joint
        to->linkFrom(from, true);
      }
    }
  }
}

void export_scene_at_keyframes(
    rai::Configuration &C, const std::vector<Robot> &robots,
    const RobotTaskPoseMap &robot_task_pose_mapping) {
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);

  std::stringstream buffer;
  buffer << "keyframe_generation_" << std::put_time(&tm, "%Y%m%d_%H%M%S");

  const std::string folder = global_params.output_path + buffer.str() + "/";
  const int res = system(STRING("mkdir -p " << folder).p);
  (void)res;

  uint cnt = 0;
  for (const auto &robot_task_pair : robot_task_pose_mapping) {
    const RobotTaskPair rtp = robot_task_pair.first;
    const TaskPoses poses = robot_task_pair.second[0];

    for (uint i = 0; i < poses.size(); ++i) {
      const auto pose = poses[i];
      std::cout << pose << std::endl;

      rai::Configuration c_copy = C;

      set_to_mode_for_primitive(c_copy, rtp, poses, i + 1);
      c_copy.watch(true);

      // export json description of scene
      const auto data = make_scene_data(c_copy, robots);

      save_json(data, folder + "scene" + std::to_string(cnt) + ".json",
                global_params.compress_data);
      ++cnt;
    }
  }
}

bool check_scene_validity(rai::Configuration C,
                          const std::vector<Robot> &robots) {
  for (const auto &r : robots) {
    setActive(C, r);
    ConfigurationProblem cp(C);

    const bool is_current_pose_valid = cp.query({}, false)->isFeasible;
    if (!is_current_pose_valid) {
      spdlog::info("The startpose of robot {} seems to be invalid", r.prefix);
      return false;
    }

    const bool is_home_pose_valid = cp.query(r.home_pose, true)->isFeasible;
    if (!is_home_pose_valid) {
      spdlog::info("The homepose of robot {} seems to be invalid", r.prefix);
      return false;
    }
  }

  // TODO: add a check for all the mode switches

  return true;
}

// TODO:
// - constrained motion planning

// this is to suppress a bunch of stacktracing
extern "C" int backtrace(void **buffer, int size) {
  return 0; // Prevent stack trace generation
}

int main(int argc, char **argv) {
  auto console = spdlog::stdout_color_mt("console");

  rai::initCmdLine(argc, argv);
  const uint seed = rai::getParameter<double>("seed", 42); // seed
  rnd.seed(seed);
  std::srand(seed);

  const bool suppress_errors = rai::getParameter<bool>("suppress_errors", true);
  if (suppress_errors) {
    freopen("/dev/null", "w",
            stderr); // Redirects stderr to /dev/null (Linux/Unix systems)
  }

  const bool display = rai::getParameter<bool>("display", false);
  global_params.allow_display = display;

  const bool export_images = rai::getParameter<bool>("export_images", false);
  global_params.export_images = export_images;

  const bool check_scene_validity_flag =
      rai::getParameter<bool>("check_scene_validity", false);

  const bool allow_early_stopping =
      rai::getParameter<bool>("early_stopping", false);
  global_params.use_early_coll_check_stopping = allow_early_stopping;

  const uint verbosity = rai::getParameter<double>("verbosity", 2);

  const rai::String mode = rai::getParameter<rai::String>(
      "mode", "two_finger_keyframes_test"); // mode

  const rai::String stippling_scenario =
      rai::getParameter<rai::String>("stippling_pts", "random"); // scenario

  const rai::String gripper =
      rai::getParameter<rai::String>("gripper", "two_finger"); // which gripper

  const rai::String scene_path = rai::getParameter<rai::String>(
      "scene_path", "./in/scenes/floor.g"); // base_scene

  const rai::String obstacle_path =
      rai::getParameter<rai::String>("obstacle_path", ""); // obstacles

  const rai::String obj_path =
      rai::getParameter<rai::String>("obj_path", "random"); // objects

  const uint num_objects_for_env =
      rai::getParameter<double>("objects", 5); // number of objects

  const rai::String robot_path = rai::getParameter<rai::String>(
      "robot_path", "./in/envs/two_opposite.json"); // robot placement and pose

  const rai::String sequence_path = rai::getParameter<rai::String>(
      "sequence_path", "./in/sequences/test.json"); // sequence

  const rai::String output_path =
      rai::getParameter<rai::String>("output_path", "./out/");
  global_params.output_path = std::string(output_path.p);

  const bool attempt_all_grasp_directions =
      rai::getParameter<bool>("attempt_all_grasp_directions", false);

  const bool use_picks = rai::getParameter<bool>("use_simple_picks", true);

  const bool use_handovers = rai::getParameter<bool>("use_handovers", true);

  const bool use_repeated_picks =
      rai::getParameter<bool>("use_repeated_picks", true);

  const uint max_attempts = rai::getParameter<double>("max_attempts", 1000);

  const bool randomize_mode_switch_duration =
      rai::getParameter<bool>("randomize_mode_switch_duration", false);
  global_params.randomize_mod_switch_durations = randomize_mode_switch_duration;

  const bool avoid_repeated_evaluations =
      rai::getParameter<bool>("avoid_repeated_evaluations", true);

  const bool compress_output =
      rai::getParameter<bool>("compress_output", false);
  global_params.compress_data = compress_output;

  const bool export_txt_files =
      rai::getParameter<bool>("export_txt_files", false);
  global_params.export_txt_files = export_txt_files;

  switch (verbosity) {
  case 0:
    spdlog::set_level(spdlog::level::off);
    break;
  case 1:
    spdlog::set_level(spdlog::level::err);
    break;
  case 2:
    spdlog::set_level(spdlog::level::warn);
    break;
  case 3:
    spdlog::set_level(spdlog::level::info);
    break;
  case 4:
    spdlog::set_level(spdlog::level::debug);
    break;
  case 5:
    spdlog::set_level(spdlog::level::trace);
    break;
  default:
    break;
  }

  if (mode == "benchmark_single_keyframe") {
    benchmark_single_arm_pick_and_place_success_rate(false, false);
    return 0;
  }

  if (mode == "benchmark_handover") {
    benchmark_dual_arm_handover_success_rate(false, false);
    return 0;
  }

  if (mode == "benchmark_pick_pick") {
    benchmark_dual_arm_pick_pick_success_rate(false, false);
    return 0;
  }

  if (mode == "benchmark_pick_pick_planning") {
    benchmark_dual_arm_multi_pick_planning();
    return 0;
  }

  if (mode == "benchmark_planning") {
    benchmark_dual_arm_planning();
    return 0;
  }

  if (mode == "perf_test") {
    vacuum_gripper_keyframe_perf_test();
    return 0;
  }

  if (mode == "run_test_folder") {
    run_all_test_problems_from_folder("./in/test_problems/");
    return 0;
  }

  if (mode == "two_finger_planning_test" || mode == "run_all_tests") {
    // single_arm_two_finger_planning_test(display);
    two_arm_two_finger_planning_test(display);
    three_arm_two_finger_planning_test(display);

    if (mode != "run_all_tests") {
      return 0;
    }
  }

  if (mode == "two_finger_handover_planning_test" || mode == "run_all_tests") {
    two_arm_two_finger_handover_planning_test(display);
    three_arm_two_finger_handover_planning_test(display);

    return 0;
  }

  rai::Configuration C;
  std::vector<Robot> robots;

  if (false) {
    // TODO: implement, make sure that not both things can be active
    // robots = make_env_from_file(C, full_scene_path.p);
  } else {
    robots = make_robot_environment_from_config(C, robot_path.p, scene_path.p);
    add_obstacles_from_config(C, obstacle_path.p);

    if (obj_path == "random") {
      random_objects(C, num_objects_for_env);
    } else if (obj_path == "line") {
      line(C, num_objects_for_env);
    } else if (obj_path == "shuffled_line") {
      shuffled_line(C, num_objects_for_env, 1.5, false);
    } else if (obj_path == "big_objs") {
      big_objs(C, num_objects_for_env);
    } else {
      add_objects_from_config(C, obj_path.p);
    }
  }

  int num_objects = 0;
  for (auto f : C.frames) {
    if (f->name.contains("obj")) {
      num_objects += 1;
    }
  }

  if (mode == "show_env") {
    C.watch(true);
    return 0;
  }

  if (mode == "show_env_at_home_pose") {
    C.watch(true);
    for (const auto &r : robots) {
      setActive(C, r);
      auto q = r.home_pose;
      C.setJointState(q);
    }
    C.watch(true);

    return 0;
  }

  if (mode == "show_env_and_move_manipulators") {
    C.watch(true);

    for (const auto &r : robots) {
      setActive(C, r);
      auto q = C.getJointState();
      const arr lims = C.getLimits();

      for (uint i = 0; i < q.d0; ++i) {
        double ub = lims(i, 1);
        double lb = lims(i, 0);

        if (lims(i, 1) < lims(i, 0)) {
          ub = 5;
          lb = -5;
        }

        const auto q_i_init = q(i);
        for (uint j = 0; j < 100; ++j) {
          q(i) = (ub - lb) * (cos(j * 3.1415 * 2 / (100 - 1)) / 2 + 0.5) + lb;
          C.setJointState(q);

          C.watch(false);
          rai::wait(0.01);
        }

        q(i) = q_i_init;
        C.setJointState(q);
      }
    }

    return 0;
  }

  // maps [robot] to home_pose
  const std::unordered_map<Robot, arr> home_poses =
      get_robot_home_poses(robots);

  if (mode == "line_test") {
    line_test();
    return 0;
  }

  if (mode == "repeated_pick") {
    compute_all_pick_and_place_with_intermediate_pose(
        C, robots, attempt_all_grasp_directions);
    return 0;
  }

  if (mode == "repeated_pick_planning") {
    const auto rtpm = compute_all_pick_and_place_with_intermediate_pose(
        C, robots, attempt_all_grasp_directions);
    const auto test_sequence_for_repeated_manip =
        make_pick_pick_seq(robots, num_objects, rtpm);

    auto plan = plan_multiple_arms_given_sequence(
        C, rtpm, test_sequence_for_repeated_manip, home_poses);

    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);

    std::stringstream buffer;
    buffer << "repeated_pick_test_" << std::put_time(&tm, "%Y%m%d_%H%M%S");

    export_plan(C, robots, home_poses, plan.plan,
                test_sequence_for_repeated_manip, buffer.str(), 0, 0);

    if (global_params.export_images) {
      const std::string image_path = global_params.output_path + buffer.str() +
                                     "/" + std::to_string(0) + "/img/";
      visualize_plan(C, plan.plan, global_params.allow_display, image_path);
    } else {
      visualize_plan(C, plan.plan, global_params.allow_display);
    }

    return 0;
  }

  if (mode == "handover_test") {
    RobotTaskPoseMap handover_rtpm =
        compute_all_handover_poses(C, robots, attempt_all_grasp_directions);
    RobotTaskPoseMap pick_rtpm = compute_all_pick_and_place_positions(
        C, robots, attempt_all_grasp_directions);

    // merge both maps
    RobotTaskPoseMap rtpm;
    rtpm.insert(pick_rtpm.begin(), pick_rtpm.end());
    rtpm.insert(handover_rtpm.begin(), handover_rtpm.end());

    const auto test_sequence_for_handover =
        make_handover_sequence(robots, num_objects, rtpm);
    auto plan = plan_multiple_arms_given_sequence(
        C, rtpm, test_sequence_for_handover, home_poses);
    visualize_plan(C, plan.plan, true);

    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);

    std::stringstream buffer;
    buffer << "handover_test_" << std::put_time(&tm, "%Y%m%d_%H%M%S");

    export_plan(C, robots, home_poses, plan.plan, test_sequence_for_handover,
                buffer.str(), 0, 0);

    return 0;
  }

  // check validity of the environment
  if (check_scene_validity_flag && !check_scene_validity(C, robots)) {
    spdlog::warn("Scene is not feasible as is - use 'show_env' or "
                 "'show_env_at_home_pose' for visual inspection.");
    return 0;
  }

  if (mode == "plan_for_sequence") {
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);

    std::stringstream buffer;
    buffer << "sequence_plan_" << std::put_time(&tm, "%Y%m%d_%H%M%S");

    RobotTaskPoseMap rtpm =
        compute_keyframes(C, robots, use_picks, use_handovers,
                          use_repeated_picks, attempt_all_grasp_directions);

    const auto start_time = std::chrono::high_resolution_clock::now();

    const std::string path = sequence_path.p;
    const auto sequences = load_sequences_from_file(path, robots);

    uint seq_num = 0;
    for (const auto &seq : sequences) {
      std::cout << ordered_sequence_to_str(seq) << std::endl;

      if (!check_sequence_validity(seq, rtpm)) {
        spdlog::error("sequence invalid.");
        return 0;
      }

      const PlanResult plan =
          plan_multiple_arms_given_sequence(C, rtpm, seq, home_poses);

      const auto end_time = std::chrono::high_resolution_clock::now();
      const auto duration =
          std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
                                                                start_time)
              .count();

      if (plan.status == PlanStatus::success) {
        export_plan(C, robots, home_poses, plan.plan, seq, buffer.str(),
                    seq_num, duration);

        if (global_params.export_images) {
          const std::string image_path = global_params.output_path +
                                         buffer.str() + "/" +
                                         std::to_string(0) + "/img/";
          visualize_plan(C, plan.plan, global_params.allow_display, image_path);
        } else {
          visualize_plan(C, plan.plan, global_params.allow_display);
        }
      } else {
        spdlog::warn("No solution found for given sequence.");
      }

      ++seq_num;
    }

    return 0;
  }

  if (mode == "compute_keyframes") {
    RobotTaskPoseMap robot_task_pose_mapping =
        compute_keyframes(C, robots, use_picks, use_handovers,
                          use_repeated_picks, attempt_all_grasp_directions);
    spdlog::info("{} poses computed.", robot_task_pose_mapping.size());

    // TODO: export?
    export_scene_at_keyframes(C, robots, robot_task_pose_mapping);

    return 0;
  }

  if (mode == "generate_candidate_sequences") {
    // make foldername for current run
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);

    std::stringstream buffer;
    buffer << "sequence_generation_" << std::put_time(&tm, "%Y%m%d_%H%M%S");

    RobotTaskPoseMap robot_task_pose_mapping =
        compute_keyframes(C, robots, use_picks, use_handovers,
                          use_repeated_picks, attempt_all_grasp_directions);

    int num_tasks = 0;
    for (auto f : C.frames) {
      if (f->name.contains("obj")) {
        num_tasks += 1;
      }
    }

    const uint num_attempts = 100;

    const std::string folder = global_params.output_path + buffer.str() + "/";
    const int res = system(STRING("mkdir -p " << folder).p);
    (void)res;

    std::unordered_set<OrderedTaskSequence> all_sequences;

    json sequences;

    uint cnt = 0;
    for (uint i = 0; i < num_attempts; ++i) {
      const auto seq = generate_random_valid_sequence(robots, num_tasks,
                                                      robot_task_pose_mapping);

      // check if the sequence was already evaluated at some point
      if (avoid_repeated_evaluations && all_sequences.count(seq) > 0) {
        continue;
      }
      all_sequences.insert(seq);

      // export sequence
      const auto data = ordered_sequence_to_json(seq);
      sequences["sequences"].push_back(data);

      // save_json(data, folder + "sequence_" + std::to_string(cnt) + ".json",
      // global_params.compress_data);

      ++cnt;
    }

    save_json(sequences, folder + "sequences.json",
              global_params.compress_data);

    return 0;
  }

  spdlog::info("Computing pick and place poses");

  // merge both maps
  RobotTaskPoseMap robot_task_pose_mapping =
      compute_keyframes(C, robots, use_picks, use_handovers, use_repeated_picks,
                        attempt_all_grasp_directions);
  spdlog::info("{} poses computed.", robot_task_pose_mapping.size());

  // initial test
  if (mode == "test") {
    const auto plan = plan_multiple_arms_unsynchronized(
        C, robot_task_pose_mapping, home_poses);
    std::cout << "Makespan: " << get_makespan_from_plan(plan) << std::endl;

    visualize_plan(C, plan);

    // rerun optimizer
    // const auto optimized_plan = reoptimize_plan(C, plan, home_poses);
    // visualize_plan(C, optimized_plan);

    // export plan
    // export_plan(robots, home_poses, new_plan, new_seq, buffer.str(), iter,
    //                duration);
  } else if (mode == "performance_benchmark") {
  } else if (mode == "optimization_benchmark") {
  } else if (mode == "random_search") {
    // random search
    const auto plan = plan_multiple_arms_random_search(
        C, robot_task_pose_mapping, home_poses, max_attempts,
        avoid_repeated_evaluations);
  } else if (mode == "greedy_random_search") {
    // greedy random search
    const auto plan = plan_multiple_arms_greedy_random_search(
        C, robot_task_pose_mapping, home_poses, max_attempts);
  } else if (mode == "simulated_annealing") {
    plan_multiple_arms_simulated_annealing(C, robot_task_pose_mapping,
                                           home_poses);
  }

  return 0;
}
