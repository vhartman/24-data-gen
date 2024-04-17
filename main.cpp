#include <deque>
#include <fstream>
#include <iomanip>
#include <numeric>

#include <algorithm>
#include <chrono>
#include <random>

#include <math.h>

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

#include <GL/gl.h>
#include <Gui/opengl.h>

#include <PlanningSubroutines/Animation.h>
#include <PlanningSubroutines/ConfigurationProblem.h>

#include "plan.h"

#include "env_util.h"
#include "planners/path_util.h"
// #include "stippling.h"
#include "util.h"

#include "searchers/annealing_searcher.h"
#include "searchers/greedy_random_searcher.h"
#include "searchers/random_searcher.h"
#include "searchers/sequencing.h"

#include "planners/optimal_planner.h"
#include "planners/planner.h"
#include "planners/prioritized_planner.h"

#include "samplers/sampler.h"
#include "tests/perf_test.h"
#include "tests/test.h"
#include "tests/benchmark.h"

#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

#include "config.h"

manip::Parameters global_params;

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

// PlanResult plan_multiple_arms_jointly(rai::Configuration C, const RobotTaskPoseMap &rtpm,
//     const OrderedTaskSequence &sequence, const std::unordered_map<Robot, arr> &home_poses){
//   CompoundTreePlanner ctp(C, sequence);
//   auto plan = ctp.plan();
//   return PlanResult(PlanStatus::failed);
// }

Plan plan_multiple_arms_unsynchronized(rai::Configuration &C,
                                       const RobotTaskPoseMap &rtpm,
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

  const auto seq = generate_alternating_random_sequence(robots, num_tasks, rtpm);

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

Plan plan_multiple_arms_squeaky_wheel(rai::Configuration &C,
                                      const RobotTaskPoseMap &rtpm,
                                      const std::unordered_map<Robot, arr> &home_poses) {
  return {};
}

bool check_sequence_validity(const OrderedTaskSequence &seq, const RobotTaskPoseMap &rtpm){
  uint cnt = 0;
  for (const auto &rtp: seq){
    if (rtpm.count(rtp) == 0){
      spdlog::warn("No task pose found for action {}", cnt);
      return false;
    }

    ++cnt;
  }
  return true;
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
    for (auto rtp: rtpm){
      auto robots = rtp.first.robots;
      if (rtp.first.task.object == i){
        available_robots.push_back(std::make_pair(robots[0], robots[1]));
      }
    }

    if (available_robots.size() == 0){
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
  while(true){
    const uint obj = std::rand() % rtps.size();

    if (rtps[obj].size() > 0){
      seq.push_back(rtps[obj].front());
      rtps[obj].pop_front();

      std::cout << obj << std::endl;
    }

    bool done = true;
    for (uint i=0; i<rtps.size(); ++i){
      if (rtps[i].size() != 0){
        done = false;
      }
    }

    if (done){break;}
  }

  return seq;
}

// TODO:
// - constrained motion planning 

int main(int argc, char **argv) {
  auto console = spdlog::stdout_color_mt("console");    

  rai::initCmdLine(argc, argv);
  const uint seed = rai::getParameter<double>("seed", 42); // seed
  rnd.seed(seed);
  std::srand(seed);

  const bool display = rai::getParameter<bool>("display", false);
  global_params.allow_display = display;
  
  const bool export_images =
      rai::getParameter<bool>("export_images", false);
  global_params.export_images = export_images;

  const bool allow_early_stopping =
      rai::getParameter<bool>("early_stopping", false);
  global_params.use_early_coll_check_stopping = allow_early_stopping;

  const uint verbosity = rai::getParameter<double>(
      "verbosity", 2);

  const rai::String mode =
      rai::getParameter<rai::String>("mode", "two_finger_keyframes_test"); // mode

  const rai::String stippling_scenario =
      rai::getParameter<rai::String>("stippling_pts", "random"); // scenario

  const rai::String gripper =
      rai::getParameter<rai::String>("gripper", "two_finger"); // which gripper

  const rai::String env =
      rai::getParameter<rai::String>("env", "random"); // environment

  const uint num_objects_for_env =
      rai::getParameter<double>("objects", 5); // number of objects

  const rai::String robot_env =
      rai::getParameter<rai::String>("robot_env", "./in/envs/two_opposite.json"); // environment

  const rai::String sequence_path =
      rai::getParameter<rai::String>("sequence_path", "./in/sequences/test.json"); // environment

  const rai::String output_path =
      rai::getParameter<rai::String>("output_path", "./out/");
  global_params.output_path = std::string(output_path.p);

  const bool use_picks = 
      rai::getParameter<bool>("use_simple_picks", true);

  const bool use_handovers = 
      rai::getParameter<bool>("use_handovers", true);

  const bool use_repeated_picks = 
      rai::getParameter<bool>("use_repeated_picks", true);

  const uint max_attempts = rai::getParameter<double>(
      "max_attempts", 1000);

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

  if (mode == "benchmark_single_keyframe"){
    benchmark_single_arm_pick_and_place_success_rate(false, false);
    return 0;
  }

  if (mode == "benchmark_handover"){
    benchmark_dual_arm_handover_success_rate(false, false);
    return 0;
  }

  if (mode=="benchmark_pick_pick"){
    benchmark_dual_arm_pick_pick_success_rate(false, false);
    return 0;
  }

  if (mode == "perf_test"){
    vacuum_gripper_keyframe_perf_test();
    return 0;
  }

  if (mode == "run_test_folder"){
    run_all_test_problems_from_folder("./in/test_problems/");
    return 0;
  }

  if (mode == "two_finger_keyframes_test" || mode == "run_all_tests") {
    single_arm_two_finger_keyframe_test(display);
    two_arms_two_finger_keyframe_test(display);
    three_arms_two_finger_keyframe_test(display);

    if (mode != "run_all_tests") {
      return 0;
    }
  }

  if (mode == "two_finger_handover_keyframes_test" || mode == "run_all_tests") {
    two_arm_two_finger_handover_keyframe_test(display);
    three_arm_two_finger_handover_keyframe_test(display);

    if (mode != "run_all_tests") {
      return 0;
    }
  }

  if (mode == "two_finger_planning_test" || mode == "run_all_tests") {
    single_arm_two_finger_planning_test(display);
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
  auto robots = make_robot_environment_from_config(C, robot_env.p);

  if (env == "random"){
    random_objects(C, num_objects_for_env);
  }
  else if (env == "line"){
    line(C, num_objects_for_env);
  }
  else if (env == "shuffled_line"){
    shuffled_line(C, num_objects_for_env, 1.5, false);
  }
  else if (env == "big_objs"){
    big_objs(C, num_objects_for_env);
  }
  else{
    add_objects_from_config(C, env.p);
  }

  int num_objects = 0;
  for (auto f : C.frames) {
    if (f->name.contains("obj")) {
      num_objects += 1;
    }
  }

  if (mode == "show_env"){
    C.watch(true);
    return 0;
  }

  // maps [robot] to home_pose
  const std::unordered_map<Robot, arr> home_poses = get_robot_home_poses(robots);

  if (mode == "line_test") {
    line_test();
    return 0;
  }

  if (mode == "repeated_pick"){
    compute_all_pick_and_place_with_intermediate_pose(C, robots);
    return 0;
  }

  if (mode == "repeated_pick_planning"){
    const auto rtpm = compute_all_pick_and_place_with_intermediate_pose(C, robots);
    const auto test_sequence_for_repeated_manip = make_pick_pick_seq(robots, num_objects, rtpm);

    auto plan = plan_multiple_arms_given_sequence(C, rtpm, test_sequence_for_repeated_manip, home_poses);

    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);

    std::stringstream buffer;
    buffer << "repeated_pick_test_" << std::put_time(&tm, "%Y%m%d_%H%M%S");

    export_plan(C, robots, home_poses, plan.plan, test_sequence_for_repeated_manip, buffer.str(), 0, 0);
    
    if (global_params.export_images){
      const std::string image_path = global_params.output_path + buffer.str() + "/" + std::to_string(0) + "/img/";
      visualize_plan(C, plan.plan, global_params.allow_display, image_path);
    }
    else{
      visualize_plan(C, plan.plan, global_params.allow_display);
    }

    return 0;
  }

  if (mode == "handover_test"){
    RobotTaskPoseMap handover_rtpm = compute_all_handover_poses(C, robots);
    RobotTaskPoseMap pick_rtpm = compute_all_pick_and_place_positions(C, robots);

    // merge both maps
    RobotTaskPoseMap rtpm;
    rtpm.insert(pick_rtpm.begin(), pick_rtpm.end());
    rtpm.insert(handover_rtpm.begin(), handover_rtpm.end());

    const auto test_sequence_for_handover = make_handover_sequence(robots, num_objects, rtpm);
    auto plan = plan_multiple_arms_given_sequence(C, rtpm, test_sequence_for_handover, home_poses);
    visualize_plan(C, plan.plan, true);

    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);

    std::stringstream buffer;
    buffer << "handover_test_" << std::put_time(&tm, "%Y%m%d_%H%M%S");

    export_plan(C, robots, home_poses, plan.plan, test_sequence_for_handover, buffer.str(), 0, 0);

    return 0;
  }

  if (mode == "plan_for_sequence"){
    RobotTaskPoseMap handover_rtpm = compute_all_handover_poses(C, robots);
    RobotTaskPoseMap pick_rtpm = compute_all_pick_and_place_positions(C, robots);

    // merge both maps
    RobotTaskPoseMap rtpm;
    rtpm.insert(pick_rtpm.begin(), pick_rtpm.end());
    rtpm.insert(handover_rtpm.begin(), handover_rtpm.end());

    const std::string path = sequence_path.p;
    const auto seq = load_sequence_from_json(path, robots);

    if (!check_sequence_validity(seq, rtpm)){
      spdlog::error("sequence invalid.");
      return 0;
    }
    const PlanResult plan = plan_multiple_arms_given_sequence(C, rtpm, seq, home_poses);
  
    if (plan.status == PlanStatus::success){
      std::time_t t = std::time(nullptr);
      std::tm tm = *std::localtime(&t);

      std::stringstream buffer;
      buffer << "sequence_plan_" << std::put_time(&tm, "%Y%m%d_%H%M%S");

      export_plan(C, robots, home_poses, plan.plan, seq, buffer.str(), 0, 0);

      if (display){
        visualize_plan(C, plan.plan);
      }
    }
    else{
      spdlog::warn("No solution found for given sequence.");
    }

    return 0;
  }

  RobotTaskPoseMap robot_task_pose_mapping;
  if (mode == "compute_keyframes"){
    if (use_picks){
      RobotTaskPoseMap pick_rtpm = compute_all_pick_and_place_positions(C, robots);
      robot_task_pose_mapping.insert(pick_rtpm.begin(), pick_rtpm.end());
    }
    if (use_handovers) {
      RobotTaskPoseMap handover_rtpm = compute_all_handover_poses(C, robots);
      robot_task_pose_mapping.insert(handover_rtpm.begin(),
                                     handover_rtpm.end());
    }
    spdlog::info("{} poses computed.", robot_task_pose_mapping.size());

    return 0;
  }

  spdlog::info("Computing pick and place poses");

  // merge both maps
  if (use_picks){
    RobotTaskPoseMap pick_rtpm = compute_all_pick_and_place_positions(C, robots);
    robot_task_pose_mapping.insert(pick_rtpm.begin(), pick_rtpm.end());
  }
  if (use_handovers) {
    RobotTaskPoseMap handover_rtpm = compute_all_handover_poses(C, robots);
    robot_task_pose_mapping.insert(handover_rtpm.begin(),
                                    handover_rtpm.end());
  }
  if (use_repeated_picks) {
    RobotTaskPoseMap pick_pick_rtpm = compute_all_pick_and_place_with_intermediate_pose(C, robots);
    robot_task_pose_mapping.insert(pick_pick_rtpm.begin(),
                                    pick_pick_rtpm.end());
  }
  spdlog::info("{} poses computed.", robot_task_pose_mapping.size());
  // }

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
        C, robot_task_pose_mapping, home_poses, max_attempts);
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
