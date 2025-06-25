#include <deque>
#include <fstream>
#include <iomanip>
#include <numeric>

#include <algorithm>
#include <chrono>
#include <random>

#include <zmq.hpp>

#include <math.h>

#include <atomic>
#include <csignal>

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

std::atomic<bool> keep_running(true);

void signal_handler(int signal) {
  if (signal == SIGINT) {
    std::cout << "\nCaught SIGINT, exiting...\n";
    keep_running = false;
  }
}

enum class RT_STATUS { NONE, KEYFRAMES, PATH };

arr std_to_arr(const std::vector<double> &std_vec) {
  arr rai_arr(std_vec.size());
  for (uint i = 0; i < std_vec.size(); ++i) {
    rai_arr(i) = std_vec[i];
  }

  return rai_arr;
}

std::vector<std::vector<double>> arr_path_to_std(arr path) {
  std::vector<std::vector<double>> vecvec;
  for (uint i = 0; i < path.d0; ++i) {
    std::vector<double> pose;
    for (uint j = 0; j < 6; ++j) {
      pose.push_back(path[i](j));
    }
    vecvec.push_back(pose);
  }

  return vecvec;
}

struct RTContext {
  std::vector<std::tuple<std::string, unsigned int, int>> sequence;
  std::map<int, std::vector<arr>> poses;

  std::map<std::string, Robot> robots;

  // Constructor
  RTContext(){};
};

RobotTaskPoseMap make_task_pose_map_from_sequence(const RTContext &ctx) {
  RobotTaskPoseMap rtpm;

  for (const auto &sequence_entry : ctx.sequence) {
    const auto robot_prefix = std::get<0>(sequence_entry);
    const auto object = std::get<1>(sequence_entry);
    const auto primitive_as_int = std::get<2>(sequence_entry);

    PrimitiveType primitive;
    if (primitive_as_int == 0) {
      primitive = PrimitiveType::pick;
    } else if (primitive_as_int == 1) {
      primitive = PrimitiveType::go_to;
    }

    RobotTaskPair rtp;
    rtp.robots = {ctx.robots.at(robot_prefix)};
    rtp.task = Task{.object = object, .type = primitive};
    rtpm[rtp].push_back({ctx.poses.at(object)});
  }

  return rtpm;
}

OrderedTaskSequence make_ordered_task_sequence(const RTContext &ctx) {
  OrderedTaskSequence seq;

  for (const auto &sequence_entry : ctx.sequence) {
    const auto robot_prefix = std::get<0>(sequence_entry);
    const auto object = std::get<1>(sequence_entry);
    const auto primitive_as_int = std::get<2>(sequence_entry);

    PrimitiveType primitive;
    if (primitive_as_int == 0) {
      primitive = PrimitiveType::pick;
    } else if (primitive_as_int == 1) {
      primitive = PrimitiveType::go_to;
    }

    RobotTaskPair rtp;
    rtp.robots = {ctx.robots.at(robot_prefix)};
    rtp.task = Task{.object = object, .type = primitive};

    seq.push_back(rtp);
  }

  return seq;
}

void add_object(rai::Configuration &C, std::string name, json value) {
  const auto shape = value["size"].get<std::vector<double>>();
  auto *obj = C.addFrame(name.c_str(), "table");

  if (value.contains("marker") && value["marker"].get<bool>()) {
    obj->setShape(rai::ST_marker, {0.3});
    obj->setContact(0);
  } else {
    obj->setShape(rai::ST_box, {shape[0], shape[1], shape[2], 0.01});
    obj->setContact(-1);
  }

  obj->setJoint(rai::JT_rigid);
}

void add_goal(rai::Configuration &C, std::string name, json value) {
  const auto shape = value["size"].get<std::vector<double>>();
  auto *obj = C.addFrame(name.c_str(), "table");

  obj->setShape(rai::ST_box, {shape[0], shape[1], shape[2], 0.01});
  obj->setContact(0);
  obj->setColor({1, 0, 0, 0.5});
}

void set_base_pose(rai::Configuration &C, json received_json,
                   const std::string &prefix, const std::string base_name) {
  const std::vector<double> r1_base_pose =
      received_json["base_poses"][prefix].get<std::vector<double>>();
  C.getFrame(base_name.c_str())
      ->setRelativePosition(
          {r1_base_pose[0], r1_base_pose[1], r1_base_pose[2]});
  C.getFrame(base_name.c_str())
      ->setRelativeQuaternion(
          {r1_base_pose[3], r1_base_pose[4], r1_base_pose[5], r1_base_pose[6]});
}

void set_relative_pose(rai::Configuration &C, json obj_field,
                       const std::string &name) {
  const std::vector<double> obj_pose =
      obj_field["pose"].get<std::vector<double>>();
  C.getFrame(name.c_str())
      ->setRelativePosition({obj_pose[0], obj_pose[1], obj_pose[2]});
  C.getFrame(name.c_str())
      ->setRelativeQuaternion(
          {obj_pose[3], obj_pose[4], obj_pose[5], obj_pose[6]});
}

std::map<int, std::vector<std::vector<double>>>
parse_poses(json received_json) {
  std::map<int, std::vector<std::vector<double>>> poses;
  for (const auto &[robot, task_pose_list] : received_json["poses"].items()) {
    std::cout << "parsign per robot poses" << std::endl;

    for (const auto &[task_id, pose_list] : task_pose_list.items()) {
      unsigned int task_id_typed =
          static_cast<unsigned int>(std::stoul(task_id));
      std::cout << "parsign task pose list" << std::endl;
      std::vector<std::vector<double>> robot_poses;
      for (const auto &pose : pose_list) {
        std::cout << "adding poses" << std::endl;
        robot_poses.emplace_back(pose.get<std::vector<double>>());
      }
      poses[task_id_typed] = robot_poses;
    }
  }

  return poses;
}

std::map<std::string, std::vector<json>> serialize_plan(const Plan &paths) {
  std::map<std::string, std::vector<json>> std_paths;
  for (auto [robot, path] : paths) {
    // std::vector<>
    const auto prefix = robot.prefix;
    std_paths[prefix] = {};
    for (auto tp : path) {
      const auto time = tp.t;
      std::vector<double> std_time;
      for (const auto t : time) {
        std_time.push_back(t);
      }

      const auto single_robot_path = arr_path_to_std(tp.path);

      const json path_segment = {{"path", single_robot_path},
                                 {"time", std_time}};
      std_paths[prefix].push_back(path_segment);
    }
  }

  return std_paths;
}

void rt_loop() {
  RTContext ctx;

  // setup configuration:
  // - put robots into the scene
  rai::Configuration C;

  auto *base = C.addFrame("world", "");
  base->setShape(rai::ST_marker, {0.001});
  base->setPosition({0., 0., .5});
  base->setContact(0.);

  C.addFile("./in/scenes/table_real.g");

  for (uint i = 0; i < 2; ++i) {
    auto *a = C.addFile("./in/robots/inflated_ur5_vacuum.g");
    C.reconfigureRoot(a, true);
    a->linkFrom(C["table"]);

    const rai::String prefix = STRING('a' << i << '_');
    a->prefixSubtree(prefix);
  }

  auto r0 = Robot("a0_", RobotType::ur5);
  auto r1 = Robot("a1_", RobotType::ur5);

  arr home_pose = ARR(-1.5708, -2.3562 + 3.1415 / 2, 1.5708,
                      -1.5707 + 3.1415 / 2, -1.5707, 0);
  r0.home_pose = home_pose;
  r0.start_pose = home_pose;
  r0.ee_type = EndEffectorType::vacuum;

  r1.home_pose = home_pose;
  r1.ee_type = EndEffectorType::vacuum;
  r1.start_pose = home_pose;

  ctx.robots["a0_"] = r0;
  ctx.robots["a1_"] = r1;

  std::unordered_map<Robot, arr> home_poses;
  home_poses[r0] = home_pose;
  home_poses[r1] = home_pose;

  const auto r0_frames = get_robot_joints(C, r0);
  const auto r1_frames = get_robot_joints(C, r1);

  zmq::context_t context(1);
  zmq::socket_t subscriber(context, ZMQ_SUB);
  zmq::socket_t publisher(context, ZMQ_PUB);

  const int publisher_port = 5570;
  const int subscriber_port = 5571;

  const std::string publisher_adress =
      "tcp://127.0.0.1:" + std::to_string(publisher_port);
  const std::string subscriber_adress =
      "tcp://127.0.0.1:" + std::to_string(subscriber_port);

  subscriber.set(zmq::sockopt::conflate, 1);
  subscriber.set(zmq::sockopt::subscribe, ""); // Subscribe to all messages

  subscriber.connect(subscriber_adress);
  publisher.bind(publisher_adress);

  auto status = RT_STATUS::NONE;

  while (keep_running) {
    status = RT_STATUS::NONE;

    // receive stuff
    zmq::message_t request;

    if (subscriber.recv(request, zmq::recv_flags::dontwait)) {
      try {
        std::string received_message(static_cast<char *>(request.data()),
                                     request.size());
        const auto received_json = json::parse(received_message);

        std::cout << "received request" << std::endl;

        if (received_json.contains("base_poses")) {
          std::cout << "base_poses" << std::endl;

          set_base_pose(C, received_json, "a0_", "a0_base");
          set_base_pose(C, received_json, "a1_", "a1_base");

          // const std::vector<double> r1_base_pose =
          //     received_json["base_poses"]["a0_"].get<std::vector<double>>();
          // C.getFrame("a0_base")->setRelativePosition(
          //     {r1_base_pose[0], r1_base_pose[1], r1_base_pose[2]});
          // C.getFrame("a0_base")->setRelativeQuaternion(
          //     {r1_base_pose[3], r1_base_pose[4], r1_base_pose[5],
          //      r1_base_pose[6]});

          // const std::vector<double> r2_base_pose =
          //     received_json["base_poses"]["a1_"].get<std::vector<double>>();
          // C.getFrame("a1_base")->setRelativePosition(
          //     {r2_base_pose[0], r2_base_pose[1], r2_base_pose[2]});
          // C.getFrame("a1_base")->setRelativeQuaternion(
          //     {r2_base_pose[3], r2_base_pose[4], r2_base_pose[5],
          //      r2_base_pose[6]});
        }

        if (received_json.contains("objects")) {
          std::cout << "objects" << std::endl;

          for (auto &[key, value] : received_json["objects"].items()) {
            if (C.getFrame(key.c_str()) == 0) {
              std::cout << "adding objs" << std::endl;
              add_object(C, key, value);
              // const auto shape = value["size"].get<std::vector<double>>();
              // auto *obj = C.addFrame(key.c_str(), "table");

              // if (value.contains("marker") && value["marker"].get<bool>()) {
              //   obj->setShape(rai::ST_marker, {0.3});
              //   obj->setContact(0);
              // } else {
              //   obj->setShape(rai::ST_box,
              //                 {shape[0], shape[1], shape[2], 0.01});
              //   obj->setContact(-1);
              // }

              // obj->setJoint(rai::JT_rigid);
            }

            set_relative_pose(C, value, key);

            // const std::vector<double> obj_pose =
            //     value["pose"].get<std::vector<double>>();
            // C.getFrame(key.c_str())
            //     ->setRelativePosition({obj_pose[0], obj_pose[1],
            //     obj_pose[2]});
            // C.getFrame(key.c_str())
            //     ->setRelativeQuaternion(
            //         {obj_pose[3], obj_pose[4], obj_pose[5], obj_pose[6]});

            if (value.contains("parent")) {
              // TODO: attach to parent
              // C.getFrame(key.c_str())->unlink();
              // C.getFrame(key.c_str())->linkFrom(parent_frame, true);
            }
          }
        }

        if (received_json.contains("goals")) {
          std::cout << "goals" << std::endl;

          for (auto &[key, value] : received_json["goals"].items()) {
            if (C.getFrame(STRING(key)) == 0) {
              std::cout << "adding goal" << std::endl;
              add_goal(C, key, value);
              // const auto shape = value["size"].get<std::vector<double>>();
              // auto *obj = C.addFrame(STRING(key), "table");

              // obj->setShape(rai::ST_box, {shape[0], shape[1], shape[2], 0.01});
              // obj->setContact(0);
              // obj->setColor({1, 0, 0, 0.5});
            }

            set_relative_pose(C, value, key);

            // const std::vector<double> goal_pose =
            //     value["pose"].get<std::vector<double>>();
            // C.getFrame(STRING(key))
            //     ->setRelativePosition(
            //         {goal_pose[0], goal_pose[1], goal_pose[2]});
            // C.getFrame(STRING(key))
            //     ->setRelativeQuaternion(
            //         {goal_pose[3], goal_pose[4], goal_pose[5],
            //         goal_pose[6]});
          }
        }

        if (received_json.contains("robot_states")) {
          std::cout << "robot_states" << std::endl;

          {
            const std::vector<double> joint_state_vector =
                received_json["robot_states"]["a0_"].get<std::vector<double>>();
            const arr joint_state = std_to_arr(joint_state_vector);
            C.setJointState(joint_state, r0_frames);
          }

          {
            const std::vector<double> joint_state_vector =
                received_json["robot_states"]["a1_"].get<std::vector<double>>();
            const arr joint_state = std_to_arr(joint_state_vector);
            C.setJointState(joint_state, r1_frames);
          }
        }

        if (received_json.contains("compute_keyframes")) {
          status = RT_STATUS::KEYFRAMES;
          // auto robot_object_pairs = parse_assignments();
        } else if (received_json.contains("compute_path")) {
          status = RT_STATUS::PATH;
          if (received_json.contains("sequence")) {
            // ensure that it is there
            std::vector<std::tuple<std::string, unsigned int, int>> sequence;
            for (const auto &entry : received_json["sequence"]) {
              sequence.emplace_back(entry[0], entry[1], entry[2]);
            }

            ctx.sequence = sequence;
          }
          if (received_json.contains("poses")) {
            std::cout << "parsign poses" << std::endl;
            // ensure that it is there
            const auto poses = parse_poses(received_json);

            for (const auto &[key, vecvec] : poses) {
              std::vector<arr> arr_list;
              for (const auto &vec : vecvec) {
                arr a = std_to_arr(vec); // convert std::vector<double> to arr
                arr_list.push_back(a);
                // std::cout << "pose" << std::endl;
                // std::cout << a << std::endl;
              }
              ctx.poses[key] = std::move(arr_list);
            }
          }
        }
      } catch (const std::exception &e) {
        std::cerr << "Error parsing JSON: " << e.what() << "\n";
        return;
      }
    }

    C.watch(false);

    // do stuff depending on what we received
    if (status == RT_STATUS::KEYFRAMES) {

    } else if (status == RT_STATUS::PATH) {
      std::cout << "Running planner" << std::endl;
      // compute plan:
      // requires
      // - a sequence
      // - poses
      const auto r0_start_pose = C.getJointState(r0_frames);
      const auto r1_start_pose = C.getJointState(r1_frames);

      ctx.robots["a0_"].start_pose = r0_start_pose;
      ctx.robots["a1_"].start_pose = r1_start_pose;

      // we need to do this because I made too many bad choices wehen setting
      // this up
      // TODO: set this to the starting pose
      home_poses.clear();
      home_poses[ctx.robots["a0_"]] = home_pose;
      home_poses[ctx.robots["a1_"]] = home_pose;

      // std::cout << r0.start_pose << std::endl;
      // std::cout << r1.start_pose << std::endl;

      // make task pose map
      std::cout << "Making task pose mapping" << std::endl;
      const RobotTaskPoseMap rtpm = make_task_pose_map_from_sequence(ctx);

      std::cout << "Making task sequence" << std::endl;
      const OrderedTaskSequence seq = make_ordered_task_sequence(ctx);
      std::cout << ordered_sequence_to_str(seq) << std::endl;

      std::cout << "Started planning" << std::endl;
      const PlanResult plan_result =
          plan_multiple_arms_given_sequence(C, rtpm, seq, home_poses);

      std::cout << "Done planning" << std::endl;

      const auto paths = plan_result.plan;
      // map[str, std::vector<path>]
      const std::map<std::string, std::vector<json>> std_paths =
          serialize_plan(paths);
      // for (auto [robot, path] : paths) {
      //   // std::vector<>
      //   const auto prefix = robot.prefix;
      //   std_paths[prefix] = {};
      //   for (auto tp : path) {
      //     const auto time = tp.t;
      //     std::vector<double> std_time;
      //     for (const auto t : time) {
      //       std_time.push_back(t);
      //     }

      //     const auto single_robot_path = arr_path_to_std(tp.path);

      //     const json path_segment = {{"path", single_robot_path},
      //                                {"time", std_time}};
      //     std_paths[prefix].push_back(path_segment);
      //   }
      // }

      // send stuff back
      const json json_msg = {{"path_segments", std_paths}};

      const std::string message = json_msg.dump();
      zmq::message_t zmq_message(message.begin(), message.end());
      publisher.send(zmq_message, zmq::send_flags::none);
    }

    ctx.poses.clear();
    ctx.sequence.clear();

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    std::cout << "loop" << std::endl;
  }

  std::cout << "exiting loop" << std::endl;

  return;
}