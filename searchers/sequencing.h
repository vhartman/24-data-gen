#pragma once

#include <random>
#include <vector>
#include <deque>

#include "plan.h"
#include "util.h"

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

// Approach to generate a sequence from the primitives:
// For all objects, collect available primitives, and choose one
// Then shuffle the primitives, and add them to the sequence one by one
OrderedTaskSequence
generate_random_valid_sequence(const std::vector<Robot> &robots,
                               const uint num_tasks,
                               const RobotTaskPoseMap &rtpm) {
  std::vector<std::deque<RobotTaskPair>> sequence_of_primitives;
  for (uint i=0; i<num_tasks; ++i){
    // extract available primitives and choose one
    std::vector<RobotTaskPair> available_primitives_for_object;
    for (const auto &e: rtpm){
      // we filter out pick_pick_2, because it is the second action of a 
      // primitive
      if (e.first.task.object == i &&
          e.first.task.type != PrimitiveType::pick_pick_2) {
        available_primitives_for_object.push_back(e.first);
      }
    }

    const uint primitive_index = std::rand() % available_primitives_for_object.size();
    const RobotTaskPair& primitive = available_primitives_for_object[primitive_index];

    if (primitive.task.type != PrimitiveType::pick_pick_1){
      sequence_of_primitives.push_back({primitive});
      // sequence_of_primitives.back().push_back(primitive);
    }
    else{
      sequence_of_primitives.push_back({primitive});
      // search for pick_pick_2 and add it
      for (const auto &rtp : rtpm) {
        auto robots = rtp.first.robots;
        if (rtp.first.task.object == i &&
            robots[0] == primitive.robots[0] &&
            robots[1] == primitive.robots[1] &&
            rtp.first.task.type == PrimitiveType::pick_pick_2) {
          sequence_of_primitives.back().push_back(rtp.first);
        }
      }
    }
  }

  OrderedTaskSequence seq;
  while(sequence_of_primitives.size() > 0){
    const uint ind = std::rand() % sequence_of_primitives.size();
    seq.push_back(sequence_of_primitives[ind].front());
    sequence_of_primitives[ind].pop_front();

    if (sequence_of_primitives[ind].size() == 0){
      // delete element from vector
      sequence_of_primitives.erase(sequence_of_primitives.begin() + ind);
    }
  }

  return seq;
}

OrderedTaskSequence
generate_alternating_random_sequence(const std::vector<Robot> &robots,
                                     const uint num_tasks,
                                     const RobotTaskPoseMap &rtpm) {
  uint r = rand() % robots.size();

  auto available_tasks = straightPerm(num_tasks);
  OrderedTaskSequence seq;
  uint cnt = 0;
  while (available_tasks.size() > 0) {
    for (uint j = 0; j < 10; ++j) {
      const uint task_index = available_tasks[rand() % available_tasks.size()];
      RobotTaskPair rtp;
      rtp.robots = {robots[r]};
      rtp.task = Task{.object=task_index, .type=PrimitiveType::pick};

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

    if (cnt > 1000){
      spdlog::error("No feasible assignment found.");
      break;
    }

    ++cnt;
  }

  return seq;
}

OrderedTaskSequence generate_alternating_greedy_sequence(
    const std::vector<Robot> &robots, const uint num_tasks,
    const RobotTaskPoseMap &rtpm, const std::unordered_map<Robot, arr> &home_poses) {
  std::cout << "Generating alternating greedy" << std::endl;
  auto available_tasks = straightPerm(num_tasks);

  // sample starting_ robot.
  uint r = rand() % robots.size();
  std::unordered_map<Robot, arr> poses = home_poses;

  const uint max_iter = 1000;

  OrderedTaskSequence seq;
  uint cnt = 0;
  while (available_tasks.size() > 0) {
    ++cnt;
    // find minimum dist pose to current robot
    auto min_dist = 1e6;
    uint task_index = 0;
    bool assigned_task = false;
    for (auto t : available_tasks) {
      RobotTaskPair rtp;
      rtp.robots = {robots[r]};
      rtp.task = Task{.object=t, .type=PrimitiveType::pick};

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
      spdlog::info("adding {} with index {}", robots[r].prefix, r);
      seq.push_back(RobotTaskPair{.robots={robots[r]}, .task=Task{.object=task_index, .type=PrimitiveType::pick}});
    }
    else{
      spdlog::info("not assigned task");
    }

    r = (r + 1) % robots.size();

    if (cnt > max_iter){
      spdlog::error("stopping sinc emax iter");
      break;
    }
  }

  return seq;
}

OrderedTaskSequence make_handover_sequence(const std::vector<Robot> &robots,
                                           const uint num_tasks,
                                           const RobotTaskPoseMap &rtpm, const uint max_attempts = 100) {
  OrderedTaskSequence seq;
  for (uint i = 0; i < num_tasks; ++i) {
    uint cnt = 0;
    while (true) {
      ++cnt;
      // choose random action
      const uint a = rand() % 2;

      // choose random robot
      uint r1 = rand() % robots.size();
      uint r2 = rand() % robots.size();

      RobotTaskPair rtp;

      if (a == 0) {
        rtp.robots = {robots[r1]};
        rtp.task = Task{.object = i, .type = PrimitiveType::pick};
      } else {
        rtp.robots = {robots[r1], robots[r2]};
        rtp.task = Task{.object = i, .type = PrimitiveType::handover};
      }

      // check if the task is feasible with the chosen robot(s)
      if (rtpm.count(rtp) != 0) {
        seq.push_back(rtp);
        break;
      }

      if (cnt > max_attempts){
        std::cout << "No sequence feasible" << std::endl;
        break;
      }
    }
  }

  return seq;
}