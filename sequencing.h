#pragma once

#include <random>
#include <vector>

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