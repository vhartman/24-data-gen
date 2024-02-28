#pragma once

struct Mode{
  uint task;
};

std::vector<std::vector<Mode>> get_mode_sequence(const OrderedTaskSequence &seq){
  std::vector<std::vector<Mode>> modes;

  std::vector<Mode> robot_modes;
  
  // find the initial modes that each robot is in

  // iterate over all tasks and add each mode change into the vector of
  // possible modes
  for (uint i=0; i<seq.size(); ++i){
    //robot_modes = 0;
  }

  return modes;
}

struct Cost{
  double makespan;
  double sum_of_lengths;
};

struct State{
  std::vector<arr> qs;
  std::vector<int> modes;
};

class CompoundTreePlanner{
  public:
    double p_goal = 0.1;
    rai::Configuration C;
    OrderedTaskSequence seq;

    CompoundTreePlanner(rai::Configuration _C, const OrderedTaskSequence &_seq): C(_C), seq(_seq){

    };
    
    // things for logging
    // - costs
    // - times

    State sample(const Mode m, const double c){
      // figure out what to sample seperately, what jointly

      // iterate over all things that we are sampling

      arr r(1);
      rndUniform(r);
      if (r(0) > p_goal){
        // sample some goals according to the modes
      }
      else{
        if (c < 0){
          // sample random pt
        }
        else{
          // sample informed
        }
      }
     
      // construct composite state
    }

    PlanResult plan(){
      // do joint planning in statespace C x Z^r with r = num_robots
      /*Tree t;
      Cost c;

      // compute list of possible modes
      std::vector<std::vector<Mode>> possible_modes = get_mode_sequence(seq);

      // initialize the active modes with the initial mode
      std::vector<std::vector<Mode>> active_modes;
      active_modes.push_back(possible_modes[0]);

      while (true){
        // sample the mode by uniformly choosing from the list of all active composite modes
        const uint random_index = 0;
        const auto random_mode = active_modes[random_index];

        // sample the joint state
        State s_rand = sample(random_mode);
        
        // find node in tree that is close
        // -> distance metric: inf if mode is not matching
        const auto n_near = t.get_nearest(s_rand);

        // steer to random node
        const auto s_new = steer(n_near.q, s_rand);

        // check edge
        const bool edge_feasible = true;

        // add node to tree if feasible 
        if (edge_feasible){
        }
        
        // rewire
        const auto nodes = t.get_near(s_new);
        for (const auto node: nodes){
          // check if cost is lower than prev. cost
          if (){
            // check if edge is feasible
            bool f = true;
            if (f){
              // rewire
            }
          }
        }
      }*/

      return PlanResult(PlanStatus::failed);

    }
};

/*
struct Node{
  std::vector<arr> qs;
  std::vector<int> modes;
};

Cost cost(){}

double heuristic(){}

class CompoundRoadmapPlanner{
  public:
    CompundRoadmapPlanner(const OrderedTaskSequence &seq){};

    // things for logging
    // - costs
    // - times
    // TODO

  void plan(){
    Cost current_best_cost;
    Path current_best_path;
    Graph G;

    // compute list of possible modes
    std::vector<std::vector<Mode>> possible_modes = get_possible_modes(seq);

    while(true){
      // sample batches in all modes and add to graph
      for (const auto mode: possible_modes){
        const auto samples = sample_batch_in_mode(mode);

        // add to graph
        // TODO: how to deal with goals?
      }

      // search
      const auto path = G.search();
      if (cost() < current_best_cost){
        current_best_path = path;
        current_best_cost = cost(path);
      }
    }

    return current_best_path;
  }

  void sample(){
  
  }
};
*/
