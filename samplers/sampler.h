#pragma once

#include "spdlog/spdlog.h"

#include <Kin/featureSymbols.h>

#include "planners/plan.h"
#include "planners/prioritized_planner.h"

#include "common/util.h"

#include "goto_sampler.h"
#include "handover_sampler.h"
#include "pick_and_place_sampler.h"
#include "repeated_pick_sampler.h"

class TaskPoseSampler{
  public:
    rai::Configuration C;
    TaskPoseSampler(rai::Configuration &_C): C(_C) {};
    virtual TaskPoses sample(const RobotTaskPair &rtp) = 0;
    virtual TaskPoses sample(const RobotTaskPair &rtp, const rai::Animation &A) = 0;
};

// class RobotTaskPoseSampler{
//   public:
//     // this implements caching of results
//     TaskPoses sample(const rai::Configuration &C, const RobotTaskPair &rtp){
//       if (cache.count(rtp) > 0){
//         return cache.at(rtp)[0];
//       }
//       else{
//         const auto res = generate(C, rtp);
//         cache[rtp].push_back(res);

//         return res;
//       }
//     };

//   private:
//     virtual TaskPoses generate(const rai::Configuration &C, const RobotTaskPair &rtp) = 0;

//     // infeasible is represented as empty vector
//     std::unordered_map<RobotTaskPair, std::vector<TaskPoses>> cache;
// };