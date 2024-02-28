#ifndef PLANNER_H
#define PLANNER_H

#include "plan.h"

class Planner{
  public:
    Planner();

    virtual void compute_plan() = 0;
};

/*class FullyOrderedPlanner: public Planner{
  public:
    FullyOrderedPlanner(const OrderedTaskSequence &seq);

    void compute_plan();
};

class PrioritizedPlanner: public FullyOrderedPlanner{
  public: 
    PrioritizedPlanner(const OrderedTaskSequence &seq){};

    void compute_plan();

  private:
    void compute_task();
};

class PartiallyOrderedPlanner: public Planner{
  public:
    PartiallyOrderedPlanner();
    void compute_plan();
};
*/

#endif 
