This repository contains the code that was used for the paper "".
It is under active development, the version that was used to produce the result in the paper is tagged.

# Installation
The code depends on 'rai' and 'rai-robotModels', which can be found in [] and [].
The path to rai needs to be added in the Makefile.

Compilation then works with 
```
make -j8
```

In case of issues, try

```
make cleanAll
```

To update dependencies

```
make depend
```

Also have a look at the intructions in 'rai'.

# Usage
Flags:
- mode [test/show\_plan/random\_search/greedy\_random\_search/simulated\_annealing]
- pnp [true/false]
- stippling\_pts [lots]
- env [lab/'']

# To Do
## Misc
- Ensure correct dependencies of rai/rai-robotModels: possibly make a submodule

## Code
- split main planning subroutine
- fix loading and visualization of previously computed paths

## Speed improvements
- squeaky wheel planner
- simulated annealing
- speed up komo runs

## Capabilities
- enable things that are not only 'go to point', e.g. drawing a line
- enable search over sequences with precendence constraints
- time-rescale path
- enable multi-arm cooperation
- look into more complex motion planning:
  - joint optimization
  - constrained sampling based planning
- more statistics
- joint optimization over the whole path
