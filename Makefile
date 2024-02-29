BASE = ../rai-fork
BASE2 = ../src

OBJS = main.o 

GL = 1
OMPL = 0
EIGEN = 1

DEPEND = Algo Control KOMO Core Geo Kin Gui Optim LGP Logic Manip Control PlanningSubroutines

#OPTIM = debug

include $(BASE)/build/generic.mk
