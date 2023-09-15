BASE = ../rai-fork
BASE2 = ../src

OBJS = main.o line.o

GL = 1
OMPL = 0
EIGEN = 1

DEPEND = Algo Control KOMO Core Geo Kin Gui Optim LGP Logic Manip Control PlanningSubroutines

include $(BASE)/build/generic.mk
