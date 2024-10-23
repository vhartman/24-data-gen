BASE = ../rai-fork
BASE2 = ../rai-manip

#OBJS = main.o path_util.o

GL = 1
OMPL = 0
EIGEN = 1

RAI_TESTS = 1

DEPEND = Algo Control KOMO Core Geo Kin Gui Optim LGP Logic Manip Control PlanningSubroutines

#OPTIM = debug

LIBS += -lspdlog -lfmt -lstdc++fs
LPATHS += -L/usr/lib/x86_64-linux-gnu/libspdlog.so

include $(BASE)/build/generic.mk

LIBS_TEST := $(filter-out lgtest, $(LIBS))
LIBS_TEST += -lgtest_main

test.exe: $(PREOBJS) $(BUILDS) test.o
	$(LINK) $(LDFLAGS) -o test.exe test.o $(LIBS_TEST)
