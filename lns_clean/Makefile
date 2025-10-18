# Build script for the refactored LNS solver components

CXX := g++
CC := gcc
AR := ar

OBJ_DIR := build
LIB := liblns_clean.a
MAIN := main_clean_lns
MAIN_SRC := main_clean_lns.cpp


RUN_EXPERIMENTS_DIR := run_experiments
RUN_EXPERIMENTS_SRC := $(RUN_EXPERIMENTS_DIR)/run_batch_experiments.cpp
RUN_EXPERIMENTS_BIN := $(RUN_EXPERIMENTS_DIR)/run_batch_experiments

CXXFLAGS := -std=gnu++17 -O2 -Wall -Wextra -pedantic -pthread -fpermissive
CFLAGS := -O2 -Wall -Wextra -pedantic

# Optional instrumentation helpers used for debugging memory issues
DEBUG_FLAGS := -O0 -g
SANITIZE_FLAGS := -fsanitize=address -fno-omit-frame-pointer
CPPFLAGS := -D__STDC_FORMAT_MACROS -I. -I.. -I../cnf -I../mdd -I../minisat -I../minisat/minisat-master -I../probSAT-master
DEPFLAGS := -MMD -MP
LDFLAGS := -pthread

LOCAL_SOURCES := \
    ExperimentLogger.cpp \
    Create_Local_Problem.cpp \
    Current_Solution.cpp \
    Lazy_SAT_Solve.cpp \
    Load_LNSProblem.cpp \
    Local_Zone.cpp \
    Local_Zone_Builder.cpp \
    LNS.cpp \
    Solve_Local_Zone.cpp \
    VerificationHelpers.cpp \
    Waiting_time_Solve.cpp

SHARED_SOURCES := \
    ../SATSolverManager.cpp \
    ../cnf/CNF.cpp \
    ../cnf/CNFConstructor.cpp \
    ../cnf/CNFProbSATConstructor.cpp \
    ../mdd/MDD.cpp \
    ../mdd/MDDConstructor.cpp \
    ../mdd/MDDNode.cpp \
    ../minisat/minisat-wrapper.cpp \
    ../minisat/minisat-master/minisat/core/Solver.cc \
    ../minisat/minisat-master/minisat/utils/System.cc

C_SOURCES := \
    ../probSAT-master/probSAT_inmem.c

# Helper to flatten object names
define make_obj
$(OBJ_DIR)/$(notdir $(basename $(1))).o
endef

LOCAL_OBJS := $(foreach src,$(LOCAL_SOURCES),$(call make_obj,$(src)))
SHARED_OBJS := $(foreach src,$(SHARED_SOURCES),$(call make_obj,$(src)))
C_OBJS := $(foreach src,$(C_SOURCES),$(call make_obj,$(src)))

OBJS := $(LOCAL_OBJS) $(SHARED_OBJS) $(C_OBJS)
DEPS := $(OBJS:.o=.d)

.PHONY: all clean asan run_experiments

all: $(LIB) $(MAIN) $(RUN_EXPERIMENTS_BIN)

$(LIB): $(OBJ_DIR) $(OBJS)
	$(AR) rcs $@ $(OBJS)

$(MAIN): $(LIB) $(MAIN_SRC)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(MAIN_SRC) -L. -llns_clean -o $@

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(RUN_EXPERIMENTS_BIN): $(RUN_EXPERIMENTS_SRC) $(RUN_EXPERIMENTS_DIR)/simple_json.h
	$(CXX) $(CXXFLAGS) -I$(RUN_EXPERIMENTS_DIR) $< -o $@

run_experiments: $(RUN_EXPERIMENTS_BIN)

define cpp_rule
$(OBJ_DIR)/$(notdir $(basename $(1))).o: $(1) | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(DEPFLAGS) -c $$< -o $$@
endef

define c_rule
$(OBJ_DIR)/$(notdir $(basename $(1))).o: $(1) | $(OBJ_DIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(DEPFLAGS) -c $$< -o $$@
endef

$(foreach src,$(LOCAL_SOURCES),$(eval $(call cpp_rule,$(src))))
$(foreach src,$(SHARED_SOURCES),$(eval $(call cpp_rule,$(src))))
$(foreach src,$(C_SOURCES),$(eval $(call c_rule,$(src))))

clean:
	rm -rf $(OBJ_DIR) $(LIB) $(MAIN) $(RUN_EXPERIMENTS_BIN)

asan:
	$(MAKE) clean
	$(MAKE) CXXFLAGS="$(filter-out -O%,$(CXXFLAGS)) $(DEBUG_FLAGS) $(SANITIZE_FLAGS)" \
		CFLAGS="$(filter-out -O%,$(CFLAGS)) $(DEBUG_FLAGS) $(SANITIZE_FLAGS)" \
		LDFLAGS="$(LDFLAGS) $(SANITIZE_FLAGS)"

-include $(DEPS)