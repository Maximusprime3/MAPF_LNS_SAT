# Build script for the refactored LNS solver components

CXX := g++
AR := ar

OBJ_DIR := build
LIB := liblns_clean.a
MAIN := main_clean_lns
MAIN_SRC := main_clean_lns.cpp


RUN_EXPERIMENTS_DIR := run_experiments
RUN_EXPERIMENTS_SRC := $(RUN_EXPERIMENTS_DIR)/run_batch_experiments.cpp
RUN_EXPERIMENTS_BIN := $(RUN_EXPERIMENTS_DIR)/run_batch_experiments

TEST_DIR := tests
VERIFIER_TEST_SRC := $(TEST_DIR)/test_solution_verifier.cpp
VERIFIER_TEST_BIN := $(OBJ_DIR)/test_solution_verifier
VARIANT_TEST_SRC := $(TEST_DIR)/test_neighborhood_variant.cpp
VARIANT_TEST_BIN := $(OBJ_DIR)/test_neighborhood_variant
TRANSACTION_TEST_SRC := $(TEST_DIR)/test_current_solution_transaction.cpp
TRANSACTION_TEST_BIN := $(OBJ_DIR)/test_current_solution_transaction
INITIAL_MDD_ID_TEST_SRC := $(TEST_DIR)/test_initial_mdd_agent_ids.cpp
INITIAL_MDD_ID_TEST_BIN := $(OBJ_DIR)/test_initial_mdd_agent_ids
SOLVE_RESULT_TEST_SRC := $(TEST_DIR)/test_solve_result.cpp
SOLVE_RESULT_TEST_BIN := $(OBJ_DIR)/test_solve_result
MINISAT_STATUS_TEST_SRC := $(TEST_DIR)/test_minisat_unsat_status.cpp
MINISAT_STATUS_TEST_BIN := $(OBJ_DIR)/test_minisat_unsat_status
SAT_PROTOCOL_TEST_SRC := $(TEST_DIR)/test_sat_protocol_characterization.cpp
SAT_PROTOCOL_TEST_BIN := $(OBJ_DIR)/test_sat_protocol_characterization
PSEUDO_SEGMENT_TEST_SRC := $(TEST_DIR)/test_pseudo_agent_segmentation.cpp
PSEUDO_SEGMENT_TEST_BIN := $(OBJ_DIR)/test_pseudo_agent_segmentation
PSEUDO_REFRESH_TEST_SRC := $(TEST_DIR)/test_pseudo_agent_refresh_reassembly.cpp
PSEUDO_REFRESH_TEST_BIN := $(OBJ_DIR)/test_pseudo_agent_refresh_reassembly
LOCAL_STATE_TEST_SRC := $(TEST_DIR)/test_local_zone_state_validation.cpp
LOCAL_STATE_TEST_BIN := $(OBJ_DIR)/test_local_zone_state_validation
WAITING_BUDGET_TEST_SRC := $(TEST_DIR)/test_waiting_budget_conservation.cpp
WAITING_BUDGET_TEST_BIN := $(OBJ_DIR)/test_waiting_budget_conservation
GRID_WALKABILITY_TEST_SRC := $(TEST_DIR)/test_grid_walkability.cpp
GRID_WALKABILITY_TEST_BIN := $(OBJ_DIR)/test_grid_walkability
CONFIG_CHARACTERIZATION_TEST_SRC := $(TEST_DIR)/test_solver_configuration_characterization.cpp
CONFIG_CHARACTERIZATION_TEST_BIN := $(OBJ_DIR)/test_solver_configuration_characterization
SOLVER_CONFIG_TEST_SRC := $(TEST_DIR)/test_solver_configuration.cpp
SOLVER_CONFIG_TEST_BIN := $(OBJ_DIR)/test_solver_configuration
BATCH_BACKEND_TEST_SRC := $(TEST_DIR)/test_batch_backend_rejection.sh
SUPPORTED_BUILD_DEPENDENCY_TEST_SRC := $(TEST_DIR)/test_supported_build_dependencies.sh
LNS_VERIFICATION_SRC := $(TEST_DIR)/run_lns_verification.cpp
LNS_VERIFICATION_BIN := $(OBJ_DIR)/run_lns_verification

CXXFLAGS := -std=gnu++17 -O2 -Wall -Wextra -pedantic -pthread -fpermissive

# Optional instrumentation helpers used for debugging memory issues
DEBUG_FLAGS := -O0 -g
SANITIZE_FLAGS := -fsanitize=address -fno-omit-frame-pointer
CPPFLAGS := -D__STDC_FORMAT_MACROS -I. -I.. -I../cnf -I../mdd -I../minisat -I../minisat/minisat-master
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
    NeighborhoodVariant.cpp \
    SolverConfiguration.cpp \
    Solve_Local_Zone.cpp \
    SolutionVerifier.cpp \
    VerificationHelpers.cpp \
    Waiting_time_Solve.cpp

SHARED_SOURCES := \
    ../SATSolverManager.cpp \
    ../cnf/CNF.cpp \
    ../cnf/CNFConstructor.cpp \
    ../mdd/MDD.cpp \
    ../mdd/MDDConstructor.cpp \
    ../mdd/MDDNode.cpp \
    ../minisat/minisat-wrapper.cpp \
    ../minisat/minisat-master/minisat/core/Solver.cc \
    ../minisat/minisat-master/minisat/utils/System.cc


# Helper to flatten object names
define make_obj
$(OBJ_DIR)/$(notdir $(basename $(1))).o
endef

LOCAL_OBJS := $(foreach src,$(LOCAL_SOURCES),$(call make_obj,$(src)))
SHARED_OBJS := $(foreach src,$(SHARED_SOURCES),$(call make_obj,$(src)))

OBJS := $(LOCAL_OBJS) $(SHARED_OBJS)
DEPS := $(OBJS:.o=.d)

.PHONY: all clean asan run_experiments test test_solution_verifier test_neighborhood_variant test_current_solution_transaction test_initial_mdd_agent_ids test_solve_result test_minisat_unsat_status test_sat_protocol_characterization test_pseudo_agent_segmentation test_pseudo_agent_refresh_reassembly test_local_zone_state_validation test_waiting_budget_conservation test_grid_walkability test_solver_configuration_characterization test_solver_configuration test_batch_backend_rejection test_supported_build_dependencies lns_verification_runner

all: $(LIB) $(MAIN) $(RUN_EXPERIMENTS_BIN)

$(LIB): $(OBJ_DIR) $(OBJS)
	$(RM) $@
	$(AR) rcs $@ $(OBJS)

$(MAIN): $(LIB) $(MAIN_SRC)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(MAIN_SRC) -L. -llns_clean -o $@

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(RUN_EXPERIMENTS_BIN): $(RUN_EXPERIMENTS_SRC) $(RUN_EXPERIMENTS_DIR)/simple_json.h
	$(CXX) $(CXXFLAGS) -I$(RUN_EXPERIMENTS_DIR) $< -o $@

run_experiments: $(RUN_EXPERIMENTS_BIN)

$(VERIFIER_TEST_BIN): $(VERIFIER_TEST_SRC) SolutionVerifier.cpp SolutionVerifier.h | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -I. $(VERIFIER_TEST_SRC) SolutionVerifier.cpp -o $@

test_solution_verifier: $(VERIFIER_TEST_BIN)
	$(VERIFIER_TEST_BIN)

$(VARIANT_TEST_BIN): $(VARIANT_TEST_SRC) NeighborhoodVariant.cpp NeighborhoodVariant.h | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -I. $(VARIANT_TEST_SRC) NeighborhoodVariant.cpp -o $@

test_neighborhood_variant: $(VARIANT_TEST_BIN)
	$(VARIANT_TEST_BIN)

$(TRANSACTION_TEST_BIN): $(TRANSACTION_TEST_SRC) $(LIB) CurrentSolutionTransaction.h | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(TRANSACTION_TEST_SRC) -L. -llns_clean -o $@

test_current_solution_transaction: $(TRANSACTION_TEST_BIN)
	$(TRANSACTION_TEST_BIN)

$(INITIAL_MDD_ID_TEST_BIN): $(INITIAL_MDD_ID_TEST_SRC) $(LIB) Lazy_SAT_Solve.h | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(INITIAL_MDD_ID_TEST_SRC) -L. -llns_clean -o $@

test_initial_mdd_agent_ids: $(INITIAL_MDD_ID_TEST_BIN)
	$(INITIAL_MDD_ID_TEST_BIN)

$(SOLVE_RESULT_TEST_BIN): $(SOLVE_RESULT_TEST_SRC) $(LIB) LNS.h SolveStatus.h | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(SOLVE_RESULT_TEST_SRC) -L. -llns_clean -o $@

test_solve_result: $(SOLVE_RESULT_TEST_BIN)
	$(SOLVE_RESULT_TEST_BIN)

$(MINISAT_STATUS_TEST_BIN): $(MINISAT_STATUS_TEST_SRC) $(LIB) | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(MINISAT_STATUS_TEST_SRC) -L. -llns_clean -o $@

test_minisat_unsat_status: $(MINISAT_STATUS_TEST_BIN)
	$(MINISAT_STATUS_TEST_BIN)

$(SAT_PROTOCOL_TEST_BIN): $(SAT_PROTOCOL_TEST_SRC) $(LIB) | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(SAT_PROTOCOL_TEST_SRC) -L. -llns_clean -o $@

test_sat_protocol_characterization: $(SAT_PROTOCOL_TEST_BIN)
	$(SAT_PROTOCOL_TEST_BIN)

$(PSEUDO_SEGMENT_TEST_BIN): $(PSEUDO_SEGMENT_TEST_SRC) $(LIB) Create_Local_Problem.h | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(PSEUDO_SEGMENT_TEST_SRC) -L. -llns_clean -o $@

test_pseudo_agent_segmentation: $(PSEUDO_SEGMENT_TEST_BIN)
	$(PSEUDO_SEGMENT_TEST_BIN)

$(PSEUDO_REFRESH_TEST_BIN): $(PSEUDO_REFRESH_TEST_SRC) $(LIB) Create_Local_Problem.h Current_Solution.h | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(PSEUDO_REFRESH_TEST_SRC) -L. -llns_clean -o $@

test_pseudo_agent_refresh_reassembly: $(PSEUDO_REFRESH_TEST_BIN)
	$(PSEUDO_REFRESH_TEST_BIN)

$(LOCAL_STATE_TEST_BIN): $(LOCAL_STATE_TEST_SRC) $(LIB) Create_Local_Problem.h Waiting_time_Solve.h | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(LOCAL_STATE_TEST_SRC) -L. -llns_clean -o $@

test_local_zone_state_validation: $(LOCAL_STATE_TEST_BIN)
	$(LOCAL_STATE_TEST_BIN)

$(WAITING_BUDGET_TEST_BIN): $(WAITING_BUDGET_TEST_SRC) $(LIB) Waiting_time_Solve.h | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(WAITING_BUDGET_TEST_SRC) -L. -llns_clean -o $@

test_waiting_budget_conservation: $(WAITING_BUDGET_TEST_BIN)
	$(WAITING_BUDGET_TEST_BIN)

$(GRID_WALKABILITY_TEST_BIN): $(GRID_WALKABILITY_TEST_SRC) $(LIB) Grid.h Load_LNSProblem.h Local_Zone.h SolutionVerifier.h Solve_Local_Zone.h | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(GRID_WALKABILITY_TEST_SRC) -L. -llns_clean -o $@

test_grid_walkability: $(GRID_WALKABILITY_TEST_BIN)
	$(GRID_WALKABILITY_TEST_BIN)

$(CONFIG_CHARACTERIZATION_TEST_BIN): $(CONFIG_CHARACTERIZATION_TEST_SRC) $(LIB) LNS.h Lazy_SAT_Solve.h | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(CONFIG_CHARACTERIZATION_TEST_SRC) -L. -llns_clean -o $@

test_solver_configuration_characterization: $(CONFIG_CHARACTERIZATION_TEST_BIN)
	$(CONFIG_CHARACTERIZATION_TEST_BIN)

$(SOLVER_CONFIG_TEST_BIN): $(SOLVER_CONFIG_TEST_SRC) SolverConfiguration.cpp SolverConfiguration.h NeighborhoodVariant.cpp NeighborhoodVariant.h | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -I. $(SOLVER_CONFIG_TEST_SRC) SolverConfiguration.cpp NeighborhoodVariant.cpp -o $@

test_solver_configuration: $(SOLVER_CONFIG_TEST_BIN)
	$(SOLVER_CONFIG_TEST_BIN)

test_batch_backend_rejection: $(RUN_EXPERIMENTS_BIN) $(BATCH_BACKEND_TEST_SRC)
	sh $(BATCH_BACKEND_TEST_SRC) $(RUN_EXPERIMENTS_BIN)

test_supported_build_dependencies: $(LIB) $(SUPPORTED_BUILD_DEPENDENCY_TEST_SRC)
	sh $(SUPPORTED_BUILD_DEPENDENCY_TEST_SRC) $(LIB)

test: test_solution_verifier test_neighborhood_variant test_current_solution_transaction test_initial_mdd_agent_ids test_solve_result test_minisat_unsat_status test_sat_protocol_characterization test_pseudo_agent_segmentation test_pseudo_agent_refresh_reassembly test_local_zone_state_validation test_waiting_budget_conservation test_grid_walkability test_solver_configuration_characterization test_solver_configuration test_batch_backend_rejection test_supported_build_dependencies

# Integration runner used by bounded benchmark checks. It calls LNS and then
# independently verifies the returned path map with SolutionVerifier.
$(LNS_VERIFICATION_BIN): $(LNS_VERIFICATION_SRC) $(LIB) | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(LNS_VERIFICATION_SRC) -L. -llns_clean -o $@

lns_verification_runner: $(LNS_VERIFICATION_BIN)

define cpp_rule
$(OBJ_DIR)/$(notdir $(basename $(1))).o: $(1) | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(DEPFLAGS) -c $$< -o $$@
endef


$(foreach src,$(LOCAL_SOURCES),$(eval $(call cpp_rule,$(src))))
$(foreach src,$(SHARED_SOURCES),$(eval $(call cpp_rule,$(src))))

clean:
	rm -rf $(OBJ_DIR) $(LIB) $(MAIN) $(RUN_EXPERIMENTS_BIN)

asan:
	$(MAKE) clean
	$(MAKE) CXXFLAGS="$(filter-out -O%,$(CXXFLAGS)) $(DEBUG_FLAGS) $(SANITIZE_FLAGS)" \
		LDFLAGS="$(LDFLAGS) $(SANITIZE_FLAGS)"

-include $(DEPS)
