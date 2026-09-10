cmake_minimum_required(VERSION 3.20)

# Refuse a source-root build before compiler detection writes build products.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "Use an out-of-tree build: cmake -S . -B build")
endif()

project(LNS_SAT LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

function(lns_project_target target)
    set_target_properties(${target} PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS YES)
    target_compile_options(${target} PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall;-Wextra;-pedantic>)
    target_link_libraries(${target} PRIVATE Threads::Threads)
endfunction()

# Only the two bundled translation units used by lns_clean/Makefile.
# Do not inherit the bundled CMakeLists.txt's extra solvers/global flags.
add_library(lns_minisat STATIC
    minisat/minisat-master/minisat/core/Solver.cc
    minisat/minisat-master/minisat/utils/System.cc)
set_target_properties(lns_minisat PROPERTIES
    CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS YES)
target_include_directories(lns_minisat SYSTEM PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/minisat/minisat-master")
target_compile_definitions(lns_minisat PUBLIC __STDC_FORMAT_MACROS)

# Project-owned adapter, shared utilities, and supported local sources retain
# project warnings. No globs, imported archives, probSAT, or legacy lns/ sources.
add_library(lns_core STATIC
    lns_clean/ExperimentLogger.cpp
    lns_clean/Create_Local_Problem.cpp
    lns_clean/Current_Solution.cpp
    lns_clean/Lazy_SAT_Solve.cpp
    lns_clean/Load_LNSProblem.cpp
    lns_clean/Local_Zone.cpp
    lns_clean/Local_Zone_Builder.cpp
    lns_clean/LNS.cpp
    lns_clean/NeighborhoodVariant.cpp
    lns_clean/SolverConfiguration.cpp
    lns_clean/Solve_Local_Zone.cpp
    lns_clean/SolutionVerifier.cpp
    lns_clean/VerificationHelpers.cpp
    lns_clean/Waiting_time_Solve.cpp
    SATSolverManager.cpp
    cnf/CNF.cpp
    cnf/CNFConstructor.cpp
    mdd/MDD.cpp
    mdd/MDDConstructor.cpp
    mdd/MDDNode.cpp
    minisat/minisat-wrapper.cpp)
lns_project_target(lns_core)
target_include_directories(lns_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/lns_clean"
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/cnf"
    "${CMAKE_CURRENT_SOURCE_DIR}/mdd"
    "${CMAKE_CURRENT_SOURCE_DIR}/minisat")
target_link_libraries(lns_core PRIVATE lns_minisat PUBLIC Threads::Threads)

add_executable(lns-sat lns_clean/main_clean_lns.cpp)
lns_project_target(lns-sat)
target_link_libraries(lns-sat PRIVATE lns_core)

# The unchanged batch driver searches its working directory for main_clean_lns.
# Keep that compatibility name entirely inside the CMake output directory.
add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/main_clean_lns"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        "$<TARGET_FILE:lns-sat>"
        "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/main_clean_lns"
    DEPENDS lns-sat
    VERBATIM)
add_custom_target(lns_batch_solver
    DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/main_clean_lns")
add_executable(run_batch_experiments
    lns_clean/run_experiments/run_batch_experiments.cpp)
lns_project_target(run_batch_experiments)
add_dependencies(run_batch_experiments lns_batch_solver)

add_executable(run_lns_verification lns_clean/tests/run_lns_verification.cpp)
lns_project_target(run_lns_verification)
target_link_libraries(run_lns_verification PRIVATE lns_core)

include(CTest)
if(BUILD_TESTING)
    find_program(LNS_SH sh REQUIRED)

    # These three tests have the same small source sets as the Make build.
    add_executable(test_solution_verifier
        lns_clean/tests/test_solution_verifier.cpp lns_clean/SolutionVerifier.cpp)
    add_executable(test_neighborhood_variant
        lns_clean/tests/test_neighborhood_variant.cpp lns_clean/NeighborhoodVariant.cpp)
    add_executable(test_solver_configuration
        lns_clean/tests/test_solver_configuration.cpp
        lns_clean/SolverConfiguration.cpp lns_clean/NeighborhoodVariant.cpp)

    set(core_tests
        test_current_solution_transaction
        test_initial_mdd_agent_ids
        test_solve_result
        test_minisat_unsat_status
        test_sat_protocol_characterization
        test_pseudo_agent_segmentation
        test_pseudo_agent_refresh_reassembly
        test_local_zone_state_validation
        test_waiting_budget_conservation
        test_grid_walkability
        test_solver_configuration_characterization)
    foreach(test IN LISTS core_tests)
        add_executable(${test} "lns_clean/tests/${test}.cpp")
        target_link_libraries(${test} PRIVATE lns_core)
    endforeach()

    set(unit_tests test_solution_verifier test_neighborhood_variant
        test_solver_configuration ${core_tests})
    foreach(test IN LISTS unit_tests)
        lns_project_target(${test})
        set(test_dir "${CMAKE_CURRENT_BINARY_DIR}/tests/${test}")
        file(MAKE_DIRECTORY "${test_dir}")
        add_test(NAME ${test} COMMAND ${test})
        set_tests_properties(${test} PROPERTIES
            WORKING_DIRECTORY "${test_dir}" TIMEOUT 60)
    endforeach()
    # Preserve existing relative fixture paths without writing into the source tree.
    foreach(test test_grid_walkability test_solve_result)
        foreach(fixture all-terrain.map all-terrain-valid.scen
                all-terrain-blocked.scen all-terrain-out-of-bounds.scen)
            configure_file("lns_clean/tests/fixtures/${fixture}"
                "tests/${test}/tests/fixtures/${fixture}" COPYONLY)
        endforeach()
    endforeach()

    add_test(NAME test_batch_backend_rejection
        COMMAND "${LNS_SH}"
            "${CMAKE_CURRENT_SOURCE_DIR}/lns_clean/tests/test_batch_backend_rejection.sh"
            "$<TARGET_FILE:run_batch_experiments>")

    # Generate inspection data from actual targets, including resolved artifacts.
    set(graph "")
    foreach(target lns_minisat lns_core lns-sat run_batch_experiments
            run_lns_verification ${unit_tests})
        string(APPEND graph
            "${target}\n$<TARGET_FILE:${target}>\n"
            "$<TARGET_PROPERTY:${target},SOURCES>\n"
            "$<TARGET_PROPERTY:${target},INCLUDE_DIRECTORIES>\n"
            "$<TARGET_PROPERTY:${target},COMPILE_OPTIONS>\n"
            "$<TARGET_PROPERTY:${target},LINK_LIBRARIES>\n")
    endforeach()
    file(GENERATE OUTPUT "supported-build-$<CONFIG>.txt" CONTENT "${graph}")
    add_test(NAME test_supported_build_dependencies
        COMMAND "${CMAKE_COMMAND}" -E env "AR=${CMAKE_AR}" "NM=${CMAKE_NM}"
            "${LNS_SH}"
            "${CMAKE_CURRENT_SOURCE_DIR}/lns_clean/tests/test_supported_build_dependencies.sh"
            --cmake "${CMAKE_CURRENT_BINARY_DIR}/supported-build-$<CONFIG>.txt"
            "$<TARGET_FILE:lns_core>" "$<TARGET_FILE:lns_minisat>")
    set_tests_properties(test_batch_backend_rejection test_supported_build_dependencies
        PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}" TIMEOUT 60)

    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/smoke")
    add_test(NAME smoke_lns_verification
        COMMAND run_lns_verification
            "${CMAKE_CURRENT_SOURCE_DIR}/mapf-map/empty-8-8.map"
            "${CMAKE_CURRENT_SOURCE_DIR}/mapf-scen-even/scen-even/empty-8-8-even-1.scen"
            4 0 42 lns-sat)
    set_tests_properties(smoke_lns_verification PROPERTIES
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/smoke"
        TIMEOUT 30 LABELS smoke
        REQUIRED_FILES "${CMAKE_CURRENT_SOURCE_DIR}/mapf-map/empty-8-8.map;${CMAKE_CURRENT_SOURCE_DIR}/mapf-scen-even/scen-even/empty-8-8-even-1.scen")
endif()
