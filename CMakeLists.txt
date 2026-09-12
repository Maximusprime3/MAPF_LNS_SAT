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

# Only the two bundled translation units used by Makefile.
# Do not inherit the bundled CMakeLists.txt's extra solvers/global flags.
add_library(lns_minisat STATIC
    third_party/minisat/minisat/core/Solver.cc
    third_party/minisat/minisat/utils/System.cc)
set_target_properties(lns_minisat PROPERTIES
    CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS YES)
target_include_directories(lns_minisat SYSTEM PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/minisat")
target_compile_definitions(lns_minisat PUBLIC __STDC_FORMAT_MACROS)

# Project-owned adapter, shared utilities, and supported local sources retain
# project warnings. No globs, imported archives, probSAT, or legacy lns/ sources.
add_library(lns_core STATIC
    src/PublicResult.cpp
    src/Sha256.cpp
    src/ExperimentLogger.cpp
    src/Create_Local_Problem.cpp
    src/Current_Solution.cpp
    src/Lazy_SAT_Solve.cpp
    src/Load_LNSProblem.cpp
    src/Local_Zone.cpp
    src/Local_Zone_Builder.cpp
    src/LNS.cpp
    src/NeighborhoodVariant.cpp
    src/SolverConfiguration.cpp
    src/Solve_Local_Zone.cpp
    src/SolutionVerifier.cpp
    src/VerificationHelpers.cpp
    src/Waiting_time_Solve.cpp
    src/SATSolverManager.cpp
    src/cnf/CNF.cpp
    src/cnf/CNFConstructor.cpp
    src/mdd/MDD.cpp
    src/mdd/MDDConstructor.cpp
    src/mdd/MDDNode.cpp
    src/sat/minisat-wrapper.cpp)
lns_project_target(lns_core)
target_include_directories(lns_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(lns_core PRIVATE lns_minisat PUBLIC Threads::Threads)

# Settings are escaped JSON strings; the always-run provenance target samples
# source revision/dirty state at build time, including incremental rebuilds.
function(lns_json_escape input output)
    string(REPLACE "\\" "\\\\" value "${input}")
    string(REPLACE "\"" "\\\"" value "${value}")
    string(REPLACE "\n" "\\n" value "${value}")
    string(REPLACE "\r" "\\r" value "${value}")
    string(REPLACE "\t" "\\t" value "${value}")
    set(${output} "${value}" PARENT_SCOPE)
endfunction()
lns_json_escape("${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION} (${CMAKE_CXX_COMPILER})" compiler)
lns_json_escape("${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_VERSION} ${CMAKE_SYSTEM_PROCESSOR}" platform)
lns_json_escape("${CMAKE_CXX_FLAGS}" common_flags)
set(build_configs Debug Release RelWithDebInfo MinSizeRel ${CMAKE_CONFIGURATION_TYPES} ${CMAKE_BUILD_TYPE})
list(REMOVE_DUPLICATES build_configs)
set(effective_flags "")
set(effective_link_flags "")
foreach(config IN LISTS build_configs)
    string(TOUPPER "${config}" upper_config)
    lns_json_escape("${CMAKE_CXX_FLAGS_${upper_config}}" config_flags)
    lns_json_escape("${CMAKE_EXE_LINKER_FLAGS_${upper_config}}" config_link_flags)
    string(APPEND effective_flags "$<$<CONFIG:${config}>:${config_flags}>")
    string(APPEND effective_link_flags "$<$<CONFIG:${config}>:${config_link_flags}>")
endforeach()
lns_json_escape("sysroot=${CMAKE_SYSROOT}; compiler_target=${CMAKE_CXX_COMPILER_TARGET}; launcher=${CMAKE_CXX_COMPILER_LAUNCHER}" toolchain)
lns_json_escape("${CMAKE_EXE_LINKER_FLAGS}" linker_flags)
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/provenance-settings-$<CONFIG>.json"
    CONTENT "\"compiler\":\"${compiler}\",\"platform\":\"${platform}\",\"configuration\":\"CMake $<CONFIG>; C++17 extensions=ON; ${common_flags} ${effective_flags}; project warnings=-Wall -Wextra -pedantic (GNU/Clang); __STDC_FORMAT_MACROS; linker=${linker_flags} ${effective_link_flags}; Threads::Threads; ${toolchain}\"")
add_custom_target(lns_provenance
    COMMAND "${CMAKE_COMMAND}" "-DROOT=${CMAKE_CURRENT_SOURCE_DIR}"
        "-DSETTINGS=${CMAKE_CURRENT_BINARY_DIR}/provenance-settings-$<CONFIG>.json"
        "-DOUT=${CMAKE_CURRENT_BINARY_DIR}/generated/$<CONFIG>/BuildProvenance.h"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/BuildProvenance.cmake"
    BYPRODUCTS "${CMAKE_CURRENT_BINARY_DIR}/generated/$<CONFIG>/BuildProvenance.h"
    VERBATIM)
add_library(lns_cli STATIC app/PublicCli.cpp)
lns_project_target(lns_cli)
target_link_libraries(lns_cli PUBLIC lns_core)
target_include_directories(lns_cli PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated/$<CONFIG>")
add_dependencies(lns_cli lns_provenance)
add_executable(lns-sat app/main_clean_lns.cpp)
lns_project_target(lns-sat)
target_link_libraries(lns-sat PRIVATE lns_cli)

add_executable(run_batch_experiments
    app/run_batch_experiments.cpp)
lns_project_target(run_batch_experiments)
target_include_directories(run_batch_experiments PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
add_dependencies(run_batch_experiments lns-sat)

add_executable(run_lns_verification app/run_lns_verification.cpp)
lns_project_target(run_lns_verification)
target_link_libraries(run_lns_verification PRIVATE lns_core)

include(CTest)
if(BUILD_TESTING)
    find_program(LNS_SH sh REQUIRED)

    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_executable(public_cli_test_driver tests/public_cli_test_driver.cpp)
    lns_project_target(public_cli_test_driver)
    target_link_libraries(public_cli_test_driver PRIVATE lns_cli)
    add_executable(test_public_format tests/test_public_format.cpp src/Sha256.cpp)
    lns_project_target(test_public_format)
    target_include_directories(test_public_format PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
    add_test(NAME test_public_format COMMAND test_public_format)
    add_test(NAME test_public_cli COMMAND "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_public_cli.py"
        "$<TARGET_FILE:lns-sat>" "$<TARGET_FILE:public_cli_test_driver>"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
    add_test(NAME test_build_provenance COMMAND "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_build_provenance.py"
        "${CMAKE_COMMAND}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/BuildProvenance.cmake")
    set_tests_properties(test_public_cli test_public_format test_build_provenance PROPERTIES TIMEOUT 60)

    # These three tests have the same small source sets as the Make build.
    add_executable(test_solution_verifier
        tests/test_solution_verifier.cpp src/SolutionVerifier.cpp)
    add_executable(test_neighborhood_variant
        tests/test_neighborhood_variant.cpp src/NeighborhoodVariant.cpp)
    add_executable(test_solver_configuration
        tests/test_solver_configuration.cpp
        src/SolverConfiguration.cpp src/NeighborhoodVariant.cpp)

    set(core_tests
        test_audit_regressions
        test_deadlines
        test_tiny_mapf_oracle
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
        add_executable(${test} "tests/${test}.cpp")
        target_link_libraries(${test} PRIVATE lns_core)
    endforeach()

    set(unit_tests test_solution_verifier test_neighborhood_variant
        test_solver_configuration ${core_tests})
    foreach(test IN LISTS unit_tests)
        lns_project_target(${test})
        target_include_directories(${test} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
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
            configure_file("tests/fixtures/${fixture}"
                "tests/${test}/tests/fixtures/${fixture}" COPYONLY)
        endforeach()
    endforeach()

    add_test(NAME test_batch_backend_rejection
        COMMAND "${LNS_SH}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_batch_backend_rejection.sh"
            "$<TARGET_FILE:run_batch_experiments>")

    add_test(NAME test_batch_paths
        COMMAND "${LNS_SH}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_batch_paths.sh"
            "$<TARGET_FILE:run_batch_experiments>"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")

    # Generate inspection data from actual targets, including resolved artifacts.
    set(graph "")
    foreach(target lns_minisat lns_core lns_cli lns-sat run_batch_experiments
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
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_supported_build_dependencies.sh"
            --cmake "${CMAKE_CURRENT_BINARY_DIR}/supported-build-$<CONFIG>.txt"
            "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}"
            "$<TARGET_FILE:lns_core>" "$<TARGET_FILE:lns_minisat>")
    set_tests_properties(test_batch_backend_rejection test_batch_paths test_supported_build_dependencies
        PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}" TIMEOUT 60)

    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/smoke")
    add_test(NAME smoke_lns_verification
        COMMAND run_lns_verification
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/empty-8-8.map"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/empty-8-8-even-1.scen"
            4 0 42 lns-sat)
    set_tests_properties(smoke_lns_verification PROPERTIES
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/smoke"
        TIMEOUT 30 LABELS smoke
        REQUIRED_FILES "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/empty-8-8.map;${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/empty-8-8-even-1.scen")
endif()
