#include "lnssat/VerificationHelpers.h"
#include "lnssat/SolutionVerifier.h"

#include <iostream>

bool verify_path_consistency(
    const std::vector<std::pair<int, int>>& path,
    const std::vector<std::vector<char>>& map) {
    const auto report = mapf::verify_path_geometry(path, map);
    for (const auto& issue : report.issues) {
        std::cout << "[VerificationHelpers] ERROR ["
                  << mapf::verification_issue_code_name(issue.code) << "]: "
                  << issue.message << std::endl;
    }
    return report.valid();
}

bool verify_solution_consistency(
    const std::unordered_map<int, std::vector<std::pair<int, int>>>& agent_paths,
    const std::vector<std::pair<int, int>>& starts,
    const std::vector<std::pair<int, int>>& goals,
    const std::vector<std::vector<char>>& map) {
    const auto report = mapf::verify_solution(agent_paths, starts, goals, map);
    for (const auto& issue : report.issues) {
        std::cout << "[VerificationHelpers] ERROR ["
                  << mapf::verification_issue_code_name(issue.code) << "]: "
                  << issue.message << std::endl;
    }
    return report.valid();
}
