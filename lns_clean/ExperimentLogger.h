#pragma once

#include "Metrics.h"

#include <filesystem>
#include <string>
#include <unordered_map>

class ExperimentLogger {
public:
    static ExperimentLogger& instance();

    std::string start_experiment(const std::string& map_path,
                                 const std::string& scenario_path,
                                 int num_agents,
                                 int scenario_index,
                                 int seed);

    void log_lazy_iteration(const std::string& experiment_id,
                            int makespan_attempt,
                            int zone_attempt,
                            int waiting_attempt,
                            const LazySatIterationMetrics& metrics);

    void log_waiting_attempt(const std::string& experiment_id,
                             int makespan_attempt,
                             int zone_attempt,
                             const WaitingAttemptMetrics& metrics);

    void log_local_zone_attempt(const std::string& experiment_id,
                                int makespan_attempt,
                                const LocalZoneAttemptMetrics& metrics);

    void log_makespan_attempt(const std::string& experiment_id,
                              const MakespanAttemptMetrics& metrics);

    void log_experiment_summary(const ExperimentSummaryMetrics& metrics);

private:
    ExperimentLogger();

    void ensure_directory() const;
    void ensure_header(const std::filesystem::path& path, const std::string& header);
    std::filesystem::path file_path(const std::string& filename) const;
    std::string sanitize(const std::string& value) const;

    std::filesystem::path base_dir_;
    mutable std::unordered_map<std::string, bool> header_written_;
};