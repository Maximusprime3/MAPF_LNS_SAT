#pragma once

#include "Current_Solution.h"

#include <unordered_map>
#include <utility>
#include <vector>

// Provides commit-or-rollback semantics for speculative local repairs.
// Unless commit() is called, every exit path restores both agent paths and
// waiting-time budgets to their state at construction.
class CurrentSolutionTransaction final {
public:
    explicit CurrentSolutionTransaction(CurrentSolution& solution)
        : solution_(solution),
          waiting_times_backup_(solution.backup_waiting_times()),
          paths_backup_(solution.backup_paths()) {}

    ~CurrentSolutionTransaction() {
        if (!committed_) {
            solution_.restore_waiting_times(waiting_times_backup_);
            solution_.restore_paths(paths_backup_);
        }
    }

    CurrentSolutionTransaction(const CurrentSolutionTransaction&) = delete;
    CurrentSolutionTransaction& operator=(const CurrentSolutionTransaction&) = delete;

    // Keep all mutations made since construction. After this call the
    // destructor becomes a no-op.
    void commit() {
        committed_ = true;
    }

private:
    CurrentSolution& solution_;
    std::unordered_map<int, int> waiting_times_backup_;
    std::unordered_map<int, std::vector<std::pair<int, int>>> paths_backup_;
    bool committed_ = false;
};
