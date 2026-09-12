#pragma once

#include "lnssat/Deadline.h"
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using SatLiteral = int;
using SatClause = std::vector<SatLiteral>;

struct SatAssumptions {
    std::vector<SatLiteral> literals;
};

enum class SatResultKind {
    Sat,
    Unsat,
    Interrupted,
    Error,
};

struct SatStatistics {
    int decisions = 0;
    int propagations = 0;
    double solve_time_seconds = 0.0;
};

struct SatOperationResult {
    bool ok = true;
    std::string diagnostic;

    static SatOperationResult error(std::string message) {
        return SatOperationResult{false, std::move(message)};
    }
};

struct SatSolveResult {
    SatResultKind kind = SatResultKind::Error;
    SatStatistics statistics;
    std::string diagnostic;
};

class SatSolver {
public:
    virtual ~SatSolver() = default;

    virtual void set_deadline(SolverDeadline deadline) = 0;
    virtual SatOperationResult reset() = 0;
    virtual SatOperationResult add_clause(const SatClause& clause) = 0;

    virtual SatOperationResult add_clauses(
        const std::vector<SatClause>& clauses) {
        for (const auto& clause : clauses) {
            SatOperationResult result = add_clause(clause);
            if (!result.ok) {
                return result;
            }
        }
        return {};
    }

    virtual SatSolveResult solve() = 0;
    virtual SatSolveResult solve(const SatAssumptions& assumptions) = 0;
    virtual const std::vector<int>& model() const = 0;
};

using SatDiagnosticSink = std::function<void(const std::string&)>;

// MiniSAT is the single implicit backend for the supported artifact.
std::unique_ptr<SatSolver> make_sat_solver(
    SatDiagnosticSink diagnostic_sink = {});
