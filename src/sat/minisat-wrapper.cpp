#include "lnssat/sat/minisat-wrapper.h"

#include "minisat/core/Solver.h"
#include "minisat/core/SolverTypes.h"
#include "minisat/mtl/Vec.h"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

class MiniSatSolver final : public SatSolver {
public:
    explicit MiniSatSolver(SatDiagnosticSink diagnostic_sink)
        : diagnostic_sink_(std::move(diagnostic_sink)) {
        const SatOperationResult result = reset();
        if (!result.ok) {
            initialization_error_ = result.diagnostic;
        }
    }

    void set_deadline(SolverDeadline deadline) override {
        deadline_ = deadline;
        if (solver_) solver_->setTerminationCallback(deadline_ ? deadline_expired : nullptr, this);
    }

    SatOperationResult reset() override {
        try {
            solver_ = std::make_unique<Minisat::Solver>();
            solver_->setTerminationCallback(deadline_ ? deadline_expired : nullptr, this);
            formula_unsat_ = false;
            model_.clear();
            last_decisions_ = 0;
            last_propagations_ = 0;
            initialization_error_.clear();
            return {};
        } catch (const std::exception& error) {
            return SatOperationResult::error(
                std::string("MiniSAT reset failed: ") + error.what());
        }
    }

    SatOperationResult add_clause(const SatClause& clause) override {
        return add_clauses(std::vector<SatClause>{clause});
    }

    SatOperationResult add_clauses(
        const std::vector<SatClause>& clauses) override {
        if (!initialization_error_.empty()) {
            return SatOperationResult::error(initialization_error_);
        }
        if (!solver_) {
            return SatOperationResult::error(
                "MiniSAT session is not initialized");
        }

        try {
            std::unordered_set<std::string> seen;
            std::size_t added = 0;
            std::size_t tautologies = 0;
            std::size_t duplicates = 0;
            std::size_t units = 0;
            std::size_t empties = 0;
            std::size_t simplified_literals = 0;

            for (const auto& raw_clause : clauses) {
                if (solver_deadline_reached(deadline_))
                    return SatOperationResult::error("Wall-clock limit reached during clause loading");
                bool tautology = false;
                std::set<int> literal_set;
                for (int literal : raw_clause) {
                    if (literal == 0) {
                        continue;
                    }
                    if (literal_set.count(-literal) != 0) {
                        tautology = true;
                        break;
                    }
                    literal_set.insert(literal);
                }
                if (tautology) {
                    ++tautologies;
                    continue;
                }

                SatClause normalized(
                    literal_set.begin(), literal_set.end());
                if (raw_clause.size() > normalized.size()) {
                    simplified_literals +=
                        raw_clause.size() - normalized.size();
                }

                std::ostringstream key_stream;
                for (std::size_t index = 0;
                     index < normalized.size(); ++index) {
                    if (index != 0) {
                        key_stream << ',';
                    }
                    key_stream << normalized[index];
                }
                if (!seen.insert(key_stream.str()).second) {
                    ++duplicates;
                    continue;
                }

                if (normalized.empty()) {
                    ++empties;
                } else if (normalized.size() == 1) {
                    ++units;
                }

                if (!add_normalized_clause(normalized)) {
                    formula_unsat_ = true;
                    break;
                }
                ++added;
            }

            if (diagnostic_sink_) {
                std::ostringstream message;
                message << "MiniSAT clause batch: input=" << clauses.size()
                        << ", added=" << added
                        << ", tautologies_skipped=" << tautologies
                        << ", duplicates_skipped=" << duplicates
                        << ", unit_clauses=" << units
                        << ", empty_clauses=" << empties
                        << ", literals_removed=" << simplified_literals
                        << ", solver_clauses=" << solver_->nClauses()
                        << ", solver_variables=" << solver_->nVars();
                diagnostic_sink_(message.str());
            }
            return {};
        } catch (const std::exception& error) {
            return SatOperationResult::error(
                std::string("MiniSAT clause loading failed: ") +
                error.what());
        }
    }

    SatSolveResult solve() override {
        return solve_internal(nullptr);
    }

    SatSolveResult solve(
        const SatAssumptions& assumptions) override {
        return solve_internal(&assumptions);
    }

    const std::vector<int>& model() const override {
        return model_;
    }

private:
    bool add_normalized_clause(const SatClause& clause) {
        if (clause.empty()) {
            return solver_->addEmptyClause();
        }

        Minisat::vec<Minisat::Lit> literals;
        for (int literal : clause) {
            const int variable = std::abs(literal) - 1;
            while (variable >= solver_->nVars()) {
                solver_->newVar();
            }
            literals.push(
                Minisat::mkLit(variable, literal < 0));
        }
        return solver_->addClause(literals);
    }

    SatSolveResult solve_internal(
        const SatAssumptions* assumptions) {
        SatSolveResult result;
        model_.clear();
        if (solver_deadline_reached(deadline_)) {
            result.kind = SatResultKind::Interrupted;
            result.diagnostic = "Wall-clock limit reached before SAT solving";
            return result;
        }
        if (!initialization_error_.empty()) {
            result.diagnostic = initialization_error_;
            return result;
        }
        if (!solver_) {
            result.diagnostic =
                "MiniSAT session is not initialized";
            return result;
        }
        if (formula_unsat_) {
            result.kind = SatResultKind::Unsat;
            return result;
        }

        try {
            Minisat::vec<Minisat::Lit> assumption_literals;
            if (assumptions != nullptr) {
                for (int literal : assumptions->literals) {
                    if (literal == 0) {
                        continue;
                    }
                    const int variable = std::abs(literal) - 1;
                    if (variable < 0 || variable >= solver_->nVars()) {
                        if (diagnostic_sink_) {
                            diagnostic_sink_(
                                "MiniSAT ignored an out-of-range assumption literal");
                        }
                        continue;
                    }
                    assumption_literals.push(
                        Minisat::mkLit(variable, literal < 0));
                }
            }

            const auto start = std::chrono::high_resolution_clock::now();
            // Unlike the bool API, solveLimited preserves the interrupted
            // outcome as l_Undef instead of misclassifying it as UNSAT.
            const auto outcome = solver_->solveLimited(assumption_literals);
            const bool interrupted = outcome == Minisat::l_Undef || solver_deadline_reached(deadline_);
            const bool satisfiable = !interrupted && outcome == Minisat::l_True;
            const auto end = std::chrono::high_resolution_clock::now();

            result.kind = interrupted ? SatResultKind::Interrupted : satisfiable
                              ? SatResultKind::Sat
                              : SatResultKind::Unsat;
            result.statistics.solve_time_seconds =
                std::chrono::duration<double>(end - start).count();

            const std::uint64_t decisions = solver_->decisions;
            const std::uint64_t propagations = solver_->propagations;
            result.statistics.decisions =
                static_cast<int>(decisions - last_decisions_);
            result.statistics.propagations =
                static_cast<int>(propagations - last_propagations_);
            last_decisions_ = decisions;
            last_propagations_ = propagations;

            model_.clear();
            if (satisfiable) {
                model_.resize(solver_->nVars(), 0);
                for (int variable = 0;
                     variable < solver_->nVars(); ++variable) {
                    model_[variable] =
                        solver_->modelValue(variable) ==
                                Minisat::l_True
                            ? 1
                            : 0;
                }
            }
        } catch (const std::exception& error) {
            result.kind = SatResultKind::Error;
            result.diagnostic =
                std::string("MiniSAT solve failed: ") +
                error.what();
        }
        if (solver_deadline_reached(deadline_)) {
            result.kind = SatResultKind::Interrupted;
            result.diagnostic = "Wall-clock limit reached during SAT solving";
            model_.clear();
        }
        return result;
    }

    static bool deadline_expired(void* context) {
        return solver_deadline_reached(static_cast<MiniSatSolver*>(context)->deadline_);
    }
    SolverDeadline deadline_;
    std::unique_ptr<Minisat::Solver> solver_;
    bool formula_unsat_ = false;
    std::vector<int> model_;
    std::uint64_t last_decisions_ = 0;
    std::uint64_t last_propagations_ = 0;
    SatDiagnosticSink diagnostic_sink_;
    std::string initialization_error_;
};

}  // namespace

std::unique_ptr<SatSolver> make_sat_solver(
    SatDiagnosticSink diagnostic_sink) {
    return std::make_unique<MiniSatSolver>(
        std::move(diagnostic_sink));
}
