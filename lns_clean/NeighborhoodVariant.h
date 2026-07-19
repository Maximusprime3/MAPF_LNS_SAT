#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

// Public algorithm variants differ only in how the local-zone radius grows
// after an unsuccessful solve.
enum class NeighborhoodVariant {
    LnsSat,
    InitialRadius2,
    FixedStep2,
    IncreasingStep
};

enum class NeighborhoodGrowth {
    FixedStep,
    IncreasingStep
};

// Fully resolved numeric policy used by the local-zone implementation. Keeping
// this separate from CLI names makes experiment behavior explicit and testable.
struct NeighborhoodPolicy {
    NeighborhoodVariant variant;
    const char* canonical_name;
    int initial_radius;
    int fixed_step;
    NeighborhoodGrowth growth;
};

NeighborhoodPolicy neighborhood_policy(NeighborhoodVariant variant);

// Accepted names are case-insensitive and ignore '-' and '_', so both paper
// names such as InitialRadius2 and CLI names such as initial-radius-2 work.
std::optional<NeighborhoodVariant> parse_neighborhood_variant(const std::string& value);

const char* neighborhood_variant_name(NeighborhoodVariant variant);

// failed_attempt_count is one for the first failure, two for the second, etc.
int next_neighborhood_radius(
    const NeighborhoodPolicy& policy,
    int current_radius,
    int failed_attempt_count);

// Convenience API used by deterministic tests and experiment reporting.
std::vector<int> neighborhood_radius_sequence(
    NeighborhoodVariant variant,
    std::size_t attempt_count);
