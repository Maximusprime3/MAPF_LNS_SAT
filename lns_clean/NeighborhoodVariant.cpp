#include "NeighborhoodVariant.h"

#include <cctype>

namespace {

std::string normalized_name(const std::string& value) {
    std::string normalized;
    normalized.reserve(value.size());
    for (unsigned char character : value) {
        if (std::isalnum(character)) {
            normalized.push_back(static_cast<char>(std::tolower(character)));
        }
    }
    return normalized;
}

}  // namespace

NeighborhoodPolicy neighborhood_policy(NeighborhoodVariant variant) {
    switch (variant) {
        case NeighborhoodVariant::LnsSat:
            return {variant, "lns-sat", 1, 1, NeighborhoodGrowth::FixedStep};
        case NeighborhoodVariant::InitialRadius2:
            return {variant, "initial-radius-2", 2, 1, NeighborhoodGrowth::FixedStep};
        case NeighborhoodVariant::FixedStep2:
            return {variant, "fixed-step-2", 1, 2, NeighborhoodGrowth::FixedStep};
        case NeighborhoodVariant::IncreasingStep:
            return {variant, "increasing-step", 1, 1, NeighborhoodGrowth::IncreasingStep};
    }
    return {NeighborhoodVariant::LnsSat, "lns-sat", 1, 1, NeighborhoodGrowth::FixedStep};
}

std::optional<NeighborhoodVariant> parse_neighborhood_variant(const std::string& value) {
    const std::string normalized = normalized_name(value);
    if (normalized == "lnssat" || normalized == "lns") {
        return NeighborhoodVariant::LnsSat;
    }
    if (normalized == "initialradius2" || normalized == "lnsinit2") {
        return NeighborhoodVariant::InitialRadius2;
    }
    if (normalized == "fixedstep2") {
        return NeighborhoodVariant::FixedStep2;
    }
    if (normalized == "increasingstep") {
        return NeighborhoodVariant::IncreasingStep;
    }
    return std::nullopt;
}

const char* neighborhood_variant_name(NeighborhoodVariant variant) {
    return neighborhood_policy(variant).canonical_name;
}

int next_neighborhood_radius(
    const NeighborhoodPolicy& policy,
    int current_radius,
    int failed_attempt_count) {
    if (failed_attempt_count <= 0) {
        return current_radius;
    }
    if (policy.growth == NeighborhoodGrowth::IncreasingStep) {
        return current_radius + failed_attempt_count;
    }
    return current_radius + policy.fixed_step;
}

std::vector<int> neighborhood_radius_sequence(
    NeighborhoodVariant variant,
    std::size_t attempt_count) {
    std::vector<int> radii;
    radii.reserve(attempt_count);
    if (attempt_count == 0) {
        return radii;
    }

    const NeighborhoodPolicy policy = neighborhood_policy(variant);
    int radius = policy.initial_radius;
    radii.push_back(radius);
    for (std::size_t attempt = 1; attempt < attempt_count; ++attempt) {
        radius = next_neighborhood_radius(policy, radius, static_cast<int>(attempt));
        radii.push_back(radius);
    }
    return radii;
}
