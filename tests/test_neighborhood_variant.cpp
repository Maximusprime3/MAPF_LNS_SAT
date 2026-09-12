#include "lnssat/NeighborhoodVariant.h"

#include <iostream>
#include <string>
#include <vector>

namespace {

bool expect_sequence(
    const std::string& name,
    NeighborhoodVariant variant,
    const std::vector<int>& expected) {
    const auto actual = neighborhood_radius_sequence(variant, expected.size());
    if (actual == expected) {
        return true;
    }

    std::cerr << "FAIL " << name << ": expected";
    for (int radius : expected) {
        std::cerr << ' ' << radius;
    }
    std::cerr << ", got";
    for (int radius : actual) {
        std::cerr << ' ' << radius;
    }
    std::cerr << std::endl;
    return false;
}

}  // namespace

int main() {
    bool passed = true;
    passed &= expect_sequence(
        "LNS-SAT", NeighborhoodVariant::LnsSat, {1, 2, 3, 4, 5, 6, 7});
    passed &= expect_sequence(
        "InitialRadius2", NeighborhoodVariant::InitialRadius2, {2, 3, 4, 5, 6, 7});
    passed &= expect_sequence(
        "FixedStep2", NeighborhoodVariant::FixedStep2, {1, 3, 5, 7, 9, 11});
    passed &= expect_sequence(
        "IncreasingStep", NeighborhoodVariant::IncreasingStep, {1, 2, 4, 7, 11, 16});

    const auto parsed = parse_neighborhood_variant("InitialRadius2");
    passed &= parsed.has_value() && *parsed == NeighborhoodVariant::InitialRadius2;
    passed &= !parse_neighborhood_variant("unknown-policy").has_value();

    if (!passed) {
        return 1;
    }
    std::cout << "PASS: all neighborhood radius sequences and parser checks" << std::endl;
    return 0;
}
