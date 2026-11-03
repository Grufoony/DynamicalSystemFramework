#include "dsf/utility/FlowAccumulator.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstddef>
#include <ctime>
#include <vector>

using dsf::FlowAccumulator;

namespace {
  // Runs nSteps time steps with an endless queue of agents: in each step the gate is
  // refilled, then agents pass while it allows. Returns the agents passed in each step.
  std::vector<std::size_t> saturate(double const capacity, std::size_t const nSteps) {
    FlowAccumulator accumulator;
    std::vector<std::size_t> passed;
    for (std::size_t step{0}; step < nSteps; ++step) {
      accumulator.refill(capacity, static_cast<std::time_t>(step));
      std::size_t n{0};
      while (accumulator.canPass()) {
        accumulator.consume();
        ++n;
      }
      passed.push_back(n);
    }
    return passed;
  }
}  // namespace

TEST_CASE("FlowAccumulator") {
  SUBCASE("A new gate is closed until its first refill fills it") {
    FlowAccumulator accumulator;
    CHECK_FALSE(accumulator.canPass());
    accumulator.refill(0.25, 7);
    CHECK(accumulator.canPass());
    CHECK_EQ(accumulator.value(), doctest::Approx(0.25));
  }
  SUBCASE("An idle gate never exceeds its capacity") {
    FlowAccumulator accumulator;
    accumulator.refill(0.25, 0);
    accumulator.refill(0.25, 100);
    CHECK_EQ(accumulator.value(), doctest::Approx(0.25));
    accumulator.consume();
    CHECK_FALSE(accumulator.canPass());
  }
  SUBCASE("A second refill in the same time step has no effect") {
    FlowAccumulator accumulator;
    accumulator.refill(0.25, 0);
    accumulator.consume();
    accumulator.refill(0.25, 1);
    accumulator.refill(0.25, 1);
    CHECK_EQ(accumulator.value(), doctest::Approx(-0.5));
  }
  SUBCASE("Refilling after idle time steps catches up on them") {
    FlowAccumulator eager;
    FlowAccumulator lazy;
    eager.refill(0.3, 0);
    lazy.refill(0.3, 0);
    eager.consume();
    lazy.consume();
    for (std::time_t step{1}; step <= 2; ++step) {
      eager.refill(0.3, step);
    }
    lazy.refill(0.3, 2);
    // Still below the capacity: the idle steps add up
    CHECK_EQ(lazy.value(), doctest::Approx(eager.value()));
    CHECK_EQ(lazy.value(), doctest::Approx(-0.1));
    for (std::time_t step{3}; step <= 10; ++step) {
      eager.refill(0.3, step);
    }
    lazy.refill(0.3, 10);
    // Above the capacity: clipped, as step by step
    CHECK_EQ(lazy.value(), doctest::Approx(eager.value()));
    CHECK_EQ(lazy.value(), doctest::Approx(0.3));
  }
  SUBCASE("A fractional capacity releases one agent every 1 / capacity steps") {
    CHECK_EQ(saturate(0.25, 9), std::vector<std::size_t>{1, 0, 0, 0, 1, 0, 0, 0, 1});
  }
  SUBCASE("Rounding errors do not shorten the headway") {
    // 0.1 - 1 plus nine refills of 0.1 is 0 up to rounding errors: still closed
    auto const passed{saturate(0.1, 21)};
    std::vector<std::size_t> expected(21, 0);
    expected[0] = expected[10] = expected[20] = 1;
    CHECK_EQ(passed, expected);
  }
  SUBCASE("A capacity above one releases at most ceil(capacity) agents per step") {
    auto const passed{saturate(1.8, 10)};
    std::size_t total{0};
    for (auto const n : passed) {
      CHECK((n == 1 || n == 2));
      total += n;
    }
    // Exactly 1.8 agents per step in the long run
    CHECK_EQ(total, 18);
  }
  SUBCASE("An integer capacity releases exactly that many agents per step") {
    CHECK_EQ(saturate(1., 4), std::vector<std::size_t>{1, 1, 1, 1});
    CHECK_EQ(saturate(3., 4), std::vector<std::size_t>{3, 3, 3, 3});
  }
}
