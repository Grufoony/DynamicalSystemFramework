#include "dsf/utility/PtrMapView.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <algorithm>
#include <iterator>
#include <memory>
#include <ranges>
#include <stdexcept>
#include <type_traits>

#include <ankerl/unordered_dense.h>

using dsf::Id;
using dsf::PtrMapView;

static_assert(std::ranges::forward_range<PtrMapView<int>>);
static_assert(std::ranges::sized_range<PtrMapView<int>>);
static_assert(std::ranges::view<PtrMapView<int>>);
static_assert(std::ranges::borrowed_range<PtrMapView<int>>);
static_assert(
    std::is_same_v<std::ranges::range_value_t<PtrMapView<int>>, std::pair<Id, int*>>);
static_assert(std::is_same_v<std::ranges::range_value_t<PtrMapView<int const>>,
                             std::pair<Id, int const*>>);
static_assert(
    std::is_same_v<decltype(std::declval<PtrMapView<int const>>().at(0)), int const&>);

TEST_CASE("PtrMapView") {
  ankerl::unordered_dense::map<Id, std::unique_ptr<int>> map;
  map.emplace(1, std::make_unique<int>(10));
  map.emplace(2, std::make_unique<int>(20));
  map.emplace(3, std::make_unique<int>(30));

  SUBCASE("Iteration yields the ids and the owned objects") {
    PtrMapView<int const> view{map};
    std::size_t count{0};
    for (auto const& [id, pValue] : view) {
      CHECK_EQ(pValue, map.at(id).get());
      ++count;
    }
    CHECK_EQ(count, map.size());
  }
  SUBCASE("Map-like queries") {
    PtrMapView<int const> view{map};
    CHECK_EQ(view.size(), 3);
    CHECK_FALSE(view.empty());
    CHECK(view.contains(2));
    CHECK_FALSE(view.contains(4));
    CHECK_EQ(view.at(3), 30);
    CHECK_THROWS_AS(view.at(4), std::out_of_range);

    ankerl::unordered_dense::map<Id, std::unique_ptr<int>> emptyMap;
    CHECK(PtrMapView<int>{emptyMap}.empty());
  }
  SUBCASE("Mutable view allows changing the owned objects") {
    PtrMapView<int> view{map};
    for (auto const& [id, pValue] : view) {
      *pValue += 1;
    }
    view.at(1) = 100;
    CHECK_EQ(*map.at(1), 100);
    CHECK_EQ(*map.at(2), 21);
  }
  SUBCASE("Iterators outlive the view") {
    auto it = PtrMapView<int const>{map}.begin();
    auto const end = PtrMapView<int const>{map}.end();
    std::advance(it, 1);
    auto const& [id, pValue] = *it;
    CHECK_EQ(pValue, map.at(id).get());
    CHECK_EQ(std::distance(it, end), 2);
  }
  SUBCASE("Works with standard algorithms") {
    PtrMapView<int const> view{map};
    auto const it =
        std::ranges::find_if(view, [](auto const& pair) { return *pair.second == 20; });
    REQUIRE_NE(it, view.end());
    CHECK_EQ((*it).first, 2);
    CHECK_EQ(
        std::ranges::count_if(view, [](auto const& pair) { return *pair.second > 15; }),
        2);
  }
}
