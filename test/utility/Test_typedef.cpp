#include "dsf/dsf.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include <format>
#include <string_view>

using dsf::FileExt;
using dsf::fileExtFromString;

static_assert(fileExtFromString("csv") == FileExt::CSV);
static_assert(fileExtFromString("json") == FileExt::JSON);
static_assert(fileExtFromString("geojson") == FileExt::GEOJSON);
static_assert(!fileExtFromString("CSV").has_value());
static_assert(!fileExtFromString("txt").has_value());
static_assert(!fileExtFromString("").has_value());

static_assert(std::string_view{dsf::detail::makeVersionString<0, 0, 0>().data()} ==
              "0.0.0");
static_assert(std::string_view{dsf::detail::makeVersionString<10, 9, 255>().data()} ==
              "10.9.255");

TEST_CASE("Version string") {
  SUBCASE("Matches the version numbers") {
    CHECK_EQ(
        dsf::version(),
        std::format("{}.{}.{}", DSF_VERSION_MAJOR, DSF_VERSION_MINOR, DSF_VERSION_PATCH));
  }
  SUBCASE("Is null-terminated") {
    CHECK_EQ(dsf::version().data()[dsf::version().size()], '\0');
  }
}
