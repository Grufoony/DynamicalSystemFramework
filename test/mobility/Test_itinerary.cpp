#include <cstdint>

#include "dsf/mobility/Itinerary.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

using namespace dsf;
using namespace dsf::mobility;

TEST_CASE("Itinerary") {
  SUBCASE("Constructors") {
    dsf::Id itineraryId{0};
    dsf::Id destinationId{2};
    Itinerary itinerary{itineraryId, destinationId};
    CHECK_EQ(itinerary.id(), itineraryId);
    CHECK_EQ(itinerary.destination(), destinationId);
  }
}
