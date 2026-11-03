#include "Intersection.hpp"

#include <limits>
#include <stdexcept>

namespace dsf::mobility {
  void Intersection::setCapacity(std::size_t const capacity) {
    if (capacity < this->nAgents()) {
      throw std::runtime_error(std::format(
          "Intersection capacity ({}) is smaller than the current queue size ({}).",
          capacity,
          this->nAgents()));
    }
    RoadJunction::setCapacity(capacity);
  }

}  // namespace dsf::mobility