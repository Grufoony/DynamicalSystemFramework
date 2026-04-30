#pragma once

#include "../utility/Typedef.hpp"

#include <list>
#include <unordered_map>
#include <vector>

namespace dsf {
  class PathCollection : public std::unordered_map<Id, std::vector<Id>> {
  public:
    using std::unordered_map<Id, std::vector<Id>>::unordered_map;  // Inherit constructors

    /// @brief Explode all possible paths from sourceId to targetId
    /// @param sourceId The starting point of the paths
    /// @param targetId The end point of the paths
    /// @return A list of vectors, each vector representing a path from sourceId to targetId
    std::list<std::vector<Id>> explode(Id const sourceId, Id const targetId) const;
  };
}  // namespace dsf