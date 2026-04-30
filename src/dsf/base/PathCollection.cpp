#include "PathCollection.hpp"

std::list<std::vector<dsf::Id>> dsf::PathCollection::explode(Id const sourceId,
                                                             Id const targetId) const {
  std::list<std::vector<Id>> paths;

  // Base case: if source equals target, return a path with just the source
  if (sourceId == targetId) {
    paths.push_back({sourceId});
    return paths;
  }

  // Check if sourceId exists in the map
  auto it = this->find(sourceId);
  if (it == this->end()) {
    return paths;  // No paths available from this source
  }

  auto const& nextHops = it->second;

  // For each possible next hop from sourceId
  for (auto const& hop : nextHops) {
    if (hop == targetId) {
      // Direct path found
      paths.push_back({sourceId, targetId});
    } else {
      // Recursively find paths from hop to target
      auto subPaths = explode(hop, targetId);

      // Prepend sourceId to each sub-path
      for (auto& subPath : subPaths) {
        subPath.insert(subPath.begin(), sourceId);
        paths.push_back(std::move(subPath));
      }
    }
  }

  return paths;
}