#include "PathCollection.hpp"

#include <stdexcept>

namespace {
  std::list<std::vector<dsf::Id>> explodeImpl(
      dsf::PathCollection const& collection,
      dsf::Id const sourceId,
      dsf::Id const targetId,
      ankerl::unordered_dense::set<dsf::Id>& onStack);
}  // namespace

std::list<std::vector<dsf::Id>> dsf::PathCollection::explode(Id const sourceId,
                                                             Id const targetId) const {
  ankerl::unordered_dense::set<Id> onStack;
  return explodeImpl(*this, sourceId, targetId, onStack);
}

namespace {
  using dsf::Id;

  std::list<std::vector<Id>> explodeImpl(dsf::PathCollection const& collection,
                                         Id const sourceId,
                                         Id const targetId,
                                         ankerl::unordered_dense::set<Id>& onStack) {
    std::list<std::vector<Id>> paths;

    // Base case: if source equals target, return a path with just the source
    if (sourceId == targetId) {
      paths.push_back({sourceId});
      return paths;
    }

    // Check if sourceId exists in the map
    auto it = collection.find(sourceId);
    if (it == collection.end()) {
      throw std::runtime_error(
          std::format("Source {} not found in PathCollection", sourceId));
    }

    // The hop graph is built to be acyclic, but a hand-assembled PathCollection need
    // not be; without this guard a cycle recurses until the stack runs out.
    if (!onStack.insert(sourceId).second) {
      return paths;
    }
    struct StackGuard {
      ankerl::unordered_dense::set<Id>& set;
      Id id;
      ~StackGuard() { set.erase(id); }
    } guard{onStack, sourceId};

    auto const& nextHops = it->second;

    // For each possible next hop from sourceId
    for (auto const& hop : nextHops) {
      if (hop == targetId) {
        // Direct path found
        paths.push_back({sourceId, targetId});
      } else {
        // Recursively find paths from hop to target
        auto subPaths = explodeImpl(collection, hop, targetId, onStack);

        // Prepend sourceId to each sub-path
        for (auto& subPath : subPaths) {
          subPath.insert(subPath.begin(), sourceId);
          paths.push_back(std::move(subPath));
        }
      }
    }

    return paths;
  }
}  // namespace