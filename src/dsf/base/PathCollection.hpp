#pragma once

#include "../utility/Typedef.hpp"

#include <format>
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

template <>
struct std::formatter<dsf::PathCollection> : std::formatter<std::string> {
  constexpr auto parse(std::format_parse_context& ctx) {
    return std::formatter<std::string>::parse(ctx);
  }
  auto format(const dsf::PathCollection& pathCollection, std::format_context& ctx) const {
    std::string result = "{\n";
    for (const auto& [key, value] : pathCollection) {
      result += std::format("  {}: [", key);
      for (const auto& id : value) {
        result += std::format("{}, ", id);
      }
      if (!value.empty()) {
        result.pop_back();
        result.pop_back();
      }
      result += "],\n";
    }
    result += "}";
    return std::formatter<std::string>::format(result, ctx);
  }
};