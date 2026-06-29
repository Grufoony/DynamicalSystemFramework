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
struct std::formatter<dsf::PathCollection> {
  constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(dsf::PathCollection const& pathCollection, FormatContext&& ctx) const {
    std::string result = "{";
    for (auto const& [key, value] : pathCollection) {
      result += std::format("{}: [", key);
      for (std::size_t i = 0; i < value.size(); ++i) {
        result += std::to_string(value[i]);
        if (i < value.size() - 1) {
          result += ", ";
        }
      }
      result += "], ";
    }
    if (!pathCollection.empty()) {
      result.pop_back();  // Remove the last space
      result.pop_back();  // Remove the last comma
    }
    result += "}";
    return std::format_to(ctx.out(), "{}", result);
  }
};