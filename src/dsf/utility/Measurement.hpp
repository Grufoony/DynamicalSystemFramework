#pragma once

#include <algorithm>
#include <cmath>

namespace dsf {
  /// @brief The Measurement struct represents the mean of a quantity and its standard deviation
  /// @tparam T The type of the quantity
  /// @param mean The mean of the sample
  /// @param std The standard deviation of the sample
  /// @param n The number of samples
  /// @param is_valid True if the measurement is valid, false otherwise (i.e. checks if the sample is not empty)
  template <typename T>
  struct Measurement {
    T mean = static_cast<T>(0);
    T std = static_cast<T>(0);
    std::size_t n = 0;
    bool is_valid = false;

    Measurement() = default;
    Measurement(T mean, T std, std::size_t n)
        : mean{mean}, std{std}, n{n}, is_valid{true} {}
    template <typename TContainer>
    Measurement(TContainer const& data) {
      if (data.empty()) {
        return;
      }
      n = data.size();
      is_valid = true;
      auto x_mean = static_cast<T>(0), x2_mean = static_cast<T>(0);

      std::for_each(data.begin(), data.end(), [&x_mean, &x2_mean](auto value) -> void {
        x_mean += value;
        x2_mean += value * value;
      });
      mean = x_mean / n;
      auto const variance = x2_mean / n - mean * mean;
      std = variance > static_cast<T>(0) ? std::sqrt(variance) : static_cast<T>(0);
    }
  };
}  // namespace dsf