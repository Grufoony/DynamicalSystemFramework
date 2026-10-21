#pragma once

#include "Typedef.hpp"

#include <cstddef>
#include <iterator>
#include <memory>
#include <ranges>
#include <type_traits>
#include <unordered_map>
#include <utility>

namespace dsf {
  /// @brief Non-owning view over a map of owning pointers, exposing (id, raw pointer) pairs
  /// @details The view does not expose the std::unique_ptr, so callers cannot reset or
  ///          release the owned objects. If T is const, the objects are only accessible as
  ///          const. The iterators only wrap the map's iterators, so they stay valid after
  ///          the view itself is destroyed.
  /// @tparam T The type of the owned objects, optionally const-qualified
  template <typename T>
  class PtrMapView : public std::ranges::view_interface<PtrMapView<T>> {
  public:
    using map_type = std::unordered_map<Id, std::unique_ptr<std::remove_const_t<T>>>;

    /// @brief Forward iterator returning (id, pointer) pairs by value
    class iterator {
      typename map_type::const_iterator m_it;

    public:
      using iterator_concept = std::forward_iterator_tag;
      // The pairs are returned by value, which a legacy forward iterator does not allow
      using iterator_category = std::input_iterator_tag;
      using value_type = std::pair<Id, T*>;
      using reference = value_type;
      using pointer = void;
      using difference_type = std::ptrdiff_t;

      iterator() = default;
      explicit iterator(typename map_type::const_iterator it) noexcept : m_it{it} {}

      // No operator->, like std::views::transform: a reference bound to a member of
      // (*it) extends the pair's lifetime, one bound through a proxy's -> would dangle
      value_type operator*() const noexcept { return {m_it->first, m_it->second.get()}; }

      iterator& operator++() noexcept {
        ++m_it;
        return *this;
      }
      iterator operator++(int) noexcept {
        auto tmp{*this};
        ++m_it;
        return tmp;
      }

      bool operator==(iterator const&) const = default;
    };

  private:
    map_type const* m_map{nullptr};

  public:
    PtrMapView() = default;
    explicit PtrMapView(map_type const& map) noexcept : m_map{&map} {}

    iterator begin() const noexcept { return iterator{m_map->cbegin()}; }
    iterator end() const noexcept { return iterator{m_map->cend()}; }
    iterator cbegin() const noexcept { return begin(); }
    iterator cend() const noexcept { return end(); }

    /// @brief Get the number of elements
    auto size() const noexcept { return m_map->size(); }
    /// @brief Check if the view is empty
    bool empty() const noexcept { return m_map->empty(); }
    /// @brief Check if an element with the given id exists
    bool contains(Id const id) const { return m_map->contains(id); }
    /// @brief Get an element by id
    /// @throws std::out_of_range if no element with the given id exists
    T& at(Id const id) const { return *m_map->at(id); }
  };
}  // namespace dsf

template <typename T>
inline constexpr bool std::ranges::enable_borrowed_range<dsf::PtrMapView<T>> = true;
