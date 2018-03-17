// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include <algorithm>
#include <array>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "glog/logging.h"

namespace utils {

template <typename T, int N>
static constexpr int array_size(const T (&)[N]) {
  return N;
}

template <typename T, uint64_t N>
static constexpr int array_size(const std::array<T, N>&) {
  return N;
}

template <typename T>
std::string ArrayToString(const T& array) {
  std::ostringstream str_out;
  str_out << "[";
  bool first = true;
  for (const auto& item : array) {
    if (first) {
      first = false;
    } else {
      str_out << ", ";
    }
    str_out << item;
  }
  str_out << "]";
  return str_out.str();
}

template <typename T>
std::string SetToString(const std::unordered_set<T>& set) {
  std::vector<T> array(set.begin(), set.end());
  std::sort(array.begin(), array.end());
  return ArrayToString(array);
}

// In a map, returns a const reference of an element's value using its key. If the key does not
// exist, then crash. It's nearly same as std::map::at() or std::unorderd_map::at(), except that
// at() method throws an exception, which can be relayed upwards on the call stack, so the true
// place that causes the exception is lost when the program crashes because of the uncaught
// exception.
// This version assumes the key is printable.
template <class Container>
const typename Container::value_type::second_type& FindOrDie(
    const Container& container, const typename Container::value_type::first_type& key) {
  const auto it = container.find(key);
  CHECK(it != container.end()) << "Map key not found: " << key;
  return it->second;
}

// Same as above, but returns a non-const reference.
template <class Container>
typename Container::value_type::second_type* FindOrDie(
    Container* container, const typename Container::value_type::first_type& key) {
  auto it = container->find(key);
  CHECK(it != container->end()) << "Map key not found: " << key;
  return &it->second;
}

// In a const map, returns a const pointer of an element's value using its key. If the key does not
// exist, then return nullptr.
template <class Container>
const typename Container::value_type::second_type* FindOrNull(
    const Container& container, const typename Container::value_type::first_type& key) {
  const auto it = container.find(key);
  return it == container.end() ? nullptr : &it->second;
}

// Same as above, but returns a non-const pointer.
template <class Container>
typename Container::value_type::second_type* FindOrNull(
    Container* container, const typename Container::value_type::first_type& key) {
  const auto it = container->find(key);
  return it == container->end() ? nullptr : &it->second;
}

namespace internal {

template <typename BidirIt>
bool NextCombination(BidirIt base_begin, BidirIt base_end, BidirIt target_begin,
                     BidirIt target_end) {
  // Base case: target with 0 length has no next combination
  if (target_begin == target_end) {
    return false;
  }
  // Find the last element in the base container
  BidirIt base_pos = std::find(base_begin, base_end, *(target_end - 1));
  CHECK(base_pos != base_end);
  // If there's a next element, use it
  if (base_pos + 1 != base_end) {
    *(target_end - 1) = *(base_pos + 1);
    return true;
  }
  // A carry happens. Recursively call NextCombination
  if (!NextCombination(base_begin, base_end - 1, target_begin, target_end - 1)) {
    return false;
  }
  CHECK(target_end - 1 != target_begin);
  // Starting from the 2nd to the last element
  BidirIt prev_base_pos = std::find(base_begin, base_end, *(target_end - 2));
  CHECK(prev_base_pos + 1 != base_end);
  // Use the next element
  *(target_end - 1) = *(prev_base_pos + 1);
  return true;
}
}  // namespace internal

// This works similarly to std::next_permutation.
//
// The inputs are begin / end of a full set of elements, and begin / end of the current
// combination.
// The function updates the current combination in place, and return true if it succeeded.
//
// Note that the function assumes that both the full set and the combination containers are sorted.
template <typename BidirIt>
bool NextCombination(BidirIt base_begin, BidirIt base_end, BidirIt target_begin,
                     BidirIt target_end) {
  for (auto it = base_begin + 1; it != base_end; it++) {
    CHECK(*(it - 1) < *it);
  }
  for (auto it = target_begin + 1; it != target_end; it++) {
    CHECK(*(it - 1) < *it);
  }
  return internal::NextCombination(base_begin, base_end, target_begin, target_end);
}

}  // namespace utils
