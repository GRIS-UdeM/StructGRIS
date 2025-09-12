#pragma once

#include <array>
#include <algorithm>
#include <iostream>

namespace gris {

template<typename T>
concept arithmetic = std::is_arithmetic_v<T>;

/**
 * Simple circular deque style structure with push, pop, max value and very simple size tracking.
 * Written as a fixed sized non allocating replacement for std::deque usage in controlgris.
 */
template <arithmetic T, std::size_t S>
class CircularDeque {
 private:
  int headIndex;

  /**
   * Array of the elements
   */
  std::array<T, S> content;
  std::size_t maxSize;
  int currentSize;

 public:

  CircularDeque()
      : headIndex{0},
        content{},
        maxSize(S),
        currentSize{0}
  {}

  std::size_t getMaxSize() {
    return maxSize;
  }

  std::size_t getCurrentSize() {
    return maxSize;
  }

  /**
   * Pop from the circular buffer.
   * You will get "garbage" if you try to pop while currentSize == 0
   */
  T pop() {
    headIndex = (headIndex - 1) <= 0 ? maxSize - 1 : headIndex - 1;
    currentSize = currentSize - 1 <= 0 ? 0 : currentSize - 1;
    return content[headIndex];
  }

  /**
   * push a new element. You will overwrite the first element if currentSize == maxSize
   */
  void push(T elem) {
    content[headIndex] = elem;
    headIndex += 1;
    headIndex %= maxSize;
    currentSize = (currentSize + 1) >= static_cast<int>(maxSize) ? static_cast<int>(maxSize) : currentSize + 1;
  }

  /**
   * Returns the numerically maximal element.
   * If you call this with currentSize == 0 you will get garbage.
   */
  T max() {
    std::size_t read_idx = headIndex - 1 <= 0 ? maxSize - 1 : headIndex - 1;
    T max_value{content[read_idx]};
    for (int i = 1; i < currentSize; i++) {
      max_value = std::max(content[(read_idx + i) % maxSize], max_value);
    }
    return max_value;
  }

  /**
   * Just marks currentSize to 0, doesn't actually clear any values.
   */
  void clear() {
    currentSize = 0;
  }
};
}  // end namespace mmesh
