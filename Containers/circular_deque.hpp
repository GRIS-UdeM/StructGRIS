#pragma once

#include <array>
#include <algorithm>
#include <iostream>

namespace gris {

template<typename T>
concept arithmetic = std::is_arithmetic_v<T>;

/**
 * Simple circular deque style structure with push, pop, max value and very simple size tracking.
   Written as a fixed sized non allocating replacement for std::deque usage in controlgris.
 */
template <arithmetic T, std::size_t S>
class circular_deque {
 private:
  int head_index;

  /**
   * Array of the elements
   */
  std::array<T, S> content;

 public:

  std::size_t size;
  int current_size;

  circular_deque()
      : size(S),
        content{},
        head_index{0},
        current_size{0}
  {}


  /**
   * Pop from the circular buffer.
   * You will get "garbage" if you try to pop while current_size == 0
   */
  T pop() {
    head_index = (head_index - 1) <= 0 ? size - 1 : head_index - 1;
    current_size = current_size <= 0 ? 0 : current_size - 1;
    return content[head_index];
  }

  /**
   * push a new element. You will overwrite the first element if current_size == size
   */
  void push(T elem) {
    content[head_index] = elem;
    head_index += 1;
    head_index %= size;
    current_size = (current_size + 1) >= size ? size : current_size + 1;
  }

  /**
   * Returns the numerically maximal element.
   * If you call this with current_size == 0 you will get garbage.
   */
  T max() {
    std::size_t read_idx = head_index - 1 <= 0 ? size - 1 : head_index - 1;
    T max_value{content[read_idx]};
    for (int i = 1; i < current_size; i++) {
      max_value = std::max(content[(read_idx + i) % size], max_value);
    }
    return max_value;
  }

  /**
   * Just marks size to 0, doesn't actually clear any values.
   */
  void clear() {
    current_size = 0;
  }
};
}  // end namespace mmesh
