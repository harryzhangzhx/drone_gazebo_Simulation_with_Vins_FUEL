#ifndef _MATRIX_HASH_H
#define _MATRIX_HASH_H

#include <cstddef>    // for std::size_t
#include <functional> // for std::hash

template <typename T>
struct matrix_hash
{
  std::size_t operator()(const T &matrix) const
  {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i)
    {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

#endif
