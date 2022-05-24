//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef L3_2D_ARRAY_H__
#define L3_2D_ARRAY_H__

#include <cstdlib>
#include <cassert>

namespace l3
{
template <class Type>
class Array2D
{
public:
  Array2D()
    : width_(0)
    , height_(0)
    , size_(0)
    , array_(nullptr)
  {}

  Array2D(size_t width, size_t height)
    : array_(nullptr)
  {
    init(width, height);
  }

  ~Array2D()
  {
    if (array_)
      delete[] array_;
  }

  void init(size_t width, size_t height)
  {
    if (array_)
      delete[] array_;

    width_ = width;
    height_ = height;
    size_ = width * height;

    array_ = new Type[size_];
  }

  Type& operator[](std::size_t idx)
  {
    assert(array_ != nullptr);
    assert(idx < size_);
    return array_[idx];
  }

  const Type& operator[](std::size_t idx) const
  {
    assert(array_ != nullptr);
    assert(idx < size_);
    return array_[idx];
  }

  inline Type& at(size_t x, size_t y)
  {
    assert(x < width_);
    assert(y < height_);
    return this->operator[](x + y * width_);
  }

  const inline Type& at(size_t x, size_t y) const
  {
    assert(x < width_);
    assert(y < height_);
    return this->operator[](x + y * width_);
  }

  inline bool empty() const { return array_ == nullptr; }
  inline bool has(int x, int y) const { return x >= 0 && y >= 0 && x < width_ && y < height_; }
  inline bool has(size_t x, size_t y) const { return x < width_ && y < height_; }

  inline Type* get() { return array_; }

  inline size_t width() const { return width_; }
  inline size_t height() const { return height_; }
  inline size_t size() const { return size_; }

private:
  size_t width_;
  size_t height_;
  size_t size_;
  Type* array_;
};
}  // namespace l3

#endif
