/*
 * Copyright (c) 2014-2017, the neonavigation authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PLANNER_CSPACE_BLOCKMEM_GRIDMAP_H
#define PLANNER_CSPACE_BLOCKMEM_GRIDMAP_H

#include <bitset>
#include <limits>
#include <memory>

#include <planner_cspace/cyclic_vec.h>

namespace planner_cspace
{
template <class T, int DIM, int NONCYCLIC>
class BlockMemGridmapBase
{
public:
  virtual const CyclicVecInt<DIM, NONCYCLIC>& size() const = 0;
  virtual size_t ser_size() const = 0;
  virtual void clear(const T zero) = 0;
  virtual void clear_partially(
      const T zero, const CyclicVecInt<DIM, NONCYCLIC>& min, const CyclicVecInt<DIM, NONCYCLIC>& max) = 0;
  virtual void copy_partially(
      const BlockMemGridmapBase<T, DIM, NONCYCLIC>& base, const CyclicVecInt<DIM, NONCYCLIC>& min,
      const CyclicVecInt<DIM, NONCYCLIC>& max) = 0;
  virtual void reset(const CyclicVecInt<DIM, NONCYCLIC>& size) = 0;
  virtual T& operator[](const CyclicVecInt<DIM, NONCYCLIC>& pos) = 0;
  virtual const T operator[](const CyclicVecInt<DIM, NONCYCLIC>& pos) const = 0;
  virtual std::function<void(CyclicVecInt<DIM, NONCYCLIC>, size_t&, size_t&)> getAddressor() const = 0;
};

template <class T, int DIM, int NONCYCLIC, int BLOCK_WIDTH = 0x20, bool ENABLE_VALIDATION = false>
class BlockMemGridmap : public BlockMemGridmapBase<T, DIM, NONCYCLIC>
{
private:
  static constexpr bool isPowOf2(const int v)
  {
    return v && ((v & (v - 1)) == 0);
  }
  static_assert(isPowOf2(BLOCK_WIDTH), "BLOCK_WIDTH must be power of 2");
  static_assert(BLOCK_WIDTH > 0, "BLOCK_WIDTH must be >0");

  static constexpr size_t log2Recursive(const size_t v, const size_t depth = 0)
  {
    return v == 1 ? depth : log2Recursive(v >> 1, depth + 1);
  }

protected:
  constexpr static size_t block_bit_ = log2Recursive(BLOCK_WIDTH);
  constexpr static size_t block_bit_mask_ = (1 << block_bit_) - 1;

  std::unique_ptr<T[]> c_;
  CyclicVecInt<DIM, NONCYCLIC> size_;
  CyclicVecInt<DIM, NONCYCLIC> block_size_;
  size_t ser_size_;
  size_t block_ser_size_;
  size_t block_num_;
  T dummy_;

  inline void block_addr(
      const CyclicVecInt<DIM, NONCYCLIC>& pos, size_t& baddr, size_t& addr) const
  {
    addr = 0;
    baddr = 0;
    for (int i = 0; i < NONCYCLIC; i++)
    {
      addr = (addr << block_bit_) + (pos[i] & block_bit_mask_);
      baddr *= block_size_[i];
      baddr += pos[i] >> block_bit_;
    }
    for (int i = NONCYCLIC; i < DIM; i++)
    {
      addr *= size_[i];
      addr += pos[i];
    }
  }

public:
  std::function<void(CyclicVecInt<DIM, NONCYCLIC>, size_t&, size_t&)> getAddressor() const
  {
    return std::bind(
        &BlockMemGridmap<T, DIM, NONCYCLIC, BLOCK_WIDTH, ENABLE_VALIDATION>::block_addr,
        this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  }
  const CyclicVecInt<DIM, NONCYCLIC>& size() const
  {
    return size_;
  }
  size_t ser_size() const
  {
    return ser_size_;
  }
  void clear(const T zero)
  {
    for (size_t i = 0; i < ser_size_; i++)
    {
      c_[i] = zero;
    }
  }
  void clear_partially(
      const T zero, const CyclicVecInt<DIM, NONCYCLIC>& min, const CyclicVecInt<DIM, NONCYCLIC>& max)
  {
    CyclicVecInt<DIM, NONCYCLIC> p = min;
    for (p[0] = min[0]; p[0] < max[0]; ++p[0])
    {
      for (p[1] = min[1]; p[1] < max[1]; ++p[1])
      {
        for (p[2] = min[2]; p[2] < max[2]; ++p[2])
        {
          (*this)[p] = zero;
        }
      }
    }
  }
  void copy_partially(
      const BlockMemGridmapBase<T, DIM, NONCYCLIC>& base, const CyclicVecInt<DIM, NONCYCLIC>& min,
      const CyclicVecInt<DIM, NONCYCLIC>& max)
  {
    CyclicVecInt<DIM, NONCYCLIC> p = min;
    for (p[0] = min[0]; p[0] < max[0]; ++p[0])
    {
      for (p[1] = min[1]; p[1] < max[1]; ++p[1])
      {
        for (p[2] = min[2]; p[2] < max[2]; ++p[2])
        {
          (*this)[p] = base[p];
        }
      }
    }
  }
  void clear_positive(const T zero)
  {
    for (size_t i = 0; i < ser_size_; i++)
    {
      if (c_[i] >= 0)
        c_[i] = zero;
    }
  }
  void reset(const CyclicVecInt<DIM, NONCYCLIC>& size)
  {
    CyclicVecInt<DIM, NONCYCLIC> size_tmp = size;

    for (int i = 0; i < NONCYCLIC; i++)
    {
      if (size_tmp[i] < BLOCK_WIDTH)
        size_tmp[i] = BLOCK_WIDTH;
    }

    block_ser_size_ = 1;
    block_num_ = 1;
    for (int i = 0; i < DIM; i++)
    {
      int width;
      if (i < NONCYCLIC)
      {
        width = BLOCK_WIDTH;
        block_size_[i] = (size_tmp[i] + width - 1) / width;
      }
      else
      {
        width = size_tmp[i];
        block_size_[i] = 1;
      }

      block_ser_size_ *= width;
      block_num_ *= block_size_[i];
    }
    ser_size_ = block_ser_size_ * block_num_;

    c_.reset(new T[ser_size_]);
    size_ = size;
  }
  explicit BlockMemGridmap(const CyclicVecInt<DIM, NONCYCLIC>& size_)
    : BlockMemGridmap()
  {
    reset(size_);
  }
  BlockMemGridmap()
    : dummy_(std::numeric_limits<T>::max())
  {
  }
  T& operator[](const CyclicVecInt<DIM, NONCYCLIC>& pos)
  {
    size_t baddr, addr;
    block_addr(pos, baddr, addr);
    const size_t a = baddr * block_ser_size_ + addr;
    if (ENABLE_VALIDATION)
    {
      if (a >= ser_size_)
        return dummy_;
    }
    return c_[a];
  }
  const T operator[](const CyclicVecInt<DIM, NONCYCLIC>& pos) const
  {
    size_t baddr, addr;
    block_addr(pos, baddr, addr);
    const size_t a = baddr * block_ser_size_ + addr;
    if (ENABLE_VALIDATION)
    {
      if (a >= ser_size_)
        return std::numeric_limits<T>::max();
    }
    return c_[a];
  }
  bool validate(const CyclicVecInt<DIM, NONCYCLIC>& pos, const int tolerance = 0) const
  {
    for (int i = 0; i < NONCYCLIC; i++)
    {
      if (pos[i] < tolerance || size_[i] - tolerance <= pos[i])
        return false;
    }
    for (int i = NONCYCLIC; i < DIM; i++)
    {
      if (pos[i] < 0 || size_[i] <= pos[i])
        return false;
    }
    return true;
  }
  const BlockMemGridmap<T, DIM, NONCYCLIC, BLOCK_WIDTH, ENABLE_VALIDATION>& operator=(
      const BlockMemGridmap<T, DIM, NONCYCLIC, BLOCK_WIDTH, ENABLE_VALIDATION>& gm)
  {
    reset(gm.size_);
    memcpy(c_.get(), gm.c_.get(), ser_size_ * sizeof(T));

    return *this;
  }
};
}  // namespace planner_cspace

#endif  // PLANNER_CSPACE_BLOCKMEM_GRIDMAP_H
