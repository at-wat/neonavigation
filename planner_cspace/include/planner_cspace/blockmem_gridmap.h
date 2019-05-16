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

#include <limits>

#include <planner_cspace/cyclic_vec.h>

template <class T, int DIM, int NONCYCLIC, int BLOCK_WIDTH = 0x20>
class BlockMemGridmap
{
protected:
  std::unique_ptr<T[]> c_;
  CyclicVecInt<DIM, NONCYCLIC> size_;
  CyclicVecInt<DIM, NONCYCLIC> block_size_;
  size_t ser_size_;
  size_t block_ser_size_;
  size_t block_num_;
  T dummy_;

  void block_addr(
      const CyclicVecInt<DIM, NONCYCLIC>& pos, size_t& baddr, size_t& addr) const
  {
    addr = 0;
    baddr = 0;
    for (int i = 0; i < NONCYCLIC; i++)
    {
      addr *= BLOCK_WIDTH;
      addr += pos[i] % BLOCK_WIDTH;
      baddr *= block_size_[i];
      baddr += pos[i] / BLOCK_WIDTH;
    }
    for (int i = NONCYCLIC; i < DIM; i++)
    {
      addr *= size_[i];
      addr += pos[i];
    }
  }

public:
  std::function<void(CyclicVecInt<3, 2>, size_t&, size_t&)> getAddressor() const
  {
    return std::bind(
        &BlockMemGridmap<T, DIM, NONCYCLIC, BLOCK_WIDTH>::block_addr,
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
    auto size_tmp = size;

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
  {
    reset(size_);
  }
  BlockMemGridmap()
  {
  }
  T& operator[](const CyclicVecInt<DIM, NONCYCLIC>& pos)
  {
    size_t baddr, addr;
    block_addr(pos, baddr, addr);
    if (addr >= block_ser_size_ || baddr >= block_num_)
    {
      dummy_ = std::numeric_limits<T>::max();
      return dummy_;
    }
    return c_[baddr * block_ser_size_ + addr];
  }
  const T operator[](const CyclicVecInt<DIM, NONCYCLIC>& pos) const
  {
    size_t baddr, addr;
    block_addr(pos, baddr, addr);
    if (addr >= block_ser_size_ || baddr >= block_num_)
      return std::numeric_limits<T>::max();
    return c_[baddr * block_ser_size_ + addr];
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
  const BlockMemGridmap<T, DIM, NONCYCLIC, BLOCK_WIDTH>& operator=(
      const BlockMemGridmap<T, DIM, NONCYCLIC, BLOCK_WIDTH>& gm)
  {
    reset(gm.size_);
    memcpy(c_.get(), gm.c_.get(), ser_size_);

    return *this;
  }
};

#endif  // PLANNER_CSPACE_BLOCKMEM_GRIDMAP_H
