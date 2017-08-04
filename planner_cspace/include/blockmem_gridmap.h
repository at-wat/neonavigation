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

#ifndef BLOCKMEM_GRIDMAP_H
#define BLOCKMEM_GRIDMAP_H

#include <cyclic_vec.h>

template <class T, int DIM, int NONCYCLIC, int block_width = 0x20>
class BlockMemGridmap
{
public:
  std::unique_ptr<T[]> c;
  CyclicVecInt<DIM, NONCYCLIC> size;
  CyclicVecInt<DIM, NONCYCLIC> block_size;
  size_t ser_size;
  size_t block_ser_size;
  size_t block_num;
  void clear(const T zero)
  {
    for (size_t i = 0; i < ser_size; i++)
    {
      c[i] = zero;
    }
  }
  void clear_positive(const T zero)
  {
    for (size_t i = 0; i < ser_size; i++)
    {
      if (c[i] >= 0)
        c[i] = zero;
    }
  }
  void reset(const CyclicVecInt<DIM, NONCYCLIC> &size)
  {
    this->size = size;

    for (int i = 0; i < NONCYCLIC; i++)
    {
      if (this->size[i] < static_cast<int>(block_width))
        this->size[i] = block_width;
    }

    block_ser_size = 1;
    block_num = 1;
    for (int i = 0; i < DIM; i++)
    {
      int width;
      if (i < NONCYCLIC)
      {
        width = block_width;
        block_size[i] = (this->size[i] + width - 1) / width;
      }
      else
      {
        width = this->size[i];
        block_size[i] = 1;
      }

      block_ser_size *= width;
      block_num *= block_size[i];
    }
    ser_size = block_ser_size * block_num;

    c.reset(new T[ser_size]);
  }
  explicit BlockMemGridmap(const CyclicVecInt<DIM, NONCYCLIC> &size)
  {
    reset(size);
  }
  BlockMemGridmap()
  {
  }
  void block_addr(const CyclicVecInt<DIM, NONCYCLIC> &pos, size_t &baddr, size_t &addr)
  {
    addr = 0;
    baddr = 0;
    for (int i = 0; i < NONCYCLIC; i++)
    {
      addr *= block_width;
      addr += pos[i] % block_width;
      baddr *= block_size[i];
      baddr += pos[i] / block_width;
    }
    for (int i = NONCYCLIC; i < DIM; i++)
    {
      addr *= size[i];
      addr += pos[i];
    }
  }
  T &operator[](const CyclicVecInt<DIM, NONCYCLIC> &pos)
  {
    size_t baddr, addr;
    block_addr(pos, baddr, addr);
    return c[baddr * block_ser_size + addr];
  }
  const BlockMemGridmap<T, DIM, NONCYCLIC, block_width>
      &operator=(const BlockMemGridmap<T, DIM, NONCYCLIC, block_width> &gm)
  {
    reset(gm.size);
    memcpy(c.get(), gm.c.get(), ser_size);

    return *this;
  }
};

#endif  // BLOCKMEM_GRIDMAP_H
