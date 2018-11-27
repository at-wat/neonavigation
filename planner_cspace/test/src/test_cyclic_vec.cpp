/*
 * Copyright (c) 2017, the neonavigation authors
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

#include <cstddef>

#include <gtest/gtest.h>

#include <planner_cspace/cyclic_vec.h>

TEST(CyclicVec, InitFloat)
{
  const float val[3] =
      {
        1.0, 2.0, 3.0
      };
  CyclicVecFloat<3, 2> v(val);
  const CyclicVecFloat<3, 2> vc(val);

  for (size_t i = 0; i < 3; ++i)
  {
    ASSERT_EQ(v[i], val[i]);
    ASSERT_EQ(vc[i], val[i]);
  }

  const float val2[3] =
      {
        4.0, 5.0, 6.0
      };
  v.set(val2);

  for (size_t i = 0; i < 3; ++i)
  {
    ASSERT_EQ(v[i], val2[i]);
  }

#define VAL3F     \
  {               \
    7.0, 8.0, 9.0 \
  }
  const float val3[3] = VAL3F;
  CyclicVecFloat<3, 2> v2(VAL3F);
  const CyclicVecFloat<3, 2> vc2(VAL3F);

  for (size_t i = 0; i < 3; ++i)
  {
    ASSERT_EQ(v2[i], val3[i]);
    ASSERT_EQ(vc2[i], val3[i]);
  }
}

TEST(CyclicVec, InitInt)
{
  const int val[3] =
      {
        1, 2, 3
      };
  CyclicVecInt<3, 2> v(val);
  const CyclicVecInt<3, 2> vc(val);

  for (size_t i = 0; i < 3; ++i)
  {
    ASSERT_EQ(v[i], val[i]);
    ASSERT_EQ(vc[i], val[i]);
  }

  const int val2[3] =
      {
        4, 5, 6
      };
  v.set(val2);

  for (size_t i = 0; i < 3; ++i)
  {
    ASSERT_EQ(v[i], val2[i]);
  }

#define VAL3I \
  {           \
    7, 8, 9   \
  }
  const int val3[3] = VAL3I;
  CyclicVecInt<3, 2> v2(VAL3I);
  const CyclicVecInt<3, 2> vc2(VAL3I);

  for (size_t i = 0; i < 3; ++i)
  {
    ASSERT_EQ(v2[i], val3[i]);
    ASSERT_EQ(vc2[i], val3[i]);
  }
}

TEST(CyclicVec, OperatorsFloat)
{
  using Vec3 = CyclicVecFloat<3, 2>;
  const float v1c[3] =
      {
        1.0, 2.0, 3.0
      };
  const float v2c[3] =
      {
        4.0, 5.0, 6.0
      };
  Vec3 v1(v1c);
  Vec3 v12(v1c);
  Vec3 v2(v2c);

  ASSERT_EQ(v1, v12);
  ASSERT_NE(v1, v2);
  ASSERT_NE(v12, v2);

  const float v1_plus_v2[3] =
      {
        5.0, 7.0, 9.0
      };
  ASSERT_EQ(v1 + v2, Vec3(v1_plus_v2));

  const float v1_minus_v2[3] =
      {
        -3.0, -3.0, -3.0
      };
  ASSERT_EQ(v1 - v2, Vec3(v1_minus_v2));

  const float v1_mul_v2[3] =
      {
        4.0, 10.0, 18.0
      };
  ASSERT_EQ(v1 * v2, Vec3(v1_mul_v2));
}

TEST(CyclicVec, OperatorsInt)
{
  using Vec3 = CyclicVecInt<3, 2>;
  const int v1c[3] =
      {
        1, 2, 3
      };
  const int v2c[3] =
      {
        4, 5, 6
      };
  Vec3 v1(v1c);
  Vec3 v12(v1c);
  Vec3 v2(v2c);

  ASSERT_EQ(v1, v12);
  ASSERT_NE(v1, v2);
  ASSERT_NE(v12, v2);

  const int v1_plus_v2[3] =
      {
        5, 7, 9
      };
  ASSERT_EQ(v1 + v2, Vec3(v1_plus_v2));

  const int v1_minus_v2[3] =
      {
        -3, -3, -3
      };
  ASSERT_EQ(v1 - v2, Vec3(v1_minus_v2));

  const int v1_mul_v2[3] =
      {
        4, 10, 18
      };
  ASSERT_EQ(v1 * v2, Vec3(v1_mul_v2));
}

TEST(CyclicVec, LengthInt)
{
  using Vec3 = CyclicVecInt<3, 2>;
  const int v1c[3] =
      {
        3, 4, 5
      };
  Vec3 v(v1c);

  ASSERT_EQ(v.sqlen(), 25);
  ASSERT_EQ(v.len(), 5.0);
  ASSERT_EQ(v.norm(), sqrtf(50));
}

TEST(CyclicVec, LengthFloat)
{
  using Vec3 = CyclicVecFloat<3, 2>;
  const float v1c[3] =
      {
        3.0, 4.0, 5.0
      };
  Vec3 v(v1c);

  ASSERT_EQ(v.sqlen(), 25.0);
  ASSERT_EQ(v.len(), 5.0);
  ASSERT_EQ(v.norm(), sqrtf(50.0));
}

TEST(CyclicVec, Cycle)
{
  using Vec3 = CyclicVecInt<3, 2>;
  const int v1c[3] =
      {
        3, 4, 5
      };
  Vec3 v(v1c);
  v.cycle(v[2], 4);
  ASSERT_EQ(v[2], 1);
  v.cycleUnsigned(v[2], 4);
  ASSERT_EQ(v[2], 1);

  v[2] = 7;
  v.cycle(v[2], 4);
  ASSERT_EQ(v[2], -1);
  v.cycleUnsigned(v[2], 4);
  ASSERT_EQ(v[2], 3);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
