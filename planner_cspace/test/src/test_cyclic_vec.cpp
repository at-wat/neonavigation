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

#include <cmath>
#include <cstddef>

#include <planner_cspace/cyclic_vec.h>

#include <gtest/gtest.h>

namespace planner_cspace
{
TEST(CyclicVec, InitFloat)
{
  const float val[3] =
      {
        1.0, 2.0, 3.0
      };
  CyclicVecFloat<3, 2> v(1.0f, 2.0f, 3.0f);
  const CyclicVecFloat<3, 2> vc(1.0f, 2.0f, 3.0f);

  for (size_t i = 0; i < 3; ++i)
  {
    ASSERT_EQ(v[i], val[i]);
    ASSERT_EQ(vc[i], val[i]);
  }
}

TEST(CyclicVec, InitInt)
{
  const int val[3] =
      {
        1, 2, 3
      };
  CyclicVecInt<3, 2> v(1, 2, 3);
  const CyclicVecInt<3, 2> vc(1, 2, 3);

  for (size_t i = 0; i < 3; ++i)
  {
    ASSERT_EQ(v[i], val[i]);
    ASSERT_EQ(vc[i], val[i]);
  }
}

TEST(CyclicVec, OperatorsFloat)
{
  using Vec3 = CyclicVecFloat<3, 2>;
  Vec3 v1(1.0f, 2.0f, 3.0f);
  Vec3 v12(1.0f, 2.0f, 3.0f);
  Vec3 v2(4.0f, 5.0f, 6.0f);

  ASSERT_EQ(v1, v12);
  ASSERT_NE(v1, v2);
  ASSERT_NE(v12, v2);

  ASSERT_EQ(v1 + v2, Vec3(5.0f, 7.0f, 9.0f));
  ASSERT_EQ(v1 - v2, Vec3(-3.0f, -3.0f, -3.0f));
  ASSERT_EQ(v1 * v2, Vec3(4.0f, 10.0f, 18.0f));
}

TEST(CyclicVec, OperatorsInt)
{
  using Vec3 = CyclicVecInt<3, 2>;
  Vec3 v1(1, 2, 3);
  Vec3 v12(1, 2, 3);
  Vec3 v2(4, 5, 6);

  ASSERT_EQ(v1, v12);
  ASSERT_NE(v1, v2);
  ASSERT_NE(v12, v2);

  ASSERT_EQ(v1 + v2, Vec3(5, 7, 9));
  ASSERT_EQ(v1 - v2, Vec3(-3, -3, -3));
  ASSERT_EQ(v1 * v2, Vec3(4, 10, 18));
}

TEST(CyclicVec, LengthInt)
{
  using Vec3 = CyclicVecInt<3, 2>;
  Vec3 v(3, 4, 5);

  ASSERT_EQ(v.sqlen(), 25);
  ASSERT_EQ(v.len(), 5.0);
  ASSERT_EQ(v.norm(), std::sqrt(50.0f));
}

TEST(CyclicVec, LengthFloat)
{
  using Vec3 = CyclicVecFloat<3, 2>;
  Vec3 v(3.0f, 4.0f, 5.0f);

  ASSERT_EQ(v.sqlen(), 25.0);
  ASSERT_EQ(v.len(), 5.0);
  ASSERT_EQ(v.norm(), std::sqrt(50.0f));
}

TEST(CyclicVec, Cycle)
{
  using Vec3 = CyclicVecInt<3, 2>;
  Vec3 v(3, 4, 5);
  v.cycle(4);
  ASSERT_EQ(v[2], 1);
  v.cycleUnsigned(4);
  ASSERT_EQ(v[2], 1);

  v[2] = 7;
  v.cycle(4);
  ASSERT_EQ(v[2], -1);
  v.cycleUnsigned(4);
  ASSERT_EQ(v[2], 3);
}
}  // namespace planner_cspace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
