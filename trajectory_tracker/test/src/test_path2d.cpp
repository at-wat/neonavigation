/*
 * Copyright (c) 2018, the neonavigation authors
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

#include <trajectory_tracker/eigen_line.h>
#include <trajectory_tracker/path2d.h>

namespace
{
double getRemainedDistance(const trajectory_tracker::Path2D& path, const Eigen::Vector2d& p)
{
  const auto nearest = path.findNearest(path.begin() + 1, path.end(), p);
  const Eigen::Vector2d pos_on_line =
      trajectory_tracker::projection2d((nearest - 1)->pos_, nearest->pos_, p);
  return path.remainedDistance(nearest, path.end(), pos_on_line);
}
}  // namespace

TEST(Path2D, RemainedDistance)
{
  trajectory_tracker::Path2D path;
  for (double x = 0; x < 10.0; x += 0.2)
    path.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(x, 0), 0));
  path.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(10.0, 0), 0));

  ASSERT_NEAR(getRemainedDistance(path, Eigen::Vector2d(0, 0)), 10.0, 1e-2);

  ASSERT_NEAR(getRemainedDistance(path, Eigen::Vector2d(8.25, 0)), 1.75, 1e-2);
  ASSERT_NEAR(getRemainedDistance(path, Eigen::Vector2d(10.25, 0)), -0.25, 1e-2);

  ASSERT_NEAR(getRemainedDistance(path, Eigen::Vector2d(8.25, 0.1)), 1.75, 1e-2);
  ASSERT_NEAR(getRemainedDistance(path, Eigen::Vector2d(8.25, -0.1)), 1.75, 1e-2);
  ASSERT_NEAR(getRemainedDistance(path, Eigen::Vector2d(10.25, 0.1)), -0.25, 1e-2);
  ASSERT_NEAR(getRemainedDistance(path, Eigen::Vector2d(10.25, -0.1)), -0.25, 1e-2);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
