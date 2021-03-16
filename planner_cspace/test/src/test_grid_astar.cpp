/*
 * Copyright (c) 2019, the neonavigation authors
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

#include <list>
#include <unordered_map>
#include <vector>

#include <boost/thread.hpp>

#include <omp.h>

#include <gtest/gtest.h>

#include <planner_cspace/grid_astar.h>

namespace planner_cspace
{
TEST(GridAstar, ParallelSearch)
{
  using Vec = CyclicVecInt<1, 1>;
  GridAstar<1, 1> as(Vec(16));
  as.setSearchTaskNum(8);
  omp_set_num_threads(2);

  class Model : public GridAstarModelBase<1, 1>
  {
  private:
    std::vector<std::vector<Vec>> search_;

  public:
    Model()
      : search_(16)
    {
      // create search table of graph edges (relative vector to the connected grid)
      // 0: connected to 1-14
      // 1-14: connected to 1-15
      for (int i = 1; i <= 14; ++i)
      {
        search_[0].push_back(Vec(i));
        for (int j = 1; j <= 15; ++j)
        {
          if (i == j)
            continue;
          search_[i].push_back(Vec(j - i));
        }
      }
    }
    float cost(const Vec&, const Vec&, const std::vector<VecWithCost>&, const Vec&) const final
    {
      return 1.0;
    }
    float costEstim(const Vec& s, const Vec& e) const final
    {
      return 0.0;
    }
    const std::vector<Vec>& searchGrids(const Vec& p, const std::vector<VecWithCost>&, const Vec&) const final
    {
      return search_[p[0]];
    }
  };
  Model::Ptr model(new Model());

  const auto cb_progress = [](const std::list<Vec>&, const SearchStats&) -> bool
  {
    return true;
  };

  for (int i = 0; i < 1000; ++i)
  {
    std::list<Vec> path;
    std::vector<Model::VecWithCost> starts;
    starts.emplace_back(Vec(0));
    ASSERT_TRUE(
        as.search(
            starts, Vec(15), path,
            model, cb_progress,
            0, 1.0));

    ASSERT_EQ(path.size(), 3u);
    ASSERT_EQ(path.front(), Vec(0));
    ASSERT_EQ(path.back(), Vec(15));
  }
}

TEST(GridAstar, TimeoutAbort)
{
  using Vec = CyclicVecInt<1, 1>;
  GridAstar<1, 1> as(Vec(2));
  as.setSearchTaskNum(8);
  omp_set_num_threads(2);

  class Model : public GridAstarModelBase<1, 1>
  {
  private:
    std::vector<std::vector<Vec>> search_;

  public:
    Model()
      : search_(3)
    {
      search_[0].push_back(Vec(1));
      search_[1].push_back(Vec(2));
    }
    float cost(const Vec&, const Vec&, const std::vector<VecWithCost>&, const Vec&) const final
    {
      return 1.0;
    }
    float costEstim(const Vec& s, const Vec& e) const final
    {
      return 0.0;
    }
    const std::vector<Vec>& searchGrids(const Vec& p, const std::vector<VecWithCost>&, const Vec&) const final
    {
      return search_[p[0]];
    }
  };
  Model::Ptr model(new Model());

  int cnt(0);
  const auto cb_progress = [&cnt](const std::list<Vec>& /* path_grid */, const SearchStats& stats) -> bool
  {
    switch (cnt++)
    {
      case 0:
        EXPECT_EQ(1u, stats.num_loop);
        EXPECT_EQ(1u, stats.num_search_queue);
        EXPECT_EQ(0u, stats.num_prev_updates);
        EXPECT_EQ(0u, stats.num_total_updates);
        return true;
      case 1:
        EXPECT_EQ(2u, stats.num_loop);
        EXPECT_EQ(1u, stats.num_search_queue);
        EXPECT_EQ(1u, stats.num_prev_updates);
        EXPECT_EQ(1u, stats.num_total_updates);
        return false;
    }
    EXPECT_TRUE(false) << "Search was not aborted";
    return false;
  };

  std::list<Vec> path;
  std::vector<Model::VecWithCost> starts;
  starts.emplace_back(Vec(0));
  ASSERT_FALSE(as.search(starts, Vec(2), path, model, cb_progress, 0, 0.0));
}

TEST(GridAstar, SearchWithMultipleStarts)
{
  using Vec = CyclicVecInt<1, 1>;
  GridAstar<1, 1> as(Vec(16));
  as.setSearchTaskNum(1);

  class Model : public GridAstarModelBase<1, 1>
  {
  private:
    std::vector<std::vector<Vec>> search_;

  public:
    Model()
      : search_(16)
    {
      // create search table of graph edges (relative vector to the connected grid)
      // 0: connected to 2-14
      // 1: connected to 2-14
      // 2-14: connected to 2-15
      for (int i = 2; i <= 14; ++i)
      {
        search_[0].push_back(Vec(i));
        search_[1].push_back(Vec(i - 1));
        for (int j = 2; j <= 15; ++j)
        {
          if (i == j)
            continue;
          search_[i].push_back(Vec(j - i));
        }
      }
    }
    float cost(const Vec&, const Vec&, const std::vector<VecWithCost>&, const Vec&) const final
    {
      return 1.0;
    }
    float costEstim(const Vec& s, const Vec& e) const final
    {
      return 0.0;
    }
    const std::vector<Vec>& searchGrids(const Vec& p, const std::vector<VecWithCost>&, const Vec&) const final
    {
      return search_[p[0]];
    }
  };
  Model::Ptr model(new Model());

  const auto cb_progress = [](const std::list<Vec>&, const SearchStats&) -> bool
  {
    return true;
  };

  for (int add_cost_to = 0; add_cost_to < 2; ++add_cost_to)
  {
    std::vector<GridAstar<1, 1>::VecWithCost> starts;
    starts.emplace_back(Vec(0));
    starts.emplace_back(Vec(1));
    starts[add_cost_to].c_ = 0.1;

    std::list<Vec> path;
    ASSERT_TRUE(
        as.search(
            starts, Vec(15), path,
            model, cb_progress,
            0, 1.0))
        << add_cost_to;
    ASSERT_EQ(path.size(), 3u);
    ASSERT_EQ(path.back(), Vec(15));
    if (add_cost_to == 0)
      ASSERT_EQ(path.front(), Vec(1));
    else
      ASSERT_EQ(path.front(), Vec(0));
  }
}

class GridAstarTestWrapper : public GridAstar<1, 1>
{
public:
  explicit GridAstarTestWrapper(const Vec& size)
    : GridAstar(size)
  {
  }
  std::unordered_map<Vec, Vec, Vec>& parentMap()
  {
    return parents_;
  }
  bool findPath(const std::vector<VecWithCost>& ss, const Vec& e, std::list<Vec>& path) const
  {
    return GridAstar::findPath(ss, e, path);
  }
};

TEST(GridAstar, FindPathLooped)
{
  using Vec = GridAstarTestWrapper::Vec;
  GridAstarTestWrapper as(Vec(4));

  as.parentMap()[Vec(3)] = Vec(2);
  as.parentMap()[Vec(2)] = Vec(1);
  as.parentMap()[Vec(1)] = Vec(2);

  std::list<Vec> path;
  const auto timeout_func = []()
  {
    try
    {
      boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    }
    catch (boost::thread_interrupted&)
    {
      return;
    }
    EXPECT_TRUE(false) << "Looks entered endless loop. Test will be aborted.";
    abort();
  };
  boost::thread timeout(timeout_func);
  std::vector<GridAstarTestWrapper::VecWithCost> starts;
  starts.emplace_back(Vec(0));
  ASSERT_FALSE(as.findPath(starts, Vec(3), path));
  timeout.interrupt();
}

TEST(GridAstar, FindPathUnconnected)
{
  using Vec = GridAstarTestWrapper::Vec;
  GridAstarTestWrapper as(Vec(3));

  as.parentMap()[Vec(2)] = Vec(1);

  std::list<Vec> path;
  std::vector<GridAstarTestWrapper::VecWithCost> starts;
  starts.emplace_back(Vec(0));
  ASSERT_FALSE(as.findPath(starts, Vec(2), path));
}

TEST(GridAstar, FindPath)
{
  using Vec = GridAstarTestWrapper::Vec;
  GridAstarTestWrapper as(Vec(3));

  as.parentMap()[Vec(2)] = Vec(1);
  as.parentMap()[Vec(1)] = Vec(0);

  // findPath must return same result for multiple calls
  for (int i = 0; i < 2; ++i)
  {
    std::list<Vec> path;
    std::vector<GridAstarTestWrapper::VecWithCost> starts;
    starts.emplace_back(Vec(0));
    ASSERT_TRUE(as.findPath(starts, Vec(2), path));
    ASSERT_EQ(path.size(), 3u);
    auto it = path.cbegin();
    ASSERT_EQ(*(it++), Vec(0));
    ASSERT_EQ(*(it++), Vec(1));
    ASSERT_EQ(*it, Vec(2));
  }
}
}  // namespace planner_cspace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
