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

#ifndef PLANNER_CSPACE_BBF_H
#define PLANNER_CSPACE_BBF_H

namespace planner_cspace
{
namespace bbf
{
constexpr float oddsToProbability(const float& o)
{
  return o / (1.0 + o);
}

constexpr float probabilityToOdds(const float& p)
{
  return p / (1.0 - p);
}

const float MIN_PROBABILITY = 0.1;
const float MAX_PROBABILITY = 1.0 - MIN_PROBABILITY;
const float MIN_ODDS = probabilityToOdds(MIN_PROBABILITY);
const float MAX_ODDS = probabilityToOdds(MAX_PROBABILITY);

class BinaryBayesFilter
{
protected:
  float odds_;

public:
  explicit BinaryBayesFilter(
      const float& initial_odds = 1.0) noexcept
    : odds_(initial_odds)
  {
  }
  float update(const float& odds)
  {
    odds_ *= odds;
    if (odds_ < MIN_ODDS)
      odds_ = MIN_ODDS;
    else if (odds_ > MAX_ODDS)
      odds_ = MAX_ODDS;
    return odds_;
  }
  float get() const
  {
    return odds_;
  }
  float getProbability() const
  {
    return oddsToProbability(odds_);
  }
  float getNormalizedProbability() const
  {
    return (getProbability() - MIN_PROBABILITY) / (MAX_PROBABILITY - MIN_PROBABILITY);
  }
};
};  // namespace bbf
}  // namespace planner_cspace

#endif  // PLANNER_CSPACE_BBF_H
