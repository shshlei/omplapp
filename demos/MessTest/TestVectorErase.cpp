/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Shi Shenglei */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/StateSampler.h>

#include <ompl/config.h>
#include <ompl/util/Console.h>
#include <ompl/util/Time.h>
#include <ompl/util/RandomNumbers.h>
#include <iostream>

using namespace ompl;

class Motion
{
public:
    Motion() = default;

    /** \brief Constructor that allocates memory for the state */
    Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
    {
    }

    ~Motion() = default;

    /** \brief The state contained by the motion */
    base::State *state{nullptr};

    /** \brief The parent motion in the exploration tree */
    Motion *parent{nullptr};

    std::vector<Motion *> children;
};

int main(int /*argc*/, char ** /*argv*/)
{
    auto space(std::make_shared<base::SE3StateSpace>());
    base::RealVectorBounds bounds(3);
    bounds.setLow(-100);
    bounds.setHigh(100);
    space->setBounds(bounds);

    auto si(std::make_shared<base::SpaceInformation>(space));
    auto sampler = si->allocStateSampler();

    std::size_t total = 1.e6;
    std::size_t remove_num = 1000;
    std::vector<Motion *> motions1, motions2;
    motions1.reserve(total);
    motions2.reserve(total);
    for (std::size_t i = 0; i < total; i++)
    {
        Motion *motion = new Motion(si);
        sampler->sampleUniform(motion->state);
        motions1.push_back(motion);

        motion = new Motion(si);
        sampler->sampleUniform(motion->state);
        motions2.push_back(motion);
    }

    RNG rng;

    std::vector<std::size_t> remove_inds;
    remove_inds.reserve(remove_num);
    for (std::size_t i = 0; i < remove_num; i++)
    {
        std::size_t ind = static_cast<std::size_t>(rng.uniformInt(0, total - 1 -i));
        remove_inds.push_back(ind);
    }

    time::point startTime = time::now();
    for (std::size_t i = 0; i < remove_num; i++)
    {
        Motion *motion = motions1[remove_inds[i]];
        motions1.erase(motions1.begin() + remove_inds[i]);
        si->freeState(motion->state);
        delete motion;       
    }
    double time_erase = time::seconds(time::now() - startTime);

    startTime = time::now();
    for (std::size_t i = 0; i < remove_num; i++)
    {
        Motion *motion = motions2[remove_inds[i]];
        std::iter_swap(motions2.begin() + remove_inds[i], motions2.end() - 1);
        motions2.pop_back();
        si->freeState(motion->state);
        delete motion;       
    }
    double time_swap_pop = time::seconds(time::now() - startTime);

    OMPL_INFORM("Vector erase time %.4f, swap pop time %.4f, swap_pop/erase ratio %.4f", time_erase, time_swap_pop, time_swap_pop/time_erase);

    for (auto & motion : motions1)
    {
        si->freeState(motion->state);
        delete motion;
    }
    for (auto & motion : motions2)
    {
        si->freeState(motion->state);
        delete motion;
    }
    motions1.clear();
    motions2.clear();

    sampler.reset();

    return 0;
}
