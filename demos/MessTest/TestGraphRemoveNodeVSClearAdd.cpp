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

#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

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

using TreeData = std::shared_ptr<NearestNeighbors<Motion *>>;

void addToTree(TreeData &tree, Motion *motion)
{
    tree->add(motion);
    for (auto & child : motion->children)
        addToTree(tree, child);
}

int main(int /*argc*/, char ** /*argv*/)
{
    auto space(std::make_shared<base::SE3StateSpace>());
    base::RealVectorBounds bounds(3);
    bounds.setLow(-100);
    bounds.setHigh(100);
    space->setBounds(bounds);

    auto si(std::make_shared<base::SpaceInformation>(space));
    auto sampler = si->allocStateSampler();

    TreeData nn = std::make_shared<NearestNeighborsGNATNoThreadSafety<Motion *>>();
    nn->setDistanceFunction([si](const Motion *a, const Motion *b)
                                 {
                                    return si->distance(a->state, b->state); 
                                 });

    Motion *start = new Motion(si);
    sampler->sampleUniform(start->state);
    nn->add(start);

    int total = 1.e4;
    int remove_num = 20;
    for (int i = 0; i < total; i++)
    {
        Motion *motion = new Motion(si);
        sampler->sampleUniform(motion->state);
        Motion *nmotion = nn->nearest(motion);
        motion->parent = nmotion;
        motion->parent->children.push_back(motion);
        nn->add(motion);
    }

    time::point startTime = time::now();
    nn->clear();
    double time_clear = time::seconds(time::now() - startTime);

    startTime = time::now();
    addToTree(nn, start);
    double time_add = time::seconds(time::now() - startTime);

    std::vector<Motion *> motions;
    startTime = time::now();
    nn->list(motions);
    double time_list = time::seconds(time::now() - startTime);

    OMPL_INFORM("Graph clear time %.6f, add time %.6f, list time %.6f", time_clear, time_add, time_list);

    RNG rng;

    double time_remove = 0.;
    for (int i = 0; i < remove_num; i++)
    {
        int ind = rng.uniformInt(0, motions.size() - 1);
        Motion *temp = motions[ind];

        time::point startTime = time::now();
        nn->remove(temp);
        time_remove += time::seconds(time::now() - startTime);

        motions.erase(motions.begin() + ind);
        si->freeState(temp->state);
        delete temp;
    }

    OMPL_INFORM("Graph remove time %.6f, clear_add VS remove ratio %.6f", time_remove, (time_clear + time_add) / time_remove);

    for (auto & motion : motions)
    {
        si->freeState(motion->state);
        delete motion;
    }
    motions.clear();

    sampler.reset();
    nn->clear();
    start = nullptr;

    return 0;
}
