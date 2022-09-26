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

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/datastructures/NearestNeighborsLinear.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

#include <ompl/util/Console.h>

#include <boost/format.hpp>
#include <iostream>
#include <iomanip>
#include <fstream>

using namespace ompl;

int main(int /*argc*/, char ** /*argv*/)
{
    unsigned int i = 0, count = 1.e4;
    std::vector<Motion *> motions;
    motions.reserve(count);

    auto space(std::make_shared<base::RealVectorStateSpace>(2));
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0.0);
    bounds.setHigh(1.0);
    space->setBounds(bounds);
    space->setup();

    ompl::base::StateSamplerPtr sampler = space->allocStateSampler();

    Motion *motion = new Motion(space);
    sampler->sampleUniform(motion->state);
    proj->computeCoordinates(motion->state, xcoord);
    disc.addMotion(motion, xcoord);
    motions.push_back(motion);

    std::ofstream ofs("disc_cell_0.txt");
    ofs << cells.format(format) << std::endl;
    ofs.close();
    cells.setZero();
    cellarray.clear();

    ofs.open("disc_pd_0.txt");
    auto rstate = motion->state->as<ompl::base::RealVectorStateSpace::StateType>();
    ofs << rstate->values[0] << " " << rstate->values[1] << std::endl; 
    ofs.close();
    int pd = 1;

    double maxDistance = 0.2;

    ompl::base::State *xstate = space->allocState();
    while (++i <= count)
    {
        sampler->sampleUniformNear(xstate, existing->state, maxDistance);

    }

    sampler.reset();
    space->freeState(xstate);
    for (auto & motion : motions)
    {
        space->freeState(motion->state);
        delete motion;
    }
    motions.clear();

    OMPL_INFORM("Min importance %.4f, max importance %.4f", minI, maxI);

    return 0;
}
