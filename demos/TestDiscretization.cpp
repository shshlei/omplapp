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
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/geometric/planners/bispace/CellDiscretization.h>

#include <ompl/util/Console.h>

#include <boost/format.hpp>
#include <iostream>
#include <iomanip>
#include <fstream>

using namespace ompl;

class Motion;
using Cell = ompl::geometric::CellDiscretization<Motion>::Cell;

class Motion
{
public:
    Motion() = default;

    /** \brief Constructor that allocates memory for the state */
    Motion(const base::StateSpacePtr &si) : state(si->allocState())
    {
    }

    ~Motion() = default;

    /** \brief The state contained by the motion */
    base::State *state{nullptr};
    Cell *cell{nullptr};
};

int main(int /*argc*/, char ** /*argv*/)
{
    unsigned int i = 0, count = 1.e4;
    std::vector<Motion *> motions;
    motions.reserve(count);

    ompl::geometric::CellDiscretization<Motion> disc;
    
    auto space(std::make_shared<base::RealVectorStateSpace>(2));
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0.0);
    bounds.setHigh(1.0);
    space->setBounds(bounds);
    space->setup();

    ompl::base::StateSamplerPtr sampler = space->allocStateSampler();

    ompl::base::ProjectionEvaluatorPtr proj = space->getDefaultProjection();
    ompl::geometric::CellDiscretization<Motion>::Coord xcoord(proj->getDimension());
    disc.setDimension(proj->getDimension());
//    if (proj->hasBounds())
//    {
//        ompl::base::RealVectorBounds bounds = proj->getBounds();
//        ompl::geometric::CellDiscretization<Motion>::Coord low(proj->getDimension()), high(proj->getDimension());
//        proj->computeCoordinates(Eigen::Map<Eigen::VectorXd>(bounds.low.data(), bounds.low.size()), low);
//        proj->computeCoordinates(Eigen::Map<Eigen::VectorXd>(bounds.high.data(), bounds.high.size()), high);
//        high.array() -= 1;
//        disc.setBounds(low, high);
//    }

    Motion *motion = new Motion(space);
    sampler->sampleUniform(motion->state);
    proj->computeCoordinates(motion->state, xcoord);
    disc.addMotion(motion, xcoord);
    motions.push_back(motion);

    double minI = std::numeric_limits<double>::max();
    double maxI = -1.0;
    Eigen::IOFormat format(Eigen::StreamPrecision, Eigen::DontAlignCols);
    Eigen::MatrixXd cells(20, 20);
    cells.setZero();
    std::vector<ompl::geometric::CellDiscretization<Motion>::Cell*> cellarray;
    disc.getGrid().getCells(cellarray);
    for (auto & cell : cellarray)
    {
        cells(cell->coord(1), cell->coord(0)) = cell->data->importance;
        if (minI > cell->data->importance)
            minI = cell->data->importance;
        if (maxI < cell->data->importance)
            maxI = cell->data->importance;
    }
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
        disc.countIteration();

        /* Decide on a state to expand from */
        Motion *existing = nullptr;
        ompl::geometric::CellDiscretization<Motion>::Cell *ecell = nullptr;
        disc.selectMotion(existing, ecell);
        assert(existing);

        sampler->sampleUniformNear(xstate, existing->state, maxDistance);

        /* create a motion */
        Motion *motion = new Motion(space);
        space->copyState(motion->state, xstate);
        proj->computeCoordinates(motion->state, xcoord);
        disc.addMotion(motion, xcoord);
        disc.updateCell(ecell);

        motions.push_back(motion);

        if (i % 1000 == 0)
        {
            disc.getGrid().getCells(cellarray);
            for (auto & cell : cellarray)
            {
                cells(cell->coord(1), cell->coord(0)) = cell->data->importance;
                if (minI > cell->data->importance)
                    minI = cell->data->importance;
                if (maxI < cell->data->importance)
                    maxI = cell->data->importance;
            }
            ofs.open(boost::str(boost::format("disc_cell_%i.txt") % pd).c_str());
            ofs << cells.format(format) << std::endl;
            ofs.close();
            cells.setZero();
            cellarray.clear();

            ofs.open(boost::str(boost::format("disc_pd_%i.txt") % pd).c_str());
            for (auto & motion : motions)
            {
                auto rstate = motion->state->as<ompl::base::RealVectorStateSpace::StateType>();
                ofs << rstate->values[0] << " " << rstate->values[1] << std::endl; 
            }
            ofs.close();
            pd++;
        }
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
