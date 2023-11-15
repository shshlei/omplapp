/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

#include "CSCRRT.h"
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

ompl::geometric::CSCRRT::CSCRRT(const base::SpaceInformationPtr &si)
  : base::Planner(si, "CSCRRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &CSCRRT::setRange, &CSCRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &CSCRRT::setGoalBias, &CSCRRT::getGoalBias, "0.:.05:1.");
}

ompl::geometric::CSCRRT::~CSCRRT()
{
    freeMemory();
}

void ompl::geometric::CSCRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });

    if (useCollisionCertificateChecker_)
    {
        if (!onn_)
            onn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<SafetyCertificate *>(this));
        onn_->setDistanceFunction([this](const SafetyCertificate *a, const SafetyCertificate *b) { return distanceFunction(a, b); });
    }
}

void ompl::geometric::CSCRRT::freeMemory()
{
    for (auto &msc : snne_)
        delete msc;

    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }

    if (onn_)
    {
        std::vector<SafetyCertificate *> certificates;
        onn_->list(certificates);
        for (auto &msc : certificates)
        {
            if (msc->sc != nullptr)
                si_->freeState(msc->sc);
            delete msc;
        }
    }
}

void ompl::geometric::CSCRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    if (onn_)
        onn_->clear();
    snne_.clear();
    lastGoalMotion_ = nullptr;
}

ompl::base::PlannerStatus ompl::geometric::CSCRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);

        SafetyCertificate *msc = new SafetyCertificate();
        msc->sc = motion->state;
        msc->dist = si_->clearance(msc->sc);
        if (msc->dist < certificateR_)
        {
            delete msc;
            msc = nullptr;
        }
        else
            snne_.push_back(msc);
        motion->msc = msc;
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    while (!ptc)
    {
        /* sample random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        bool newsc = false;
        SafetyCertificate *msc = nullptr; 
        if (!nmotion->msc || si_->distance(nmotion->msc->sc, dstate) >= nmotion->msc->dist) // safety certificate false
        {
            newsc = true;
            msc = new SafetyCertificate();
            msc->sc = si_->allocState();
            si_->copyState(msc->sc, dstate);

            if (useCollisionCertificateChecker_ && onn_->size())
            {
                SafetyCertificate *nsc = onn_->nearest(msc);
                if (si_->distance(nsc->sc, dstate) <= nsc->dist) // collision certificate true
                {
                    si_->freeState(msc->sc);
                    delete msc;
                    continue;
                }
            }

            double dist = 0.0;
            if (si_->isValid(dstate, dist))
            {
                si_->freeState(msc->sc);
                if (dist > certificateR_)
                    msc->dist = dist;
                else 
                {
                    delete msc;
                    msc = nullptr;
                }
            }
            else 
            {
                if (useCollisionCertificateChecker_)
                {
                    msc->dist = -dist;
                    if (msc->dist > certificateR_)
                        onn_->add(msc);
                    else 
                    {
                        si_->freeState(msc->sc);
                        delete msc;
                    }
                }
                else 
                {
                    si_->freeState(msc->sc);
                    delete msc;
                }
                continue;
            }
        }
        else
            msc = nmotion->msc;
        bool valid = !newsc || (nmotion->msc && msc ? checkInterMotion(nmotion->state, dstate, nmotion->msc->sc, nmotion->msc->dist, dstate, msc->dist) : si_->checkMotion(nmotion->state, dstate, true));
        if (valid)
        {
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
            nn_->add(motion);
            nmotion = motion;

            motion->msc = msc; // add certificate
            if (newsc && msc)
            {
                msc->sc = motion->state;
                snne_.push_back(msc);
            }

            double dist = 0.0;
            bool sat = goal->isSatisfied(nmotion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = nmotion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = nmotion;
            }
        }
        else if (newsc)
            delete msc;
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}

void ompl::geometric::CSCRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}

bool ompl::geometric::CSCRRT::checkInterMotion(const base::State *s1, const base::State *s2, const base::State *sc1, double dist1, const base::State *sc2, double dist2)
{
    bool valid = true;
    int nd = si_->getStateSpace()->validSegmentCount(s1, s2);
    if (nd >= 2)
    {
        double dist = si_->distance(sc1, s2);
        if (dist < dist1)
            return valid;
        dist = si_->distance(sc2, s1);
        if (dist < dist2)
            return valid;
        base::State *st = si_->allocState();
        base::State *st2 = si_->allocState();
        double low = 0.0, high = 1.0, middle = 0.5, delta = 1.0 / (double)nd;
        while (high > low + delta)
        {
            si_->getStateSpace()->interpolate(s1, s2, middle, st);
            double d1 = si_->distance(sc1, st);
            double d2 = si_->distance(sc2, st);
            if (d1 < dist1)
            {
                if (d2 < dist2)
                    break;
                else
                    low = middle;
            }
            else if (d2 < dist2)
                high = middle;
            else if (!si_->isValid(st))
            {
                valid = false;
                break;
            }
            else 
            {
                double temp = middle, middle2 = 0.5 * (low + middle);
                si_->copyState(st2, st);
                while (middle > low + delta)
                {
                    si_->getStateSpace()->interpolate(s1, s2, middle2, st);
                    d1 = si_->distance(sc1, st);
                    if (d1 < dist1)
                        low = middle2;
                    else if (!si_->isValid(st))
                    {
                        valid = false;
                        break;
                    }
                    else if (!si_->checkMotion(st, st2, true))
                    {
                        valid = false;
                        break;
                    }
                    else
                    {
                        si_->copyState(st2, st);
                        middle = middle2;
                    }
                    middle2 = 0.5 * (low + middle);
                }
                if (!valid)
                    break;
                middle = temp, middle2 = 0.5 * (high + middle);
                si_->getStateSpace()->interpolate(s1, s2, middle, st2);
                while (high > middle + delta)
                {
                    si_->getStateSpace()->interpolate(s1, s2, middle2, st);
                    d2 = si_->distance(sc2, st);
                    if (d2 < dist2)
                        high = middle2;
                    else if (!si_->isValid(st))
                    {
                        valid = false;
                        break;
                    }
                    else if (!si_->checkMotion(st2, st, true))
                    {
                        valid = false;
                        break;
                    }
                    else 
                    {
                        si_->copyState(st2, st);
                        middle = middle2;
                    }
                    middle2 = 0.5 * (high + middle);
                }
                break;
            }
            middle = 0.5 * (low + high);
        }
        si_->freeState(st);
        si_->freeState(st2);
    }

    return valid;
}
