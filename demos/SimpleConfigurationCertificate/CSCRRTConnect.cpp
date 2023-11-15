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

#include "CSCRRTConnect.h"
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/util/String.h>

ompl::geometric::CSCRRTConnect::CSCRRTConnect(const base::SpaceInformationPtr &si)
  : base::Planner(si, "CSCRRTConnect")
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &CSCRRTConnect::setRange, &CSCRRTConnect::getRange, "0.:1.:10000.");

    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

ompl::geometric::CSCRRTConnect::~CSCRRTConnect()
{
    freeMemory();
}

void ompl::geometric::CSCRRTConnect::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    tStart_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });

    if (useCollisionCertificateChecker_)
    {
        if (!onn_)
            onn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<SafetyCertificate *>(this));
        onn_->setDistanceFunction([this](const SafetyCertificate *a, const SafetyCertificate *b) { return distanceFunction(a, b); });
    }
}

void ompl::geometric::CSCRRTConnect::freeMemory()
{
    for (auto &msc : snne_)
        delete msc;

    std::vector<Motion *> motions;

    if (tStart_)
    {
        tStart_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }

    if (tGoal_)
    {
        tGoal_->list(motions);
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

void ompl::geometric::CSCRRTConnect::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    if (onn_)
        onn_->clear();
    snne_.clear();
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

ompl::base::PlannerStatus ompl::geometric::CSCRRTConnect::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tStart_->add(motion);

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

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
                (int)(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    bool startTree = true;
    bool solved = false;

    while (!ptc)
    {
        TreeData &tree = startTree ? tStart_ : tGoal_;
        tgi.start = startTree;
        startTree = !startTree;
        TreeData &otherTree = startTree ? tStart_ : tGoal_;

        if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st != nullptr)
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                tGoal_->add(motion);

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

            if (tGoal_->size() == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                break;
            }
        }

        /* sample random state */
        sampler_->sampleUniform(rstate);

        GrowState gs = growTree(tree, tgi, rmotion);

        if (gs != TRAPPED)
        {
            /* remember which motion was just added */
            Motion *addedMotion = tgi.xmotion;

            /* attempt to connect trees */

            /* if reached, it means we used rstate directly, no need to copy again */
            if (gs != REACHED)
                si_->copyState(rstate, addedMotion->state);

            tgi.start = startTree;

            /* if initial progress cannot be done from the otherTree, restore tgi.start */
            GrowState gsc = growTree(otherTree, tgi, rmotion);
            if (gsc == TRAPPED)
                tgi.start = !tgi.start;

            while (gsc == ADVANCED)
                gsc = growTree(otherTree, tgi, rmotion);

            /* update distance between trees */
            const double newDist = tree->getDistanceFunction()(addedMotion, otherTree->nearest(addedMotion));
            if (newDist < distanceBetweenTrees_)
            {
                distanceBetweenTrees_ = newDist;
                // OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
            }

            Motion *startMotion = tgi.start ? tgi.xmotion : addedMotion;
            Motion *goalMotion = tgi.start ? addedMotion : tgi.xmotion;

            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
            {
                // it must be the case that either the start tree or the goal tree has made some progress
                // so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
                // on the solution path
                if (startMotion->parent != nullptr)
                    startMotion = startMotion->parent;
                else
                    goalMotion = goalMotion->parent;

                connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

                /* construct the solution path */
                Motion *solution = startMotion;
                std::vector<Motion *> mpath1;
                while (solution != nullptr)
                {
                    mpath1.push_back(solution);
                    solution = solution->parent;
                }

                solution = goalMotion;
                std::vector<Motion *> mpath2;
                while (solution != nullptr)
                {
                    mpath2.push_back(solution);
                    solution = solution->parent;
                }

                auto path(std::make_shared<PathGeometric>(si_));
                path->getStates().reserve(mpath1.size() + mpath2.size());
                for (int i = mpath1.size() - 1; i >= 0; --i)
                    path->append(mpath1[i]->state);
                for (auto &i : mpath2)
                    path->append(i->state);

                pdef_->addSolutionPath(path, false, 0.0, getName());
                solved = true;
                break;
            }
            else
            {
                // We didn't reach the goal, but if we were extending the start
                // tree, then we can mark/improve the approximate path so far.
                if (tgi.start)
                {
                    // We were working from the startTree.
                    double dist = 0.0;
                    goal->isSatisfied(tgi.xmotion->state, &dist);
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = tgi.xmotion;
                    }
                }
            }
        }
    }

    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                tStart_->size(), tGoal_->size());

    if (approxsol && !solved)
    {
        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (approxsol != nullptr)
        {
            mpath.push_back(approxsol);
            approxsol = approxsol->parent;
        }

        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, true, approxdif, getName());
        return base::PlannerStatus::APPROXIMATE_SOLUTION;
    }

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

ompl::geometric::CSCRRTConnect::GrowState ompl::geometric::CSCRRTConnect::growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion)
{
    /* find closest state in the tree */
    Motion *nmotion = tree->nearest(rmotion);

    /* assume we can reach the state we go towards */
    bool reach = true;

    /* find state to add */
    base::State *dstate = rmotion->state;
    double d = si_->distance(nmotion->state, rmotion->state);
    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tgi.xstate);

        /* Check if we have moved at all. Due to some stranger state spaces (e.g., the constrained state spaces),
         * interpolate can fail and no progress is made. Without this check, the algorithm gets stuck in a loop as it
         * thinks it is making progress, when none is actually occurring. */
        if (si_->equalStates(nmotion->state, tgi.xstate))
            return TRAPPED;

        dstate = tgi.xstate;
        reach = false;
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
                return TRAPPED;
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
            return TRAPPED;
        }
    }
    else
        msc = nmotion->msc;

    bool validMotion = !newsc || (nmotion->msc && msc ? checkInterMotion(nmotion->state, dstate, nmotion->msc->sc, nmotion->msc->dist, dstate, msc->dist) : si_->checkMotion(nmotion->state, dstate, true));
    if (!validMotion)
    {
        if (newsc)
            delete msc;
        return TRAPPED;
    }

    auto *motion = new Motion(si_);
    si_->copyState(motion->state, dstate);
    motion->parent = nmotion;
    motion->root = nmotion->root;
    tree->add(motion);

    motion->msc = msc; // add certificate
    if (newsc && msc)
    {
        msc->sc = motion->state;
        snne_.push_back(msc);
    }

    tgi.xmotion = motion;

    return reach ? REACHED : ADVANCED;
}

void ompl::geometric::CSCRRTConnect::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (tStart_)
        tStart_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state, 1));
        else
        {
            data.addEdge(base::PlannerDataVertex(motion->parent->state, 1), base::PlannerDataVertex(motion->state, 1));
        }
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addGoalVertex(base::PlannerDataVertex(motion->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(motion->state, 2), base::PlannerDataVertex(motion->parent->state, 2));
        }
    }

    // Add the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));

    // Add some info.
    data.properties["approx goal distance REAL"] = ompl::toString(distanceBetweenTrees_);
}

bool ompl::geometric::CSCRRTConnect::checkInterMotion(const base::State *s1, const base::State *s2, const base::State *sc1, double dist1, const base::State *sc2, double dist2)
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
