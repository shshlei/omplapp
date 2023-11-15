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

#include <omplapp/config.h>
#include <omplapp/geometry/detail/Box2dStateValidityChecker.h>

#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

// feasible planners
#include "NaiveConnect.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/bispace/RRTBispace.h>
#include <ompl/geometric/planners/bispace/CellBispace.h>
#include <ompl/geometric/planners/ase/BiASE.h>
#include <ompl/geometric/planners/hsc/BiHSC.h>
//#include <ompl/geometric/planners/hsc/HSCASE.h>

// optimal planners
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/BiRRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/bispace/RRTBispacestar.h>
#include <ompl/geometric/planners/bispace/CellBispacestar.h>
#include <ompl/geometric/planners/ase/BiASEstar.h>
#include <ompl/geometric/planners/hsc/BiHSCstar.h>
//#include <ompl/geometric/planners/hsc/HSCASEstar.h>

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <iostream>

using namespace ompl;
namespace ob = ompl::base;
namespace og = ompl::geometric;

// An enum of supported optimal planners, alphabetical order
enum PlannerType
{
    PLANNER_BFMTSTAR,
    PLANNER_BITSTAR,
    PLANNER_CFOREST,
    PLANNER_FMTSTAR,
    PLANNER_INF_RRTSTAR,
    PLANNER_PRMSTAR,
    PLANNER_RRTSTAR,
    PLANNER_BIRRTSTAR,
    PLANNER_SORRTSTAR,
    PLANNER_BIASE,
    PLANNER_BIASESTAR,
    PLANNER_BIHSC,
    PLANNER_BIHSCSTAR,
    PLANNER_HSCASE,
    PLANNER_HSCASESTAR,
    PLANNER_RRT,
    PLANNER_RRTBISPACE,
    PLANNER_RRTBISPACESTAR,
    PLANNER_CELLBISPACE,
    PLANNER_CELLBISPACESTAR,
    PLANNER_RRTCONNECT,
    PLANNER_NAIVECONNECT,
    PLANNER_SBL,
};

ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr si, PlannerType plannerType, double range, double pen_distance, std::shared_ptr<app::Box2dStateValidityChecker> svc)
{
    switch (plannerType)
    {
		case PLANNER_BFMTSTAR:
        {
            auto planner = std::make_shared<og::BFMT>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_BITSTAR:
        {
            auto planner = std::make_shared<og::BITstar>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_CFOREST:
        {
            auto planner = std::make_shared<og::CForest>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_FMTSTAR:
        {
            auto planner = std::make_shared<og::FMT>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_INF_RRTSTAR:
        {
            auto planner = std::make_shared<og::InformedRRTstar>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("collision_range")))
                params.setParam(std::string("collision_range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_PRMSTAR:
        {
            auto planner = std::make_shared<og::PRMstar>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_RRTSTAR:
        {
            auto planner = std::make_shared<og::RRTstar>(si);
//            planner->setKNearest(false);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("collision_range")))
                params.setParam(std::string("collision_range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_BIRRTSTAR:
        {
            auto planner = std::make_shared<og::BiRRTstar>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("collision_range")))
                params.setParam(std::string("collision_range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_SORRTSTAR:
        {
            auto planner = std::make_shared<og::SORRTstar>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_RRT:
        {
            auto planner = std::make_shared<og::RRT>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_RRTBISPACE:
        {
            auto planner = std::make_shared<og::RRTBispace>(si);
            planner->setLazyPath(false);
            planner->setLazyNode(false);
            planner->setUseBispace(true);
            //planner->setRewire(true);
            //planner->setRewireSort(true);
            //planner->setAddIntermediateState(true);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_RRTBISPACESTAR:
        {
            auto planner = std::make_shared<og::RRTBispacestar>(si);
            planner->setLazyPath(false);
            planner->setLazyNode(false);
            planner->setUseBispace(true);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("pen_distance")))
                params.setParam(std::string("pen_distance"), ompl::toString(pen_distance));

            return planner;
            break;
        }
        case PLANNER_CELLBISPACE:
        {
            auto planner = std::make_shared<og::CellBispace>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            return planner;
            break;
        }
        case PLANNER_CELLBISPACESTAR:
        {
            auto planner = std::make_shared<og::CellBispacestar>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            return planner;
            break;
        }
        case PLANNER_BIASE:
        {
            auto planner = std::make_shared<og::BiASE>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("pen_distance")))
                params.setParam(std::string("pen_distance"), ompl::toString(pen_distance));
            planner->setAddIntermediateState(false);
            //planner->setUseBispace(false);
            //planner->setBackRewire(false);
            return planner;
            break;
        }
        case PLANNER_BIASESTAR:
        {
            auto planner = std::make_shared<og::BiASEstar>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("pen_distance")))
                params.setParam(std::string("pen_distance"), ompl::toString(pen_distance));
            planner->setAddIntermediateState(false);
            //planner->setUseBispace(false);
            //planner->setBackRewire(false);
            return planner;
            break;
        }
        case PLANNER_BIHSC:
        {
            auto planner = std::make_shared<og::BiHSC>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            planner->setUseCollisionCertificateChecker(true);
//            planner->setMaxInvalidNodeRatio(0.2);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("pen_distance")))
                params.setParam(std::string("pen_distance"), ompl::toString(pen_distance));
            return planner;
            break;
        }
        case PLANNER_BIHSCSTAR:
        {
            auto planner = std::make_shared<og::BiHSCstar>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            planner->setUseCollisionCertificateChecker(true);
//            planner->setAddIntermediateState(false);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("pen_distance")))
                params.setParam(std::string("pen_distance"), ompl::toString(pen_distance));
            return planner;
            break;
        }
        /*
        case PLANNER_HSCASE:
        {
            auto planner = std::make_shared<og::HSCASE>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            planner->setMaxInvalidNodeRatio(0.05);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_HSCASESTAR:
        {
            auto planner = std::make_shared<og::HSCASEstar>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        */
        case PLANNER_RRTCONNECT:
        {
            auto planner = std::make_shared<og::RRTConnect>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_NAIVECONNECT:
        {
            auto planner = std::make_shared<og::NaiveConnect>(si);
            planner->setUseBispace(true);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            return planner;
            break;
        }
        case PLANNER_SBL:
        {
            auto planner = std::make_shared<og::SBL>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        default:
        {
            OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
            return ob::PlannerPtr(); // Address compiler warning re: no return value.
            break;
        }
    }
}

// Parse the command-line arguments
bool argParse(int argc, char** argv, double &runTime, PlannerType &planner, bool &optimal, std::string &env, std::string &robot, bool &default_param, std::string &spaceType,
        unsigned int &run_cycle, double &checkResolution, bool &doRandomSampling, bool &bispaceSampling);

const base::State* getState(const base::State* state, unsigned int /*index*/)
{
    return state;
}

int main(int argc, char* argv[])
{
    // The parsed arguments
    bool optimal;
    bool default_param;
    double runTime;
    double checkResolution;
    PlannerType plannerType;
    std::string env, robot;
    std::string spaceType;
    unsigned int run_cycle;
    bool doRandomSampling, bispaceSampling;
    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, runTime, plannerType, optimal, env, robot, default_param, spaceType, run_cycle, checkResolution, doRandomSampling, bispaceSampling))
        return -1;

    ob::StateSpacePtr space;
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0.0);
    bounds.setHigh(1.0);
    if (spaceType == "RealVector2")
    {
        auto space1(std::make_shared<ob::RealVectorStateSpace>(2));
        space1->setBounds(bounds);
        space = space1;
    }
    else if (spaceType == "SE2")
    {
        auto space1(std::make_shared<ob::SE2StateSpace>());
        space1->setBounds(bounds);
        space = space1;
    }
    else if (spaceType == "Dubins")
    {
        auto space1(std::make_shared<ob::DubinsStateSpace>(0.1, false));
        space1->setBounds(bounds);
        space = space1;
    }
    else if (spaceType == "ReedsShepp")
    {
        auto space1(std::make_shared<ob::ReedsSheppStateSpace>(0.1));
        space1->setBounds(bounds);
        space = space1;
    }

    auto si(std::make_shared<ob::SpaceInformation>(space));
    auto svc = std::make_shared<app::Box2dStateValidityChecker>(si, app::Motion_2D, space, getState);
    svc->setEnvironmentFile(std::string(OMPLAPP_RESOURCE_DIR) + "/" + env);
    svc->addRobotShape(std::string(OMPLAPP_RESOURCE_DIR) + "/" + robot);

    si->setStateValidityChecker(svc);
    si->setStateValidityCheckingResolution(checkResolution);
    si->setup();

    if (doRandomSampling)
    {
        if (bispaceSampling)
        {
            base::ScopedState<base::RealVectorStateSpace> start(si);
            base::ScopedState<base::RealVectorStateSpace> goal(si);
            start->values[0] = 0.05;
            start->values[1] = 0.05;
            goal->values[0] = 0.95;
            goal->values[1] = 0.95;

            ompl::base::StateSamplerPtr sampler = si->allocStateSampler();
            ompl::base::State *state = si->allocState();

            std::ofstream ofs1("start_bispace_sampling_points.txt", std::ios::binary | std::ios::out);
            std::ofstream ofs2("goal_bispace_sampling_points.txt", std::ios::binary | std::ios::out);
            for (int i = 0; i < 100;)
            {
                sampler->sampleUniform(state);
                if (start.distance(state) <= goal.distance(state) && svc->isValid(state))
                {
                    const auto* rvstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
                    ofs1 << rvstate->values[0] << " " <<  rvstate->values[1] << std::endl;
                    i++;
                }
            }
            for (int i = 0; i < 100;)
            {
                sampler->sampleUniform(state);
                if (start.distance(state) >= goal.distance(state) && svc->isValid(state))
                {
                    const auto* rvstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
                    ofs2 << rvstate->values[0] << " " <<  rvstate->values[1] << std::endl;
                    i++;
                }
            }
            ofs1.close();
            ofs2.close();
            si->freeState(state);
        }
        else 
        {
            ompl::base::StateSamplerPtr sampler = si->allocStateSampler();
            ompl::base::State *state = si->allocState();

            std::ofstream ofs1("start_sampling_points.txt", std::ios::binary | std::ios::out);
            std::ofstream ofs2("goal_sampling_points.txt", std::ios::binary | std::ios::out);
            for (int i = 0; i < 100;)
            {
                sampler->sampleUniform(state);
                if (svc->isValid(state))
                {
                    const auto* rvstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
                    ofs1 << rvstate->values[0] << " " <<  rvstate->values[1] << std::endl;
                    i++;
                }
            }
            for (int i = 0; i < 100;)
            {
                sampler->sampleUniform(state);
                if (svc->isValid(state))
                {
                    const auto* rvstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
                    ofs2 << rvstate->values[0] << " " <<  rvstate->values[1] << std::endl;
                    i++;
                }
            }
            ofs1.close();
            ofs2.close();
            si->freeState(state);
        }
    }

    ompl::geometric::SimpleSetup setup(si);

    if (spaceType == "RealVector2")
    {
        // define start state
        base::ScopedState<base::RealVectorStateSpace> start(setup.getSpaceInformation());
        start->values[0] = 0.05;
        start->values[1] = 0.05;

        // define goal state
        base::ScopedState<base::RealVectorStateSpace> goal(start);
        goal->values[0] = 0.95;
        goal->values[1] = 0.95;
        setup.setStartAndGoalStates(start, goal);
    }
    else 
    {
        base::ScopedState<base::SE2StateSpace> start(setup.getSpaceInformation());
        start->setX(0.05);
        start->setY(0.05);
        start->setYaw(0.0);

        // define goal state
        base::ScopedState<base::SE2StateSpace> goal(start);
        goal->setX(0.95);
        goal->setY(0.95);
        goal->setYaw(0.0);

        setup.setStartAndGoalStates(start, goal);
    }

    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(1.25));
        if (spaceType == "RealVector2") // todo change the value for different problems
            obj->setCostThreshold(base::Cost(0.0));
        setup.setOptimizationObjective(obj);
    }

    setup.setup();

    double range = 0.0;
    double pen_distance = 0.0;

    pen_distance = 0.05;
    if (optimal)
        range = 0.2;
    else 
        range = 0.1;

    if (default_param)
    {
        range = 0.0;
        pen_distance = 0.0;
    }

    // Construct the optimal planner specified by our command line argument.
    // This helper function is simply a switch statement.
    ob::PlannerPtr planner = allocatePlanner(si, plannerType, range, pen_distance, svc);
    setup.setPlanner(planner);

    for (unsigned int i = 0; i < run_cycle; i++)
    {
        std::cout << "Iteration " << i + 1 << std::endl;

        setup.clear();
        setup.solve(runTime);

        // attempt to solve the problem, and print it to screen if a solution is found
        if (setup.haveSolutionPath())
        {
            std::ofstream out(boost::str(boost::format("pathsol_%i.txt") % i).c_str());
            og::PathGeometric &p = setup.getSolutionPath();
            p.interpolate();
            p.printAsMatrix(out);
            out.close();
        }

        base::PlannerData pd(si);
        setup.getPlannerData(pd);

        base::PlannerDataStorage pds;
        pds.store(pd, boost::str(boost::format("pdsol_%i.txt") % i).c_str());
    }

    return 0;
}

bool argParse(int argc, char** argv, double &runTime, PlannerType &planner, bool &optimal, std::string &env, std::string &robot, bool &default_param,
        std::string &spaceType, unsigned int &run_cycle, double &checkResolution, bool &doRandomSampling, bool &bispaceSampling)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("env,e", bpo::value<std::string>()->default_value("random_scenarios.ply"), "(Optional) Specify the polygon and circle environment, defaults to random_scenarios.ply if not given.")
        ("robot", bpo::value<std::string>()->default_value("robot_triangle1.ply"), "(Optional) Specify the robot, defaults to robot_triangle1.ply if not given.")
        ("default_param,d", bpo::value<bool>()->default_value(false), "(Optional) Specify if the planner is set the default params.")
        ("runtime,t", bpo::value<double>()->default_value(1.0), "(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.")
        ("planner,p", bpo::value<std::string>()->default_value("RRTstar"), "(Optional) Specify the optimal planner to use, defaults to RRTstar if not given. Valid options are BFMTstar, BITstar, CForest, FMTstar, InformedRRTstar, PRMstar, RRTstar, BiRRTstar, SORRTstar, RRT, RRTConnect, NaiveConnect, RRTBispace, RRTBispacestar, CellBispace, CellBispacestar, BiASE, BiASEstar, BiHSC, BiHSCstar, HSCASE, HSCASEstar, SBL, LSCAI, LSCAIstar.")
        ("spacetype,s", bpo::value<std::string>()->default_value("RealVector2"), "(Optional) Specify the planning space type, default to RealVector2 if not given. Valid options are RealVector2, SE2, Dubins, ReedsShepp.")
        ("run_cycle,r", bpo::value<unsigned int>()->default_value(1), "(Optional) Specify the run cycles.")
        ("checkResolution", bpo::value<double>()->default_value(0.01), "(Optional) Specify the collision checking resolution. Defaults to 0.01 and must be greater than 0.")
        ("doRandomSampling", bpo::value<bool>()->default_value(false), "(Optional) Specify if does random sampling test.")
        ("bispaceSampling", bpo::value<bool>()->default_value(false), "(Optional) Specify if does bispace random sampling test.");
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);

    // Check if the help flag has been given:
    if (vm.count("help") != 0u)
    {
        std::cout << desc << std::endl;
        return false;
    }

    // Get the runtime as a double
    runTime = vm["runtime"].as<double>();

    // Sanity check
    if (runTime <= 0.0)
    {
        std::cout << "Invalid runtime." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    default_param = vm["default_param"].as<bool>();
    env = vm["env"].as<std::string>();
    robot = vm["robot"].as<std::string>();
    run_cycle = vm["run_cycle"].as<unsigned int>();

    // Get the specified planner as a string
    std::string plannerStr = vm["planner"].as<std::string>();

    optimal = false;
    // Map the string to the enum
    if (boost::iequals("BFMTstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_BFMTSTAR;
    }
    else if (boost::iequals("BITstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_BITSTAR;
    }
    else if (boost::iequals("CForest", plannerStr))
    {
        optimal = true;
        planner = PLANNER_CFOREST;
    }
    else if (boost::iequals("FMTstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_FMTSTAR;
    }
    else if (boost::iequals("InformedRRTstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_INF_RRTSTAR;
    }
    else if (boost::iequals("PRMstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_PRMSTAR;
    }
    else if (boost::iequals("RRTstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_RRTSTAR;
    }
    else if (boost::iequals("BiRRTstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_BIRRTSTAR;
    }
    else if (boost::iequals("SORRTstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_SORRTSTAR;
    }
    else if (boost::iequals("RRT", plannerStr))
    {
        planner = PLANNER_RRT;
    }
    else if (boost::iequals("RRTBispace", plannerStr))
    {
        planner = PLANNER_RRTBISPACE;
    }
    else if (boost::iequals("RRTBispacestar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_RRTBISPACESTAR;
    }
    else if (boost::iequals("CellBispace", plannerStr))
    {
        planner = PLANNER_CELLBISPACE;
    }
    else if (boost::iequals("CellBispacestar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_CELLBISPACESTAR;
    }
    else if (boost::iequals("BiASE", plannerStr))
    {
        planner = PLANNER_BIASE;
    }
    else if (boost::iequals("BiASEstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_BIASESTAR;
    }
    else if (boost::iequals("BiHSC", plannerStr))
    {
        planner = PLANNER_BIHSC;
    }
    else if (boost::iequals("BiHSCstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_BIHSCSTAR;
    }
    else if (boost::iequals("HSCASE", plannerStr))
    {
        planner = PLANNER_HSCASE;
    }
    else if (boost::iequals("HSCASEstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_HSCASESTAR;
    }
    else if (boost::iequals("RRTConnect", plannerStr))
    {
        planner = PLANNER_RRTCONNECT;
    }
    else if (boost::iequals("NaiveConnect", plannerStr))
    {
        planner = PLANNER_NAIVECONNECT;
    }
    else if (boost::iequals("SBL", plannerStr))
    {
        planner = PLANNER_SBL;
    }
    else
    {
        std::cout << "Invalid planner string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    spaceType = vm["spacetype"].as<std::string>();
    if (boost::iequals("RealVector2", spaceType))
    {
        spaceType = "RealVector2";
    }
    else if (boost::iequals("SE2", spaceType))
    {
        spaceType = "SE2";
    }
    else if (boost::iequals("Dubins", spaceType))
    {
        spaceType = "Dubins";
    }
    else if (boost::iequals("ReedsShepp", spaceType))
    {
        spaceType = "ReedsShepp";
    }

    checkResolution = vm["checkResolution"].as<double>();
    doRandomSampling = vm["doRandomSampling"].as<bool>();
    bispaceSampling = vm["bispaceSampling"].as<bool>();
    
    return true;
}
