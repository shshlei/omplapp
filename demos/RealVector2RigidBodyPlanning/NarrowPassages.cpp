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
#include <omplapp/apps/RealVector2RigidBodyPlanning.h>

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>

// feasible planners
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/ase/BiASE.h>

// optimal planners
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/ase/BiASEstar.h>

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
    PLANNER_FMTSTAR,
    PLANNER_BFMTSTAR,
    PLANNER_BITSTAR,
    PLANNER_INF_RRTSTAR,
    PLANNER_PRMSTAR,
    PLANNER_RRTSTAR,
    PLANNER_BIASE,
    PLANNER_BIASESTAR,
    PLANNER_RRT,
    PLANNER_RRTCONNECT,
    PLANNER_LAZYRRT,
};

bool isStateValidEasy(const base::SpaceInformationPtr &si, const ob::State *state)
{
    if (!si->satisfiesBounds(state))
        return false;
    const auto* rvstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
    double x = rvstate->values[0], y = rvstate->values[1];
    if (x < 0.4)
        return true;
    if (x > 0.6)
        return true;
    if (y > 0.49 && y < 0.51)
        return true;
    return false;
}

bool isStateValidHard(const base::SpaceInformationPtr &si, const ob::State *state)
{
    if (!si->satisfiesBounds(state))
        return false;
    const auto* rvstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
    double x = rvstate->values[0], y = rvstate->values[1];
    if (x < 0.15)
        return true;
    if (x > 0.85)
        return true;
    if (y < 0.15)
        return true;
    if (y > 0.85)
        return true;
    if (x > 0.25 && x < 0.75)
    {
        if (y > 0.61 && y < 0.75)
            return true;
        if (y > 0.25 && y < 0.39)
            return true;
        if (y > 0.49 && y < 0.51)
            return true;
        if (x > 0.65 && y > 0.25 && y < 0.75)
            return true;
    }
    if (y > 0.49 && y < 0.51 && x < 0.75)
        return true;
    return false;
}

ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr si, PlannerType plannerType)
{
    switch (plannerType)
    {
		case PLANNER_BFMTSTAR:
        {
            auto planner = std::make_shared<og::BFMT>(si);
            return planner;
            break;
        }
        case PLANNER_BITSTAR:
        {
            auto planner = std::make_shared<og::BITstar>(si);
            return planner;
            break;
        }
        case PLANNER_FMTSTAR:
        {
            auto planner = std::make_shared<og::FMT>(si);
            return planner;
            break;
        }
        case PLANNER_INF_RRTSTAR:
        {
            auto planner = std::make_shared<og::InformedRRTstar>(si);
            return planner;
            break;
        }
        case PLANNER_PRMSTAR:
        {
            auto planner = std::make_shared<og::PRMstar>(si);
            return planner;
            break;
        }
        case PLANNER_RRTSTAR:
        {
            auto planner = std::make_shared<og::RRTstar>(si);
            return planner;
            break;
        }
        case PLANNER_RRT:
        {
            auto planner = std::make_shared<og::RRT>(si);
            return planner;
            break;
        }
        case PLANNER_BIASE:
        {
            auto planner = std::make_shared<og::BiASE>(si);
            planner->setAddIntermediateState(false);
            planner->setUseBispace(false);
            planner->setBackRewire(false);
            return planner;
            break;
        }
        case PLANNER_BIASESTAR:
        {
            auto planner = std::make_shared<og::BiASEstar>(si);
            return planner;
            break;
        }
        case PLANNER_RRTCONNECT:
        {
            auto planner = std::make_shared<og::RRTConnect>(si);
            return planner;
            break;
        }
        case PLANNER_LAZYRRT:
        {
            auto planner = std::make_shared<og::LazyRRT>(si);
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

void envNarrowPassages(app::RealVector2RigidBodyPlanning &setup, bool optimal, bool hard)
{
    setup.getStateSpace()->as<ob::RealVectorStateSpace>()->setBounds(0.0, 1.0);

    // define start state
    base::ScopedState<base::RealVectorStateSpace> start(setup.getSpaceInformation());
    start->values[0] = 0.05;
    start->values[1] = 0.05;

    // define goal state
    base::ScopedState<base::RealVectorStateSpace> goal(start);
    goal->values[0] = 0.95;
    goal->values[1] = 0.95;
    if (hard)
    {
        start->values[0] = 0.30;
        start->values[1] = 0.30;
        goal->values[0] = 0.05;
        goal->values[1] = 0.95;
    }
    setup.setStartAndGoalStates(start, goal);
    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(std::sqrt(0.0)));
        setup.setOptimizationObjective(obj);
    }
    setup.setup();
}

// Parse the command-line arguments
bool argParse(int argc, char** argv, double &runTime, PlannerType &planner, bool &optimal, bool &hard);

int main(int argc, char* argv[])
{
    // The parsed arguments
    bool optimal;
    bool hard;
    double runTime;
    PlannerType plannerType;

    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, runTime, plannerType, optimal, hard))
        return -1;

    ompl::app::RealVector2RigidBodyPlanning setup;
    envNarrowPassages(setup, optimal, hard);

    auto si = setup.getSpaceInformation();
    auto isStateValid = hard ? isStateValidHard : isStateValidEasy;
    setup.setStateValidityChecker([isStateValid, si](const ob::State *state)
        {
            return isStateValid(si, state);
        });
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.001);

    // Construct the optimal planner specified by our command line argument.
    // This helper function is simply a switch statement.
    ob::PlannerPtr planner = allocatePlanner(si, plannerType);
    setup.setPlanner(planner);

    for (unsigned int i = 0; i < 10; i++)
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

bool argParse(int argc, char** argv, double &runTime, PlannerType &planner, bool &optimal, bool &hard)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("runtime,t", bpo::value<double>()->default_value(1.0), "(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.")
        ("planner,p", bpo::value<std::string>()->default_value("RRTstar"), "(Optional) Specify the optimal planner to use, defaults to RRTstar if not given. Valid options are BFMTstar, BITstar, FMTstar, InformedRRTstar, PRMstar, RRTstar, RRT, RRTConnect, LazyRRT, BiASE, BiASEstar.")
        ("hard", bpo::value<bool>()->default_value(false), "(Optional) Specify if it is the hard motion planning problem.");
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

    hard = vm["hard"].as<bool>();

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
    else if (boost::iequals("RRT", plannerStr))
    {
        planner = PLANNER_RRT;
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
    else if (boost::iequals("RRTConnect", plannerStr))
    {
        planner = PLANNER_RRTCONNECT;
    }
    else if (boost::iequals("LazyRRT", plannerStr))
    {
        planner = PLANNER_LAZYRRT;
    }
    else
    {
        std::cout << "Invalid planner string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    return true;
}
