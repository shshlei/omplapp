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
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

// feasible planners
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/ase/BiASE.h>
#include <ompl/geometric/planners/bispace/RRTBispace.h>
#include <ompl/geometric/planners/bispace/CellBispace.h>

// optimal planners
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/ase/BiASEstar.h>
#include <ompl/geometric/planners/bispace/RRTBispacestar.h>
#include <ompl/geometric/planners/bispace/CellBispacestar.h>

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
    PLANNER_PRMSTAR,
    PLANNER_RRTSTAR,
    PLANNER_BIASE,
    PLANNER_BIASESTAR,
    PLANNER_RRT,
    PLANNER_RRTBISPACE,
    PLANNER_RRTBISPACESTAR,
    PLANNER_CELLBISPACE,
    PLANNER_CELLBISPACESTAR,
    PLANNER_RRTCONNECT,
};

ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr si, PlannerType plannerType, double range = 0.0)
{
    switch (plannerType)
    {
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
            planner->setAddIntermediateState(false);
            //planner->setUseBispace(false);
            //planner->setBackRewire(false);
            return planner;
            break;
        }
        case PLANNER_RRTCONNECT:
        {
            auto planner = std::make_shared<og::RRTConnect>(si);
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
bool argParse(int argc, char** argv, double &runTime, PlannerType &planner, bool &optimal, std::string &env, std::string &robot,
        std::string &spaceType, unsigned int &run_cycle, double &checkResolution);

const base::State* getState(const base::State* state, unsigned int /*index*/)
{
    return state;
}

int main(int argc, char* argv[])
{
    // The parsed arguments
    bool optimal;
    double runTime;
    double checkResolution;
    PlannerType plannerType;
    std::string env, robot;
    std::string spaceType;
    unsigned int run_cycle;
    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, runTime, plannerType, optimal, env, robot, spaceType, run_cycle, checkResolution)) return -1;

    ob::StateSpacePtr space;
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0.0);
    bounds.setHigh(1.0);
    if (spaceType == "Dubins")
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
    auto svc = std::make_shared<app::Box2dStateValidityChecker>(si, space, getState);
    svc->setEnvironmentFile(std::string(OMPLAPP_RESOURCE_DIR) + "/" + env);
    svc->addRobotShape(std::string(OMPLAPP_RESOURCE_DIR) + "/" + robot);
    si->setStateValidityChecker(svc);
    si->setStateValidityCheckingResolution(checkResolution);
    si->setup();

    ompl::geometric::SimpleSetup setup(si);
    base::ScopedState<base::SE2StateSpace> start(setup.getSpaceInformation());
    start->setX(0.05);
    start->setY(0.05);
    start->setYaw(0.0);

    // define goal state
    base::ScopedState<base::SE2StateSpace> goal(start);
    goal->setX(0.85);
    goal->setY(0.95);
    goal->setYaw(0.0);
    setup.setStartAndGoalStates(start, goal);

    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(0.0));
        setup.setOptimizationObjective(obj);
    }

    // Construct the optimal planner specified by our command line argument.
    // This helper function is simply a switch statement.
    ob::PlannerPtr planner = allocatePlanner(si, plannerType);
    setup.setPlanner(planner);
    setup.setup();

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
            p.interpolate(30);
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

bool argParse(int argc, char** argv, double &runTime, PlannerType &planner, bool &optimal, std::string &env, std::string &robot,
        std::string &spaceType, unsigned int &run_cycle, double &checkResolution)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("env,e", bpo::value<std::string>()->default_value("random_scenarios.ply"), "(Optional) Specify the polygon and circle environment, defaults to random_scenarios.ply if not given.")
        ("robot", bpo::value<std::string>()->default_value("robot_triangle1.ply"), "(Optional) Specify the robot, defaults to robot_triangle1.ply if not given.")
        ("runtime,t", bpo::value<double>()->default_value(1.0), "(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.")
        ("planner,p", bpo::value<std::string>()->default_value("RRTstar"), "(Optional) Specify the optimal planner to use, defaults to RRTstar if not given. Valid options are PRMstar, RRTstar, RRT, RRTConnect, RRTBispace, RRTBispacestar, CellBispace, CellBispacestar, BiASE, BiASEstar.")
        ("spacetype,s", bpo::value<std::string>()->default_value("ReedsShepp"), "(Optional) Specify the planning space type, default to ReedsShepp if not given. Valid options are Dubins, ReedsShepp.")
        ("run_cycle,r", bpo::value<unsigned int>()->default_value(1), "(Optional) Specify the run cycles.")
        ("checkResolution", bpo::value<double>()->default_value(0.01), "(Optional) Specify the collision checking resolution. Defaults to 0.01 and must be greater than 0.");
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

    env = vm["env"].as<std::string>();
    robot = vm["robot"].as<std::string>();
    run_cycle = vm["run_cycle"].as<unsigned int>();
    checkResolution = vm["checkResolution"].as<double>();

    spaceType = vm["spacetype"].as<std::string>();
    if (boost::iequals("Dubins", spaceType))
    {
        spaceType = "Dubins";
    }
    else
    {
        spaceType = "ReedsShepp";
    }

    // Get the specified planner as a string
    std::string plannerStr = vm["planner"].as<std::string>();
    optimal = false;
    // Map the string to the enum
    if (boost::iequals("PRMstar", plannerStr))
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
    else if (boost::iequals("RRTConnect", plannerStr))
    {
        planner = PLANNER_RRTCONNECT;
    }
    else
    {
        std::cout << "Invalid planner string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    return true;
}
