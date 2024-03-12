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

#include "KinematicCarSetup.h"

#include <omplapp/config.h>
#include <omplapp/geometry/detail/Box2dStateValidityChecker.h>

#include <ompl/base/StateSpace.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/SpaceInformation.h>

#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/math/constants/constants.hpp>
#include <iostream>

using namespace ompl;
namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

// An enum of supported optimal planners, alphabetical order
enum PlannerType
{
    PLANNER_RRT,
    PLANNER_SST,
    PLANNER_KPIECE,
};

ob::PlannerPtr allocatePlanner(oc::SpaceInformationPtr si, PlannerType plannerType)
{
    switch (plannerType)
    {
        case PLANNER_RRT:
        {
            auto planner = std::make_shared<oc::RRT>(si);
            return planner;
            break;
        }
        case PLANNER_SST:
        {
            auto planner = std::make_shared<oc::SST>(si);
            return planner;
            break;
        }
        case PLANNER_KPIECE:
        {
            auto planner = std::make_shared<oc::KPIECE1>(si);
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
bool argParse(int argc, char** argv, double &runTime, PlannerType &planner, std::string &env, std::string &robot, unsigned int &run_cycle, double &checkResolution);

const base::State* getState(const base::State* state, unsigned int /*index*/)
{
    return state;
}

int main(int argc, char* argv[])
{
    // The parsed arguments
    double runTime;
    PlannerType plannerType;
    std::string env, robot;
    unsigned int run_cycle;
    double checkResolution;
    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, runTime, plannerType, env, robot, run_cycle, checkResolution)) return -1;

    ompl::app::demos::KinematicCarSetup setup;
    setup.setVehicleLength(0.1);
    ob::StateSpacePtr space(setup.getStateSpace());
    oc::SpaceInformationPtr si(setup.getSpaceInformation());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(1);
    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    ob::RealVectorBounds cbounds(2);
    cbounds.low[0] = -0.1;
    cbounds.high[0] = 0.1;
    cbounds.low[1] = -boost::math::constants::pi<double>() * 47.0 / 180.0;
    cbounds.high[1] = boost::math::constants::pi<double>() * 47.0 / 180.0;
    setup.getControlSpace()->as<control::RealVectorControlSpace>()->setBounds(cbounds);

    // define start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(0.05);
    start->setY(0.05);
    start->setYaw(0);

    // define goal state
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(0.85);
    goal->setY(0.95);
    // goal->setYaw(boost::math::constants::pi<double>());
    goal->setYaw(0.0);

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal, 0.1);

    si->setMinMaxControlDuration(1, 20);
    si->setPropagationStepSize(0.05);

    auto svc = std::make_shared<app::Box2dStateValidityChecker>(si, space, getState);
    svc->setEnvironmentFile(std::string(OMPLAPP_RESOURCE_DIR) + "/" + env);
    svc->addRobotShape(std::string(OMPLAPP_RESOURCE_DIR) + "/" + robot);
    si->setStateValidityChecker(svc);
    // si->setStateValidityCheckingResolution(checkResolution);
    si->setup();

    // Construct the optimal planner specified by our command line argument.
    // This helper function is simply a switch statement.
    ob::PlannerPtr planner = allocatePlanner(si, plannerType);
    setup.setPlanner(planner);
    setup.setup();
    setup.print();

    for (unsigned int i = 0; i < run_cycle; i++)
    {
        std::cout << "Iteration " << i + 1 << std::endl;

        setup.clear();
        setup.solve(runTime);

        // attempt to solve the problem, and print it to screen if a solution is found
        if (setup.haveSolutionPath())
        {
            std::ofstream out(boost::str(boost::format("pathsol_%i.txt") % i).c_str());
            oc::PathControl &p = setup.getSolutionPath();
            // p.interpolate();
            p.printAsMatrix(out);
            out.close();
        }

        ob::PlannerData pd(si);
        setup.getPlannerData(pd);
        ob::PlannerDataStorage pds;
        pds.store(pd, boost::str(boost::format("pdsol_%i.txt") % i).c_str());
    }

    return 0;
}

bool argParse(int argc, char** argv, double &runTime, PlannerType &planner, std::string &env, std::string &robot, unsigned int &run_cycle, double &checkResolution)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("env,e", bpo::value<std::string>()->default_value("random_scenarios.ply"), "(Optional) Specify the polygon and circle environment, defaults to random_scenarios.ply if not given.")
        ("robot", bpo::value<std::string>()->default_value("robot_triangle1.ply"), "(Optional) Specify the robot, defaults to robot_triangle1.ply if not given.")
        ("runtime,t", bpo::value<double>()->default_value(1.0), "(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.")
        ("planner,p", bpo::value<std::string>()->default_value("RRT"), "(Optional) Specify the planner to use, defaults to RRT if not given. Valid options are RRT, SST, KPIECE.")
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

    // Get the specified planner as a string
    std::string plannerStr = vm["planner"].as<std::string>();
    // Map the string to the enum
    if (boost::iequals("RRT", plannerStr))
    {
        planner = PLANNER_RRT;
    }
    else if (boost::iequals("SST", plannerStr))
    {
        planner = PLANNER_SST;
    }
    else if (boost::iequals("KPIECE", plannerStr))
    {
        planner = PLANNER_KPIECE;
    }
    else
    {
        std::cout << "Invalid planner string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    env = vm["env"].as<std::string>();
    robot = vm["robot"].as<std::string>();
    run_cycle = vm["run_cycle"].as<unsigned int>();
    checkResolution = vm["checkResolution"].as<double>();
    return true;
}
