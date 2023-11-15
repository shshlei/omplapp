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
#include "SimpleCertificateStateValidityChecker.h"

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// feasible planners
#include "CSCRRT.h"
#include "CSCRRTConnect.h"

// optimal planners
#include "CSCRRTstar.h"
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <iostream>
#include <fstream>

using namespace ompl;
namespace ob = ompl::base;
namespace og = ompl::geometric;

// An enum of supported optimal planners, alphabetical order
enum PlannerType
{
    PLANNER_RRT,
    PLANNER_RRTSTAR,
    PLANNER_RRTCONNECT,
};

ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr si, PlannerType plannerType, double range)
{
    switch (plannerType)
    {
        case PLANNER_RRT:
        {
            auto planner = std::make_shared<og::CSCRRT>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_RRTSTAR:
        {
            auto planner = std::make_shared<og::CSCRRTstar>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("collision_range")))
                params.setParam(std::string("collision_range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_RRTCONNECT:
        {
            auto planner = std::make_shared<og::CSCRRTConnect>(si);
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
bool argParse(int argc, char** argv, double &runTime, PlannerType &planner, bool &optimal, std::string &env, bool &default_param, unsigned int &run_cycle, bool &doDistanceTest, double &checkResolution, double &optimalT);

int main(int argc, char* argv[])
{
    // The parsed arguments
    bool optimal;
    bool default_param;
    double runTime;
    double checkResolution;
    double optimalT;
    PlannerType plannerType;
    std::string env;
    unsigned int run_cycle;
    bool doDistanceTest;
    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, runTime, plannerType, optimal, env, default_param, run_cycle, doDistanceTest, checkResolution, optimalT))
        return -1;

    auto space(std::make_shared<ob::RealVectorStateSpace>(2));
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0.0);
    bounds.setHigh(1.0);
    space->setBounds(bounds);

    auto si(std::make_shared<ob::SpaceInformation>(space));
    auto svc = std::make_shared<app::SimpleCertificateStateValidityChecker>(si, 0.05);
    svc->setEnvironmentFile(std::string(OMPLAPP_RESOURCE_DIR) + "/" + env);

    auto circle(std::make_shared<app::geometries::Circle>(0.0));
    svc->setRobotShape(circle);

    si->setStateValidityChecker(svc);
    si->setStateValidityCheckingResolution(checkResolution);
    si->setup();

    if (doDistanceTest) // check if the distance function is correct
    {
        ompl::base::StateSamplerPtr sampler = si->allocStateSampler();
        ompl::base::State *state = si->allocState();

        std::ofstream ofs1("simple_safety_certificate.txt", std::ios::binary | std::ios::out);
        std::ofstream ofs2("simple_collision_certificate.txt", std::ios::binary | std::ios::out);
        for (int i = 0; i < 100; i++)
        {
            sampler->sampleUniform(state);

            double dist = 0.0;
            const auto* rvstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
            if (svc->isValid(state, dist))
                ofs1 << rvstate->values[0] << " " <<  rvstate->values[1] << " " << dist << std::endl;
            else
                ofs2 << rvstate->values[0] << " " <<  rvstate->values[1] << " " << -dist << std::endl;
        }
        ofs1.close();
        ofs2.close();
        si->freeState(state);
    }

    ompl::geometric::SimpleSetup setup(si);

    // define start state
    base::ScopedState<base::RealVectorStateSpace> start(setup.getSpaceInformation());
    start->values[0] = 0.05;
    start->values[1] = 0.05;

    // define goal state
    base::ScopedState<base::RealVectorStateSpace> goal(start);
    goal->values[0] = 0.95;
    goal->values[1] = 0.95;
    setup.setStartAndGoalStates(start, goal);

    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(optimalT));
        setup.setOptimizationObjective(obj);
    }

    setup.setup();

    double range = 0.1;
    if (optimal)
        range = 0.2;
    if (default_param)
        range = 0.0;

    // Construct the optimal planner specified by our command line argument.
    // This helper function is simply a switch statement.
    ob::PlannerPtr planner = allocatePlanner(si, plannerType, range);
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
            //p.interpolate();
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

bool argParse(int argc, char** argv, double &runTime, PlannerType &planner, bool &optimal, std::string &env, bool &default_param, unsigned int &run_cycle, bool &doDistanceTest, double &checkResolution, double &optimalT)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("env,e", bpo::value<std::string>()->default_value("random_scenarios.ply"), "(Optional) Specify the polygon and circle environment, defaults to random_scenarios.ply if not given.")
        ("default_param,d", bpo::value<bool>()->default_value(false), "(Optional) Specify if the planner is set the default params.")
        ("runtime,t", bpo::value<double>()->default_value(1.0), "(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.")
        ("planner,p", bpo::value<std::string>()->default_value("RRTstar"), "(Optional) Specify the optimal planner to use, defaults to RRTstar if not given. Valid options are RRT, RRTstar, RRTConnect.")
        ("run_cycle,r", bpo::value<unsigned int>()->default_value(1), "(Optional) Specify the run cycles.")
        ("doDistanceTest", bpo::value<bool>()->default_value(false), "(Optional) Specify if do the distance test.")
        ("checkResolution", bpo::value<double>()->default_value(0.01), "(Optional) Specify the collision checking resolution. Defaults to 0.01 and must be greater than 0.")
        ("optimalT", bpo::value<double>()->default_value(1.30), "(Optional) Specify the optimal path length threshold. Defaults to 1.30 and must be greater than 0.");
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
    run_cycle = vm["run_cycle"].as<unsigned int>();

    // Get the specified planner as a string
    std::string plannerStr = vm["planner"].as<std::string>();

    optimal = false;
    // Map the string to the enum
    if (boost::iequals("RRTstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_RRTSTAR;
    }
    else if (boost::iequals("RRT", plannerStr))
    {
        planner = PLANNER_RRT;
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

    doDistanceTest = vm["doDistanceTest"].as<bool>();
    checkResolution = vm["checkResolution"].as<double>();
    optimalT = vm["optimalT"].as<double>();
    return true;
}
