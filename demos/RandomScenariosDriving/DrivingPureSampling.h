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
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

// feasible planners
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/ase/BiASE.h>

// optimal planners
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/ase/BiASEstar.h>

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>

#include <ompl/util/Time.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <iostream>

using namespace ompl;
namespace ob = ompl::base;
namespace og = ompl::geometric;

// An enum of supported planners
enum PlannerType
{
    PLANNER_RRT,
    PLANNER_RRTCONNECT,
    PLANNER_BIASE,
    PLANNER_RRTSTAR,
    PLANNER_INF_RRTSTAR,
    PLANNER_BITSTAR,
    PLANNER_BIASESTAR,
};

ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr si, PlannerType plannerType)
{
    switch (plannerType)
    {
        case PLANNER_RRT:
        {
            auto planner = std::make_shared<og::RRT>(si);
            return planner;
            break;
        }
        case PLANNER_RRTCONNECT:
        {
            auto planner = std::make_shared<og::RRTConnect>(si);
            return planner;
            break;
        }
        case PLANNER_BIASE:
        {
            auto planner = std::make_shared<og::BiASE>(si);
            return planner;
            break;
        }
        case PLANNER_RRTSTAR:
        {
            auto planner = std::make_shared<og::RRTstar>(si);
            return planner;
            break;
        }
        case PLANNER_INF_RRTSTAR:
        {
            auto planner = std::make_shared<og::InformedRRTstar>(si);
            return planner;
            break;
        }
        case PLANNER_BITSTAR:
        {
            auto planner = std::make_shared<og::BITstar>(si);
            return planner;
            break;
        }
        case PLANNER_BIASESTAR:
        {
            auto planner = std::make_shared<og::BiASEstar>(si);
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

void setBounds(ob::RealVectorBounds & bounds)
{
    bounds.setLow(0.0);
    bounds.setHigh(1.0);
}

void setStartAndGoal(ompl::geometric::SimpleSetup & setup, double & x0, double & y0, double & theta0, double & xf, double & yf, double & thetaf)
{
    x0 = 0.05;
    y0 = 0.05;
    theta0 = 0.0;
    base::ScopedState<base::SE2StateSpace> start(setup.getSpaceInformation());
    start->setX(x0);
    start->setY(y0);
    start->setYaw(theta0);

    xf = 0.95;
    yf = 0.95;
    thetaf = 0.0;
    base::ScopedState<base::SE2StateSpace> goal(start);
    goal->setX(xf);
    goal->setY(yf);
    goal->setYaw(thetaf);

    setup.setStartAndGoalStates(start, goal);
}

void setCost(double & cost)
{
    cost = 1.6;
}

bool argParse(int argc, char** argv, std::string &env, std::string &spaceType, double &checkResolution, double &runTime, PlannerType &planner, bool & do_simplification, std::size_t & expected_nnodes, int & save_num)
{
    namespace bpo = boost::program_options;
    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("env,e", bpo::value<std::string>()->default_value("driving_scenario_0.ply"), "(Optional) Specify the polygon and circle environment, defaults to driving_scenario_0.ply if not given.")
        ("spacetype,s", bpo::value<std::string>()->default_value("ReedsShepp"), "(Optional) Specify the planning space type, default to RealVector2 if not given. Valid options are RealVector2, SE2, Dubins, ReedsShepp.")
        ("checkResolution", bpo::value<double>()->default_value(0.01), "(Optional) Specify the collision checking resolution. Defaults to 0.01 and must be greater than 0.")
        ("runtime,t", bpo::value<double>()->default_value(1.0), "(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.")
        ("planner,p", bpo::value<std::string>()->default_value("RRT"), "(Optional) Specify the planner to use, defaults to RRT if not given. Valid options are RRT, RRTConnect, BiASE, RRTstar, InformedRRTstar, BITstar, BiASEstar.")
        ("do_simplification", bpo::value<bool>()->default_value(false), "(Optional) Specify if do path simplification")
        ("expected_nnodes", bpo::value<std::size_t>()->default_value(100), "(Optional) Specify the discrete nnodes")
        ("save_num", bpo::value<int>()->default_value(0), "(Optional) Specify the save num");
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);
    // Check if the help flag has been given:
    if (vm.count("help") != 0u)
    {
        std::cout << desc << std::endl;
        return false;
    }

    env = vm["env"].as<std::string>();
    spaceType = vm["spacetype"].as<std::string>();
    if (boost::iequals("SE2", spaceType))
    {
        spaceType = "SE2";
    }
    else if (boost::iequals("Dubins", spaceType))
    {
        spaceType = "Dubins";
    }
    else
    {
        spaceType = "ReedsShepp";
    }
    checkResolution = vm["checkResolution"].as<double>();

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
    else if (boost::iequals("RRTConnect", plannerStr))
    {
        planner = PLANNER_RRTCONNECT;
    }
    else if (boost::iequals("BiASE", plannerStr))
    {
        planner = PLANNER_BIASE;
    }
    else if (boost::iequals("RRTstar", plannerStr))
    {
        planner = PLANNER_RRTSTAR;
    }
    else if (boost::iequals("InformedRRTstar", plannerStr))
    {
        planner = PLANNER_INF_RRTSTAR;
    }
    else if (boost::iequals("BITstar", plannerStr))
    {
        planner = PLANNER_BITSTAR;
    }
    else if (boost::iequals("BiASEstar", plannerStr))
    {
        planner = PLANNER_BIASESTAR;
    }
    else
    {
        std::cout << "Invalid planner string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    do_simplification = vm["do_simplification"].as<bool>();
    expected_nnodes = vm["expected_nnodes"].as<std::size_t>();
    save_num = vm["save_num"].as<int>();
    
    return true;
}

bool solve(int argc, char* argv[], std::shared_ptr<app::Box2dStateValidityChecker> & svc, double & x0, double & y0, double & theta0, double & xf, double & yf, double & thetaf,
    std::vector<double> & trajx, std::vector<double> & trajy, std::vector<double> & trajt, double & timeUsed, int & save_num)
{
    // The parsed arguments
    std::string env;
    std::string spaceType;
    double checkResolution;
    double runTime;
    PlannerType plannerType;
    bool do_simplification;
    std::size_t expected_nnodes;
    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, env, spaceType, checkResolution, runTime, plannerType, do_simplification, expected_nnodes, save_num)) return false;

    ob::StateSpacePtr space;
    ob::RealVectorBounds bounds(2);
    setBounds(bounds);
    if (spaceType == "SE2")
    {
        auto space1(std::make_shared<ob::SE2StateSpace>());
        space1->setBounds(bounds);
        space = space1;
    }
    else if (spaceType == "Dubins")
    {
        auto space1(std::make_shared<ob::DubinsStateSpace>(0.045, false));
        space1->setBounds(bounds);
        space = space1;
    }
    else if (spaceType == "ReedsShepp")
    {
        auto space1(std::make_shared<ob::ReedsSheppStateSpace>(0.045));
        space1->setBounds(bounds);
        space = space1;
    }

    auto si(std::make_shared<ob::SpaceInformation>(space));
    svc = std::make_shared<app::Box2dStateValidityChecker>(si, space);
    svc->setEnvironmentFile(std::string(OMPLAPP_RESOURCE_DIR) + "/" + env);
    svc->addRobotShape(std::string(OMPLAPP_RESOURCE_DIR) + "/driving_vehicle.ply");

    si->setStateValidityChecker(svc);
    si->setStateValidityCheckingResolution(checkResolution);
    si->setup();

    ompl::geometric::SimpleSetup setup(si);
    setStartAndGoal(setup, x0, y0, theta0, xf, yf, thetaf);

    if (plannerType >= PLANNER_RRTSTAR)
    {
        double cost = 0.0;
        setCost(cost);
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(cost));
        setup.setOptimizationObjective(obj);
    }

    // Construct the planner specified by our command line argument.
    // This helper function is simply a switch statement.
    ob::PlannerPtr planner = allocatePlanner(si, plannerType);
    setup.setPlanner(planner);

    // attempt to solve the problem, and print it to screen if a solution is found
    setup.setup();
    bool res = false;
    time::point timeStart = time::now();
    auto status = setup.solve(runTime);
    timeUsed = time::seconds(time::now() - timeStart);
    if (status == ob::PlannerStatus::EXACT_SOLUTION)
    {
        res = true;
        if (do_simplification) setup.simplifySolution();
        og::PathGeometric &path = setup.getSolutionPath();
        path.interpolate();
        if (path.getStateCount() < expected_nnodes) path.interpolate(expected_nnodes);
        /*
        std::ofstream ofs("pathsol.txt");
        path.printAsMatrix(ofs);
        ofs.close();
        */

        time::point timeStart = time::now();
        // extract expected_nnodes states 
        trajx.clear();
        trajy.clear();
        trajt.clear();
        std::size_t nmod = (expected_nnodes != 0 ? path.getStateCount() / expected_nnodes : 1);
        for (std::size_t i = 0;; i+=nmod)
        {
            if (i >= path.getStateCount())
            {
                if (i < path.getStateCount() + nmod - 1) i = path.getStateCount() - 1;
                else break;
            }
            const auto* se2state = path.getState(i)->as<ob::SE2StateSpace::StateType>();
            trajx.push_back(se2state->getX());
            trajy.push_back(se2state->getY());
            trajt.push_back(se2state->getYaw());
        }
        timeUsed += time::seconds(time::now() - timeStart);
    }

    /*
    base::PlannerData pd(si);
    setup.getPlannerData(pd);
    base::PlannerDataStorage pds;
    pds.store(pd, "pdsol.txt");
    */

    // svc->clearRobotShape();

    return res;
}
