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
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

// optimal planners
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
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

ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr si)
{
    auto planner = std::make_shared<og::BiASEstar>(si);
    planner->setStopImmediately(true);
    return planner;
}

void setBounds(int parking_scenario, ob::RealVectorBounds & bounds)
{
    bounds.setLow(-20.0);
    bounds.setHigh(20.0);
    if (parking_scenario == 0)
    {
        bounds.setLow(0, -10.0);
        bounds.setHigh(0, 10.0);
        bounds.setLow(1, -10.0);
        bounds.setHigh(1, 3.0);
    }
    else if (parking_scenario == 1)
    {
        bounds.setLow(0, -12.0);
        bounds.setHigh(0, 12.0);
        bounds.setLow(1, -5.0);
        bounds.setHigh(1, 3.0);
    }
    else if (parking_scenario == 2)
    {
        bounds.setLow(0, -6.0);
        bounds.setHigh(0, 6.0);
        bounds.setLow(1, -6.0);
        bounds.setHigh(1, 6.0);
    }
    else if (parking_scenario == 3)
    {
        bounds.setLow(0, -15.0);
        bounds.setHigh(0, 15.0);
        bounds.setLow(1, 0.0);
        bounds.setHigh(1, 10.0);
    }
}

void setStartAndGoal(ompl::geometric::SimpleSetup & setup, bool parking_forward, int parking_scenario, int parking_case, double & x0, double & y0, double & theta0)
{
    x0 = 0.0;
    y0 = 0.0;
    theta0 = 0.0;
    double pi = boost::math::constants::pi<double>();
    if (parking_forward)
    {
        if (parking_scenario == 0)
        {
            if (parking_case == 0)
            {
                x0 = -6.0;
                y0 = -2.75;
                theta0 = 0.0;
            }
            else if (parking_case == 1)
            {
                x0 = 0.0;
                y0 = -2.75;
                theta0 = 0.0;
            }
            else if (parking_case == 2)
            {
                x0 = 0.0;
                y0 = -2.75;
                theta0 = pi - 0.0001;
            }
        }
        else if (parking_scenario == 1)
        {
            if (parking_case == 0)
            {
                x0 = -6.0;
                y0 = -2.5;
                theta0 = 0.0;
            }
            else if (parking_case == 1)
            {
                x0 = -8.0;
                y0 = 0.0;
                theta0 = -35.0 * pi / 180.0;
            }
            else if (parking_case == 2)
            {
                x0 = -4.0;
                y0 = -3.0;
                theta0 = 0.0;
            }
        }
        else if (parking_scenario == 2)
        {
            if (parking_case == 0)
            {
                x0 = -6.0;
                y0 = -2.0;
                theta0 = 0.0;
            }
            else if (parking_case == 1)
            {
                x0 = 1.0;
                y0 = -2.0;
                theta0 = 0.0;
            }
            else if (parking_case == 2)
            {
                x0 = -3.0;
                y0 = -2.0;
                theta0 = pi - 0.0001;
            }
        }
        else if (parking_scenario == 3)
        {
            if (parking_case == 0)
            {
                x0 = -5.0;
                y0 = 7.5;
                theta0 = 0.0;
            }
            else if (parking_case == 1)
            {
                x0 = -5.0;
                y0 = 5.0;
                theta0 = 0.0;
            }
            else if (parking_case == 2)
            {
                x0 = -5.0;
                y0 = 5.0;
                theta0 = pi - 0.0001;
            }
        }
    }
    else
    {
        if (parking_scenario == 0)
        {
            if (parking_case == 0)
            {
                x0 = -6.0;
                y0 = -2.75;
                theta0 = 0.0;
            }
            else if (parking_case == 1)
            {
                x0 = 0.0;
                y0 = -2.75;
                theta0 = 0.0;
            }
            else if (parking_case == 2)
            {
                x0 = 0.0;
                y0 = -2.75;
                theta0 = pi - 0.0001;
            }
        }
        else if (parking_scenario == 1)
        {
            if (parking_case == 0)
            {
                x0 = -6.0;
                y0 = -2.0;
                theta0 = 0.0;
            }
            else if (parking_case == 1)
            {
                x0 = -8.0;
                y0 = 0.0;
                theta0 = -pi / 6.0;
            }
            else if (parking_case == 2)
            {
                x0 = -4.0;
                y0 = -3.0;
                theta0 = -pi / 5.0;
            }
        }
        else if (parking_scenario == 2)
        {
            if (parking_case == 0)
            {
                x0 = -6.0;
                y0 = -2.0;
                theta0 = 0.0;
            }
            else if (parking_case == 1)
            {
                x0 = 1.0;
                y0 = -2.0;
                theta0 = 0.0;
            }
            else if (parking_case == 2)
            {
                x0 = -3.0;
                y0 = -2.0;
                theta0 = pi - 0.0001;
            }
        }
        else if (parking_scenario == 3)
        {
            if (parking_case == 0)
            {
                x0 = -5.0;
                y0 = 7.5;
                theta0 = 0.0;
            }
            else if (parking_case == 1)
            {
                x0 = -5.0;
                y0 = 5.0;
                theta0 = -pi / 18.0;
            }
            else if (parking_case == 2)
            {
                x0 = -1.0;
                y0 = 6.0;
                theta0 = 0.0;
            }
        }
    }

    base::ScopedState<base::SE2StateSpace> start(setup.getSpaceInformation());
    start->setX(x0);
    start->setY(y0);
    start->setYaw(theta0);

    // define goal state
    double xmiddle = 3.0;
    double ymiddle = 1.25;
    double delta = 1.4155;
    if (parking_forward) delta = -delta;
    if (parking_scenario == 0)
        xmiddle = 2.82 + delta;
    else if (parking_scenario == 1)
        xmiddle = 2.82 + delta;
    if (parking_scenario == 2)
    {
        xmiddle = 1.25;
        ymiddle = 3.0 + delta;
    }
    else if (parking_scenario == 3)
        xmiddle = 3.0 + delta;

    base::ScopedState<base::SE2StateSpace> goal(start);
    goal->setX(xmiddle);
    goal->setY(ymiddle);
    goal->setYaw(0.0);
    if (parking_scenario == 2)
      goal->setYaw(0.5 * pi);

    setup.setStartAndGoalStates(start, goal);
}

// Parse the command-line arguments
bool argParse(int argc, char** argv, bool & parking_forward, int & parking_scenario, int & parking_case, std::size_t & expected_nnodes)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("parking_forward", bpo::value<bool>()->default_value(false), "(Optional) Specify if forward parking, default to false if not given")
        ("parking_scenario", bpo::value<int>()->default_value(0), "(Optional) Specify the parking scenario, default to 0 if not given. Valid options are 0, 1, 2, 3")
        ("parking_case", bpo::value<int>()->default_value(0), "(Optional) Specify the parking case, default to 0 if not given. Valid options are 0, 1, 2")
        ("expected_nnodes", bpo::value<std::size_t>()->default_value(20), "(Optional) Specify the discrete nnodes");
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);
    // Check if the help flag has been given:
    if (vm.count("help") != 0u)
    {
        std::cout << desc << std::endl;
        return false;
    }

    parking_forward = vm["parking_forward"].as<bool>();
    parking_scenario = vm["parking_scenario"].as<int>();
    parking_case = vm["parking_case"].as<int>();
    expected_nnodes = vm["expected_nnodes"].as<std::size_t>();
    return true;
}

bool solve(bool parking_forward, int parking_scenario, int parking_case, std::size_t expected_nnodes,
    std::shared_ptr<app::Box2dStateValidityChecker> & svc, double & x0, double & y0, double & theta0,
    std::vector<double> & trajx, std::vector<double> & trajy, std::vector<double> & trajt, double & timeUsed)
{
    ob::RealVectorBounds bounds(2);
    setBounds(parking_scenario, bounds);
    auto space(std::make_shared<ob::ReedsSheppStateSpace>(4.5));
    space->setBounds(bounds);

    auto si(std::make_shared<ob::SpaceInformation>(space));
    svc = std::make_shared<app::Box2dStateValidityChecker>(si, space);
    std::string env = boost::str(boost::format("parking_scenario_%i.ply") % parking_scenario).c_str();
    svc->setEnvironmentFile(std::string(OMPLAPP_RESOURCE_DIR) + "/" + env);
    if (parking_forward)
        svc->addRobotShape(std::string(OMPLAPP_RESOURCE_DIR) + "/parking_vehicle_forward.ply");
    else
        svc->addRobotShape(std::string(OMPLAPP_RESOURCE_DIR) + "/parking_vehicle_reverse.ply");

    si->setStateValidityChecker(svc);
    si->setup();

    ompl::geometric::SimpleSetup setup(si);
    setStartAndGoal(setup, parking_forward, parking_scenario, parking_case, x0, y0, theta0);

    double cost = 0.0;
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
    obj->setCostThreshold(base::Cost(cost));
    setup.setOptimizationObjective(obj);

    // Construct the planner specified by our command line argument.
    // This helper function is simply a switch statement.
    ob::PlannerPtr planner = allocatePlanner(si);
    setup.setPlanner(planner);

    // attempt to solve the problem, and print it to screen if a solution is found
    setup.setup();
    bool res = false;
    time::point timeStart = time::now();
    auto status = setup.solve(10.0);
    timeUsed = time::seconds(time::now() - timeStart);
    if (status == ob::PlannerStatus::EXACT_SOLUTION)
    {
        res = true;
        // setup.simplifySolution();
        og::PathGeometric &path = setup.getSolutionPath();
        path.interpolate();
        if (path.getStateCount() < expected_nnodes) path.interpolate(expected_nnodes);

        std::ofstream ofs("pathsol.txt");
        path.printAsMatrix(ofs);
        ofs.close();

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

    base::PlannerData pd(si);
    setup.getPlannerData(pd);
    std::ofstream ofs("pdsol.txt");
    for (unsigned int i = 0; i < pd.numVertices(); i++)
    {
        std::vector<unsigned int> edgeList;
        if (!pd.getEdges(i, edgeList)) continue;

        const ompl::base::State * statei = pd.getVertex(i).getState();
        const ompl::base::SE2StateSpace::StateType * se2_statei = statei->as<ompl::base::SE2StateSpace::StateType>();

        ofs << se2_statei->getX() << " " << se2_statei->getY() << " "; 
        for (unsigned int j : edgeList)
        {
            const ompl::base::State * statej = pd.getVertex(j).getState();
            const ompl::base::SE2StateSpace::StateType * se2_statej = statej->as<ompl::base::SE2StateSpace::StateType>();
            ofs << se2_statej->getX() << " " << se2_statej->getY() << std::endl; 
        }
    }
    ofs.close();

    return res;
}
