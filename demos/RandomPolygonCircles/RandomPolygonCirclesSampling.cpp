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

#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <iostream>

using namespace ompl;
namespace ob = ompl::base;

// Parse the command-line arguments
bool argParse(int argc, char** argv, std::string &env, std::string &robot, std::string &spaceType, bool &randomSampling, bool &bispaceSampling, bool &collisionSampling);

const base::State* getState(const base::State* state, unsigned int /*index*/)
{
    return state;
}

int main(int argc, char* argv[])
{
    // The parsed arguments
    std::string env, robot;
    std::string spaceType;
    bool randomSampling, bispaceSampling, collisionSampling;
    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, env, robot, spaceType, randomSampling, bispaceSampling, collisionSampling))
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
    auto svc = std::make_shared<app::Box2dStateValidityChecker>(si, space, getState);
    svc->setEnvironmentFile(std::string(OMPLAPP_RESOURCE_DIR) + "/" + env);
    svc->addRobotShape(std::string(OMPLAPP_RESOURCE_DIR) + "/" + robot);

    si->setStateValidityChecker(svc);
    si->setup();

    if (randomSampling)
    {
        ompl::base::StateSamplerPtr sampler = si->allocStateSampler();
        ompl::base::State *state = si->allocState();

        std::ofstream ofs1("start_random_sampling_points.txt", std::ios::binary | std::ios::out);
        std::ofstream ofs2("goal_random_sampling_points.txt", std::ios::binary | std::ios::out);
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

    if (collisionSampling)
    {
        ompl::base::StateSamplerPtr sampler = si->allocStateSampler();
        ompl::base::State *state = si->allocState();

        std::ofstream ofs1("valid_sampling_points.txt", std::ios::binary | std::ios::out);
        std::ofstream ofs2("invalid_sampling_points.txt", std::ios::binary | std::ios::out);
        for (int i = 0; i < 10000; i++)
        {
            sampler->sampleUniform(state);
            if (spaceType == "RealVector2")
            {
                const auto* rvstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
                if (svc->isValid(state))
                    ofs1 << rvstate->values[0] << " " <<  rvstate->values[1] << std::endl;
                else
                    ofs2 << rvstate->values[0] << " " <<  rvstate->values[1] << std::endl;
            }
            else
            {
                const auto* rvstate = state->as<ompl::base::SE2StateSpace::StateType>();
                if (svc->isValid(state))
                    ofs1 << rvstate->getX() << " " <<  rvstate->getY() <<  " " << rvstate->getYaw() << std::endl;
                else
                    ofs2 << rvstate->getX() << " " <<  rvstate->getY() <<  " " << rvstate->getYaw() << std::endl;
            }
        }
        ofs1.close();
        ofs2.close();
        si->freeState(state);
    }

    return 0;
}

bool argParse(int argc, char** argv, std::string &env, std::string &robot, std::string &spaceType, bool &randomSampling, bool &bispaceSampling, bool &collisionSampling)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("env,e", bpo::value<std::string>()->default_value("random_scenarios.ply"), "(Optional) Specify the polygon and circle environment, defaults to random_scenarios.ply if not given.")
        ("robot", bpo::value<std::string>()->default_value("robot_triangle1.ply"), "(Optional) Specify the robot, defaults to robot_triangle1.ply if not given.")
        ("spacetype,s", bpo::value<std::string>()->default_value("RealVector2"), "(Optional) Specify the planning space type, default to RealVector2 if not given. Valid options are RealVector2, SE2, Dubins, ReedsShepp.")
        ("randomSampling", bpo::value<bool>()->default_value(false), "(Optional) Specify if does random sampling test.")
        ("bispaceSampling", bpo::value<bool>()->default_value(false), "(Optional) Specify if does bispace random sampling test.")
        ("collisionSampling", bpo::value<bool>()->default_value(false), "(Optional) Specify if does collision sampling test.");
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
    robot = vm["robot"].as<std::string>();
    randomSampling = vm["randomSampling"].as<bool>();
    bispaceSampling = vm["bispaceSampling"].as<bool>();
    collisionSampling = vm["collisionSampling"].as<bool>();

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
    
    return true;
}
