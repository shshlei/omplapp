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

#include "RandomScenarioRead.h"
#include <box2d_collision/b2_bvh_manager.h>
#include <omplapp/config.h>
#include <ompl/util/Console.h>
#include <iostream>
#include <fstream>
#include <string>
#include <queue>
#include <boost/program_options.hpp>

bool argParse(int argc, char** argv, std::string &env);

int main(int argc, char* argv[])
{
    std::string env;
    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, env))
        return -1;

    b2BVHManager manager;
    read(manager, std::string(OMPLAPP_RESOURCE_DIR) + "/" + env);
    b2BroadPhase * broad = manager.GetBroadPhase();
    b2DynamicTree * tree = broad->GetDynamicTree();
    tree->RebuildBottomUp();

    std::queue<int> queue;
    queue.push(tree->GetRoot());
    std::ofstream ofs("dynamic_aabb.txt", std::ios::binary | std::ios::out);
    while (!queue.empty())
    {
        int nodeId = queue.front();
        queue.pop();
        if (nodeId == b2_nullNode)
            continue;
        const b2AABB & aabb = tree->GetFatAABB(nodeId);
        ofs << aabb.min().x() << " " << aabb.min().y() << " " << aabb.max().x() << " " << aabb.max().y() << std::endl;
        const b2TreeNode * node = tree->GetNode(nodeId);
        if (!node->IsLeaf())
        {
            queue.push(node->child1);
            queue.push(node->child2);
        }
    }
    ofs.close();

    return 0;
}

bool argParse(int argc, char** argv, std::string &env)
{
    namespace bpo = boost::program_options;
    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("env,e", bpo::value<std::string>()->default_value("random_scenarios.ply"), "(Optional) Specify the polygon and circle environment, defaults to random_scenarios.ply if not given.");
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

    return true;
}
