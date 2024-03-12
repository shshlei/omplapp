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

#include <ompl/util/RandomNumbers.h>
#include <box2d_collision/b2_bvh_manager.h>
#include <boost/math/constants/constants.hpp>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <vector>
#include <iostream>
#include <fstream>

bool argParse(int argc, char** argv, std::size_t &objects, std::size_t &points, bool &circle, bool &ellipse, bool &capsule, bool &rectangle);

int main(int argc, char* argv[])
{
    std::size_t objects = 1000;
    std::size_t points = 1000;

    bool circle = false;
    bool ellipse = false;
    bool capsule = false;
    bool rectangle = false;

    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, objects, points, circle, ellipse, capsule, rectangle))
        return -1;

    b2AABB aabb;
    aabb.min().setConstant(b2Scalar(0.0));
    aabb.max().setConstant(b2Scalar(1.0));

    ompl::RNG rng;
    
    std::ofstream ofs;
    b2CircleShape *circleshape = nullptr;
    b2EllipseShape *ellipseshape = nullptr;
    b2CapsuleShape *capsuleshape = nullptr;
    b2RectangleShape *rectshape = nullptr;
    if (circle)
    {
        ofs.open("data/circle_point_collision_datas.txt", std::ios::binary | std::ios::out);
        circleshape = new b2CircleShape();
    }
    else if (ellipse)
    {
        ofs.open("data/ellipse_point_collision_datas.txt", std::ios::binary | std::ios::out);
        ellipseshape = new b2EllipseShape();
    }
    else if (capsule)
    {
        ofs.open("data/capsule_point_collision_datas.txt", std::ios::binary | std::ios::out);
        capsuleshape = new b2CapsuleShape();
    }
    else if (rectangle)
    {
        ofs.open("data/rectangle_point_collision_datas.txt", std::ios::binary | std::ios::out);
        rectshape = new b2RectangleShape();
    }

    int valid = 0, invalid = 0;
    int diff = std::floor(0.05 * objects * points);
    for (std::size_t i = 0; i < objects;)
    {
        double a = rng.uniformReal(0.05, 0.45);
        double b = rng.uniformReal(0.05, 0.45);
        if (circle)
            circleshape->SetRadius(b2Scalar(a));
        else if (ellipse)
            ellipseshape->SetHalfSides(b2Scalar(a), b2Scalar(b));
        else if (capsule)
        {
            capsuleshape->SetRadius(b2Scalar(a));
            capsuleshape->SetHeight(b2Scalar(b));
        }
        else if (rectangle)
            rectshape->Set(b2Scalar(a), b2Scalar(b));
        double xc = rng.uniform01();
        double yc = rng.uniform01();
        double yaw = rng.uniformReal(-boost::math::constants::pi<double>(), boost::math::constants::pi<double>());
        if (circle)
            yaw = 0.0;
        b2Transform xf = b2Transform::Identity();
        xf.translation() = b2Vec2(b2Scalar(xc), b2Scalar(yc));
        xf.linear() = b2Rot(b2Scalar(yaw)).toRotationMatrix();
        b2AABB saabb;
        if (circle)
            circleshape->ComputeAABB(&saabb, xf);
        else if (ellipse)
            ellipseshape->ComputeAABB(&saabb, xf);
        else if (capsule)
            capsuleshape->ComputeAABB(&saabb, xf);
        else if (rectangle)
            rectshape->ComputeAABB(&saabb, xf);
        if (aabb.contains(saabb))
        {
            for (std::size_t j = 0; j < points;)
            {
                xc = rng.uniform01();
                yc = rng.uniform01();
                bool collision = false;
                if (circle)
                    collision = circleshape->TestPoint(xf, b2Vec2(xc, yc));
                else if (ellipse) 
                    collision = ellipseshape->TestPoint(xf, b2Vec2(xc, yc));
                else if (capsule)
                    collision = capsuleshape->TestPoint(xf, b2Vec2(xc, yc));
                else if (rectangle)
                    collision = rectshape->TestPoint(xf, b2Vec2(xc, yc));
                if (valid - invalid > diff && !collision)
                    continue;
                if (invalid - valid > diff && collision)
                    continue;
                j++;
                if (collision)
                    invalid++;
                else 
                    valid++;
                ofs << static_cast<double>(xf.translation().x()) << " " << static_cast<double>(xf.translation().y()) << " ";
                if (!circle)
                {
                    double angle = static_cast<double>(b2Rot(xf.linear()).angle());
                    ofs << std::cos(angle) << " " << std::sin(angle) << " " << a << " " << b << " ";
                }
                else
                    ofs << a << " ";
                ofs << xc << " " << yc << " " << collision << std::endl; 
            }
            i++;
        }
    }
    ofs.close();
    if (circle)
        delete circleshape;
    else if (ellipse)
        delete ellipseshape;
    else if (capsule)
        delete capsuleshape;
    else if (rectangle)
        delete rectshape;
    std::cout << "Valid: " << valid << " Invalid: " << invalid << " Valid-Ratio: " << float(valid) / float(valid + invalid) << std::endl;
    return 0;
}

bool argParse(int argc, char** argv, std::size_t &objects, std::size_t &points, bool &circle, bool &ellipse, bool &capsule, bool &rectangle)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("objects", bpo::value<std::size_t>()->default_value(1000), "Objects number")
        ("points", bpo::value<std::size_t>()->default_value(1000), "Points number")
        ("circle", bpo::value<bool>()->default_value(false), "Generate circle datas")
        ("ellipse", bpo::value<bool>()->default_value(true), "Generate ellipse datas")
        ("capsule", bpo::value<bool>()->default_value(false), "Generate capsule datas")
        ("rectangle", bpo::value<bool>()->default_value(false), "Generate rectangle datas");
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);

    // Check if the help flag has been given:
    if (vm.count("help") != 0u)
    {
        std::cout << desc << std::endl;
        return false;
    }

    objects = vm["objects"].as<std::size_t>();
    points = vm["points"].as<std::size_t>();
    circle = vm["circle"].as<bool>();
    ellipse = vm["ellipse"].as<bool>();
    capsule = vm["capsule"].as<bool>();
    rectangle = vm["rectangle"].as<bool>();

    return true;
}
