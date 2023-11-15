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

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <set>
#include <iostream>
#include <fstream>

class MQueryCallback : public b2NaiveCallback
{
public:
    MQueryCallback(const b2Body * body) : b2NaiveCallback()
    {
        body_ = body;
    }

    ~MQueryCallback() override = default;

    bool ReportCollision(b2Fixture* fixture) override
    {
        if (fixture->GetBody() != body_)
        {
            const b2Fixture* flist = body_->GetFixtureList();
            if (b2CollideShapes(flist->GetShape(), flist->GetGlobalTransform(), fixture->GetShape(), fixture->GetGlobalTransform()))
                collisionBodies_.insert(fixture->GetBody()->GetName());
        }
        return false; // Never stop early
    }

    std::set<std::string> getCollisionBodies() const 
    {
        return collisionBodies_;
    }

private:
    const b2Body* body_;
    std::set<std::string> collisionBodies_; 
};

bool argParse(int argc, char** argv, double &rmin, double &deltar, int &expect_obstacles,
              double &circleratio, double &ellipseratio, double &capsuleratio, double &rectangeleratio, double &polygonratio,
              bool &disjoint, bool &decorate);

int main(int argc, char* argv[])
{
    double width = 1.0;
    double rmin = 0.01 * width, deltar = 0.01 * width;
    double bmin = 0.005 * width, bmax = width - bmin;
    double circleratio = 0.5;
    double ellipseratio = 0.5;
    double capsuleratio = 0.5;
    double rectangeleratio = 0.5;
    double polygonratio = 0.5;
    int nobstacle = 1000;
    bool disjoint = true;
    bool decorate = true;

    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, rmin, deltar, nobstacle, circleratio, ellipseratio, capsuleratio, rectangeleratio, polygonratio, disjoint, decorate))
        return -1;

    double total = circleratio + ellipseratio + capsuleratio + rectangeleratio + polygonratio;
    circleratio = circleratio / total;
    ellipseratio = circleratio + ellipseratio / total;
    capsuleratio = ellipseratio + capsuleratio / total;
    rectangeleratio = capsuleratio + rectangeleratio / total;

    b2AABB aabb;
    aabb.min().setConstant(b2Scalar(bmin));
    aabb.max().setConstant(b2Scalar(bmax));

    ompl::RNG rng;

    b2BVHManager manager;

    for (int i = 0; i < nobstacle;)
    {
        b2Shape *shape = nullptr;
        double xc = width * rng.uniform01();
        double yc = width * rng.uniform01();
        double yaw = rng.uniformReal(-boost::math::constants::pi<double>(), boost::math::constants::pi<double>());

        bool polygon = false;
        if (rng.uniform01() < circleratio) // circle shape
        {
            yaw = 0.0;
            double  r = 0.5 * rmin + 0.5 * deltar * rng.uniform01();
            b2CircleShape *cshape = new b2CircleShape(b2Scalar(r));
            shape = cshape;
        }
        else if (rng.uniform01() < ellipseratio) // ellipse shape
        {
            double  r = 0.6 * rmin + 0.75 * deltar * rng.uniform01();
            b2EllipseShape *eshape = new b2EllipseShape(b2Scalar(r), b2Scalar(rng.uniformReal(0.5, 0.7) * r));
            shape = eshape;
        }
        else if (rng.uniform01() < capsuleratio) // capsule shape
        {
            double  r = 0.35 * rmin + 0.35 * deltar * rng.uniform01();
            b2CapsuleShape *cshape = new b2CapsuleShape(b2Scalar(rng.uniformReal(0.8, 1.2) * r), b2Scalar(r));
            shape = cshape;
        }
        else if (rng.uniform01() < rectangeleratio) // rectangle shape
        {
            double  r = 0.6 * rmin + 0.6 * deltar * rng.uniform01();
            b2RectangleShape *rectshape = new b2RectangleShape(b2Scalar(r), b2Scalar(rng.uniformReal(0.5, 0.7) * r));
            shape = rectshape;
        }
        else // polygon 
        {
            polygon = true;
            double  r = rmin + deltar * rng.uniform01();
            double carea = boost::math::constants::pi<double>() * r * r;
            int count = rng.uniformInt(3, 5);
            double rad = boost::math::constants::pi<double>()/(double)count;
            b2Vec2 vecs[count];
            for (int j = 0; j < count; j++)
            {
                double vrad = ((double)j + rng.uniform01()) * rad;
                double x = xc + r * std::cos(vrad);
                double y = yc + r * std::sin(vrad);
                vecs[j] = b2Vec2(b2Scalar(x), b2Scalar(y));
            }

            b2PolygonShape *pshape = new b2PolygonShape();
            pshape->Set(vecs, count);
            b2Scalar parea = pshape->ComputeArea();
            if (parea > b2Scalar(0.2 * carea))
                shape = pshape;
            else 
                delete pshape;
        }

        if (shape != nullptr)
        {
            b2AABB saabb;
            b2Transform xf = b2Transform::Identity();
            if (!polygon)
            {
                xf.translation() = b2Vec2(b2Scalar(xc), b2Scalar(yc));
                xf.linear() = b2Rot(b2Scalar(yaw)).toRotationMatrix();
            }
            shape->ComputeAABB(&saabb, xf);
            if (aabb.contains(saabb))
            {
                i++;
                manager.AddBody("shape" + std::to_string(i), shape, b2Transform::Identity(), false);
                if (!polygon)
                    manager.SetBodyTransform("shape" + std::to_string(i), xf);
            }
            delete shape;
        }
    }

    if (disjoint)
    {
        const b2Body* bodylist = manager.GetBodyList();
        while (bodylist)
        {
            MQueryCallback callback(bodylist);

            const b2Fixture *flist = bodylist->GetFixtureList();
            while (flist)
            {
                b2AABB faabb = flist->GetAABB();
                manager.Collide(&callback, faabb);
                flist = flist->GetNext();
            }

            std::set<std::string> collisions = callback.getCollisionBodies();
            for (auto & name : collisions)
                manager.RemoveBody(name);

            bodylist = bodylist->GetNext();
        }
    }

    if (decorate)
        std::cout << "Obstacles number " << manager.GetProxyCount() + 160 << std::endl;
    else
        std::cout << "Obstacles number " << manager.GetProxyCount() << std::endl;

    std::ofstream ofs("random_scenarios.ply", std::ios::binary | std::ios::out);
    const std::string& fileHeader = "# Random 2D scenarios with circle, polygon, ellipse, capsule and rectangle shapes";
    ofs << fileHeader <<"\n# (feel free to add / change comments, but leave the first line as it is!)\n#\n";
    ofs << "# 0: circle, 1: polygon, 2: ellipse, 3: capsule, 4: rectangle" << std::endl;
    ofs << "# circle: 0 xc yc radius" << std::endl;
    ofs << "# polygon: 1 count x1 y1 x2 y2 ..." << std::endl;
    ofs << "# ellipse: 2 xc yc yaw h_a h_b" << std::endl;
    ofs << "# capsule: 3 xc yc yaw r h" << std::endl;
    ofs << "# rectangle: 4 xc yc yaw h_a h_b" << std::endl;
    if (decorate)
        ofs << "numbers " << manager.GetProxyCount() + 160 << std::endl;
    else 
        ofs << "numbers " << manager.GetProxyCount() << std::endl;
    ofs << "shapes" << std::endl;

    const b2Body* bodylist = manager.GetBodyList();
    while (bodylist)
    {
        const b2Fixture *flist = bodylist->GetFixtureList();
        const b2Shape *shape = flist->GetShape();
        const b2Transform& xf = flist->GetGlobalTransform();

        if (shape->GetType() == b2Shape::e_circle)
        {
            const b2CircleShape *cshape = static_cast<const b2CircleShape *>(shape);
            ofs << 0 << " " << static_cast<double>(xf.translation().x()) << " " << static_cast<double>(xf.translation().y()) << " " << static_cast<double>(cshape->GetRadius()) << std::endl; 
        }
        else if (shape->GetType() == b2Shape::e_polygon)
        {
            const b2PolygonShape *pshape = static_cast<const b2PolygonShape *>(shape);
            ofs << 1 << " " << pshape->GetVerticesCount();
            for (int i = 0; i < pshape->GetVerticesCount(); i++)
            {
                ofs << " " << static_cast<double>(pshape->GetVertice(i).x()) << " " << static_cast<double>(pshape->GetVertice(i).y());
            }
            ofs << std::endl;
        }
        else if (shape->GetType() == b2Shape::e_ellipse)
        {
            const b2EllipseShape *eshape = static_cast<const b2EllipseShape *>(shape);
            b2Vec2 hsides = eshape->GetHalfSides();
            ofs << 2 << " " << static_cast<double>(xf.translation().x()) << " " << static_cast<double>(xf.translation().y()) << " " << static_cast<double>(b2Rot(xf.linear()).angle()) << " " << static_cast<double>(hsides.x()) << " " << static_cast<double>(hsides.y()) << std::endl; 
        }
        else if (shape->GetType() == b2Shape::e_capsule)
        {
            const b2CapsuleShape *cshape = static_cast<const b2CapsuleShape *>(shape);
            ofs << 3 << " " << static_cast<double>(xf.translation().x()) << " " << static_cast<double>(xf.translation().y()) << " " << static_cast<double>(b2Rot(xf.linear()).angle()) << " " << static_cast<double>(cshape->GetRadius()) << " " << static_cast<double>(cshape->GetHeight()) << std::endl; 
        }
        else if (shape->GetType() == b2Shape::e_rectangle)
        {
            const b2RectangleShape *rectshape = static_cast<const b2RectangleShape *>(shape);
            b2Vec2 hsides = rectshape->GetHalfSides();
            ofs << 4 << " " << static_cast<double>(xf.translation().x()) << " " << static_cast<double>(xf.translation().y()) << " " << static_cast<double>(b2Rot(xf.linear()).angle()) << " " << static_cast<double>(hsides.x()) << " " << static_cast<double>(hsides.y()) << std::endl; 
        }

        bodylist = bodylist->GetNext();
    }

    if (decorate)
    {
        int count = 20;
        double r = 1.0 / (4.0 * count);
        for (int j = 0; j < 2 * count; j++)
        {
            double xc = 2.0 * r * j + r;
            double yc = 0.0;
            ofs << 0 << " " << xc << " " << yc << " " << r << std::endl; 
            ofs << 0 << " " << 1.0 - xc << " " << 1.0 + yc << " " << r << std::endl;
            ofs << 0 << " " << yc << " " << 1.0 - xc << " " << r << std::endl;
            ofs << 0 << " " << 1.0 + yc << " " << xc << " " << r << std::endl;
            /*
            xc += 2.0 * r;
            yc = -std::sqrt(3) * r;
            ofs << 0 << " " << xc << " " << yc << " " << 2.0 * r << std::endl;
            ofs << 0 << " " << yc << " " << 1.0 - xc << " " << 2.0 * r << std::endl;

            yc = std::sqrt(3) * r;
            ofs << 0 << " " << 1.0 - xc << " " << 1.0 + yc << " " << 2.0 * r << std::endl;
            ofs << 0 << " " << 1.0 + yc << " " << xc << " " << 2.0 * r << std::endl;
            */
        }
    }

    ofs.close();

    return 0;
}

bool argParse(int argc, char** argv, double &rmin, double &deltar, int &expect_obstacles,
              double &circleratio, double &ellipseratio, double &capsuleratio, double &rectangeleratio, double &polygonratio,
              bool &disjoint, bool &decorate)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("rmin,r", bpo::value<double>()->default_value(0.01), "(Optional) Specify the minimum radius. Default to 0.01 and must be in range (0.0, 1.0)")
        ("deltar,d", bpo::value<double>()->default_value(0.01), "(Optional) Specify the radius increase range. Default to 0.01 and must be in range [0.0, 1.0 - rmin)")
        ("expect_obstacles,o", bpo::value<int>()->default_value(1000), "(Optional) Specify the expected obstacles number. Default to 1000 and must be greater than 1")
        ("circleratio,c", bpo::value<double>()->default_value(0.5), "(Optional) Specify the circle shapes' ratio. Default to 0.5 and must be in range [0.0, 1.0]")
        ("ellipseratio,e", bpo::value<double>()->default_value(0.5), "(Optional) Specify the circle shapes' ratio. Default to 0.5 and must be in range [0.0, 1.0]")
        ("capsuleratio,cap", bpo::value<double>()->default_value(0.5), "(Optional) Specify the circle shapes' ratio. Default to 0.5 and must be in range [0.0, 1.0]")
        ("rectangeleratio,rect", bpo::value<double>()->default_value(0.5), "(Optional) Specify the circle shapes' ratio. Default to 0.5 and must be in range [0.0, 1.0]")
        ("polygonratio,p", bpo::value<double>()->default_value(0.5), "(Optional) Specify the circle shapes' ratio. Default to 0.5 and must be in range [0.0, 1.0]")
        ("disjoint,dis", bpo::value<bool>()->default_value(true), "(Optional) Specify if these shapes are disjoint. Default to true")
        ("decorate,dec", bpo::value<bool>()->default_value(true), "(Optional) Specify if build circles along the edges. Default to true");
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
    rmin = vm["rmin"].as<double>();
    if (rmin <= 0.0 || rmin >= 1.0)
    {
        std::cout << "Invalid minimum radius." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    deltar = vm["deltar"].as<double>();
    if (deltar < 0.0 || deltar >= 1.0 - rmin)
    {
        std::cout << "Invalid radius increase range." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    expect_obstacles = vm["expect_obstacles"].as<int>();
    if (expect_obstacles < 1)
    {
        std::cout << "Invalid expected obstacles number." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    circleratio = vm["circleratio"].as<double>();
    if (circleratio < 0.0 || circleratio > 1.0)
    {
        std::cout << "Invalid circle shapes' ratio." << std::endl << std::endl << desc << std::endl;
        return false;
    }
    ellipseratio = vm["ellipseratio"].as<double>();
    if (ellipseratio < 0.0 || ellipseratio > 1.0)
    {
        std::cout << "Invalid ellipse shapes' ratio." << std::endl << std::endl << desc << std::endl;
        return false;
    }
    capsuleratio = vm["capsuleratio"].as<double>();
    if (capsuleratio < 0.0 || capsuleratio > 1.0)
    {
        std::cout << "Invalid capsule shapes' ratio." << std::endl << std::endl << desc << std::endl;
        return false;
    }
    rectangeleratio = vm["rectangeleratio"].as<double>();
    if (rectangeleratio < 0.0 || rectangeleratio > 1.0)
    {
        std::cout << "Invalid rectangle shapes' ratio." << std::endl << std::endl << desc << std::endl;
        return false;
    }
    polygonratio = vm["polygonratio"].as<double>();
    if (polygonratio < 0.0 || polygonratio > 1.0)
    {
        std::cout << "Invalid polygon shapes' ratio." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    disjoint = vm["disjoint"].as<bool>();
    decorate = vm["decorate"].as<bool>();

    return true;
}
