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

class MQueryCallback : public b2QueryCallback
{
    public:
        MQueryCallback(const b2Body * body) : b2QueryCallback()
    {
        xf.SetIdentity();
        body_ = body;
    }

        ~MQueryCallback() override = default;

        /// Called for each fixture found in the query AABB.
        /// @return false to terminate the query.
        bool ReportFixture(b2Fixture* fixture, int32 childIndex) override
        {
            if (fixture->GetBody() != body_)
            {
                const b2Fixture* flist = body_->GetFixtureList();
                int32 count = flist->GetProxyCount();
                for (int32 i = 0; i < count; i++)
                {
                    if (b2TestOverlap(flist->GetShape(), i, fixture->GetShape(), childIndex, xf, xf))
                    {
                        collisionBodies_.insert(fixture->GetBody()->GetName());
                        return true;
                    }
                }
            }

            return true;
        }

        std::set<std::string> getCollisionBodies() const 
        {
            return collisionBodies_;
        }

    private:
        b2Transform xf;
        const b2Body* body_;
        std::set<std::string> collisionBodies_; 
};

bool argParse(int argc, char** argv, double &rmin, double &deltar, int &expect_obstacles, double &circleratio, double &polygonarearatio, bool &disjoint);

int main(int argc, char* argv[])
{
    double width = 1.0;
    double rmin = 0.01 * width, deltar = 0.01 * width;
    double bmin = 0.005 * width, bmax = width - bmin;
    double circleratio = 0.05;
    double polygonarearatio = 0.2;
    int nobstacle = 1000;
    bool disjoint = true;

    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, rmin, deltar, nobstacle, circleratio, polygonarearatio, disjoint))
    {
        return -1;
    }

    b2AABB aabb;
    aabb.SetMin(b2Scalar(bmin));
    aabb.SetMax(b2Scalar(bmax));

    ompl::RNG rng;

    b2BVHManager manager;

    for (int i = 0; i < nobstacle;)
    {
        b2Shape *shape = nullptr;
        double xc = width * rng.uniform01();
        double yc = width * rng.uniform01();

        if (rng.uniform01() < circleratio) // circle shape
        {
            double  r = 0.5 * rmin + 0.5 * deltar * rng.uniform01();
            b2CircleShape *cshape = new b2CircleShape();
            cshape->SetRadius(b2Scalar(r));
            cshape->m_p.Set(b2Scalar(xc), b2Scalar(yc));
            shape = cshape;
        }
        else // polygon 
        {
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
                vecs[j].Set(b2Scalar(x), b2Scalar(y));
            }

            b2PolygonShape *pshape = new b2PolygonShape();
            pshape->SetRadius(b2Scalar(0.0));
            pshape->Set(vecs, count);
            b2Scalar parea = pshape->ComputeArea();
            if (parea > b2Scalar(polygonarearatio * carea))
                shape = pshape;
            else 
                delete pshape;
        }

        if (shape != nullptr)
        {
            b2AABB saabb;
            b2Transform xf;
            xf.SetIdentity();
            shape->ComputeAABB(&saabb, xf, 0);
            if (aabb.Contains(saabb))
            {
                i++;
                manager.AddBody("shape" + std::to_string(i), shape, false);
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
                int count = flist->GetProxyCount();
                for (int i = 0; i < count; i++)
                {
                    b2AABB faabb = flist->GetAABB(i);
                    manager.QueryAABB(&callback, faabb);
                }
                flist = flist->GetNext();
            }

            std::set<std::string> collisions = callback.getCollisionBodies();
            for (auto & name : collisions)
                manager.RemoveBody(name);

            bodylist = bodylist->GetNext();
        }
    }

    std::cout << "Obstacles number " << manager.GetProxyCount() << std::endl;

    std::ofstream ofs("random_scenarios.ply", std::ios::binary | std::ios::out);
    const std::string& fileHeader = "# Random polygons and circles file";
    ofs << fileHeader <<"\n# (feel free to add / change comments, but leave the first line as it is!)\n#\n";
    ofs << "numbers " << manager.GetProxyCount() << std::endl;
    ofs << "shapes" << std::endl;

    const b2Body* bodylist = manager.GetBodyList();
    while (bodylist)
    {
        const b2Fixture *flist = bodylist->GetFixtureList();
        const b2Shape *shape = flist->GetShape();

        if (shape->GetType() == b2Shape::e_circle)
        {
            const b2CircleShape *cshape = static_cast<const b2CircleShape *>(shape);
            ofs << 0 << " " << static_cast<double>(cshape->m_p.x) << " " << static_cast<double>(cshape->m_p.y) << " " << static_cast<double>(cshape->GetRadius()) << std::endl; 
        }
        else if (shape->GetType() == b2Shape::e_polygon)
        {
            const b2PolygonShape *pshape = static_cast<const b2PolygonShape *>(shape);
            ofs << 1 << " " << pshape->m_count;
            for (int32 i = 0; i < pshape->m_count; i++)
            {
                ofs << " " << static_cast<double>(pshape->m_vertices[i].x) << " " << static_cast<double>(pshape->m_vertices[i].y);
            }
            ofs << std::endl;
        }

        bodylist = bodylist->GetNext();
    }

    if (true)
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

bool argParse(int argc, char** argv, double &rmin, double &deltar, int &expect_obstacles, double &circleratio, double &polygonarearatio, bool &disjoint)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("rmin,r", bpo::value<double>()->default_value(0.01), "(Optional) Specify the minimum radius. Default to 0.01 and must be in range (0.0, 1.0)")
        ("deltar,d", bpo::value<double>()->default_value(0.01), "(Optional) Specify the radius increase range. Default to 0.01 and must be in range [0.0, 1.0 - rmin)")
        ("expect_obstacles,o", bpo::value<int>()->default_value(1000), "(Optional) Specify the expected obstacles number. Default to 1000 and must be greater than 1")
        ("circleratio,c", bpo::value<double>()->default_value(0.05), "(Optional) Specify the circle shapes' ratio. Default to 0.05 and must be in range [0.0, 1.0]")
        ("polygonarearatio,p", bpo::value<double>()->default_value(0.2), "(Optional) Specify the polygon area ratio with respect to a circle. Default to 0.2 and must be in range (0.0, 0.5)")
        ("disjoint,dis", bpo::value<bool>()->default_value(true), "(Optional) Specify if these shapes are disjoint. Default to true");
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

    polygonarearatio = vm["polygonarearatio"].as<double>();
    if (polygonarearatio < 0.0 || polygonarearatio > 0.5)
    {
        std::cout << "Invalid polygon area ratio." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    disjoint = vm["disjoint"].as<bool>();

    return true;
}
