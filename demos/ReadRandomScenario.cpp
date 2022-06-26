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

#include <box2d_collision/b2_bvh_manager.h>
#include <ompl/util/Time.h>
#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/math/constants/constants.hpp>

using namespace ompl;

class MQueryCallback : public b2QueryCallback
{
public:
    MQueryCallback(const b2Shape * shape) : b2QueryCallback()
    {
        shape_ = shape;
        xf_.SetIdentity();
        collision_ = false;
    }

    ~MQueryCallback() override = default;

    /// Called for each fixture found in the query AABB.
    /// @return false to terminate the query.
    bool ReportFixture(b2Fixture* fixture, int32 childIndex) override
    {
        b2Transform xf;
        xf.SetIdentity();
        if (b2TestOverlap(shape_, 0, fixture->GetShape(), childIndex, xf_, xf))
        {
            collision_ = true;
            return false;
        }
        return true;
    }

    void setTransform(const b2Transform &xf)
    {
        xf_ = xf;
        collision_ = false;
    }

    bool getCollisionStatus() const 
    {
        return collision_;
    }

private:
    const b2Shape *shape_;
    b2Transform xf_;
    bool collision_;
};

bool readHeader(std::istream& s, int& numbers)
{
    numbers = 0;

    std::string token;
    bool headerRead = false;

    while (s.good() && !headerRead)
    {
        s >> token;
        if (token == "shapes")
        {
            headerRead = true;
            // skip forward until end of line:
            char c;
            do {
                c = s.get();
            } while(s.good() && (c != '\n'));
        }
        else if (token.compare(0, 1, "#") == 0)
        {
            // comment line, skip forward until end of line:
            char c;
            do {
                c = s.get();
            } while(s.good() && (c != '\n'));
        }
        else if (token == "numbers")
            s >> numbers;
        else
        {
            OMPL_WARN("Unknown keyword in Random Polygons and Circles header, skipping: %s", token);
            char c;
            do {
                c = s.get();
            } while(s.good() && (c != '\n'));

        }
    }

    if (!headerRead)
    {
        OMPL_ERROR("Error reading Random Polygons and Circles header!");
        return false;
    }

    return true;
}

bool read(b2BVHManager &manager, std::istream &s)
{
    const std::string& fileHeader = "# Random polygons and circles file";

    // check if first line valid:
    std::string line;
    std::getline(s, line);
    if (line.compare(0, fileHeader.length(), fileHeader) !=0)
    {
        OMPL_ERROR("First line of file header does not start with %s", fileHeader);
        return false;
    }

    int numbers = 0;
    if (!readHeader(s, numbers))
        return false;

    int shapec = 0;
    int type = 0;
    int count = 0;
    double x = 0.0, y = 0.0, r = 0.0;
    while (s.good())
    {
        s >> type;
        if (!s.good())
            break;
        b2Shape *shape = nullptr;
        if (type == 0)
        {
            s >> x >> y >> r;
            b2CircleShape *cshape = new b2CircleShape();
            cshape->SetRadius(b2Scalar(r));
            cshape->m_p.Set(b2Scalar(x), b2Scalar(y));
            shape = cshape;
        }
        else if (type == 1) 
        {
            s >> count; 
            b2Vec2 vecs[count];
            for (int i = 0; i < count; i++)
            {
                s >> x >> y;
                vecs[i].Set(b2Scalar(x), b2Scalar(y));
            }

            b2PolygonShape *pshape = new b2PolygonShape();
            pshape->SetRadius(b2Scalar(0.0));
            pshape->Set(vecs, count);
            shape = pshape;
        }

        if (shape != nullptr)
        {
            shapec++;
            manager.AddBody("shape" + std::to_string(shapec), shape, false);
            delete shape;
        }
    }

    if (numbers != manager.GetProxyCount())
        OMPL_WARN("The shape number is wrong in this file, %d VS %d!", numbers, manager.GetProxyCount());

    return true;
}

bool read(b2BVHManager &manager, const std::string& filename)
{
    std::ifstream file(filename.c_str(), std::ios_base::in);

    if (!file.is_open())
    {
        OMPL_ERROR("Filestream to %s is not open, nothing read.", filename);
        return false;
    }

    return read(manager, file);
}

int main()
{
    b2BVHManager manager;
    read(manager, "../../resources/random_scenarios.ply");

    int count = 3;
    b2Vec2 vecs[count];
    double x[3] = {0.0086579571682871, -0.02506512753291945, 0.012808997914287135};
    double y[3] = {0.028723505664735693, 0.01648451945791818, -0.027128021904145316};
    for (int i = 0; i < count; i++)
        vecs[i].Set(b2Scalar(x[i]), b2Scalar(y[i]));
    b2PolygonShape *pshape = new b2PolygonShape();
    pshape->SetRadius(b2Scalar(0.0));
    pshape->Set(vecs, count);
    manager.AddBody("robot", pshape, true);

//    MQueryCallback callback(pshape);
    if (false)
    {
        std::ofstream ofs("collision_status.txt", std::ios::binary | std::ios::out);
        for (int i = 0; i < 1001; i++)
        {
            double yy = 0.001 * i;
            for (int j = 0; j < 1001; j++)
            {
                double xx = 0.001 * j;
                b2Transform xf(b2Vec2(b2Scalar(xx), b2Scalar(yy)), b2Scalar(0.0));
//                callback.setTransform(xf);
//                b2AABB aabb;
//                pshape->ComputeAABB(&aabb, xf, 0);
//                manager.QueryAABB(&callback, aabb);
//                bool collision = callback.getCollisionStatus();
                manager.SetBodyTransform("robot", xf);
                bool collision = manager.ContactTest();
                ofs << collision << " ";
            }
            ofs << std::endl;
        }
        ofs.close();
    }

    if (false)
    {
        ompl::RNG rng;
        std::ofstream ofs("contact_test.txt", std::ios::binary | std::ios::out);
        std::ofstream ofss("contact_spheres.txt", std::ios::binary | std::ios::out);
        for (int c = 0; c < 10; c++)
        {
            double xc = rng.uniform01();
            double yc = rng.uniform01();
            b2Transform xf(b2Vec2(b2Scalar(xc), b2Scalar(yc)), b2Scalar(0.0));
            //        callback.setTransform(xf);
            //        b2AABB aabb;
            //        pshape->ComputeAABB(&aabb, xf, 0);;
            //        manager.QueryAABB(&callback, aabb);
            //        bool collision = callback.getCollisionStatus();
            //        std::cout << xc << "  " << yc << "  " << collision << std::endl;
            manager.SetBodyTransform("robot", xf);
            b2WorldManifold manifold;
//            b2ContactResult contacts;
            b2InscribedSpheres inscribedSpheres;
            if (manager.ContactTest(&manifold, &inscribedSpheres))
            {
                ofs << xc << " " << yc << " ";
                for (int i = 0; i < manifold.pointCount; i++)
                {
                    b2Vec2 p1 = manifold.points[i];
                    b2Vec2 p2 = manifold.points[i] + manifold.separations[i] * manifold.normal;
                    ofs << p1.x << " " << p1.y << " " << p2.x << " " << p2.y << " ";
                }
                ofs << std::endl;
                /*
                b2Vec2 p1 = contacts.points[0];
                b2Vec2 p2 = contacts.points[1];
                ofs << p1.x << " " << p1.y << " " << p2.x << " " << p2.y << std::endl;
                */
                if (inscribedSpheres.has_sphere1)
                    ofss << inscribedSpheres.center1.x << " " << inscribedSpheres.center1.y << " " << inscribedSpheres.radius1 << " ";
                if (inscribedSpheres.has_sphere2)
                    ofss << inscribedSpheres.center2.x << " " << inscribedSpheres.center2.y << " " << inscribedSpheres.radius2 << " ";
                ofss << std::endl;
            }
        }
        ofs.close();
        ofss.close();
    }

    if (false)
    {
        ompl::RNG rng;
        std::ofstream ofs1("collision_points.txt", std::ios::binary | std::ios::out);
        int collision_count = 0;
        std::vector<double> xcs, ycs, radiuss;
        while (collision_count < 100)
        {
            double xc = rng.uniform01();
            double yc = rng.uniform01();
//            if (xc < 0.0125 || xc > 0.9875 || yc < 0.0125 || yc > 0.9875)
//                continue;
            b2Transform xf(b2Vec2(b2Scalar(xc), b2Scalar(yc)), b2Scalar(0.0));
            manager.SetBodyTransform("robot", xf);
            b2ContactResult contacts;
            bool collision = manager.ContactTest(&contacts);
            if (collision)
            {
                bool sc = false;
                for (std::size_t i = 0; i < xcs.size(); i++)
                {
                    double dist = std::sqrt((xc - xcs[i]) * (xc - xcs[i]) + (yc - ycs[i]) * (yc - ycs[i]));
                    if (dist <= radiuss[i])
                    {
                        sc = true;
                        break;
                    }
                }
                if (sc)
                    continue;

                double pd = 1.0;

                manager.DisableAll();
                manager.EnableBody(contacts.names[0]);
                manager.EnableBody(contacts.names[1]);
                double rmin = 0.0, rmax = 1.0, rmiddle = 0.5;
                while (true)
                { // x+
                    while (rmax - rmin > 1.e-8)
                    {
                        rmiddle = (rmin + rmax) / 2.0;
                        xf.Set(b2Vec2(b2Scalar(xc + rmiddle), b2Scalar(yc)), b2Scalar(0.0));
                        manager.SetBodyTransform("robot", xf);
                        if (manager.ContactTest())
                            rmin = rmiddle;
                        else 
                            rmax = rmiddle;
                    }

                    manager.EnableAll();
                    xf.Set(b2Vec2(b2Scalar(xc + rmax), b2Scalar(yc)), b2Scalar(0.0));
                    manager.SetBodyTransform("robot", xf);
                    b2ContactResult contact;
                    if (manager.ContactTest(&contact))
                    {
                        manager.DisableAll();
                        manager.EnableBody(contact.names[0]);
                        manager.EnableBody(contact.names[1]);
                        rmin = rmax;
                        rmax = 1.0;
                    }
                    else 
                    {
                        pd = std::min(rmin, pd);
                        break;
                    }
                }

                manager.DisableAll();
                manager.EnableBody(contacts.names[0]);
                manager.EnableBody(contacts.names[1]);
                rmin = 0.0, rmax = 1.0, rmiddle = 0.5;
                while (true)
                { // x-
                    while (rmax - rmin > 1.e-8)
                    {
                        rmiddle = (rmin + rmax) / 2.0;
                        xf.Set(b2Vec2(b2Scalar(xc - rmiddle), b2Scalar(yc)), b2Scalar(0.0));
                        manager.SetBodyTransform("robot", xf);
                        if (manager.ContactTest())
                            rmin = rmiddle;
                        else 
                            rmax = rmiddle;
                    }

                    manager.EnableAll();
                    xf.Set(b2Vec2(b2Scalar(xc - rmax), b2Scalar(yc)), b2Scalar(0.0));
                    manager.SetBodyTransform("robot", xf);
                    b2ContactResult contact;
                    if (manager.ContactTest(&contact))
                    {
                        manager.DisableAll();
                        manager.EnableBody(contact.names[0]);
                        manager.EnableBody(contact.names[1]);
                        rmin = rmax;
                        rmax = 1.0;
                    }
                    else 
                    {
                        pd = std::min(rmin, pd);
                        break;
                    }
                }

                manager.DisableAll();
                manager.EnableBody(contacts.names[0]);
                manager.EnableBody(contacts.names[1]);
                rmin = 0.0, rmax = 1.0, rmiddle = 0.5;
                while (true)
                { // y+
                    while (rmax - rmin > 1.e-8)
                    {
                        rmiddle = (rmin + rmax) / 2.0;
                        xf.Set(b2Vec2(b2Scalar(xc), b2Scalar(yc + rmiddle)), b2Scalar(0.0));
                        manager.SetBodyTransform("robot", xf);
                        if (manager.ContactTest())
                            rmin = rmiddle;
                        else 
                            rmax = rmiddle;
                    }

                    manager.EnableAll();
                    xf.Set(b2Vec2(b2Scalar(xc), b2Scalar(yc + rmax)), b2Scalar(0.0));
                    manager.SetBodyTransform("robot", xf);
                    b2ContactResult contact;
                    if (manager.ContactTest(&contact))
                    {
                        manager.DisableAll();
                        manager.EnableBody(contact.names[0]);
                        manager.EnableBody(contact.names[1]);
                        rmin = rmax;
                        rmax = 1.0;
                    }
                    else 
                    {
                        pd = std::min(rmin, pd);
                        break;
                    }
                }

                manager.DisableAll();
                manager.EnableBody(contacts.names[0]);
                manager.EnableBody(contacts.names[1]);
                rmin = 0.0, rmax = 1.0, rmiddle = 0.5;
                while (true)
                { // y-
                    while (rmax - rmin > 1.e-8)
                    {
                        rmiddle = (rmin + rmax) / 2.0;
                        xf.Set(b2Vec2(b2Scalar(xc), b2Scalar(yc - rmiddle)), b2Scalar(0.0));
                        manager.SetBodyTransform("robot", xf);
                        if (manager.ContactTest())
                            rmin = rmiddle;
                        else 
                            rmax = rmiddle;
                    }

                    manager.EnableAll();
                    xf.Set(b2Vec2(b2Scalar(xc), b2Scalar(yc - rmax)), b2Scalar(0.0));
                    manager.SetBodyTransform("robot", xf);
                    b2ContactResult contact;
                    if (manager.ContactTest(&contact))
                    {
                        manager.DisableAll();
                        manager.EnableBody(contact.names[0]);
                        manager.EnableBody(contact.names[1]);
                        rmin = rmax;
                        rmax = 1.0;
                    }
                    else 
                    {
                        pd = std::min(rmin, pd);
                        break;
                    }
                }

                manager.EnableAll();
                if (pd > 0.02 && pd < 0.04)
                {
                    collision_count++;
                    ofs1 << xc << " " << yc << " " << pd << std::endl;
                    xcs.push_back(xc);
                    ycs.push_back(yc);
                    radiuss.push_back(pd);
                    std::cout << "collision_count " << collision_count << std::endl;
                }
            }
        }
        ofs1.close();
    }

    if (false)
    {
        ompl::RNG rng;
        std::ofstream ofs2("safe_points.txt", std::ios::binary | std::ios::out);
        int safe_cont = 0;
        std::vector<double> xcs, ycs, radiuss;
        while (safe_cont < 350)
        {
            double xc = rng.uniform01();
            double yc = rng.uniform01();
            b2Transform xf(b2Vec2(b2Scalar(xc), b2Scalar(yc)), b2Scalar(0.0));
            manager.SetBodyTransform("robot", xf);
            bool collision = manager.ContactTest();
            if (!collision)
            {
                bool sc = false;
                for (std::size_t i = 0; i < xcs.size(); i++)
                {
                    double dist = std::sqrt((xc - xcs[i]) * (xc - xcs[i]) + (yc - ycs[i]) * (yc - ycs[i]));
                    if (dist <= radiuss[i])
                    {
                        sc = true;
                        break;
                    }
                }
                if (sc)
                    continue;
                double rmin = 0.0, rmax = 0.1, rmiddle = 0.05;
                while (rmax - rmin > 1.e-8)
                {
                    manager.RemoveBody("robot");
                    pshape->SetRadius(b2Scalar(rmiddle));
                    manager.AddBody("robot", pshape, true);
                    manager.SetBodyTransform("robot", xf);
                    if (manager.ContactTest())
                        rmax = rmiddle;
                    else 
                        rmin = rmiddle;
                    rmiddle = (rmin + rmax) / 2.0;
                }
                manager.RemoveBody("robot");
                pshape->SetRadius(b2Scalar(0.0));
                manager.AddBody("robot", pshape, true);
                if (rmin >= 0.015)
                {
                    safe_cont++;
                    ofs2 << xc << " " << yc << " " << rmin << std::endl;
                    xcs.push_back(xc);
                    ycs.push_back(yc);
                    radiuss.push_back(rmin);
                    std::cout << "safe_cont " << safe_cont << std::endl;
                }
            }
        }
        ofs2.close();
    }

    if (false)
    {
        int robotC = 15;
        manager.RemoveBody("robot");
        for (int i = 0; i < robotC; i++)
            manager.AddBody("robot" + std::to_string(i), pshape, true);

        b2BVHManager manager2;
        read(manager2, "../../resources/random_scenarios.ply");

        b2BVHManager manager3;
        for (int i = 0; i < robotC; i++)
            manager3.AddBody("robot" + std::to_string(i), pshape, true);

        ompl::RNG rng;
        int iter = 0, count = 1.e4;
        double time1 = 0.0, time2 = 0.0, time3 = 0.0;
        double pi = boost::math::constants::pi<double>();
        while (iter < count)
        {
            for (int i = 0; i < robotC; i++)
            {
                double xc = rng.uniform01();
                double yc = rng.uniform01();
                double yaw= rng.uniformReal(-pi, pi);
                /*
                xc = 0.0235317;
                yc = 0.403731;
                yaw= 0.435234;
                std::cout << xc << " " << yc << " " << yaw << std::endl;
                */
                b2Transform xf;
                xf.Set(b2Vec2(b2Scalar(xc), b2Scalar(yc)), b2Scalar(yaw));

                time::point start = time::now();
                manager.SetBodyTransform("robot" + std::to_string(i), xf);
                time1 += time::seconds(time::now() - start);

                start = time::now();
                manager3.SetBodyTransform("robot" + std::to_string(i), xf);
                time2 += time::seconds(time::now() - start);
                time3 += time::seconds(time::now() - start);
            }

            time::point start = time::now();
            bool collision1 = manager.ContactTest();
            time1 += time::seconds(time::now() - start);

            start = time::now();
            bool collision2 = manager3.ContactTest(&manager2);
            time2 += time::seconds(time::now() - start);

            start = time::now();
            bool collision3 = manager2.ContactTest(&manager3);
            time3 += time::seconds(time::now() - start);

            if (collision1 != collision2)
                OMPL_ERROR("Error!");
            else if (collision2 != collision3)
                OMPL_ERROR("Error!");

            iter++;
        }

        OMPL_INFORM("Time1 %.4f, Time2 %.4f, Time3 %.4f", time1, time2, time3);
    }

    delete pshape;
    return 0;
}
