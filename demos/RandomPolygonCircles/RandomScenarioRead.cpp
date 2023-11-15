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

#include <omplapp/config.h>
#include <box2d_collision/b2_bvh_manager.h>
#include <ompl/util/Time.h>
#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

using namespace ompl;

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
            OMPL_WARN("Unknown keyword in Random 2D Scenarios header, skipping: %s", token);
            char c;
            do {
                c = s.get();
            } while(s.good() && (c != '\n'));
        }
    }

    if (!headerRead)
    {
        OMPL_ERROR("Error reading Random 2D Scenarios header!");
        return false;
    }

    return true;
}

bool read(b2BVHManager &manager, std::istream &s)
{
    const std::string& fileHeader = "# Random 2D scenarios with circle, polygon, ellipse, capsule and rectangle shapes";

    // check if first line valid:
    std::string line;
    std::getline(s, line);
    if (line.compare(0, fileHeader.length(), fileHeader) !=0)
    {
        OMPL_ERROR("First line of file header does not start with %s", fileHeader);
        return false;
    }

    // read shape numbers
    int numbers = 0;
    if (!readHeader(s, numbers))
        return false;

    int shapec = 0;
    int type = 0;
    int count = 0;
    double x = 0.0, y = 0.0, yaw = 0.0, r = 0.0, h = 0.0, a = 0.0, b = 0.0;
    while (s.good())
    {
        s >> type;
        if (!s.good())
            break;
        b2Shape *shape = nullptr;
        b2Transform xf = b2Transform::Identity();
        bool polygon = false;
        if (type == 0) // circle
        {
            s >> x >> y >> r;
            yaw = 0.0;
            b2CircleShape *cshape = new b2CircleShape(b2Scalar(r));
            xf.translation() = b2Vec2(b2Scalar(x), b2Scalar(y));
            shape = cshape;
        }
        else if (type == 1) // polygon 
        {
            polygon = true;
            s >> count; 
            b2Vec2 vecs[count];
            for (int i = 0; i < count; i++)
            {
                s >> x >> y;
                vecs[i] = b2Vec2(b2Scalar(x), b2Scalar(y));
            }

            b2PolygonShape *pshape = new b2PolygonShape();
            pshape->Set(vecs, count);
            shape = pshape;
        }
        else if (type == 2) // ellipse
        {
            s >> x >> y >> yaw >> a >> b;
            b2EllipseShape *eshape = new b2EllipseShape(b2Scalar(a), b2Scalar(b));
            xf.translation() = b2Vec2(b2Scalar(x), b2Scalar(y));
            xf.linear() = b2Rot(b2Scalar(yaw)).toRotationMatrix();
            shape = eshape;
        }
        else if (type == 3) // capsule 
        {
            s >> x >> y >> yaw >> r >> h;
            b2CapsuleShape *cshape = new b2CapsuleShape(b2Scalar(r), b2Scalar(h));
            xf.translation() = b2Vec2(b2Scalar(x), b2Scalar(y));
            xf.linear() = b2Rot(b2Scalar(yaw)).toRotationMatrix();
            shape = cshape;
        }
        else if (type == 4) // rectangle 
        {
            s >> x >> y >> yaw >> a >> b;
            b2RectangleShape *rectshape = new b2RectangleShape(b2Scalar(a), b2Scalar(b));
            xf.translation() = b2Vec2(b2Scalar(x), b2Scalar(y));
            xf.linear() = b2Rot(b2Scalar(yaw)).toRotationMatrix();
            shape = rectshape;
        }

        if (shape != nullptr)
        {
            shapec++;
            manager.AddBody("shape" + std::to_string(shapec), shape, b2Transform::Identity(), false);
            if (!polygon)
                manager.SetBodyTransform("shape" + std::to_string(shapec), xf);
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

bool argParse(int argc, char** argv, std::string &env, bool &collision_status, bool &contact_test, bool &collision_points, bool &safe_points, bool &compare);

int main(int argc, char* argv[])
{
    std::string env;
    bool collision_status = false;
    bool contact_test = false;
    bool collision_points = false;
    bool safe_points = false;
    bool compare = false;

    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, env, collision_status, contact_test, collision_points, safe_points, compare))
        return -1;

    b2BVHManager manager;
    read(manager, std::string(OMPLAPP_RESOURCE_DIR) + "/" + env);

    int count = 3;
    b2Vec2 vecs[count];
    double x[3] = {0.0086579571682871, -0.02506512753291945, 0.012808997914287135};
    double y[3] = {0.028723505664735693, 0.01648451945791818, -0.027128021904145316};
    for (int i = 0; i < count; i++)
        vecs[i] = b2Vec2(b2Scalar(x[i]), b2Scalar(y[i]));
    b2PolygonShape *pshape = new b2PolygonShape();
    pshape->Set(vecs, count);
    manager.AddBody("robot", pshape, b2Transform::Identity(), true);

    if (collision_status)
    {
        std::ofstream ofs("collision_status.txt", std::ios::binary | std::ios::out);
        for (int i = 0; i < 1001; i++)
        {
            double yy = 0.001 * i;
            for (int j = 0; j < 1001; j++)
            {
                double xx = 0.001 * j;
                b2Transform xf = b2Transform::Identity();
                xf.translation() = b2Vec2(b2Scalar(xx), b2Scalar(yy));
                manager.SetBodyTransform("robot", xf);
                bool collision = manager.Collide();
                ofs << collision << " ";
            }
            ofs << std::endl;
        }
        ofs.close();
    }

    if (contact_test)
    {
        ompl::RNG rng;
        std::ofstream ofs("contact_test.txt", std::ios::binary | std::ios::out);
        std::ofstream ofss("contact_spheres.txt", std::ios::binary | std::ios::out);
        for (int c = 0; c < 10; c++)
        {
            double xc = rng.uniform01();
            double yc = rng.uniform01();

            b2Transform xf = b2Transform::Identity();
            xf.translation() = b2Vec2(b2Scalar(xc), b2Scalar(yc));
            manager.SetBodyTransform("robot", xf);
            b2ContactResult contacts;
            b2InscribedSpheres inscribedSpheres;
            if (manager.Collide(&contacts, &inscribedSpheres))
            {
                ofs << xc << " " << yc << " ";
                b2Vec2 p1 = contacts.points[0];
                b2Vec2 p2 = contacts.points[1];
                ofs << p1.x() << " " << p1.y() << " " << p2.x() << " " << p2.y() << " ";
                ofs << std::endl;
                if (inscribedSpheres.has_sphere1)
                    ofss << inscribedSpheres.center1.x() << " " << inscribedSpheres.center1.y() << " " << inscribedSpheres.radius1 << " ";
                if (inscribedSpheres.has_sphere2)
                    ofss << inscribedSpheres.center2.x() << " " << inscribedSpheres.center2.y() << " " << inscribedSpheres.radius2 << " ";
                ofss << std::endl;
            }
        }
        ofs.close();
        ofss.close();
    }

    if (collision_points)
    {
        ompl::RNG rng;
        std::ofstream ofs1("collision_points.txt", std::ios::binary | std::ios::out);
        int collision_count = 0;
        std::vector<double> xcs, ycs, radiuss;
        while (collision_count < 100)
        {
            double xc = rng.uniform01();
            double yc = rng.uniform01();
            if (xc < 0.05 || xc > 0.95 || yc < 0.05 || yc > 0.95)
                continue;
            b2Transform xf = b2Transform::Identity();
            xf.translation() = b2Vec2(b2Scalar(xc), b2Scalar(yc));
            manager.SetBodyTransform("robot", xf);
            b2ContactResult contacts;
            bool collision = manager.Collide(&contacts);
            if (collision)
            {
                bool sc = false;
                for (std::size_t i = 0; i < xcs.size(); i++) // filter out
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
                    while (rmax - rmin > 1.e-2) // minimum translation
                    {
                        rmiddle = (rmin + rmax) / 2.0;
                        xf.translation() = b2Vec2(b2Scalar(xc + rmiddle), b2Scalar(yc));
                        manager.SetBodyTransform("robot", xf);
                        if (manager.Collide())
                            rmin = rmiddle;
                        else 
                            rmax = rmiddle;
                    }

                    manager.EnableAll();
                    xf.translation() = b2Vec2(b2Scalar(xc + rmax), b2Scalar(yc));
                    manager.SetBodyTransform("robot", xf);
                    b2ContactResult contact;
                    if (manager.Collide(&contact)) // another contact
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
                    while (rmax - rmin > 1.e-2) // minimum translation
                    {
                        rmiddle = (rmin + rmax) / 2.0;
                        xf.translation() = b2Vec2(b2Scalar(xc - rmiddle), b2Scalar(yc));
                        manager.SetBodyTransform("robot", xf);
                        if (manager.Collide())
                            rmin = rmiddle;
                        else 
                            rmax = rmiddle;
                    }

                    manager.EnableAll();
                    xf.translation() = b2Vec2(b2Scalar(xc - rmax), b2Scalar(yc));
                    manager.SetBodyTransform("robot", xf);
                    b2ContactResult contact;
                    if (manager.Collide(&contact)) // another contact
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
                    while (rmax - rmin > 1.e-2) // minimum translation
                    {
                        rmiddle = (rmin + rmax) / 2.0;
                        xf.translation() = b2Vec2(b2Scalar(xc), b2Scalar(yc + rmiddle));
                        manager.SetBodyTransform("robot", xf);
                        if (manager.Collide())
                            rmin = rmiddle;
                        else 
                            rmax = rmiddle;
                    }

                    manager.EnableAll();
                    xf.translation() = b2Vec2(b2Scalar(xc), b2Scalar(yc + rmax));
                    manager.SetBodyTransform("robot", xf);
                    b2ContactResult contact;
                    if (manager.Collide(&contact)) // another contact
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
                    while (rmax - rmin > 1.e-2) // minimum translation
                    {
                        rmiddle = (rmin + rmax) / 2.0;
                        xf.translation() = b2Vec2(b2Scalar(xc), b2Scalar(yc - rmiddle));
                        manager.SetBodyTransform("robot", xf);
                        if (manager.Collide())
                            rmin = rmiddle;
                        else 
                            rmax = rmiddle;
                    }

                    manager.EnableAll();
                    xf.translation() = b2Vec2(b2Scalar(xc), b2Scalar(yc - rmax));
                    manager.SetBodyTransform("robot", xf);
                    b2ContactResult contact;
                    if (manager.Collide(&contact)) // another contact
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

    /*
    if (safe_points)
    {
        ompl::RNG rng;
        std::ofstream ofs2("safe_points.txt", std::ios::binary | std::ios::out);
        int safe_cont = 0;
        std::vector<double> xcs, ycs, radiuss;
        while (safe_cont < 35)
        {
            double xc = rng.uniform01();
            double yc = rng.uniform01();

            b2Transform xf = b2Transform::Identity();
            xf.translation() = b2Vec2(b2Scalar(xc), b2Scalar(yc));
            manager.SetBodyTransform("robot", xf);
            bool collision = manager.Collide();
            if (!collision)
            {
                bool sc = false;
                for (std::size_t i = 0; i < xcs.size(); i++) // filter
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
                    if (manager.Collide())
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
    */

    if (compare)
    {
        int robotC = 15;
        manager.RemoveBody("robot"); // self collision
        for (int i = 0; i < robotC; i++)
            manager.AddBody("robot" + std::to_string(i), pshape, b2Transform::Identity(), true);

        b2BVHManager manager2; // static obstacles
        read(manager2, std::string(OMPLAPP_RESOURCE_DIR) + "/" + env);

        b2BVHManager manager3; // moving robots 
        for (int i = 0; i < robotC; i++)
            manager3.AddBody("robot" + std::to_string(i), pshape, b2Transform::Identity(), true);

        ompl::RNG rng;

        rng.setLocalSeed(74459601);
        OMPL_INFORM("rng seed %u", rng.getLocalSeed());

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

                b2Transform xf = b2Transform::Identity();
                xf.translation() = b2Vec2(b2Scalar(xc), b2Scalar(yc));
                xf.linear() = b2Rot(b2Scalar(yaw)).toRotationMatrix();

                time::point start = time::now();
                manager.SetBodyTransform("robot" + std::to_string(i), xf);
                time1 += time::seconds(time::now() - start);

                start = time::now();
                manager3.SetBodyTransform("robot" + std::to_string(i), xf);
                time2 += time::seconds(time::now() - start);
                time3 += time::seconds(time::now() - start);
            }

            time::point start = time::now();
            bool collision1 = manager.Collide();
            time1 += time::seconds(time::now() - start);

            start = time::now();
            bool collision2 = manager3.Collide(&manager2);
            time2 += time::seconds(time::now() - start);

            start = time::now();
            bool collision3 = manager2.Collide(&manager3);
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

bool argParse(int argc, char** argv, std::string &env, bool &collision_status, bool &contact_test, bool &collision_points, bool &safe_points, bool &compare)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("env,e", bpo::value<std::string>()->default_value("random_scenarios.ply"), "(Optional) Specify the polygon and circle environment, defaults to random_scenarios.ply if not given.")
        ("collision_status", bpo::value<bool>()->default_value(false), "Specify collision status. Default to False")
        ("contact_test", bpo::value<bool>()->default_value(false), "Specify contact test. Default to False")
        ("collision_points", bpo::value<bool>()->default_value(false), "Specify collision pointst. Default to False")
        ("safe_points", bpo::value<bool>()->default_value(false), "Specify safe points. Default to False")
        ("compare", bpo::value<bool>()->default_value(false), "Specify safe points. Default to False");
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

    collision_status = vm["collision_status"].as<bool>();
    contact_test = vm["contact_test"].as<bool>();
    collision_points = vm["collision_points"].as<bool>();
    safe_points = vm["safe_points"].as<bool>();
    compare = vm["compare"].as<bool>();

    return true;
}
