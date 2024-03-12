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
#include <ompl/util/Time.h>
#include <ompl/util/Console.h>
#include <box2d_collision/b2_bvh_manager.h>
#include <box2d_collision/b2_distance.h>

#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <set>
#include <iostream>
#include <fstream>

using namespace ompl;

template<typename b2Shape1, typename b2Shape2>
void timeCollect(const b2Shape1 *s1, const std::vector<b2Transform> &transforms1,
                 const b2Shape2 *s2, const std::vector<b2Transform> &transforms2,
                 double &time1, double &time2, double &error);

int main()
{
    // Circle-Ellipse-Capsule-Rectangle-Polygon
    std::size_t num = 5, i = 0, j = 0;
    std::vector<std::vector<double>> vtime1(num, std::vector<double>(num, 0.0));
    std::vector<std::vector<double>> vtime2(num, std::vector<double>(num, 0.0));
    std::vector<std::vector<double>> verror(num, std::vector<double>(num, 0.0));

    ompl::RNG rng;
    for (std::size_t len = 0; len < 1; len++)
    {
        std::size_t n = 100; // generate n random transforms
        double extents[] = {-rng.uniformReal(5.0, 100.0), -rng.uniformReal(5.0, 100.0), rng.uniformReal(5.0, 100.0), rng.uniformReal(5.0, 100.0)};
        std::vector<b2Transform> transforms1(n), transforms2(n);
        for (std::size_t i = 0; i < n; i++)
        {
            double xc = rng.uniformReal(extents[0], extents[2]);
            double yc = rng.uniformReal(extents[1], extents[3]);
            double yaw = rng.uniformReal(-boost::math::constants::pi<double>(), boost::math::constants::pi<double>());
            b2Transform xf = b2Transform::Identity();
            xf.translation() = b2Vec2(b2Scalar(xc), b2Scalar(yc));
            xf.linear() = b2Rot(b2Scalar(yaw)).toRotationMatrix();
            transforms1[i] = xf;

            xc = rng.uniformReal(extents[0], extents[2]);
            yc = rng.uniformReal(extents[1], extents[3]);
            yaw = rng.uniformReal(-boost::math::constants::pi<double>(), boost::math::constants::pi<double>());
            xf.translation() = b2Vec2(b2Scalar(xc), b2Scalar(yc));
            xf.linear() = b2Rot(b2Scalar(yaw)).toRotationMatrix();
            transforms2[i] = xf;
        }

        double low = rng.uniformReal(0.1, 10.0), high = rng.uniformReal(0.1, 10.0) + low;

        //==============================================================================
        // Circle
        i = 0;
        {
            j = 0;
            b2CircleShape s1(rng.uniformReal(low, high));
            b2CircleShape s2(rng.uniformReal(low, high));
            double time1 = 0.0, time2 = 0.0, error = 0.0;
            timeCollect<b2CircleShape, b2CircleShape>(&s1, transforms1, &s2, transforms2, time1, time2, error);
            std::cout << "Circle-Circle " << "GJK-Time " << time1 << " Bisection-Time " << time2 << " error " << error << std::endl;
            vtime1[i][j] += time1;
            vtime2[i][j] += time2;
            verror[i][j] += error;
        }

        {
            j = 1;
            b2CircleShape s1(rng.uniformReal(low, high));
            b2EllipseShape s2(rng.uniformReal(low, high), rng.uniformReal(low, high));
            double time1 = 0.0, time2 = 0.0, error = 0.0;
            timeCollect<b2CircleShape, b2EllipseShape>(&s1, transforms1, &s2, transforms2, time1, time2, error);
            std::cout << "Circle-Ellipse " << "GJK-Time " << time1 << " Bisection-Time " << time2 << " error " << error << std::endl;
            vtime1[i][j] += time1;
            vtime2[i][j] += time2;
            verror[i][j] += error;
        }
        
        {
            j = 2;
            b2CircleShape s1(rng.uniformReal(low, high));
            b2CapsuleShape s2(rng.uniformReal(low, high), rng.uniformReal(low, high));
            double time1 = 0.0, time2 = 0.0, error = 0.0;
            timeCollect<b2CircleShape, b2CapsuleShape>(&s1, transforms1, &s2, transforms2, time1, time2, error);
            std::cout << "Circle-Capsule " << "GJK-Time " << time1 << " Bisection-Time " << time2 << " error " << error << std::endl;
            vtime1[i][j] += time1;
            vtime2[i][j] += time2;
            verror[i][j] += error;
        }

        {
            j = 3;
            b2CircleShape s1(rng.uniformReal(low, high));
            b2RectangleShape s2(rng.uniformReal(low, high), rng.uniformReal(low, high));
            double time1 = 0.0, time2 = 0.0, error = 0.0;
            timeCollect<b2CircleShape, b2RectangleShape>(&s1, transforms1, &s2, transforms2, time1, time2, error);
            std::cout << "Circle-Rectangle " << "GJK-Time " << time1 << " Bisection-Time " << time2 << " error " << error << std::endl;
            vtime1[i][j] += time1;
            vtime2[i][j] += time2;
            verror[i][j] += error;
        }

        std::cout << std::endl << std::endl;

        //==============================================================================
        // Ellipse
        i = 1;
        {
            j = 1;
            b2EllipseShape s1(rng.uniformReal(low, high), rng.uniformReal(low, high));
            b2EllipseShape s2(rng.uniformReal(low, high), rng.uniformReal(low, high));
            double time1 = 0.0, time2 = 0.0, error = 0.0;
            timeCollect<b2EllipseShape, b2EllipseShape>(&s1, transforms1, &s2, transforms2, time1, time2, error);
            std::cout << "Ellipse-Ellipse " << "GJK-Time " << time1 << " Bisection-Time " << time2 << " error " << error << std::endl;
            vtime1[i][j] += time1;
            vtime2[i][j] += time2;
            verror[i][j] += error;
        }
        
        {
            j = 2;
            b2EllipseShape s1(rng.uniformReal(low, high), rng.uniformReal(low, high));
            b2CapsuleShape s2(rng.uniformReal(low, high), rng.uniformReal(low, high));
            double time1 = 0.0, time2 = 0.0, error = 0.0;
            timeCollect<b2EllipseShape, b2CapsuleShape>(&s1, transforms1, &s2, transforms2, time1, time2, error);
            std::cout << "Ellipse-Capsule " << "GJK-Time " << time1 << " Bisection-Time " << time2 << " error " << error << std::endl;
            vtime1[i][j] += time1;
            vtime2[i][j] += time2;
            verror[i][j] += error;
        }

        {
            j = 3;
            b2EllipseShape s1(rng.uniformReal(low, high), rng.uniformReal(low, high));
            b2RectangleShape s2(rng.uniformReal(low, high), rng.uniformReal(low, high));
            double time1 = 0.0, time2 = 0.0, error = 0.0;
            timeCollect<b2EllipseShape, b2RectangleShape>(&s1, transforms1, &s2, transforms2, time1, time2, error);
            std::cout << "Ellipse-Rectangle " << "GJK-Time " << time1 << " Bisection-Time " << time2 << " error " << error << std::endl;
            vtime1[i][j] += time1;
            vtime2[i][j] += time2;
            verror[i][j] += error;
        }

        std::cout << std::endl << std::endl;

        //==============================================================================
        // Capsule
        i = 2;
        {
            j = 2;
            b2CapsuleShape s1(rng.uniformReal(low, high), rng.uniformReal(low, high));
            b2CapsuleShape s2(rng.uniformReal(low, high), rng.uniformReal(low, high));
            double time1 = 0.0, time2 = 0.0, error = 0.0;
            timeCollect<b2CapsuleShape, b2CapsuleShape>(&s1, transforms1, &s2, transforms2, time1, time2, error);
            std::cout << "Capsule-Capsule " << "GJK-Time " << time1 << " Bisection-Time " << time2 << " error " << error << std::endl;
            vtime1[i][j] += time1;
            vtime2[i][j] += time2;
            verror[i][j] += error;
        }

        {
            j = 3;
            b2CapsuleShape s1(rng.uniformReal(low, high), rng.uniformReal(low, high));
            b2RectangleShape s2(rng.uniformReal(low, high), rng.uniformReal(low, high));
            double time1 = 0.0, time2 = 0.0, error = 0.0;
            timeCollect<b2CapsuleShape, b2RectangleShape>(&s1, transforms1, &s2, transforms2, time1, time2, error);
            std::cout << "Capsule-Rectangle " << "GJK-Time " << time1 << " Bisection-Time " << time2 << " error " << error << std::endl;
            vtime1[i][j] += time1;
            vtime2[i][j] += time2;
            verror[i][j] += error;
        }

        std::cout << std::endl << std::endl;

        //==============================================================================
        // Rectangle
        i = 3;
        {
            j = 3;
            b2RectangleShape s1(rng.uniformReal(low, high), rng.uniformReal(low, high));
            b2RectangleShape s2(rng.uniformReal(low, high), rng.uniformReal(low, high));
            double time1 = 0.0, time2 = 0.0, error = 0.0;
            timeCollect<b2RectangleShape, b2RectangleShape>(&s1, transforms1, &s2, transforms2, time1, time2, error);
            std::cout << "Rectangle-Rectangle " << "GJK-Time " << time1 << " Bisection-Time " << time2 << " error " << error << std::endl;
            vtime1[i][j] += time1;
            vtime2[i][j] += time2;
            verror[i][j] += error;
        }

        std::cout << std::endl << std::endl;

        //==============================================================================
        // Polygon
        i = 4;
        std::cout << std::endl << std::endl;
    }

    std::cout << std::endl << std::endl;
    std::cout << "Total Circle-Circle " << "GJK-Time " << vtime1[0][0] << " Bisection-Time " << vtime2[0][0] << " error " << verror[0][0] << std::endl;
    std::cout << "Total Circle-Ellipse " << "GJK-Time " << vtime1[0][1] << " Bisection-Time " << vtime2[0][1] << " error " << verror[0][1] << std::endl;
    std::cout << "Total Ellipse-Ellipse " << "GJK-Time " << vtime1[1][1] << " Bisection-Time " << vtime2[1][1] << " error " << verror[1][1] << std::endl;
    std::cout << "Total Rectangle-Rectangle " << "GJK-Time " << vtime1[3][3] << " Bisection-Time " << vtime2[3][3] << " error " << verror[3][3] << std::endl;

    /*
    std::ofstream ofs1("vtime1.txt", std::ios::binary | std::ios::out);
    std::ofstream ofs2("vtime2.txt", std::ios::binary | std::ios::out);
    std::ofstream ofs3("verror.txt", std::ios::binary | std::ios::out);

    for (i = 0; i < num; i++)
    {
        for (j = 0; j < num; j++)
        {
            ofs1 << vtime1[i][j] << " ";
            ofs2 << vtime2[i][j] << " ";           
            ofs3 << verror[i][j] << " ";
        }
        ofs1 << std::endl;
        ofs2 << std::endl;
        ofs3 << std::endl;
    }
    ofs1.close();
    ofs2.close();
    ofs3.close();
    */

    return 0;
}

template<typename b2Shape1, typename b2Shape2>
void timeCollect(const b2Shape1 *s1, const std::vector<b2Transform> &transforms1,
                 const b2Shape2 *s2, const std::vector<b2Transform> &transforms2,
                 double &time1, double &time2, double &error)
{
    time1 = time2 = error = 0.0;
    b2ShapeDistance dist;
    dist.distance_tolerance = 1.e-6;
    for (std::size_t i = 0; i < transforms1.size(); i++)
    {
        for (std::size_t j = 0; j < transforms2.size(); j++)
        {
            double dist1, dist2;
            b2Vec2 p1, p2;

            bool res1 = dist.Distance(s1, transforms1[i], s2, transforms2[j], &dist1, &p1,  &p2);
            if (res1)
            {
                time::point start = time::now();
                res1 = dist.Distance(s1, transforms1[i], s2, transforms2[j], &dist1, &p1,  &p2);
                time1 += time::seconds(time::now() - start);

                start = time::now();
                bool res2 = dist.BisectionDistance(s1, transforms1[i], s2, transforms2[j], &dist2, &p1,  &p2);
                time2 += time::seconds(time::now() - start);               
                error += abs(dist1 - dist2);

                if (!res2)
                    OMPL_ERROR("The bisection distance function is error!");
            }
        }
    }
}
