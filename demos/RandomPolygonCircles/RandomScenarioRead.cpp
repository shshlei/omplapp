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
#include <ompl/util/Console.h>

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
    int count = 0; // polygon vertex count
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
            shape = eshape;
        }
        else if (type == 3) // capsule 
        {
            s >> x >> y >> yaw >> r >> h;
            b2CapsuleShape *cshape = new b2CapsuleShape(b2Scalar(r), b2Scalar(h));
            shape = cshape;
        }
        else if (type == 4) // rectangle 
        {
            s >> x >> y >> yaw >> a >> b;
            b2RectangleShape *rectshape = new b2RectangleShape(b2Scalar(a), b2Scalar(b));
            shape = rectshape;
        }

        if (shape != nullptr)
        {
            shapec++;
            manager.AddBody("shape" + std::to_string(shapec), shape, b2Transform::Identity(), false);
            if (!polygon)
            {
                xf.translation() = b2Vec2(b2Scalar(x), b2Scalar(y));
                xf.linear() = b2Rot(b2Scalar(yaw)).toRotationMatrix();
                manager.SetBodyTransform("shape" + std::to_string(shapec), xf);
            }
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
