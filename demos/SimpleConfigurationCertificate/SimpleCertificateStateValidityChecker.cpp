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

#include "SimpleCertificateStateValidityChecker.h"
#include <omplapp/geometry/geometries/Geometry.h>
#include <omplapp/geometry/geometries/Types.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>

#include <ompl/util/Console.h>

#include <thread>
#include <limits>
#include <algorithm>

using namespace ompl::app::collision;

namespace ompl
{
    namespace app
    {
        SimpleCertificateStateValidityChecker::SimpleCertificateStateValidityChecker(const ompl::base::SpaceInformationPtr& si, double contact_distance)
          : StateValidityChecker(si), contact_distance_(contact_distance + std::numeric_limits<double>::epsilon())
        {
            assert(si_->getStateSpace()->getType() == ompl::base::STATE_SPACE_REAL_VECTOR);
            assert(si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getDimension() == 2);
            collision_manager_ = std::make_shared<collision_box2d::Box2dDiscreteBVHManager>();
            //collision_manager_->setContactDistanceThreshold(0.5 * contact_distance);
        }

        bool SimpleCertificateStateValidityChecker::setEnvironmentFile(const std::string &env)
        {
            assert(collision_manager_->getBox2dBroadphse()->GetBodyCount() == 0);
            return read(env);
        }

        bool SimpleCertificateStateValidityChecker::setRobotShape(const collision::CollisionShapePtr& shape)
        {
            robotCount_ = 1;
            std::string name = "Robot" + std::to_string(robotCount_);
            if (collision_manager_->addCollisionObject(name, shape, Eigen::Isometry2d::Identity(), true))
            {
                names_ = name;
                return true;
            }
            robotCount_--;
            return false;
        }

        bool SimpleCertificateStateValidityChecker::isValid(const ompl::base::State *state) const
        {
            Eigen::Isometry2d tf = calculateCurrentTransform(state); 
            collision_manager_->setCollisionObjectsTransform(names_, tf);
            return !collision_manager_->contactTest();
        }

        bool SimpleCertificateStateValidityChecker::isValid(const ompl::base::State *state, double &dist) const
        {
            dist = clearance(state);
            return dist > 0.0;
        }

        double SimpleCertificateStateValidityChecker::clearance(const ompl::base::State *state) const
        {
            double distance = contact_distance_;
            Eigen::Isometry2d tf = calculateCurrentTransform(state); 
            collision_manager_->setCollisionObjectsTransform(names_, tf);
            collision_manager_->distanceTest(distance);
            return distance;
        }

        Eigen::Isometry2d SimpleCertificateStateValidityChecker::calculateCurrentTransform(const ompl::base::State *state) const
        {
            Eigen::Isometry2d link_transform = Eigen::Isometry2d::Identity();
            const auto* rvstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
            link_transform.pretranslate(Eigen::Vector2d(rvstate->values[0], rvstate->values[1]));           
            return link_transform;
        }

        boost::filesystem::path SimpleCertificateStateValidityChecker::findMeshFile(const std::string& fname)
        {
            boost::filesystem::path path(fname);
            if (boost::filesystem::exists(path))
                return boost::filesystem::absolute(path);
            if (path.is_absolute())
                return {};
            for (const auto &dir : meshPath_)
            {
                boost::filesystem::path candidate(dir / path);
                if (boost::filesystem::exists(candidate))
                    return boost::filesystem::absolute(candidate);
            }
            return {};
        }

        bool SimpleCertificateStateValidityChecker::read(const std::string& filename)
        {
            const boost::filesystem::path path = findMeshFile(filename);
            if (path.empty())
            {
                OMPL_ERROR("File '%s' not found in mesh path.", filename.c_str());
                return false;
            }

            std::ifstream file(path.string().c_str(), std::ios_base::binary | std::ios_base::in);
            if (!file.is_open())
            {
                OMPL_ERROR("Filestream to %s is not open, nothing read.", filename.c_str());
                return false;
            }

            return read(file);
        }

        bool SimpleCertificateStateValidityChecker::read(std::istream &s)
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
                std::string name = "";
                if (type == 0)
                {
                    s >> x >> y >> r;
                    yaw = 0.0;
                    b2CircleShape *cshape = new b2CircleShape();
                    cshape->SetRadius(b2Scalar(r));
                    xf.translation() = b2Vec2(b2Scalar(x), b2Scalar(y));
                    xf.linear() = b2Rot(b2Scalar(yaw)).toRotationMatrix();
                    shape = cshape;
                    shapec++;
                    name = "Circle" + std::to_string(shapec);
                }
                else if (type == 1) 
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
                    shapec++;
                    name = "Polygon" + std::to_string(shapec);
                }
                else if (type == 2) // ellipse
                {
                    s >> x >> y >> yaw >> a >> b;
                    b2EllipseShape *eshape = new b2EllipseShape(b2Scalar(a), b2Scalar(b));
                    xf.translation() = b2Vec2(b2Scalar(x), b2Scalar(y));
                    xf.linear() = b2Rot(b2Scalar(yaw)).toRotationMatrix();
                    shape = eshape;
                    shapec++;
                    name = "Ellipse" + std::to_string(shapec);
                }
                else if (type == 3) // capsule 
                {
                    s >> x >> y >> yaw >> r >> h;
                    b2CapsuleShape *cshape = new b2CapsuleShape(b2Scalar(r), b2Scalar(h));
                    xf.translation() = b2Vec2(b2Scalar(x), b2Scalar(y));
                    xf.linear() = b2Rot(b2Scalar(yaw)).toRotationMatrix();
                    shape = cshape;
                    shapec++;
                    name = "Capsule" + std::to_string(shapec);
                }
                else if (type == 4) // rectangle 
                {
                    s >> x >> y >> yaw >> a >> b;
                    b2RectangleShape *rectshape = new b2RectangleShape(b2Scalar(a), b2Scalar(b));
                    xf.translation() = b2Vec2(b2Scalar(x), b2Scalar(y));
                    xf.linear() = b2Rot(b2Scalar(yaw)).toRotationMatrix();
                    shape = rectshape;
                    shapec++;
                    name = "Capsule" + std::to_string(shapec);
                }

                if (shape != nullptr)
                {
                    collision_manager_->getBox2dBroadphse()->AddBody(name, shape, b2Transform::Identity(), false);
                    if (!polygon)
                        collision_manager_->getBox2dBroadphse()->SetBodyTransform(name, xf);
                    delete shape;
                }
            }

            if (numbers != collision_manager_->getBox2dBroadphse()->GetBodyCount())
                OMPL_WARN("The shape number is wrong in this file, %d VS %d!", numbers, collision_manager_->getBox2dBroadphse()->GetBodyCount());

            return true;
        }

        bool SimpleCertificateStateValidityChecker::readHeader(std::istream& s, int& numbers)
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
    }
}
