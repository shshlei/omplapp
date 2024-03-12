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

#include "DrivingPureSampling.h"

#include <box2d_collision/b2_math.h>
#include <box2d_collision/b2_distance.h>
#include <box2d_collision/b2_bvh_manager.h>

#include <bomp/utils.hpp>
#include <bomp/collision_constraints/J_function_collision_constraints.h>
#include <bomp/collision_constraints/active_points_collision_constraints.h>

class APCallback : public b2NaiveCallback
{
public:
    APCallback(double activeDist, const b2RectangleShape * rect, const b2OBB * obb) : b2NaiveCallback()
    {
        activeDist_ = minDist_ = activeDist;

        rect_ = rect;
        hsides_= rect_->GetHalfSides();

        xf_.setIdentity();
        xf_.linear() = obb->axis();
        xf_.translation() = obb->center();

        invxf_.setIdentity();
        invxf_.linear() = obb->axis().transpose();
        invxf_.translation() = -invxf_.linear() * obb->center();
    }

    ~APCallback() override = default;

    bool ReportCollision(b2Fixture* fixture) override
    {
        if (fixture->GetBody()->IsActive()) return false;
        b2Scalar d;
        b2Vec2 p1, p2;
        dist_.SignedDistance(rect_, fixture->GetShape(), invxf_ * fixture->GetGlobalTransform(), &d, &p1, &p2);
        if (d >= activeDist_) return false;
        b2Vec2 normal = p2 - p1;
        p2 = xf_ * p2;
        if (d < minDist_) minDist_ = d;
        if (d < 0.0) normal = -normal;
        if (d < 0.0) error_ = true;

        // without principal normal
        // if (abs(normal(0)) < 2.0 * abs(normal(1)) && abs(normal(1)) < 2.0 * abs(normal(0))) continue;
        
        if (d > 0.0)
        {
            if (abs(normal(0)) > 0.5 * abs(normal(1)))
            {
                psopt::ActivePoint<double> apoint;
                apoint.Ba.push_back(p2);
                if (normal(0) > 0.0)
                {
                    apoint.Aa.push_back(b2Vec2(hsides_[0], -hsides_[1]));
                    apoint.Aa.push_back(b2Vec2(hsides_[0], hsides_[1]));
                }
                else
                {
                    apoint.Aa.push_back(b2Vec2(-hsides_[0], hsides_[1]));
                    apoint.Aa.push_back(b2Vec2(-hsides_[0], -hsides_[1]));
                }
                activePoints_.push_back(apoint);
            }
            if (abs(normal(1)) > 0.5 * abs(normal(0)))
            {
                psopt::ActivePoint<double> apoint;
                apoint.Ba.push_back(p2);
                if (normal(1) > 0.0)
                {
                    apoint.Aa.push_back(b2Vec2(hsides_[0], hsides_[1]));
                    apoint.Aa.push_back(b2Vec2(-hsides_[0], hsides_[1]));
                }
                else
                {
                    apoint.Aa.push_back(b2Vec2(-hsides_[0], -hsides_[1]));
                    apoint.Aa.push_back(b2Vec2(hsides_[0], -hsides_[1]));
                }
                activePoints_.push_back(apoint);
            }
        }
        else
        {
            if (abs(normal(0)) > 0.1 * abs(normal(1)))
            {
                psopt::ActivePoint<double> apoint;
                apoint.Ba.push_back(p2);
                if (normal(0) > 0.0)
                {
                    apoint.Aa.push_back(b2Vec2(hsides_[0], -hsides_[1]));
                    apoint.Aa.push_back(b2Vec2(hsides_[0], hsides_[1]));
                }
                else
                {
                    apoint.Aa.push_back(b2Vec2(-hsides_[0], hsides_[1]));
                    apoint.Aa.push_back(b2Vec2(-hsides_[0], -hsides_[1]));
                }
                activePoints_.push_back(apoint);
            }
            if (abs(normal(1)) > 0.1 * abs(normal(0)))
            {
                psopt::ActivePoint<double> apoint;
                apoint.Ba.push_back(p2);
                if (normal(1) > 0.0)
                {
                    apoint.Aa.push_back(b2Vec2(hsides_[0], hsides_[1]));
                    apoint.Aa.push_back(b2Vec2(-hsides_[0], hsides_[1]));
                }
                else
                {
                    apoint.Aa.push_back(b2Vec2(-hsides_[0], -hsides_[1]));
                    apoint.Aa.push_back(b2Vec2(hsides_[0], -hsides_[1]));
                }
                activePoints_.push_back(apoint);
            }
        }
        return false; // Never stop early
    }

    double getError() const
    {
        return error_;
    }

    double getMinDist() const
    {
        return minDist_;
    }

    std::vector<psopt::ActivePoint<double>> & getActivePoints()
    {
        return activePoints_;
    }

private:

    bool error_{false};

    double activeDist_, minDist_;

    // Distance Calculation
    b2ShapeDistance dist_;

    // Local vehicle rectangle
    const b2RectangleShape* rect_;
    b2Vec2 hsides_;

    // Vehicle Global Transform
    b2Transform xf_, invxf_;

    std::vector<psopt::ActivePoint<double>> activePoints_;
};

int main(int argc, char* argv[])
{
    std::shared_ptr<app::Box2dStateValidityChecker> svc;
    double x0, y0, theta0, xf, yf, thetaf;
    std::vector<double> trajx, trajy, trajt;
    double timeUsed;
    int save_num;
    if (!solve(argc, argv, svc, x0, y0, theta0, xf, yf, thetaf, trajx, trajy, trajt, timeUsed, save_num)) return -1;
    std::size_t expected_nnodes = trajx.size();

    std::ofstream ofs;
    ofs.open("pathsol_interp.txt");
    for (std::size_t i = 0; i < expected_nnodes; i++)
    {
        ofs << trajx[i] << " " << trajy[i] << " " << trajt[i] << std::endl;
    }
    ofs.close();

    // vehicle param
    VehicleParam<double> *vehicleParam = new VehicleParam<double>(0.028, 0.0096, 0.00929, 0.01942);
    double delta = vehicleParam->delta;

    // local vehicle rectangle
    b2RectangleShape * rect = new b2RectangleShape(0.5 * vehicleParam->L, 0.5 * vehicleParam->W);

    // obb extent
    double active_dist = 0.020;
    b2Vec2 extent(0.5 * vehicleParam->L + active_dist, 0.5 * vehicleParam->W + active_dist);

    ofs.open("active_points.txt");
    for (std::size_t i = 0; i < expected_nnodes; i++)
    {
        double x = trajx[i];
        double y = trajy[i];
        double yaw = trajt[i];

        b2Mat22 axis = b2Rot(yaw).toRotationMatrix();
        b2Vec2 center(x, y);
        center += delta * axis.col(0);
        b2OBB obb(axis, center, extent);
        APCallback callback(active_dist, rect, &obb);
        svc->getBox2dDiscreteBVHManager()->getBox2dBroadphse()->Collide(&callback, obb);
        if (callback.getError())
        {
            std::cout << "Error!" << std::endl;
            std::cout << x << " " << y << " " << yaw << " " << std::endl;             
        }

        for (const psopt::ActivePoint<double> & apoint : callback.getActivePoints())
        {
            for (const Eigen::Matrix<double, 2, 1> & p : apoint.Aa)
            {
                Eigen::Matrix<double, 2, 1> wp = axis * p + center;
                ofs << i << " " << wp(0) << " " << wp(1) << std::endl;
            }
            for (const Eigen::Matrix<double, 2, 1> & p : apoint.Ba)
            {
                ofs << i << " " << p(0) << " " << p(1) << std::endl;
            }
        }
    }
    ofs.close();

    delete rect;
    delete vehicleParam;
    rect = nullptr;
    vehicleParam = nullptr;

    return 0;
}
