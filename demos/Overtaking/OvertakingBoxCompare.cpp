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

#include "OvertakingStageOne.h" 

#include <box2d_collision/b2_distance.h>
#include <box2d_collision/b2_bvh_manager.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <bitset>

// two-circle approximation of the vehicle
template <typename Scalar = double>
struct VehicleCircleParam
{
    Scalar rx, fx, radius;
    VehicleCircleParam(const VehicleParam<Scalar> * vehicle)
    {
        rx = 0.25 * vehicle->L - vehicle->l2;
        fx = 0.75 * vehicle->L - vehicle->l2;
        radius = std::hypot(0.25 * vehicle->L, 0.5 * vehicle->W);
    }
};

// check if an aabb shape collide with the envrionment 
class CollisionCallback : public b2NaiveCallback
{
public:
    CollisionCallback(const b2AABB & taabb) : b2NaiveCallback()
    {
        xf_.setIdentity();
        xf_.translation() = taabb.center();

        b2Vec2 hsizes = 0.5 * taabb.sizes();
        shape_.Set(hsizes.x(), hsizes.y());
    }

    ~CollisionCallback() override = default;

    bool ReportCollision(b2Fixture* fixture) override
    {
        if (fixture->GetBody()->IsActive()) return false;
        if (b2CollideShapes(&shape_, xf_, fixture->GetShape(), fixture->GetGlobalTransform()))
        {
            collision_ = true;
            return true;
        }
        return false; // Never Stop
    }

    bool getCollison() const
    {
        return collision_;
    }

private:

    bool collision_{false};

    b2Transform xf_;

    b2RectangleShape shape_;
};

// circle x, y
// local expanded aabb
bool checkCollision(const std::shared_ptr<b2BVHManager> & svc, double x, double y, const b2AABB & aabb)
{
    b2AABB taabb = aabb.translated(b2Vec2(x, y));
    CollisionCallback callback(taabb);
    svc->Collide(&callback, taabb);
    return callback.getCollison();
}

// circle center
// local circle caabb
// global aabb result
bool generateBox(const std::shared_ptr<b2BVHManager> & svc, const b2Vec2 & center, const b2AABB & caabb, int corridor_max_iter, double corridor_incremental_limit, b2AABB & result)
{
    int inc = 4;
    double real_x = center.x(), real_y = center.y();
    while (checkCollision(svc, real_x, real_y, caabb) && inc < corridor_max_iter)
    {
        // initial condition not satisfied, involute to find feasible box
        int iter = inc / 4;
        uint8_t edge = inc % 4;
        inc++;

        real_x = center.x(), real_y = center.y();
        if (edge == 0)
        {
            real_x -= iter * 0.05;
        }
        else if (edge == 1)
        {
            real_x += iter * 0.05;
        }
        else if (edge == 2)
        {
            real_y -= iter * 0.05;
        }
        else if (edge == 3)
        {
            real_y += iter * 0.05;
        }
    }
    if (inc >= corridor_max_iter) return false;

    inc = 4;
    std::bitset<4> blocked;
    double incremental[4] = {0.0};
    double step = 0.1 * caabb.max().x();
    while (!blocked.all() && inc < corridor_max_iter)
    {
        int iter = inc / 4;
        uint8_t edge = inc % 4;
        inc++;
        if (blocked[edge]) continue;

        incremental[edge] = iter * step;
        b2AABB aabb = caabb;
        aabb.min().x() -= incremental[0];
        aabb.max().x() += incremental[1];
        aabb.min().y() -= incremental[2];
        aabb.max().y() += incremental[3];
        if (checkCollision(svc, real_x, real_y, aabb) || incremental[edge] >= corridor_incremental_limit)
        {
            incremental[edge] -= step;
            blocked[edge] = true;
        }
    }
    if (inc >= corridor_max_iter) return false;

    result.min().x() = real_x - incremental[0];
    result.max().x() = real_x + incremental[1];
    result.min().y() = real_y - incremental[2];
    result.max().y() = real_y + incremental[3];
    return true;
}

template <typename Scalar = double, typename Scalar2 = Scalar>
class OvertakingProblemBox : public OvertakingProblemBase<Scalar, Scalar2>
{
public:

    OvertakingProblemBox(psopt::ProblemInfo<Scalar2>* prob, const VehicleParam<Scalar2>* vehicleParam, const VehicleCircleParam<Scalar2>* vehicleCircleParam) : OvertakingProblemBase<Scalar, Scalar2>(prob, vehicleParam)
    {
        vehicleCircleParam_ = vehicleCircleParam;
    }

    virtual ~OvertakingProblemBox() = default;

    psopt::Problem<adouble, Scalar2>* clone() const override
    {
        OvertakingProblemBox<adouble, Scalar2>* prob = new OvertakingProblemBox<adouble, Scalar2>(this->problemInfo_, this->vehicleParam_, this->vehicleCircleParam_);
        prob->setLinearizedParameters(this);
        return prob;
    }

    void path(Scalar* paths, const Scalar* states, const Scalar* /*parameters*/,
        const Scalar* /*x*/, std::size_t iphase) const override
    {
        std::size_t offset = 0;
        std::size_t nnodes = this->problemInfo_->getPhaseNumberNodes(iphase);
        std::size_t nstates = this->problemInfo_->getPhaseNumberStates(iphase);
        const Scalar* cstates = states + nstates;
        for (std::size_t i = 1; i < nnodes; i++)
        {
            const Scalar x = cstates[0];
            const Scalar y = cstates[1];
            const Scalar theta = cstates[2];
            cstates += nstates;

            Scalar c = cos(theta);
            Scalar s = sin(theta);
            paths[offset] = x + vehicleCircleParam_->rx * c;
            offset++;
            paths[offset] = y + vehicleCircleParam_->rx * s;
            offset++;
            paths[offset] = x + vehicleCircleParam_->fx * c;
            offset++;
            paths[offset] = y + vehicleCircleParam_->fx * s;
            offset++;
        }
    }

protected:
    const VehicleCircleParam<Scalar2>* vehicleCircleParam_;
};

class APCallback : public b2NaiveCallback
{
public:
    APCallback(double activeDist, const b2RectangleShape * rect, const b2OBB * obb) : b2NaiveCallback()
    {
        activeDist_ = minDist_ = activeDist;

        rect_ = rect;

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
        if (d < minDist_) minDist_ = d;

        b2Vec2 normal = p2 - p1;
        if (d < 0.0) normal = -normal;
        if (abs(d) < 1.e-6) normal = p1;

        b2Vec2 tau = b2Cross(1.0, normal).normalized();
        psopt::ActivePoint2D<double> apoint;
        apoint.normal = 1000.0 * normal.normalized(); // TODO
        apoint.b = xf_ * p2;
        b2Vec2 a1 = p1 - tau + apoint.normal;
        b2Vec2 a2 = p1 + tau + apoint.normal;
        apoint.a = a2 - a1;
        apoint.det = a1.x() * a2.y() - a1.y() * a2.x();
        activePoints_.push_back(apoint);

        return false; // Never stop early
    }

    double getMinDist() const
    {
        return minDist_;
    }

    std::vector<psopt::ActivePoint2D<double>> & getActivePoints()
    {
        return activePoints_;
    }

private:

    double activeDist_, minDist_;

    // Distance Calculation
    b2ShapeDistance dist_;

    // Local vehicle rectangle
    const b2RectangleShape* rect_;

    // Vehicle Global Transform
    b2Transform xf_, invxf_;

    std::vector<psopt::ActivePoint2D<double>> activePoints_;
};

template <typename Scalar = double, typename Scalar2 = Scalar>
class OvertakingProblemBoxAp : public OvertakingProblemBase<Scalar, Scalar2>
{
public:

    OvertakingProblemBoxAp(psopt::ProblemInfo<Scalar2>* prob, const VehicleParam<Scalar2>* vehicleParam) : OvertakingProblemBase<Scalar, Scalar2>(prob, vehicleParam)
    {
    }

    virtual ~OvertakingProblemBoxAp() = default;

    psopt::Problem<adouble, Scalar2>* clone() const override
    {
        OvertakingProblemBoxAp<adouble, Scalar2>* prob = new OvertakingProblemBoxAp<adouble, Scalar2>(this->problemInfo_, this->vehicleParam_);
        prob->setActivePoints(activePoints_);
        prob->setLinearizedParameters(this);
        return prob;
    }

    void setActivePoints(const std::vector<std::vector<psopt::ActivePoints2D<Scalar2>>> & activePoints)
    {
        activePoints_ = activePoints;
    }

    void path(Scalar* paths, const Scalar* states, const Scalar* /*parameters*/,
        const Scalar* /*x*/, std::size_t iphase) const override
    {
        std::size_t offset = 0;
        std::size_t nnodes = this->problemInfo_->getPhaseNumberNodes(iphase);
        std::size_t nstates = this->problemInfo_->getPhaseNumberStates(iphase);
        const Scalar* cstates = states + nstates;
        for (std::size_t i = 1; i < nnodes; i++)
        {
            const Scalar x = cstates[0];
            const Scalar y = cstates[1];
            cstates += nstates;

            paths[offset] = x;
            offset++;
            paths[offset] = y;
            offset++;
        }
        
        const std::vector<psopt::ActivePoints2D<Scalar2>> activePoints = activePoints_[iphase];
        for (const psopt::ActivePoints2D<Scalar2> & apoints : activePoints)
        {
            const Scalar * cstates = states + apoints.index * nstates;
            const Scalar x = cstates[0];
            const Scalar y = cstates[1];
            const Scalar theta = cstates[2];
            Eigen::Matrix<Scalar, 2, 3> invtransform = invtransform_2D(x, y, theta);
            invtransform(0, 2) -= this->vehicleParam_->delta;
            MJ_2_Active_Constraints_2D(paths + offset, invtransform, apoints.activePoints);
            offset += apoints.activePoints.size();
        }
    }

private:

    std::vector<std::vector<psopt::ActivePoints2D<Scalar2>>> activePoints_;
};

int main(int argc, char* argv[])
{
    std::vector<double> time1, time2, time3, costs1, costs2;
    double best_cost = std::numeric_limits<double>::max();
    for (std::size_t iteration = 0; iteration < 100;)
    {
        std::cout << "Iteration " << iteration << std::endl;
        psopt::MultiSegmentData msdata;
        psopt::Solver<double> solver;
        double stage1_time;
        if (!solve(argc, argv, msdata, solver, stage1_time)) continue;

        bool success = true;
        // box constraints
        {
            ompl::time::point timeStart = ompl::time::now();

            psopt::ProblemInfo<double> * info = new psopt::ProblemInfo<double>(msdata);
            info->setLinearSolver("ma57");
            info->setTolerance(1.e-8);
            info->setPathsAlongTrajectory(false);
            info->setParametersAlongTrajectory(false);

            overtaking_bounds(info);
            info->setPhaseLowerBoundStartTime(0.0);
            info->setPhaseUpperBoundStartTime(0.0);
            info->setPhaseLowerBoundEndTime(0.0);
            info->setPhaseUpperBoundEndTime(10.0);
            for (std::size_t i = 1; i < msdata.nsegments; i++)
            {
                info->setPhaseLowerBoundStartTime(0.0, i);
                info->setPhaseUpperBoundStartTime(10.0, i);
                info->setPhaseLowerBoundEndTime(0.0, i);
                info->setPhaseUpperBoundEndTime(10.0, i);
            }
            if (overtaking_case == 2)
            {
                info->setPhaseLowerBoundEndTime(3.0, msdata.nsegments - 1);
                info->setPhaseUpperBoundEndTime(3.0, msdata.nsegments - 1);
            }
            else if (overtaking_case == 1)
            {
                info->setPhaseLowerBoundEndTime(2.0, msdata.nsegments - 1);
                info->setPhaseUpperBoundEndTime(2.0, msdata.nsegments - 1);
            }
            else
            {
                info->setPhaseLowerBoundEndTime(1.5, msdata.nsegments - 1);
                info->setPhaseUpperBoundEndTime(1.5, msdata.nsegments - 1);
            }

            for (std::size_t j = 0; j < msdata.nsegments; j++)
            {
                info->setPhaseNumberParameters(0, j);
                info->setPhaseNumberPaths(0, j);
                info->setPhaseGuessStates(solver.getPhaseStates(j), j);
                info->setPhaseGuessControls(solver.getPhaseControls(j), j);
                info->setPhaseGuessTime(solver.getPhaseTime(j), j);
            }

            VehicleParam<double> *vehicleParam = new VehicleParam<double>(2.8, 0.96, 0.929, 1.942);
            VehicleCircleParam<double> *vehicleCircleParam = new VehicleCircleParam<double>(vehicleParam);
            b2AABB caabb(b2Vec2(-vehicleCircleParam->radius, -vehicleCircleParam->radius), b2Vec2(vehicleCircleParam->radius, vehicleCircleParam->radius));
            OvertakingProblemBox<double>* problem = new OvertakingProblemBox<double>(info, vehicleParam, vehicleCircleParam);

            // second stage box constraints
            // info->setUseLinearizedDae(true); // TODO
            // problem->setUpLinearizedParameters(info->getVariables().data());
            std::shared_ptr<b2BVHManager> svc = std::make_shared<b2BVHManager>();
            b2RectangleShape * rect = new b2RectangleShape(0.5 * vehicleParam->L, 0.5 * vehicleParam->W);
            std::string obs_name1 = "obstacle1", obs_name2 = "obstacle2";
            svc->AddBody(obs_name1, rect, b2Transform::Identity(), false);
            if (overtaking_case == 2) svc->AddBody(obs_name2, rect, b2Transform::Identity(), false);
            else if (overtaking_case == 0)
            {
                b2Transform oxf = b2Transform::Identity();
                oxf.translation().x() = 10.0 + vehicleParam->delta; // overtaking_case == 0
                svc->SetBodyTransform(obs_name1, oxf);
            }

            int sol = 0;
            bool use_linearized_system = false;
            double dynamics_cost_weight = 10.0;
            best_cost = std::numeric_limits<double>::max();
            while (success && sol < 10)
            {
                sol++;
                for (std::size_t k = 0; k < msdata.nsegments; k++)
                {
                    // get box bounds 
                    std::vector<double> lower_box_bounds, upper_box_bounds;
                    lower_box_bounds.reserve(4 * msdata.nnodes[k]);
                    upper_box_bounds.reserve(4 * msdata.nnodes[k]);
                    const double * time = solver.getPhaseTime(k).data();
                    const double * states = solver.getPhaseStates(k).data() + msdata.nstates;
                    for (std::size_t i = 1; i < msdata.nnodes[k]; i++)
                    {
                        double x = states[0];
                        double y = states[1];
                        double yaw = states[2];
                        states += msdata.nstates;

                        if (overtaking_case > 0)
                        {
                            b2Transform oxf = b2Transform::Identity();
                            oxf.translation().x() = 10.0 + vehicleParam->delta + 10.0 * time[i]; // overtaking_case 1 
                            svc->SetBodyTransform(obs_name1, oxf);
                            if (overtaking_case == 2)
                            {
                                oxf.translation().x() += 10.0; // overtaking_case 2 
                                svc->SetBodyTransform(obs_name2, oxf);
                            }
                        }

                        b2Mat22 axis = b2Rot(yaw).toRotationMatrix();
                        b2Vec2 center(x, y);
                        b2Vec2 rear = center + vehicleCircleParam->rx * axis.col(0);
                        b2Vec2 front = center + vehicleCircleParam->fx * axis.col(0);

                        b2AABB result1, result2;
                        if (!generateBox(svc, rear, caabb, 1000, 5.0, result1))
                        {
                            success = false;
                            OMPL_ERROR("Failed to generate box constraints!");
                            break;
                        }

                        if (!generateBox(svc, front, caabb, 1000, 5.0, result2))
                        {
                            success = false;
                            OMPL_ERROR("Failed to generate box constraints!");
                            break;
                        }

                        lower_box_bounds.push_back(result1.min().x());
                        lower_box_bounds.push_back(result1.min().y());
                        lower_box_bounds.push_back(result2.min().x());
                        lower_box_bounds.push_back(result2.min().y());

                        upper_box_bounds.push_back(result1.max().x());
                        upper_box_bounds.push_back(result1.max().y());
                        upper_box_bounds.push_back(result2.max().x());
                        upper_box_bounds.push_back(result2.max().y());
                    }
                    if (!success) break;

                    info->setPhaseNumberPaths(lower_box_bounds.size(), k);
                    info->setPhaseLowerBoundPaths(lower_box_bounds, k);
                    info->setPhaseUpperBoundPaths(upper_box_bounds, k);
                }
                if (!success) break;

                // solve with first sampling guess
                if (msdata.treat_dynamics_as_cost) info->setDynamicsCostWeight(dynamics_cost_weight);
                if (use_linearized_system)
                {
                    info->setUseLinearizedDae(true);
                    problem->setUpLinearizedParameters(info->getVariables().data());
                    success = solver.solve(problem);
                    if (!success) break;
                    info->setUseLinearizedDae(false);
                }
                success = solver.solve(problem);
                if (!success) break;

                if (msdata.treat_dynamics_as_cost)
                {
                    OMPL_INFORM("Current cost %0.4f, current infeasiblity %0.4f, current w_penalty %0.4f", solver.getCost(), info->getDynamicsCost(), dynamics_cost_weight);
                    if (info->getDynamicsCost() < 1.e-4)
                    {
                        OMPL_INFORM("Found the best trajectory!");
                        break;
                    }
                    dynamics_cost_weight *= 10.0;
                }
                else
                    OMPL_INFORM("Current cost %0.4f", solver.getCost());
                if (std::abs(best_cost - solver.getCost()) < 1.e-4 * std::abs(best_cost))
                {
                    OMPL_INFORM("Found the best trajectory!");
                    break;
                }
                if (solver.getCost() <= best_cost)
                {
                    best_cost = solver.getCost();
                }
                // use_linearized_system = true; // TODO
                // info->setUseLinearizedDae(true);
                // problem->setUpLinearizedParameters(info->getVariables().data());
            }

            if (success)
            {
                double opti_time = ompl::time::seconds(ompl::time::now() - timeStart);
                time1.push_back(stage1_time);
                time2.push_back(opti_time);
                costs1.push_back(best_cost);
            }

            delete rect;
            delete info;
            delete vehicleParam;
            delete vehicleCircleParam;
            delete problem;
            rect = nullptr;
            info = nullptr;
            vehicleParam = nullptr;
            vehicleCircleParam = nullptr;
            problem = nullptr;
        }

        // box ap constraints
        if (success)
        {
            double box_best_cost = best_cost;

            ompl::time::point timeStart = ompl::time::now();

            psopt::ProblemInfo<double> * info = new psopt::ProblemInfo<double>(msdata);
            info->setLinearSolver("ma57");
            info->setTolerance(1.e-8);
            info->setPathsAlongTrajectory(false);
            info->setParametersAlongTrajectory(false);

            overtaking_bounds(info);
            info->setPhaseLowerBoundStartTime(0.0);
            info->setPhaseUpperBoundStartTime(0.0);
            info->setPhaseLowerBoundEndTime(0.0);
            info->setPhaseUpperBoundEndTime(10.0);
            for (std::size_t i = 1; i < msdata.nsegments; i++)
            {
                info->setPhaseLowerBoundStartTime(0.0, i);
                info->setPhaseUpperBoundStartTime(10.0, i);
                info->setPhaseLowerBoundEndTime(0.0, i);
                info->setPhaseUpperBoundEndTime(10.0, i);
            }
            if (overtaking_case == 2)
            {
                info->setPhaseLowerBoundEndTime(3.0, msdata.nsegments - 1);
                info->setPhaseUpperBoundEndTime(3.0, msdata.nsegments - 1);
            }
            else if (overtaking_case == 1)
            {
                info->setPhaseLowerBoundEndTime(2.0, msdata.nsegments - 1);
                info->setPhaseUpperBoundEndTime(2.0, msdata.nsegments - 1);
            }
            else
            {
                info->setPhaseLowerBoundEndTime(1.5, msdata.nsegments - 1);
                info->setPhaseUpperBoundEndTime(1.5, msdata.nsegments - 1);
            }

            for (std::size_t j = 0; j < msdata.nsegments; j++)
            {
                info->setPhaseNumberParameters(0, j);
                info->setPhaseNumberPaths(0, j);
                info->setPhaseGuessStates(solver.getPhaseStates(j), j);
                info->setPhaseGuessControls(solver.getPhaseControls(j), j);
                info->setPhaseGuessTime(solver.getPhaseTime(j), j);
            }

            VehicleParam<double> *vehicleParam = new VehicleParam<double>(2.8, 0.96, 0.929, 1.942);

            // second stage box constraints
            // info->setUseLinearizedDae(true); // TODO
            // problem->setUpLinearizedParameters(info->getVariables().data());
            std::shared_ptr<b2BVHManager> svc = std::make_shared<b2BVHManager>();
            // local vehicle rectangle
            b2RectangleShape * rect = new b2RectangleShape(0.5 * vehicleParam->L, 0.5 * vehicleParam->W);
            std::string obs_name1 = "obstacle1", obs_name2 = "obstacle2";
            svc->AddBody(obs_name1, rect, b2Transform::Identity(), false);
            if (overtaking_case == 2) svc->AddBody(obs_name2, rect, b2Transform::Identity(), false);
            else if (overtaking_case == 0)
            {
                b2Transform oxf = b2Transform::Identity();
                oxf.translation().x() = 10.0 + vehicleParam->delta; // overtaking_case == 0
                svc->SetBodyTransform(obs_name1, oxf);
            }

            // obb extent
            double active_dist = 5.0;
            b2Vec2 extent(0.5 * vehicleParam->L + active_dist, 0.5 * vehicleParam->W + active_dist);
            b2Vec2 iextent(0.5 * vehicleParam->L, 0.5 * vehicleParam->W);

            int sol = 0;
            bool use_linearized_system = false;
            double dynamics_cost_weight = 10.0;
            best_cost = std::numeric_limits<double>::max();
            OvertakingProblemBoxAp<double>* problem = new OvertakingProblemBoxAp<double>(info, vehicleParam);
            while (success && sol < 10)
            {
                sol++;
                // box and ap constraints
                std::vector<std::vector<psopt::ActivePoints2D<double>>> phaseActivePoints; 
                phaseActivePoints.reserve(msdata.nsegments);
                for (std::size_t k = 0; k < msdata.nsegments; k++)
                {
                    // get active points
                    std::size_t nactive = 0, nactive_state = 0;
                    std::vector<double> lower_box_bounds, upper_box_bounds;
                    lower_box_bounds.reserve(2 * msdata.nnodes[k]);
                    upper_box_bounds.reserve(2 * msdata.nnodes[k]);
                    std::vector<psopt::ActivePoints2D<double>> activePoints; 
                    activePoints.reserve(msdata.nnodes[k]);
                    const double * time = solver.getPhaseTime(k).data();
                    const double * states = solver.getPhaseStates(k).data() + msdata.nstates;
                    for (std::size_t i = 1; i < msdata.nnodes[k]; i++)
                    {
                        double x = states[0];
                        double y = states[1];
                        double yaw = states[2];
                        states += msdata.nstates;

                        if (overtaking_case > 0)
                        {
                            b2Transform oxf = b2Transform::Identity();
                            oxf.translation().x() = 10.0 + vehicleParam->delta + 10.0 * time[i]; // overtaking_case 1 
                            svc->SetBodyTransform(obs_name1, oxf);
                            if (overtaking_case == 2)
                            {
                                oxf.translation().x() += 10.0; // overtaking_case 2 
                                svc->SetBodyTransform(obs_name2, oxf);
                            }
                        }

                        b2Mat22 axis = b2Rot(yaw).toRotationMatrix();
                        b2Vec2 center(x, y);
                        center += vehicleParam->delta * axis.col(0); // local vehicle center
                        b2OBB obb(axis, center, extent);
                        APCallback callback(active_dist, rect, &obb);
                        svc->Collide(&callback, obb);
                        double bound = active_dist;
                        if (callback.getMinDist() < active_dist) bound = std::max(callback.getMinDist(), 0.5 * active_dist); // TODO
                        lower_box_bounds.push_back(x - bound);
                        lower_box_bounds.push_back(y - bound);
                        upper_box_bounds.push_back(x + bound);
                        upper_box_bounds.push_back(y + bound);
                    
                        if (callback.getActivePoints().empty()) continue;
                        psopt::ActivePoints2D<double> apoints; 
                        apoints.index = i;
                        apoints.activePoints.swap(callback.getActivePoints());
                        activePoints.push_back(apoints);
                        nactive_state++;
                        nactive += apoints.activePoints.size();
                    }

                    info->setPhaseNumberPaths(lower_box_bounds.size() + nactive, k);
                    for (std::size_t i = 0; i < nactive; i++)
                    {
                        lower_box_bounds.push_back(1.0005); 
                        upper_box_bounds.push_back(std::numeric_limits<double>::infinity());
                    }
                    info->setPhaseLowerBoundPaths(lower_box_bounds, k);
                    info->setPhaseUpperBoundPaths(upper_box_bounds, k);
                    phaseActivePoints.push_back(activePoints);
                }

                // solve with current box and ap constraints
                problem->setActivePoints(phaseActivePoints);
                if (msdata.treat_dynamics_as_cost) info->setDynamicsCostWeight(dynamics_cost_weight);
                if (use_linearized_system)
                {
                    info->setUseLinearizedDae(true);
                    problem->setUpLinearizedParameters(info->getVariables().data());
                    success = solver.solve(problem);
                    if (!success)
                    {
                        OMPL_ERROR("Failed to find a feasible trajectory!");
                        break;
                    }
                    info->setUseLinearizedDae(false);
                }
                success = solver.solve(problem);
                if (!success)
                {
                    OMPL_ERROR("Failed to find a feasible trajectory!");
                    break;
                }

                // repair current local optimal trajectory
                while (success)
                {
                    bool error = false;
                    for (std::size_t k = 0; k < msdata.nsegments; k++)
                    {
                        std::size_t nnactive = 0, nnactive_state = 0;
                        std::vector<psopt::ActivePoints2D<double>> tempPoints;
                        const double * time = solver.getPhaseTime(k).data();
                        const double * states = solver.getPhaseStates(k).data() + msdata.nstates;
                        for (std::size_t i = 1; i < msdata.nnodes[k]; i++)
                        {
                            double x = states[0];
                            double y = states[1];
                            double yaw = states[2];
                            states += msdata.nstates;

                            if (overtaking_case > 0)
                            {
                                b2Transform oxf = b2Transform::Identity();
                                oxf.translation().x() = 10.0 + vehicleParam->delta + 10.0 * time[i]; // overtaking_case 1 
                                svc->SetBodyTransform(obs_name1, oxf);
                                if (overtaking_case == 2)
                                {
                                    oxf.translation().x() += 10.0; // overtaking_case 2 
                                    svc->SetBodyTransform(obs_name2, oxf);
                                }
                            }

                            b2Mat22 axis = b2Rot(yaw).toRotationMatrix();
                            b2Vec2 center(x, y);
                            center += vehicleParam->delta * axis.col(0); // local vehicle center
                            b2OBB obb(axis, center, iextent);
                            APCallback callback(0.05, rect, &obb);
                            svc->Collide(&callback, obb);

                            if (callback.getActivePoints().empty()) continue;
                            psopt::ActivePoints2D<double> apoints; 
                            apoints.index = i;
                            apoints.activePoints.swap(callback.getActivePoints());
                            tempPoints.push_back(apoints);
                            nnactive_state += 1;
                            nnactive += apoints.activePoints.size();
                        }
                        if (nnactive_state == 0) continue;
                        error = true;
                        std::vector<double> lower_box_bounds = info->getPhaseLowerBoundPaths(k);
                        std::vector<double> upper_box_bounds = info->getPhaseUpperBoundPaths(k);
                        info->setPhaseNumberPaths(lower_box_bounds.size() + nnactive, k);
                        for (std::size_t i = 0; i < nnactive; i++)
                        {
                            lower_box_bounds.push_back(1.0005); 
                            upper_box_bounds.push_back(std::numeric_limits<double>::infinity());
                        }
                        info->setPhaseLowerBoundPaths(lower_box_bounds, k);
                        info->setPhaseUpperBoundPaths(upper_box_bounds, k);
                        phaseActivePoints[k].insert(phaseActivePoints[k].end(), tempPoints.begin(), tempPoints.end());
                    }
                    if (!error) break;
                    OMPL_WARN("Current local optimal trajectory is infeasible, try to fix it!");
                    /*
                    if (use_linearized_system)
                    {
                        info->setUseLinearizedDae(true);
                        success = solver.solve(problem);
                        if (!success) break;
                        info->setUseLinearizedDae(false);
                    }
                    */
                    problem->setActivePoints(phaseActivePoints);
                    success = solver.solve(problem);
                }
                if (!success)
                {
                    OMPL_ERROR("Failed to find a feasible trajectory!");
                    break;
                }
                if (msdata.treat_dynamics_as_cost)
                {
                    OMPL_INFORM("Current cost %0.4f, current infeasiblity %0.4f, current w_penalty %0.4f", solver.getCost(), info->getDynamicsCost(), dynamics_cost_weight);
                    if (info->getDynamicsCost() < 1.e-2) // TODO
                    {
                        OMPL_INFORM("Found the best trajectory!");
                        break;
                    }
                    dynamics_cost_weight *= 10.0;
                }
                else
                    OMPL_INFORM("Current cost %0.4f", solver.getCost());
                if (solver.getCost() <= best_cost)
                {
                    best_cost = solver.getCost();
                }
                if (solver.getCost() < box_best_cost)//(std::abs(best_cost - solver.getCost()) < 1.e-2 * std::abs(best_cost)) // TODO
                {
                    OMPL_INFORM("Found the best trajectory!");
                    break;
                }
                // use_linearized_system = true; // TODO
                // info->setUseLinearizedDae(true);
                // problem->setUpLinearizedParameters(info->getVariables().data());
            }

            if (success)
            {
                double opti_time = ompl::time::seconds(ompl::time::now() - timeStart);
                time3.push_back(opti_time);
                costs2.push_back(best_cost);
                iteration++;
            }
            else
            {
                time1.pop_back();
                time2.pop_back();
                costs1.pop_back();
            }

            delete rect;
            delete info;
            delete vehicleParam;
            delete problem;
            rect = nullptr;
            info = nullptr;
            vehicleParam = nullptr;
            problem = nullptr;
        }
    }

    {
        std::ofstream ofs(boost::str(boost::format("box_time_%i.txt") % overtaking_case).c_str());
        for (const double & time : time1)
            ofs << time << " ";
        ofs << std::endl;
        for (const double & time : time2)
            ofs << time << " ";
        ofs.close();

        ofs.open(boost::str(boost::format("box_cost_%i.txt") % overtaking_case).c_str());
        for (const double & cost : costs1)
            ofs << cost << " ";
        ofs.close();
    }

    {
        std::ofstream ofs(boost::str(boost::format("box_ap_time_%i.txt") % overtaking_case).c_str());
        for (const double & time : time1)
            ofs << time << " ";
        ofs << std::endl;
        for (const double & time : time3)
            ofs << time << " ";
        ofs.close();

        ofs.open(boost::str(boost::format("box_ap_cost_%i.txt") % overtaking_case).c_str());
        for (const double & cost : costs2)
            ofs << cost << " ";
        ofs.close();
    }

    return 0;
}
