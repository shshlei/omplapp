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

#include <box2d_collision/b2_bvh_manager.h>

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
            real_x -= iter * 0.02;
        }
        else if (edge == 1)
        {
            real_x += iter * 0.02;
        }
        else if (edge == 2)
        {
            real_y -= iter * 0.02;
        }
        else if (edge == 3)
        {
            real_y += iter * 0.02;
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

    OvertakingProblemBox(psopt::OptimalControlProblemInfo<Scalar2>* prob, const VehicleParam<Scalar2>* vehicleParam, const VehicleCircleParam<Scalar2>* vehicleCircleParam) : OvertakingProblemBase<Scalar, Scalar2>(prob, vehicleParam)
    {
        vehicleCircleParam_ = vehicleCircleParam;
    }

    virtual ~OvertakingProblemBox() = default;

    psopt::OptimalControlProblem<adouble, Scalar2>* clone() const override
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

int main(int argc, char* argv[])
{
    psopt::MultiSegmentData msdata;
    psopt::OptimalControlSolver<double> solver;
    double stage1_time;
    if (!solve(argc, argv, msdata, solver, stage1_time)) return -1;

    psopt::OptimalControlProblemInfo<double> * info = new psopt::OptimalControlProblemInfo<double>(msdata);
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
        oxf.translation().x() = overtaking_dist1 + vehicleParam->delta; // overtaking_case == 0
        svc->SetBodyTransform(obs_name1, oxf);
    }

    int sol = 0;
    bool success = true;
    bool use_linearized_system = false;
    double dynamics_cost_weight = 10.0;
    double best_cost = std::numeric_limits<double>::max();
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
                    oxf.translation().x() = overtaking_dist1 + vehicleParam->delta + overtaking_speed * time[i]; // overtaking_case 1 
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
        std::ofstream ofs;
        ofs.open("states.txt");
        for (std::size_t i = 0; i < msdata.nsegments; i++)
        {
            std::size_t start = 0;
            if (i > 0) start += 1;
            const std::vector<double> & states = solver.getPhaseStates(i);
            for (std::size_t j = start; j < msdata.nnodes[i]; j++)
            {
                std::size_t offset = j * msdata.nstates;
                for (std::size_t k = 0; k < msdata.nstates; k++)
                {
                    ofs << states[offset + k] << " ";
                }
                ofs << std::endl;
            }
        }
        ofs.close();

        ofs.open("controls.txt");
        for (std::size_t i = 0; i < msdata.ncontrols; i++)
        {
            for (std::size_t j = 0; j < msdata.nsegments; j++)
            {
                std::vector<double> traj = solver.getPhaseTrajectoryControls(msdata.ncontrols, msdata.nnodes[j], i, j);
                if (j > 0) traj.erase(traj.begin());
                for (double data : traj) ofs << data << " ";
            }
            ofs << std::endl;
        }
        ofs.close();

        ofs.open("time.txt");
        for (std::size_t j = 0; j < msdata.nsegments; j++)
        {
            std::vector<double> traj = solver.getPhaseTime(j);
            if (j > 0) traj.erase(traj.begin());
            for (double data : traj) ofs << data << " ";
        }
        ofs.close();
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

    return 0;
}
