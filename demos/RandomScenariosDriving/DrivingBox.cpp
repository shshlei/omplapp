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
#include "DrivingWoObstacles.h" 

#include <box2d_collision/b2_collision.h>

#include <psopt/interpolation.hpp>

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
bool checkCollision(const std::shared_ptr<app::Box2dStateValidityChecker> & svc, double x, double y, const b2AABB & aabb)
{
    b2AABB taabb = aabb.translated(b2Vec2(x, y));
    CollisionCallback callback(taabb);
    svc->getBox2dDiscreteBVHManager()->getBox2dBroadphse()->Collide(&callback, taabb);
    return callback.getCollison();
}

// circle center
// local circle caabb
// global aabb result
bool generateBox(const std::shared_ptr<app::Box2dStateValidityChecker> & svc, const b2Vec2 & center, const b2AABB & caabb, int corridor_max_iter, double corridor_incremental_limit, b2AABB & result)
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
            real_x -= iter * 0.0005;
        }
        else if (edge == 1)
        {
            real_x += iter * 0.0005;
        }
        else if (edge == 2)
        {
            real_y -= iter * 0.0005;
        }
        else if (edge == 3)
        {
            real_y += iter * 0.0005;
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
class DrivingProblemBox : public DrivingProblemBase<Scalar, Scalar2>
{
public:

    DrivingProblemBox(psopt::ProblemInfo<Scalar2>* prob, const VehicleParam<Scalar2>* vehicleParam, const VehicleCircleParam<Scalar2>* vehicleCircleParam) : DrivingProblemBase<Scalar, Scalar2>(prob, vehicleParam)
    {
        vehicleCircleParam_ = vehicleCircleParam;
    }

    virtual ~DrivingProblemBox() = default;

    psopt::Problem<adouble, Scalar2>* clone() const override
    {
        DrivingProblemBox<adouble, Scalar2>* prob = new DrivingProblemBox<adouble, Scalar2>(this->problemInfo_, this->vehicleParam_, this->vehicleCircleParam_);
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
    // sampling
    std::shared_ptr<app::Box2dStateValidityChecker> svc;
    double x0, y0, theta0, xf, yf, thetaf;
    std::vector<double> trajx, trajy, trajt;
    double sampling_time = 0.0;
    int save_num;
    if (!solve(argc, argv, svc, x0, y0, theta0, xf, yf, thetaf, trajx, trajy, trajt, sampling_time, save_num)) return -1;
    std::size_t expected_nnodes = trajx.size();

    // basic problem definition
    psopt::MultiSegmentData msdata;
    std::size_t p = 30;
    msdata.nsegments = expected_nnodes / p;
    if (expected_nnodes % p != 0) msdata.nsegments += 1;
    msdata.nstates = 4;
    msdata.ncontrols = 2;
    msdata.nparameters = 0;
    msdata.ninitial_events = 4;
    msdata.nfinal_events = 4;
    msdata.npaths = 0;
    msdata.continuous_controls = true;
    msdata.paths_along_trajectory = false;
    msdata.parameters_along_trajectory = false;
    msdata.treat_dynamics_as_cost = false;

    std::size_t nmod = expected_nnodes / msdata.nsegments;
    std::vector<std::size_t> nnodes(msdata.nsegments, nmod);
    nnodes.back() = expected_nnodes - (msdata.nsegments - 1) * (nmod - 1);
    msdata.nnodes.swap(nnodes);
    psopt::ProblemInfo<double>* info = new psopt::ProblemInfo<double>(msdata);
    info->setLinearSolver("ma57");
    info->setTolerance(1.e-8);

    // interpolated states
    /*
    info->setupPScore();
    Eigen::VectorXd etime = Eigen::VectorXd::LinSpaced(expected_nnodes, -1.0, 1.0);
    std::vector<double> vtime(etime.data(), etime.data() + expected_nnodes);
    std::vector<double> ptime = info->getNodes(expected_nnodes - 1);
    psopt::LinearInterpolation<double, double> linearInterp(vtime, ptime);
    std::vector<double> interp_trajx = linearInterp.interpolate(trajx);
    std::vector<double> interp_trajy = linearInterp.interpolate(trajy);
    std::vector<double> interp_trajt = linearInterp.interpolate(trajt);
    */
    std::vector<double> interp_trajx;
    std::vector<double> interp_trajy;
    std::vector<double> interp_trajt;
    interp_trajx.swap(trajx);
    interp_trajy.swap(trajy);
    interp_trajt.swap(trajt);

    // vehicle param
    VehicleParam<double> *vehicleParam = new VehicleParam<double>(0.028, 0.0096, 0.00929, 0.01942);

    // initialization other trajectory parameters
    std::vector<double> interp_trajalpha(expected_nnodes, 0.0);
    std::vector<double> interp_trajv(expected_nnodes, 0.0);
    std::vector<double> interp_trajomega(expected_nnodes, 0.0);
    for (std::size_t i = 1; i < expected_nnodes; i++)
    {
        interp_trajv[i] = 0.1 * std::hypot(interp_trajx[i] - interp_trajx[i - 1], interp_trajy[i] - interp_trajy[i - 1]); 
        if (interp_trajx[i] < interp_trajx[i - 1]) interp_trajv[i] = -interp_trajv[i];
        if (abs(interp_trajv[i]) > 1.e-6)
            interp_trajalpha[i] = (interp_trajt[i] - interp_trajt[i - 1]) * vehicleParam->l / interp_trajv[i]; 
    }
    for (std::size_t i = 1; i < expected_nnodes; i++)
    {
        interp_trajomega[i] = (interp_trajalpha[i] - interp_trajalpha[i - 1]); 
    }

    // bounds and guess
    driving_bounds(info, x0, y0, theta0, xf, yf, thetaf);
    info->setPhaseLowerBoundStartTime(0.0);
    info->setPhaseUpperBoundStartTime(0.0);
    info->setPhaseLowerBoundEndTime(2.0);
    info->setPhaseUpperBoundEndTime(200.0);
    for (std::size_t i = 1; i < msdata.nsegments; i++)
    {
        info->setPhaseLowerBoundStartTime(2.0, i);
        info->setPhaseUpperBoundStartTime(200.0, i);
        info->setPhaseLowerBoundEndTime(2.0, i);
        info->setPhaseUpperBoundEndTime(200.0, i);
    }
    driving_guess(info, x0, y0, theta0);

    // initial guess states
    psopt::Solver<double> solver;
    solver.setPhaseNumbers(msdata.nsegments);
    std::size_t start = 0;
    for (std::size_t k = 0; k < msdata.nsegments; k++)
    {
        std::vector<double> guess_states, guess_controls;
        guess_states.reserve(msdata.nstates * msdata.nnodes[k]);
        guess_controls.reserve(msdata.ncontrols * msdata.nnodes[k]);
        for (std::size_t i = 0; i < msdata.nnodes[k]; i++)
        {
            std::size_t offset = start + i;
            guess_states.push_back(interp_trajx[offset]);
            guess_states.push_back(interp_trajy[offset]);
            guess_states.push_back(interp_trajt[offset]);
            guess_states.push_back(interp_trajalpha[offset]);
            guess_controls.push_back(interp_trajv[offset]);
            guess_controls.push_back(interp_trajomega[offset]);
            for (std::size_t j = 4; j < msdata.nstates; j++)
                guess_states.push_back(0.0);
            for (std::size_t j = 2; j < msdata.ncontrols; j++)
                guess_controls.push_back(0.0);
        }
        start += (msdata.nnodes[k] - 1);
        info->setPhaseGuessStates(guess_states, k);
        info->setPhaseGuessControls(guess_controls, k);
        solver.setPhaseStates(guess_states, k);
    }
    interp_trajx.clear();
    interp_trajy.clear();
    interp_trajt.clear();
    interp_trajalpha.clear();
    interp_trajv.clear();
    interp_trajomega.clear();

    // vehicle param
    VehicleCircleParam<double> *vehicleCircleParam = new VehicleCircleParam<double>(vehicleParam);
    b2AABB caabb(b2Vec2(-vehicleCircleParam->radius, -vehicleCircleParam->radius), b2Vec2(vehicleCircleParam->radius, vehicleCircleParam->radius));

    int sol = 0;
    bool success = true;
    bool use_linearized_system = false;
    double dynamics_cost_weight = 10.0;
    double best_cost = std::numeric_limits<double>::max();
    DrivingProblemBox<double>* problem = new DrivingProblemBox<double>(info, vehicleParam, vehicleCircleParam);
    while (success && sol < 10)
    {
        std::ofstream ofs;
        ofs.open(boost::str(boost::format("states_%i.txt") % sol).c_str());
        for (std::size_t k = 0; k < msdata.nsegments; k++)
        {
            // current states trajectory
            const double * states = solver.getPhaseStates(k).data();
            for (std::size_t j = 0; j < msdata.nnodes[k]; j++)
            {
                for (std::size_t i = 0; i < msdata.nstates; i++)
                {
                    ofs << states[i] << " ";
                }
                ofs << std::endl;
                states += msdata.nstates;
            }
        }
        ofs.close();
        sol++;

        /*
        if (sol == 1)
            ofs.open("box.txt");
        */
        for (std::size_t k = 0; k < msdata.nsegments; k++)
        {
            // get active points
            std::vector<double> lower_box_bounds, upper_box_bounds;
            lower_box_bounds.reserve(4 * msdata.nnodes[k]);
            upper_box_bounds.reserve(4 * msdata.nnodes[k]);
            const double * states = solver.getPhaseStates(k).data() + msdata.nstates;
            for (std::size_t i = 1; i < msdata.nnodes[k]; i++)
            {
                double x = states[0];
                double y = states[1];
                double yaw = states[2];
                states += msdata.nstates;

                b2Mat22 axis = b2Rot(yaw).toRotationMatrix();
                b2Vec2 center(x, y);
                b2Vec2 rear = center + vehicleCircleParam->rx * axis.col(0);
                b2Vec2 front = center + vehicleCircleParam->fx * axis.col(0);

                b2AABB result1, result2;
                if (!generateBox(svc, rear, caabb, 1000, 0.02, result1))
                {
                    success = false;
                    OMPL_ERROR("Failed to generate box constraints!");
                    break;
                }

                if (!generateBox(svc, front, caabb, 1000, 0.02, result2))
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
                /*
                if (sol == 1)
                {
                    ofs << result1.min().x() << " " << result1.min().y() << " " << result1.sizes().x() << " " << result1.sizes().y() << std::endl; 
                    // ofs << result2.min().x() << " " << result2.min().y() << " " << result2.sizes().x() << " " << result2.sizes().y() << std::endl; 
                }
                */
            }
            if (!success) break;

            info->setPhaseNumberPaths(lower_box_bounds.size(), k);
            info->setPhaseLowerBoundPaths(lower_box_bounds, k);
            info->setPhaseUpperBoundPaths(upper_box_bounds, k);
        }
        /*
        if (sol == 1)
            ofs.close();
        */
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
            if (info->getDynamicsCost() < 1.e-2)
            {
                OMPL_INFORM("Found the best trajectory!");
                break;
            }
            dynamics_cost_weight *= 10.0;
        }
        else
            OMPL_INFORM("Current cost %0.4f", solver.getCost());
        if (std::abs(best_cost - solver.getCost()) < 1.e-2 * std::abs(best_cost))
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
        ofs.open(boost::str(boost::format("states_%i.txt") % sol).c_str());
        for (std::size_t k = 0; k < msdata.nsegments; k++)
        {
            // current states trajectory
            const double * states = solver.getPhaseStates(k).data();
            for (std::size_t j = 0; j < msdata.nnodes[k]; j++)
            {
                for (std::size_t i = 0; i < msdata.nstates; i++)
                {
                    ofs << states[i] << " ";
                }
                ofs << std::endl;
                states += msdata.nstates;
            }
        }
        ofs.close();
        sol++;


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

    delete info;
    delete vehicleParam;
    delete vehicleCircleParam;
    delete problem;
    info = nullptr;
    vehicleParam = nullptr;
    vehicleCircleParam = nullptr;
    problem = nullptr;

    return 0;
}
