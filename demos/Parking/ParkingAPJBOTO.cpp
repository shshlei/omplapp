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

#include "ParkingPureSampling.h" 

#include <box2d_collision/b2_math.h>
#include <box2d_collision/b2_distance.h>
#include <box2d_collision/b2_bvh_manager.h>

#include <psopt/problem.hpp>
#include <psopt/solver.hpp>
#include <psopt/interpolation.hpp>
#include <psopt/bomp/utils.hpp>
#include <psopt/bomp/collision_constraints/J_function_collision_constraints.h>
#include <psopt/bomp/four_scenarios_parking/four_scenarios_basic_setup.hpp>

class APCallback : public b2NaiveCallback
{
public:
    APCallback(double minDist, const b2RectangleShape * rect, const b2OBB * obb) : b2NaiveCallback()
    {
        minDist_ = minDist;

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
        if (!fixture->GetBody()->IsActive())
        {
            b2Scalar d;
            b2Vec2 p1, p2;
            dist_.SignedDistance(rect_, fixture->GetShape(), invxf_ * fixture->GetGlobalTransform(), &d, &p1, &p2);
            b2Vec2 normal = p2 - p1;
            p2 = xf_ * p2;
            if (d < minDist_) minDist_ = d;
            if (d < 0.0) error_ = true;

            psopt::ActivePoint<double> apoint;
            apoint.Ba.push_back(p2);
            if (abs(normal(0)) >= abs(normal(1)))
            {
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
                // without principal normal
                if (abs(normal(0)) < 2.0 * abs(normal(1)))
                {
                    minDist_ = std::min(minDist_, abs(normal(1)));
                    psopt::ActivePoint<double> apoint2;
                    apoint2.Ba.push_back(p2);
                    if (normal(1) > 0.0)
                    {
                        apoint2.Aa.push_back(b2Vec2(hsides_[0], hsides_[1]));
                        apoint2.Aa.push_back(b2Vec2(-hsides_[0], hsides_[1]));
                    }
                    else
                    {
                        apoint2.Aa.push_back(b2Vec2(-hsides_[0], -hsides_[1]));
                        apoint2.Aa.push_back(b2Vec2(hsides_[0], -hsides_[1]));
                    }
                    activePoints_.push_back(apoint2);
                }
            }
            else
            {
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
                // without principal normal
                if (abs(normal(1)) < 2.0 * abs(normal(0)))
                {
                    minDist_ = std::min(minDist_, abs(normal(0)));
                    psopt::ActivePoint<double> apoint2;
                    apoint2.Ba.push_back(p2);
                    if (normal(0) > 0.0)
                    {
                        apoint2.Aa.push_back(b2Vec2(hsides_[0], -hsides_[1]));
                        apoint2.Aa.push_back(b2Vec2(hsides_[0], hsides_[1]));
                    }
                    else
                    {
                        apoint2.Aa.push_back(b2Vec2(-hsides_[0], hsides_[1]));
                        apoint2.Aa.push_back(b2Vec2(-hsides_[0], -hsides_[1]));
                    }
                    activePoints_.push_back(apoint2);
                }
            }
            activePoints_.push_back(apoint);
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

    double minDist_;

    // Distance Calculation
    b2ShapeDistance dist_;

    // Local vehicle rectangle
    const b2RectangleShape* rect_;
    b2Vec2 hsides_;

    // Vehicle Global Transform
    b2Transform xf_, invxf_;

    std::vector<psopt::ActivePoint<double>> activePoints_;
};

template <typename Scalar = double, typename Scalar2 = Scalar>
class FourScenariosProblemJ2 : public FourScenariosProblemBase<Scalar, Scalar2>
{
public:

    FourScenariosProblemJ2(psopt::ProblemInfo<Scalar2>* prob, const VehicleParam<Scalar2>* vehicleParam, bool forward = true) : FourScenariosProblemBase<Scalar, Scalar2>(prob, vehicleParam, forward)
    {
    }

    virtual ~FourScenariosProblemJ2() = default;

    psopt::Problem<adouble, Scalar2>* clone() const override
    {
        FourScenariosProblemJ2<adouble, Scalar2>* prob = new FourScenariosProblemJ2<adouble, Scalar2>(this->problemInfo_, this->vehicleParam_, this->forward_);
        prob->setActive(active_);
        prob->setPathWeights(pathWeights_);
        prob->setActivePoints(activePoints_);
        prob->setActiveTrajx(activeTrajx_);
        prob->setActiveTrajy(activeTrajy_);
        prob->setActiveTrajt(activeTrajt_);
        return prob;
    }

    void setActive(bool value)
    {
        active_ = value;
    }

    void setPathWeights(const std::vector<Scalar2> & pathWeights)
    {
        pathWeights_ = pathWeights;
    }

    void setActivePoints(const std::vector<psopt::ActivePoints<Scalar2>> & activePoints)
    {
        activePoints_ = activePoints;
    }

    void setActiveTrajx(const std::vector<Scalar2> & activeTrajx)
    {
        activeTrajx_ = activeTrajx; 
    }

    void setActiveTrajy(const std::vector<Scalar2> & activeTrajy)
    {
        activeTrajy_ = activeTrajy; 
    }

    void setActiveTrajt(const std::vector<Scalar2> & activeTrajt)
    {
        activeTrajt_ = activeTrajt; 
    }

    Scalar endpointCost(const Scalar* initial_states, const Scalar* /*final_states*/,
        const Scalar* /*initial_controls*/, const Scalar* /*final_controls*/, 
        const Scalar* /*parameters*/, const Scalar& t0, const Scalar& tf,
        const Scalar* /*xad*/, std::size_t iphase) const override
    {
        Scalar res = 10.0 * (tf - t0);
        if (!active_) return res;

        //const std::vector<Scalar2> & activeTrajx = activeTrajx_[iphase];
        //const std::vector<Scalar2> & activeTrajy = activeTrajy_[iphase];
        std::size_t nnodes = this->problemInfo_->getPhaseNumberNodes(iphase);
        std::size_t nstates = this->problemInfo_->getPhaseNumberStates(iphase);
        const Scalar* states = initial_states;
        for (std::size_t i = 0; i < nnodes; i++)
        {
            Scalar dist = (states[0] - activeTrajx_[i]) * (states[0] - activeTrajx_[i]) + (states[1] - activeTrajy_[i]) * (states[1] - activeTrajy_[i]);
            dist += 4.0 * (states[2] - activeTrajt_[i]) * (states[2] - activeTrajt_[i]);
            res += pathWeights_[i] * dist;
            states += nstates;
        }
        return res;
    }

    void pathAlongTrajectory(Scalar* /*paths*/, const Scalar* /*states*/,
        const Scalar* /*controls*/, const Scalar* /*parameters*/, const Scalar& /*time*/, const Scalar* /*xad*/, std::size_t /*iphase*/) const override
    {
    }

    void path(Scalar* paths, const Scalar* states, const Scalar* /*parameters*/,
        const Scalar* /*x*/, std::size_t iphase) const override
    {
        std::size_t offset = 0;
        std::size_t nstates = this->problemInfo_->getPhaseNumberStates(iphase);
        for (const psopt::ActivePoints<Scalar2> & apoints : activePoints_)
        {
            const Scalar * cstates = states + apoints.index * nstates;
            const Scalar x = cstates[0];
            const Scalar y = cstates[1];
            const Scalar theta = cstates[2];
            Eigen::Matrix<Scalar, 2, 3> invtransform = invtransform_2D(x, y, theta);
            if (this->forward_)
                invtransform(0, 2) -= this->vehicleParam_->delta;
            else
                invtransform(0, 2) += this->vehicleParam_->delta;
            MJ_2_Active_Constraints_2D(paths + offset, invtransform, apoints.activePoints);
            offset += apoints.activePoints.size();
        }
    }

private:

    bool active_{false};

    std::vector<Scalar2> pathWeights_;

    std::vector<psopt::ActivePoints<Scalar2>> activePoints_;

    std::vector<Scalar2> activeTrajx_, activeTrajy_, activeTrajt_;
};

int main(int argc, char* argv[])
{
    std::shared_ptr<app::Box2dStateValidityChecker> svc;
    bool parking_forward = false;
    double x0, y0, theta0;
    std::vector<double> trajx, trajy, trajt;
    double sampling_time;
    if (!solve(argc, argv, svc, parking_scenario, parking_forward, x0, y0, theta0, trajx, trajy, trajt, sampling_time)) return -1;
    std::size_t expected_nnodes = trajx.size();

    std::cout << "*** Active Phase!" << std::endl;

    // basic problem definition
    psopt::MultiSegmentData msdata;
    msdata.nsegments = 1;
    msdata.nstates = 4;
    msdata.ncontrols = 2;
    msdata.nparameters = 0;
    msdata.ninitial_events = 4;
    msdata.nfinal_events = 4;
    msdata.npaths = 0;
    msdata.parameters_along_trajectory = false;
    msdata.continuous_controls = false;
    if (msdata.nsegments == 2) // TODO
    {
        msdata.nnodes.push_back(15);
        msdata.nnodes.push_back(15);
    }
    else
        msdata.nnodes.push_back(expected_nnodes);
    psopt::ProblemInfo<double>* info = new psopt::ProblemInfo<double>(msdata);
    info->setLinearSolver("ma57");
    info->setTolerance(1.e-8);

    // interpolated states
    info->setupPScore();
    Eigen::VectorXd etime = Eigen::VectorXd::LinSpaced(expected_nnodes, -1.0, 1.0);
    std::vector<double> vtime(etime.data(), etime.data() + expected_nnodes);
    std::vector<double> ptime = info->getNodes(expected_nnodes - 1);
    psopt::LinearInterpolation<double, double> linearInterp(vtime, ptime);
    std::vector<double> interp_trajx = linearInterp.interpolate(trajx);
    std::vector<double> interp_trajy = linearInterp.interpolate(trajy);
    std::vector<double> interp_trajt = linearInterp.interpolate(trajt);
    ofs.open("pathsol_interp.txt"); // TODO delete
    for (std::size_t i = 0; i < expected_nnodes; i++)
    {
        ofs << interp_trajx[i] << " " << interp_trajy[i] << " " << interp_trajt[i] << std::endl;
    }
    ofs.close();

    // vehicle param
    VehicleParam<double> *vehicleParam = new VehicleParam<double>(2.8, 0.96, 0.929, 1.942);
    double delta = vehicleParam->delta;
    if (!parking_forward) delta = -delta;

    // local vehicle rectangle
    b2RectangleShape * rect = new b2RectangleShape(0.5 * vehicleParam->L, 0.5 * vehicleParam->W);

    // obb extent
    double active_dist = 0.5;
    b2Vec2 extent(0.5 * vehicleParam->L + active_dist, 0.5 * vehicleParam->W + active_dist);

    // get active points
    std::size_t nactive = 0;
    // if the robot is far away from obstacles, set a large path weight
    double wo_weight = 10.0;
    std::vector<double> pathWeights;
    std::vector<psopt::ActivePoints<double>> activePoints; 
    pathWeights.reserve(expected_nnodes);
    activePoints.reserve(expected_nnodes);
    std::ofstream ofs1;
    ofs.open("active_points_0.txt");
    ofs1.open("active_points_1.txt");
    for (std::size_t i = 0; i < expected_nnodes; i++)
    {
        double x = interp_trajx[i];
        double y = interp_trajy[i];
        double yaw = interp_trajt[i];

        b2Mat22 axis = b2Rot(yaw).toRotationMatrix();
        b2Vec2 center(x, y);
        center += delta * axis.col(0); // local vehicle center
        b2OBB obb(axis, center, extent);
        APCallback callback(active_dist, rect, &obb);
        svc->getBox2dDiscreteBVHManager()->getBox2dBroadphse()->Collide(&callback, obb);
        if (callback.getMinDist() >= active_dist) pathWeights.push_back(wo_weight);
        else pathWeights.push_back(std::exp(callback.getMinDist()));
    
        if (callback.getError())
        {
            std::cout << "Error!" << std::endl;
            std::cout << x << " " << y << " " << yaw << " " << std::endl;             
        }
        if (callback.getActivePoints().empty()) continue;

        bool first = true;
        for (const psopt::ActivePoint<double> & apoint : callback.getActivePoints())
        {
            for (const Eigen::Matrix<double, 2, 1> & p : apoint.Aa)
            {
                Eigen::Matrix<double, 2, 1> wp = axis * p + center;
                if (first) ofs << wp(0) << " " << wp(1) << std::endl;
                else ofs1 << wp(0) << " " << wp(1) << std::endl;
            }
            for (const Eigen::Matrix<double, 2, 1> & p : apoint.Ba)
            {
                if (first) ofs << p(0) << " " << p(1) << std::endl;
                else ofs1 << p(0) << " " << p(1) << std::endl;
            }
            first = false;
        }

        psopt::ActivePoints<double> apoints; 
        apoints.index = i;
        apoints.activePoints.swap(callback.getActivePoints());
        activePoints.push_back(apoints);
        nactive += apoints.activePoints.size();
    }
    std::cout << "Number Active Constraints " << nactive << std::endl;
    ofs.close();
    ofs1.close();

    four_scenarios_bounds(info, x0, y0, theta0);
    info->setPhaseLowerBoundStartTime(0.0);
    info->setPhaseUpperBoundStartTime(0.0);
    info->setPhaseLowerBoundEndTime(2.0);
    info->setPhaseUpperBoundEndTime(50.0);
    for (std::size_t i = 1; i < msdata.nsegments; i++)
    {
        info->setPhaseLowerBoundStartTime(2.0, i);
        info->setPhaseUpperBoundStartTime(50.0, i);
        info->setPhaseLowerBoundEndTime(2.0, i);
        info->setPhaseUpperBoundEndTime(50.0, i);
    }
    four_scenarios_guess(info, x0, y0, theta0);

    // active parameters and paths
    double Jeps = 0.05;
    info->setPathsAlongTrajectory(false);
    info->setParametersAlongTrajectory(false);
    info->setPhaseNumberPaths(nactive);
    info->setPhaseConstantLowerBoundPaths(1.0 + Jeps);
    info->setPhaseInfinityUpperBoundPaths();

    // initial guess states
    std::vector<double> guess_states;
    guess_states.reserve(msdata.nstates * msdata.nnodes[0]);
    for (std::size_t i = 0; i < msdata.nnodes[0]; i++)
    {
        guess_states.push_back(interp_trajx[i]);
        guess_states.push_back(interp_trajy[i]);
        guess_states.push_back(interp_trajt[i]);
        for (std::size_t j = 3; j < msdata.nstates; j++)
            guess_states.push_back(0.0);
    }
    info->setPhaseGuessStates(guess_states);

    FourScenariosProblemJ2<double>* problem = new FourScenariosProblemJ2<double>(info, vehicleParam, parking_forward);
    problem->setActive(true);
    problem->setPathWeights(pathWeights);
    problem->setActivePoints(activePoints); // TODO
    problem->setActiveTrajx(interp_trajx);
    problem->setActiveTrajy(interp_trajy);
    problem->setActiveTrajt(interp_trajt);

    psopt::Solver<double> solver;
    if (solver.solve(problem))
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
    delete problem;
    rect = nullptr;
    info = nullptr;
    vehicleParam = nullptr;
    problem = nullptr;

    return 0;
}
