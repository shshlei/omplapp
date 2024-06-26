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

#include <box2d_collision/b2_distance.h>

#include <psopt/interpolation.hpp>
#include <bomp/collision_constraints/active_collision_constraints.h>

class APCallback : public b2NaiveCallback
{
public:
    APCallback(double activeDist, const b2RectangleShape * rect, const b2Transform & xf) : b2NaiveCallback()
    {
        activeDist_ = minDist_ = activeDist;

        rect_ = rect;

        xf_ = xf;
        invxf_.setIdentity();
        invxf_.linear() = xf.linear().transpose();
        invxf_.translation() = -invxf_.linear() * xf.translation();
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
        apoint.normal = normal.normalized(); // TODO
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
class DrivingProblemBoxAp : public DrivingProblemBase<Scalar, Scalar2>
{
public:

    DrivingProblemBoxAp(psopt::OptimalControlProblemInfo<Scalar2>* prob, const VehicleParam<Scalar2>* vehicleParam) : DrivingProblemBase<Scalar, Scalar2>(prob, vehicleParam)
    {
    }

    virtual ~DrivingProblemBoxAp() = default;

    psopt::OptimalControlProblem<adouble, Scalar2>* clone() const override
    {
        DrivingProblemBoxAp<adouble, Scalar2>* prob = new DrivingProblemBoxAp<adouble, Scalar2>(this->problemInfo_, this->vehicleParam_);
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
            active_Constraints_2D(paths + offset, invtransform, apoints.activePoints);
            offset += apoints.activePoints.size();
        }
    }

private:

    std::vector<std::vector<psopt::ActivePoints2D<Scalar2>>> activePoints_;
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
    psopt::OptimalControlProblemInfo<double>* info = new psopt::OptimalControlProblemInfo<double>(msdata);
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
    psopt::OptimalControlSolver<double> solver;
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

    // local vehicle rectangle
    b2RectangleShape * rect = new b2RectangleShape(0.5 * vehicleParam->L, 0.5 * vehicleParam->W);

    // obb extent
    double active_dist = 0.02;
    b2Vec2 extent(0.5 * vehicleParam->L + active_dist, 0.5 * vehicleParam->W + active_dist);
    b2Vec2 iextent(0.5 * vehicleParam->L, 0.5 * vehicleParam->W);

    int sol = 0;
    bool success = true;
    bool use_linearized_system = false;
    double dynamics_cost_weight = 10.0;
    double best_cost = std::numeric_limits<double>::max();
    DrivingProblemBoxAp<double>* problem = new DrivingProblemBoxAp<double>(info, vehicleParam);
    while (success && sol < 10)
    {
        // save current states
        /*
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
        */
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
            const double * states = solver.getPhaseStates(k).data() + msdata.nstates;
            for (std::size_t i = 1; i < msdata.nnodes[k]; i++)
            {
                double x = states[0];
                double y = states[1];
                double yaw = states[2];
                states += msdata.nstates;

                b2Mat22 axis = b2Rot(yaw).toRotationMatrix();
                b2Vec2 center(x, y);
                center += vehicleParam->delta * axis.col(0); // local vehicle center

                b2Transform xf = b2Transform::Identity();
                xf.linear() = axis;
                xf.translation() = center;

                b2AABB aabb;
                rect->ComputeAABB(&aabb, xf);
                aabb.min() -= b2Vec2(active_dist, active_dist);
                aabb.max() += b2Vec2(active_dist, active_dist);

                APCallback callback(active_dist, rect, xf);
                svc->getBox2dDiscreteBVHManager()->getBox2dBroadphse()->Collide(&callback, aabb);

                double bound = active_dist;
                if (callback.getMinDist() < active_dist) bound = std::max(callback.getMinDist(), 0.5 * active_dist);
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
                const double * states = solver.getPhaseStates(k).data() + msdata.nstates;
                for (std::size_t i = 1; i < msdata.nnodes[k]; i++)
                {
                    double x = states[0];
                    double y = states[1];
                    double yaw = states[2];
                    states += msdata.nstates;

                    b2Mat22 axis = b2Rot(yaw).toRotationMatrix();
                    b2Vec2 center(x, y);
                    center += vehicleParam->delta * axis.col(0); // local vehicle center

                    b2Transform xf = b2Transform::Identity();
                    xf.linear() = axis;
                    xf.translation() = center;

                    b2AABB aabb;
                    rect->ComputeAABB(&aabb, xf);
                    aabb.min() -= b2Vec2(active_dist, active_dist);
                    aabb.max() += b2Vec2(active_dist, active_dist);

                    APCallback callback(0.0001, rect, xf);
                    svc->getBox2dDiscreteBVHManager()->getBox2dBroadphse()->Collide(&callback, aabb);

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
        if (std::abs(best_cost - solver.getCost()) < 1.e-2 * std::abs(best_cost)) // TODO
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
            const double * states = solver.getPhaseStates(k).data();
            for (std::size_t j = 0; j < msdata.nnodes[k]; j++)
            {
                for (std::size_t i = 0; i < msdata.nstates; i++)
                {
                    ofs << states[i] << " ";
                }
                states += msdata.nstates;
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
