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

#include <bomp/collision_constraints/local_collision_constraints.h>

#include <box2d_collision/b2_distance.h>

#include <psopt/interpolation.hpp>

class LocalCollisionCallback : public b2NaiveCallback
{
public:
    LocalCollisionCallback(double activeDist, const b2RectangleShape * rect, const b2OBB * obb) : b2NaiveCallback()
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

    ~LocalCollisionCallback() override = default;

    bool ReportCollision(b2Fixture* fixture) override
    {
        if (fixture->GetBody()->IsActive()) return false;
        b2Scalar d;
        b2Vec2 p1, p2;
        dist_.SignedDistance(rect_, xf_, fixture->GetShape(), fixture->GetGlobalTransform(), &d, &p1, &p2);
        if (d >= activeDist_) return false;
        if (d < minDist_) minDist_ = d;

        b2Vec2 normal = p1 - p2;
        if (d < 0.0) normal = -normal;
        if (abs(d) < 1.e-6)
        {
            const b2Transform & xf = fixture->GetGlobalTransform();
            normal = xf.linear() * (xf.inverse() * p2);
        }

        psopt::LocalCollisionElement2D<double> apoint;
        apoint.normal = normal.normalized();
        apoint.a = p2;
        apoint.b = invxf_ * p1;
        localCollisionElements_.push_back(apoint);

        return false; // Never stop early
    }

    double getMinDist() const
    {
        return minDist_;
    }

    std::vector<psopt::LocalCollisionElement2D<double>> & getLocalCollisionElements()
    {
        return localCollisionElements_;
    }

private:

    double activeDist_, minDist_;

    // Distance Calculation
    b2ShapeDistance dist_;

    // Local vehicle rectangle
    const b2RectangleShape* rect_;

    // Vehicle Global Transform
    b2Transform xf_, invxf_;

    std::vector<psopt::LocalCollisionElement2D<double>> localCollisionElements_;
};

template <typename Scalar = double, typename Scalar2 = Scalar>
class DrivingProblemJ2 : public DrivingProblemBase<Scalar, Scalar2>
{
public:

    DrivingProblemJ2(psopt::ProblemInfo<Scalar2>* prob, const VehicleParam<Scalar2>* vehicleParam) : DrivingProblemBase<Scalar, Scalar2>(prob, vehicleParam)
    {
    }

    virtual ~DrivingProblemJ2() = default;

    psopt::Problem<adouble, Scalar2>* clone() const override
    {
        DrivingProblemJ2<adouble, Scalar2>* prob = new DrivingProblemJ2<adouble, Scalar2>(this->problemInfo_, this->vehicleParam_);
        prob->setLocalCollisioniElements(this->localCollisionElements_);
        prob->setLinearizedParameters(this);
        return prob;
    }

    void setLocalCollisioniElements(const std::vector<std::vector<psopt::LocalCollisionElements2D<Scalar2>>> & localCollisionElements)
    {
        localCollisionElements_ = localCollisionElements;
    }

    void path(Scalar* paths, const Scalar* states, const Scalar* /*parameters*/,
        const Scalar* /*x*/, std::size_t iphase) const override
    {
        std::size_t offset = 0;
        std::size_t nnodes = this->problemInfo_->getPhaseNumberNodes(iphase);
        std::size_t nstates = this->problemInfo_->getPhaseNumberStates(iphase);
        const Scalar* cstates = states + nstates;
        for (std::size_t i = 0; i < nnodes - 1; i++)
        {
            const Scalar x = cstates[0];
            const Scalar y = cstates[1];
            cstates += nstates;

            paths[offset] = x;
            offset++;
            paths[offset] = y;
            offset++;
        }
        
        const std::vector<psopt::LocalCollisionElements2D<Scalar2>> localCollisionElements = localCollisionElements_[iphase];
        for (const psopt::LocalCollisionElements2D<Scalar2> & apoints : localCollisionElements)
        {
            const Scalar * cstates = states + apoints.index * nstates;
            const Scalar x = cstates[0];
            const Scalar y = cstates[1];
            const Scalar theta = cstates[2];
            Eigen::Matrix<Scalar, 2, 3> transform = transform_2D(x, y, theta);
            transform.col(2) += this->vehicleParam_->delta * transform.col(0);
            local_Collision_Constraints_2D(paths + offset, transform, apoints.localCollisionElements);
            offset += apoints.localCollisionElements.size();
        }
    }

private:

    std::vector<std::vector<psopt::LocalCollisionElements2D<Scalar2>>> localCollisionElements_;
};

int main(int argc, char* argv[])
{
    // sampling
    std::shared_ptr<app::Box2dStateValidityChecker> svc;
    double x0, y0, theta0, xf, yf, thetaf;
    std::vector<double> trajx, trajy, trajt;
    double timeUsed;
    int save_num;
    if (!solve(argc, argv, svc, x0, y0, theta0, xf, yf, thetaf, trajx, trajy, trajt, timeUsed, save_num)) return -1;
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
        std::vector<double> guess_states;
        guess_states.reserve(msdata.nstates * msdata.nnodes[k]);
        for (std::size_t i = 0; i < msdata.nnodes[k]; i++)
        {
            std::size_t offset = start + i;
            guess_states.push_back(interp_trajx[offset]);
            guess_states.push_back(interp_trajy[offset]);
            guess_states.push_back(interp_trajt[offset]);
            for (std::size_t j = 3; j < msdata.nstates; j++)
                guess_states.push_back(0.0);
        }
        start += (msdata.nnodes[k] - 1);
        info->setPhaseGuessStates(guess_states, k);
        solver.setPhaseStates(guess_states, k);
    }
    interp_trajx.clear();
    interp_trajy.clear();
    interp_trajt.clear();

    // vehicle param
    VehicleParam<double> *vehicleParam = new VehicleParam<double>(0.028, 0.0096, 0.00929, 0.01942);
    double delta = vehicleParam->delta;

    // local vehicle rectangle
    b2RectangleShape * rect = new b2RectangleShape(0.5 * vehicleParam->L, 0.5 * vehicleParam->W);

    // obb extent
    double active_dist = 0.020;
    b2Vec2 extent(0.5 * vehicleParam->L + active_dist, 0.5 * vehicleParam->W + active_dist);
    b2Vec2 iextent(0.5 * vehicleParam->L, 0.5 * vehicleParam->W);

    int sol = 0;
    bool success = true;
    bool use_linearized_system = false;
    double dynamics_cost_weight = 10.0;
    double best_cost = std::numeric_limits<double>::max();
    DrivingProblemJ2<double>* problem = new DrivingProblemJ2<double>(info, vehicleParam);
    while (success)
    {
        // save current states
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

        // box and ap constraints
        std::vector<std::vector<psopt::LocalCollisionElements2D<double>>> phaseLocalCollisionElements; 
        std::vector<std::vector<bool>> phaseApConstraints; 
        phaseLocalCollisionElements.reserve(msdata.nsegments);
        phaseApConstraints.reserve(msdata.nsegments);
        for (std::size_t k = 0; k < msdata.nsegments; k++)
        {
            // get active points
            std::size_t nactive = 0, nactive_state = 0;
            std::vector<double> lower_box_bounds, upper_box_bounds;
            lower_box_bounds.reserve(2 * msdata.nnodes[k]);
            upper_box_bounds.reserve(2 * msdata.nnodes[k]);
            std::vector<psopt::LocalCollisionElements2D<double>> localCollisionElements; 
            localCollisionElements.reserve(msdata.nnodes[k]);
            std::vector<bool> apConstraints(msdata.nnodes[k], false); 
            const double * states = solver.getPhaseStates(k).data() + msdata.nstates;
            for (std::size_t i = 1; i < msdata.nnodes[k]; i++)
            {
                double x = states[0];
                double y = states[1];
                double yaw = states[2];
                states += msdata.nstates;

                b2Mat22 axis = b2Rot(yaw).toRotationMatrix();
                b2Vec2 center(x, y);
                center += delta * axis.col(0); // local vehicle center
                b2OBB obb(axis, center, extent);
                LocalCollisionCallback callback(active_dist, rect, &obb);
                svc->getBox2dDiscreteBVHManager()->getBox2dBroadphse()->Collide(&callback, obb);
                double bound = active_dist;
                if (callback.getMinDist() < active_dist) bound = std::max(callback.getMinDist(), 0.5 * active_dist); // TODO
                lower_box_bounds.push_back(x - bound);
                lower_box_bounds.push_back(y - bound);
                upper_box_bounds.push_back(x + bound);
                upper_box_bounds.push_back(y + bound);
            
                if (callback.getLocalCollisionElements().empty()) continue;
                apConstraints[i] = true;
                psopt::LocalCollisionElements2D<double> apoints; 
                apoints.index = i;
                apoints.localCollisionElements.swap(callback.getLocalCollisionElements());
                localCollisionElements.push_back(apoints);
                nactive_state++;
                nactive += apoints.localCollisionElements.size();
            }

            info->setPhaseNumberPaths(lower_box_bounds.size() + nactive, k);
            for (std::size_t i = 0; i < nactive; i++)
            {
                lower_box_bounds.push_back(0.00001); 
                upper_box_bounds.push_back(std::numeric_limits<double>::infinity());
            }
            info->setPhaseLowerBoundPaths(lower_box_bounds, k);
            info->setPhaseUpperBoundPaths(upper_box_bounds, k);
            phaseLocalCollisionElements.push_back(localCollisionElements);
            phaseApConstraints.push_back(apConstraints);
        }

        // solve with current box and ap constraints
        problem->setLocalCollisioniElements(phaseLocalCollisionElements);
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
                std::vector<psopt::LocalCollisionElements2D<double>> tempPoints;
                std::vector<bool> & apConstraints = phaseApConstraints[k]; 
                const double * states = solver.getPhaseStates(k).data() + msdata.nstates;
                for (std::size_t i = 1; i < msdata.nnodes[k]; i++)
                {
                    double x = states[0];
                    double y = states[1];
                    double yaw = states[2];
                    states += msdata.nstates;
                    if (apConstraints[i]) continue;

                    b2Mat22 axis = b2Rot(yaw).toRotationMatrix();
                    b2Vec2 center(x, y);
                    center += delta * axis.col(0); // local vehicle center
                    b2OBB obb(axis, center, iextent);
                    LocalCollisionCallback callback(0.000001, rect, &obb);
                    svc->getBox2dDiscreteBVHManager()->getBox2dBroadphse()->Collide(&callback, obb);

                    if (callback.getLocalCollisionElements().empty()) continue;
                    psopt::LocalCollisionElements2D<double> apoints; 
                    apoints.index = i;
                    apoints.localCollisionElements.swap(callback.getLocalCollisionElements());
                    tempPoints.push_back(apoints);
                    nnactive_state += 1;
                    nnactive += apoints.localCollisionElements.size();
                    apConstraints[i] = true;
                }
                if (nnactive_state == 0) continue;
                error = true;
                std::vector<double> lower_box_bounds = info->getPhaseLowerBoundPaths(k);
                std::vector<double> upper_box_bounds = info->getPhaseUpperBoundPaths(k);
                info->setPhaseNumberPaths(lower_box_bounds.size() + nnactive, k);
                for (std::size_t i = 0; i < nnactive; i++)
                {
                    lower_box_bounds.push_back(0.00001); 
                    upper_box_bounds.push_back(std::numeric_limits<double>::infinity());
                }
                info->setPhaseLowerBoundPaths(lower_box_bounds, k);
                info->setPhaseUpperBoundPaths(upper_box_bounds, k);
                phaseLocalCollisionElements[k].insert(phaseLocalCollisionElements[k].end(), tempPoints.begin(), tempPoints.end());
            }
            if (!error) break;
            OMPL_WARN("Current local optimal trajectory is infeasible, try to fix it!");
            /*
            if (use_linearized_system)
            {
                info->setUseLinearizedDae(true);
                problem->setUpLinearizedParameters(info->getVariables().data());
                success = solver.solve(problem);
                if (!success) break;
                info->setUseLinearizedDae(false);
            }
            */
            problem->setLocalCollisioniElements(phaseLocalCollisionElements);
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
            if (info->getDynamicsCost() < 1.e-4) // TODO
            {
                OMPL_INFORM("Found the best trajectory!");
                break;
            }
            dynamics_cost_weight *= 10.0;
        }
        else
            OMPL_INFORM("Current cost %0.4f", solver.getCost());
        if (std::abs(best_cost - solver.getCost()) < 1.e-4 * std::abs(best_cost)) // TODO
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
