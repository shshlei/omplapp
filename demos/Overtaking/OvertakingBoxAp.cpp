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
    psopt::MultiSegmentData msdata;
    psopt::Solver<double> solver;
    double stage1_time;
    if (!solve(argc, argv, msdata, solver, stage1_time)) return -1;

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
    bool success = true;
    bool use_linearized_system = false;
    double dynamics_cost_weight = 10.0;
    double best_cost = std::numeric_limits<double>::max();
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
                    APCallback callback(0.5, rect, &obb);
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
