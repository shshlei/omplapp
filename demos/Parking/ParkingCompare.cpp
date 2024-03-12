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

#include <psopt/problem.hpp>
#include <psopt/solver.hpp>

#include <bomp/utils.hpp>
#include <bomp/vehicle_basic_model/vehicle_basic_model.hpp>
#include <bomp/collision_constraints/basic_collision_constraints.h>
#include <bomp/collision_constraints/J_function_collision_constraints.h>
#include <bomp/collision_constraints/dual_distance_collision_constraints.h>
#include <bomp/collision_constraints/dual_J_function_collision_constraints.h>
#include <bomp/four_scenarios_parking/four_scenarios_basic_setup.hpp>

// MJ2
Eigen::Matrix<double, 2, 4> rect_mj;
std::vector<Eigen::Matrix<double, 2, 4>> obstacles;

template <typename Scalar = double, typename Scalar2 = Scalar>
class FourScenariosProblemMJ2 : public VehicleBasicModel<Scalar, Scalar2>
{
public:

    FourScenariosProblemMJ2(psopt::ProblemInfo<Scalar2>* prob, const VehicleParam<Scalar2>* vehicleParam, bool forward = true) : VehicleBasicModel<Scalar, Scalar2>(prob, vehicleParam), forward_(forward)
    {
    }

    virtual ~FourScenariosProblemMJ2() = default;

    psopt::Problem<adouble, Scalar2>* clone() const override
    {
        FourScenariosProblemMJ2<adouble, Scalar2>* prob = new FourScenariosProblemMJ2<adouble, Scalar2>(this->problemInfo_, this->vehicleParam_, this->forward_);
        prob->setLinearizedParameters(this);
        return prob;
    }

    void pathAlongTrajectory(Scalar* paths, const Scalar* states,
        const Scalar* /*controls*/, const Scalar* parameters, const Scalar& /*time*/, const Scalar* /*xad*/, std::size_t /*iphase*/) const override
    {
        if (obstacles.empty()) return;
        const Scalar& x = states[0];
        const Scalar& y = states[1];
        const Scalar& theta = states[2];

        // rectangle obstacles
        Eigen::Matrix<Scalar, 2, 3> invtransform = invtransform_2D(x, y, theta);
        if (this->forward_)
            invtransform(0, 2) -= this->vehicleParam_->delta;
        else
            invtransform(0, 2) += this->vehicleParam_->delta;
        Eigen::Matrix<Scalar2, 3, 4> rect2 = Eigen::Matrix<Scalar2, 3, 4>::Ones();
        for (std::size_t i = 0; i < obstacles.size(); i++)
        {
            rect2.topRows(2) = obstacles[i];
            psopt::rectangle_rectangle_MJ_2_KKT_Norm(paths + 6 * i, parameters + 19 * i, (rect_mj.template cast<Scalar>()).eval(), (invtransform * rect2.template cast<Scalar>()).eval());
        }
    }

private:

    bool forward_;
};

// AC
Eigen::Matrix<double, 2, 4> rect_ac;

template <typename Scalar = double, typename Scalar2 = Scalar>
class FourScenariosProblemAreaCriterion : public VehicleBasicModel<Scalar, Scalar2>
{
public:

    FourScenariosProblemAreaCriterion(psopt::ProblemInfo<Scalar2>* prob, const VehicleParam<Scalar2>* vehicleParam) : VehicleBasicModel<Scalar, Scalar2>(prob, vehicleParam)
    {
    }

    virtual ~FourScenariosProblemAreaCriterion() = default;

    psopt::Problem<adouble, Scalar2>* clone() const override
    {
        FourScenariosProblemAreaCriterion<adouble, Scalar2>* prob = new FourScenariosProblemAreaCriterion<adouble, Scalar2>(this->problemInfo_, this->vehicleParam_);
        prob->setLinearizedParameters(this);
        return prob;
    }

    void pathAlongTrajectory(Scalar* paths, const Scalar* states,
        const Scalar* /*controls*/, const Scalar* /*parameters*/, const Scalar& /*time*/, const Scalar* /*xad*/, std::size_t /*iphase*/) const override
    {
        if (obstacles.empty()) return;
        // parameters for the car
        Scalar2 W = this->vehicleParam_->W;
        Scalar2 L = this->vehicleParam_->L;

        const Scalar& x = states[0];
        const Scalar& y = states[1];
        const Scalar& theta = states[2];

        // rectangle obstacles
        Eigen::Matrix<Scalar, 2, 3> transform = transform_2D(x, y, theta);
        Eigen::Matrix<Scalar2, 3, 4> temp = Eigen::Matrix<Scalar2, 3, 4>::Ones();
        temp.topRows(2) = rect_ac;
        Eigen::Matrix<Scalar, 2, 4> crect1= transform * temp.template cast<Scalar>(); 
        for (std::size_t i = 0; i < obstacles.size(); i++)
        {
            psopt::rectangle_rectangle(paths + 8 * i, crect1, obstacles[i], L * W);
        }
    }
};

// Dual Distance

std::vector<Eigen::Matrix<double, 2, 3>> obstacles_duald;

template <typename Scalar = double, typename Scalar2 = Scalar>
class FourScenariosProblemDualDistance : public VehicleBasicModel<Scalar, Scalar2>
{
public:

    FourScenariosProblemDualDistance(psopt::ProblemInfo<Scalar2>* prob, const VehicleParam<Scalar2>* vehicleParam, bool forward = true) : VehicleBasicModel<Scalar, Scalar2>(prob, vehicleParam), forward_(forward)
    {
    }

    virtual ~FourScenariosProblemDualDistance() = default;

    psopt::Problem<adouble, Scalar2>* clone() const override
    {
        FourScenariosProblemDualDistance<adouble, Scalar2>* prob = new FourScenariosProblemDualDistance<adouble, Scalar2>(this->problemInfo_, this->vehicleParam_, this->forward_);
        prob->setLinearizedParameters(this);
        return prob;
    }

    void pathAlongTrajectory(Scalar* paths, const Scalar* states,
        const Scalar* /*controls*/, const Scalar* parameters, const Scalar& /*time*/, const Scalar* /*xad*/, std::size_t /*iphase*/) const override
    {
        if (obstacles_duald.empty()) return;
        const Scalar& x = states[0];
        const Scalar& y = states[1];
        const Scalar& theta = states[2];

        // rectangle obstacles_duald
        Eigen::Matrix<Scalar, 2, 3> invtransform = invtransform_2D(x, y, theta);
        if (this->forward_)
            invtransform(0, 2) -= this->vehicleParam_->delta;
        else
            invtransform(0, 2) += this->vehicleParam_->delta;

        Eigen::Matrix<Scalar2, 2, 1> rect_mj(0.5 * this->vehicleParam_->L, 0.5 * this->vehicleParam_->W);
        Eigen::Matrix<Scalar2, 2, 1> rect2 = rect_mj;
        if (parking_scenario == 2)
        {
            rect2[0] = 0.5 * this->vehicleParam_->W;
            rect2[1] = 0.5 * this->vehicleParam_->L;
        }

        for (std::size_t i = 0; i < obstacles_duald.size(); i++)
        {
            Eigen::Matrix<Scalar, 2, 3> transform = obstacles_duald[i].template cast<Scalar>();
            Eigen::Matrix<Scalar, 2, 2> invR2 = invtransform.topLeftCorner(2, 2) * transform.topLeftCorner(2, 2);
            Eigen::Matrix<Scalar, 2, 1> invT2 = invtransform.topLeftCorner(2, 2) * transform.col(2) + invtransform.col(2);
            psopt::rectangle_rectangle_dual_distance(paths + 4 * i, parameters + 8 * i, (rect_mj.template cast<Scalar>()).eval(), (rect2.template cast<Scalar>()).eval(), invR2, invT2);
        }
    }

private:

    bool forward_;
};

// Dual MJ2
template <typename Scalar = double, typename Scalar2 = Scalar>
class FourScenariosProblemDualMJ2 : public VehicleBasicModel<Scalar, Scalar2>
{
public:

    FourScenariosProblemDualMJ2(psopt::ProblemInfo<Scalar2>* prob, const VehicleParam<Scalar2>* vehicleParam, bool forward = true) : VehicleBasicModel<Scalar, Scalar2>(prob, vehicleParam), forward_(forward)
    {
    }

    virtual ~FourScenariosProblemDualMJ2() = default;

    psopt::Problem<adouble, Scalar2>* clone() const override
    {
        FourScenariosProblemDualMJ2<adouble, Scalar2>* prob = new FourScenariosProblemDualMJ2<adouble, Scalar2>(this->problemInfo_, this->vehicleParam_, this->forward_);
        prob->setLinearizedParameters(this);
        return prob;
    }

    void pathAlongTrajectory(Scalar* paths, const Scalar* states,
        const Scalar* /*controls*/, const Scalar* parameters, const Scalar& /*time*/, const Scalar* /*xad*/, std::size_t /*iphase*/) const override
    {
        if (obstacles.empty()) return;
        const Scalar& x = states[0];
        const Scalar& y = states[1];
        const Scalar& theta = states[2];

        // rectangle obstacles
        Eigen::Matrix<Scalar, 2, 3> invtransform = invtransform_2D(x, y, theta);
        if (this->forward_)
            invtransform(0, 2) -= this->vehicleParam_->delta;
        else
            invtransform(0, 2) += this->vehicleParam_->delta;
        Eigen::Matrix<Scalar2, 3, 4> rect2 = Eigen::Matrix<Scalar2, 3, 4>::Ones();
        for (std::size_t i = 0; i < obstacles.size(); i++)
        {
            rect2.topRows(2) = obstacles[i];
            psopt::rectangle_rectangle_MJ_2_Dual_Norm(paths + 2 * i, parameters + 11 * i, (rect_mj.template cast<Scalar>()).eval(), (invtransform * rect2.template cast<Scalar>()).eval());
        }
    }

private:

    bool forward_;
};

int main(int argc, char* argv[])
{
    std::vector<double> v_sampling_time;
    std::vector<double> v_mj_time, v_ac_time, v_duald_time, v_dualmj_time;
    std::vector<double> v_mj_cost, v_ac_cost, v_duald_cost, v_dualmj_cost;

    for (std::size_t iteration = 0; iteration < 100; iteration++)
    {
        std::cout << "Iteration " << iteration << std::endl;

        std::shared_ptr<app::Box2dStateValidityChecker> svc;
        bool parking_forward = false;
        double x0, y0, theta0;
        std::vector<double> trajx, trajy, trajt;
        double sampling_time;
        if (!solve(argc, argv, svc, parking_scenario, parking_forward, x0, y0, theta0, trajx, trajy, trajt, sampling_time)) return -1;
        std::size_t expected_nnodes = trajx.size();

        v_sampling_time.push_back(sampling_time);

        int nobstacle = 2;
        if (parking_scenario == 3)
            nobstacle = 4;

        // MJ2
        {
            time::point timeStart = time::now();

            // basic problem definition
            psopt::MultiSegmentData msdata;
            std::size_t p = 30;
            msdata.nsegments = expected_nnodes / p;
            if (expected_nnodes % p != 0) msdata.nsegments += 1;
            msdata.nstates = 4;
            msdata.ncontrols = 2;
            msdata.nparameters = 19 * nobstacle;
            msdata.ninitial_events = 4;
            msdata.nfinal_events = 4;
            msdata.npaths = 6 * nobstacle;
            msdata.continuous_controls = true;
            msdata.paths_along_trajectory = true;
            msdata.parameters_along_trajectory = true;
            msdata.treat_dynamics_as_cost = false;

            std::size_t nmod = expected_nnodes / msdata.nsegments;
            std::vector<std::size_t> nnodes(msdata.nsegments, nmod);
            nnodes.back() = expected_nnodes - (msdata.nsegments - 1) * (nmod - 1);
            msdata.nnodes.swap(nnodes);
            psopt::ProblemInfo<double>* info = new psopt::ProblemInfo<double>(msdata);
            info->setLinearSolver("ma57");
            info->setTolerance(1.e-8);

            VehicleParam<double> *vehicleParam = new VehicleParam<double>(2.8, 0.96, 0.929, 1.942);
            FourScenariosProblemMJ2<double>* problem = new FourScenariosProblemMJ2<double>(info, vehicleParam, parking_forward);

            four_scenarios_bounds(info, vehicleParam, parking_forward, x0, y0, theta0);
            info->setPhaseLowerBoundStartTime(0.0);
            info->setPhaseUpperBoundStartTime(0.0);
            info->setPhaseLowerBoundEndTime(2.0);
            info->setPhaseUpperBoundEndTime(20.0);
            info->setPhaseConstantGuessParameters(1.0);
            for (std::size_t i = 1; i < msdata.nsegments; i++)
            {
                info->setPhaseLowerBoundStartTime(2.0, i);
                info->setPhaseUpperBoundStartTime(20.0, i);
                info->setPhaseLowerBoundEndTime(2.0, i);
                info->setPhaseUpperBoundEndTime(20.0, i);
                info->setPhaseConstantGuessParameters(1.0, i);
            }
            four_scenarios_guess(info, x0, y0, theta0);

            // interpolated states
            std::vector<double> interp_trajx = trajx;
            std::vector<double> interp_trajy = trajy;
            std::vector<double> interp_trajt = trajt;

            // initialization other trajectory parameters
            std::vector<double> interp_trajalpha(expected_nnodes, 0.0);
            std::vector<double> interp_trajv(expected_nnodes, 0.0);
            std::vector<double> interp_trajomega(expected_nnodes, 0.0);
            for (std::size_t i = 1; i < expected_nnodes; i++)
            {
                interp_trajv[i] = 1.0 * std::hypot(interp_trajx[i] - interp_trajx[i - 1], interp_trajy[i] - interp_trajy[i - 1]); 
                if (interp_trajx[i] < interp_trajx[i - 1]) interp_trajv[i] = -interp_trajv[i];
                if (abs(interp_trajv[i]) > 1.e-6)
                    interp_trajalpha[i] = (interp_trajt[i] - interp_trajt[i - 1]) * vehicleParam->l / interp_trajv[i]; 
            }
            for (std::size_t i = 1; i < expected_nnodes; i++)
            {
                interp_trajomega[i] = (interp_trajalpha[i] - interp_trajalpha[i - 1]); 
            }

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
            }
            interp_trajx.clear();
            interp_trajy.clear();
            interp_trajt.clear();
            interp_trajalpha.clear();
            interp_trajv.clear();
            interp_trajomega.clear();

            double eps = 0.1;
            double Jeps = 0.05;
            bounds_J_2_KKT_Norm_2D(info, nobstacle, 8, 3, eps, Jeps);

            rect_mj << 0.5 * vehicleParam->L, 0.5 * vehicleParam->L, -0.5 * vehicleParam->L, -0.5 * vehicleParam->L, 
                     0.5 * vehicleParam->W, -0.5 * vehicleParam->W, -0.5 * vehicleParam->W, 0.5 * vehicleParam->W; 
            obstacles = generate_obstacles(vehicleParam);

            psopt::Solver<double> solver;
            bool success = solver.solve(problem); // solve with first obstacles
            if (success)
            {
                int trial = 1;
                for (int i = 0; i < trial; i++)
                {
                    eps *= 0.1;
                    bounds_J_2_KKT_Norm_Update_2D(info, nobstacle, 3, eps, Jeps);
                    if (!solver.solveAgain(problem))
                    {
                        success = false;
                        break;
                    }
                }
            }

            if (success)
            {
                double opti_time = time::seconds(time::now() - timeStart);
                v_mj_time.push_back(opti_time);
                v_mj_cost.push_back(solver.getCost());
            }

            delete info;
            delete vehicleParam;
            delete problem;
            info = nullptr;
            vehicleParam = nullptr;
            problem = nullptr;
        }

        // AC 
        {
            time::point timeStart = time::now();

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
            msdata.npaths = 8 * nobstacle;
            msdata.continuous_controls = true;
            msdata.paths_along_trajectory = true;
            msdata.parameters_along_trajectory = true;
            msdata.treat_dynamics_as_cost = false;

            std::size_t nmod = expected_nnodes / msdata.nsegments;
            std::vector<std::size_t> nnodes(msdata.nsegments, nmod);
            nnodes.back() = expected_nnodes - (msdata.nsegments - 1) * (nmod - 1);
            msdata.nnodes.swap(nnodes);
            psopt::ProblemInfo<double>* info = new psopt::ProblemInfo<double>(msdata);
            info->setLinearSolver("ma57");
            info->setTolerance(1.e-8);

            VehicleParam<double>* vehicleParam = new VehicleParam<double>(2.8, 0.96, 0.929, 1.942);
            FourScenariosProblemAreaCriterion<double>* problem = new FourScenariosProblemAreaCriterion<double>(info, vehicleParam);

            four_scenarios_bounds(info, vehicleParam, parking_forward, x0, y0, theta0);
            info->setPhaseLowerBoundStartTime(0.0);
            info->setPhaseUpperBoundStartTime(0.0);
            info->setPhaseLowerBoundEndTime(2.0);
            info->setPhaseUpperBoundEndTime(20.0);
            for (std::size_t i = 1; i < msdata.nsegments; i++)
            {
                info->setPhaseLowerBoundStartTime(2.0, i);
                info->setPhaseUpperBoundStartTime(20.0, i);
                info->setPhaseLowerBoundEndTime(2.0, i);
                info->setPhaseUpperBoundEndTime(20.0, i);
            }
            four_scenarios_guess(info, x0, y0, theta0);

            std::vector<double> interp_trajx = trajx;
            std::vector<double> interp_trajy = trajy;
            std::vector<double> interp_trajt = trajt;

            // initialization other trajectory parameters
            std::vector<double> interp_trajalpha(expected_nnodes, 0.0);
            std::vector<double> interp_trajv(expected_nnodes, 0.0);
            std::vector<double> interp_trajomega(expected_nnodes, 0.0);
            for (std::size_t i = 1; i < expected_nnodes; i++)
            {
                interp_trajv[i] = 1.0 * std::hypot(interp_trajx[i] - interp_trajx[i - 1], interp_trajy[i] - interp_trajy[i - 1]); 
                if (interp_trajx[i] < interp_trajx[i - 1]) interp_trajv[i] = -interp_trajv[i];
                if (abs(interp_trajv[i]) > 1.e-6)
                    interp_trajalpha[i] = (interp_trajt[i] - interp_trajt[i - 1]) * vehicleParam->l / interp_trajv[i]; 
            }
            for (std::size_t i = 1; i < expected_nnodes; i++)
            {
                interp_trajomega[i] = (interp_trajalpha[i] - interp_trajalpha[i - 1]); 
            }

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
            }
            interp_trajx.clear();
            interp_trajy.clear();
            interp_trajt.clear();
            interp_trajalpha.clear();
            interp_trajv.clear();
            interp_trajomega.clear();

            psopt::Solver<double> solver;
            for (std::size_t j = 0; j < msdata.nsegments; j++)
            {
                info->setPhaseConstantLowerBoundPaths(0.025, j);
                info->setPhaseInfinityUpperBoundPaths(j);
            }

            if (parking_forward)
                rect_ac << vehicleParam->l1 + vehicleParam->l, vehicleParam->l1 + vehicleParam->l, -vehicleParam->l2, -vehicleParam->l2, 
                         0.5 * vehicleParam->W, -0.5 * vehicleParam->W, -0.5 * vehicleParam->W, 0.5 * vehicleParam->W; 
            else
                rect_ac << vehicleParam->l2, vehicleParam->l2, -vehicleParam->l1 - vehicleParam->l, -vehicleParam->l1 - vehicleParam->l, 
                         0.5 * vehicleParam->W, -0.5 * vehicleParam->W, -0.5 * vehicleParam->W, 0.5 * vehicleParam->W;
            obstacles = generate_obstacles(vehicleParam);
            // info->setUseLinearizedDae(true);
            // problem->setUpLinearizedParameters(info->getVariables().data());
            bool success = solver.solve(problem);

            if (success)
            {
                double opti_time = time::seconds(time::now() - timeStart);
                v_ac_time.push_back(opti_time);
                v_ac_cost.push_back(solver.getCost());
            }

            delete info;
            delete vehicleParam;
            delete problem;
            info = nullptr;
            vehicleParam = nullptr;
            problem = nullptr;
        }

        // Dual Distance
        {
            time::point timeStart = time::now();

            // basic problem definition
            psopt::MultiSegmentData msdata;
            std::size_t p = 30;
            msdata.nsegments = expected_nnodes / p;
            if (expected_nnodes % p != 0) msdata.nsegments += 1;
            msdata.nstates = 4;
            msdata.ncontrols = 2;
            msdata.nparameters = 8 * nobstacle;
            msdata.ninitial_events = 4;
            msdata.nfinal_events = 4;
            msdata.npaths = 4 * nobstacle;
            msdata.continuous_controls = true;
            msdata.paths_along_trajectory = true;
            msdata.parameters_along_trajectory = true;
            msdata.treat_dynamics_as_cost = false;

            std::size_t nmod = expected_nnodes / msdata.nsegments;
            std::vector<std::size_t> nnodes(msdata.nsegments, nmod);
            nnodes.back() = expected_nnodes - (msdata.nsegments - 1) * (nmod - 1);
            msdata.nnodes.swap(nnodes);
            psopt::ProblemInfo<double>* info = new psopt::ProblemInfo<double>(msdata);
            info->setLinearSolver("ma57");
            info->setTolerance(1.e-8);

            VehicleParam<double> *vehicleParam = new VehicleParam<double>(2.8, 0.96, 0.929, 1.942);
            FourScenariosProblemDualDistance<double>* problem = new FourScenariosProblemDualDistance<double>(info, vehicleParam, parking_forward);

            four_scenarios_bounds(info, vehicleParam, parking_forward, x0, y0, theta0);
            info->setPhaseLowerBoundStartTime(0.0);
            info->setPhaseUpperBoundStartTime(0.0);
            info->setPhaseLowerBoundEndTime(2.0);
            info->setPhaseUpperBoundEndTime(20.0);
            for (std::size_t i = 1; i < msdata.nsegments; i++)
            {
                info->setPhaseLowerBoundStartTime(2.0, i);
                info->setPhaseUpperBoundStartTime(20.0, i);
                info->setPhaseLowerBoundEndTime(2.0, i);
                info->setPhaseUpperBoundEndTime(20.0, i);
            }
            four_scenarios_guess(info, x0, y0, theta0);

            std::vector<double> interp_trajx = trajx;
            std::vector<double> interp_trajy = trajy;
            std::vector<double> interp_trajt = trajt;

            // initialization other trajectory parameters
            std::vector<double> interp_trajalpha(expected_nnodes, 0.0);
            std::vector<double> interp_trajv(expected_nnodes, 0.0);
            std::vector<double> interp_trajomega(expected_nnodes, 0.0);
            for (std::size_t i = 1; i < expected_nnodes; i++)
            {
                interp_trajv[i] = 1.0 * std::hypot(interp_trajx[i] - interp_trajx[i - 1], interp_trajy[i] - interp_trajy[i - 1]); 
                if (interp_trajx[i] < interp_trajx[i - 1]) interp_trajv[i] = -interp_trajv[i];
                if (abs(interp_trajv[i]) > 1.e-6)
                    interp_trajalpha[i] = (interp_trajt[i] - interp_trajt[i - 1]) * vehicleParam->l / interp_trajv[i]; 
            }
            for (std::size_t i = 1; i < expected_nnodes; i++)
            {
                interp_trajomega[i] = (interp_trajalpha[i] - interp_trajalpha[i - 1]); 
            }

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
            }
            interp_trajx.clear();
            interp_trajy.clear();
            interp_trajt.clear();
            interp_trajalpha.clear();
            interp_trajv.clear();
            interp_trajomega.clear();

            double safety_distance = 0.05;
            psopt::Solver<double> solver;
            for (std::size_t j = 0; j < msdata.nsegments; j++)
            {
                info->setPhaseConstantGuessParameters(1.0, j);
            }
            bounds_dual_distance(info, nobstacle, 8, safety_distance);

            obstacles_duald = generate_dual_obstacles(vehicleParam);
            bool success = solver.solve(problem); // solve with first obstacles

            if (success)
            {
                double opti_time = time::seconds(time::now() - timeStart);
                v_duald_time.push_back(opti_time);
                v_duald_cost.push_back(solver.getCost());
            }

            delete info;
            delete vehicleParam;
            delete problem;
            info = nullptr;
            vehicleParam = nullptr;
            problem = nullptr;
        }

        // Dual MJ2
        {
            time::point timeStart = time::now();

            // basic problem definition
            psopt::MultiSegmentData msdata;
            std::size_t p = 30;
            msdata.nsegments = expected_nnodes / p;
            if (expected_nnodes % p != 0) msdata.nsegments += 1;
            msdata.nstates = 4;
            msdata.ncontrols = 2;
            msdata.nparameters = 11 * nobstacle;
            msdata.ninitial_events = 4;
            msdata.nfinal_events = 4;
            msdata.npaths = 2 * nobstacle;
            msdata.continuous_controls = true;
            msdata.paths_along_trajectory = true;
            msdata.parameters_along_trajectory = true;
            msdata.treat_dynamics_as_cost = false;

            std::size_t nmod = expected_nnodes / msdata.nsegments;
            std::vector<std::size_t> nnodes(msdata.nsegments, nmod);
            nnodes.back() = expected_nnodes - (msdata.nsegments - 1) * (nmod - 1);
            msdata.nnodes.swap(nnodes);
            psopt::ProblemInfo<double>* info = new psopt::ProblemInfo<double>(msdata);
            info->setLinearSolver("ma57");
            info->setTolerance(1.e-8);

            VehicleParam<double> *vehicleParam = new VehicleParam<double>(2.8, 0.96, 0.929, 1.942);
            FourScenariosProblemDualMJ2<double>* problem = new FourScenariosProblemDualMJ2<double>(info, vehicleParam, parking_forward);

            four_scenarios_bounds(info, vehicleParam, parking_forward, x0, y0, theta0);
            info->setPhaseLowerBoundStartTime(0.0);
            info->setPhaseUpperBoundStartTime(0.0);
            info->setPhaseLowerBoundEndTime(2.0);
            info->setPhaseUpperBoundEndTime(20.0);
            for (std::size_t i = 1; i < msdata.nsegments; i++)
            {
                info->setPhaseLowerBoundStartTime(2.0, i);
                info->setPhaseUpperBoundStartTime(20.0, i);
                info->setPhaseLowerBoundEndTime(2.0, i);
                info->setPhaseUpperBoundEndTime(20.0, i);
            }
            four_scenarios_guess(info, x0, y0, theta0);

            std::vector<double> interp_trajx = trajx;
            std::vector<double> interp_trajy = trajy;
            std::vector<double> interp_trajt = trajt;

            // initialization other trajectory parameters
            std::vector<double> interp_trajalpha(expected_nnodes, 0.0);
            std::vector<double> interp_trajv(expected_nnodes, 0.0);
            std::vector<double> interp_trajomega(expected_nnodes, 0.0);
            for (std::size_t i = 1; i < expected_nnodes; i++)
            {
                interp_trajv[i] = 1.0 * std::hypot(interp_trajx[i] - interp_trajx[i - 1], interp_trajy[i] - interp_trajy[i - 1]); 
                if (interp_trajx[i] < interp_trajx[i - 1]) interp_trajv[i] = -interp_trajv[i];
                if (abs(interp_trajv[i]) > 1.e-6)
                    interp_trajalpha[i] = (interp_trajt[i] - interp_trajt[i - 1]) * vehicleParam->l / interp_trajv[i]; 
            }
            for (std::size_t i = 1; i < expected_nnodes; i++)
            {
                interp_trajomega[i] = (interp_trajalpha[i] - interp_trajalpha[i - 1]); 
            }

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
            }
            interp_trajx.clear();
            interp_trajy.clear();
            interp_trajt.clear();
            interp_trajalpha.clear();
            interp_trajv.clear();
            interp_trajomega.clear();

            double Jeps = 0.05;
            psopt::Solver<double> solver;

            for (std::size_t j = 0; j < msdata.nsegments; j++)
            {
                info->setPhaseConstantGuessParameters(1.0, j);
            }
            double eps = 0.1;
            bounds_J_2_Dual_Norm_2D(info, nobstacle, 8, 3, eps, Jeps);

            rect_mj << 0.5 * vehicleParam->L, 0.5 * vehicleParam->L, -0.5 * vehicleParam->L, -0.5 * vehicleParam->L, 
                     0.5 * vehicleParam->W, -0.5 * vehicleParam->W, -0.5 * vehicleParam->W, 0.5 * vehicleParam->W; 
            obstacles = generate_obstacles(vehicleParam);

            bool success = solver.solve(problem); // solve with first obstacles
            if (success)
            {
                // info->setUseLinearizedDae(true);
                int trial = 1;
                for (int i = 0; i < trial; i++)
                {
                    eps *= 0.1;
                    bounds_J_2_Dual_Norm_Update_2D(info, nobstacle, eps, Jeps);
                    // problem->setUpLinearizedParameters(info->getVariables().data());
                    if (!solver.solveAgain(problem))
                    {
                        success = false;
                        break;
                    }
                }
            }

            if (success)
            {
                double opti_time = time::seconds(time::now() - timeStart);
                v_dualmj_time.push_back(opti_time);
                v_dualmj_cost.push_back(solver.getCost());
            }

            delete info;
            delete vehicleParam;
            delete problem;
            info = nullptr;
            vehicleParam = nullptr;
            problem = nullptr;
        }
    }

    int save_num = 0;
    {
        std::ofstream ofs(boost::str(boost::format("mj_time_%i.txt") % save_num).c_str());
        for (const double & time : v_mj_time)
            ofs << time << " ";
        ofs.close();

        ofs.open(boost::str(boost::format("mj_cost_%i.txt") % save_num).c_str());
        for (const double & cost : v_mj_cost)
            ofs << cost << " ";
        ofs.close();
    }

    {
        std::ofstream ofs(boost::str(boost::format("ac_time_%i.txt") % save_num).c_str());
        for (const double & time : v_ac_time)
            ofs << time << " ";
        ofs.close();

        ofs.open(boost::str(boost::format("ac_cost_%i.txt") % save_num).c_str());
        for (const double & cost : v_ac_cost)
            ofs << cost << " ";
        ofs.close();
    }

    {
        std::ofstream ofs(boost::str(boost::format("duald_time_%i.txt") % save_num).c_str());
        for (const double & time : v_duald_time)
            ofs << time << " ";
        ofs.close();

        ofs.open(boost::str(boost::format("duald_cost_%i.txt") % save_num).c_str());
        for (const double & cost : v_duald_cost)
            ofs << cost << " ";
        ofs.close();
    }

    {
        std::ofstream ofs(boost::str(boost::format("dualmj_time_%i.txt") % save_num).c_str());
        for (const double & time : v_dualmj_time)
            ofs << time << " ";
        ofs.close();

        ofs.open(boost::str(boost::format("dualmj_cost_%i.txt") % save_num).c_str());
        for (const double & cost : v_dualmj_cost)
            ofs << cost << " ";
        ofs.close();
    }

    return 0;
}
