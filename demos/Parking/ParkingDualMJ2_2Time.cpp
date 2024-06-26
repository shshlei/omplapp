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

#include <psopt/optimal_control_problem.hpp>
#include <psopt/optimal_control_solver.hpp>

#include <bomp/utils.hpp>
#include <bomp/vehicle_basic_model/vehicle_basic_model.hpp>
#include <bomp/collision_constraints/dual_J_function_collision_constraints_2.h>
#include <bomp/four_scenarios_parking/four_scenarios_basic_setup.hpp>

Eigen::Matrix<double, 2, 1> rect1, rect2;
std::vector<Eigen::Matrix<double, 2, 3>> obstacles;

template <typename Scalar = double, typename Scalar2 = Scalar>
class FourScenariosProblemDualMJ2_2 : public VehicleBasicModel<Scalar, Scalar2>
{
public:

    FourScenariosProblemDualMJ2_2(psopt::OptimalControlProblemInfo<Scalar2>* prob, const VehicleParam<Scalar2>* vehicleParam, bool forward = true) : VehicleBasicModel<Scalar, Scalar2>(prob, vehicleParam), forward_(forward)
    {
    }

    virtual ~FourScenariosProblemDualMJ2_2() = default;

    psopt::OptimalControlProblem<adouble, Scalar2>* clone() const override
    {
        FourScenariosProblemDualMJ2_2<adouble, Scalar2>* prob = new FourScenariosProblemDualMJ2_2<adouble, Scalar2>(this->problemInfo_, this->vehicleParam_, this->forward_);
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

        for (std::size_t i = 0; i < obstacles.size(); i++)
        {
            Eigen::Matrix<Scalar, 2, 3> transform = obstacles[i].template cast<Scalar>();
            Eigen::Matrix<Scalar, 2, 2> R2 = invtransform.topLeftCorner(2, 2) * transform.topLeftCorner(2, 2); // vehicle 2 obstacle
            Eigen::Matrix<Scalar, 2, 1> T2 = invtransform.topLeftCorner(2, 2) * transform.col(2) + invtransform.col(2);
            psopt::rectangle_rectangle_dual_MJ2(paths + 4 * i, parameters + 8 * i, (rect1.template cast<Scalar>()).eval(), (rect2.template cast<Scalar>()).eval(), R2, T2);
        }
    }

private:

    bool forward_;
};

int main(int argc, char* argv[])
{
    /*
    bool parking_forward;
    int parking_scenario;
    int parking_case;
    std::size_t o_expected_nnodes;
    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, parking_forward, parking_scenario, parking_case, o_expected_nnodes)) return -1;
    */

    bool parking_forward = false;
    int parking_scenario = 1;
    std::size_t o_expected_nnodes = 20;
    for (int parking_case = 0; parking_case < 3; parking_case++)
    {
    std::ofstream ofs(boost::str(boost::format("mj_dual_2_time_cost_%i_%i.txt") % parking_scenario % parking_case).c_str());
    for (int iteration = 0; iteration < 100; iteration++)
    {
        OMPL_INFORM("Current iteration %d", iteration);

        std::shared_ptr<app::Box2dStateValidityChecker> svc;
        double x0, y0, theta0;
        std::vector<double> trajx, trajy, trajt;
        double sampling_time;
        if (!solve(parking_forward, parking_scenario, parking_case, o_expected_nnodes, svc, x0, y0, theta0, trajx, trajy, trajt, sampling_time)) continue;
        std::size_t expected_nnodes = trajx.size();

        time::point timeStart = time::now();

        int nobstacle = 2;
        if (parking_scenario == 3)
            nobstacle = 4;

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
        psopt::OptimalControlProblemInfo<double>* info = new psopt::OptimalControlProblemInfo<double>(msdata);
        info->setLinearSolver("ma57");
        info->setTolerance(1.e-8);

        VehicleParam<double> *vehicleParam = new VehicleParam<double>(2.8, 0.96, 0.929, 1.942);
        FourScenariosProblemDualMJ2_2<double>* problem = new FourScenariosProblemDualMJ2_2<double>(info, vehicleParam, parking_forward);

        four_scenarios_bounds(info, vehicleParam, parking_forward, parking_scenario, x0, y0, theta0);
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

        std::vector<double> interp_trajx;
        std::vector<double> interp_trajy;
        std::vector<double> interp_trajt;
        interp_trajx.swap(trajx);
        interp_trajy.swap(trajy);
        interp_trajt.swap(trajt);

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

        double eps = 0.01;
        double Jeps = 0.05;
        bounds_dual_MJ2(info, nobstacle, 8, Jeps);

        obstacles = generate_dual_obstacles(vehicleParam, parking_scenario);
        rect1 = Eigen::Matrix<double, 2, 1>(0.5 * vehicleParam->L, 0.5 * vehicleParam->W);
        rect2 = rect1;
        if (parking_scenario == 2)
        {
            rect2[0] = 0.5 * vehicleParam->W;
            rect2[1] = 0.5 * vehicleParam->L;
        }

        psopt::OptimalControlSolver<double> solver;
        bool success = solver.solve(problem); // solve with first obstacles

        delete info;
        delete vehicleParam;
        delete problem;
        info = nullptr;
        vehicleParam = nullptr;
        problem = nullptr;

        if (success)
        {
            double opti_time = time::seconds(time::now() - timeStart);
            ofs << sampling_time << " " << opti_time << " " << solver.getCost() << std::endl;
        }
    }
    }

    return 0;
}
