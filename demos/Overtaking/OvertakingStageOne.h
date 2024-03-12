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

#include <psopt/problem.hpp>
#include <psopt/solver.hpp>
#include <psopt/interpolation.hpp>

#include <bomp/utils.hpp>
#include <bomp/overtaking/overtaking_basic_setup.h>

#include <ompl/util/Console.h>
#include <ompl/util/Time.h>

#include <fstream>

template <typename Scalar = double, typename Scalar2 = Scalar>
class OvertakingProblemStageOne : public OvertakingProblemBase<Scalar, Scalar2>
{
public:

    OvertakingProblemStageOne(psopt::ProblemInfo<Scalar2>* prob, const VehicleParam<Scalar2>* vehicleParam) : OvertakingProblemBase<Scalar, Scalar2>(prob, vehicleParam)
    {
    }

    virtual ~OvertakingProblemStageOne() = default;

    psopt::Problem<adouble, Scalar2>* clone() const override
    {
        OvertakingProblemStageOne<adouble, Scalar2>* prob = new OvertakingProblemStageOne<adouble, Scalar2>(this->problemInfo_, this->vehicleParam_);
        prob->setWoObstacles(wo_obstacles_);
        prob->setRectangle(rect1_);
        prob->setLinearizedParameters(this);
        return prob;
    }

    void setRectangle(const Eigen::Matrix<double, 2, 4> & rect1)
    {
        rect1_ = rect1;
    }

    void setWoObstacles(bool value)
    {
        wo_obstacles_ = value;
    }

    void pathAlongTrajectory(Scalar* paths, const Scalar* states,
        const Scalar* /*controls*/, const Scalar* parameters, const Scalar& time, const Scalar* /*xad*/, std::size_t /*iphase*/) const override
    {
        if (wo_obstacles_) return;

        const Scalar& x = states[0];
        const Scalar& y = states[1];
        const Scalar& theta = states[2];

        // rectangle obstacles
        Eigen::Matrix<Scalar, 2, 3> invtransform = invtransform_2D(x, y, theta);
        invtransform(0, 2) -= this->vehicleParam_->delta;
        Eigen::Matrix<Scalar, 3, 4> rect2 = Eigen::Matrix<Scalar, 3, 4>::Ones();
        rect2.topRows(2) = (rect1_.template cast<Scalar>()).eval();

        rect2.topRows(1).array() += (10.0 + this->vehicleParam_->delta); // overtaking_case == 0
        if (overtaking_case >= 1) rect2.topRows(1).array() += 10.0 * time;
        psopt::rectangle_rectangle_MJ_2_KKT_Norm(paths, parameters, (rect1_.template cast<Scalar>()).eval(), (invtransform * rect2).eval());
        if (overtaking_case == 2)
        {
            rect2.topRows(1).array() += 10.0;
            psopt::rectangle_rectangle_MJ_2_KKT_Norm(paths + 6, parameters + 19, (rect1_.template cast<Scalar>()).eval(), (invtransform * rect2).eval());
        }
    }

protected:
    bool wo_obstacles_{false};

    Eigen::Matrix<double, 2, 4> rect1_;
};

bool argParse(int argc, char** argv, int & overtaking_case, std::size_t & expected_nnodes)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("case,c", bpo::value<int>()->default_value(0), "(Optional) Specify the parking case.")
        ("expected_nnodes", bpo::value<std::size_t>()->default_value(30), "(Optional) Specify the discrete nnodes");
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);
    // Check if the help flag has been given:
    if (vm.count("help") != 0u)
    {
        std::cout << desc << std::endl;
        return false;
    }

    overtaking_case = vm["case"].as<int>();
    expected_nnodes = vm["expected_nnodes"].as<std::size_t>();
    return true;
}

bool solve(int argc, char* argv[], psopt::MultiSegmentData & msdata, psopt::Solver<double> & solver, double & timeUsed)
{
    std::size_t expected_nnodes = 30;
    if (!argParse(argc, argv, overtaking_case, expected_nnodes)) return false;

    ompl::time::point timeStart = ompl::time::now();

    msdata.nsegments = 1;
    msdata.nstates = 4;
    msdata.ncontrols = 2;
    msdata.nparameters = 0;
    msdata.ninitial_events = 4;
    msdata.nfinal_events = 4;
    msdata.npaths = 0;
    msdata.continuous_controls = false;
    msdata.paths_along_trajectory = true;
    msdata.parameters_along_trajectory = true;
    msdata.nnodes.push_back(expected_nnodes);
    psopt::ProblemInfo<double> * info = new psopt::ProblemInfo<double>(msdata);
    info->setLinearSolver("ma57");
    info->setTolerance(1.e-8);

    overtaking_bounds(info);
    info->setPhaseLowerBoundStartTime(0.0);
    info->setPhaseUpperBoundStartTime(0.0);
    info->setPhaseLowerBoundEndTime(0.0);
    info->setPhaseUpperBoundEndTime(10.0);
    overtaking_guess(info);

    VehicleParam<double> *vehicleParam = new VehicleParam<double>(2.8, 0.96, 0.929, 1.942);
    OvertakingProblemStageOne<double>* problem = new OvertakingProblemStageOne<double>(info, vehicleParam);
    problem->setWoObstacles(true);

    info->setPhaseLowerBoundEndTime(6.0, msdata.nsegments - 1);
    info->setPhaseUpperBoundEndTime(6.0, msdata.nsegments - 1);
    if (!solver.solve(problem)) // initialization without obstacles
    {
        delete info;
        delete vehicleParam;
        delete problem;
        info = nullptr;
        vehicleParam = nullptr;
        problem = nullptr;
        return false;
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

    int nobstacle = 1;
    if (overtaking_case == 2)
        nobstacle = 2;
    for (std::size_t j = 0; j < msdata.nsegments; j++)
    {
        info->setPhaseNumberPaths(6 * nobstacle, j);
        info->setPhaseNumberParameters(19 * nobstacle, j);
        info->setPhaseConstantGuessParameters(1.0, j);

        info->setPhaseGuessStates(solver.getPhaseStates(j), j);
        info->setPhaseGuessControls(solver.getPhaseControls(j), j);
        info->setPhaseGuessTime(solver.getPhaseTime(j), j);
    }
    double eps = 0.1;
    double Jeps = 0.05;
    bounds_J_2_KKT_Norm_2D(info, nobstacle, 8, 3, eps, Jeps);

    Eigen::Matrix<double, 2, 4> rect1;
    rect1 << 0.5 * vehicleParam->L, 0.5 * vehicleParam->L, -0.5 * vehicleParam->L, -0.5 * vehicleParam->L, 
             0.5 * vehicleParam->W, -0.5 * vehicleParam->W, -0.5 * vehicleParam->W, 0.5 * vehicleParam->W; 
    problem->setRectangle(rect1);
    problem->setWoObstacles(false);
    if (!solver.solve(problem)) // solve with first obstacles 
    {
        delete info;
        delete vehicleParam;
        delete problem;
        info = nullptr;
        vehicleParam = nullptr;
        problem = nullptr;
        return false;
    }

    int trial = 1;
    // info->setUseLinearizedDae(true); // TODO
    for (int i = 0; i < trial; i++)
    {
        eps *= 0.1;
        bounds_J_2_KKT_Norm_Update_2D(info, nobstacle, 3, eps, Jeps);
        // problem->setUpLinearizedParameters(info->getVariables().data()); // TODO
        if (!solver.solveAgain(problem))
        {
            delete info;
            delete vehicleParam;
            delete problem;
            info = nullptr;
            vehicleParam = nullptr;
            problem = nullptr;
            return false;
        }
    }

    /*
    // trajectory 
    std::vector<double> trajx, trajy, trajt, trajalpha, trajv, trajomega;
    const double * states = solver.getPhaseStates().data();
    const double * controls = solver.getPhaseControls().data();
    for (std::size_t j = 0; j < expected_nnodes; j++)
    {
        trajx.push_back(states[0]);
        trajy.push_back(states[1]);
        trajt.push_back(states[2]);
        trajalpha.push_back(states[3]);
        trajv.push_back(controls[0]);
        trajomega.push_back(controls[1]);
        states += msdata.nstates;
        controls += msdata.ncontrols;
    }

    // interpolation
    std::vector<double> vtime = info->getNodes(expected_nnodes - 1);
    expected_nnodes = 80; // TODO
    std::vector<double> ptime = info->getNodes(expected_nnodes - 1);
    psopt::LinearInterpolation<double, double> linearInterp(vtime, ptime);
    std::vector<double> interp_trajx = linearInterp.interpolate(trajx);
    std::vector<double> interp_trajy = linearInterp.interpolate(trajy);
    std::vector<double> interp_trajt = linearInterp.interpolate(trajt);
    std::vector<double> interp_trajalpha = linearInterp.interpolate(trajalpha);
    std::vector<double> interp_trajv = linearInterp.interpolate(trajv);
    std::vector<double> interp_trajomega = linearInterp.interpolate(trajomega);
    std::vector<double> interp_time = linearInterp.interpolate(solver.getPhaseTime());

    std::size_t p = 30;
    msdata.nsegments = expected_nnodes / p;
    if (expected_nnodes % p != 0) msdata.nsegments += 1;
    // msdata.nsegments = 1; // TODO
    std::size_t nmod = expected_nnodes / msdata.nsegments;
    std::vector<std::size_t> nnodes(msdata.nsegments, nmod);
    nnodes.back() = expected_nnodes - (msdata.nsegments - 1) * (nmod - 1);
    msdata.nnodes.swap(nnodes);

    std::size_t start = 0;
    solver.setPhaseNumbers(msdata.nsegments);
    for (std::size_t k = 0; k < msdata.nsegments; k++)
    {
        std::vector<double> guess_states, guess_controls, guess_time;
        guess_states.reserve(msdata.nstates * msdata.nnodes[k]);
        guess_controls.reserve(msdata.ncontrols * msdata.nnodes[k]);
        guess_time.reserve(msdata.nnodes[k]);
        for (std::size_t i = 0; i < msdata.nnodes[k]; i++)
        {
            std::size_t offset = start + i;
            guess_states.push_back(interp_trajx[offset]);
            guess_states.push_back(interp_trajy[offset]);
            guess_states.push_back(interp_trajt[offset]);
            guess_states.push_back(interp_trajalpha[offset]);
            guess_controls.push_back(interp_trajv[offset]);
            guess_controls.push_back(interp_trajomega[offset]);
            guess_time.push_back(interp_time[offset]);
            for (std::size_t j = 4; j < msdata.nstates; j++)
                guess_states.push_back(0.0);
            for (std::size_t j = 2; j < msdata.ncontrols; j++)
                guess_controls.push_back(0.0);
        }
        start += (msdata.nnodes[k] - 1);
        solver.setPhaseStates(guess_states, k);
        solver.setPhaseControls(guess_controls, k);
        solver.setPhaseTime(guess_time, k);
    }
    */

    timeUsed = ompl::time::seconds(ompl::time::now() - timeStart);
    OMPL_INFORM("Time used %.4f", timeUsed);

    if (false)
    {
        std::ofstream ofs;
        ofs.open("states_stage1.txt");
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

        ofs.open("controls_stage1.txt");
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

        ofs.open("time_stage1.txt");
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
    delete problem;
    info = nullptr;
    vehicleParam = nullptr;
    problem = nullptr;

    return true;
}
