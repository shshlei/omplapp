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

#include "DrivingWoObstacles.h" 

#include <fstream>

int main(int argc, char* argv[])
{
    std::size_t expected_nnodes = 20; 
    // Parse the arguments
    if (!argParse(argc, argv, expected_nnodes)) return -1;

    // basic problem definition
    psopt::MultiSegmentData msdata;
    msdata.nsegments = 1;
    msdata.nstates = 4;
    msdata.ncontrols = 2;
    msdata.nparameters = 0;
    msdata.ninitial_events = 4;
    msdata.nfinal_events = 4;
    msdata.npaths = 0;
    msdata.continuous_controls = false;
    msdata.parameters_along_trajectory = false;
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

    double x0, y0, theta0, xf, yf, thetaf;
    setStartAndGoal(x0, y0, theta0, xf, yf, thetaf);

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

    // active parameters and paths
    info->setPathsAlongTrajectory(false);
    info->setParametersAlongTrajectory(false);

    VehicleParam<double> *vehicleParam = new VehicleParam<double>(0.028, 0.0096, 0.00929, 0.01942);
    DrivingProblemBase<double>* problem = new DrivingProblemBase<double>(info, vehicleParam);

    psopt::Solver<double> solver;
    if (solver.solve(problem))
    {
        std::ofstream ofs;
        ofs.open("states.txt");
        const std::vector<double> & states = solver.getPhaseStates();
        for (std::size_t j = 0; j < msdata.nnodes[0]; j++)
        {
            std::size_t offset = j * msdata.nstates;
            for (std::size_t i = 0; i < msdata.nstates; i++)
            {
                ofs << states[offset + i] << " ";
            }
            ofs << std::endl;
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

    delete info;
    delete vehicleParam;
    delete problem;
    info = nullptr;
    vehicleParam = nullptr;
    problem = nullptr;

    return 0;
}
