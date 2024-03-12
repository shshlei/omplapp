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

#include "DifferentialDriving.hpp" 

#include <box2d_collision/b2_bvh_manager.h>
#include <box2d_collision/b2_distance.h>

#include <psopt/solver.hpp>

#include <ompl/util/Console.h>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

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
    apoint.normal = 100.0 * normal.normalized(); // TODO
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

int main()
{
  int num_iterative_ = 1;

  double active_dist_ = 2.0;

  double vehicle_half_length_ = 0.3;

  double vehicle_half_width_ = 0.3;

  b2RectangleShape * rect_{nullptr};

  std::shared_ptr<b2BVHManager> svc_;

  psopt::ProblemInfo<double>* info_{nullptr};

  DifferentialDriving<double>* problem_{nullptr};

  // local vehicle rectangle
  rect_ = new b2RectangleShape(vehicle_half_length_, vehicle_half_width_);

  // collision environment
  std::string env_shapes_file = "/home/shshlei/Autoware-Experiment/src/vox_nav/vox_nav_planning/config/env_shapes.ply";
  svc_ = std::make_shared<b2BVHManager>();
  svc_->setEnvironmentFile(env_shapes_file);

  // basic problem definition
  psopt::MultiSegmentData msdata;
  msdata.nsegments = 1;
  msdata.nstates = 3;
  msdata.ncontrols = 2;
  msdata.nparameters = 0;
  msdata.ninitial_events = 4;
  msdata.nfinal_events = 4;
  msdata.npaths = 0;
  msdata.continuous_controls = true;
  msdata.paths_along_trajectory = false;
  msdata.parameters_along_trajectory = false;
  msdata.treat_dynamics_as_cost = false;
  msdata.nnodes.push_back(20);
  info_ = new psopt::ProblemInfo<double>(msdata);
  info_->setLinearSolver("ma57");
  info_->setTolerance(1.e-8);

  // optimization problem
  problem_ = new DifferentialDriving<double>(info_);

  // bounds
  double minx = -5.0;
  double miny = -8.0;
  double maxx = 30.0;
  double maxy = 1.0;
  double maxv = 3.0;
  double maxo = 1.0;

  std::vector<double> slower{minx, miny, -1.1 * M_PI};
  std::vector<double> supper{maxx, maxy, 1.1 * M_PI};
  std::vector<double> clower{-maxv, -maxo};
  std::vector<double> cupper{maxv, maxo};
  for (std::size_t i = 0; i < info_->getPhaseNumbers(); i++)
  {
    info_->setPhaseLowerBoundStates(slower, i);
    info_->setPhaseUpperBoundStates(supper, i);
    info_->setPhaseLowerBoundControls(clower, i);
    info_->setPhaseUpperBoundControls(cupper, i);
  }

  info_->setPhaseLowerBoundStartTime(0.0);
  info_->setPhaseUpperBoundStartTime(0.0);
  info_->setPhaseLowerBoundEndTime(2.0);
  info_->setPhaseUpperBoundEndTime(200.0);
  for (std::size_t i = 1; i < info_->getPhaseNumbers(); i++)
  {
    info_->setPhaseLowerBoundStartTime(2.0, i);
    info_->setPhaseUpperBoundStartTime(200.0, i);
    info_->setPhaseLowerBoundEndTime(2.0, i);
    info_->setPhaseUpperBoundEndTime(200.0, i);
  }

  // interpolated states
  std::vector<double> interp_trajx{6.1446,  7.34047,  8.27391,  9.48763, 10.1178, 11.4036, 12.5512, 13.7449 , 14.4113, 15.6944 , 16.3085, 17.4767, 18.5757 , 19.0524, 20.0162, 20.4309 , 21.1067, 21.9408, 23.5313};
  std::vector<double> interp_trajy{-2.08101, -3.60348, -4.24535, -3.28806, -3.613, -4.05119, -5.14293, -4.73489, -4.4662, -3.09255, -4.00959, -3.92188, -2.96567, -4.22079, -4.07115, -6.43546, -6.02755, -7.29654, -6.74568};
  std::vector<double> interp_trajt{-0.231646, -0.660765, -0.530362, -0.467959, -0.459306, -0.438025,  0.0747913,  0.363017, 0.373143,  0.146714, 0.0801819, 0.0651839, -0.380557  , -0.655674, -0.733082, -0.734475, -0.677983, -0.287284, -0.00459961};
  std::size_t expected_nnodes = interp_trajx.size();

  info_->setPhaseNumberNodes(expected_nnodes);

  // initialization other trajectory parameters
  std::vector<double> interp_trajv(expected_nnodes, 0.0);
  std::vector<double> interp_trajomega(expected_nnodes, 0.0);
  for (std::size_t i = 1; i < expected_nnodes; i++)
  {
    interp_trajv[i] = 0.1 * std::hypot(interp_trajx[i] - interp_trajx[i - 1], interp_trajy[i] - interp_trajy[i - 1]); 
    if (interp_trajx[i] < interp_trajx[i - 1]) interp_trajv[i] = -interp_trajv[i];
    interp_trajomega[i] = 0.1 * (interp_trajt[i] - interp_trajt[i - 1]); 
  }

  // start end event bounds
  double x0 = interp_trajx[0];
  double y0 = interp_trajy[0];
  double theta0 = interp_trajt[0];
  double xf = interp_trajx.back();
  double yf = interp_trajy.back();
  double thetaf = interp_trajt.back();
  if (info_->getPhaseNumbers() == 1)
  {
    double epsf = 0.1;
    double epsthetaf = 0.035;
    std::vector<double> elower{x0, y0, theta0, 0.0, xf - epsf, yf - epsf, thetaf - epsthetaf, -epsf};
    std::vector<double> eupper{x0, y0, theta0, 0.0, xf + epsf, yf + epsf, thetaf + epsthetaf, epsf};
    info_->setPhaseLowerBoundEvents(elower);
    info_->setPhaseUpperBoundEvents(eupper);
  }
  else
  {
    std::vector<double> elower{x0, y0, theta0, 0.0};
    std::vector<double> eupper{x0, y0, theta0, 0.0};
    info_->setPhaseLowerBoundEvents(elower);
    info_->setPhaseUpperBoundEvents(eupper);

    double epsf = 0.001;
    double epsthetaf = 0.035;
    std::vector<double> felower{xf - epsf, yf - epsf, thetaf - epsthetaf, -epsf};
    std::vector<double> feupper{xf + epsf, yf + epsf, thetaf + epsthetaf, epsf};
    info_->setPhaseLowerBoundEvents(felower, info_->getPhaseNumbers() - 1);
    info_->setPhaseUpperBoundEvents(feupper, info_->getPhaseNumbers() - 1);
  }

  const std::size_t nphases = info_->getPhaseNumbers();
  const std::size_t nstates = info_->getPhaseNumberStates();
  const std::size_t ncontrols = info_->getPhaseNumberControls();

  // initial guess states
  psopt::Solver<double> solver;
  solver.setPhaseNumbers(nphases);
  std::size_t start = 0;
  for (std::size_t k = 0; k < nphases; k++)
  {
    std::vector<double> guess_states, guess_controls;
    guess_states.reserve(nstates * info_->getPhaseNumberNodes(k));
    guess_controls.reserve(ncontrols * info_->getPhaseNumberNodes(k));
    for (std::size_t i = 0; i < info_->getPhaseNumberNodes(k); i++)
    {
      std::size_t offset = start + i;
      guess_states.push_back(interp_trajx[offset]);
      guess_states.push_back(interp_trajy[offset]);
      guess_states.push_back(interp_trajt[offset]);
      guess_controls.push_back(interp_trajv[offset]);
      guess_controls.push_back(interp_trajomega[offset]);
      for (std::size_t j = 3; j < nstates; j++)
        guess_states.push_back(0.0);
      for (std::size_t j = 2; j < ncontrols; j++)
        guess_controls.push_back(0.0);
    }
    start += (info_->getPhaseNumberNodes(k) - 1);
    info_->setPhaseGuessStates(guess_states, k);
    info_->setPhaseGuessControls(guess_controls, k);
    solver.setPhaseStates(guess_states, k);
  }
  interp_trajx.clear();
  interp_trajy.clear();
  interp_trajt.clear();
  interp_trajv.clear();
  interp_trajomega.clear();

  // guess time
  double stime = 0.0;
  double dtime = 10.0 / nphases;
  for (std::size_t i = 0; i < nphases; i++)
  {
    info_->setPhaseGuessTime(stime, stime + dtime, i);
    stime += dtime;
  }

  // obb extent
  b2Vec2 extent(vehicle_half_length_ + active_dist_, vehicle_half_width_ + active_dist_);
  b2Vec2 iextent(vehicle_half_length_, vehicle_half_width_);

  int sol = 0;
  bool success = true;
  bool use_linearized_system = false;
  double dynamics_cost_weight = 10.0;
  double best_cost = std::numeric_limits<double>::max();
  while (success && sol < num_iterative_)
  {
    sol++;
    // box and ap constraints
    std::vector<std::vector<psopt::ActivePoints2D<double>>> phaseActivePoints; 
    phaseActivePoints.reserve(nphases);
    for (std::size_t k = 0; k < nphases; k++)
    {
      // get active points
      std::size_t nactive = 0, nactive_state = 0;
      std::vector<double> lower_box_bounds, upper_box_bounds;
      lower_box_bounds.reserve(2 * info_->getPhaseNumberNodes(k));
      upper_box_bounds.reserve(2 * info_->getPhaseNumberNodes(k));
      std::vector<psopt::ActivePoints2D<double>> activePoints; 
      activePoints.reserve(info_->getPhaseNumberNodes(k));
      const double * states = solver.getPhaseStates(k).data() + nstates;
      for (std::size_t i = 1; i < info_->getPhaseNumberNodes(k); i++)
      {
        double x = states[0];
        double y = states[1];
        double yaw = states[2];
        states += nstates;

        b2Mat22 axis = b2Rot(yaw).toRotationMatrix();
        b2Vec2 center(x, y);
        b2OBB obb(axis, center, extent);
        APCallback callback(active_dist_, rect_, &obb);
        svc_->Collide(&callback, obb);
        double bound = active_dist_;
        if (callback.getMinDist() < active_dist_) bound = std::max(callback.getMinDist(), 0.5 * active_dist_);
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

      OMPL_INFORM("Number active constraints %d!", nactive);

      info_->setPhaseNumberPaths(lower_box_bounds.size() + nactive, k);
      for (std::size_t i = 0; i < nactive; i++)
      {
        lower_box_bounds.push_back(1.0005); 
        upper_box_bounds.push_back(std::numeric_limits<double>::infinity());
      }
      info_->setPhaseLowerBoundPaths(lower_box_bounds, k);
      info_->setPhaseUpperBoundPaths(upper_box_bounds, k);
      phaseActivePoints.push_back(activePoints);
    }

    // solve with current box and ap constraints
    problem_->setActivePoints(phaseActivePoints);
    if (info_->getTreatDynamicsAsCost()) info_->setDynamicsCostWeight(dynamics_cost_weight);
    if (use_linearized_system)
    {
      info_->setUseLinearizedDae(true);
      problem_->setUpLinearizedParameters(info_->getVariables().data());
      success = solver.solve(problem_);
      if (!success)
      {
        OMPL_ERROR("Failed to find a feasible trajectory!");
        break;
      }
      info_->setUseLinearizedDae(false);
    }
    success = solver.solve(problem_);
    if (!success)
    {
      OMPL_ERROR("Failed to find a feasible trajectory!");
      break;
    }

    // repair current local optimal trajectory
    while (success)
    {
      bool error = false;
      for (std::size_t k = 0; k < nphases; k++)
      {
        std::size_t nnactive = 0, nnactive_state = 0;
        std::vector<psopt::ActivePoints2D<double>> tempPoints;
        tempPoints.reserve(info_->getPhaseNumberNodes(k));
        const double * states = solver.getPhaseStates(k).data() + nstates;
        for (std::size_t i = 1; i < info_->getPhaseNumberNodes(k); i++)
        {
          double x = states[0];
          double y = states[1];
          double yaw = states[2];
          states += nstates;

          b2Mat22 axis = b2Rot(yaw).toRotationMatrix();
          b2Vec2 center(x, y);
          b2OBB obb(axis, center, iextent);
          APCallback callback(0.0001, rect_, &obb);
          svc_->Collide(&callback, obb);

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
        std::vector<double> lower_box_bounds = info_->getPhaseLowerBoundPaths(k);
        std::vector<double> upper_box_bounds = info_->getPhaseUpperBoundPaths(k);
        info_->setPhaseNumberPaths(lower_box_bounds.size() + nnactive, k);
        for (std::size_t i = 0; i < nnactive; i++)
        {
          lower_box_bounds.push_back(1.0005); 
          upper_box_bounds.push_back(std::numeric_limits<double>::infinity());
        }
        info_->setPhaseLowerBoundPaths(lower_box_bounds, k);
        info_->setPhaseUpperBoundPaths(upper_box_bounds, k);
        phaseActivePoints[k].insert(phaseActivePoints[k].end(), tempPoints.begin(), tempPoints.end());
      }
      if (!error) break;
      OMPL_WARN("Current local optimal trajectory is infeasible, try to fix it!");
      /*
         if (use_linearized_system)
         {
         info_->setUseLinearizedDae(true);
         success = solver.solve(problem_);
         if (!success) break;
         info_->setUseLinearizedDae(false);
         }
         */
      problem_->setActivePoints(phaseActivePoints);
      success = solver.solve(problem_);
    }
    if (!success)
    {
      OMPL_ERROR("Failed to find a feasible trajectory!");
      break;
    }
    if (info_->getTreatDynamicsAsCost())
    {
      OMPL_INFORM("Current cost %0.4f, current infeasiblity %0.4f, current w_penalty %0.4f", solver.getCost(), info_->getDynamicsCost(), dynamics_cost_weight);
      if (info_->getDynamicsCost() < 1.e-2) // TODO
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
    // info_->setUseLinearizedDae(true);
    // problem_->setUpLinearizedParameters(info_->getVariables().data());
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

  delete rect_;
  delete info_;
  delete problem_;
  rect_ = nullptr;
  info_ = nullptr;
  problem_ = nullptr;

  return 0;
}
