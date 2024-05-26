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

#include <psopt/optimal_control_problem.hpp>
#include <bomp/utils.hpp>
//#include <bomp/collision_constraints/J_function_collision_constraints.h>
#include <bomp/collision_constraints/active_collision_constraints.h>

template <typename Scalar = double, typename Scalar2 = Scalar>
class DifferentialDriving : public psopt::OptimalControlProblem<Scalar, Scalar2>
{
public:

  DifferentialDriving(psopt::OptimalControlProblemInfo<Scalar2>* prob) : psopt::OptimalControlProblem<Scalar, Scalar2>(prob)
  {
  }

  virtual ~DifferentialDriving() = default;

  psopt::OptimalControlProblem<adouble, Scalar2>* clone() const override
  {
    DifferentialDriving<adouble, Scalar2>* prob = new DifferentialDriving<adouble, Scalar2>(this->problemInfo_);
    prob->setActivePoints(activePoints_);
    // prob->setLinearizedParameters(this);
    return prob;
  }

  Scalar endpointCost(const Scalar* /*initial_states*/, const Scalar* /*final_states*/,
      const Scalar* /*initial_controls*/, const Scalar* /*final_controls*/, 
      const Scalar* /*parameters*/, const Scalar& t0, const Scalar& tf,
      const Scalar* /*xad*/, std::size_t /*iphase*/) const override
  {
    return (tf - t0);
  }

  Scalar integrandCost(const Scalar* /*states*/, const Scalar* controls, const Scalar* /*parameters*/,
      const Scalar& /*time*/, const Scalar* /*xad*/, std::size_t /*iphase*/) const override
  {
    const Scalar& v = controls[0];
    return 0.5 * v * v;
  }

  void dae(Scalar* derivatives, const Scalar* states,
      const Scalar* controls, const Scalar* /*parameters*/, const Scalar& /*time*/, const Scalar* /*xad*/, std::size_t /*iphase*/) const override
  {
    const Scalar& theta = states[2];
    const Scalar& v = controls[0];
    const Scalar& omega = controls[1];

    // kinematics
    derivatives[0] = v * cos(theta);
    derivatives[1] = v * sin(theta);
    derivatives[2] = omega;
  }

  void pathAlongTrajectory(Scalar* /*paths*/, const Scalar* /*states*/,
      const Scalar* /*controls*/, const Scalar* /*parameters*/, const Scalar& /*time*/, const Scalar* /*xad*/, std::size_t /*iphase*/) const override
  {
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
      active_Constraints_2D(paths + offset, invtransform, apoints.activePoints);
      offset += apoints.activePoints.size();
    }
  }

  void events(Scalar* e, const Scalar* initial_states, const Scalar* final_states,
      const Scalar* initial_controls, const Scalar* final_controls, const Scalar* /*parameters*/,
      const Scalar& /*t0*/, const Scalar& /*tf*/, const Scalar* /*xad*/, std::size_t iphase) const override
  {
    if (iphase == 0) // first phase
    {
      const Scalar& x0 = initial_states[0];
      const Scalar& y0 = initial_states[1];
      const Scalar& theta0 = initial_states[2];
      const Scalar& v0 = initial_controls[0];

      // 4 initial events
      int offset = 0;
      e[offset] = x0;
      offset += 1;
      e[offset] = y0;
      offset += 1;
      e[offset] = theta0;
      offset += 1;
      e[offset] = v0;
      offset += 1;
    }
    if (iphase == this->problemInfo_->getPhaseNumbers() - 1) // last phase
    {
      const Scalar& xf = final_states[0];
      const Scalar& yf = final_states[1];
      const Scalar& thetaf = final_states[2];
      const Scalar& vf = final_controls[0];

      int offset = 0;
      if (this->problemInfo_->getPhaseNumbers() == 1) // one phase
        offset = 4; // 4 initial events

      // 4 final events
      e[offset] = xf;
      offset += 1;
      e[offset] = yf;
      offset += 1;
      e[offset] = thetaf;
      offset += 1;
      e[offset] = vf;
      offset += 1;
    }
  }

public:

  void setActivePoints(const std::vector<std::vector<psopt::ActivePoints2D<Scalar2>>> & activePoints)
  {
    activePoints_ = activePoints;
  }

private:

  std::vector<std::vector<psopt::ActivePoints2D<Scalar2>>> activePoints_;
};
