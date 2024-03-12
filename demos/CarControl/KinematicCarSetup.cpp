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

#include "KinematicCarSetup.h"
#include <boost/math/constants/constants.hpp>

ompl::app::demos::KinematicCarSetup::KinematicCarSetup() : ompl::control::SimpleSetup(constructControlSpace()),
    odeSolver_(std::make_shared<control::ODEBasicSolver<>>(si_, [this](const control::ODESolver::StateType& q, const control::Control *ctrl, control::ODESolver::StateType& qdot)
    {
        ode(q, ctrl, qdot);
    }))
{
    setDefaultControlBounds();
    si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver_,
        [this](const base::State* state, const control::Control* control, const double duration, base::State* result)
        {
            postPropagate(state, control, duration, result);
        }));
}

void ompl::app::demos::KinematicCarSetup::setDefaultControlBounds()
{
    base::RealVectorBounds cbounds(2);
    cbounds.low[0] = -5.;
    cbounds.high[0] = 5.;
    cbounds.low[1] = -boost::math::constants::pi<double>() * 30. / 180.;
    cbounds.high[1] = boost::math::constants::pi<double>() * 30. / 180.;
    getControlSpace()->as<control::RealVectorControlSpace>()->setBounds(cbounds);
}

void ompl::app::demos::KinematicCarSetup::ode(const control::ODESolver::StateType& q, const control::Control *ctrl, control::ODESolver::StateType& qdot)
{
    const double *u = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;

    // zero out qdot
    qdot.resize(q.size (), 0);
    qdot[0] = u[0] * cos(q[2]);
    qdot[1] = u[0] * sin(q[2]);
    qdot[2] = u[0] * lengthInv_ * tan(u[1]);
}

void ompl::app::demos::KinematicCarSetup::postPropagate(const base::State* /*state*/, const control::Control* /*control*/, const double /*duration*/, base::State* result)
{
    // Normalize orientation value between 0 and 2*pi
    const base::SO2StateSpace* SO2 = getStateSpace()->as<base::SE2StateSpace>()->as<base::SO2StateSpace>(1);
    auto* so2 = result->as<base::SE2StateSpace::StateType>()->as<base::SO2StateSpace::StateType>(1);
    SO2->enforceBounds(so2);
}
