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

#ifndef OMPLAPP_DEMOS_KINEMATIC_CAR_SETUP_
#define OMPLAPP_DEMOS_KINEMATIC_CAR_SETUP_

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ompl
{
    namespace app
    {
        namespace demos 
        {
            /** \brief A class to facilitate planning for a generic kinematic car
                model

                The dynamics of the kinematic car are described by the following
                equations:
                \f{eqnarray*}{
                \dot x &=& u_0 \cos\theta,\\
                \dot y &=& u_0\sin\theta,\\
                \dot\theta &=& \frac{u_0}{L}\tan u_1,\f}
                where the control inputs \f$(u_0,u_1)\f$ are the translational
                velocity and the steering angle, respectively, and \f$L\f$ is the
                distance between the front and rear axle of the car (set to 1 by
                default).
            */
            class KinematicCarSetup : public control::SimpleSetup
            {
            public:
                KinematicCarSetup();

                virtual ~KinematicCarSetup() override = default;

                virtual void setDefaultControlBounds();

                void setVehicleLength(double length)
                {
                    lengthInv_ = 1./length;
                }

                double getVehicleLength()
                {
                    return 1./lengthInv_;
                }

            protected:

                virtual void ode(const control::ODESolver::StateType& q, const control::Control *ctrl, control::ODESolver::StateType& qdot);

                virtual void postPropagate(const base::State* state, const control::Control* control, double duration, base::State* result);

                static control::ControlSpacePtr constructControlSpace()
                {
                    return std::make_shared<control::RealVectorControlSpace>(constructStateSpace(), 2);
                }

                static base::StateSpacePtr constructStateSpace()
                {
                    return std::make_shared<base::SE2StateSpace>();
                }

                double timeStep_{1e-2};

                double lengthInv_{1.};

                control::ODESolverPtr odeSolver_;
            };
        }
    }
}
#endif
