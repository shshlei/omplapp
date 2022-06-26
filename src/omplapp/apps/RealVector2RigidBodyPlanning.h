/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Shi Shenglei */

#ifndef OMPLAPP_REALVECTOR2_RIGID_BODY_PLANNING_
#define OMPLAPP_REALVECTOR2_RIGID_BODY_PLANNING_

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "omplapp/apps/AppBase.h"

namespace ompl
{
    namespace app
    {

        /** \brief Wrapper for ompl::app::RigidBodyPlanning that plans
            for rigid bodies in REALVECTOR2. */
        class RealVector2RigidBodyPlanning : public AppBase<AppType::GEOMETRIC>
        {
        public:
            RealVector2RigidBodyPlanning() : AppBase<AppType::GEOMETRIC>(std::make_shared<base::RealVectorStateSpace>(2), Motion_2D)
            {
                name_ = "RealVector Rigid body planning (2D)";
            }

            ~RealVector2RigidBodyPlanning() override = default;

            bool isSelfCollisionEnabled() const override
            {
                return false;
            }

            base::ScopedState<> getDefaultStartState() const override;

            base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const override
            {
                return state;
            }

            const base::StateSpacePtr& getGeometricComponentStateSpace() const override
            {
                return getStateSpace();
            }

            unsigned int getRobotCount() const override
            {
                return 1;
            }

        protected:

            const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int /*index*/) const override
            {
                return state;
            }

        };

    }
}

#endif
