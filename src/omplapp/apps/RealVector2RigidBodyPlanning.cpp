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

#include "omplapp/apps/RealVector2RigidBodyPlanning.h"

ompl::base::ScopedState<> ompl::app::RealVector2RigidBodyPlanning::getDefaultStartState() const
{
    base::ScopedState<base::RealVectorStateSpace> st(getGeometricComponentStateSpace());

    st->values[0] = 0.0;
    st->values[1] = 0.0;

    return st;
}
