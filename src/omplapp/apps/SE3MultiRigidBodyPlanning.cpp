/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2012, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ryan Luna */

#include "omplapp/apps/SE3MultiRigidBodyPlanning.h"

ompl::app::SE3MultiRigidBodyPlanning::SE3MultiRigidBodyPlanning(unsigned int n) :
    AppBase<AppType::GEOMETRIC>(std::make_shared<base::CompoundStateSpace>(), Motion_3D), n_(n)
{
    assert (n > 0);
    name_ = "Multi rigid body planning (3D)";
    // Adding n SE(3) StateSpaces
    for (unsigned int i = 0; i < n_; ++i)
        si_->getStateSpace()->as<base::CompoundStateSpace>()->addSubspace(
            std::make_shared<base::SE3StateSpace>(), 1.0);
}

void ompl::app::SE3MultiRigidBodyPlanning::inferEnvironmentBounds()
{
    // Infer bounds for all n SE(3) spaces
    for (unsigned int i = 0; i < n_; ++i)
        InferEnvironmentBounds(getGeometricComponentStateSpace(i), *static_cast<RigidBodyGeometry*>(this));

    auto bounds = getGeometricComponentStateSpace(0)->as<base::SE3StateSpace>()->getBounds();
    for (unsigned int i = 1; i < n_; ++i)
    {
        auto b = getGeometricComponentStateSpace(i)->as<base::SE3StateSpace>()->getBounds();
        if (bounds.low[0] > b.low[0])
            bounds.low[0] = b.low[0];
        if (bounds.low[1] > b.low[1])
            bounds.low[1] = b.low[1];
        if (bounds.low[2] > b.low[2])
            bounds.low[2] = b.low[2];
        if (bounds.high[0] < b.high[0])
            bounds.high[0] = b.high[0];
        if (bounds.high[1] < b.high[1])
            bounds.high[1] = b.high[1];
        if (bounds.high[2] < b.high[2])
            bounds.high[2] = b.high[2];
    }

    for (unsigned int i = 0; i < n_; ++i)
        getGeometricComponentStateSpace(i)->as<base::SE3StateSpace>()->setBounds(bounds);
}

void ompl::app::SE3MultiRigidBodyPlanning::inferProblemDefinitionBounds()
{
    // Make sure that all n SE(3) spaces get the same bounds, if they are adjusted
    for (unsigned int i = 0; i < n_; ++i)
        InferProblemDefinitionBounds(AppTypeSelector<AppType::GEOMETRIC>::SimpleSetup::getProblemDefinition(),
                                    getGeometricStateExtractor(), factor_, add_,
                                    n_, getGeometricComponentStateSpace(i), mtype_);

    auto bounds = getGeometricComponentStateSpace(0)->as<base::SE3StateSpace>()->getBounds();
    for (unsigned int i = 1; i < n_; ++i)
    {
        auto b = getGeometricComponentStateSpace(i)->as<base::SE3StateSpace>()->getBounds();
        if (bounds.low[0] > b.low[0])
            bounds.low[0] = b.low[0];
        if (bounds.low[1] > b.low[1])
            bounds.low[1] = b.low[1];
        if (bounds.low[2] > b.low[2])
            bounds.low[2] = b.low[2];
        if (bounds.high[0] < b.high[0])
            bounds.high[0] = b.high[0];
        if (bounds.high[1] < b.high[1])
            bounds.high[1] = b.high[1];
        if (bounds.high[2] < b.high[2])
            bounds.high[2] = b.high[2];
    }

    for (unsigned int i = 0; i < n_; ++i)
        getGeometricComponentStateSpace(i)->as<base::SE3StateSpace>()->setBounds(bounds);
}

ompl::base::ScopedState<> ompl::app::SE3MultiRigidBodyPlanning::getDefaultStartState() const
{
    base::ScopedState<> st(getStateSpace());
    auto* c_st = st->as<base::CompoundStateSpace::StateType>();
    for (unsigned int i = 0; i < n_; ++i)
    {
        aiVector3D s = getRobotCenter(i);
        auto* sub = c_st->as<base::SE3StateSpace::StateType>(i);
        sub->setX(s.x);
        sub->setY(s.y);
        sub->setZ(s.z);
        sub->rotation().setIdentity();
    }
    return st;
}

const ompl::base::State* ompl::app::SE3MultiRigidBodyPlanning::getGeometricComponentStateInternal(const ompl::base::State* state, unsigned int index) const
{
    assert (index < n_);
    const auto* st = state->as<base::CompoundStateSpace::StateType>()->as<base::SE3StateSpace::StateType>(index);
    return static_cast<const base::State*>(st);
}
