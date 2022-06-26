/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2012, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Shi Shenglei */

#include "omplapp/apps/RealVector2MultiRigidBodyPlanning.h"

ompl::app::RealVector2MultiRigidBodyPlanning::RealVector2MultiRigidBodyPlanning(unsigned int n) :
    AppBase<AppType::GEOMETRIC>(std::make_shared<base::CompoundStateSpace>(), Motion_2D), n_(n)
{
    assert (n > 0);
    name_ = "RealVector Multi rigid body planning (2D)";
    // Adding n Realvector(2) StateSpaces
    for (unsigned int i = 0; i < n_; ++i)
        si_->getStateSpace()->as<base::CompoundStateSpace>()->addSubspace(
            std::make_shared<base::RealVectorStateSpace>(2), 1.0);
}

void ompl::app::RealVector2MultiRigidBodyPlanning::inferEnvironmentBounds()
{
    // Infer bounds for all n Realvector(2) spaces
    for (unsigned int i = 0; i < n_; ++i)
        InferEnvironmentBounds(getGeometricComponentStateSpace(i), *static_cast<RigidBodyGeometry*>(this));

    auto bounds = getGeometricComponentStateSpace(0)->as<base::RealVectorStateSpace>()->getBounds();
    for (unsigned int i = 1; i < n_; ++i)
    {
        auto b = getGeometricComponentStateSpace(i)->as<base::RealVectorStateSpace>()->getBounds();
        if (bounds.low[0] > b.low[0])
            bounds.low[0] = b.low[0];
        if (bounds.low[1] > b.low[1])
            bounds.low[1] = b.low[1];
        if (bounds.high[0] < b.high[0])
            bounds.high[0] = b.high[0];
        if (bounds.high[1] < b.high[1])
            bounds.high[1] = b.high[1];
    }

    for (unsigned int i = 0; i < n_; ++i)
        getGeometricComponentStateSpace(i)->as<base::RealVectorStateSpace>()->setBounds(bounds);
}

void ompl::app::RealVector2MultiRigidBodyPlanning::inferProblemDefinitionBounds()
{
    // Make sure that all n Realvector(2) spaces get the same bounds, if they are adjusted
    for (unsigned int i = 0; i < n_; ++i)
        InferProblemDefinitionBounds(AppTypeSelector<AppType::GEOMETRIC>::SimpleSetup::getProblemDefinition(),
                                    getGeometricStateExtractor(), factor_, add_,
                                    n_, getGeometricComponentStateSpace(i), mtype_);

    auto bounds = getGeometricComponentStateSpace(0)->as<base::RealVectorStateSpace>()->getBounds();
    for (unsigned int i = 1; i < n_; ++i)
    {
        auto b = getGeometricComponentStateSpace(i)->as<base::RealVectorStateSpace>()->getBounds();
        if (bounds.low[0] > b.low[0])
            bounds.low[0] = b.low[0];
        if (bounds.low[1] > b.low[1])
            bounds.low[1] = b.low[1];
        if (bounds.high[0] < b.high[0])
            bounds.high[0] = b.high[0];
        if (bounds.high[1] < b.high[1])
            bounds.high[1] = b.high[1];
    }

    for (unsigned int i = 0; i < n_; ++i)
        getGeometricComponentStateSpace(i)->as<base::RealVectorStateSpace>()->setBounds(bounds);
}

ompl::base::ScopedState<> ompl::app::RealVector2MultiRigidBodyPlanning::getDefaultStartState() const
{
    base::ScopedState<> st(getStateSpace());
    auto* c_st = st->as<base::CompoundStateSpace::StateType>();
    for (unsigned int i = 0; i < n_; ++i)
    {
        auto* sub = c_st->as<base::RealVectorStateSpace::StateType>(i);
        sub->values[0] = 0.0;
        sub->values[1] = 0.0;
    }
    return st;
}

const ompl::base::State* ompl::app::RealVector2MultiRigidBodyPlanning::getGeometricComponentStateInternal(const ompl::base::State* state, unsigned int index) const
{
    assert (index < n_);
    const auto* st = state->as<base::CompoundStateSpace::StateType>()->as<base::RealVectorStateSpace::StateType>(index);
    return static_cast<const base::State*>(st);
}
