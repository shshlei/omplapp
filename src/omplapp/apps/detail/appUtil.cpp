/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan */

#include "omplapp/apps/detail/appUtil.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include "ompl/tools/config/MagicConstants.h"
#include <limits>
#include <utility>

void ompl::app::InferProblemDefinitionBounds(const base::ProblemDefinitionPtr &pdef, const GeometricStateExtractor &se, double factor, double add,
                                             unsigned int robotCount, const base::StateSpacePtr &space, MotionModel mtype)
{
    // update the bounds based on start states, if needed
    base::RealVectorBounds bounds = mtype == Motion_2D ?
        (space->getType() == base::STATE_SPACE_REAL_VECTOR ? space->as<base::RealVectorStateSpace>()->getBounds() : space->as<base::SE2StateSpace>()->getBounds()) :
        space->as<base::SE3StateSpace>()->getBounds();

    std::vector<const base::State*> states;
    pdef->getInputStates(states);

    double minX = std::numeric_limits<double>::infinity();
    double minY = minX;
    double minZ = minX;
    double maxX = -minX;
    double maxY = maxX;
    double maxZ = maxX;
    for (auto & state : states)
    {
        for (unsigned int r = 0 ; r < robotCount ; ++r)
        {
            const base::State *s = se(state, r);
            double x = mtype == Motion_2D ? (space->getType() == base::STATE_SPACE_REAL_VECTOR ? s->as<base::RealVectorStateSpace::StateType>()->values[0] :
                                             s->as<base::SE2StateSpace::StateType>()->getX()) :
                                            s->as<base::SE3StateSpace::StateType>()->getX();
            double y = mtype == Motion_2D ? (space->getType() == base::STATE_SPACE_REAL_VECTOR ? s->as<base::RealVectorStateSpace::StateType>()->values[1] :
                                             s->as<base::SE2StateSpace::StateType>()->getY()) : s->as<base::SE3StateSpace::StateType>()->getY();
            double z = mtype == Motion_2D ? 0.0 : s->as<base::SE3StateSpace::StateType>()->getZ();
            if (minX > x) minX = x;
            if (maxX < x) maxX = x;
            if (minY > y) minY = y;
            if (maxY < y) maxY = y;
            if (minZ > z) minZ = z;
            if (maxZ < z) maxZ = z;
        }
    }
    double dx = (maxX - minX) * (factor - 1.0) + add;
    double dy = (maxY - minY) * (factor - 1.0) + add;
    double dz = (maxZ - minZ) * (factor - 1.0) + add;

    if (bounds.low[0] > minX - dx) bounds.low[0] = minX - dx;
    if (bounds.low[1] > minY - dy) bounds.low[1] = minY - dy;

    if (bounds.high[0] < maxX + dx) bounds.high[0] = maxX + dx;
    if (bounds.high[1] < maxY + dy) bounds.high[1] = maxY + dy;

    if (mtype == Motion_3D)
    {
        if (bounds.high[2] < maxZ + dz) bounds.high[2] = maxZ + dz;
        if (bounds.low[2] > minZ - dz) bounds.low[2] = minZ - dz;

        space->as<base::SE3StateSpace>()->setBounds(bounds);
    }
    else
    {
        if (space->getType() == ompl::base::STATE_SPACE_REAL_VECTOR)
            space->as<base::RealVectorStateSpace>()->setBounds(bounds);
        else
            space->as<base::SE2StateSpace>()->setBounds(bounds);
    }
}

void ompl::app::InferEnvironmentBounds(const base::StateSpacePtr &space, const RigidBodyGeometry &rbg)
{
    bool valid = true;
    if (space->getType() == ompl::base::STATE_SPACE_REAL_VECTOR)
    {
        base::RealVectorBounds bounds = space->as<base::RealVectorStateSpace>()->getBounds();
        if (bounds.getVolume() < std::numeric_limits<double>::epsilon())
            valid = false;
    }
    else if (space->getType() == ompl::base::STATE_SPACE_SE2)
    {
        base::RealVectorBounds bounds = space->as<base::SE2StateSpace>()->getBounds();
        if (bounds.getVolume() < std::numeric_limits<double>::epsilon())
            valid = false;
    }
    else if (space->getType() == ompl::base::STATE_SPACE_SE3)
    {
        base::RealVectorBounds bounds = space->as<base::SE3StateSpace>()->getBounds();
        if (bounds.getVolume() < std::numeric_limits<double>::epsilon())
            valid = false;
    }

    // if bounds are not valid
    if (!valid)
    {
        if (space->getType() == ompl::base::STATE_SPACE_REAL_VECTOR)
            space->as<base::RealVectorStateSpace>()->setBounds(rbg.inferEnvironmentBounds());
        else if (space->getType() == ompl::base::STATE_SPACE_SE2)
            space->as<base::SE2StateSpace>()->setBounds(rbg.inferEnvironmentBounds());
        else if (space->getType() == ompl::base::STATE_SPACE_SE3)
            space->as<base::SE3StateSpace>()->setBounds(rbg.inferEnvironmentBounds());
    }
}

namespace ompl
{
    namespace app
    {
        namespace detail
        {
            class GeometricStateProjectorRealVector2D : public base::ProjectionEvaluator
            {
            public:

                GeometricStateProjectorRealVector2D(const base::StateSpacePtr &space, const base::StateSpacePtr &gspace, GeometricStateExtractor se) :
                    base::ProjectionEvaluator(space), gm_(gspace->as<base::RealVectorStateSpace>()), se_(std::move(se))
                {
                }

                unsigned int getDimension() const override
                {
                    return 2 * space_->getSubspaceCount();
                }

                void project(const base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
                {
                    for (std::size_t i = 0; i < space_->getSubspaceCount(); i++)
                    {
                        const base::State *gs = se_(state, i);
                        projection(2*i) = gs->as<base::RealVectorStateSpace::StateType>()->values[0];
                        projection(2*i+1) = gs->as<base::RealVectorStateSpace::StateType>()->values[1];
                    }
                }

                void defaultCellSizes() override
                {
                    const auto bounds = gm_->getBounds();
                    const std::vector<double> b = bounds.getDifference();
                    cellSizes_.resize(getDimension());
                    bounds_.resize(getDimension());
                    for (std::size_t i = 0; i < space_->getSubspaceCount(); i++)
                    {
                        bounds_.setLow(2*i, bounds.low[0]);
                        bounds_.setLow(2*i+1, bounds.low[1]);
                        bounds_.setHigh(2*i, bounds.high[0]);
                        bounds_.setHigh(2*i+1, bounds.high[1]);
                        cellSizes_[2*i] = b[0] / magic::PROJECTION_DIMENSION_SPLITS;
                        cellSizes_[2*i+1] = b[1] / magic::PROJECTION_DIMENSION_SPLITS;
                    }
                }

            protected:

                const base::RealVectorStateSpace *gm_;
                GeometricStateExtractor    se_;

            };

            class GeometricStateProjector2D : public base::ProjectionEvaluator
            {
            public:

                GeometricStateProjector2D(const base::StateSpacePtr &space, const base::StateSpacePtr &gspace, GeometricStateExtractor se) : base::ProjectionEvaluator(space), gm_(gspace->as<base::SE2StateSpace>()), se_(std::move(se))
                {
                }

                unsigned int getDimension() const override
                {
                    return 2 * space_->getSubspaceCount();
                }

                void project(const base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
                {
                    for (std::size_t i = 0; i < space_->getSubspaceCount(); i++)
                    {
                        const base::State *gs = se_(state, i);
                        projection(2*i) = gs->as<base::SE2StateSpace::StateType>()->getX();
                        projection(2*i+1) = gs->as<base::SE2StateSpace::StateType>()->getY();
                    }
                }

                void defaultCellSizes() override
                {
                    const auto bounds = gm_->getBounds();
                    const std::vector<double> b = bounds.getDifference();
                    cellSizes_.resize(getDimension());
                    bounds_.resize(getDimension());
                    for (std::size_t i = 0; i < space_->getSubspaceCount(); i++)
                    {
                        bounds_.setLow(2*i, bounds.low[0]);
                        bounds_.setLow(2*i+1, bounds.low[1]);
                        bounds_.setHigh(2*i, bounds.high[0]);
                        bounds_.setHigh(2*i+1, bounds.high[1]);
                        cellSizes_[2*i] = b[0] / magic::PROJECTION_DIMENSION_SPLITS;
                        cellSizes_[2*i+1] = b[1] / magic::PROJECTION_DIMENSION_SPLITS;
                    }
                }

            protected:

                const base::SE2StateSpace *gm_;
                GeometricStateExtractor    se_;

            };

            class GeometricStateProjector3D : public base::ProjectionEvaluator
            {
            public:

                GeometricStateProjector3D(const base::StateSpacePtr &space, const base::StateSpacePtr &gspace, GeometricStateExtractor se) : base::ProjectionEvaluator(space), gm_(gspace->as<base::SE3StateSpace>()), se_(std::move(se))
                {
                }

                unsigned int getDimension() const override
                {
                    return 3 * space_->getSubspaceCount();
                }

                void project(const base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
                {
                    for (std::size_t i = 0; i < space_->getSubspaceCount(); i++)
                    {
                        const base::State *gs = se_(state, i);
                        projection(3*i) = gs->as<base::SE3StateSpace::StateType>()->getX();
                        projection(3*i+1) = gs->as<base::SE3StateSpace::StateType>()->getY();
                        projection(3*i+2) = gs->as<base::SE3StateSpace::StateType>()->getZ();
                    }
                }

                void defaultCellSizes() override
                {
                    const auto bounds = gm_->getBounds();
                    const std::vector<double> b = bounds_.getDifference();
                    cellSizes_.resize(getDimension());
                    bounds_.resize(getDimension());
                    for (std::size_t i = 0; i < space_->getSubspaceCount(); i++)
                    {
                        bounds_.low[3*i] = bounds.low[0];
                        bounds_.low[3*i+1] = bounds.low[1];
                        bounds_.low[3*i+2] = bounds.low[2];
                        bounds_.high[3*i] = bounds.high[0];
                        bounds_.high[3*i+1] = bounds.high[1];
                        bounds_.high[3*i+2] = bounds.high[2];
                        cellSizes_[3*i] = b[0] / magic::PROJECTION_DIMENSION_SPLITS;
                        cellSizes_[3*i+1] = b[1] / magic::PROJECTION_DIMENSION_SPLITS;
                        cellSizes_[3*i+2] = b[2] / magic::PROJECTION_DIMENSION_SPLITS;
                    }
                }

            protected:

                const base::SE3StateSpace *gm_;
                GeometricStateExtractor    se_;
            };


            // a decomposition is only needed for SyclopRRT and SyclopEST
            class Decomposition2D : public ompl::control::GridDecomposition
            {
            public:
                // 32 x 32 grid
                Decomposition2D(const ompl::base::RealVectorBounds &bounds, const base::StateSpacePtr &space)
                    : GridDecomposition(32, 2, bounds), space_(space), position_(space->getValueLocations()[0])
                {
                }
                void project(const ompl::base::State *s, std::vector<double> &coord) const override
                {
                    const double* pos = space_->getValueAddressAtLocation(s, position_);
                    coord.resize(2);
                    coord[0] = pos[0];
                    coord[1] = pos[1];
                }

                void sampleFullState(const ompl::base::StateSamplerPtr &sampler,
                const std::vector<double>& coord, ompl::base::State *s) const override
                {
                    double* pos = space_->getValueAddressAtLocation(s, position_);
                    sampler->sampleUniform(s);
                    pos[0] = coord[0];
                    pos[1] = coord[1];
                }

            protected:
                const base::StateSpacePtr space_;
                const ompl::base::StateSpace::ValueLocation& position_;
            };

            class Decomposition3D : public ompl::control::GridDecomposition
            {
            public:
                // 16 x 16 x 16 grid
                Decomposition3D(const ompl::base::RealVectorBounds &bounds, const base::StateSpacePtr &space)
                    : GridDecomposition(16, 3, bounds), space_(space), position_(space->getValueLocations()[0])
                {
                }
                void project(const ompl::base::State *s, std::vector<double> &coord) const override
                {
                    const double* pos = space_->getValueAddressAtLocation(s, position_);
                    coord.resize(3);
                    coord[0] = pos[0];
                    coord[1] = pos[1];
                    coord[2] = pos[2];
                }

                void sampleFullState(const ompl::base::StateSamplerPtr &sampler,
                const std::vector<double>& coord, ompl::base::State *s) const override
                {
                    double* pos = space_->getValueAddressAtLocation(s, position_);
                    sampler->sampleUniform(s);
                    pos[0] = coord[0];
                    pos[1] = coord[1];
                    pos[2] = coord[2];
                }

            protected:
                const base::StateSpacePtr space_;
                const ompl::base::StateSpace::ValueLocation& position_;
            };
        }
    }
}

ompl::base::ProjectionEvaluatorPtr ompl::app::allocGeometricStateProjector(const base::StateSpacePtr &space, MotionModel mtype,
                                                                           const base::StateSpacePtr &gspace, const GeometricStateExtractor &se)
{
    if (mtype == Motion_2D)
    {
        if (gspace->getType() == ompl::base::STATE_SPACE_REAL_VECTOR)
            return std::make_shared<detail::GeometricStateProjectorRealVector2D>(space, gspace, se);
        else
            return std::make_shared<detail::GeometricStateProjector2D>(space, gspace, se);
    }
    return std::make_shared<detail::GeometricStateProjector3D>(space, gspace, se);
}

ompl::control::DecompositionPtr ompl::app::allocDecomposition(const base::StateSpacePtr &space, MotionModel mtype,
    const base::StateSpacePtr &gspace)
{
    // \todo shouldn't this be done automatically?
    const_cast<ompl::base::StateSpace*>(space.get())->computeLocations();

    if (mtype == Motion_2D)
        return std::make_shared<detail::Decomposition2D>(gspace->as<ompl::base::SE2StateSpace>()->getBounds(), space);
    return std::make_shared<detail::Decomposition3D>(gspace->as<ompl::base::SE3StateSpace>()->getBounds(), space);
}

ompl::base::OptimizationObjectivePtr ompl::app::getOptimizationObjective(
    const base::SpaceInformationPtr &si, const std::string &objective, double threshold)
{
    base::OptimizationObjectivePtr obj;
    if (objective == std::string("length"))
        obj = std::make_shared<base::PathLengthOptimizationObjective>(si);
    else if (objective == std::string("max min clearance"))
        obj = std::make_shared<base::MaximizeMinClearanceObjective>(si);
    else if (objective == std::string("mechanical work"))
        obj = std::make_shared<base::MechanicalWorkOptimizationObjective>(si);
    else
    {
        OMPL_WARN("ompl::app::getOptimizationObjective: unknown optimization objective called \"%s\"; using \"length\" instead", objective.c_str());
        obj = std::make_shared<base::PathLengthOptimizationObjective>(si);
    }
    obj->setCostThreshold(base::Cost(threshold));
    return obj;
}
