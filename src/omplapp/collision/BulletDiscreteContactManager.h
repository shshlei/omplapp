/**
 * @file discrete_contact_manager.h
 * @brief This is the discrete contact manager base class
 *
 * It should be used to perform discrete contact checking.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @author Shi Shenglei 
 * @date April 1, 2020
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef OMPLAPP_COLLISION_DISCRETE_CONTACT_MANAGER_
#define OMPLAPP_COLLISION_DISCRETE_CONTACT_MANAGER_

#include "omplapp/collision/Common.h"

#include "ompl/util/ClassForward.h"

#include <boost/concept_check.hpp>

namespace ompl 
{
    namespace app 
    {
        namespace collision
        {
            OMPL_CLASS_FORWARD(DiscreteContactManager);

            class DiscreteContactManager
            {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                DiscreteContactManager() = default;

                virtual ~DiscreteContactManager() = default;

                DiscreteContactManager(const DiscreteContactManager&) = delete;
                DiscreteContactManager& operator=(const DiscreteContactManager&) = delete;
                DiscreteContactManager(DiscreteContactManager&&) = delete;
                DiscreteContactManager& operator=(DiscreteContactManager&&) = delete;

                template <class T>
                T *as()
                {
                    /** \brief Make sure the type we are casting to is indeed a planner */
                    BOOST_CONCEPT_ASSERT((boost::Convertible<T *, DiscreteContactManager *>));

                    return static_cast<T *>(this);
                }

                /** \brief Cast this instance to a desired type. */
                template <class T>
                const T *as() const
                {
                    /** \brief Make sure the type we are casting to is indeed a Planner */
                    BOOST_CONCEPT_ASSERT((boost::Convertible<T *, DiscreteContactManager *>));

                    return static_cast<const T *>(this);
                }

                /**
                * @brief Clone the manager
                *
                * This is to be used for multi threaded application. A user should
                * make a clone for each thread.
                */
                virtual std::shared_ptr<DiscreteContactManager> clone() const = 0;

                /**
                * @brief Add a object to the checker
                * @param name            The name of the object, must be unique.
                * @param mask_id         User defined id which gets stored in the results structure.
                * @param shapes          A vector of shapes that make up the collision object.
                * @param shape_poses     A vector of poses for each shape, must be same length as shapes
                * @return true if successfully added, otherwise false.
                */
                virtual bool addCollisionObject(const std::string& name,
                                              const int& mask_id,
                                              const CollisionShapes& shapes,
                                              const geometries::VectorIsometry3d& shape_poses,
                                              bool enabled = true) = 0;

                bool addCollisionObject(const std::string& name,
                                      const int& mask_id,
                                      const CollisionShapePtr& shape,
                                      const Eigen::Isometry3d& shape_pose,
                                      bool enabled = true)
                {
                    CollisionShapes shapes(1, shape);
                    geometries::VectorIsometry3d shape_poses(1, shape_pose);
                    return addCollisionObject(name, mask_id, shapes, shape_poses, enabled);
                }

                /**
                * @brief Get a collision objects collision geometries
                * @param name The collision objects name
                * @return A vector of collision geometries. The vector will be empty if the collision object is not found.
                */
                virtual const CollisionShapes& getCollisionObjectGeometries(const std::string& name) const = 0;

                virtual const CollisionShapePtr getCollisionObjectGeometry(const std::string& name, std::size_t sub = 0) const = 0;

                /**
                * @brief Get a collision objects collision geometries transforms
                * @param name  The collision objects name
                * @return A vector of collision geometries transforms. The vector will be empty if the collision object is not found.
                */
                virtual const geometries::VectorIsometry3d& getCollisionObjectGeometriesTransforms(const std::string& name) const = 0;

                virtual const Eigen::Isometry3d getCollisionObjectGeometryTransform(const std::string& name, std::size_t sub = 0) const = 0;

                /**
                * @brief Find if a collision object already exists
                * @param name The name of the collision object
                * @return true if it exists, otherwise false.
                */
                virtual bool hasCollisionObject(const std::string& name) const = 0;

                /**
                * @brief Remove an object from the checker
                * @param name The name of the object
                * @return true if successfully removed, otherwise false.
                */
                virtual bool removeCollisionObject(const std::string& name) = 0;

                /**
                * @brief Enable an object
                * @param name The name of the object
                */
                virtual bool enableCollisionObject(const std::string& name) = 0;

                /**
                * @brief Disable an object
                * @param name The name of the object
                */
                virtual bool disableCollisionObject(const std::string& name) = 0;

                /**
                * @brief Set a single collision object's tansforms
                * @param name The name of the object
                * @param pose The tranformation in world
                */
                virtual void setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose) = 0;

                /**
                * @brief Set a series of collision object's tranforms
                * @param names The name of the object
                * @param poses The tranformation in world
                */
                virtual void setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                        const geometries::VectorIsometry3d& poses) = 0;

                /**
                * @brief Set a series of collision object's tranforms
                * @param transforms A transform map <name, pose>
                */
                virtual void setCollisionObjectsTransform(const geometries::TransformMap& transforms) = 0;

                /**
                * @brief Set which collision objects can move
                * @param names A vector of collision object names
                */
                virtual void setActiveCollisionObjects(const std::vector<std::string>& names) = 0;

                /**
                * @brief Get which collision objects can move
                * @return A list of collision object names
                */
                virtual const std::vector<std::string>& getActiveCollisionObjects() const = 0;

                /**
                * @brief Set the contact distance threshold for which collision should be considered.
                * @param contact_distance The contact distance
                */
                virtual void setContactDistanceThreshold(double contact_distance) = 0;

                /**
                * @brief Get the contact distance threshold
                * @return The contact distance
                */
                virtual double getContactDistanceThreshold() const = 0;

                virtual void setNegativeDistanceThreshold(double negative_distance) = 0;

                virtual double getNegativeDistanceThreshold() const = 0;

                /** @brief Set the active function for determining if two links are allowed to be in collision */
                virtual void setIsContactAllowedFn(IsContactAllowedFn fn) = 0;

                /** @brief Get the active function for determining if two links are allowed to be in collision */
                virtual IsContactAllowedFn getIsContactAllowedFn() const = 0;

                /**
                * @brief Perform a contact test for all objects based
                * @param collisions The Contact results data
                */
                virtual void contactTest(base::ContactResultMap& collisions, const ContactTestType& type, unsigned int max_contacts = 5) = 0;
            };
        }
    }
}

#endif
