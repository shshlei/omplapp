/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

#ifndef OMPLAPP_COLLISION_BOX2D_DISCRETE_BVH_MANAGERS_
#define OMPLAPP_COLLISION_BOX2D_DISCRETE_BVH_MANAGERS_

#include <box2d_collision/b2_bvh_manager.h>

#include "omplapp/collision/Common.h"

#include "ompl/util/Console.h"
#include "ompl/util/ClassForward.h"

namespace ompl 
{
    namespace app 
    {
        namespace collision
        {
            namespace collision_box2d
            {
                inline b2Vec2 convertEigenToBox2d(const Eigen::Vector2d& v)
                {
                    return v;
                }

                inline Eigen::Vector2d convertBox2dToEigen(const b2Vec2& v)
                {
                    return v;
                }

                inline b2Mat22 convertEigenToBox2d(const Eigen::Matrix2d& r)
                {
                    return r;
                }

                inline Eigen::Matrix2d convertBox2dToEigen(const b2Mat22& r)
                {
                    return r;
                }

                inline b2Mat33 convertEigenToBox2d(const Eigen::Matrix3d& r)
                {
                    return r;
                }

                inline Eigen::Matrix3d convertBox2dToEigen(const b2Mat33& r)
                {
                    return r;
                }

                inline b2Transform convertEigenToBox2d(const Eigen::Isometry2d& t)
                {
                    return t;
                }

                inline Eigen::Isometry2d convertBox2dToEigen(const b2Transform& t)
                {
                    return t;
                }

                inline Eigen::Isometry3d convertBox2dToEigen3(const b2Transform& t)
                {
                    Eigen::Isometry3d i = Eigen::Isometry3d::Identity();
                    i.linear().topLeftCorner(2, 2) = t.linear();
                    i.translation().head<2>() = t.translation();
                    return i;
                }

                b2Shape* createShapePrimitive(const CollisionShapePtr& geom);

                OMPL_CLASS_FORWARD(Box2dDiscreteBVHManager);
                class Box2dDiscreteBVHManager
                {
                public:
                    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                    Box2dDiscreteBVHManager();

                    virtual ~Box2dDiscreteBVHManager() = default;

                    Box2dDiscreteBVHManager(const Box2dDiscreteBVHManager&) = delete;
                    Box2dDiscreteBVHManager& operator=(const Box2dDiscreteBVHManager&) = delete;
                    Box2dDiscreteBVHManager(Box2dDiscreteBVHManager&&) = delete;
                    Box2dDiscreteBVHManager& operator=(Box2dDiscreteBVHManager&&) = delete;

                    /**
                    * @brief Clone the manager
                    *
                    * This is to be used for multi threaded application. A user should
                    * make a clone for each thread.
                    */
                    virtual Box2dDiscreteBVHManagerPtr clone() const;

                    /**
                    * @brief Add a object to the checker
                    * @param name            The name of the object, must be unique.
                    * @param shapes          A vector of shapes that make up the collision object.
                    * @param shape_poses     A vector of poses for each shape, must be same length as shapes
                    * @return true if successfully added, otherwise false.
                    */
                    virtual bool addCollisionObject(const std::string& name,
                                                  const CollisionShapes& shapes,
                                                  const geometries::VectorIsometry2d& shape_poses,
                                                  bool active = true);

                    bool addCollisionObject(const std::string& name,
                                          const CollisionShapePtr& shape,
                                          const Eigen::Isometry2d& shape_pose,
                                          bool active = true)
                    {
                        CollisionShapes shapes(1, shape);
                        geometries::VectorIsometry2d shape_poses(1, shape_pose);
                        return addCollisionObject(name, shapes, shape_poses, active);
                    }

                    /*
                    const CollisionShapes& getCollisionObjectGeometries(const std::string& name) const;

                    const CollisionShapePtr getCollisionObjectGeometry(const std::string& name, std::size_t sub = 0) const;

                    const geometries::VectorIsometry3d& getCollisionObjectGeometriesTransforms(const std::string& name) const;

                    const Eigen::Isometry3d getCollisionObjectGeometryTransform(const std::string& name, std::size_t sub = 0) const;
                    */

                    /**
                    * @brief Find if a collision object already exists
                    * @param name The name of the collision object
                    * @return true if it exists, otherwise false.
                    */
                    virtual bool hasCollisionObject(const std::string& name) const;

                    /**
                    * @brief Remove an object from the checker
                    * @param name The name of the object
                    * @return true if successfully removed, otherwise false.
                    */
                    virtual bool removeCollisionObject(const std::string& name);

                    /**
                    * @brief Enable an object
                    * @param name The name of the object
                    */
                    virtual bool enableCollisionObject(const std::string& name);

                    /**
                    * @brief Disable an object
                    * @param name The name of the object
                    */
                    virtual bool disableCollisionObject(const std::string& name);

                    /**
                    * @brief Set a single collision object's tansforms
                    * @param name The name of the object
                    * @param pose The tranformation in world
                    */
                    virtual void setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry2d& pose);

                    /**
                    * @brief Set a series of collision object's tranforms
                    * @param names The name of the object
                    * @param poses The tranformation in world
                    */
                    virtual void setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                            const geometries::VectorIsometry2d& poses);

                    /**
                    * @brief Set which collision objects can move
                    * @param names A vector of collision object names
                    */
                    virtual void setActiveCollisionObjects(const std::vector<std::string>& names);

                    /**
                    * @brief Get which collision objects can move
                    * @return A list of collision object names
                    */
                    virtual const std::vector<std::string>& getActiveCollisionObjects() const;

                    virtual void setContactDistanceThreshold(double contact_distance);

                    /**
                    * @brief Get the contact distance threshold
                    * @return The contact distance
                    */
                    virtual double getContactDistanceThreshold() const;

                    /**
                    * @brief Perform a contact test for all objects based
                    * @param collisions The Contact results data
                    */
                    virtual bool contactTest();
                    virtual bool pointTest(const Eigen::Vector2d& point);
                    virtual bool contactTest(base::ContactResult& collisions);

                    virtual double distanceTest(double &dist);
                    //virtual double distanceTest(base::ContactResult& collisions);

                    std::unique_ptr<b2BVHManager>& getBox2dBroadphse()
                    {
                        return broadphase_;
                    }

                private:

                    class PointQueryCallback : public b2NaiveCallback
                    {
                    public:
                        PointQueryCallback(const b2Vec2& point) : b2NaiveCallback()
                        {
                            point_ = point;
                        }

                        ~PointQueryCallback() override = default;

                        bool ReportCollision(b2Fixture* fixture) override
                        {
                            if (fixture->GetShape()->TestPoint(fixture->GetGlobalTransform(), point_))
                                return true;
                            return false;
                        }

                    private:
                        b2Vec2 point_;
                    };

                    std::vector<std::string> active_; 
                    double contact_distance_;         /**< @brief The contact distance threshold */
                    std::unique_ptr<b2BVHManager> broadphase_; /**< @brief The box2d broadphase interface */
                    CollisionShapes m_shapes;                    /**< @brief The shapes that define the collison object */
                    geometries::VectorIsometry2d m_shape_poses; /**< @brief The shpaes poses information */
                };
            }
        }
    }
}

#endif
