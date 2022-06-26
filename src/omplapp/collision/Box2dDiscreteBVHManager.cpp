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

#include "omplapp/collision/Box2dDiscreteBVHManager.h"

namespace ompl 
{
    namespace app 
    {
        namespace collision
        {
            namespace collision_box2d
            {
                b2Shape* createShapePrimitive(const geometries::Circle& geom)
                {
                    b2CircleShape* shape = new b2CircleShape();
                    shape->SetRadius(static_cast<b2Scalar>(geom.getRadius()));
                    return shape;
                }

                b2Shape* createShapePrimitive(const geometries::Polygon& geom)
                {
                    b2PolygonShape* shape = new b2PolygonShape();

                    int count = geom.getVerticeCount();
                    const Eigen::Vector2d* vecs = geom.getVertices();
                    b2Vec2 vec[count];
                    for (int i = 0; i < count; i++)
                        vec[i] = convertEigenToBox2d(vecs[i]);

                    shape->Set(&vec[0], count);
                    shape->SetRadius(b2Scalar(0.0));
                    return shape;
                }

                /**
                * @brief Create a box2d collision shape from tesseract collision shape
                * @param geom Tesseract collision shape
                * @return Box2d collision shape.
                */
                b2Shape* createShapePrimitive(const CollisionShapePtr& geo)
                {
                    b2Shape* shape = nullptr;
                    switch (geo->getType())
                    {
                        case geometries::GeometryType::CIRCLE:
                        {
                            auto geom = geo->as<geometries::Circle>();
                            shape = createShapePrimitive(*geom);
                            break;
                        }
                        case geometries::GeometryType::POLYGON:
                        {
                            auto geom = geo->as<geometries::Polygon>();
                            shape = createShapePrimitive(*geom);
                            break;
                        }
                        default:
                        {
                            OMPL_ERROR("This geometric shape type (%d) is not supported using BOX2D yet",
                                                  static_cast<int>(geo->getType()));
                            break;
                        }
                    }
                    return shape;
                }

                Box2dDiscreteBVHManager::Box2dDiscreteBVHManager()
                {
                    broadphase_ = std::make_unique<b2BVHManager>();
                    contact_distance_ = 0.0;
                    negative_distance_= 0.0;
                }

                Box2dDiscreteBVHManagerPtr Box2dDiscreteBVHManager::clone() const
                {
                    std::shared_ptr<Box2dDiscreteBVHManager> manager(new Box2dDiscreteBVHManager());
                    std::unique_ptr<b2BVHManager>& clone_broadphase = manager->getBox2dBroadphse();

                    manager->setActiveCollisionObjects(active_);
                    manager->setContactDistanceThreshold(contact_distance_);
                    manager->setNegativeDistanceThreshold(negative_distance_);
                    clone_broadphase->SetContactFilter(broadphase_->GetContactFilter());

                    const b2Body* bodylist = broadphase_->GetBodyList();
                    while (bodylist)
                    {
                        b2Body* body = clone_broadphase->CreateBody(bodylist->GetName(), bodylist->IsActive());
                        const b2Fixture *f = bodylist->GetFixtureList();
                        while (f)
                        {
                            clone_broadphase->AddShapeToBody(body, f->GetShape());
                            body->GetFixtureList()->SetFilterData(f->GetFilterData());
                            body->GetFixtureList()->SetUserData(f->GetUserData());
                            f = f->GetNext();
                        }
                        body->SetEnabled(bodylist->IsEnabled());
                        body->SetUserData(bodylist->GetUserData());
                        body->SetTransform(bodylist->GetTransform());
                        bodylist = bodylist->GetNext();
                    }
                    return manager;
                }

                bool Box2dDiscreteBVHManager::addCollisionObject(const std::string& name,
                                                          const CollisionShapes& shapes,
                                                          const geometries::VectorIsometry2d& shape_poses,
                                                          bool active)
                {
                    if (hasCollisionObject(name))
                        removeCollisionObject(name);
                    b2Body *body = nullptr;
                    std::size_t index = 0;
                    for (const CollisionShapePtr& geom : shapes)
                    {
                        b2Shape *shape = createShapePrimitive(geom);
                        if (shape)
                        {
                            if (body == nullptr)
                                body = broadphase_->CreateBody(name, active);
                            b2Transform xf = convertEigenToBox2d(shape_poses[index]);
                            shape->SetLocalTransform(xf);
                            broadphase_->AddShapeToBody(body, shape, index);
                            delete shape;
                        }
                        index++;
                    }
                    if (body)
                    {
                        if (active)
                            active_.push_back(name);
                        return true;
                    }
                    return false;
                }

                bool Box2dDiscreteBVHManager::hasCollisionObject(const std::string& name) const
                {
                    return broadphase_->HasBody(name);
                }

                bool Box2dDiscreteBVHManager::removeCollisionObject(const std::string& name)
                {
                    if (broadphase_->IsBodyActive(name))
                        active_.erase(std::find(active_.begin(), active_.end(), name));
                    return broadphase_->RemoveBody(name);
                }

                bool Box2dDiscreteBVHManager::enableCollisionObject(const std::string& name)
                {
                    return broadphase_->EnableBody(name);
                }

                bool Box2dDiscreteBVHManager::disableCollisionObject(const std::string& name)
                {
                    return broadphase_->DisableBody(name);
                }

                void Box2dDiscreteBVHManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry2d& pose)
                {
                    broadphase_->SetBodyTransform(name, convertEigenToBox2d(pose));
                }

                void Box2dDiscreteBVHManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                                    const geometries::VectorIsometry2d& poses)
                {
                    assert(names.size() == poses.size());
                    for (std::size_t i = 0; i < names.size(); ++i)
                        setCollisionObjectsTransform(names[i], poses[i]);
                }

                void Box2dDiscreteBVHManager::setActiveCollisionObjects(const std::vector<std::string>& names)
                {
                    active_ = names;
                    broadphase_->SetActiveBodys(names);
                }

                const std::vector<std::string>& Box2dDiscreteBVHManager::getActiveCollisionObjects() const { return active_; }

                void Box2dDiscreteBVHManager::setContactDistanceThreshold(double contact_distance)
                {
                    contact_distance_ = contact_distance;
                }

                double Box2dDiscreteBVHManager::getContactDistanceThreshold() const { return contact_distance_; }

                void Box2dDiscreteBVHManager::setNegativeDistanceThreshold(double negative_distance)
                {
                    negative_distance_ = negative_distance;
                }

                double Box2dDiscreteBVHManager::getNegativeDistanceThreshold() const { return negative_distance_; }

                bool Box2dDiscreteBVHManager::contactTest()
                {
                    return broadphase_->ContactTest();
                }

                bool Box2dDiscreteBVHManager::pointTest(const Eigen::Vector2d& point)
                {
                    b2Vec2 bpoint = convertEigenToBox2d(point);
                    PointQueryCallback callback(bpoint);
                    broadphase_->QueryPoint(&callback, bpoint);
                    return callback.isCollision();
                }

                bool Box2dDiscreteBVHManager::contactTest(base::ContactResult& collisions)
                {
                    b2ContactResult contact;
                    b2InscribedSpheres inscribedSpheres;
                    collisions.clear();
                    if (broadphase_->ContactTest(&contact, &inscribedSpheres))
                    {
                        collisions.type_id[0] = 1;
                        collisions.type_id[1] = 1;
                        bool swap = false;
                        if (std::find(active_.begin(), active_.end(), contact.names[0]) == active_.end())
                        {
                            swap = true;
                            collisions.type_id[1] = 0;
                        }
                        if (std::find(active_.begin(), active_.end(), contact.names[1]) == active_.end())
                            collisions.type_id[1] = 0;

                        if (swap)
                        {
                            collisions.link_names[0] = contact.names[1];
                            collisions.link_names[1] = contact.names[0];
                            collisions.shape_id[0] = contact.shape_id[1];
                            collisions.shape_id[1] = contact.shape_id[0];
                            collisions.distance = contact.separation;
                            collisions.normal.head<2>() = -convertBox2dToEigen(contact.normal);
                            collisions.nearest_points[0].head<2>() = convertBox2dToEigen(contact.points[1]);
                            collisions.nearest_points[1].head<2>() = convertBox2dToEigen(contact.points[0]);
                            collisions.nearest_points_local[0].head<2>() = convertBox2dToEigen(contact.local_points[1]);
                            collisions.nearest_points_local[1].head<2>() = convertBox2dToEigen(contact.local_points[0]);
                            collisions.transform[0] = convertBox2dToEigen3(contact.transforms[1]);
                            collisions.transform[1] = convertBox2dToEigen3(contact.transforms[0]);

                            collisions.has_sphere[0] = inscribedSpheres.has_sphere2;
                            collisions.has_sphere[1] = inscribedSpheres.has_sphere1;
                            collisions.radius[0] = inscribedSpheres.radius2;
                            collisions.radius[1] = inscribedSpheres.radius1;
                            collisions.center[0].head<2>() = convertBox2dToEigen(inscribedSpheres.center2);
                            collisions.center[1].head<2>() = convertBox2dToEigen(inscribedSpheres.center1);
                            collisions.local_center[0].head<2>() = convertBox2dToEigen(inscribedSpheres.local_center2);
                            collisions.local_center[1].head<2>() = convertBox2dToEigen(inscribedSpheres.local_center1);
                        }
                        else
                        {
                            collisions.link_names[0] = contact.names[0];
                            collisions.link_names[1] = contact.names[1];
                            collisions.shape_id[0] = contact.shape_id[0];
                            collisions.shape_id[1] = contact.shape_id[1];
                            collisions.distance = contact.separation;
                            collisions.normal.head<2>() = convertBox2dToEigen(contact.normal);
                            collisions.nearest_points[0].head<2>() = convertBox2dToEigen(contact.points[0]);
                            collisions.nearest_points[1].head<2>() = convertBox2dToEigen(contact.points[1]);
                            collisions.nearest_points_local[0].head<2>() = convertBox2dToEigen(contact.local_points[0]);
                            collisions.nearest_points_local[1].head<2>() = convertBox2dToEigen(contact.local_points[1]);
                            collisions.transform[0] = convertBox2dToEigen3(contact.transforms[0]);
                            collisions.transform[1] = convertBox2dToEigen3(contact.transforms[1]);

                            collisions.has_sphere[0] = inscribedSpheres.has_sphere1;
                            collisions.has_sphere[1] = inscribedSpheres.has_sphere2;
                            collisions.radius[0] = inscribedSpheres.radius1;
                            collisions.radius[1] = inscribedSpheres.radius2;
                            collisions.center[0].head<2>() = convertBox2dToEigen(inscribedSpheres.center1);
                            collisions.center[1].head<2>() = convertBox2dToEigen(inscribedSpheres.center2);
                            collisions.local_center[0].head<2>() = convertBox2dToEigen(inscribedSpheres.local_center1);
                            collisions.local_center[1].head<2>() = convertBox2dToEigen(inscribedSpheres.local_center2);
                        }
                        return true;
                    }
                    return false;
                }
            }
        }
    }
}
