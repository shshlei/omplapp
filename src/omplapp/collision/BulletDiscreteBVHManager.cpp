/**
 * @file bullet_discrete_bvh_manager.cpp
 * @brief Tesseract ROS Bullet Discrete BVH Manager implementation.
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
 * Software License Agreement (BSD-2-Clause)
 * @par
 * All rights reserved.
 * @par
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * @par
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * @par
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "omplapp/collision/BulletDiscreteBVHManager.h"

namespace ompl 
{
    namespace app 
    {
        namespace collision
        {
            namespace collision_bullet
            {
                static const CollisionShapes EMPTY_COLLISION_SHAPES_CONST;
                static const geometries::VectorIsometry3d EMPTY_COLLISION_SHAPES_TRANSFORMS;

                BulletDiscreteBVHManager::BulletDiscreteBVHManager()
                {
                    dispatcher_ = std::make_unique<btCollisionDispatcher>(&coll_config_);

                    dispatcher_->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE, coll_config_.getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE, CONVEX_SHAPE_PROXYTYPE));

                    dispatcher_->setDispatcherFlags(dispatcher_->getDispatcherFlags() & ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);

                    broadphase_ = std::make_unique<btDbvtBroadphase>();

                    contact_distance_ = 0;
                    negative_distance_= 0;
                }

                BulletDiscreteBVHManager::~BulletDiscreteBVHManager()
                {
                    // clean up remaining objects
                    for (auto& co : link2cow_)
                        removeCollisionObjectFromBroadphase(co.second, broadphase_, dispatcher_);
                }

                DiscreteContactManagerPtr BulletDiscreteBVHManager::clone() const
                {
                    std::shared_ptr<BulletDiscreteBVHManager> manager(new BulletDiscreteBVHManager());

                    manager->setActiveCollisionObjects(active_);
                    manager->setContactDistanceThreshold(contact_distance_);
                    manager->setNegativeDistanceThreshold(negative_distance_);
                    manager->setIsContactAllowedFn(fn_);

                    for (const auto& cow : link2cow_)
                    {
                        COWPtr new_cow = cow.second->clone();

                        assert(new_cow->getCollisionShape());
                        assert(new_cow->getCollisionShape()->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);

                        new_cow->setWorldTransform(cow.second->getWorldTransform());
                        new_cow->setContactProcessingThreshold(static_cast<btScalar>(contact_distance_));
                        manager->addCollisionObject(new_cow);
                    }

                    return manager;
                }

                bool BulletDiscreteBVHManager::addCollisionObject(const std::string& name,
                                                          const int& mask_id,
                                                          const CollisionShapes& shapes,
                                                          const geometries::VectorIsometry3d& shape_poses,
                                                          bool enabled)
                {
                    if (link2cow_.find(name) != link2cow_.end())
                        removeCollisionObject(name);

                    COWPtr new_cow = createCollisionObject(name, mask_id, shapes, shape_poses, enabled);
                    if (new_cow != nullptr)
                    {
                        new_cow->setContactProcessingThreshold(static_cast<btScalar>(contact_distance_));
                        addCollisionObject(new_cow);
                        return true;
                    }

                    return false;
                }

                const CollisionShapes& BulletDiscreteBVHManager::getCollisionObjectGeometries(const std::string& name) const
                {
                    auto cow = link2cow_.find(name);
                    return (cow != link2cow_.end()) ? cow->second->getCollisionGeometries() : EMPTY_COLLISION_SHAPES_CONST;
                }

                const CollisionShapePtr BulletDiscreteBVHManager::getCollisionObjectGeometry(const std::string& name, std::size_t sub) const
                {
                    auto cow = link2cow_.find(name);
                    return (cow != link2cow_.end()) ? cow->second->getCollisionGeometry(sub) : nullptr;
                }


                const geometries::VectorIsometry3d& BulletDiscreteBVHManager::getCollisionObjectGeometriesTransforms(const std::string& name) const
                {
                    auto cow = link2cow_.find(name);
                    return (cow != link2cow_.end()) ? cow->second->getCollisionGeometriesTransforms() : EMPTY_COLLISION_SHAPES_TRANSFORMS;
                }

                const Eigen::Isometry3d BulletDiscreteBVHManager::getCollisionObjectGeometryTransform(const std::string& name, std::size_t sub) const
                {
                    auto cow = link2cow_.find(name);
                    return (cow != link2cow_.end()) ? cow->second->getCollisionGeometryTransform(sub) : Eigen::Isometry3d::Identity();
                }

                bool BulletDiscreteBVHManager::hasCollisionObject(const std::string& name) const
                {
                    return (link2cow_.find(name) != link2cow_.end());
                }

                bool BulletDiscreteBVHManager::removeCollisionObject(const std::string& name)
                {
                    auto it = link2cow_.find(name);  // Levi TODO: Should these check be removed?
                    if (it != link2cow_.end())
                    {
                        removeCollisionObjectFromBroadphase(it->second, broadphase_, dispatcher_);
                        link2cow_.erase(name);
                        return true;
                    }

                    return false;
                }

                bool BulletDiscreteBVHManager::enableCollisionObject(const std::string& name)
                {
                    auto it = link2cow_.find(name);  // Levi TODO: Should these check be removed?
                    if (it != link2cow_.end())
                    {
                        it->second->m_enabled = true;

                        // Need to clean the proxy from broadphase cache so BroadPhaseFilter gets called again.
                        // The BroadPhaseFilter only gets called once, so if you change when two objects can be in collision, like filters
                        // this must be called or contacts between shapes will be missed.
                        if (it->second->getBroadphaseHandle())
                            broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(it->second->getBroadphaseHandle(), dispatcher_.get());

                        return true;
                    }
                    return false;
                }

                bool BulletDiscreteBVHManager::disableCollisionObject(const std::string& name)
                {
                    auto it = link2cow_.find(name);  // Levi TODO: Should these check be removed?
                    if (it != link2cow_.end())
                    {
                        it->second->m_enabled = false;
                        // Need to clean the proxy from broadphase cache so BroadPhaseFilter gets called again.
                        // The BroadPhaseFilter only gets called once, so if you change when two objects can be in collision, like filters
                        // this must be called or contacts between shapes will be missed.
                        if (it->second->getBroadphaseHandle())
                            broadphase_->getOverlappingPairCache()->cleanProxyFromPairs(it->second->getBroadphaseHandle(), dispatcher_.get());
                        return true;
                    }
                    return false;
                }

                void BulletDiscreteBVHManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose)
                {
                    // TODO: Find a way to remove this check. Need to store information in Tesseract EnvState indicating transforms with
                    // geometry
                    auto it = link2cow_.find(name);
                    if (it != link2cow_.end())
                    {
                        COWPtr& cow = it->second;
                        cow->setWorldTransform(convertEigenToBt(pose));

                        // Update Collision Object Broadphase AABB
                        updateBroadphaseAABB(cow, broadphase_, dispatcher_);
                    }
                }

                void BulletDiscreteBVHManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                                    const geometries::VectorIsometry3d& poses)
                {
                    assert(names.size() == poses.size());
                    for (auto i = 0u; i < names.size(); ++i)
                        setCollisionObjectsTransform(names[i], poses[i]);
                }

                void BulletDiscreteBVHManager::setCollisionObjectsTransform(const geometries::TransformMap& transforms)
                {
                    for (const auto& transform : transforms)
                        setCollisionObjectsTransform(transform.first, transform.second);
                }

                void BulletDiscreteBVHManager::setActiveCollisionObjects(const std::vector<std::string>& names)
                {
                    active_ = names;

                    // Now need to update the broadphase with correct aabb
                    for (auto& co : link2cow_)
                    {
                        COWPtr& cow = co.second;

                        updateCollisionObjectFilters(active_, cow, broadphase_, dispatcher_);
                    }
                }

                const std::vector<std::string>& BulletDiscreteBVHManager::getActiveCollisionObjects() const { return active_; }

                void BulletDiscreteBVHManager::setContactDistanceThreshold(double contact_distance)
                {
                    contact_distance_ = contact_distance;

                    for (auto& co : link2cow_)
                    {
                        COWPtr& cow = co.second;
                        cow->setContactProcessingThreshold(static_cast<btScalar>(contact_distance));
                        assert(cow->getBroadphaseHandle() != nullptr);
                        updateBroadphaseAABB(cow, broadphase_, dispatcher_);
                    }
                }

                double BulletDiscreteBVHManager::getContactDistanceThreshold() const { return contact_distance_; }

                void BulletDiscreteBVHManager::setNegativeDistanceThreshold(double negative_distance)
                {
                    negative_distance_ = negative_distance;
                }

                double BulletDiscreteBVHManager::getNegativeDistanceThreshold() const { return negative_distance_; }

                void BulletDiscreteBVHManager::setIsContactAllowedFn(IsContactAllowedFn fn) { fn_ = fn; }

                IsContactAllowedFn BulletDiscreteBVHManager::getIsContactAllowedFn() const { return fn_; }

                void BulletDiscreteBVHManager::contactTest(base::ContactResultMap& collisions, const ContactTestType& type, unsigned int max_contacts)
                {
                    ContactTestData cdata(active_, contact_distance_, negative_distance_, fn_, type, collisions, max_contacts);

                    broadphase_->calculateOverlappingPairs(dispatcher_.get());

                    btOverlappingPairCache* pairCache = broadphase_->getOverlappingPairCache();

                    DiscreteBroadphaseContactResultCallback cc(cdata, contact_distance_);

                    TesseractCollisionPairCallback collisionCallback(dispatch_info_, dispatcher_.get(), cc);

                    pairCache->processAllOverlappingPairs(&collisionCallback, dispatcher_.get());
                }

                void BulletDiscreteBVHManager::addCollisionObject(const COWPtr& cow)
                {
                    link2cow_[cow->getName()] = cow;

                    // Add collision object to broadphase
                    addCollisionObjectToBroadphase(cow, broadphase_, dispatcher_);
                }

                const Link2Cow& BulletDiscreteBVHManager::getCollisionObjects() const { return link2cow_; }
            }
        }
    }
}
