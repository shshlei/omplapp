/**
 * @file bullet_utils.h
 * @brief Tesseract ROS Bullet environment utility function.
 *
 * @author John Schulman
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @author Shi Shenglei 
 * @date April 1, 2020
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 * @copyright Copyright (c) 2013, John Schulman
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

#ifndef OMPLAPP_COLLISION_BULLET_UTILS_
#define OMPLAPP_COLLISION_BULLET_UTILS_

#include <btBulletCollisionCommon.h>

#include "omplapp/collision/Common.h"

#include "ompl/util/Console.h"
#include "ompl/util/ClassForward.h"

namespace ompl 
{
    namespace app 
    {
        namespace collision
        {
            namespace collision_bullet
            {
                #define METERS

                const btScalar BULLET_MARGIN = 0.0f;
                const btScalar BULLET_SUPPORT_FUNC_TOLERANCE = 0.01f METERS;
                const btScalar BULLET_LENGTH_TOLERANCE = 0.001f METERS;
                const btScalar BULLET_EPSILON = 1e-3f;
                const btScalar BULLET_DEFAULT_CONTACT_DISTANCE = 0.05f;
                const bool BULLET_COMPOUND_USE_DYNAMIC_AABB = true;

                inline btVector3 convertEigenToBt(const Eigen::Vector3d& v)
                {
                    return btVector3{ static_cast<btScalar>(v[0]), static_cast<btScalar>(v[1]), static_cast<btScalar>(v[2]) };
                }

                inline Eigen::Vector3d convertBtToEigen(const btVector3& v)
                {
                    return Eigen::Vector3d{ static_cast<double>(v.x()), static_cast<double>(v.y()), static_cast<double>(v.z()) };
                }

                inline btQuaternion convertEigenToBt(const Eigen::Quaterniond& q)
                {
                    return btQuaternion{ static_cast<btScalar>(q.x()),
                                   static_cast<btScalar>(q.y()),
                                   static_cast<btScalar>(q.z()),
                                   static_cast<btScalar>(q.w()) };
                }

                inline btMatrix3x3 convertEigenToBt(const Eigen::Matrix3d& r)
                {
                    return btMatrix3x3{ static_cast<btScalar>(r(0, 0)), static_cast<btScalar>(r(0, 1)), static_cast<btScalar>(r(0, 2)),
                                  static_cast<btScalar>(r(1, 0)), static_cast<btScalar>(r(1, 1)), static_cast<btScalar>(r(1, 2)),
                                  static_cast<btScalar>(r(2, 0)), static_cast<btScalar>(r(2, 1)), static_cast<btScalar>(r(2, 2)) };
                }

                inline Eigen::Matrix3d convertBtToEigen(const btMatrix3x3& r)
                {
                    Eigen::Matrix3d m;
                    m << static_cast<double>(r[0][0]), static_cast<double>(r[0][1]), static_cast<double>(r[0][2]),
                    static_cast<double>(r[1][0]), static_cast<double>(r[1][1]), static_cast<double>(r[1][2]),
                    static_cast<double>(r[2][0]), static_cast<double>(r[2][1]), static_cast<double>(r[2][2]);
                    return m;
                }

                inline btTransform convertEigenToBt(const Eigen::Isometry3d& t)
                {
                    const Eigen::Matrix3d& rot = t.matrix().block<3, 3>(0, 0);
                    const Eigen::Vector3d& tran = t.translation();

                    return btTransform{ convertEigenToBt(rot), convertEigenToBt(tran) };
                }

                inline Eigen::Isometry3d convertBtToEigen(const btTransform& t)
                {
                    Eigen::Isometry3d i = Eigen::Isometry3d::Identity();
                    i.linear() = convertBtToEigen(t.getBasis());
                    i.translation() = convertBtToEigen(t.getOrigin());

                    return i;
                }

                OMPL_CLASS_FORWARD(CollisionObjectWrapper);

                /**
                * @brief This is a tesseract bullet collsion object.
                *
                * It is a wrapper around bullet's collision object which
                * contains specific information related to tesseract
                */
                class CollisionObjectWrapper : public btCollisionObject
                {
                public:
                    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                    CollisionObjectWrapper(std::string name,
                                     const int& type_id,
                                     const CollisionShapes shapes,
                                     const geometries::VectorIsometry3d shape_poses);

                    short int m_collisionFilterGroup;
                    short int m_collisionFilterMask;
                    bool m_enabled{ true };

                    /** @brief Get the collision object name */
                    const std::string& getName() const { return m_name; }

                    /** @brief Get a user defined type */
                    const int& getTypeID() const { return m_type_id; }

                    /** \brief Check if two CollisionObjectWrapper objects point to the same source object */
                    bool sameObject(const CollisionObjectWrapper& other) const
                    {
                        return m_name == other.m_name && m_type_id == other.m_type_id && m_shapes.size() == other.m_shapes.size() &&
                           m_shape_poses.size() == other.m_shape_poses.size() &&
                           std::equal(m_shapes.begin(), m_shapes.end(), other.m_shapes.begin()) &&
                           std::equal(m_shape_poses.begin(),
                                      m_shape_poses.end(),
                                      other.m_shape_poses.begin(),
                                      [](const Eigen::Isometry3d& t1, const Eigen::Isometry3d& t2) { return t1.isApprox(t2); });
                    }

                    const CollisionShapes& getCollisionGeometries() const
                    {
                        return m_shapes;
                    }

                    const CollisionShapePtr& getCollisionGeometry(std::size_t sub = 0) const
                    {
                        return m_shapes[sub];
                    }

                    const geometries::VectorIsometry3d& getCollisionGeometriesTransforms() const
                    {
                        return m_shape_poses;
                    }

                    const Eigen::Isometry3d& getCollisionGeometryTransform(std::size_t sub = 0) const
                    {
                        return m_shape_poses[sub];
                    }

                    /**
                    * @brief Get the collision objects axis aligned bounding box
                    * @param aabb_min The minimum point
                    * @param aabb_max The maximum point
                    */
                    void getAABB(btVector3& aabb_min, btVector3& aabb_max) const
                    {
                        getCollisionShape()->getAabb(getWorldTransform(), aabb_min, aabb_max);
                        const btScalar& d = getContactProcessingThreshold();
                        btVector3 contactThreshold(d, d, d);
                        aabb_min -= contactThreshold;
                        aabb_max += contactThreshold;
                    }

                    /**
                    * @brief This clones the collision objects but not the collision shape wich is const.
                    * @return Shared Pointer to the cloned collision object
                    */
                    std::shared_ptr<CollisionObjectWrapper> clone()
                    {
                        std::shared_ptr<CollisionObjectWrapper> clone_cow(new CollisionObjectWrapper(m_name, m_type_id, m_shapes, m_shape_poses, m_data));
                        clone_cow->setCollisionShape(getCollisionShape());
                        clone_cow->setWorldTransform(getWorldTransform());
                        clone_cow->m_collisionFilterGroup = m_collisionFilterGroup;
                        clone_cow->m_collisionFilterMask = m_collisionFilterMask;
                        clone_cow->m_enabled = m_enabled;
                        clone_cow->setBroadphaseHandle(nullptr);
                        return clone_cow;
                    }

                    template <class T>
                    void manage(T* t)
                    {  // manage memory of this object
                        m_data.push_back(std::shared_ptr<T>(t));
                    }
                    template <class T>
                    void manage(std::shared_ptr<T> t)
                    {
                        m_data.push_back(t);
                    }

                protected:
                    /** @brief This is a special constructor used by the clone method */
                    CollisionObjectWrapper(std::string name,
                                     const int& type_id,
                                     const CollisionShapes shapes,
                                     const geometries::VectorIsometry3d shape_poses,
                                     std::vector<std::shared_ptr<void>> data);

                    std::string m_name;                               /**< @brief The name of the collision object */
                    int m_type_id;                                    /**< @brief A user defined type id */
                    CollisionShapes m_shapes;                    /**< @brief The shapes that define the collison object */
                    geometries::VectorIsometry3d m_shape_poses; /**< @brief The shpaes poses information */
                    std::vector<std::shared_ptr<void>> m_data; /**< @brief This manages the collision shape pointer so they get destroyed */
                };

                using COW = CollisionObjectWrapper;
                using COWPtr = CollisionObjectWrapperPtr;
                using Link2Cow = std::map<std::string, COWPtr>;
                using Link2ConstCow = std::map<std::string, const COWPtr>;

                /**
                * @brief Create a bullet collision shape from tesseract collision shape
                * @param geom Tesseract collision shape
                * @param cow The collision object wrapper the collision shape is associated with
                * @param shape_index The collision shapes index within the collision shape wrapper. This can be accessed from the
                * bullet collision shape by calling getUserIndex function.
                * @return Bullet collision shape.
                */
                btCollisionShape* createShapePrimitive(const CollisionShapePtr& geom, CollisionObjectWrapper* cow, int shape_index);


                inline COWPtr createCollisionObject(const std::string& name,
                                              const int& type_id,
                                              const CollisionShapes& shapes,
                                              const geometries::VectorIsometry3d& shape_poses,
                                              bool enabled = true)
                {
                    // dont add object that does not have geometry
                    if (shapes.empty() || shape_poses.empty() || (shapes.size() != shape_poses.size()))
                    {
                        OMPL_DEBUG("ignoring link %s", name.c_str());
                        return nullptr;
                    }

                    COWPtr new_cow(new COW(name, type_id, shapes, shape_poses));

                    new_cow->m_enabled = enabled;
                    new_cow->setContactProcessingThreshold(BULLET_DEFAULT_CONTACT_DISTANCE);

                    return new_cow;
                }

                /**
                * @brief Update a collision objects filters
                * @param active A list of active collision objects
                * @param cow The collision object to update.
                * @param continuous Indicate if the object is a continuous collision object.
                *
                * Currently continuous collision objects can only be checked against static objects. Continuous to Continuous
                * collision checking is currently not supports. TODO LEVI: Add support for Continuous to Continuous collision checking.
                */
                inline void updateCollisionObjectFilters(const std::vector<std::string>& active, const COWPtr& cow)
                {
                    cow->m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;

                    if (!isLinkActive(active, cow->getName()))
                    {
                        cow->m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
                    }

                    if (cow->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
                    {
                        cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
                    }
                    else
                    {
                        cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter;
                    }

                    if (cow->getBroadphaseHandle())
                    {
                        cow->getBroadphaseHandle()->m_collisionFilterGroup = cow->m_collisionFilterGroup;
                        cow->getBroadphaseHandle()->m_collisionFilterMask = cow->m_collisionFilterMask;
                    }
                }

                /**
                 * @brief Update a collision objects filters for broadphase
                 * @param active A list of active collision objects
                 * @param cow The collision object to update.
                 * @param broadphase The collision object to update.
                 * @param dispatcher The collision object to update.
                 */
                inline void updateCollisionObjectFilters(const std::vector<std::string>& active,
                        const COWPtr& cow,
                        const std::unique_ptr<btBroadphaseInterface>& broadphase,
                        const std::unique_ptr<btCollisionDispatcher>& dispatcher)
                {
                    updateCollisionObjectFilters(active, cow);

                    // Need to clean the proxy from broadphase cache so BroadPhaseFilter gets called again.
                    // The BroadPhaseFilter only gets called once, so if you change when two objects can be in collision, like filters
                    // this must be called or contacts between shapes will be missed.
                    broadphase->getOverlappingPairCache()->cleanProxyFromPairs(cow->getBroadphaseHandle(), dispatcher.get());
                }

                /**
                * @brief Update the Broadphase AABB for the input collision object
                * @param cow The collision objects
                * @param broadphase The bullet broadphase interface
                * @param dispatcher The bullet collision dispatcher
                */
                inline void updateBroadphaseAABB(const COWPtr& cow,
                                         const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                         const std::unique_ptr<btCollisionDispatcher>& dispatcher)
                {
                    // Calculate the aabb
                    btVector3 aabb_min, aabb_max;
                    cow->getAABB(aabb_min, aabb_max);

                    // Update the broadphase aabb
                    assert(cow->getBroadphaseHandle() != nullptr);
                    broadphase->setAabb(cow->getBroadphaseHandle(), aabb_min, aabb_max, dispatcher.get());
                }

                /**
                * @brief Remove the collision object from broadphase
                * @param cow The collision objects
                * @param broadphase The bullet broadphase interface
                * @param dispatcher The bullet collision dispatcher
                */
                inline void removeCollisionObjectFromBroadphase(const COWPtr& cow,
                                                        const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                                        const std::unique_ptr<btCollisionDispatcher>& dispatcher)
                {
                    btBroadphaseProxy* bp = cow->getBroadphaseHandle();
                    if (bp)
                    {
                        // only clear the cached algorithms
                        broadphase->getOverlappingPairCache()->cleanProxyFromPairs(bp, dispatcher.get());
                        broadphase->destroyProxy(bp, dispatcher.get());
                        cow->setBroadphaseHandle(nullptr);
                    }
                }

                /**
                * @brief Add the collision object to broadphase
                * @param cow The collision objects
                * @param broadphase The bullet broadphase interface
                * @param dispatcher The bullet collision dispatcher
                */
                inline void addCollisionObjectToBroadphase(const COWPtr& cow,
                                                   const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                                   const std::unique_ptr<btCollisionDispatcher>& dispatcher)
                {
                    btVector3 aabb_min, aabb_max;
                    cow->getAABB(aabb_min, aabb_max);

                    // Add the active collision object to the broadphase
                    int type = cow->getCollisionShape()->getShapeType();
                    cow->setBroadphaseHandle(broadphase->createProxy(aabb_min, aabb_max, type, cow.get(), cow->m_collisionFilterGroup, cow->m_collisionFilterMask, dispatcher.get()));
                }

                inline void GetAverageSupport(const btConvexShape* shape, const btVector3& localNormal, btScalar& outsupport, btVector3& outpt)
                {
                    btVector3 ptSum(0, 0, 0);
                    btScalar ptCount = 0;
                    btScalar maxSupport = -1000;

                    const auto* pshape = dynamic_cast<const btPolyhedralConvexShape*>(shape);
                    if (pshape)
                    {
                        int nPts = pshape->getNumVertices();

                        for (int i = 0; i < nPts; ++i)
                        {
                            btVector3 pt;
                            pshape->getVertex(i, pt);

                            btScalar sup = pt.dot(localNormal);
                            if (sup > maxSupport + BULLET_EPSILON)
                            {
                                ptCount = 1;
                                ptSum = pt;
                                maxSupport = sup;
                            }
                            else if (sup < maxSupport - BULLET_EPSILON)
                            {
                            }
                            else
                            {
                                ptCount += 1;
                                ptSum += pt;
                            }
                        }
                        outsupport = maxSupport;
                        outpt = ptSum / ptCount;
                    }
                    else
                    {
                        // The margins are set to zero for most shapes, but for a sphere the margin is used so must use
                        // localGetSupportingVertex instead of localGetSupportingVertexWithoutMargin.
                        outpt = shape->localGetSupportingVertex(localNormal);
                        outsupport = localNormal.dot(outpt);
                    }
                }

                /**
                * @brief This transversus the parent tree to find the base object and return its world transform
                * This should be the links transform and it should only need to transverse twice at max.
                * @param cow Bullet collision object wrapper.
                * @return Transform of link in world coordinates
                */
                inline btTransform getLinkTransformFromCOW(const btCollisionObjectWrapper* cow)
                {
                    if (cow->m_parent)
                    {
                        if (cow->m_parent->m_parent)
                        {
                            assert(cow->m_parent->m_parent->m_parent == nullptr);
                            return cow->m_parent->m_parent->getWorldTransform();
                        }

                        return cow->m_parent->getWorldTransform();
                    }

                    return cow->getWorldTransform();
                }

                /**
                * @brief This is used to check if a collision check is required between the provided two collision objects
                * @param cow1 The first collision object
                * @param cow2 The second collision object
                * @param acm  The contact allowed function pointer
                * @param verbose Indicate if verbose information should be printed to the terminal
                * @return True if the two collision objects should be checked for collision, otherwise false
                */
                inline bool needsCollisionCheck(const COW& cow1, const COW& cow2, const IsContactAllowedFn& acm, bool verbose = false)
                {
                    return cow1.m_enabled && cow2.m_enabled && (cow2.m_collisionFilterGroup & cow1.m_collisionFilterMask) &&
                        (cow1.m_collisionFilterGroup & cow2.m_collisionFilterMask) &&
                        !isContactAllowed(cow1.getName(), cow2.getName(), acm, verbose);
                }

                inline btScalar addDiscreteSingleResult(btManifoldPoint& cp,
                                                const btCollisionObjectWrapper* colObj0Wrap,
                                                const btCollisionObjectWrapper* colObj1Wrap,
                                                ContactTestData& collisions)
                {
                    assert(dynamic_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject()) != nullptr);
                    assert(dynamic_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject()) != nullptr);
                    const auto* cd0 = static_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject());
                    const auto* cd1 = static_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject());

                    ObjectPairKey pc = getObjectPairKey(cd0->getName(), cd1->getName());

                    const auto& it = collisions.res.find(pc);
                    bool found = (it != collisions.res.end());

                    btTransform tf0 = getLinkTransformFromCOW(colObj0Wrap);
                    btTransform tf1 = getLinkTransformFromCOW(colObj1Wrap);
                    btTransform tf0_inv = tf0.inverse();
                    btTransform tf1_inv = tf1.inverse();

                    base::ContactResult contact;
                    contact.link_names[0] = cd0->getName();
                    contact.link_names[1] = cd1->getName();
                    contact.shape_id[0] = colObj0Wrap->getCollisionShape()->getUserIndex();
                    contact.shape_id[1] = colObj1Wrap->getCollisionShape()->getUserIndex();
                    contact.subshape_id[0] = colObj0Wrap->m_index;
                    contact.subshape_id[1] = colObj1Wrap->m_index;
                    contact.nearest_points[0] = convertBtToEigen(cp.m_positionWorldOnA);
                    contact.nearest_points[1] = convertBtToEigen(cp.m_positionWorldOnB);
                    contact.nearest_points_local[0] = convertBtToEigen(tf0_inv * cp.m_positionWorldOnA);
                    contact.nearest_points_local[1] = convertBtToEigen(tf1_inv * cp.m_positionWorldOnB);
                    contact.nearest_points_local2[0] = convertBtToEigen(tf0_inv * cp.m_positionWorldOnB);
                    contact.nearest_points_local2[1] = convertBtToEigen(tf1_inv * cp.m_positionWorldOnA);
                    contact.transform[0] = convertBtToEigen(tf0);
                    contact.transform[1] = convertBtToEigen(tf1);
                    contact.type_id[0] = cd0->getTypeID();
                    contact.type_id[1] = cd1->getTypeID();
                    contact.distance = static_cast<double>(cp.m_distance1);
                    contact.normal = convertBtToEigen(-1 * cp.m_normalWorldOnB);

                    if (!processResult(collisions, contact, pc, found))
                    {
                        return 0;
                    }

                    return 1;
                }

                /** @brief The BroadphaseContactResultCallback is used to report contact points */
                struct BroadphaseContactResultCallback
                {
                    ContactTestData& collisions_;
                    double contact_distance_;
                    bool verbose_;

                    BroadphaseContactResultCallback(ContactTestData& collisions, double contact_distance, bool verbose = false)
                        : collisions_(collisions), contact_distance_(contact_distance), verbose_(verbose)
                    {
                    }

                    virtual ~BroadphaseContactResultCallback() = default;
                    BroadphaseContactResultCallback(const BroadphaseContactResultCallback&) = default;
                    BroadphaseContactResultCallback& operator=(const BroadphaseContactResultCallback&) = delete;
                    BroadphaseContactResultCallback(BroadphaseContactResultCallback&&) = default;
                    BroadphaseContactResultCallback& operator=(BroadphaseContactResultCallback&&) = delete;

                    virtual bool needsCollision(const CollisionObjectWrapper* cow0, const CollisionObjectWrapper* cow1) const
                    {
                        return !collisions_.done && needsCollisionCheck(*cow0, *cow1, collisions_.fn, verbose_);
                    }

                    virtual btScalar addSingleResult(btManifoldPoint& cp,
                                               const btCollisionObjectWrapper* colObj0Wrap,
                                               int partId0,
                                               int index0,
                                               const btCollisionObjectWrapper* colObj1Wrap,
                                               int partId1,
                                               int index1) = 0;
                };

                struct DiscreteBroadphaseContactResultCallback : public BroadphaseContactResultCallback
                {
                    DiscreteBroadphaseContactResultCallback(ContactTestData& collisions, double contact_distance, bool verbose = false)
                        : BroadphaseContactResultCallback(collisions, contact_distance, verbose)
                    {
                    }

                    btScalar addSingleResult(btManifoldPoint& cp,
                                       const btCollisionObjectWrapper* colObj0Wrap,
                                       int /*partId0*/,
                                       int /*index0*/,
                                       const btCollisionObjectWrapper* colObj1Wrap,
                                       int /*partId1*/,
                                       int /*index1*/) override
                    {
                        if (cp.m_distance1 > static_cast<btScalar>(contact_distance_))
                            return 0;

                        return addDiscreteSingleResult(cp, colObj0Wrap, colObj1Wrap, collisions_);
                    }
                };

                struct TesseractBroadphaseBridgedManifoldResult : public btManifoldResult
                {
                    BroadphaseContactResultCallback& result_callback_;

                    TesseractBroadphaseBridgedManifoldResult(const btCollisionObjectWrapper* obj0Wrap,
                                                       const btCollisionObjectWrapper* obj1Wrap,
                                                       BroadphaseContactResultCallback& result_callback)
                    : btManifoldResult(obj0Wrap, obj1Wrap), result_callback_(result_callback)
                    {
                    }

                    void addContactPoint(const btVector3& normalOnBInWorld, const btVector3& pointInWorld, btScalar depth) override
                    {
                        if (result_callback_.collisions_.done || depth > static_cast<btScalar>(result_callback_.contact_distance_))
                            return;

                        bool isSwapped = m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject();
                        btVector3 pointA = pointInWorld + normalOnBInWorld * depth;
                        btVector3 localA;
                        btVector3 localB;
                        if (isSwapped)
                        {
                            localA = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointA);
                            localB = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
                        }
                        else
                        {
                            localA = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointA);
                            localB = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
                        }

                        btManifoldPoint newPt(localA, localB, normalOnBInWorld, depth);
                        newPt.m_positionWorldOnA = pointA;
                        newPt.m_positionWorldOnB = pointInWorld;

                        // BP mod, store contact triangles.
                        if (isSwapped)
                        {
                            newPt.m_partId0 = m_partId1;
                            newPt.m_partId1 = m_partId0;
                            newPt.m_index0 = m_index1;
                            newPt.m_index1 = m_index0;
                        }
                        else
                        {
                            newPt.m_partId0 = m_partId0;
                            newPt.m_partId1 = m_partId1;
                            newPt.m_index0 = m_index0;
                            newPt.m_index1 = m_index1;
                        }

                        // experimental feature info, for per-triangle material etc.
                        const btCollisionObjectWrapper* obj0Wrap = isSwapped ? m_body1Wrap : m_body0Wrap;
                        const btCollisionObjectWrapper* obj1Wrap = isSwapped ? m_body0Wrap : m_body1Wrap;
                        result_callback_.addSingleResult(newPt, obj0Wrap, newPt.m_partId0, newPt.m_index0, obj1Wrap, newPt.m_partId1, newPt.m_index1);
                    }
                };

                /**
                * @brief A callback function that is called as part of the broadphase collision checking.
                *
                * If the AABB of two collision objects are overlapping the processOverlap method is called
                * and they are checked for collision/distance and the results are stored in collision_.
                */
                class TesseractCollisionPairCallback : public btOverlapCallback
                {
                    const btDispatcherInfo& dispatch_info_;
                    btCollisionDispatcher* dispatcher_;
                    BroadphaseContactResultCallback& results_callback_;

                public:
                    TesseractCollisionPairCallback(const btDispatcherInfo& dispatchInfo,
                                                   btCollisionDispatcher* dispatcher,
                                                   BroadphaseContactResultCallback& results_callback)
                    : dispatch_info_(dispatchInfo), dispatcher_(dispatcher), results_callback_(results_callback)
                    {
                    }

                    ~TesseractCollisionPairCallback() override = default;
                    TesseractCollisionPairCallback(const TesseractCollisionPairCallback&) = default;
                    TesseractCollisionPairCallback& operator=(const TesseractCollisionPairCallback&) = delete;
                    TesseractCollisionPairCallback(TesseractCollisionPairCallback&&) = default;
                    TesseractCollisionPairCallback& operator=(TesseractCollisionPairCallback&&) = delete;

                    bool processOverlap(btBroadphasePair& pair) override
                    {
                        if (results_callback_.collisions_.done)
                            return false;

                        const auto* cow0 = static_cast<const CollisionObjectWrapper*>(pair.m_pProxy0->m_clientObject);
                        const auto* cow1 = static_cast<const CollisionObjectWrapper*>(pair.m_pProxy1->m_clientObject);

                        if (results_callback_.needsCollision(cow0, cow1))
                        {
                            btCollisionObjectWrapper obj0Wrap(nullptr, cow0->getCollisionShape(), cow0, cow0->getWorldTransform(), -1, -1);
                            btCollisionObjectWrapper obj1Wrap(nullptr, cow1->getCollisionShape(), cow1, cow1->getWorldTransform(), -1, -1);

                            // dispatcher will keep algorithms persistent in the collision pair
                            if (!pair.m_algorithm)
                            {
                                pair.m_algorithm = dispatcher_->findAlgorithm(&obj0Wrap, &obj1Wrap, nullptr, BT_CLOSEST_POINT_ALGORITHMS);
                            }

                            if (pair.m_algorithm)
                            {
                                TesseractBroadphaseBridgedManifoldResult contactPointResult(&obj0Wrap, &obj1Wrap, results_callback_);
                                contactPointResult.m_closestPointDistanceThreshold = static_cast<btScalar>(results_callback_.contact_distance_);

                                // discrete collision detection query
                                pair.m_algorithm->processCollision(&obj0Wrap, &obj1Wrap, dispatch_info_, &contactPointResult);
                            }
                        }
                        return false;
                    }
                };

                /** @brief This class is used to filter broadphase */
                class TesseractOverlapFilterCallback : public btOverlapFilterCallback
                {
                public:
                    TesseractOverlapFilterCallback(bool verbose = false) : verbose_(verbose) {}

                    bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const override
                    {
                        // Note: We do not pass the allowed collision matrix because if it changes we do not know and this function only
                        // gets called under certain cases and it could cause overlapping pairs to not be processed.
                        return needsCollisionCheck(*(static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject)), *(static_cast<CollisionObjectWrapper*>(proxy1->m_clientObject)), nullptr, verbose_);
                    }

                private:
                    bool verbose_{ false };
                };
            }
        }
    }
}

#endif
