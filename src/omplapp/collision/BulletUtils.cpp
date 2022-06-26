/**
 * @file bullet_utils.cpp
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

#include "omplapp/collision/BulletUtils.h"
#include <BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <boost/thread/mutex.hpp>
#include <memory>
#include <octomap/octomap.h>

namespace ompl 
{
    namespace app 
    {
        namespace collision
        {
            namespace collision_bullet
            {
                btCollisionShape* createShapePrimitive(const CollisionShapePtr& geo, CollisionObjectWrapper* cow, int shape_index)
                {
                    btCollisionShape* shape = nullptr;

                    switch (geo->getType())
                    {
                        case geometries::GeometryType::BOX:
                        {
                            auto geom = geo->as<geometries::Box>();
                            auto a = static_cast<btScalar>(geom->getX() / 2);
                            auto b = static_cast<btScalar>(geom->getY() / 2);
                            auto c = static_cast<btScalar>(geom->getZ() / 2);

                            shape = new btBoxShape(btVector3(a, b, c));

                            shape->setUserIndex(shape_index);
                            shape->setMargin(BULLET_MARGIN);
                            break;
                        }
                        case geometries::GeometryType::SPHERE:
                        {
                            auto geom = geo->as<geometries::Sphere>();
                            shape =  new btSphereShape(static_cast<btScalar>(geom->getRadius()));
                            shape->setUserIndex(shape_index);
                            // Sphere is a special case where you do not modify the margin which is internally set to the radius
                            break;
                        }
                        case geometries::GeometryType::CYLINDER:
                        {
                            auto geom = geo->as<geometries::Cylinder>();
                            auto r = static_cast<btScalar>(geom->getRadius());
                            auto l = static_cast<btScalar>(geom->getLength() / 2);
                            shape = new btCylinderShapeZ(btVector3(r, r, l));

                            shape->setUserIndex(shape_index);
                            shape->setMargin(BULLET_MARGIN);
                            break;
                        }
                        case geometries::GeometryType::CONE:
                        {
                            auto geom = geo->as<geometries::Cone>();
                            double r = static_cast<btScalar>(geom->getRadius());
                            double l = static_cast<btScalar>(geom->getLength());
                            shape = new btConeShapeZ(r, l);

                            shape->setUserIndex(shape_index);
                            shape->setMargin(BULLET_MARGIN);
                            break;
                        }
                        case geometries::GeometryType::CAPSULE:
                        {
                            auto geom = geo->as<geometries::Capsule>();
                            auto r = static_cast<btScalar>(geom->getRadius());
                            auto l = static_cast<btScalar>(geom->getLength());
                            shape = new btCapsuleShapeZ(r, l);

                            shape->setUserIndex(shape_index);
                            shape->setMargin(BULLET_MARGIN);
                            break;
                        }
                        case geometries::GeometryType::MESH:
                        {
                            auto geom = geo->as<geometries::Mesh>();
                            int vertice_count = geom->getVerticeCount();
                            int triangle_count = geom->getTriangleCount();
                            const geometries::VectorVector3d& vertices = *(geom->getVertices());
                            const Eigen::VectorXi& triangles = *(geom->getTriangles());

                            if (vertice_count > 0 && triangle_count > 0)
                            {
                                auto* compound = new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, static_cast<int>(triangle_count));
                                compound->setMargin(BULLET_MARGIN);  // margin: compound. seems to have no
                                                                 // effect when positive but has an
                                                                 // effect when negative

                                for (int i = 0; i < triangle_count; ++i)
                                {
                                    btVector3 v[3];
                                    assert(triangles[4 * i] == 3);
                                    for (unsigned x = 0; x < 3; ++x)
                                    {
                                        // Note: triangles structure is number of vertices that represent the triangle followed by vertex indexes
                                        const Eigen::Vector3d& vertice = vertices[static_cast<size_t>(triangles[(4 * i) + (static_cast<int>(x) + 1)])];
                                        for (unsigned y = 0; y < 3; ++y)
                                            v[x][y] = static_cast<btScalar>(vertice[y]);
                                    }

                                    btCollisionShape* subshape = new btTriangleShapeEx(v[0], v[1], v[2]);
                                    if (subshape != nullptr)
                                    {
                                        subshape->setUserIndex(shape_index);
                                        cow->manage(subshape);
                                        subshape->setMargin(BULLET_MARGIN);
                                        btTransform geomTrans;
                                        geomTrans.setIdentity();
                                        compound->addChildShape(geomTrans, subshape);
                                    }
                                }

                                shape = compound;
                            }

                            if (shape == nullptr)
                                OMPL_ERROR("The mesh is empty!");

                            shape->setUserIndex(shape_index);
                            shape->setMargin(BULLET_MARGIN);
                            break;
                        }
                        case geometries::GeometryType::CONVEX_MESH:
                        {
                            auto geom = geo->as<geometries::ConvexMesh>();
                            int vertice_count = geom->getVerticeCount();
                            int triangle_count = geom->getFaceCount();
                            const geometries::VectorVector3d& vertices = *(geom->getVertices());

                            if (vertice_count > 0 && triangle_count > 0)
                            {
                                auto* subshape = new btConvexHullShape();
                                for (const auto& v : vertices)
                                    subshape->addPoint(btVector3(static_cast<btScalar>(v[0]), static_cast<btScalar>(v[1]), static_cast<btScalar>(v[2])));

                                shape = subshape;
                            }

                            if (shape == nullptr)
                                OMPL_ERROR("The mesh is empty!");

                            shape->setUserIndex(shape_index);
                            shape->setMargin(BULLET_MARGIN);
                            break;
                        }
#if OMPL_HAS_OCTOMAP
                        case geometries::GeometryType::OCTREE:
                        {
                            auto geom = geo->as<geometries::Octree>();
                            const octomap::OcTree& octree = *(geom->getOctree());
                            auto* subshape = new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, static_cast<int>(octree.size()));
                            double occupancy_threshold = octree.getOccupancyThres();

                            switch (geom->getSubType())
                            {
                                case geometries::Octree::SubType::BOX:
                                {
                                    for (auto it = octree.begin(static_cast<unsigned char>(octree.getTreeDepth())), end = octree.end(); it != end;
                                       ++it)
                                    {
                                        if (it->getOccupancy() >= occupancy_threshold)
                                        {
                                            double size = it.getSize();
                                            btTransform geomTrans;
                                            geomTrans.setIdentity();
                                            geomTrans.setOrigin(btVector3(
                                                static_cast<btScalar>(it.getX()), static_cast<btScalar>(it.getY()), static_cast<btScalar>(it.getZ())));
                                            auto l = static_cast<btScalar>(size / 2.0);
                                            auto* childshape = new btBoxShape(btVector3(l, l, l));
                                            childshape->setUserIndex(shape_index);
                                            childshape->setMargin(BULLET_MARGIN);
                                            cow->manage(childshape);

                                            subshape->addChildShape(geomTrans, childshape);
                                        }
                                    }
                                    shape = subshape;
                                }
                                case geometries::Octree::SubType::SPHERE_INSIDE:
                                {
                                    for (auto it = octree.begin(static_cast<unsigned char>(octree.getTreeDepth())), end = octree.end(); it != end;
                                       ++it)
                                    {
                                        if (it->getOccupancy() >= occupancy_threshold)
                                        {
                                            double size = it.getSize();
                                            btTransform geomTrans;
                                            geomTrans.setIdentity();
                                            geomTrans.setOrigin(btVector3(
                                            static_cast<btScalar>(it.getX()), static_cast<btScalar>(it.getY()), static_cast<btScalar>(it.getZ())));
                                            auto* childshape = new btSphereShape(static_cast<btScalar>((size / 2)));
                                            childshape->setUserIndex(shape_index);
                                            // Sphere is a special case where you do not modify the margin which is internally set to the radius
                                            cow->manage(childshape);

                                            subshape->addChildShape(geomTrans, childshape);
                                        }
                                    }
                                    shape = subshape;
                                }
                                case geometries::Octree::SubType::SPHERE_OUTSIDE:
                                {
                                    for (auto it = octree.begin(static_cast<unsigned char>(octree.getTreeDepth())), end = octree.end(); it != end;
                                       ++it)
                                    {
                                        if (it->getOccupancy() >= occupancy_threshold)
                                        {
                                            double size = it.getSize();
                                            btTransform geomTrans;
                                            geomTrans.setIdentity();
                                            geomTrans.setOrigin(btVector3(static_cast<btScalar>(it.getX()), static_cast<btScalar>(it.getY()), static_cast<btScalar>(it.getZ())));
                                            auto* childshape = new btSphereShape(static_cast<btScalar>(std::sqrt(2 * ((size / 2) * (size / 2)))));
                                            childshape->setUserIndex(shape_index);
                                            // Sphere is a special case where you do not modify the margin which is internally set to the radius
                                            cow->manage(childshape);

                                            subshape->addChildShape(geomTrans, childshape);
                                        }
                                    }
                                    shape = subshape;
                                }
                            }

                            if (shape == nullptr)
                                OMPL_ERROR("This bullet shape type (%d) is not supported for geometry octree",
                                                  static_cast<int>(geom->getSubType()));

                            shape->setUserIndex(shape_index);
                            shape->setMargin(BULLET_MARGIN);
                            break;
                        }
#endif
                        default:
                        {
                            OMPL_ERROR("This geometric shape type (%d) is not supported using BULLET yet",
                                                  static_cast<int>(geo->getType()));
                            break;
                        }
                    }

                    return shape;
                }

                CollisionObjectWrapper::CollisionObjectWrapper(std::string name,
                                                       const int& type_id,
                                                       const CollisionShapes shapes,
                                                       const geometries::VectorIsometry3d shape_poses)
                : m_name(std::move(name)), m_type_id(type_id), m_shapes(std::move(shapes)), m_shape_poses(std::move(shape_poses))
                {
                    assert(!m_name.empty());
                    assert(!m_shapes.empty());
                    assert(!m_shape_poses.empty());
                    assert(m_shapes.size() == m_shape_poses.size());

                    m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;
                    m_collisionFilterMask = btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter;
                    if (m_shapes.size() == 1 && m_shape_poses[0].matrix().isIdentity())
                    {
                        btCollisionShape* shape = createShapePrimitive(m_shapes[0], this, 0);
                        manage(shape);
                        setCollisionShape(shape);
                    }
                    else
                    {
                        auto* compound = new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, static_cast<int>(m_shapes.size()));
                        manage(compound);
                        compound->setMargin(BULLET_MARGIN);  // margin: compound. seems to have no
                                                         // effect when positive but has an
                                                         // effect when negative
                        setCollisionShape(compound);
                        std::size_t index = 0;
                        for (std::size_t j = 0; j < m_shapes.size(); ++j)
                        {
                            btCollisionShape* subshape = createShapePrimitive(m_shapes[j], this, static_cast<int>(j));
                            if (subshape != nullptr)
                            {
                                manage(subshape);
                                btTransform geomTrans = convertEigenToBt(m_shape_poses[j]);
                                compound->addChildShape(geomTrans, subshape);
                            }
                        }
                    }

                    btTransform trans;
                    trans.setIdentity();
                    setWorldTransform(trans);
                }

                CollisionObjectWrapper::CollisionObjectWrapper(std::string name,
                                                       const int& type_id,
                                                       const CollisionShapes shapes,
                                                       const geometries::VectorIsometry3d shape_poses,
                                                       std::vector<std::shared_ptr<void>> data)
                : m_name(std::move(name))
                , m_type_id(type_id)
                , m_shapes(std::move(shapes))
                , m_shape_poses(std::move(shape_poses))
                , m_data(std::move(data))
                {
                }
            }
        }
    }
}
