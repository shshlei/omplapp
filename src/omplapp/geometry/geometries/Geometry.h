/**
 * @file geometry.h
 * @brief Tesseract Geometries
 *
 * @author Levi Armstrong
 * @date January 18, 2018
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

#ifndef OMPLAPP_GEOMETRY_GEOMETRIES_GEOMETRY_
#define OMPLAPP_GEOMETRY_GEOMETRIES_GEOMETRY_

#include "omplapp/config.h"
#include "ompl/util/ClassForward.h"
#include "omplapp/geometry/geometries/Types.h"
#include <octomap/octomap.h>
#include <boost/concept_check.hpp>
#include <memory>
#include <vector>

namespace ompl
{
    namespace app
    {
        namespace geometries
        {
            OMPL_CLASS_FORWARD(Geometry);

            enum GeometryType
            {
                CIRCLE,
                POLYGON,
                POLYGONSET,
                SPHERE,
                CYLINDER,
                CAPSULE,
                CONE,
                BOX,
                PLANE,
                MESH,
                CONVEX_MESH,
                SDF_MESH,
                OCTREE
            };

            class Geometry
            {
            public:
                Geometry(const Geometry&) = delete;
                Geometry& operator=(const Geometry&) = delete;
                Geometry(Geometry&&) = delete;
                Geometry& operator=(Geometry&&) = delete;

                explicit Geometry(GeometryType type) : type_(type) {}

                virtual ~Geometry() = default;
                
                /** \brief Cast this instance to a desired type. */
                template <class T>
                T *as()
                {
                    /** \brief Make sure the type we are casting to is indeed a planner */
                    BOOST_CONCEPT_ASSERT((boost::Convertible<T *, Geometry *>));

                    return static_cast<T *>(this);
                }

                /** \brief Cast this instance to a desired type. */
                template <class T>
                const T *as() const
                {
                    /** \brief Make sure the type we are casting to is indeed a Planner */
                    BOOST_CONCEPT_ASSERT((boost::Convertible<T *, Geometry *>));

                    return static_cast<const T *>(this);
                }

                /** \brief Create a copy of this shape */
                virtual GeometryPtr clone() const = 0;

                GeometryType getType() const { return type_; }

            private:
                /** \brief The type of the shape */
                GeometryType type_;
            };

            class Circle : public Geometry
            {
            public:
                explicit Circle(double r) : Geometry(GeometryType::CIRCLE), r_(r) {}

                ~Circle() override = default;

                double getRadius() const { return r_; }

                GeometryPtr clone() const override { return std::make_shared<Circle>(r_); }

            private:
                double r_;
            };

            class Polygon : public Geometry
            {
            public:
                Polygon() : Geometry(GeometryType::POLYGON)
                {
                    count_ = 0;
                }

                Polygon(const Polygon& polygon) : Geometry(GeometryType::POLYGON)
                {
                    count_ = polygon.getVerticeCount();
                    const Eigen::Vector2d* vecs = polygon.getVertices();
                    for (int i = 0; i < count_; i++)
                    {
                        vertices_[i] = vecs[i];
                    }
                }

                ~Polygon() override = default;

                void set(const Eigen::Vector2d* points, int count)
                {
                    assert(3 <= count && count <= 8);
                    count_ = count;
                    for (int i = 0; i < count; i++)
                    {
                        vertices_[i] = points[i];
                    }
                }

                /// Build vertices to represent an axis-aligned box centered on the local origin.
                /// @param hx the half-width.
                /// @param hy the half-height.
                void setAsBox(double hx, double hy)
                {
                    count_ = 4;
                    vertices_[0] << -hx, -hy;
                    vertices_[1] << hx, -hy;
                    vertices_[2] << hx,  hy;
                    vertices_[3] << -hx,  hy;
                }

                /// Build vertices to represent an oriented box.
                /// @param hx the half-width.
                /// @param hy the half-height.
                /// @param center the center of the box in local coordinates.
                /// @param angle the rotation of the box in local coordinates.
                void setAsBox(double hx, double hy, const Eigen::Vector2d& center, double angle)
                {
                    double s = std::sin(angle), c = std::cos(angle);
                    count_ = 4;
                    vertices_[0] << center[0]-c*hx + s*hy, center[1]-s*hx-c*hy;
                    vertices_[1] << center[0]+c*hx + s*hy, center[1]+s*hx-c*hy;
                    vertices_[2] << center[0]+c*hx - s*hy, center[1]+s*hx+c*hy;
                    vertices_[3] << center[0]-c*hx -s*hy, center[1]-s*hx+c*hy;
                }

                GeometryPtr clone() const override
                {
                    auto geom = std::make_shared<Polygon>();
                    geom->set(&vertices_[0], count_);
                    return geom;
                }

                int getVerticeCount() const 
                {
                    return count_;
                }

                const Eigen::Vector2d* getVertices() const
                {
                    return &vertices_[0];
                }

                bool samePolygon(const Polygon &other) const
                {
                    if (count_ != other.getVerticeCount())
                        return false;

                    const Eigen::Vector2d *vertices = other.getVertices();
                    for (int i = 0; i < count_; i++)
                    {
                        Eigen::Vector2d v1 = vertices_[i];
                        Eigen::Vector2d v2 = vertices[i];

                        if (std::abs(v1[0] - v2[0]) > std::numeric_limits<double>::epsilon() || std::abs(v1[1] - v2[1]) > std::numeric_limits<double>::epsilon())
                            return false;
                    }

                    return true;
                }

            private:
                Eigen::Vector2d vertices_[8];
                int count_;
            };

            class PolygonSet : public Geometry
            {
            public:
                PolygonSet(const std::vector<Polygon> polygons)
                : Geometry(GeometryType::POLYGONSET)
                , polygons_(polygons)
                {
                }

                ~PolygonSet() override = default;

                GeometryPtr clone() const override
                {
                    return std::make_shared<PolygonSet>(polygons_);
                }

                const std::vector<Polygon>& getPolygons() const 
                {
                    return polygons_;
                }

                int getPolygonCount() const 
                {
                    return static_cast<int>(polygons_.size());
                }

            private:

                const std::vector<Polygon> polygons_;
            };

            class Sphere : public Geometry
            {
            public:
                explicit Sphere(double r) : Geometry(GeometryType::SPHERE), r_(r) {}

                ~Sphere() override = default;

                double getRadius() const { return r_; }

                GeometryPtr clone() const override { return std::make_shared<Sphere>(r_); }

            private:
                double r_;
            };

            class Cylinder : public Geometry
            {
            public:
                Cylinder(double r, double l) : Geometry(GeometryType::CYLINDER), r_(r), l_(l) {}

                ~Cylinder() override = default;

                double getRadius() const { return r_; }

                double getLength() const { return l_; }

                GeometryPtr clone() const override { return std::make_shared<Cylinder>(r_, l_); }

            private:
                double r_, l_;
            };

            class Capsule : public Geometry
            {
            public:
                Capsule(double r, double l) : Geometry(GeometryType::CAPSULE), r_(r), l_(l) {}

                ~Capsule() override = default;

                double getRadius() const { return r_; }

                double getLength() const { return l_; }

                GeometryPtr clone() const override { return std::make_shared<Capsule>(r_, l_); }

            private:
                double r_, l_;
            };

            class Cone : public Geometry
            {
            public:
                Cone(double r, double l) : Geometry(GeometryType::CONE), r_(r), l_(l) {}

                ~Cone() override = default;

                double getRadius() const { return r_; }

                double getLength() const { return l_; }

                GeometryPtr clone() const override { return std::make_shared<Cone>(r_, l_); }

            private:
                double r_, l_;
            };

            class Box : public Geometry
            {
            public:
                Box(double x, double y, double z) : Geometry(GeometryType::BOX), x_(x), y_(y), z_(z) {}

                ~Box() override = default;

                double getX() const { return x_; }
                double getY() const { return y_; }
                double getZ() const { return z_; }

                GeometryPtr clone() const override { return std::make_shared<Box>(x_, y_, z_); }

            private:
                double x_, y_, z_;
            };

            class Plane : public Geometry
            {
            public:
                Plane(double a, double b, double c, double d) : Geometry(GeometryType::PLANE), a_(a), b_(b), c_(c), d_(d) {}

                ~Plane() override = default;
                
                double getA() const { return a_; }

                double getB() const { return b_; }

                double getC() const { return c_; }

                double getD() const { return d_; }

                GeometryPtr clone() const override { return std::make_shared<Plane>(a_, b_, c_, d_); }

            private:
                double a_, b_, c_, d_;
            };

            class Mesh : public Geometry
            {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                Mesh(std::shared_ptr<const VectorVector3d> vertices,
                     std::shared_ptr<const Eigen::VectorXi> triangles,
                     Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1))
                : Geometry(GeometryType::MESH)
                , vertices_(std::move(vertices))
                , triangles_(std::move(triangles))
                , scale_(std::move(scale))
                {
                    vertice_count_ = static_cast<int>(vertices_->size());

                    triangle_count_ = 0;
                    for (int i = 0; i < triangles_->size(); ++i)
                    {
                        ++triangle_count_;
                        int num_verts = (*triangles_)(i);
                        i += num_verts;
                        assert(num_verts == 3);
                    }
                }

                Mesh(std::shared_ptr<const VectorVector3d> vertices,
                     std::shared_ptr<const Eigen::VectorXi> triangles,
                     int triangle_count,
                     Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1))
                : Geometry(GeometryType::MESH)
                , vertices_(std::move(vertices))
                , triangles_(std::move(triangles))
                , triangle_count_(triangle_count)
                , scale_(std::move(scale))
                {
                    vertice_count_ = static_cast<int>(vertices_->size());
                    assert((triangle_count_ * 4) == triangles_->size());
                }

                ~Mesh() override = default;

                const std::shared_ptr<const VectorVector3d>& getVertices() const { return vertices_; }

                const std::shared_ptr<const Eigen::VectorXi>& getTriangles() const { return triangles_; }

                int getVerticeCount() const { return vertice_count_; }

                int getTriangleCount() const { return triangle_count_; }

                const Eigen::Vector3d& getScale() const { return scale_; }

                GeometryPtr clone() const override
                {
                    return std::make_shared<Mesh>(vertices_, triangles_, triangle_count_, scale_);
                }

            private:

                std::shared_ptr<const VectorVector3d> vertices_;

                std::shared_ptr<const Eigen::VectorXi> triangles_;

                int vertice_count_;

                int triangle_count_;

                Eigen::Vector3d scale_;
            };

            class ConvexMesh : public Geometry
            {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                ConvexMesh(std::shared_ptr<const VectorVector3d> vertices,
                           std::shared_ptr<const Eigen::VectorXi> faces,
                           Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1))
                : Geometry(GeometryType::CONVEX_MESH)
                , vertices_(std::move(vertices))
                , faces_(std::move(faces))
                , scale_(std::move(scale))
                {
                    vertice_count_ = static_cast<int>(vertices_->size());

                    face_count_ = 0;
                    for (int i = 0; i < faces_->size(); ++i)
                    {
                        ++face_count_;
                        int num_verts = (*faces_)(i);
                        i += num_verts;
                    }
                }

                ConvexMesh(std::shared_ptr<const VectorVector3d> vertices,
                           std::shared_ptr<const Eigen::VectorXi> faces,
                           int face_count,
                           Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1))
                : Geometry(GeometryType::CONVEX_MESH)
                , vertices_(std::move(vertices))
                , faces_(std::move(faces))
                , face_count_(face_count)
                , scale_(std::move(scale))
                {
                    vertice_count_ = static_cast<int>(vertices_->size());
                }

                ~ConvexMesh() override = default;

                const std::shared_ptr<const VectorVector3d>& getVertices() const { return vertices_; }

                const std::shared_ptr<const Eigen::VectorXi>& getFaces() const { return faces_; }

                int getVerticeCount() const { return vertice_count_; }

                int getFaceCount() const { return face_count_; }

                const Eigen::Vector3d& getScale() const { return scale_; }

                GeometryPtr clone() const override
                {
                    return std::make_shared<ConvexMesh>(vertices_, faces_, face_count_, scale_);
                }

            private:
                std::shared_ptr<const VectorVector3d> vertices_;

                std::shared_ptr<const Eigen::VectorXi> faces_;

                int vertice_count_;

                int face_count_;

                Eigen::Vector3d scale_;
            };

            class SDFMesh : public Geometry
            {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                SDFMesh(std::shared_ptr<const VectorVector3d> vertices,
                        std::shared_ptr<const Eigen::VectorXi> triangles,
                        Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1))
                : Geometry(GeometryType::SDF_MESH)
                , vertices_(std::move(vertices))
                , triangles_(std::move(triangles))
                , scale_(std::move(scale))
                {
                    vertice_count_ = static_cast<int>(vertices_->size());

                    triangle_count_ = 0;
                    for (int i = 0; i < triangles_->size(); ++i)
                    {
                        ++triangle_count_;
                        int num_verts = (*triangles_)(i);
                        i += num_verts;
                        assert(num_verts == 3);
                    }
                }

                SDFMesh(std::shared_ptr<const VectorVector3d> vertices,
                        std::shared_ptr<const Eigen::VectorXi> triangles,
                        int triangle_count,
                        Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1))
                : Geometry(GeometryType::SDF_MESH)
                , vertices_(std::move(vertices))
                , triangles_(std::move(triangles))
                , triangle_count_(triangle_count)
                , scale_(std::move(scale))
                {
                    vertice_count_ = static_cast<int>(vertices_->size());
                    assert((triangle_count_ * 4) == triangles_->size());
                }

                ~SDFMesh() override = default;

                const std::shared_ptr<const VectorVector3d>& getVertices() const { return vertices_; }

                const std::shared_ptr<const Eigen::VectorXi>& getTriangles() const { return triangles_; }

                int getVerticeCount() const { return vertice_count_; }

                int getTriangleCount() const { return triangle_count_; }

                const Eigen::Vector3d& getScale() const { return scale_; }

                GeometryPtr clone() const override
                {
                    return std::make_shared<SDFMesh>(vertices_, triangles_, triangle_count_, scale_);
                }

            private:
                std::shared_ptr<const VectorVector3d> vertices_;

                std::shared_ptr<const Eigen::VectorXi> triangles_;

                int vertice_count_;

                int triangle_count_;

                Eigen::Vector3d scale_;
            };

            class Octree : public Geometry
            {
            public:
                enum SubType
                {
                    BOX,
                    SPHERE_INSIDE,
                    SPHERE_OUTSIDE
                };

                Octree(std::shared_ptr<const octomap::OcTree> octree, const SubType sub_type)
                : Geometry(GeometryType::OCTREE), octree_(std::move(octree)), sub_type_(sub_type)
                {
                }

                template <typename PointT>
                Octree(const PointT& point_cloud, const double resolution, const SubType sub_type, const bool prune)
                : Geometry(GeometryType::OCTREE), sub_type_(sub_type)
                {
                    auto ot = std::make_shared<octomap::OcTree>(resolution);

                    for (auto& point : point_cloud.points)
                    ot->updateNode(point.x, point.y, point.z, true);

                    if (prune)
                        Octree::prune(*ot);

                    octree_ = ot;
                }

                ~Octree() override = default;

                const std::shared_ptr<const octomap::OcTree>& getOctree() const { return octree_; }

                SubType getSubType() const { return sub_type_; }

                GeometryPtr clone() const override { return std::make_shared<Octree>(octree_, sub_type_); }

                /**
                * @brief Octrees are typically generated from 3D sensor data so this method
                * should be used to efficiently update the collision shape.
                */
                void update() { assert(false); }

                /**
                * @brief Calculate the number of sub shapes that would get generated for this octree
                *
                * This is expensive and should not be called multiple times
                *
                * @return number of sub shapes
                */
                long calcNumSubShapes() const
                {
                    long cnt = 0;
                    double occupancy_threshold = octree_->getOccupancyThres();
                    for (auto it = octree_->begin(static_cast<unsigned char>(octree_->getTreeDepth())), end = octree_->end(); it != end;
                     ++it)
                    if (it->getOccupancy() >= occupancy_threshold)
                    ++cnt;

                    return cnt;
                }

            private:
                std::shared_ptr<const octomap::OcTree> octree_;
                SubType sub_type_;

                static bool isNodeCollapsible(octomap::OcTree& octree, octomap::OcTreeNode* node)
                {
                    if (!octree.nodeChildExists(node, 0))
                        return false;

                    double occupancy_threshold = octree.getOccupancyThres();

                    const octomap::OcTreeNode* firstChild = octree.getNodeChild(node, 0);
                    if (octree.nodeHasChildren(firstChild) || firstChild->getOccupancy() < occupancy_threshold)
                        return false;

                    for (unsigned int i = 1; i < 8; i++)
                    {
                        // comparison via getChild so that casts of derived classes ensure
                        // that the right == operator gets called
                        if (!octree.nodeChildExists(node, i))
                            return false;

                        if (octree.nodeHasChildren(octree.getNodeChild(node, i)))
                            return false;

                        if (octree.getNodeChild(node, i)->getOccupancy() < occupancy_threshold)
                            return false;
                    }

                    return true;
                }

                static bool pruneNode(octomap::OcTree& octree, octomap::OcTreeNode* node)
                {
                    if (!isNodeCollapsible(octree, node))
                        return false;

                    // set value to children's values (all assumed equal)
                    node->copyData(*(octree.getNodeChild(node, 0)));

                    // delete children (known to be leafs at this point!)
                    for (unsigned int i = 0; i < 8; i++)
                    {
                        octree.deleteNodeChild(node, i);
                    }

                    return true;
                }

                static void pruneRecurs(octomap::OcTree& octree,
                                  octomap::OcTreeNode* node,
                                  unsigned int depth,
                                  unsigned int max_depth,
                                  unsigned int& num_pruned)
                {
                    assert(node);

                    if (depth < max_depth)
                    {
                        for (unsigned int i = 0; i < 8; i++)
                        {
                            if (octree.nodeChildExists(node, i))
                            {
                                pruneRecurs(octree, octree.getNodeChild(node, i), depth + 1, max_depth, num_pruned);
                            }
                        }
                    }  // end if depth

                    else
                    {
                        // max level reached
                        if (pruneNode(octree, node))
                        {
                            num_pruned++;
                        }
                    }
                }

            public:
                /**
                * @brief A custom octree prune which will prune if all children are above the occupency threshold.
                *
                * This is different from the octomap::OcTree::prune which requires all children to have the same
                * occupency to be collapsed.
                *
                * @param octree The octree to be pruned.
                */
                static void prune(octomap::OcTree& octree)
                {
                    if (octree.getRoot() == nullptr)
                        return;

                    for (unsigned int depth = octree.getTreeDepth() - 1; depth > 0; --depth)
                    {
                        unsigned int num_pruned = 0;
                        pruneRecurs(octree, octree.getRoot(), 0, depth, num_pruned);
                        if (num_pruned == 0)
                            break;
                    }
                }
            };
        }
    }
}
#endif
