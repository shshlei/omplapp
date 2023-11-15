/**
 * @file utils.h
 * @brief Tesseract Geometry Utility Function
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
#ifndef OMPLAPP_GEOMETRY_GEOMETRIES_UTILS_
#define OMPLAPP_GEOMETRY_GEOMETRIES_UTILS_

#include "omplapp/geometry/geometries/Geometry.h"
#include <ompl/util/Console.h>

#include <string>
#include <sstream>
#include <stdexcept>

namespace ompl
{
    namespace app
    {
        namespace geometries
        {
            /**
             * @brief Check if two Geometries are identical
             * @return True if identical, otherwise false
             */
            inline bool isIdentical(const Geometry& geom1, const Geometry& geom2)
            {
                if (geom1.getType() != geom2.getType())
                    return false;

                switch (geom1.getType())
                {
                    case GeometryType::CIRCLE:
                    {
                        const Circle& s1 = static_cast<const Circle&>(geom1);
                        const Circle& s2 = static_cast<const Circle&>(geom2);

                        if (std::abs(s1.getRadius() - s2.getRadius()) > std::numeric_limits<double>::epsilon())
                            return false;

                        break;
                    }
                    case GeometryType::ELLIPSE:
                    {
                        const Ellipse& s1 = static_cast<const Ellipse&>(geom1);
                        const Ellipse& s2 = static_cast<const Ellipse&>(geom2);

                        if (std::abs(s1.getA() - s2.getA()) > std::numeric_limits<double>::epsilon())
                            return false;
                        if (std::abs(s1.getB() - s2.getB()) > std::numeric_limits<double>::epsilon())
                            return false;

                        break;
                    }
                    case GeometryType::RECTANGLE:
                    {
                        const Rectangle& s1 = static_cast<const Rectangle&>(geom1);
                        const Rectangle& s2 = static_cast<const Rectangle&>(geom2);

                        if (std::abs(s1.getA() - s2.getA()) > std::numeric_limits<double>::epsilon())
                            return false;
                        if (std::abs(s1.getB() - s2.getB()) > std::numeric_limits<double>::epsilon())
                            return false;

                        break;
                    }
                    case GeometryType::CAPSULE:
                    {
                        const Capsule& s1 = static_cast<const Capsule&>(geom1);
                        const Capsule& s2 = static_cast<const Capsule&>(geom2);

                        if (std::abs(s1.getRadius() - s2.getRadius()) > std::numeric_limits<double>::epsilon())
                            return false;
                        if (std::abs(s1.getLength() - s2.getLength()) > std::numeric_limits<double>::epsilon())
                            return false;

                        break;
                    }
                    case GeometryType::POLYGON:
                    {
                        const Polygon& s1 = static_cast<const Polygon&>(geom1);
                        const Polygon& s2 = static_cast<const Polygon&>(geom2);

                        if (!s1.samePolygon(s2))
                            return false;

                        break;
                    }
                    case GeometryType::BOX:
                    {
                        const Box& s1 = static_cast<const Box&>(geom1);
                        const Box& s2 = static_cast<const Box&>(geom2);

                        if (std::abs(s1.getX() - s2.getX()) > std::numeric_limits<double>::epsilon())
                            return false;

                        if (std::abs(s1.getY() - s2.getY()) > std::numeric_limits<double>::epsilon())
                            return false;

                        if (std::abs(s1.getZ() - s2.getZ()) > std::numeric_limits<double>::epsilon())
                            return false;

                        break;
                    }
                    case GeometryType::SPHERE:
                    {
                        const Sphere& s1 = static_cast<const Sphere&>(geom1);
                        const Sphere& s2 = static_cast<const Sphere&>(geom2);

                        if (std::abs(s1.getRadius() - s2.getRadius()) > std::numeric_limits<double>::epsilon())
                            return false;

                        break;
                    }
                    case GeometryType::CYLINDER:
                    {
                        const Cylinder& s1 = static_cast<const Cylinder&>(geom1);
                        const Cylinder& s2 = static_cast<const Cylinder&>(geom2);

                        if (std::abs(s1.getRadius() - s2.getRadius()) > std::numeric_limits<double>::epsilon())
                            return false;

                        if (std::abs(s1.getLength() - s2.getLength()) > std::numeric_limits<double>::epsilon())
                            return false;

                        break;
                    }
                    case GeometryType::CONE:
                    {
                        const Cone& s1 = static_cast<const Cone&>(geom1);
                        const Cone& s2 = static_cast<const Cone&>(geom2);

                        if (std::abs(s1.getRadius() - s2.getRadius()) > std::numeric_limits<double>::epsilon())
                            return false;

                        if (std::abs(s1.getLength() - s2.getLength()) > std::numeric_limits<double>::epsilon())
                            return false;

                        break;
                    }
                    case GeometryType::MESH:
                    {
                        const Mesh& s1 = static_cast<const Mesh&>(geom1);
                        const Mesh& s2 = static_cast<const Mesh&>(geom2);

                        if (s1.getVerticeCount() != s2.getVerticeCount())
                            return false;

                        if (s1.getTriangleCount() != s2.getTriangleCount())
                            return false;

                        break;
                    }
                    case GeometryType::CONVEX_MESH:
                    {
                        const ConvexMesh& s1 = static_cast<const ConvexMesh&>(geom1);
                        const ConvexMesh& s2 = static_cast<const ConvexMesh&>(geom2);

                        if (s1.getVerticeCount() != s2.getVerticeCount())
                            return false;

                        if (s1.getFaceCount() != s2.getFaceCount())
                            return false;

                        break;
                    }
                    case GeometryType::SDF_MESH:
                    {
                        const SDFMesh& s1 = static_cast<const SDFMesh&>(geom1);
                        const SDFMesh& s2 = static_cast<const SDFMesh&>(geom2);

                        if (s1.getVerticeCount() != s2.getVerticeCount())
                            return false;

                        if (s1.getTriangleCount() != s2.getTriangleCount())
                            return false;

                        break;
                    }
#if OMPL_HAS_OCTOMAP
                    case GeometryType::OCTREE:
                    {
                        const Octree& s1 = static_cast<const Octree&>(geom1);
                        const Octree& s2 = static_cast<const Octree&>(geom2);

                        const std::shared_ptr<const octomap::OcTree>& octree1 = s1.getOctree();
                        const std::shared_ptr<const octomap::OcTree>& octree2 = s2.getOctree();

                        if (octree1->getTreeType() != octree2->getTreeType())
                            return false;

                        if (octree1->size() != octree2->size())
                            return false;

                        if (octree1->getTreeDepth() != octree2->getTreeDepth())
                            return false;

                        if (octree1->memoryUsage() != octree2->memoryUsage())
                            return false;

                        if (octree1->memoryFullGrid() != octree2->memoryFullGrid())
                            return false;

                        break;
                    }
#endif
                    default:
                    {
                        OMPL_ERROR("This geometric shape type (%d) is not supported", static_cast<int>(geom1.getType()));
                        return false;
                    }
                }

                return true;
            }
        
            inline bool isNumeric(const std::string& s)
            {
                if (s.empty())
                    return false;

                std::stringstream ss;
                ss.imbue(std::locale::classic());

                ss << s;

                double out;
                ss >> out;

                return !(ss.fail() || !ss.eof());
            }

            inline bool isNumeric(const std::vector<std::string>& sv)
            {
                for (const auto& s : sv)
                {
                    if (!isNumeric(s))
                        return false;
                }

                return true;
            }

            /**
            * @brief Convert a string to a numeric value type
            * @return True if successful, otherwise false
            */
            template <typename FloatType>
            inline bool toNumeric(const std::string& s, FloatType& value)
            {
                if (s.empty())
                    return false;

                std::stringstream ss;
                ss.imbue(std::locale::classic());

                ss << s;

                FloatType out;
                ss >> out;

                if (ss.fail() || !ss.eof())
                    return false;

                value = out;
                return true;
            }
        }
    }
}

#endif
