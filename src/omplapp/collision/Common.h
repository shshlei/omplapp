/**
 * @file common.h
 * @brief This is a collection of common methods
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

#ifndef OMPLAPP_COLLISION_COMMON_
#define OMPLAPP_COLLISION_COMMON_

#include "omplapp/config.h"
#include "omplapp/geometry/geometries/Geometry.h"
#include "omplapp/geometry/geometries/Types.h"
#include "omplapp/geometry/geometries/Utils.h"

#include "ompl/base/Contact.h"
#include <ompl/util/Console.h>

#include <functional>

#include <LinearMath/btConvexHullComputer.h>
#include <cstdio>
#include <cctype>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <boost/algorithm/string.hpp>

namespace ompl 
{
    namespace app 
    {
        namespace collision
        {
            using CollisionShapes = std::vector<geometries::GeometryPtr>;
            using CollisionShapePtr = geometries::GeometryPtr;

            using ObjectPairKey = std::pair<std::string, std::string>;

            /**
            * @brief Should return true if contact allowed, otherwise false.
            *
            * Also the order of strings should not matter, the function should handled by the function.
            */
            using IsContactAllowedFn = std::function<bool(const std::string&, const std::string&)>;

            enum class ContactTestType
            {
                FIRST = 0,   /**< Return at first contact for any pair of objects */
                CLOSEST = 1, /**< Return the global minimum for a pair of objects */
                ALL = 2,     /**< Return all contacts for a pair of objects */
                LIMITED = 3,  /**< Return limited set of contacts for a pair of objects */
                NEGATIVE = 4
            };

            /// Contact test data and query results information
            struct ContactTestData
            {
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                ContactTestData(const std::vector<std::string>& active,
                                double contact_distance,
                                double negative_distance,
                                const IsContactAllowedFn& fn,
                                const ContactTestType& type,
                                base::ContactResultMap& res,
                                unsigned int max_contacts = 5)
                : active(active), contact_distance(contact_distance), negative_distance(negative_distance), fn(fn), type(type), res(res),
                  max_contacts(max_contacts), done(false)
                {
                }

                const std::vector<std::string>& active;
                double contact_distance;
                double negative_distance;
                IsContactAllowedFn fn;
                ContactTestType type;

                base::ContactResultMap &res;
                
                unsigned int max_contacts;

                bool done;
            };

            inline base::ContactResult* processResult(ContactTestData& cdata,
                                    base::ContactResult& contact,
                                    const std::pair<std::string, std::string>& key,
                                    bool found)
            {
                if (!found)
                {
                    base::ContactResultVector data;
                    if (cdata.type == ContactTestType::FIRST)
                    {
                        data.emplace_back(contact);
                        cdata.done = true;
                    }
                    else
                    {
                        data.reserve(cdata.max_contacts);
                        data.emplace_back(contact);

                        if (cdata.type == ContactTestType::NEGATIVE && contact.distance < cdata.negative_distance)
                            cdata.done = true;
                    }

                    return &(cdata.res.insert(std::make_pair(key, data)).first->second.back());
                }

                assert(cdata.type != ContactTestType::FIRST);

                base::ContactResultVector& dr = cdata.res[key];

                if (cdata.type == ContactTestType::ALL)
                {
                    if (dr.size() == cdata.max_contacts)
                    {
                        cdata.max_contacts *= 2;
                        dr.reserve(cdata.max_contacts);
                    }

                    dr.emplace_back(contact);
                    return &(dr.back());
                }
                else if (cdata.type == ContactTestType::CLOSEST)
                {
                    if (contact.distance < dr[0].distance)
                    {
                        dr[0] = contact;
                        return &(dr[0]);
                    }
                }
                else if (cdata.type == ContactTestType::LIMITED)
                {
                    if (dr.size() < cdata.max_contacts)
                    {
                        dr.emplace_back(contact);
                        if (dr.size() == cdata.max_contacts)
                            cdata.done = true;
                        return &(dr.back());
                    }
                }
                else if (cdata.type == ContactTestType::NEGATIVE)
                {
                    if (contact.distance < dr[0].distance)
                    {
                        dr[0] = contact;
                        if (contact.distance < cdata.negative_distance)
                            cdata.done = true;
                        return &(dr[0]);
                    }
                }

                return nullptr;
            }

            /**
            * @brief Get a key for two object to search the collision matrix
            * @param obj1 First collision object name
            * @param obj2 Second collision object name
            * @return The collision pair key
            */
            inline ObjectPairKey getObjectPairKey(const std::string& obj1, const std::string& obj2)
            {
                return obj1 < obj2 ? std::make_pair(obj1, obj2) : std::make_pair(obj2, obj1);
            }

            /**
            * @brief This will check if a link is active provided a list. If the list is empty the link is considered active.
            * @param active List of active link names
            * @param name The name of link to check if it is active.
            */
            inline bool isLinkActive(const std::vector<std::string>& active, const std::string& name)
            {
                return active.empty() || (std::find(active.begin(), active.end(), name) != active.end());
            }

            /**
            * @brief Determine if contact is allowed between two objects.
            * @param name1 The name of the first object
            * @param name2 The name of the second object
            * @param acm The contact allowed function
            * @param verbose If true print debug informaton
            * @return True if contact is allowed between the two object, otherwise false.
            */
            inline bool isContactAllowed(const std::string& name1,
                             const std::string& name2,
                             const IsContactAllowedFn& acm,
                             bool verbose = false)
            {
                // do not distance check geoms part of the same object / link / attached body
                if (name1 == name2)
                    return true;

                if (acm != nullptr && acm(name1, name2))
                {
                    if (verbose)
                    {
                        OMPL_DEBUG("Collision between '%s' and '%s' is allowed. No contacts are computed.", name1.c_str(), name2.c_str());
                    }
                    return true;
                }

                if (verbose)
                {
                    OMPL_DEBUG("Actually checking collisions between %s and %s", name1.c_str(), name2.c_str());
                }

                return false;
            }

            /**
            * @brief Create a convex hull from vertices using Bullet Convex Hull Computer
            * @param (Output) vertices A vector of vertices
            * @param (Output) faces The first values indicates the number of vertices that define the face followed by the vertice
            * index
            * @param (input) input A vector of point to create a convex hull from
            * @param (input) shrink If positive, the convex hull is shrunken by that amount (each face is moved by "shrink" length
            *                units towards the center along its normal).
            * @param (input) shrinkClamp If positive, "shrink" is clamped to not exceed "shrinkClamp * innerRadius", where
            *                "innerRadius" is the minimum distance of a face to the center of the convex hull.
            * @return The number of faces. If less than zero an error occured when trying to create the convex hull
            */
            inline int createConvexHull(geometries::VectorVector3d& vertices,
                            Eigen::VectorXi& faces,
                            const geometries::VectorVector3d& input,
                            double shrink = -1,
                            double shrinkClamp = -1)
            {
                vertices.clear();

                btConvexHullComputer conv;
                std::vector<double> points;
                points.reserve(input.size() * 3);
                for (const auto& v : input)
                {
                    points.push_back(v[0]);
                    points.push_back(v[1]);
                    points.push_back(v[2]);
                }

                btScalar val = conv.compute(points.data(),
                                  3 * sizeof(double),
                                  static_cast<int>(input.size()),
                                  static_cast<btScalar>(shrink),
                                  static_cast<btScalar>(shrinkClamp));
                if (val < 0)
                {
                    OMPL_DEBUG("Failed to create convex hull");
                    return -1;
                }

                int num_verts = conv.vertices.size();
                vertices.reserve(static_cast<size_t>(num_verts));
                for (int i = 0; i < num_verts; i++)
                {
                    btVector3& v = conv.vertices[i];
                    vertices.push_back(
                    Eigen::Vector3d(static_cast<double>(v.getX()), static_cast<double>(v.getY()), static_cast<double>(v.getZ())));
                }

                auto num_faces = static_cast<size_t>(conv.faces.size());
                std::vector<int> local_faces;
                local_faces.reserve(3ul * num_faces);
                for (int i = 0; i < conv.faces.size(); i++)
                {
                    std::vector<int> face;
                    face.reserve(3);

                    const btConvexHullComputer::Edge* sourceEdge = &(conv.edges[conv.faces[i]]);
                    int a = sourceEdge->getSourceVertex();
                    face.push_back(a);

                    int b = sourceEdge->getTargetVertex();
                    face.push_back(b);

                    const btConvexHullComputer::Edge* edge = sourceEdge->getNextEdgeOfFace();
                    int c = edge->getTargetVertex();
                    face.push_back(c);

                    edge = edge->getNextEdgeOfFace();
                    c = edge->getTargetVertex();
                    while (c != a)
                    {
                        face.push_back(c);

                        edge = edge->getNextEdgeOfFace();
                        c = edge->getTargetVertex();
                    }
                    local_faces.push_back(static_cast<int>(face.size()));
                    local_faces.insert(local_faces.end(), face.begin(), face.end());
                }

                faces.resize(static_cast<long>(local_faces.size()));
                for (size_t i = 0; i < local_faces.size(); ++i)
                    faces[static_cast<long>(i)] = local_faces[i];

                return conv.faces.size();
            }

            inline std::shared_ptr<geometries::ConvexMesh> makeConvexMesh(const geometries::Mesh& mesh)
            {
                std::shared_ptr<geometries::VectorVector3d> ch_vertices = std::make_shared<geometries::VectorVector3d>();
                std::shared_ptr<Eigen::VectorXi> ch_faces = std::make_shared<Eigen::VectorXi>();
                int ch_num_faces = collision::createConvexHull(*ch_vertices, *ch_faces, *mesh.getVertices());
                return std::make_shared<geometries::ConvexMesh>(ch_vertices, ch_faces, ch_num_faces);
            }

            inline geometries::GeometryPtr makeConvexMesh(const geometries::GeometryPtr& mesh)
            {
                if (mesh->getType() == geometries::MESH)
                {
                    std::shared_ptr<geometries::VectorVector3d> ch_vertices = std::make_shared<geometries::VectorVector3d>();
                    std::shared_ptr<Eigen::VectorXi> ch_faces = std::make_shared<Eigen::VectorXi>();
                    int ch_num_faces = collision::createConvexHull(*ch_vertices, *ch_faces, *mesh->as<geometries::Mesh>()->getVertices());
                    return std::make_shared<geometries::ConvexMesh>(ch_vertices, ch_faces, ch_num_faces);
                }
                else if (mesh->getType() == geometries::CONVEX_MESH)
                {
                    return mesh->as<geometries::ConvexMesh>()->clone();
                }
                else 
                {
                    OMPL_ERROR("The input geometry is not mesh type");
                    std::shared_ptr<geometries::VectorVector3d> ch_vertices = std::make_shared<geometries::VectorVector3d>();
                    std::shared_ptr<Eigen::VectorXi> ch_faces = std::make_shared<Eigen::VectorXi>();
                    return std::make_shared<geometries::ConvexMesh>(ch_vertices, ch_faces);
                }
            }

            inline std::vector<geometries::GeometryPtr> makeConvexMeshes(const std::vector<geometries::GeometryPtr>& meshes)
            {
                std::vector<geometries::GeometryPtr> ch_meshes;

                for (auto & mesh : meshes)
                {
                    if (mesh->getType() == geometries::MESH)
                    {
                        std::shared_ptr<geometries::VectorVector3d> ch_vertices = std::make_shared<geometries::VectorVector3d>();
                        std::shared_ptr<Eigen::VectorXi> ch_faces = std::make_shared<Eigen::VectorXi>();
                        int ch_num_faces = collision::createConvexHull(*ch_vertices, *ch_faces, *mesh->as<geometries::Mesh>()->getVertices());
                        ch_meshes.push_back(std::make_shared<geometries::ConvexMesh>(ch_vertices, ch_faces, ch_num_faces));
                    }
                    else if (mesh->getType() == geometries::CONVEX_MESH)
                    {
                        ch_meshes.push_back(mesh->as<geometries::ConvexMesh>()->clone());
                    }
                    else
                    {
                        OMPL_ERROR("The input geometry is not mesh type");
                        return std::vector<geometries::GeometryPtr>();
                    }
                }

                return ch_meshes;
            }

            /**
            * @brief Write a simple ply file given vertices and faces
            * @param path The file path
            * @param vertices A vector of vertices
            * @param vertices_color The vertices color (0-255,0-255,0-255), if empty uses a default color
            * @param faces The first values indicates the number of vertices that define the face followed by the vertice index
            * @param num_faces The number of faces
            * @return False if failed to write file, otherwise true
            */
            inline bool writeSimplePlyFile(const std::string& path,
                               const geometries::VectorVector3d& vertices,
                               const std::vector<Eigen::Vector3i>& vectices_color,
                               const Eigen::VectorXi& faces,
                               int num_faces)
            {
                //  ply
                //  format ascii 1.0           { ascii/binary, format version number }
                //  comment made by Greg Turk  { comments keyword specified, like all lines }
                //  comment this file is a cube
                //  element vertex 8           { define "vertex" element, 8 of them in file }
                //  property float x           { vertex contains float "x" coordinate }
                //  property float y           { y coordinate is also a vertex property }
                //  property float z           { z coordinate, too }
                //  property uchar red         { start of vertex color }
                //  property uchar green
                //  property uchar blue
                //  element face 6             { there are 6 "face" elements in the file }
                //  property list uchar int vertex_index { "vertex_indices" is a list of ints }
                //  end_header                 { delimits the end of the header }
                //  0 0 0                      { start of vertex list }
                //  0 0 1
                //  0 1 1
                //  0 1 0
                //  1 0 0
                //  1 0 1
                //  1 1 1
                //  1 1 0
                //  4 0 1 2 3                  { start of face list }
                //  4 7 6 5 4
                //  4 0 4 5 1
                //  4 1 5 6 2
                //  4 2 6 7 3
                //  4 3 7 4 0
                std::ofstream myfile;
                myfile.open(path);
                if (myfile.fail())
                {
                    OMPL_ERROR("Failed to open file: %s", path.c_str());
                    return false;
                }

                myfile << "ply\n";
                myfile << "format ascii 1.0\n";
                myfile << "comment made by tesseract\n";
                myfile << "element vertex " << vertices.size() << "\n";
                myfile << "property float x\n";
                myfile << "property float y\n";
                myfile << "property float z\n";
                if (!vectices_color.empty())
                {
                    myfile << "property uchar red\n";
                    myfile << "property uchar green\n";
                    myfile << "property uchar blue\n";
                }
                myfile << "element face " << num_faces << "\n";
                myfile << "property list uchar int vertex_indices\n";
                myfile << "end_header\n";

                // Add vertices
                if (vectices_color.empty())
                {
                    for (const auto& v : vertices)
                    {
                        myfile << std::fixed << std::setprecision(std::numeric_limits<float>::digits10 + 1) << v[0] << " " << v[1] << " "
                            << v[2] << "\n";
                    }
                }
                else if (vectices_color.size() == 1)
                {
                    const Eigen::Vector3i& default_color = vectices_color[0];
                    for (const auto& v : vertices)
                    {
                        myfile << std::fixed << std::setprecision(std::numeric_limits<float>::digits10 + 1) << v[0] << " " << v[1] << " "
                            << v[2] << " " << default_color[0] << " " << default_color[1] << " " << default_color[2] << "\n";
                    }
                }
                else
                {
                    for (std::size_t i = 0; i < vertices.size(); ++i)
                    {
                        const Eigen::Vector3d& v = vertices[i];
                        const Eigen::Vector3i& v_color = vectices_color[i];
                        myfile << std::fixed << std::setprecision(std::numeric_limits<float>::digits10 + 1) << v[0] << " " << v[1] << " "
                            << v[2] << " " << v_color[0] << " " << v_color[1] << " " << v_color[2] << "\n";
                    }
                }

                // Add faces
                long idx = 0;
                for (long i = 0; i < num_faces; ++i)
                {
                    long num_vert = faces[idx];
                    for (long j = 0; j < num_vert; ++j)
                    {
                        myfile << faces[idx] << " ";
                        ++idx;
                    }
                    myfile << faces[idx] << "\n";
                    ++idx;
                }

                myfile.close();
                return true;
            }

            /**
            * @brief Write a simple ply file given vertices and faces
            * @param path The file path
            * @param vertices A vector of vertices
            * @param faces The first values indicates the number of vertices that define the face followed by the vertice index
            * @param num_faces The number of faces
            * @return False if failed to write file, otherwise true
            */
            inline bool writeSimplePlyFile(const std::string& path,
                               const geometries::VectorVector3d& vertices,
                               const Eigen::VectorXi& faces,
                               int num_faces)
            {
                std::vector<Eigen::Vector3i> vertices_color;
                return writeSimplePlyFile(path, vertices, vertices_color, faces, num_faces);
            }

            /**
            * @brief Loads a simple ply file given a path
            * @param path The file path
            * @param vertices A vector of vertices
            * @param faces The first values indicates the number of vertices that define the face followed by the vertice index
            * @param triangles_only Convert to only include triangles
            * @return Number of faces, If returned 0 it failed to load.
            */
            inline int loadSimplePlyFile(const std::string& path,
                             geometries::VectorVector3d& vertices,
                             Eigen::VectorXi& faces,
                             bool triangles_only = false)
            {
                //  ply
                //  format ascii 1.0           { ascii/binary, format version number }
                //  comment made by Greg Turk  { comments keyword specified, like all lines }
                //  comment this file is a cube
                //  element vertex 8           { define "vertex" element, 8 of them in file }
                //  property float x           { vertex contains float "x" coordinate }
                //  property float y           { y coordinate is also a vertex property }
                //  property float z           { z coordinate, too }
                //  element face 6             { there are 6 "face" elements in the file }
                //  property list uchar int vertex_index { "vertex_indices" is a list of ints }
                //  end_header                 { delimits the end of the header }
                //  0 0 0                      { start of vertex list }
                //  0 0 1
                //  0 1 1
                //  0 1 0
                //  1 0 0
                //  1 0 1
                //  1 1 1
                //  1 1 0
                //  4 0 1 2 3                  { start of face list }
                //  4 7 6 5 4
                //  4 0 4 5 1
                //  4 1 5 6 2
                //  4 2 6 7 3
                //  4 3 7 4 0

                vertices.clear();

                std::ifstream myfile;
                myfile.open(path);
                if (myfile.fail())
                {
                    OMPL_ERROR("Failed to open file: %s", path.c_str());
                    return false;
                }
                std::string str;
                std::getline(myfile, str);
                std::getline(myfile, str);
                std::getline(myfile, str);
                std::getline(myfile, str);
                std::vector<std::string> tokens;
                boost::split(tokens, str, boost::is_any_of(" "));
                if (tokens.size() != 3 || !geometries::isNumeric(tokens.back()))
                {
                    OMPL_ERROR("Failed to parse file: %s", path.c_str());
                    return false;
                }
                auto num_vertices = static_cast<size_t>(std::stoi(tokens.back()));

                std::getline(myfile, str);
                std::getline(myfile, str);
                std::getline(myfile, str);
                std::getline(myfile, str);
                std::getline(myfile, str);
                std::getline(myfile, str);
                std::getline(myfile, str);

                tokens.clear();
                boost::split(tokens, str, boost::is_any_of(" "));
                if (tokens.size() != 3 || !geometries::isNumeric(tokens.back()))
                {
                    OMPL_ERROR("Failed to parse file: %s", path.c_str());
                    return false;
                }

                auto num_faces = static_cast<size_t>(std::stoi(tokens.back()));
                std::getline(myfile, str);
                std::getline(myfile, str);
                if (str != "end_header")
                {
                    OMPL_ERROR("Failed to parse file: %s", path.c_str());
                    return false;
                }

                vertices.reserve(num_vertices);
                for (size_t i = 0; i < num_vertices; ++i)
                {
                    std::getline(myfile, str);
                    tokens.clear();
                    boost::split(tokens, str, boost::is_any_of(" "));
                    if (tokens.size() != 3)
                    {
                        OMPL_ERROR("Failed to parse file: %s", path.c_str());
                        return false;
                    }

                    vertices.push_back(Eigen::Vector3d(std::stod(tokens[0]), std::stod(tokens[1]), std::stod(tokens[2])));
                }

                std::vector<int> local_faces;
                local_faces.reserve(num_faces * 3);
                size_t copy_num_faces = num_faces;  // Becuase num_faces can change within for loop
                for (size_t i = 0; i < copy_num_faces; ++i)
                {
                    std::getline(myfile, str);
                    tokens.clear();
                    boost::split(tokens, str, boost::is_any_of(" "));
                    if (tokens.size() < 3)
                    {
                        OMPL_ERROR("Failed to parse file: %s", path.c_str());
                        return false;
                    }

                    auto num_verts = static_cast<int>(tokens.size());
                    assert(num_verts >= 3);
                    if (triangles_only && num_verts > 3)
                    {
                        local_faces.push_back(3);
                        local_faces.push_back(std::stoi(tokens[0]));
                        local_faces.push_back(std::stoi(tokens[1]));
                        local_faces.push_back(std::stoi(tokens[2]));
                        for (size_t i = 3; i < tokens.size(); ++i)
                        {
                            num_faces += 1;
                            local_faces.push_back(3);
                            local_faces.push_back(std::stoi(tokens[0]));
                            local_faces.push_back(std::stoi(tokens[i - 1]));
                            local_faces.push_back(std::stoi(tokens[i]));
                        }
                    }
                    else
                    {
                        local_faces.push_back(static_cast<int>(tokens.size()));
                        for (const auto& t : tokens)
                        local_faces.push_back(std::stoi(t));
                    }
                }

                faces.resize(static_cast<long>(local_faces.size()));
                for (size_t i = 0; i < local_faces.size(); ++i)
                    faces[static_cast<long>(i)] = local_faces[i];

                myfile.close();
                return static_cast<int>(num_faces);
            }

        }
    }
}

#endif
