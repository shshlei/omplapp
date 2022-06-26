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

/* Author: Ioan Sucan, Shi Shenglei */

#ifndef OMPLAPP_GEOMETRY_GEOMETRIES_MESH_PARSER_
#define OMPLAPP_GEOMETRY_GEOMETRIES_MESH_PARSER_

#include "omplapp/config.h"
#include <ompl/util/Console.h>
#include "omplapp/geometry/geometries/Types.h"

#include <memory>
#include <vector>

#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

namespace ompl
{
    namespace app
    {
        namespace geometries
        {
            template <class T>
            inline std::vector<GeometryPtr> extractMeshData(const aiScene* scene,
                                                            const aiNode* node,
                                                            const aiMatrix4x4& parent_transform,
                                                            const Eigen::Vector3d &scale,
                                                            const Eigen::Vector3d &center = Eigen::Vector3d::Zero())
            {
                std::vector<GeometryPtr> meshes;

                aiMatrix4x4 transform = parent_transform;
                transform *= node->mTransformation;

                for (unsigned int j = 0; j < node->mNumMeshes; ++j)
                {
                    auto vertices = std::make_shared<VectorVector3d>();
                    auto triangles = std::make_shared<Eigen::VectorXi>();

                    const aiMesh* a = scene->mMeshes[node->mMeshes[j]];

                    long triangle_count = 0;
                    std::vector<int> local_triangles;
                    local_triangles.reserve(4*a->mNumFaces);
                    for (unsigned int i = 0; i < a->mNumFaces; ++i)
                    {
                        if (a->mFaces[i].mNumIndices >= 3)
                        {
                            triangle_count++;
                            local_triangles.push_back(static_cast<int>(a->mFaces[i].mNumIndices));
                            for (size_t k = 0; k < a->mFaces[i].mNumIndices; ++k)
                                local_triangles.push_back(static_cast<int>(a->mFaces[i].mIndices[k]));
                        }
//                        else
//                        {
//                            OMPL_WARN("Mesh had a face with less than three verticies");
//                        }
                    }

                    if (triangle_count == 0)
                        continue;

                    for (unsigned int i = 0; i < a->mNumVertices; ++i)
                    {
                        aiVector3D v = transform * a->mVertices[i];
                        Eigen::Vector3d vert = Eigen::Vector3d::Zero();
                        if (center[0] != 0.0 || center[1] != 0.0 || center[2] != 0.0)
                            vert = Eigen::Vector3d((static_cast<double>(v.x) - center[0]) * scale(0),
                                                   (static_cast<double>(v.y) - center[1]) * scale(1),
                                                   (static_cast<double>(v.z) - center[2]) * scale(2));
                        else 
                            vert = Eigen::Vector3d(static_cast<double>(v.x) * scale(0),
                                                   static_cast<double>(v.y) * scale(1),
                                                   static_cast<double>(v.z) * scale(2));
                        vertices->push_back(vert);
                    }

                    triangles->resize(static_cast<long>(local_triangles.size()));
                    for (long i = 0; i < triangles->size(); ++i)
                        (*triangles)[i] = local_triangles[static_cast<size_t>(i)];

                    meshes.push_back(std::make_shared<T>(vertices, triangles, static_cast<int>(triangle_count), scale));
                }

                for (unsigned int n = 0; n < node->mNumChildren; ++n)
                {
                    std::vector<GeometryPtr> child_meshes = extractMeshData<T>(scene, node->mChildren[n], transform, scale, center);
                    meshes.insert(meshes.end(), child_meshes.begin(), child_meshes.end());
                }

                return meshes;
            }

            template <class T>
            inline std::vector<GeometryPtr> createMeshFromAsset(const aiScene* scene,
                                                                const Eigen::Vector3d &scale, const Eigen::Vector3d &center = Eigen::Vector3d::Zero())
            {
                if (!scene->HasMeshes())
                {
                    OMPL_WARN("Assimp reports scene has no meshes");
                    return std::vector<GeometryPtr>();
                }

                std::vector<GeometryPtr> meshes = extractMeshData<T>(scene, scene->mRootNode, aiMatrix4x4(), scale, center);

                if (meshes.empty())
                {
                    OMPL_WARN("There are no meshes in the scene");
                    return std::vector<GeometryPtr>();
                }

                return meshes;
            }

            template <class T>
            inline std::vector<GeometryPtr> createMeshFromPath(const std::string& path,
                                                               const Eigen::Vector3d &scale = Eigen::Vector3d(1, 1, 1),
                                                               const Eigen::Vector3d &center = Eigen::Vector3d::Zero(),
                                                               bool triangulate = false,
                                                               bool flatten = false)
            {
                Assimp::Importer importer;

                // Issue #38 fix: as part of the post-processing, we remove all other components in file but
                // the meshes, as anyway the resulting shapes:Mesh object just receives vertices and triangles.
                importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
                          aiComponent_NORMALS | aiComponent_TANGENTS_AND_BITANGENTS | aiComponent_COLORS |
                          aiComponent_TEXCOORDS | aiComponent_BONEWEIGHTS | aiComponent_ANIMATIONS |
                          aiComponent_TEXTURES | aiComponent_LIGHTS | aiComponent_CAMERAS |
                          aiComponent_MATERIALS);

                const aiScene* scene = nullptr;
                if (triangulate)
                    scene = importer.ReadFile(path.c_str(), aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType | aiProcess_RemoveComponent);
                else
                    scene = importer.ReadFile(path.c_str(), aiProcess_JoinIdenticalVertices | aiProcess_SortByPType | aiProcess_RemoveComponent);

                if (!scene)
                    return std::vector<GeometryPtr>();

                // Assimp enforces Y_UP convention by rotating models with different conventions.
                // However, that behaviour is confusing and doesn't match the ROS convention
                // where the Z axis is pointing up.
                // Hopefully this doesn't undo legit use of the root node transformation...
                // Note that this is also what RViz does internally.
                scene->mRootNode->mTransformation = aiMatrix4x4();

                if (flatten)
                {
                    // These post processing steps flatten the root node transformation into child nodes,
                    // so they must be delayed until after clearing the root node transform above.
                    importer.ApplyPostProcessing(aiProcess_OptimizeMeshes | aiProcess_OptimizeGraph);
                }
                else
                {
                    importer.ApplyPostProcessing(aiProcess_OptimizeGraph);
                }

                return createMeshFromAsset<T>(scene, scale, center);
            }
        }
    }
}  // namespace tesseract_geometry
#endif
