/**
 * @file types.h
 * @brief Common Tesseract Types
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

#ifndef OMPLAPP_GEOMETRY_GEOMETRIES_TYPES_
#define OMPLAPP_GEOMETRY_GEOMETRIES_TYPES_

#include "ompl/base/Contact.h"

namespace ompl
{
    namespace app
    {
        namespace geometries 
        {
            using namespace ompl::base;

            using VectorIsometry3d = AlignedVector<Eigen::Isometry3d>;
            using VectorVector4d = AlignedVector<Eigen::Vector4d>;
            using VectorVector3d = AlignedVector<Eigen::Vector3d>;
            using TransformMap = AlignedMap<std::string, Eigen::Isometry3d>;
            typedef std::vector<Eigen::Isometry2f, Eigen::aligned_allocator<Eigen::Isometry2f> > vector_Isometry2f;
            typedef std::vector<Eigen::Isometry2d, Eigen::aligned_allocator<Eigen::Isometry2d> > vector_Isometry2d;
            using VectorIsometry2d = AlignedVector<Eigen::Isometry2d>;
        }
    }
}
#endif
