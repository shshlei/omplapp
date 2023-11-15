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

#ifndef SIMPLECERTIFICATE_STATE_VALIDITY_CHECKER_
#define SIMPLECERTIFICATE_STATE_VALIDITY_CHECKER_

#include <ompl/base/StateValidityChecker.h>
#include <omplapp/collision/Box2dDiscreteBVHManager.h>

#include <boost/filesystem.hpp>

namespace ompl
{
    namespace app
    {
        class SimpleCertificateStateValidityChecker : public ompl::base::StateValidityChecker
        {
        public:

            SimpleCertificateStateValidityChecker(const ompl::base::SpaceInformationPtr& si, double contact_distance);

            bool setEnvironmentFile(const std::string &env);
            bool setRobotShape(const collision::CollisionShapePtr& shape);

            bool isValid(const ompl::base::State *state) const override;

            bool isValid(const ompl::base::State *state, double &dist) const override;

            double clearance(const ompl::base::State *state) const override;

            void setContactDistanceThreshold(double cs)
            {
                contact_distance_ = cs;
                //collision_manager_->setContactDistanceThreshold(0.5 * cs);
            }

            double getContactDistanceThreshold() const 
            {
                return contact_distance_;
            }

            const std::string& getActiveLinks() const 
            {
                return names_;
            }

            /** \brief set path to search for mesh files */
            void setMeshPath(const std::vector<boost::filesystem::path>& path)
            {
                for (const auto &p : path)
                    if (!boost::filesystem::is_directory(p))
                        OMPL_WARN("Mesh path '%s' is not an existing directory", p.c_str());
                meshPath_ = path;
            }

        private:

            /** \brief return absolute path to mesh file if it exists and an empty path otherwise */
            boost::filesystem::path findMeshFile(const std::string& fname);

            bool read(const std::string& filename);

            bool read(std::istream &s);

            bool readHeader(std::istream& s, int& numbers);

            std::string names_;
            
            int robotCount_{0};

            double contact_distance_{0.0};

            /** @brief The discrete contact manager used for creating cached discrete contact managers. */
            collision::collision_box2d::Box2dDiscreteBVHManagerPtr collision_manager_;

            Eigen::Isometry2d calculateCurrentTransform(const ompl::base::State *state) const;

            /** \brief Paths to search for mesh files if mesh file names do not correspond to
             * absolute paths */
            std::vector<boost::filesystem::path> meshPath_{OMPLAPP_RESOURCE_DIR};
        };
    }
}

#endif
