#ifndef OMPLAPP_GEOMETRY_DETAIL_BOX2D_STATE_VALIDITY_CHECKER_
#define OMPLAPP_GEOMETRY_DETAIL_BOX2D_STATE_VALIDITY_CHECKER_

#include <ompl/base/Contact.h>
#include <ompl/base/SafetyCertificate.h>
#include <ompl/base/StateValidityChecker.h>
#include "omplapp/collision/Box2dDiscreteBVHManager.h"
#include "omplapp/geometry/GeometrySpecification.h"

#include <boost/filesystem.hpp>

namespace ompl
{
    namespace app
    {
        class Box2dStateValidityChecker : public ompl::base::StateValidityChecker
        {
        public:

            using SafetyCertificateChecker = std::function<bool(const ompl::base::State *state, const ompl::base::SafetyCertificate *sc, std::vector<double> &dist)>;

            using CollisionCertificateChecker = std::function<bool(const ompl::base::State *state, const std::vector<ompl::base::SafetyCertificate *> &ocv)>;

            using DistanceCertificate = std::function<std::vector<double>(const base::State *a, const base::State *b)>;

            Box2dStateValidityChecker(const ompl::base::SpaceInformationPtr& si, const base::StateSpacePtr &gspace);

            Box2dStateValidityChecker(const ompl::base::SpaceInformationPtr& si, const base::StateSpacePtr &gspace, const GeometricStateExtractor &se);

            bool isValid(const ompl::base::State *state) const override;

            bool isValid(const ompl::base::State *state, double &dist) const override;

            bool isValid(const ompl::base::State *state, ompl::base::ContactResultVector &contact_vector, double &dist) const override;

            double clearance(const ompl::base::State *state) const override;

            void clearance(const ompl::base::State *state, ompl::base::ContactResultVector &contact_vector) const override;

            bool collisionCertificate(const ompl::base::State *state, const std::vector<base::SafetyCertificate *> &ocv) const;

            bool safetyCertificate(const ompl::base::State *state, const ompl::base::SafetyCertificate *sc, std::vector<double> &dist) const;

            std::vector<double> distanceCertificate(const base::State *a, const base::State *b) const;

            const std::vector<std::string> & getRobotNames() const;

            const collision::collision_box2d::Box2dDiscreteBVHManagerPtr getBox2dDiscreteBVHManager() const 
            {
                return collision_manager_;
            }

            CollisionCertificateChecker getCollisionCertificateChecker() const
            {
                return collisionCertificateChecker_;
            }

            SafetyCertificateChecker getSafetyCertificateChecker() const
            {
                return safetyCertificateChecker_;
            }

            DistanceCertificate getDistanceCertificate() const 
            {
                return distanceCertificate_;
            }

            std::vector<std::string> getActiveLinks() const 
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

        public:

            bool setEnvironmentFile(const std::string &env);

            bool addRobotShape(const collision::CollisionShapePtr& shape);

            bool addRobotShape(const std::string &robot);

            void removeRobotShape(const std::string & name);

            void clearRobotShapes();

        private:

            /** \brief return absolute path to mesh file if it exists and an empty path otherwise */
            boost::filesystem::path findMeshFile(const std::string& fname);

            base::StateSpacePtr gspace_;

            GeometricStateExtractor se_;

            std::vector<std::string> names_;
            
            int robotCount_{0};

            /** @brief The discrete contact manager used for creating cached discrete contact managers. */
            collision::collision_box2d::Box2dDiscreteBVHManagerPtr collision_manager_;

            SafetyCertificateChecker safetyCertificateChecker_ = std::bind(&Box2dStateValidityChecker::safetyCertificate, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

            CollisionCertificateChecker collisionCertificateChecker_ = std::bind(&Box2dStateValidityChecker::collisionCertificate, this, std::placeholders::_1, std::placeholders::_2);

            DistanceCertificate distanceCertificate_ = std::bind(&Box2dStateValidityChecker::distanceCertificate, this, std::placeholders::_1, std::placeholders::_2);

            Eigen::Isometry2d calculateCurrentTransform(const ompl::base::State *state, unsigned int index) const;

            /** \brief Paths to search for mesh files if mesh file names do not correspond to
             * absolute paths */
            std::vector<boost::filesystem::path> meshPath_{OMPLAPP_RESOURCE_DIR};
        };
    }
}

#endif
