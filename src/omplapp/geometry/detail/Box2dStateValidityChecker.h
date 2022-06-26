#ifndef OMPLAPP_GEOMETRY_DETAIL_BOX2D_STATE_VALIDITY_CHECKER_
#define OMPLAPP_GEOMETRY_DETAIL_BOX2D_STATE_VALIDITY_CHECKER_

#include <ompl/base/Contact.h>
#include <ompl/base/SafetyCertificate.h>
#include <ompl/base/StateValidityChecker.h>
#include "omplapp/collision/Box2dDiscreteBVHManager.h"
#include "omplapp/geometry/GeometrySpecification.h"

#include <boost/filesystem.hpp>
#include <mutex>

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

            Box2dStateValidityChecker(const ompl::base::SpaceInformationPtr& si, MotionModel mtype,  
                                        double collision_safety_margin, double negative_distance, const base::StateSpacePtr &gspace,
                                        const GeometricStateExtractor &se);

            bool setEnvironmentFile(const std::string &env);
//            void setPointRobotNumber(int num);
            bool addRobotShape(const collision::CollisionShapePtr& shape);

            bool isValid(const ompl::base::State *state) const override;

            bool isValid(const ompl::base::State *state, double &dist) const override;

            bool isValid(const ompl::base::State *state, ompl::base::ContactResultVector &contact_vector, double &dist) const override;

            double clearance(const ompl::base::State *state) const override;

            void clearance(const ompl::base::State *state, ompl::base::ContactResultVector &contact_vector) const override;

            bool collisionCertificate(const ompl::base::State *state, const std::vector<base::SafetyCertificate *> &ocv) const;

            bool safetyCertificate(const ompl::base::State *state, const ompl::base::SafetyCertificate *sc, std::vector<double> &dist) const;

            std::vector<double> distanceCertificate(const base::State *a, const base::State *b) const;

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

            void setCollisionSafetyMargin(double cs)
            {
                collision_safety_margin_ = cs;
                collision_manager_->setContactDistanceThreshold(cs);
            }

            double getCollisionSafetyMargin() const 
            {
                return collision_safety_margin_;
            }

            void setNegativeDistance(double nd)
            {
                collision_manager_->setNegativeDistanceThreshold(nd);
            }

            double getNegativeDistance() const 
            {
                return negative_distance_;
            }

            std::vector<std::string> getActiveLinks() const 
            {
                return names_;
            }

            MotionModel getMotionModel() const 
            {
                return mtype_;
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

            MotionModel   mtype_;

            base::StateSpacePtr gspace_;

            GeometricStateExtractor se_;

            std::vector<std::string> names_;
            
            int robotCount_{0};

            /** @brief The discrete contact manager used for creating cached discrete contact managers. */
            collision::collision_box2d::Box2dDiscreteBVHManagerPtr collision_manager_;

            /** @brief Contact manager caching mutex */
            mutable std::mutex mutex_;
            /** @brief The discrete contact manager cache */
            mutable std::map<unsigned long int, collision::collision_box2d::Box2dDiscreteBVHManagerPtr> collision_managers_;

            double collision_safety_margin_{0.0001};
            double negative_distance_{-0.05};

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
