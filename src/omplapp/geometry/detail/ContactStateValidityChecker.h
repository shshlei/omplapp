#ifndef OMPLAPP_GEOMETRY_DETAIL_CONTACT_STATE_VALIDITY_CHECKER_
#define OMPLAPP_GEOMETRY_DETAIL_CONTACT_STATE_VALIDITY_CHECKER_

#include <ompl/base/Contact.h>
#include <ompl/base/SafetyCertificate.h>
#include <ompl/base/StateValidityChecker.h>

#include "omplapp/collision/BulletDiscreteContactManager.h"

#include "omplapp/geometry/GeometrySpecification.h"

#include <mutex>

namespace ompl
{
    namespace app
    {
        class ContactStateValidityChecker : public ompl::base::StateValidityChecker
        {
        public:

            using SafetyCertificateChecker = std::function<bool(const ompl::base::State *state, const ompl::base::SafetyCertificate *sc, std::vector<double> &dist)>;

            using CollisionCertificateChecker = std::function<bool(const ompl::base::State *state, const std::vector<ompl::base::SafetyCertificate *> &ocv)>;

            using DistanceCertificate = std::function<std::vector<double>(const base::State *a, const base::State *b)>;

            ContactStateValidityChecker(const ompl::base::SpaceInformationPtr& si, MotionModel mtype,  
                                        double collision_safety_margin, double negative_distance, const base::StateSpacePtr &gspace,
                                        const GeometricStateExtractor &se, const GeometrySpecification &geom);

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

        private:

            MotionModel   mtype_;

            base::StateSpacePtr gspace_;

            GeometricStateExtractor se_;

            std::vector<std::string> names_;

            ompl::base::AlignedMap<std::string, std::pair<collision::CollisionShapePtr, Eigen::Isometry3d>> internalShapes1, internalShapes2;

            /** @brief The discrete contact manager used for creating cached discrete contact managers. */
            collision::DiscreteContactManagerPtr collision_manager_;

            collision::DiscreteContactManagerPtr collision_manager_inter_;

            // The items below are to cache the contact manager based on thread ID. Currently ompl is multi
            // threaded but the methods used to implement collision checking are not thread safe. To prevent
            // reconstructing the collision environment for every check this will cache a contact manager
            // based on its thread ID.

            /** @brief Contact manager caching mutex */
            mutable std::mutex mutex_;

            /** @brief The discrete contact manager cache */
            mutable std::map<unsigned long int, collision::DiscreteContactManagerPtr> collision_managers_;

            mutable std::map<unsigned long int, collision::DiscreteContactManagerPtr> collision_managers_inter_;

            double collision_safety_margin_{0.01};

            double negative_distance_{-0.05};

            SafetyCertificateChecker safetyCertificateChecker_ = std::bind(&ContactStateValidityChecker::safetyCertificate, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

            CollisionCertificateChecker collisionCertificateChecker_ = std::bind(&ContactStateValidityChecker::collisionCertificate, this, std::placeholders::_1, std::placeholders::_2);

            DistanceCertificate distanceCertificate_ = std::bind(&ContactStateValidityChecker::distanceCertificate, this, std::placeholders::_1, std::placeholders::_2);

        private:

            bool isValid(const ompl::base::State *state1, const ompl::base::State *state2) const;

            Eigen::Isometry3d calculateCurrentTransform(const ompl::base::State *state, unsigned int index) const;

            struct DistanceCompare 
            {
                DistanceCompare()
                {
                }

                bool operator()(const base::ContactResult &a, const base::ContactResult &b)
                {
                    return a.distance < b.distance;
                }
            };

            struct PositionCompare
            {
                PositionCompare(const std::vector<std::string> &names, MotionModel mtype)
                    : names_(names), mtype_(mtype)
                {
                }

                bool operator()(const base::ContactResult &a, const base::ContactResult &b)
                {
                    auto itA = std::find(names_.begin(), names_.end(), a.link_names[0]);

                    unsigned int i = 0, j = 0;

                    if (itA != names_.end())
                    {
                        if (b.link_names[1] == a.link_names[i])
                            j = 1;
                    }
                    else 
                    {
                        i = 1, j = 1;
                        if (b.link_names[0] == a.link_names[i])
                            j = 0;
                    }

                    if (a.nearest_points_local[i][0] != b.nearest_points_local[j][0])
                        return a.nearest_points_local[i][0] < b.nearest_points_local[j][0];
                    else if (a.nearest_points_local[i][1] != b.nearest_points_local[j][1])
                        return a.nearest_points_local[i][1] < b.nearest_points_local[j][1];
                    else if (mtype_ == Motion_3D)
                        return a.nearest_points_local[i][2] < b.nearest_points_local[j][2];
                    else 
                        return false;
                }

                std::vector<std::string> names_;
                MotionModel mtype_;
            };

            struct PositionUniqueCompare
            {
                PositionUniqueCompare(const std::vector<std::string> &names, MotionModel mtype)
                    : names_(names), mtype_(mtype)
                {
                }

                bool operator()(const base::ContactResult &a, const base::ContactResult &b)
                {
                    auto itA = std::find(names_.begin(), names_.end(), a.link_names[0]);
                    
                    unsigned int i = 0, j = 0;

                    if (itA != names_.end())
                    {
                        if (b.link_names[1] == a.link_names[i])
                            j = 1;
                    }
                    else 
                    {
                        i = 1, j = 1;
                        if (b.link_names[0] == a.link_names[i])
                            j = 0;
                    }

                    if (a.nearest_points_local[i][0] != b.nearest_points_local[j][0])
                        return std::abs(a.nearest_points_local[i][0] - b.nearest_points_local[j][0]) < 0.1;
                    else if (a.nearest_points_local[i][1] != b.nearest_points_local[j][1])
                        return std::abs(a.nearest_points_local[i][1] - b.nearest_points_local[j][1]) < 0.1;
                    else if (mtype_ == Motion_3D)
                        return std::abs(a.nearest_points_local[i][2] - b.nearest_points_local[j][2]) < 0.1;
                    else 
                        return true;
                }

                std::vector<std::string> names_;
                MotionModel mtype_;
            };
        };
    }
}

#endif
