#include "omplapp/geometry/detail/ContactStateValidityChecker.h"
#include "omplapp/geometry/geometries/Geometry.h"
#include "omplapp/geometry/geometries/MeshParser.h"
#include "omplapp/geometry/geometries/Types.h"

#include "omplapp/collision/BulletDiscreteBVHManager.h"

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>

#include <ompl/util/Console.h>

#include <thread>
#include <limits>
#include <algorithm>

using namespace ompl::app::collision;

namespace ompl
{
    namespace app
    {
        ContactStateValidityChecker::ContactStateValidityChecker(const ompl::base::SpaceInformationPtr& si, MotionModel mtype,  
                                                                double collision_safety_margin, double negative_distance, const base::StateSpacePtr &gspace,
                                                                const GeometricStateExtractor &se, const GeometrySpecification &geom)
          : StateValidityChecker(si), mtype_(mtype), gspace_(gspace), se_(se),
            collision_safety_margin_(collision_safety_margin), negative_distance_(negative_distance)
        {
            collision_manager_ = std::make_shared<collision::collision_bullet::BulletDiscreteBVHManager>();
            collision_manager_inter_ = std::make_shared<collision::collision_bullet::BulletDiscreteBVHManager>();

            geometries::VectorIsometry3d mesh_poses;

            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            Eigen::Vector3d center = Eigen::Vector3d::Zero();

            names_.clear();
        
            for (unsigned int i = 0; i < geom.robot.size(); i++)
            {
                if (geom.robotShift.size() > i)
                {
                    aiVector3D v = geom.robotShift[i];

                    center = Eigen::Vector3d(static_cast<double>(v.x), static_cast<double>(v.y), static_cast<double>(v.z));
                }
                else 
                    center = Eigen::Vector3d::Zero();

                auto ch_meshes = geometries::createMeshFromAsset<geometries::Mesh>(geom.robot[i], Eigen::Vector3d(1, 1, 1), center);

                mesh_poses.resize(ch_meshes.size());
                std::fill(mesh_poses.begin(), mesh_poses.end(), pose);

                names_.push_back("robot" + std::to_string(i));
                collision_manager_->addCollisionObject(names_[i], 1, ch_meshes, mesh_poses);

                for (unsigned int j = 0; j < ch_meshes.size(); j++)
                {
                    internalShapes1[names_[i] + std::to_string(j)] = std::make_pair(ch_meshes[j], mesh_poses[j]);
                }
            }

            for (unsigned int i = 0; i < geom.obstacles.size(); i++)
            {
                if (geom.obstaclesShift.size() > i)
                {
                    aiVector3D v = geom.obstaclesShift[i];

                    center = Eigen::Vector3d(static_cast<double>(v.x), static_cast<double>(v.y), static_cast<double>(v.z));
                }
                else 
                    center = Eigen::Vector3d::Zero();

                auto ch_meshes = geometries::createMeshFromAsset<geometries::Mesh>(geom.obstacles[i], Eigen::Vector3d(1, 1, 1), center);

                mesh_poses.resize(ch_meshes.size());
                std::fill(mesh_poses.begin(), mesh_poses.end(), pose);

                collision_manager_->addCollisionObject("env" + std::to_string(i), 0, ch_meshes, mesh_poses);

                for (unsigned int j = 0; j < ch_meshes.size(); j++)
                    internalShapes2["env" + std::to_string(i) + std::to_string(j)] = std::make_pair(ch_meshes[j], mesh_poses[j]);
            }

            collision_manager_->setActiveCollisionObjects(names_);
            collision_manager_->setContactDistanceThreshold(collision_safety_margin);
            collision_manager_->setNegativeDistanceThreshold(negative_distance);

            collision_manager_inter_->setContactDistanceThreshold(collision_safety_margin);
            collision_manager_inter_->setNegativeDistanceThreshold(negative_distance);
        }

        bool ContactStateValidityChecker::isValid(const ompl::base::State *state) const
        {
            unsigned long int hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
            collision::DiscreteContactManagerPtr cm;
            mutex_.lock();
            auto it = collision_managers_.find(hash);
            if (it == collision_managers_.end())
            {
                cm = collision_manager_->clone();
                collision_managers_[hash] = cm;
            }
            else
            {
                cm = it->second;
            }
            mutex_.unlock();

            for (unsigned int i = 0; i < names_.size(); i++)
            {
                Eigen::Isometry3d tf = calculateCurrentTransform(state, i); 
                cm->setCollisionObjectsTransform(names_[i], tf);
            }

            base::ContactResultMap contact_map;
            cm->contactTest(contact_map, collision::ContactTestType::FIRST);

            return contact_map.empty();
        }

        bool ContactStateValidityChecker::isValid(const ompl::base::State *state, double &dist) const
        {
            dist = clearance(state);

            return dist >= collision_safety_margin_;
        }

        bool ContactStateValidityChecker::isValid(const ompl::base::State *state, ompl::base::ContactResultVector &contact_vector, double &dist) const
        {
            dist = collision_safety_margin_;

            clearance(state, contact_vector);

            if (!contact_vector.empty())
            {
                for (const auto& res : contact_vector)
                    if (res.distance < dist)
                        dist = res.distance;
            }

            return dist >= collision_safety_margin_;
        }

        double ContactStateValidityChecker::clearance(const ompl::base::State *state) const
        {
            double dist = collision_safety_margin_;

            ompl::base::ContactResultVector contact_vector;

            clearance(state, contact_vector);

            if (!contact_vector.empty())
            {
                for (const auto& res : contact_vector)
                    if (res.distance < dist)
                        dist = res.distance;
            }

            return dist;
        }

        void ContactStateValidityChecker::clearance(const ompl::base::State *state, ompl::base::ContactResultVector &contact_vector) const
        {
            unsigned long int hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
            collision::DiscreteContactManagerPtr cm;
            mutex_.lock();
            auto it = collision_managers_.find(hash);
            if (it == collision_managers_.end())
            {
                cm = collision_manager_->clone();
                collision_managers_[hash] = cm;
            }
            else
            {
                cm = it->second;
            }
            mutex_.unlock();

            for (unsigned int i = 0; i < names_.size(); i++)
            {
                Eigen::Isometry3d tf = calculateCurrentTransform(state, i); 

                cm->setCollisionObjectsTransform(names_[i], tf);
            }

            base::ContactResultMap contact_map;
            cm->contactTest(contact_map, collision::ContactTestType::NEGATIVE);

            contact_vector.clear();

            if (!contact_map.empty())
                flattenCopyResults(contact_map, contact_vector);
        }

        bool ContactStateValidityChecker::collisionCertificate(const base::State *state, const std::vector<base::SafetyCertificate *> &ocv) const
        {
            bool osc = false;

            unsigned long int hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
            collision::DiscreteContactManagerPtr cm;
            mutex_.lock();
            auto it = collision_managers_inter_.find(hash);
            if (it == collision_managers_inter_.end())
            {
                cm = collision_manager_inter_->clone();
                collision_managers_inter_[hash] = cm;
            }
            else
            {
                cm = it->second;
            }
            mutex_.unlock();

            for (const auto & oc : ocv)
            {
                bool check = true;
                const auto & ocontact_vector = *(oc->contact);
                for (const auto & res : ocontact_vector)
                {
                    if (res.distance < 0.0 || check)
                    {
                        std::string name_a = res.link_names[0], name_b = res.link_names[1];
                        int type_id_a = res.type_id[0], type_id_b = res.type_id[1];

                        Eigen::Isometry3d link_transform_a = Eigen::Isometry3d::Identity();
                        Eigen::Isometry3d link_transform_b = Eigen::Isometry3d::Identity();
                        if (type_id_a)
                        {
                            auto itA = std::find(names_.begin(), names_.end(), name_a);
                            link_transform_a = calculateCurrentTransform(state, itA - names_.begin());
                        }
                        if (type_id_b)
                        {
                            auto itB = std::find(names_.begin(), names_.end(), name_b);
                            link_transform_b = calculateCurrentTransform(state, itB - names_.begin());
                        }

                        if (res.distance < 0.0)
                        {
                            Eigen::Vector3d pointa = !type_id_a ? res.nearest_points[1] : link_transform_a * res.nearest_points_local2[0];
                            Eigen::Vector3d pointb = !type_id_b ? res.nearest_points[1] : link_transform_b * res.nearest_points_local[1];

                            Eigen::Vector3d pointa_2 = !type_id_a ? res.nearest_points[0] : link_transform_a * res.nearest_points_local[0];
                            Eigen::Vector3d pointb_2 = !type_id_b ? res.nearest_points[0] : link_transform_b * res.nearest_points_local2[1];

                            double dist = (pointa - pointb).norm(), dist_2 = (pointa_2 - pointb_2).norm();
                            if (dist + res.distance <= 2.0*collision_safety_margin_ && dist_2 + res.distance <= 2.0*collision_safety_margin_)
                            {
                                osc = true;
                                break;
                            }
                        }
                        else 
                        {
                            Eigen::Vector3d pointa = !type_id_a ? res.nearest_points[0] : link_transform_a * res.nearest_points_local[0];
                            Eigen::Vector3d pointb = !type_id_b ? res.nearest_points[1] : link_transform_b * res.nearest_points_local[1];

                            double dist = (pointa - pointb).norm();
                            if (dist <= 2.0*collision_safety_margin_)
                            {
                                osc = true;
                                break;
                            }
                        }

                        if (check)
                        {
                            name_a += std::to_string(res.shape_id[0]);
                            if (!cm->hasCollisionObject(name_a))
                            {
                                if (!type_id_a)
                                {
                                    auto subobject = internalShapes2.find(name_a)->second;
                                    cm->addCollisionObject(name_a, type_id_a, subobject.first, subobject.second);
                                }
                                else 
                                {
                                    auto subobject = internalShapes1.find(name_a)->second;
                                    cm->addCollisionObject(name_a, type_id_a, subobject.first, subobject.second);
                                }
                            }
                            if (type_id_a)
                                cm->setCollisionObjectsTransform(name_a, link_transform_a);

                            name_b += std::to_string(res.shape_id[1]);
                            if (!cm->hasCollisionObject(name_b))
                            {
                                if (!type_id_b)
                                {
                                    auto subobject = internalShapes2.find(name_b)->second;
                                    cm->addCollisionObject(name_b, type_id_b, subobject.first, subobject.second);
                                }
                                else 
                                {
                                    auto subobject = internalShapes1.find(name_b)->second;
                                    cm->addCollisionObject(name_b, type_id_b, subobject.first, subobject.second);
                                }
                            }
                            if (type_id_b)
                                cm->setCollisionObjectsTransform(name_b, link_transform_b);

                            cm->enableCollisionObject(name_a);
                            cm->enableCollisionObject(name_b);

                            base::ContactResultMap contact_map;
                            cm->contactTest(contact_map, collision::ContactTestType::FIRST);

                            cm->disableCollisionObject(name_a);
                            cm->disableCollisionObject(name_b);

                            if (!contact_map.empty())
                            {
                                osc = true;
                                break;
                            }

                            check = false;
                        }
                    }
                }

                if (osc)
                    break;
            }

            return osc;
        }

        bool ContactStateValidityChecker::safetyCertificate(const base::State *state, const base::SafetyCertificate *sc, std::vector<double> &dist) const
        {
            bool fsc = true;

            dist = distanceCertificate(sc->state, state); 

            for (std::size_t i = 0; i < names_.size(); i++)
            {
                if (dist[i] > sc->confidence_[i])
                {
                    fsc = false;
                    break;
                }
            }

            return fsc;
        }

        std::vector<double> ContactStateValidityChecker::distanceCertificate(const base::State *a, const base::State *b) const
        {
            std::vector<double> dist(names_.size());
            
            if (mtype_ == MotionModel::Motion_2D)
            {
                for (std::size_t i = 0; i < names_.size(); i++)
                {
                    const base::State *rvstate1 = se_(a, i)->as<ompl::base::SE2StateSpace::StateType>();
                    const base::State *rvstate2 = se_(b, i)->as<ompl::base::SE2StateSpace::StateType>();
                    double d = gspace_->as<ompl::base::SE2StateSpace>()->distance(rvstate1, rvstate2);

                    dist[i] = d;
                }
            }
            else 
            {
                for (std::size_t i = 0; i < names_.size(); i++)
                {
                    const base::State *rvstate1 = se_(a, i)->as<ompl::base::SE3StateSpace::StateType>();
                    const base::State *rvstate2 = se_(b, i)->as<ompl::base::SE3StateSpace::StateType>();
                    double d = gspace_->as<ompl::base::SE3StateSpace>()->distance(rvstate1, rvstate2);

                    dist[i] = d;
                }
            }

            return dist;
        }

        Eigen::Isometry3d ContactStateValidityChecker::calculateCurrentTransform(const ompl::base::State *state, unsigned int index) const
        {
            Eigen::Isometry3d link_transform = Eigen::Isometry3d::Identity();

            if (mtype_ == MotionModel::Motion_2D)
            {
                const auto* rvstate = se_(state, index)->as<ompl::base::SE2StateSpace::StateType>();

                link_transform.rotate(Eigen::AngleAxisd(rvstate->getYaw(), Eigen::Vector3d(0, 0, 1)).matrix());
                link_transform.pretranslate(Eigen::Vector3d(rvstate->getX(), rvstate->getY(), 0.0));
            }
            else 
            {
                const auto* rvstate = se_(state, index)->as<ompl::base::SE3StateSpace::StateType>();
                const auto &q = rvstate->rotation();

                link_transform.rotate(Eigen::Quaterniond(q.w, q.x, q.y, q.z).matrix());
                link_transform.pretranslate(Eigen::Vector3d(rvstate->getX(), rvstate->getY(), rvstate->getZ()));
            }

            return link_transform;
        }

        bool ContactStateValidityChecker::isValid(const ompl::base::State *state1, const ompl::base::State *state2) const
        {
            unsigned long int hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
            collision::DiscreteContactManagerPtr cm;
            mutex_.lock();
            auto it = collision_managers_.find(hash);
            if (it == collision_managers_.end())
            {
                cm = collision_manager_->clone();
                collision_managers_[hash] = cm;
            }
            else
            {
                cm = it->second;
            }
            mutex_.unlock();

            if (mtype_ == MotionModel::Motion_2D)
            {
                for (unsigned int i = 0; i < names_.size(); i++)
                {
                    const auto* rvstate1 = se_(state1, i)->as<ompl::base::SE2StateSpace::StateType>();
                    const auto* rvstate2 = se_(state2, i)->as<ompl::base::SE2StateSpace::StateType>();

                    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity(); 
                    tf.rotate(Eigen::AngleAxisd(rvstate1->getYaw(), Eigen::Vector3d(0, 0, 1)).matrix());
                    tf.pretranslate(Eigen::Vector3d(rvstate2->getX(), rvstate2->getY(), 0.0));

                    cm->setCollisionObjectsTransform(names_[i], tf);
                }
            }
            else 
            {
                for (unsigned int i = 0; i < names_.size(); i++)
                {
                    const auto* rvstate1 = se_(state1, i)->as<ompl::base::SE3StateSpace::StateType>();
                    const auto &q = rvstate1->rotation();

                    const auto* rvstate2 = se_(state2, i)->as<ompl::base::SE3StateSpace::StateType>();

                    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity(); 
                    tf.rotate(Eigen::Quaterniond(q.w, q.x, q.y, q.z).matrix());
                    tf.pretranslate(Eigen::Vector3d(rvstate2->getX(), rvstate2->getY(), rvstate2->getZ()));

                    cm->setCollisionObjectsTransform(names_[i], tf);
                }
            }

            base::ContactResultMap contact_map;
            cm->contactTest(contact_map, collision::ContactTestType::FIRST);

            return contact_map.empty();
        }
    }
}
