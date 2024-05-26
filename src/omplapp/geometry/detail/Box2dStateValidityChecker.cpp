#include "omplapp/geometry/detail/Box2dStateValidityChecker.h"
#include "omplapp/geometry/geometries/Geometry.h"
#include "omplapp/geometry/geometries/Types.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>

#include <ompl/util/Console.h>

#include <limits>
#include <algorithm>

using namespace ompl::app::collision;

namespace ompl
{
    namespace app
    {
        Box2dStateValidityChecker::Box2dStateValidityChecker(const ompl::base::SpaceInformationPtr& si, const base::StateSpacePtr &gspace)
          : StateValidityChecker(si), gspace_(gspace), se_(defaultGeometricStateExtractor)
        {
            collision_manager_ = std::make_shared<collision_box2d::Box2dDiscreteBVHManager>();
        }

        Box2dStateValidityChecker::Box2dStateValidityChecker(const ompl::base::SpaceInformationPtr& si, const base::StateSpacePtr &gspace, const GeometricStateExtractor &se)
          : StateValidityChecker(si), gspace_(gspace), se_(se)
        {
            collision_manager_ = std::make_shared<collision_box2d::Box2dDiscreteBVHManager>();
        }

        const std::vector<std::string> & Box2dStateValidityChecker::getRobotNames() const
        {
            return names_;
        }

        bool Box2dStateValidityChecker::isValid(const ompl::base::State *state) const
        {
            for (unsigned int i = 0; i < names_.size(); i++)
            {
                Eigen::Isometry2d tf = calculateCurrentTransform(state, i); 
                collision_manager_->setCollisionObjectsTransform(names_[i], tf);
            }
            return !collision_manager_->contactTest();
        }

        bool Box2dStateValidityChecker::isValid(const ompl::base::State *state, double &dist) const
        {
            dist = clearance(state);
            return dist > 0.0;
        }

        bool Box2dStateValidityChecker::isValid(const ompl::base::State *state, ompl::base::ContactResultVector &contact_vector, double &dist) const
        {
            dist = std::numeric_limits<double>::max();
            clearance(state, contact_vector);
            if (!contact_vector.empty())
            {
                for (const auto& res : contact_vector)
                    if (res.distance < dist)
                        dist = res.distance;
            }
            return dist > 0.0;
        }

        double Box2dStateValidityChecker::clearance(const ompl::base::State *state) const
        {
            double dist = std::numeric_limits<double>::max();
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

        void Box2dStateValidityChecker::clearance(const ompl::base::State *state, ompl::base::ContactResultVector &contact_vector) const
        {
            for (unsigned int i = 0; i < names_.size(); i++)
            {
                Eigen::Isometry2d tf = calculateCurrentTransform(state, i); 
                collision_manager_->setCollisionObjectsTransform(names_[i], tf);
            }
            contact_vector.clear();
            base::ContactResult contact;
            if (collision_manager_->contactTest(contact))
                contact_vector.push_back(contact);
        }

        /*
        bool Box2dStateValidityChecker::collisionCertificate(const base::State *state, const std::vector<base::SafetyCertificate *> &ocv) const
        {
            b2CircleShape *cshape = new b2CircleShape();
            collision_box2d::Box2dDiscreteBVHManagerPtr cm = std::make_shared<collision_box2d::Box2dDiscreteBVHManager>();
            b2Body* robot = cm->getBox2dBroadphse()->CreateBody("robot", true);
            int shapec = 0;
            for (const auto & oc : ocv)
            {
                const auto & ocontact_vector = *(oc->contact);
                for (const auto & res : ocontact_vector)
                {
                    if (res.distance > 0.0)
                        continue;
                    std::string name_a = res.link_names[0], name_b = res.link_names[1], name = "";
                    int type_id_a = res.type_id[0], type_id_b = res.type_id[1];
                    Eigen::Vector2d center_a = res.center[0].head<2>(), center_b = res.center[1].head<2>();
                    double radius_a = res.radius[0], radius_b = res.radius[1];
                    Eigen::Vector2d pointa = res.nearest_points[0].head<2>(), pointb = res.nearest_points[1].head<2>();
                    Eigen::Isometry2d link_transform_a = Eigen::Isometry2d::Identity();
                    Eigen::Isometry2d link_transform_b = Eigen::Isometry2d::Identity();
                    if (type_id_a)
                    {
                        auto itA = std::find(names_.begin(), names_.end(), name_a);
                        link_transform_a = calculateCurrentTransform(state, itA - names_.begin());
                        if (res.has_sphere[0])
                        {
                            center_a = link_transform_a * res.local_center[0].head<2>();
                            cshape->SetRadius(b2Scalar(radius_a));
                            cshape->m_p = collision_box2d::convertEigenToBox2d(center_a);
                            cm->getBox2dBroadphse()->AddShapeToBody(robot, cshape);
                        }

                        pointa = link_transform_a * res.nearest_points_local[0].head<2>();
                        cshape->SetRadius(b2Scalar(0.0));
                        cshape->m_p = collision_box2d::convertEigenToBox2d(pointa);
                        cm->getBox2dBroadphse()->AddShapeToBody(robot, cshape);
                    }
                    else  
                    {
                        if (res.has_sphere[0])
                        {
                            name = "obstacles" + std::to_string(shapec);
                            shapec++;
                            cshape->SetRadius(b2Scalar(radius_a));
                            cshape->m_p = collision_box2d::convertEigenToBox2d(center_a);
                            cm->getBox2dBroadphse()->AddBody(name, cshape, type_id_a);
                        }

                        name = "obstacles" + std::to_string(shapec);
                        shapec++;
                        cshape->SetRadius(b2Scalar(0.0));
                        cshape->m_p = collision_box2d::convertEigenToBox2d(pointa);
                        cm->getBox2dBroadphse()->AddBody(name, cshape, type_id_a);
                    }

                    if (type_id_b)
                    {
                        auto itB = std::find(names_.begin(), names_.end(), name_b);
                        link_transform_b = calculateCurrentTransform(state, itB - names_.begin());
                        if (res.has_sphere[1])
                        {
                            center_b = link_transform_b * res.local_center[1].head<2>();
                            cshape->SetRadius(b2Scalar(radius_b));
                            cshape->m_p = collision_box2d::convertEigenToBox2d(center_b);
                            cm->getBox2dBroadphse()->AddShapeToBody(robot, cshape);
                        }

                        pointb = link_transform_b * res.nearest_points_local[1].head<2>();
                        cshape->SetRadius(b2Scalar(0.0));
                        cshape->m_p = collision_box2d::convertEigenToBox2d(pointb);
                        cm->getBox2dBroadphse()->AddShapeToBody(robot, cshape);
                    }
                    else 
                    {
                        if (res.has_sphere[1])
                        {
                            name = "obstacles" + std::to_string(shapec);
                            shapec++;
                            cshape->SetRadius(b2Scalar(radius_b));
                            cshape->m_p = collision_box2d::convertEigenToBox2d(center_b);
                            cm->getBox2dBroadphse()->AddBody(name, cshape, type_id_b);
                        }

                        name = "obstacles" + std::to_string(shapec);
                        shapec++;
                        cshape->SetRadius(b2Scalar(0.0));
                        cshape->m_p = collision_box2d::convertEigenToBox2d(pointb);
                        cm->getBox2dBroadphse()->AddBody(name, cshape, type_id_b);
                    }
                }
            }

            delete cshape;
            return cm->contactTest();
        }
        */

        bool Box2dStateValidityChecker::collisionCertificate(const base::State *state, const std::vector<base::SafetyCertificate *> &ocv) const
        {
            bool osc = false;
            for (const auto & oc : ocv)
            {
                const auto & ocontact_vector = *(oc->contact);
                for (const auto & res : ocontact_vector)
                {
                    if (res.distance > 0.0)
                        continue;
                    std::string name_a = res.link_names[0], name_b = res.link_names[1];
                    int type_id_a = res.type_id[0], type_id_b = res.type_id[1];
                    Eigen::Vector2d center_a = res.center[0].head<2>(), center_b = res.center[1].head<2>();
                    double radius_a = res.radius[0], radius_b = res.radius[1];
                    Eigen::Isometry2d link_transform_a = Eigen::Isometry2d::Identity();
                    Eigen::Isometry2d link_transform_b = Eigen::Isometry2d::Identity();
                    if (type_id_a)
                    {
                        auto itA = std::find(names_.begin(), names_.end(), name_a);
                        link_transform_a = calculateCurrentTransform(state, itA - names_.begin());
                        center_a = link_transform_a * res.local_center[0].head<2>();
                    }
                    if (type_id_b)
                    {
                        auto itB = std::find(names_.begin(), names_.end(), name_b);
                        link_transform_b = calculateCurrentTransform(state, itB - names_.begin());
                        center_b = link_transform_b * res.local_center[1].head<2>();
                    }

                    if (res.has_sphere[0] && res.has_sphere[1])
                    {
                        double dist = (center_a - center_b).norm();
                        if (dist <= radius_a + radius_b)
                        {
                            osc = true;
                            break;
                        }
                    }
                    if (res.has_sphere[0])
                    {
                        Eigen::Vector2d pointb = !type_id_b ? res.nearest_points[1].head<2>() : link_transform_b * res.nearest_points_local[1].head<2>();
                        double dist = (center_a - pointb).norm();
                        if (dist <= radius_a)
                        {
                            osc = true;
                            break;
                        }
                    }
                    if (res.has_sphere[1])
                    {
                        Eigen::Vector2d pointa = !type_id_a ? res.nearest_points[0].head<2>() : link_transform_a * res.nearest_points_local[0].head<2>();
                        double dist = (center_b - pointa).norm();
                        if (dist <= radius_b)
                        {
                            osc = true;
                            break;
                        }
                    }
                }
                if (osc)
                    break;
            }
            return osc;
        }
                    /*
                    if (type_id_a)
                        cm->setCollisionObjectsTransform(name_a, link_transform_a);
                    if (type_id_b)
                        cm->setCollisionObjectsTransform(name_b, link_transform_b);

                    cm->enableCollisionObject(name_a);
                    cm->enableCollisionObject(name_b);
                    osc = cm->contactTest();
                    cm->disableCollisionObject(name_a);
                    cm->disableCollisionObject(name_b);
                    if (osc)
                        break;
                    check = false;
                    */

        bool Box2dStateValidityChecker::safetyCertificate(const base::State *state, const base::SafetyCertificate *sc, std::vector<double> &dist) const
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

        std::vector<double> Box2dStateValidityChecker::distanceCertificate(const base::State *a, const base::State *b) const
        {
            std::vector<double> dist(names_.size());
            for (std::size_t i = 0; i < names_.size(); i++)
            {
                const base::State *rvstate1 = se_(a, i);
                const base::State *rvstate2 = se_(b, i);
                double d = gspace_->distance(rvstate1, rvstate2);
                dist[i] = d;
            }
            return dist;
        }

        Eigen::Isometry2d Box2dStateValidityChecker::calculateCurrentTransform(const ompl::base::State *state, unsigned int index) const
        {
            Eigen::Isometry2d link_transform = Eigen::Isometry2d::Identity();
            if (gspace_->getType() == ompl::base::STATE_SPACE_REAL_VECTOR)
            {
                const auto* rvstate = se_(state, index)->as<ompl::base::RealVectorStateSpace::StateType>();
                link_transform.pretranslate(Eigen::Vector2d(rvstate->values[0], rvstate->values[1]));           
            }
            else 
            {
                const auto* rvstate = se_(state, index)->as<ompl::base::SE2StateSpace::StateType>();
                link_transform.rotate(rvstate->getYaw());
                link_transform.pretranslate(Eigen::Vector2d(rvstate->getX(), rvstate->getY()));
            }
            return link_transform;
        }

        boost::filesystem::path Box2dStateValidityChecker::findMeshFile(const std::string& fname)
        {
            boost::filesystem::path path(fname);
            if (boost::filesystem::exists(path))
                return boost::filesystem::absolute(path);
            if (path.is_absolute())
                return {};
            for (const auto &dir : meshPath_)
            {
                boost::filesystem::path candidate(dir / path);
                if (boost::filesystem::exists(candidate))
                    return boost::filesystem::absolute(candidate);
            }
            return {};
        }

        bool Box2dStateValidityChecker::setEnvironmentFile(const std::string &env)
        {
            if (collision_manager_->setEnvironmentFile(env))
            {
                 OMPL_INFORM("Set environment file successs!");
                return true;
            }
            OMPL_ERROR("Set environment file failed!");
            return false;
        }

        bool Box2dStateValidityChecker::addRobotShape(const collision::CollisionShapePtr& shape)
        {
            robotCount_++;
            std::string name = "Robot" + std::to_string(robotCount_);
            if (collision_manager_->addCollisionObject(name, shape, Eigen::Isometry2d::Identity(), true))
            {
                names_.push_back(name);
                OMPL_INFORM("Add robot shape successs!");
                return true;
            }
            OMPL_ERROR("Add robot shape failed!");
            robotCount_--;
            return false;
        }

        bool Box2dStateValidityChecker::addRobotShape(const std::string &robot)
        {
            robotCount_++;
            std::string name = "Robot" + std::to_string(robotCount_);
            if (collision_manager_->addRobotShape(robot, name))
            {
                names_.push_back(name);
                OMPL_INFORM("Add robot shape successs!");
                return true;
            }
            OMPL_ERROR("Add robot shape failed!");
            robotCount_--;
            return false;
        }

        void Box2dStateValidityChecker::removeRobotShape(const std::string & name)
        {
            names_.erase(std::find(names_.begin(), names_.end(), name));
            collision_manager_->removeRobotShape(name);
        }

        void Box2dStateValidityChecker::clearRobotShapes()
        {
            names_.clear();
            collision_manager_->clearRobotShapes();
        }
    }
}