#include "omplapp/geometry/detail/Box2dStateValidityChecker.h"
#include "omplapp/geometry/geometries/Geometry.h"
#include "omplapp/geometry/geometries/Types.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
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
        Box2dStateValidityChecker::Box2dStateValidityChecker(const ompl::base::SpaceInformationPtr& si, MotionModel mtype,  
                                                                double collision_safety_margin, double negative_distance, const base::StateSpacePtr &gspace,
                                                                const GeometricStateExtractor &se)
          : StateValidityChecker(si), mtype_(mtype), gspace_(gspace), se_(se),
            collision_safety_margin_(collision_safety_margin), negative_distance_(negative_distance)
        {
            assert(mtype == Motion_2D);
            collision_manager_ = std::make_shared<collision_box2d::Box2dDiscreteBVHManager>();
            collision_manager_->setContactDistanceThreshold(collision_safety_margin);
            collision_manager_->setNegativeDistanceThreshold(negative_distance);
        }

        bool Box2dStateValidityChecker::setEnvironmentFile(const std::string &env)
        {
            assert(collision_manager_->getBox2dBroadphse()->GetBodyCount() == 0);
            return read(env);
        }

//        void Box2dStateValidityChecker::setPointRobotNumber(int num)
//        {
//            assert(robotCount_ == 0);
//            robotCount_ = num;
//            names_.clear();
//            names_ = std::vector<std::string>(num, "");
//        }

        bool Box2dStateValidityChecker::addRobotShape(const collision::CollisionShapePtr& shape)
        {
            robotCount_++;
            std::string name = "Robot" + std::to_string(robotCount_);
            if (collision_manager_->addCollisionObject(name, shape, Eigen::Isometry2d::Identity(), true))
            {
                names_.push_back(name);
                return true;
            }
            robotCount_--;
            return false;
        }

        bool Box2dStateValidityChecker::isValid(const ompl::base::State *state) const
        {
            unsigned long int hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
            collision_box2d::Box2dDiscreteBVHManagerPtr cm;
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
                Eigen::Isometry2d tf = calculateCurrentTransform(state, i); 
//                if (names_[i].empty())
//                {
//                    if (cm->pointTest(tf.translation()))
//                        return false;
//                }
//                else 
                cm->setCollisionObjectsTransform(names_[i], tf);
            }
            return !cm->contactTest();
        }

        bool Box2dStateValidityChecker::isValid(const ompl::base::State *state, double &dist) const
        {
            dist = clearance(state);
            return dist >= collision_safety_margin_;
        }

        bool Box2dStateValidityChecker::isValid(const ompl::base::State *state, ompl::base::ContactResultVector &contact_vector, double &dist) const
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

        double Box2dStateValidityChecker::clearance(const ompl::base::State *state) const
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

        void Box2dStateValidityChecker::clearance(const ompl::base::State *state, ompl::base::ContactResultVector &contact_vector) const
        {
            unsigned long int hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
            collision_box2d::Box2dDiscreteBVHManagerPtr cm;
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
                Eigen::Isometry2d tf = calculateCurrentTransform(state, i); 
                cm->setCollisionObjectsTransform(names_[i], tf);
            }
            contact_vector.clear();
            base::ContactResult contact;
            if (cm->contactTest(contact))
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

        bool Box2dStateValidityChecker::read(const std::string& filename)
        {
            const boost::filesystem::path path = findMeshFile(filename);
            if (path.empty())
            {
                OMPL_ERROR("File '%s' not found in mesh path.", filename.c_str());
                return false;
            }

            std::ifstream file(path.string().c_str(), std::ios_base::binary | std::ios_base::in);
            if (!file.is_open())
            {
                OMPL_ERROR("Filestream to %s is not open, nothing read.", filename.c_str());
                return false;
            }

            return read(file);
        }

        bool Box2dStateValidityChecker::read(std::istream &s)
        {
            const std::string& fileHeader = "# Random polygons and circles file";

            // check if first line valid:
            std::string line;
            std::getline(s, line);
            if (line.compare(0, fileHeader.length(), fileHeader) !=0)
            {
                OMPL_ERROR("First line of file header does not start with %s", fileHeader);
                return false;
            }

            int numbers = 0;
            if (!readHeader(s, numbers))
                return false;

            int shapec = 0;
            int type = 0;
            int count = 0;
            double x = 0.0, y = 0.0, r = 0.0;
            while (s.good())
            {
                s >> type;
                if (!s.good())
                    break;
                b2Shape *shape = nullptr;
                std::string name = "";
                if (type == 0)
                {
                    s >> x >> y >> r;
                    b2CircleShape *cshape = new b2CircleShape();
                    cshape->SetRadius(b2Scalar(r));
                    cshape->m_p.Set(b2Scalar(x), b2Scalar(y));
                    shape = cshape;
                    shapec++;
                    name = "Circle" + std::to_string(shapec);
                }
                else if (type == 1) 
                {
                    s >> count; 
                    b2Vec2 vecs[count];
                    for (int i = 0; i < count; i++)
                    {
                        s >> x >> y;
                        vecs[i].Set(b2Scalar(x), b2Scalar(y));
                    }
                    b2PolygonShape *pshape = new b2PolygonShape();
                    pshape->SetRadius(b2Scalar(0.0));
                    pshape->Set(vecs, count);
                    shape = pshape;
                    shapec++;
                    name = "Polygon" + std::to_string(shapec);
                }

                if (shape != nullptr)
                {
                    collision_manager_->getBox2dBroadphse()->AddBody(name, shape, false);
                    delete shape;
                }
            }

            if (numbers != collision_manager_->getBox2dBroadphse()->GetBodyCount())
                OMPL_WARN("The shape number is wrong in this file, %d VS %d!", numbers, collision_manager_->getBox2dBroadphse()->GetBodyCount());

            return true;
        }

        bool Box2dStateValidityChecker::readHeader(std::istream& s, int& numbers)
        {
            numbers = 0;

            std::string token;
            bool headerRead = false;

            while (s.good() && !headerRead)
            {
                s >> token;
                if (token == "shapes")
                {
                    headerRead = true;
                    // skip forward until end of line:
                    char c;
                    do {
                        c = s.get();
                    } while(s.good() && (c != '\n'));
                }
                else if (token.compare(0, 1, "#") == 0)
                {
                    // comment line, skip forward until end of line:
                    char c;
                    do {
                        c = s.get();
                    } while(s.good() && (c != '\n'));
                }
                else if (token == "numbers")
                    s >> numbers;
                else
                {
                    OMPL_WARN("Unknown keyword in Random Polygons and Circles header, skipping: %s", token);
                    char c;
                    do {
                        c = s.get();
                    } while(s.good() && (c != '\n'));

                }
            }

            if (!headerRead)
            {
                OMPL_ERROR("Error reading Random Polygons and Circles header!");
                return false;
            }

            return true;
        }
    }
}
