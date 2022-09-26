#include <omplapp/geometry/geometries/Geometry.h>
#include <omplapp/geometry/geometries/MeshParser.h>
#include <omplapp/collision/BulletDiscreteBVHManager.h>

#include <ompl/util/Console.h>

#include <iostream>

#include <boost/filesystem.hpp>

using namespace ompl;
using namespace app;
using namespace geometries;
using namespace collision;

std::string toString(const Eigen::MatrixXd& a)
{
    std::stringstream ss;
    ss << a;
    return ss.str();
}

std::string toString(bool b)
{
    return b ? "true" : "false";
}

int main(int /*argc*/, char** /*argv*/)
{
    // Create Collision Manager
    collision_bullet::BulletDiscreteBVHManager checker;

    // Add box to checker
    CollisionShapePtr box(new Box(1.0, 1.0, 1.0));
    Eigen::Isometry3d box_pose;
    box_pose.setIdentity();

    CollisionShapes obj1_shapes;
    geometries::VectorIsometry3d obj1_poses;
    obj1_shapes.push_back(box);
    obj1_poses.push_back(box_pose);

    checker.addCollisionObject("box_link", 0, obj1_shapes, obj1_poses);

    // Add thin box to checker which is disabled
    CollisionShapePtr thin_box(new Box(0.1, 1.0, 1.0));
    Eigen::Isometry3d thin_box_pose;
    thin_box_pose.setIdentity();

    CollisionShapes obj2_shapes;
    geometries::VectorIsometry3d obj2_poses;
    obj2_shapes.push_back(thin_box);
    obj2_poses.push_back(thin_box_pose);

    checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses, false);

    // Add second box to checker, but convert to convex hull mesh.
    CollisionShapePtr second_box;

    geometries::VectorVector3d mesh_vertices;
    Eigen::VectorXi mesh_faces;
    loadSimplePlyFile(std::string(OMPLAPP_RESOURCE_DIR) + "/box_2m.ply", mesh_vertices, mesh_faces);

    // This is required because convex hull cannot have multiple faces on the same plane.
    std::shared_ptr<geometries::VectorVector3d> ch_verticies(new geometries::VectorVector3d());
    std::shared_ptr<Eigen::VectorXi> ch_faces(new Eigen::VectorXi());
    int ch_num_faces = createConvexHull(*ch_verticies, *ch_faces, mesh_vertices);
    second_box.reset(new ConvexMesh(ch_verticies, ch_faces, ch_num_faces));

    Eigen::Isometry3d second_box_pose;
    second_box_pose.setIdentity();

    CollisionShapes obj3_shapes;
    geometries::VectorIsometry3d obj3_poses;
    obj3_shapes.push_back(second_box);
    obj3_poses.push_back(second_box_pose);

    checker.addCollisionObject("second_box_link", 0, obj3_shapes, obj3_poses);

    OMPL_INFORM("Test when object is inside another");
    checker.setActiveCollisionObjects({ "box_link", "second_box_link" });
    checker.setContactDistanceThreshold(0.1);

    // Set the collision object transforms
    geometries::TransformMap location;
    location["box_link"] = Eigen::Isometry3d::Identity();
    location["box_link"].translation()(0) = 0.2;
    location["box_link"].translation()(1) = 0.1;
    location["second_box_link"] = Eigen::Isometry3d::Identity();

    checker.setCollisionObjectsTransform(location);

    // Perform collision check
    base::ContactResultMap result;
    checker.contactTest(result, ContactTestType::CLOSEST);

    ContactResultVector result_vector;
    flattenCopyResults(std::move(result), result_vector);

    OMPL_INFORM("Has collision: %s", toString(result_vector.empty()).c_str());
    OMPL_INFORM("Distance: %f", result_vector[0].distance);
    OMPL_INFORM("Link %s nearest point: %s",
                           result_vector[0].link_names[0].c_str(),
                           toString(result_vector[0].nearest_points[0]).c_str());
    OMPL_INFORM("Link %s nearest point: %s",
                           result_vector[0].link_names[1].c_str(),
                           toString(result_vector[0].nearest_points[1]).c_str());
    OMPL_INFORM("Direction to move Link %s out of collision with Link %s: %s",
                           result_vector[0].link_names[0].c_str(),
                           result_vector[0].link_names[1].c_str(),
                           toString(result_vector[0].normal).c_str());

    OMPL_INFORM("Test object is out side the contact distance");
    location["box_link"].translation() = Eigen::Vector3d(1.60, 0, 0);
    result = base::ContactResultMap();
    result.clear();
    result_vector.clear();

    checker.setCollisionObjectsTransform(location);

    // Check for collision after moving object
    checker.contactTest(result, ContactTestType::CLOSEST);
    flattenCopyResults(std::move(result), result_vector);

    OMPL_INFORM("Has collision: %s", toString(result_vector.empty()).c_str());

    OMPL_INFORM("Test object inside the contact distance");
    result = ContactResultMap();
    result.clear();
    result_vector.clear();

    // Set higher contact distance threshold
    checker.setContactDistanceThreshold(0.25);

    // Check for contact with new threshold
    checker.contactTest(result, ContactTestType::CLOSEST);
    flattenCopyResults(std::move(result), result_vector);

    OMPL_INFORM("Has collision: %s", toString(result_vector.empty()).c_str());
    OMPL_INFORM("Distance: %f", result_vector[0].distance);
    OMPL_INFORM("Link %s nearest point: %s",
                           result_vector[0].link_names[0].c_str(),
                           toString(result_vector[0].nearest_points[0]).c_str());
    OMPL_INFORM("Link %s nearest point: %s",
                           result_vector[0].link_names[1].c_str(),
                           toString(result_vector[0].nearest_points[1]).c_str());
    OMPL_INFORM("Direction to move Link %s further from Link %s: %s",
                           result_vector[0].link_names[0].c_str(),
                           result_vector[0].link_names[1].c_str(),
                           toString(result_vector[0].normal).c_str());
}
