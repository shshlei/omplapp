#include <omplapp/geometry/geometries/Geometry.h>
#include <omplapp/geometry/geometries/Types.h>
#include <ompl/util/Console.h>

#include <iostream>

using namespace ompl;
using namespace app;

int main(int /*argc*/, char** /*argv*/)
{
    // Shape Box
    auto box = std::make_shared<geometries::Box>(1.0, 1.0, 1.0);
    // Shape Cone
    auto cone = std::make_shared<geometries::Cone>(1.0, 1.0);
    // Shape Capsule
    auto capsule = std::make_shared<geometries::Capsule>(1.0, 1.0);
    // Shape Cylinder
    auto cylinder = std::make_shared<geometries::Cylinder>(1.0, 1.0);
    // Shape Plane
    auto plane = std::make_shared<geometries::Plane>(1.0, 1.0, 1.0, 1.0);
    // Shape Sphere
    auto sphere = std::make_shared<geometries::Sphere>(1.0);

    // Manually create mesh
    std::shared_ptr<const geometries::VectorVector3d> mesh_vertices = std::make_shared<const geometries::VectorVector3d>();
    std::shared_ptr<const Eigen::VectorXi> mesh_faces = std::make_shared<const Eigen::VectorXi>();
    // Next fill out vertices and triangles
    auto mesh = std::make_shared<geometries::Mesh>(mesh_vertices, mesh_faces);

    // Manually create signed distance field mesh
    std::shared_ptr<const geometries::VectorVector3d> sdf_vertices = std::make_shared<const geometries::VectorVector3d>();
    std::shared_ptr<const Eigen::VectorXi> sdf_faces = std::make_shared<const Eigen::VectorXi>();
    // Next fill out vertices and triangles
    auto sdf_mesh = std::make_shared<geometries::SDFMesh>(sdf_vertices, sdf_faces);

    // Manually create convex mesh
    std::shared_ptr<const geometries::VectorVector3d> convex_vertices = std::make_shared<const geometries::VectorVector3d>();
    std::shared_ptr<const Eigen::VectorXi> convex_faces = std::make_shared<const Eigen::VectorXi>();
    // Next fill out vertices and triangles
    auto convex_mesh = std::make_shared<geometries::ConvexMesh>(convex_vertices, convex_faces);

#ifdef OMPL_HAS_OCTOMAP
    // Create an octree
    std::shared_ptr<const octomap::OcTree> octree;
    auto octree_t = std::make_shared<geometries::Octree>(octree, geometries::Octree::SubType::BOX);
#endif
}
