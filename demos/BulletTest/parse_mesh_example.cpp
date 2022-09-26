#include <omplapp/geometry/geometries/Geometry.h>
#include <omplapp/geometry/geometries/MeshParser.h>

#include <ompl/util/Console.h>

#include <iostream>

#include <boost/filesystem.hpp>

using namespace ompl;
using namespace app;
using namespace geometries;

int main(int /*argc*/, char** /*argv*/)
{
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/car1_planar_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/Maze_planar_env.dae";

    std::vector<GeometryPtr> meshes = createMeshFromPath<Mesh>(env_fname);

    // Print mesh information
    OMPL_INFORM("Number of meshes: %u", meshes.size());
    OMPL_INFORM("Mesh #1 Triangle Count: %u", meshes[0]->as<Mesh>()->getTriangleCount());
    OMPL_INFORM("Mesh #1 Triangle Count: %u", meshes[0]->as<Mesh>()->getVerticeCount());
    OMPL_INFORM("Mesh #2 Triangle Count: %u", meshes[1]->as<Mesh>()->getTriangleCount());
    OMPL_INFORM("Mesh #2 Triangle Count: %u", meshes[1]->as<Mesh>()->getVerticeCount());
}
