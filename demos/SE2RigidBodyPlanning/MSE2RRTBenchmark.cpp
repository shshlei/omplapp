#include <omplapp/config.h>
#include <omplapp/apps/SE2RigidBodyPlanning.h>
#include <omplapp/geometry/detail/ContactStateValidityChecker.h>

// feasible planners
#include <ompl/geometric/planners/rrt/RRT.h>

// optimal planners
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

// benchmark
#include <ompl/tools/benchmark/Benchmark.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

using namespace ompl;
namespace ob = ompl::base;

namespace 
{
    void addPlanner(ompl::tools::Benchmark &benchmark, const ompl::base::PlannerPtr& planner, double range, double collision_range)
    {
        ompl::base::ParamSet& params = planner->params();
        if (params.hasParam(std::string("range")))
            params.setParam(std::string("range"), ompl::toString(range));
        if (params.hasParam(std::string("collision_range")))
            params.setParam(std::string("collision_range"), ompl::toString(collision_range));
        benchmark.addPlanner(planner);
    }
}

void benchmarkMaze(bool optimal, std::string &benchmark_name, app::SE2RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    if (optimal)
        benchmark_name = std::string("RRTOptimalMaze");
    else 
        benchmark_name = std::string("RRTFeasibleMaze");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/car1_planar_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/Maze_planar_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE2StateSpace> start(setup.getSpaceInformation());
    start->setX(-30.0);
    start->setY(-20.0);
    start->setYaw(10.0 * 3.1415926 /180.0);

    // define goal state
    base::ScopedState<base::SE2StateSpace> goal(start);
    goal->setX(40.0);
    goal->setY(28.0);
    goal->setYaw(-25.0 * 3.1415926 /180.0);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(135.0));
        setup.setOptimizationObjective(obj);
    }
    setup.setup();

    runtime_limit = 20.0;
    memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    run_count = 20;
    if (optimal)
        runtime_limit = 50.0;
}

bool argParse(int argc, char** argv, std::string &env, bool &optimal);

int main(int argc, char* argv[])
{
    // The parsed arguments
    std::string env;
    bool optimal;

    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, env, optimal))
    {
        return -1;
    }

    ompl::app::SE2RigidBodyPlanning setup;
    std::string benchmark_name;
    double runtime_limit, memory_limit;
    int run_count;

    if (env == "Maze")
        benchmarkMaze(optimal, benchmark_name, setup, runtime_limit, memory_limit, run_count);

    auto si = setup.getSpaceInformation();
    auto svc = std::make_shared<app::ContactStateValidityChecker>(si, setup.getMotionModel(), 0.05, -0.05,
            setup.getGeometricComponentStateSpace(), setup.getGeometricStateExtractor(), setup.getGeometrySpecification());
    setup.setStateValidityChecker(svc);

    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5, true, true, false);
    ompl::tools::Benchmark b(setup, benchmark_name);

    double range = 15.0;
    if (optimal)
    {
        for (unsigned int i = 0; i < 15; i++)
        {
            addPlanner(b, std::make_shared<ompl::geometric::RRTstar>(si), range, range);
            range += 15.0;
        }   
    }
    else 
    {
        for (unsigned int i = 0; i < 15; i++)
        {
            addPlanner(b, std::make_shared<ompl::geometric::RRT>(si), range, range);
            range += 15.0;
        }
    }

    b.benchmark(request);
    b.saveResultsToFile((benchmark_name + ".log").c_str());

    return 0;
}

/** Parse the command line arguments into a string for an output file and the planner/optimization types */
bool argParse(int argc, char** argv, std::string &env, bool &optimal)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("optimal,o", bpo::value<bool>()->default_value(false), "(Optional) Specify if it is an optimal benchmark.")
        ("env,e", bpo::value<std::string>()->default_value("Maze"), "(Optional) Specify the Benchmark environment, defaults to Maze if not given. Valid options are Maze.");
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);

    // Check if the help flag has been given:
    if (vm.count("help") != 0u)
    {
        std::cout << desc << std::endl;
        return false;
    }

    optimal = vm["optimal"].as<bool>();
    std::string envStr = vm["env"].as<std::string>();

    // Map the string to the enum
    if (boost::iequals("Maze", envStr))
    {
        env = "Maze";
    }
    else
    {
        std::cout << "Invalid environment string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    return true;
}
