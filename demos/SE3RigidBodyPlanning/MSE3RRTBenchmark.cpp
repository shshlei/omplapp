#include <omplapp/config.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/geometry/detail/ContactStateValidityChecker.h>

// feasible planners
#include <ompl/geometric/planners/rrt/RRT.h>

// optimal planners
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

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

void benchmarkHome(bool optimal, std::string &benchmark_name, app::SE3RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    if (optimal)
        benchmark_name = std::string("RRTOptimalHome");
    else 
        benchmark_name = std::string("RRTFeasibleHome");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Home_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Home_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(252.95); // home
    start->setY(-214.95);
    start->setZ(46.19);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(262.95); // home
    goal->setY(75.05);
    goal->setZ(46.19);
    goal->rotation().setIdentity();

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(1400.0));
        setup.setOptimizationObjective(obj);
    }
    setup.setup();

    runtime_limit = 50.0;
    memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    run_count = 20;
    if (optimal)
        runtime_limit = 200.0;
}

void benchmarkCubicles(bool optimal, std::string &benchmark_name, app::SE3RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    if (optimal)
        benchmark_name = std::string("RRTOptimalCubicles");
    else 
        benchmark_name = std::string("RRTFeasibleCubicles");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(-4.96);
    start->setY(-40.62);
    start->setZ(70.57);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(200.49);
    goal->setY(-40.62);
    goal->setZ(70.57);
    goal->rotation().setIdentity();

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(2000.0));
        setup.setOptimizationObjective(obj);
    }
    setup.setup();

    runtime_limit = 20.0;
    memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    run_count = 20;
    if (optimal)
        runtime_limit = 50;
}

void benchmarkTwistycool(bool optimal, std::string &benchmark_name, app::SE3RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    if (optimal)
        benchmark_name = std::string("RRTOptimalTwistycool");
    else 
        benchmark_name = std::string("RRTFeasibleTwistycool");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(100.0); // Twistycool
    start->setY(15.0);
    start->setZ(-400.0);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(400.0); // Twistycool
    goal->setY(15.0);
    goal->setZ(-200.0);
    goal->rotation().setIdentity();

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(600.0));
        setup.setOptimizationObjective(obj);
    }
    setup.setup();

    runtime_limit = 50.0;
    memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    run_count = 20;
    if (optimal)
        runtime_limit = 200.0;
}

void benchmarkTwistycooler(bool optimal, std::string &benchmark_name, app::SE3RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    if (optimal)
        benchmark_name = std::string("RRTOptimalTwistycooler");
    else 
        benchmark_name = std::string("RRTFeasibleTwistycooler");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycooler_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycooler_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(121.81); // Twistycooler 
    start->setY(151.73);
    start->setZ(-188.99);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(388.81); // Twistycooler
    goal->setY(70.73);
    goal->setZ(-457.99);
    goal->rotation().setIdentity();

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(1100.0));
        setup.setOptimizationObjective(obj);
    }
    setup.setup();

    runtime_limit = 10.0;
    memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    run_count = 20;
    if (optimal)
        runtime_limit = 200.0;
}

void benchmarkAbstract(bool optimal, std::string &benchmark_name, app::SE3RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    if (optimal)
        benchmark_name = std::string("RRTOptimalAbstract");
    else 
        benchmark_name = std::string("RRTFeasibleAbstract");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Abstract_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Abstract_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(84.98); // Abstract 
    start->setY(-60.00);
    start->setZ(220.00);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(-121.02); // Abstract 
    goal->setY(12.00);
    goal->setZ(153.16);
    goal->rotation().setAxisAngle(1.0, 0.0, 0.0, 1.570796327);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(700.0));
        setup.setOptimizationObjective(obj);
    }
    setup.setup();

    runtime_limit = 50.0;
    memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    run_count = 20;
    if (optimal)
        runtime_limit = 200.0;
}

void benchmarkApartment(bool optimal, std::string &benchmark_name, app::SE3RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    if (optimal)
        benchmark_name = std::string("RRTOptimalApartment");
    else 
        benchmark_name = std::string("RRTFeasibleApartment");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(-31.19); // apartment 
    start->setY(-99.85);
    start->setZ(36.46);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(241.81); // apartment 
    goal->setY(106.15);
    goal->setZ(36.46);
    goal->rotation().setIdentity();

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(500.0));
        setup.setOptimizationObjective(obj);
    }
    setup.setup();

    runtime_limit = 200.0;
    memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    run_count = 20;
    if (optimal)
        runtime_limit = 500.0;
}

void benchmarkApartmentReverse(bool optimal, std::string &benchmark_name, app::SE3RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    if (optimal)
        benchmark_name = std::string("RRTOptimalApartmentReverse");
    else 
        benchmark_name = std::string("RRTFeasibleApartmentReverse");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(241.81); // apartment-reverse 
    start->setY(106.15);
    start->setZ(36.46);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(-31.19); // apartment-reverse 
    goal->setY(-99.85);
    goal->setZ(36.46);
    goal->rotation().setIdentity();

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(500.0));
        setup.setOptimizationObjective(obj);
    }
    setup.setup();

    runtime_limit = 200.0;
    memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    run_count = 20;
    if (optimal)
        runtime_limit = 500.0;
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

    ompl::app::SE3RigidBodyPlanning setup;
    std::string benchmark_name;
    double runtime_limit, memory_limit;
    int run_count;

    if (env == "Home")
        benchmarkHome(optimal, benchmark_name, setup, runtime_limit, memory_limit, run_count);
    else if (env == "Cubicles")
        benchmarkCubicles(optimal, benchmark_name, setup, runtime_limit, memory_limit, run_count);
    else if (env == "Twistycool")
        benchmarkTwistycool(optimal, benchmark_name, setup, runtime_limit, memory_limit, run_count);
    else if (env == "Twistycooler")
        benchmarkTwistycooler(optimal, benchmark_name, setup, runtime_limit, memory_limit, run_count);
    else if (env == "Abstract")
        benchmarkAbstract(optimal, benchmark_name, setup, runtime_limit, memory_limit, run_count);
    else if (env == "Apartment")
        benchmarkApartment(optimal, benchmark_name, setup, runtime_limit, memory_limit, run_count);
    else if (env == "ApartmentReverse")
        benchmarkApartmentReverse(optimal, benchmark_name, setup, runtime_limit, memory_limit, run_count);
    
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
        ("env,e", bpo::value<std::string>()->default_value("Home"), "(Optional) Specify the Benchmark environment, defaults to Home if not given. Valid options are Home, Cubicles, Twistycool, Twistycooler, Abstract, Apartment, Apartment-Reverse.");
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
    if (boost::iequals("Home", envStr))
    {
        env = "Home";
    }
    else if (boost::iequals("Cubicles", envStr))
    {
        env = "Cubicles";
    }
    else if (boost::iequals("Twistycool", envStr))
    {
        env = "Twistycool";
    }
    else if (boost::iequals("Twistycooler", envStr))
    {
        env = "Twistycooler";
    }
    else if (boost::iequals("Abstract", envStr))
    {
        env = "Abstract";
    }
    else if (boost::iequals("Apartment", envStr))
    {
        env = "Apartment";
    }
    else if (boost::iequals("Apartment-Reverse", envStr))
    {
        env = "Apartment-Reverse";
    }
    else
    {
        std::cout << "Invalid environment string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    return true;
}
