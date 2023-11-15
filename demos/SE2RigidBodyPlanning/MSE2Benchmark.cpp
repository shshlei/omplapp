#include <omplapp/config.h>
#include <omplapp/apps/SE2RigidBodyPlanning.h>
#include <omplapp/geometry/detail/ContactStateValidityChecker.h>

// feasible planners
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/bispace/CellBispace.h>
#include <ompl/geometric/planners/bispace/RRTBispace.h>
#include <ompl/geometric/planners/ase/BiASE.h>
#include <ompl/geometric/planners/hsc/BiHSC.h>
//#include <ompl/geometric/planners/hsc/HSCASE.h>

// optimal planners
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/bispace/CellBispacestar.h>
#include <ompl/geometric/planners/bispace/RRTBispacestar.h>
#include <ompl/geometric/planners/ase/BiASEstar.h>
#include <ompl/geometric/planners/hsc/BiHSCstar.h>
//#include <ompl/geometric/planners/hsc/HSCASEstar.h>

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
    void addPlanner(ompl::tools::Benchmark &benchmark, const ompl::base::PlannerPtr& planner)
    {
        benchmark.addPlanner(planner);
    }
}

void benchmarkMaze(bool optimal, std::string &benchmark_name, app::SE2RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    if (optimal)
        benchmark_name = std::string("OptimalMaze");
    else 
        benchmark_name = std::string("FeasibleMaze");
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

    runtime_limit = 50.0;
    memory_limit = 1024.0;
    run_count = 20;
    if (optimal)
        runtime_limit = 20.0;
}

void benchmarkBarriers(bool optimal, std::string &benchmark_name, app::SE2RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    if (optimal)
        benchmark_name = std::string("OptimalBarriers");
    else 
        benchmark_name = std::string("FeasibleBarriers");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/Barriers_easy_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/Barriers_easy_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE2StateSpace> start(setup.getSpaceInformation());
    start->setX(34.81);
    start->setY(-75.0);
    start->setYaw(0.0);

    // define goal state
    base::ScopedState<base::SE2StateSpace> goal(start);
    goal->setX(620.0);
    goal->setY(-375.0);
    goal->setYaw(-3.1415926);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(1000.0));
        setup.setOptimizationObjective(obj);
    }
    setup.setup();

    runtime_limit = 20.0;
    memory_limit = 1024.0;
    run_count = 20;
    if (optimal)
        runtime_limit = 20.0;
}

void benchmarkRandomPolygons(bool optimal, std::string &benchmark_name, app::SE2RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    if (optimal)
        benchmark_name = std::string("OptimalRandomPolygons");
    else 
        benchmark_name = std::string("FeasibleRandomPolygons");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/car2_planar_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/RandomPolygons_planar_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE2StateSpace> start(setup.getSpaceInformation());
    start->setX(-32.99);
    start->setY(42.85);
    start->setYaw(0.0);

    // define goal state
    base::ScopedState<base::SE2StateSpace> goal(start);
    goal->setX(14.01);
    goal->setY(-43.15);
    goal->setYaw(0.802851455917);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(1000.0));
        setup.setOptimizationObjective(obj);
    }
    setup.setup();

    runtime_limit = 20.0;
    memory_limit = 10000.0;
    run_count = 20;
    if (optimal)
        runtime_limit = 100.0;
}

void benchmarkBugTrap(bool optimal, std::string &benchmark_name, app::SE2RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    if (optimal)
        benchmark_name = std::string("OptimalBugTrap");
    else 
        benchmark_name = std::string("FeasibleBugTrap");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/car1_planar_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/BugTrap_planar_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE2StateSpace> start(setup.getSpaceInformation());
    start->setX(6.0);
    start->setY(12.0);
    start->setYaw(0.0);

    // define goal state
    base::ScopedState<base::SE2StateSpace> goal(start);
    goal->setX(-39.0);
    goal->setY(0.0);
    goal->setYaw(0.0);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(135.0));
        setup.setOptimizationObjective(obj);
    }
    setup.setup();

    runtime_limit = 50.0;
    memory_limit = 10000.0;
    run_count = 20;
    if (optimal)
        runtime_limit = 200.0;
}

bool argParse(int argc, char** argv, std::string &env, bool &optimal);

int main(int argc, char* argv[])
{
    // The parsed arguments
    std::string env;
    bool optimal;

    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, env, optimal))
        return -1;

    ompl::app::SE2RigidBodyPlanning setup;
    std::string benchmark_name;
    double runtime_limit, memory_limit;
    int run_count;

    if (env == "Maze")
        benchmarkMaze(optimal, benchmark_name, setup, runtime_limit, memory_limit, run_count);
    else if (env == "Barriers")
        benchmarkBarriers(optimal, benchmark_name, setup, runtime_limit, memory_limit, run_count);
    else if (env == "RandomPolygons")
        benchmarkRandomPolygons(optimal, benchmark_name, setup, runtime_limit, memory_limit, run_count);
    else if (env == "BugTrap")
        benchmarkBugTrap(optimal, benchmark_name, setup, runtime_limit, memory_limit, run_count);

    auto si = setup.getSpaceInformation();
    auto svc = std::make_shared<app::ContactStateValidityChecker>(si, setup.getMotionModel(), 0.05, -0.05,
            setup.getGeometricComponentStateSpace(), setup.getGeometricStateExtractor(), setup.getGeometrySpecification());
    setup.setStateValidityChecker(svc);

    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5, true, true, false);
    ompl::tools::Benchmark b(setup, benchmark_name);

    if (optimal)
    {
        /*
        addPlanner(b, std::make_shared<ompl::geometric::RRTstar>(si));
        addPlanner(b, std::make_shared<ompl::geometric::InformedRRTstar>(si));
        addPlanner(b, std::make_shared<ompl::geometric::PRMstar>(si));
        addPlanner(b, std::make_shared<ompl::geometric::LazyPRMstar>(si));
        addPlanner(b, std::make_shared<ompl::geometric::BITstar>(si));
        addPlanner(b, std::make_shared<ompl::geometric::LBTRRT>(si));
        */

        /*
        {
            auto planner = std::make_shared<ompl::geometric::RRTBispacestar>(si);
            addPlanner(b, planner);
        }
        */

        /*
        {
            auto planner = std::make_shared<ompl::geometric::CellBispacestar>(si);
            addPlanner(b, planner);
        }
        */

        {
            auto planner = std::make_shared<ompl::geometric::BiASEstar>(si);
            addPlanner(b, planner);
        }

        /*
        {
            auto planner = std::make_shared<ompl::geometric::BiHSCstar>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setUseCollisionCertificateChecker(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::BiHSCstar>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::HSCASEstar>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::HSCASEstar>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::HSCASEstar>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            addPlanner(b, planner);
        }
        */
    }
    else 
    {
        /*
        addPlanner(b, std::make_shared<ompl::geometric::PRM>(si));
        addPlanner(b, std::make_shared<ompl::geometric::SBL>(si));
        addPlanner(b, std::make_shared<ompl::geometric::EST>(si));
        addPlanner(b, std::make_shared<ompl::geometric::BKPIECE1>(si));
        addPlanner(b, std::make_shared<ompl::geometric::LBKPIECE1>(si));
        addPlanner(b, std::make_shared<ompl::geometric::RRT>(si));
        addPlanner(b, std::make_shared<ompl::geometric::RRTConnect>(si));
        addPlanner(b, std::make_shared<ompl::geometric::LazyRRT>(si));
        addPlanner(b, std::make_shared<ompl::geometric::PRM>(si));
        addPlanner(b, std::make_shared<ompl::geometric::STRIDE>(si));
        */

        /*
        {
            auto planner = std::make_shared<ompl::geometric::RRTBispace>(si);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::CellBispace>(si);
            addPlanner(b, planner);
        }
        */

        /*
        {
            auto planner = std::make_shared<ompl::geometric::BiHSC>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setUseCollisionCertificateChecker(true);
            addPlanner(b, planner);
        }
        */

        {
            auto planner = std::make_shared<ompl::geometric::BiASE>(si);
            addPlanner(b, planner);
        }

        /*
        {
            auto planner = std::make_shared<ompl::geometric::BiHSC>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setUseCollisionCertificateChecker(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::BiHSC>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::HSCASE>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::HSCASE>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::HSCASE>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            addPlanner(b, planner);
        }
        */
    }

    b.benchmark(request);
    if (optimal)
        b.saveResultsToFile((benchmark_name + "BiASEstar" + ".log").c_str());
    else
        b.saveResultsToFile((benchmark_name + "BiASE" + ".log").c_str());

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
        ("env,e", bpo::value<std::string>()->default_value("Maze"), "(Optional) Specify the Benchmark environment, defaults to Maze if not given. Valid options are Maze, Barriers, RandomPolygons, BugTrap.");
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
    else if (boost::iequals("Barriers", envStr))
    {
        env = "Barriers";
    }
    else if (boost::iequals("RandomPolygons", envStr))
    {
        env = "RandomPolygons";
    }
    else if (boost::iequals("BugTrap", envStr))
    {
        env = "BugTrap";
    }
    else
    {
        std::cout << "Invalid environment string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    return true;
}
