#include <omplapp/config.h>
#include <omplapp/apps/SE2MultiRigidBodyPlanning.h>
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
//#include <ompl/geometric/planners/ase/BiASE.h>
//#include <ompl/geometric/planners/hsc/BiHSC.h>
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
//#include <ompl/geometric/planners/ase/BiASEstar.h>
//#include <ompl/geometric/planners/hsc/BiHSCstar.h>
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

void benchmarkMaze(bool optimal, std::string &benchmark_name, app::SE2MultiRigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    if (optimal)
        benchmark_name = std::string("MultiOptimalMaze");
    else 
        benchmark_name = std::string("MultiFeasibleMaze");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/car1_planar_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/Maze_planar_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.addRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::CompoundStateSpace> start(setup.getSpaceInformation());
    base::ScopedState<base::CompoundStateSpace> goal(setup.getSpaceInformation());

    // define starting state for robot 1
    auto *start1 = start->as<base::SE2StateSpace::StateType>(0);
    start1->setXY(-30., -20.);
    start1->setYaw(0.17453292519943295);
    // define goal state for robot 1
    auto *goal1 = goal->as<base::SE2StateSpace::StateType>(0);
    goal1->setXY(40., 28.);
    goal1->setYaw(-0.4363323129985824);

    // define starting state for robot 2
    auto *start2 = start->as<base::SE2StateSpace::StateType>(1);
    start2->setXY(-33., 28.);
    start2->setYaw(0.);
    // define goal state for robot 2
    auto *goal2 = goal->as<base::SE2StateSpace::StateType>(1);
    goal2->setXY(38., -28.);
    goal2->setYaw(0.4363323129985824);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(400.0));
        setup.setOptimizationObjective(obj);
    }
    setup.setup();

    runtime_limit = 300.0;
    memory_limit = 10000.0;
    run_count = 20;
    if (optimal)
        runtime_limit = 2000.0;
}

void benchmarkBarriers(bool optimal, std::string &benchmark_name, app::SE2MultiRigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    if (optimal)
        benchmark_name = std::string("MultiOptimalBarriers");
    else 
        benchmark_name = std::string("MultiFeasibleBarriers");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/Barriers_easy_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/Barriers_easy_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.addRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::CompoundStateSpace> start(setup.getSpaceInformation());
    base::ScopedState<base::CompoundStateSpace> goal(setup.getSpaceInformation());

    // define starting state for robot 1
    auto *start1 = start->as<base::SE2StateSpace::StateType>(0);
    start1->setX(34.81);
    start1->setY(-75.0);
    start1->setYaw(0.0);
    // define goal state for robot 1
    auto *goal1 = goal->as<base::SE2StateSpace::StateType>(0);
    goal1->setX(620.0);
    goal1->setY(-375.0);
    goal1->setYaw(-3.1415926);

    // define starting state for robot 2
    auto *start2 = start->as<base::SE2StateSpace::StateType>(1);
    start2->setX(620.0);
    start2->setY(-75.0);
    start2->setYaw(-3.1415926);
    // define goal state for robot 2
    auto *goal2 = goal->as<base::SE2StateSpace::StateType>(1);
    goal2->setX(34.81);
    goal2->setY(-375.0);
    goal2->setYaw(0.0);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(2000.0));
        setup.setOptimizationObjective(obj);
    }
    setup.setup();

    runtime_limit = 300.0;
    memory_limit = 10000.0;
    run_count = 20;
    if (optimal)
        runtime_limit = 2000.0;
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

    app::SE2MultiRigidBodyPlanning setup(2);
    std::string benchmark_name;
    double runtime_limit, memory_limit;
    int run_count;

    if (env == "Maze")
        benchmarkMaze(optimal, benchmark_name, setup, runtime_limit, memory_limit, run_count);
    else if (env == "Barriers")
        benchmarkBarriers(optimal, benchmark_name, setup, runtime_limit, memory_limit, run_count);

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

        {
            auto planner = std::make_shared<ompl::geometric::RRTBispacestar>(si);
            planner->setLazyPath(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::RRTBispacestar>(si);
            planner->setLazyNode(true);
            addPlanner(b, planner);
        }
        */

        {
            auto planner = std::make_shared<ompl::geometric::RRTBispacestar>(si);
            addPlanner(b, planner);
        }

        /*
        {
            auto planner = std::make_shared<ompl::geometric::BiASEstar>(si);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::BiASEstar>(si);
            planner->setLazyNode(true);
            addPlanner(b, planner);
        }
        */

        /*
        {
            auto planner = std::make_shared<ompl::geometric::BiHSCstar>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
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
            auto planner = std::make_shared<ompl::geometric::BiHSCstar>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            planner->setStrictCertificate(true);
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
            planner->setStrictCertificate(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::RRTBispacestar>(si);
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::RRTBispacestar>(si);
            planner->setLazyPath(true);
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::RRTBispacestar>(si);
            planner->setLazyNode(true);
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::BiASEstar>(si);
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::BiASEstar>(si);
            planner->setLazyNode(true);
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::BiHSCstar>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::BiHSCstar>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::BiHSCstar>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            planner->setStrictCertificate(true);
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::HSCASEstar>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::HSCASEstar>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::HSCASEstar>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            planner->setStrictCertificate(true);
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }
        */

    }
    else 
    {
        /*
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
            auto planner = std::make_shared<ompl::geometric::RRTBispace>(si);
            planner->setLazyPath(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::RRTBispace>(si);
            planner->setLazyNode(true);
            addPlanner(b, planner);
        }
        */

        {
            auto planner = std::make_shared<ompl::geometric::RRTBispace>(si);
            addPlanner(b, planner);
        }

        /*
        {
            auto planner = std::make_shared<ompl::geometric::BiASE>(si);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::BiASE>(si);
            planner->setLazyNode(true);
            addPlanner(b, planner);
        }
        */

        /*
        {
            auto planner = std::make_shared<ompl::geometric::BiHSC>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
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
            auto planner = std::make_shared<ompl::geometric::BiHSC>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            planner->setStrictCertificate(true);
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
            planner->setStrictCertificate(true);
            addPlanner(b, planner);
        }


        {
            auto planner = std::make_shared<ompl::geometric::RRTBispace>(si);
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::RRTBispace>(si);
            planner->setLazyPath(true);
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::RRTBispace>(si);
            planner->setLazyNode(true);
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::BiASE>(si);
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::BiASE>(si);
            planner->setLazyNode(true);
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::BiHSC>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::BiHSC>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::BiHSC>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            planner->setStrictCertificate(true);
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::HSCASE>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::HSCASE>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::HSCASE>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            planner->setStrictCertificate(true);
            planner->setTreatedAsMultiSubapce(true);
            addPlanner(b, planner);
        }
        */

    }

    b.benchmark(request);
    b.saveResultsToFile((benchmark_name + "Bispace" + ".log").c_str());

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
        ("env,e", bpo::value<std::string>()->default_value("Maze"), "(Optional) Specify the Benchmark environment, defaults to Maze if not given. Valid options are Maze, Barriers.");
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
    else
    {
        std::cout << "Invalid environment string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    return true;
}
