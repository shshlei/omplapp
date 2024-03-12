#include <omplapp/config.h>
#include <omplapp/geometry/detail/Box2dStateValidityChecker.h>

#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

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

bool argParse(int argc, char** argv, std::string &env, std::string &robot, std::string &spaceType, double &checkResolution, bool &optimal, double &optimalT);

const base::State* getState(const base::State* state, unsigned int /*index*/)
{
    return state;
}

int main(int argc, char* argv[])
{
    bool optimal;
    double checkResolution;
    double optimalT;
    std::string env;
    std::string spaceType;
    std::string robot;
    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, env, robot, spaceType, checkResolution, optimal, optimalT))
        return -1;

    std::string benchmark_name = "RandomPolygons";
    if (optimal)
        benchmark_name = "Optimal" + benchmark_name;

    ob::StateSpacePtr space;
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0.0);
    bounds.setHigh(1.0);
    if (spaceType == "RealVector2")
    {
        auto space1(std::make_shared<ob::RealVectorStateSpace>(2));
        space1->setBounds(bounds);
        space = space1;
        benchmark_name += "RealVector2";
    }
    else if (spaceType == "SE2")
    {
        auto space1(std::make_shared<ob::SE2StateSpace>());
        space1->setBounds(bounds);
        space = space1;
        benchmark_name += "SE2";
    }
    else if (spaceType == "Dubins")
    {
        auto space1(std::make_shared<ob::DubinsStateSpace>(0.02, false));
        space1->setBounds(bounds);
        space = space1;
        benchmark_name += "Dubins";
    }
    else if (spaceType == "ReedsShepp")
    {
        auto space1(std::make_shared<ob::ReedsSheppStateSpace>(0.02));
        space1->setBounds(bounds);
        space = space1;
        benchmark_name += "ReedsShepp";
    }

    auto si(std::make_shared<ob::SpaceInformation>(space));
    auto svc = std::make_shared<app::Box2dStateValidityChecker>(si, space, getState);
    svc->setEnvironmentFile(std::string(OMPLAPP_RESOURCE_DIR) + "/" + env);
    svc->addRobotShape(std::string(OMPLAPP_RESOURCE_DIR) + "/" + robot);

    si->setStateValidityChecker(svc);
    si->setStateValidityCheckingResolution(checkResolution);
    si->setup();

    ompl::geometric::SimpleSetup setup(si);

    if (spaceType == "RealVector2")
    {
        // define start state
        base::ScopedState<base::RealVectorStateSpace> start(setup.getSpaceInformation());
        start->values[0] = 0.05;
        start->values[1] = 0.05;

        // define goal state
        base::ScopedState<base::RealVectorStateSpace> goal(start);
        goal->values[0] = 0.95;
        goal->values[1] = 0.95;
        setup.setStartAndGoalStates(start, goal);
    }
    else 
    {
        base::ScopedState<base::SE2StateSpace> start(setup.getSpaceInformation());
        start->setX(0.05);
        start->setY(0.05);
        start->setYaw(0.0);

        // define goal state
        base::ScopedState<base::SE2StateSpace> goal(start);
        goal->setX(0.95);
        goal->setY(0.95);
        goal->setYaw(0.0);

        setup.setStartAndGoalStates(start, goal);
    }

    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(optimalT));
        setup.setOptimizationObjective(obj);
    }

    setup.setup();

    double runtime_limit = 200.0, memory_limit = 1024;
    int run_count = 2;
    //if (optimal)
    //    runtime_limit = 20.0;

    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.01, true, true, false);
    ompl::tools::Benchmark b(setup, benchmark_name);

    if (optimal)
    {
        /*
        addPlanner(b, std::make_shared<ompl::geometric::RRTstar>(si));
        addPlanner(b, std::make_shared<ompl::geometric::InformedRRTstar>(si));
        addPlanner(b, std::make_shared<ompl::geometric::BITstar>(si));
        addPlanner(b, std::make_shared<ompl::geometric::PRMstar>(si));
        addPlanner(b, std::make_shared<ompl::geometric::LazyPRMstar>(si));
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

        /*
        {
            auto planner = std::make_shared<ompl::geometric::RRTBispacestar>(si);
            addPlanner(b, planner);
        }

        {
            auto planner = std::make_shared<ompl::geometric::CellBispacestar>(si);
            addPlanner(b, planner);
        }
        */

        {
            auto planner = std::make_shared<ompl::geometric::BiASEstar>(si);
            planner->setAddIntermediateState(false);
            planner->setUseBispace(false);
            //planner->setBackRewire(false);
            //planner->setUpdateNbCell(true);
            addPlanner(b, planner);
        }

        /*
        {
            auto planner = std::make_shared<ompl::geometric::BiHSCstar>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            //planner->setUseCollisionCertificateChecker(true);
            planner->setUseBispace(false);
            addPlanner(b, planner);
        }
        */

        /*
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
        addPlanner(b, std::make_shared<ompl::geometric::RRT>(si));
        addPlanner(b, std::make_shared<ompl::geometric::RRTConnect>(si));
        /*
        addPlanner(b, std::make_shared<ompl::geometric::SBL>(si));
        addPlanner(b, std::make_shared<ompl::geometric::EST>(si));
        addPlanner(b, std::make_shared<ompl::geometric::BKPIECE1>(si));
        addPlanner(b, std::make_shared<ompl::geometric::LBKPIECE1>(si));
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
            //planner->setUseCollisionCertificateChecker(true);
            planner->setUseBispace(false);
            addPlanner(b, planner);
        }
        */
    }

    b.benchmark(request);
    b.saveResultsToFile((benchmark_name + ".log").c_str());

    return 0;
}

/** Parse the command line arguments into a string for an output file and the planner/optimization types */
bool argParse(int argc, char** argv, std::string &env, std::string &robot, std::string &spaceType, double &checkResolution, bool &optimal, double &optimalT)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("env,e", bpo::value<std::string>()->default_value("random_scenarios.ply"), "(Optional) Specify the polygon and circle environment, defaults to random_scenarios.ply if not given.")
        ("robot", bpo::value<std::string>()->default_value("robot_triangle1.ply"), "(Optional) Specify the robot, defaults to robot_triangle1.ply if not given.")
        ("spacetype,s", bpo::value<std::string>()->default_value("RealVector2"), "(Optional) Specify the planning space type, default to RealVector2 if not given. Valid options are RealVector2, SE2, Dubins, ReedsShepp.")
        ("checkResolution", bpo::value<double>()->default_value(0.01), "(Optional) Specify the collision checking resolution. Defaults to 0.01 and must be greater than 0.")
        ("optimal,o", bpo::value<bool>()->default_value(false), "(Optional) Specify if it is an optimal benchmark.")
        ("optimalT", bpo::value<double>()->default_value(1.30), "(Optional) Specify the optimal path length threshold. Defaults to 1.30 and must be greater than 0.");
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);

    // Check if the help flag has been given:
    if (vm.count("help") != 0u)
    {
        std::cout << desc << std::endl;
        return false;
    }

    env = vm["env"].as<std::string>();
    robot = vm["robot"].as<std::string>();

    spaceType = vm["spacetype"].as<std::string>();
    if (boost::iequals("RealVector2", spaceType))
    {
        spaceType = "RealVector2";
    }
    else if (boost::iequals("SE2", spaceType))
    {
        spaceType = "SE2";
    }
    else if (boost::iequals("Dubins", spaceType))
    {
        spaceType = "Dubins";
    }
    else if (boost::iequals("ReedsShepp", spaceType))
    {
        spaceType = "ReedsShepp";
    }

    checkResolution = vm["checkResolution"].as<double>();
    optimal = vm["optimal"].as<bool>();
    optimalT = vm["optimalT"].as<double>();

    return true;
}
