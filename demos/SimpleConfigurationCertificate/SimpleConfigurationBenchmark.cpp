#include <omplapp/config.h>
#include "SimpleCertificateStateValidityChecker.h"

#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// feasible planners
#include "CSCRRT.h"
#include "CSCRRTConnect.h"
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

// optimal planners
#include "CSCRRTstar.h"
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

void addPlanner(ompl::tools::Benchmark &benchmark, const ompl::base::PlannerPtr& planner)
{
    benchmark.addPlanner(planner);
}

bool argParse(int argc, char** argv, std::string &env, bool &optimal, double &checkResolution, double &optimalT);

int main(int argc, char* argv[])
{
    bool optimal;
    std::string env;
    double checkResolution;
    double optimalT;
    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, env, optimal, checkResolution, optimalT))
        return -1;

    std::string benchmark_name = "SimpleConfigurationCertificate";
    if (optimal)
        benchmark_name = "Optimal" + benchmark_name + env;

    auto space(std::make_shared<ob::RealVectorStateSpace>(2));
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0.0);
    bounds.setHigh(1.0);
    space->setBounds(bounds);

    auto si(std::make_shared<ob::SpaceInformation>(space));
    auto svc = std::make_shared<app::SimpleCertificateStateValidityChecker>(si, 0.05);
    svc->setEnvironmentFile(std::string(OMPLAPP_RESOURCE_DIR) + "/" + env);

    auto circle(std::make_shared<app::geometries::Circle>(0.0));
    svc->setRobotShape(circle);

    si->setStateValidityChecker(svc);
    si->setStateValidityCheckingResolution(checkResolution);
    si->setup();

    ompl::geometric::SimpleSetup setup(si);

    // define start state
    base::ScopedState<base::RealVectorStateSpace> start(setup.getSpaceInformation());
    start->values[0] = 0.05;
    start->values[1] = 0.05;

    // define goal state
    base::ScopedState<base::RealVectorStateSpace> goal(start);
    goal->values[0] = 0.95;
    goal->values[1] = 0.95;
    setup.setStartAndGoalStates(start, goal);
    
    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(optimalT));
        setup.setOptimizationObjective(obj);
    }

    setup.setup();

    double runtime_limit = 1000.0, memory_limit = 1024;
    int run_count = 20;

    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.01, true, true, false);
    ompl::tools::Benchmark b(setup, benchmark_name);

    if (optimal)
    {
        addPlanner(b, std::make_shared<ompl::geometric::RRTstar>(si));
        addPlanner(b, std::make_shared<ompl::geometric::CSCRRTstar>(si));
    }
    else 
    {
        addPlanner(b, std::make_shared<ompl::geometric::RRT>(si));
        addPlanner(b, std::make_shared<ompl::geometric::RRTConnect>(si));
        addPlanner(b, std::make_shared<ompl::geometric::CSCRRT>(si));
        addPlanner(b, std::make_shared<ompl::geometric::CSCRRTConnect>(si));
    }

    b.benchmark(request);
    b.saveResultsToFile((benchmark_name + ".log").c_str());

    return 0;
}

/** Parse the command line arguments into a string for an output file and the planner/optimization types */
bool argParse(int argc, char** argv, std::string &env, bool &optimal, double &checkResolution, double &optimalT)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("env,e", bpo::value<std::string>()->default_value("random_scenarios.ply"), "(Optional) Specify the polygon and circle environment, defaults to random_scenarios.ply if not given.")
        ("optimal,o", bpo::value<bool>()->default_value(false), "(Optional) Specify if it is an optimal benchmark.")
        ("checkResolution", bpo::value<double>()->default_value(0.01), "(Optional) Specify the collision checking resolution. Defaults to 0.01 and must be greater than 0.")
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
    optimal = vm["optimal"].as<bool>();
    checkResolution = vm["checkResolution"].as<double>();
    optimalT = vm["optimalT"].as<double>();

    return true;
}
