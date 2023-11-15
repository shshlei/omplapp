#include <omplapp/config.h>

#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// feasible planners
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/ase/BiASE.h>

// optimal planners
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/ase/BiASEstar.h>

// benchmark
#include <ompl/tools/benchmark/Benchmark.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

using namespace ompl;
namespace ob = ompl::base;

bool isStateValidEasy(const base::SpaceInformationPtr &si, const ob::State *state)
{
    if (!si->satisfiesBounds(state))
        return false;
    const auto* rvstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
    double x = rvstate->values[0], y = rvstate->values[1];
    if (x < 0.4)
        return true;
    if (x > 0.6)
        return true;
    if (y > 0.49 && y < 0.51)
        return true;
    return false;
}

bool isStateValidHard(const base::SpaceInformationPtr &si, const ob::State *state)
{
    if (!si->satisfiesBounds(state))
        return false;
    const auto* rvstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
    double x = rvstate->values[0], y = rvstate->values[1];
    if (x < 0.15)
        return true;
    if (x > 0.85)
        return true;
    if (y < 0.15)
        return true;
    if (y > 0.85)
        return true;
    if (x > 0.25 && x < 0.75)
    {
        if (y > 0.61 && y < 0.75)
            return true;
        if (y > 0.25 && y < 0.39)
            return true;
        if (y > 0.49 && y < 0.51)
            return true;
        if (x > 0.65 && y > 0.25 && y < 0.75)
            return true;
    }
    if (y > 0.49 && y < 0.51 && x < 0.75)
        return true;
    return false;
}

void addPlanner(ompl::tools::Benchmark &benchmark, const ompl::base::PlannerPtr& planner)
{
    benchmark.addPlanner(planner);
}

bool argParse(int argc, char** argv, double &checkResolution, bool &optimal, double &optimalT, bool &hard);

int main(int argc, char* argv[])
{
    double checkResolution;
    bool optimal;
    double optimalT;
    bool hard;
    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, checkResolution, optimal, optimalT, hard))
        return -1;

    std::string benchmark_name = "NarrowPassages";
    if (optimal)
        benchmark_name = "Optimal" + benchmark_name;
    if (hard)
        benchmark_name += "Hard";

    ob::RealVectorBounds bounds(2);
    bounds.setLow(0.0);
    bounds.setHigh(1.0);

    auto space(std::make_shared<ob::RealVectorStateSpace>(2));
    space->setBounds(bounds);

    auto si(std::make_shared<ob::SpaceInformation>(space));
    auto isStateValid = hard ? isStateValidHard : isStateValidEasy;
    si->setStateValidityChecker([isStateValid, si](const ob::State *state)
        {
            return isStateValid(si, state);
        });

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
    if (hard)
    {
        start->values[0] = 0.30;
        start->values[1] = 0.30;
        goal->values[0] = 0.05;
        goal->values[1] = 0.95;
    }
    setup.setStartAndGoalStates(start, goal);
    
    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(optimalT));
        setup.setOptimizationObjective(obj);
    }

    setup.setup();

    double runtime_limit = 10.0, memory_limit = 1024;
    int run_count = 100;

    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.01, true, true, false);
    ompl::tools::Benchmark b(setup, benchmark_name);

    if (optimal)
    {
        /*
        addPlanner(b, std::make_shared<ompl::geometric::RRTstar>(si));
        addPlanner(b, std::make_shared<ompl::geometric::InformedRRTstar>(si));
        addPlanner(b, std::make_shared<ompl::geometric::PRMstar>(si));
        addPlanner(b, std::make_shared<ompl::geometric::LazyPRMstar>(si));
        addPlanner(b, std::make_shared<ompl::geometric::BITstar>(si));
        */

        /*
        {
            auto planner = std::make_shared<ompl::geometric::BiASEstar>(si);
            addPlanner(b, planner);
        }
        */
    }
    else 
    {
        addPlanner(b, std::make_shared<ompl::geometric::RRT>(si));
        addPlanner(b, std::make_shared<ompl::geometric::RRTConnect>(si));
        /*
        addPlanner(b, std::make_shared<ompl::geometric::BKPIECE1>(si));
        addPlanner(b, std::make_shared<ompl::geometric::LBKPIECE1>(si));
        addPlanner(b, std::make_shared<ompl::geometric::LazyRRT>(si));
        addPlanner(b, std::make_shared<ompl::geometric::PRM>(si));
        */

        {
            auto planner = std::make_shared<ompl::geometric::BiASE>(si);
            planner->setAddIntermediateState(false);
            planner->setUseBispace(false);
            planner->setBackRewire(false);
            addPlanner(b, planner);
        }
    }

    b.benchmark(request);
    b.saveResultsToFile((benchmark_name + ".log").c_str());

    return 0;
}

bool argParse(int argc, char** argv, double &checkResolution, bool &optimal, double &optimalT, bool &hard)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("checkResolution", bpo::value<double>()->default_value(0.01), "(Optional) Specify the collision checking resolution. Defaults to 0.01 and must be greater than 0.")
        ("optimal,o", bpo::value<bool>()->default_value(false), "(Optional) Specify if it is an optimal benchmark.")
        ("optimalT", bpo::value<double>()->default_value(1.30), "(Optional) Specify the optimal path length threshold. Defaults to 1.30 and must be greater than 0.")
        ("hard", bpo::value<bool>()->default_value(false), "(Optional) Specify if it is the hard motion planning problem.");
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);

    // Check if the help flag has been given:
    if (vm.count("help") != 0u)
    {
        std::cout << desc << std::endl;
        return false;
    }

    checkResolution = vm["checkResolution"].as<double>();
    optimal = vm["optimal"].as<bool>();
    optimalT = vm["optimalT"].as<double>();
    hard = vm["hard"].as<bool>();
    return true;
}
