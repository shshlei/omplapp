/*********************************************************************
 * Rice University Software Distribution License
 *
 * Copyright (c) 2010, Rice University
 * All Rights Reserved.
 *
 * For a full description see the file named LICENSE.
 *
 *********************************************************************/

/* Author: Shi Shenglei */

#include <omplapp/config.h>
#include <omplapp/apps/RealVector2RigidBodyPlanning.h>
#include <omplapp/geometry/detail/Box2dStateValidityChecker.h>

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>

// feasible planners
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/bispace/RRTBispace.h>
#include <ompl/geometric/planners/ase/BiASE.h>
#include <ompl/geometric/planners/hsc/BiHSC.h>
//#include <ompl/geometric/planners/hsc/HSCASE.h>

// optimal planners
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/bispace/RRTBispacestar.h>
#include <ompl/geometric/planners/ase/BiASEstar.h>
#include <ompl/geometric/planners/hsc/BiHSCstar.h>
//#include <ompl/geometric/planners/hsc/HSCASEstar.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <iostream>

using namespace ompl;
namespace ob = ompl::base;
namespace og = ompl::geometric;

// An enum of supported optimal planners, alphabetical order
enum PlannerType
{
    PLANNER_BFMTSTAR,
    PLANNER_BITSTAR,
    PLANNER_CFOREST,
    PLANNER_FMTSTAR,
    PLANNER_INF_RRTSTAR,
    PLANNER_PRMSTAR,
    PLANNER_RRTSTAR,
    PLANNER_SORRTSTAR,
    PLANNER_BIASE,
    PLANNER_BIASESTAR,
    PLANNER_BIHSC,
    PLANNER_BIHSCSTAR,
    PLANNER_HSCASE,
    PLANNER_HSCASESTAR,
    PLANNER_RRT,
    PLANNER_RRTBISPACE,
    PLANNER_RRTBISPACESTAR,
    PLANNER_RRTCONNECT,
    PLANNER_SBL,
};

ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr si, PlannerType plannerType, double range, double pen_distance, std::shared_ptr<app::Box2dStateValidityChecker> svc)
{
    switch (plannerType)
    {
		case PLANNER_BFMTSTAR:
        {
            auto planner = std::make_shared<og::BFMT>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_BITSTAR:
        {
            auto planner = std::make_shared<og::BITstar>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_CFOREST:
        {
            auto planner = std::make_shared<og::CForest>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_FMTSTAR:
        {
            auto planner = std::make_shared<og::FMT>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_INF_RRTSTAR:
        {
            auto planner = std::make_shared<og::InformedRRTstar>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("collision_range")))
                params.setParam(std::string("collision_range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_PRMSTAR:
        {
            auto planner = std::make_shared<og::PRMstar>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_RRTSTAR:
        {
            auto planner = std::make_shared<og::RRTstar>(si);
//            planner->setKNearest(false);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("collision_range")))
                params.setParam(std::string("collision_range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_SORRTSTAR:
        {
            auto planner = std::make_shared<og::SORRTstar>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_RRT:
        {
            auto planner = std::make_shared<og::RRT>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_RRTBISPACE:
        {
            auto planner = std::make_shared<og::RRTBispace>(si);
            planner->setLazyPath(false);
            planner->setLazyNode(false);
            planner->setUseBispace(true);
            planner->setRewire(false);
            planner->setRewireSort(false);
            planner->setAddIntermediateState(false);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_RRTBISPACESTAR:
        {
            auto planner = std::make_shared<og::RRTBispacestar>(si);
            planner->setLazyPath(true);
            planner->setLazyNode(true);
            planner->setUseBispace(true);
            planner->setAddIntermediateState(false);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("pen_distance")))
                params.setParam(std::string("pen_distance"), ompl::toString(pen_distance));

            return planner;
            break;
        }
        case PLANNER_BIASE:
        {
            auto planner = std::make_shared<og::BiASE>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("pen_distance")))
                params.setParam(std::string("pen_distance"), ompl::toString(pen_distance));
            return planner;
            break;
        }
        case PLANNER_BIASESTAR:
        {
            auto planner = std::make_shared<og::BiASEstar>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("pen_distance")))
                params.setParam(std::string("pen_distance"), ompl::toString(pen_distance));
            return planner;
            break;
        }
        case PLANNER_BIHSC:
        {
            auto planner = std::make_shared<og::BiHSC>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            planner->setUseCollisionCertificateChecker(true);
//            planner->setMaxInvalidNodeRatio(0.2);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("pen_distance")))
                params.setParam(std::string("pen_distance"), ompl::toString(pen_distance));
            return planner;
            break;
        }
        case PLANNER_BIHSCSTAR:
        {
            auto planner = std::make_shared<og::BiHSCstar>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            planner->setUseCollisionCertificateChecker(true);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("pen_distance")))
                params.setParam(std::string("pen_distance"), ompl::toString(pen_distance));
            return planner;
            break;
        }
        /*
        case PLANNER_HSCASE:
        {
            auto planner = std::make_shared<og::HSCASE>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            planner->setMaxInvalidNodeRatio(0.05);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_HSCASESTAR:
        {
            auto planner = std::make_shared<og::HSCASEstar>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            planner->setLazyNode(true);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        */
        case PLANNER_RRTCONNECT:
        {
            auto planner = std::make_shared<og::RRTConnect>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_SBL:
        {
            auto planner = std::make_shared<og::SBL>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        default:
        {
            OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
            return ob::PlannerPtr(); // Address compiler warning re: no return value.
            break;
        }
    }
}

void envRandomPolygonsCircles(bool optimal, bool default_param, app::RealVector2RigidBodyPlanning &setup, double &range, double &pen_distance)
{
    setup.getStateSpace()->as<ob::RealVectorStateSpace>()->setBounds(0.0, 1.0);

    // define start state
    base::ScopedState<base::RealVectorStateSpace> start(setup.getSpaceInformation());
    start->values[0] = 0.05;
    start->values[1] = 0.05;

    // define goal state
    base::ScopedState<base::RealVectorStateSpace> goal(start);
    goal->values[0] = 0.95;
    goal->values[1] = 0.95;

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.001);
    if (optimal)
    {
        auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
        obj->setCostThreshold(base::Cost(std::sqrt(1.0)));
        setup.setOptimizationObjective(obj);
    }
    setup.setup();

    pen_distance = 0.05;
    if (optimal)
        range = 0.2;
    else 
        range = 0.1;

    if (default_param)
    {
        range = 0.0;
        pen_distance = 0.0;
    }
}

// Parse the command-line arguments
bool argParse(int argc, char** argv, double &runTime, PlannerType &planner, bool &optimal, std::string &env, bool &default_param);

int main(int argc, char* argv[])
{
    // The parsed arguments
    bool optimal;
    bool default_param;
    double runTime;
    PlannerType plannerType;
    std::string env;

    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, runTime, plannerType, optimal, env, default_param))
        return -1;

    ompl::app::RealVector2RigidBodyPlanning setup;
    double range = 0.0;
    double pen_distance = 0.0;
    if (env == "RandomPolygonsCircles")
        envRandomPolygonsCircles(optimal, default_param, setup, range, pen_distance);

    auto si = setup.getSpaceInformation();
    auto svc = std::make_shared<app::Box2dStateValidityChecker>(si, setup.getMotionModel(),
            setup.getGeometricComponentStateSpace(), setup.getGeometricStateExtractor());
    svc->setEnvironmentFile(std::string(OMPLAPP_RESOURCE_DIR) + "/random_scenarios.ply");
    int vcount = 3;
    Eigen::Vector2d vecs[vcount];
//    double x[3] = {5.0 * 0.0086579571682871, -5.0 * 0.02506512753291945, 5.0 * 0.012808997914287135};
//    double y[3] = {5.0 * 0.028723505664735693, 5.0 * 0.01648451945791818, -5.0 * 0.027128021904145316};
    double x[3] = {0.0086579571682871, -0.02506512753291945, 0.012808997914287135};
    double y[3] = {0.028723505664735693, 0.01648451945791818, -0.027128021904145316};
    for (int i = 0; i < vcount; i++)
        vecs[i] = Eigen::Vector2d(x[i], y[i]);
    auto polygon(std::make_shared<app::geometries::Polygon>());
    polygon->set(vecs, vcount);
    svc->addRobotShape(polygon);
//    if (robot == 2)
//        svc->addRobotShape(polygon);
    setup.setStateValidityChecker(svc);

    // Construct the optimal planner specified by our command line argument.
    // This helper function is simply a switch statement.
    ob::PlannerPtr planner = allocatePlanner(si, plannerType, range, pen_distance, svc);
    setup.setPlanner(planner);

    for (unsigned int i = 0; i < 20; i++)
    {
        std::cout << "Iteration " << i + 1 << std::endl;

        setup.clear();
        setup.solve(runTime);

        // attempt to solve the problem, and print it to screen if a solution is found
        if (setup.haveSolutionPath())
        {
            std::ofstream out(boost::str(boost::format("pathsol_%i.txt") % i).c_str());
            og::PathGeometric &p = setup.getSolutionPath();
            p.interpolate();
            p.printAsMatrix(out);
            out.close();
        }

        base::PlannerData pd(si);
        setup.getPlannerData(pd);

        base::PlannerDataStorage pds;
        pds.store(pd, boost::str(boost::format("pdsol_%i.txt") % i).c_str());
    }

    return 0;
}

bool argParse(int argc, char** argv, double &runTime, PlannerType &planner, bool &optimal, std::string &env, bool &default_param)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("env,e", bpo::value<std::string>()->default_value("RandomPolygonsCircles"), "(Optional) Specify the Benchmark environment, defaults to RandomPolygonsCircles if not given. Valid options are RandomPolygonsCircles.")
        ("default_param,d", bpo::value<bool>()->default_value(false), "(Optional) Specify if the planner is set the default params.")
        ("runtime,t", bpo::value<double>()->default_value(1.0), "(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.")
        ("planner,p", bpo::value<std::string>()->default_value("RRTstar"), "(Optional) Specify the optimal planner to use, defaults to RRTstar if not given. Valid options are BFMTstar, BITstar, CForest, FMTstar, InformedRRTstar, PRMstar, RRTstar, SORRTstar, RRT, RRTConnect, RRTBispace, RRTBispacestar, BiASE, BiASEstar, BiHSC, BiHSCstar, HSCASE, HSCASEstar, SBL, LSCAI, LSCAIstar.");
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);

    // Check if the help flag has been given:
    if (vm.count("help") != 0u)
    {
        std::cout << desc << std::endl;
        return false;
    }

    // Get the runtime as a double
    runTime = vm["runtime"].as<double>();

    // Sanity check
    if (runTime <= 0.0)
    {
        std::cout << "Invalid runtime." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    default_param = vm["default_param"].as<bool>();

    std::string envStr = vm["env"].as<std::string>();

    // Map the string to the enum
    if (boost::iequals("RandomPolygonsCircles", envStr))
    {
        env = "RandomPolygonsCircles";
    }
    else
    {
        std::cout << "Invalid environment string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    // Get the specified planner as a string
    std::string plannerStr = vm["planner"].as<std::string>();

    optimal = false;
    // Map the string to the enum
    if (boost::iequals("BFMTstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_BFMTSTAR;
    }
    else if (boost::iequals("BITstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_BITSTAR;
    }
    else if (boost::iequals("CForest", plannerStr))
    {
        optimal = true;
        planner = PLANNER_CFOREST;
    }
    else if (boost::iequals("FMTstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_FMTSTAR;
    }
    else if (boost::iequals("InformedRRTstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_INF_RRTSTAR;
    }
    else if (boost::iequals("PRMstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_PRMSTAR;
    }
    else if (boost::iequals("RRTstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_RRTSTAR;
    }
    else if (boost::iequals("SORRTstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_SORRTSTAR;
    }
    else if (boost::iequals("RRT", plannerStr))
    {
        planner = PLANNER_RRT;
    }
    else if (boost::iequals("RRTBispace", plannerStr))
    {
        planner = PLANNER_RRTBISPACE;
    }
    else if (boost::iequals("RRTBispacestar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_RRTBISPACESTAR;
    }
    else if (boost::iequals("BiASE", plannerStr))
    {
        planner = PLANNER_BIASE;
    }
    else if (boost::iequals("BiASEstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_BIASESTAR;
    }
    else if (boost::iequals("BiHSC", plannerStr))
    {
        planner = PLANNER_BIHSC;
    }
    else if (boost::iequals("BiHSCstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_BIHSCSTAR;
    }
    else if (boost::iequals("HSCASE", plannerStr))
    {
        planner = PLANNER_HSCASE;
    }
    else if (boost::iequals("HSCASEstar", plannerStr))
    {
        optimal = true;
        planner = PLANNER_HSCASESTAR;
    }
    else if (boost::iequals("RRTConnect", plannerStr))
    {
        planner = PLANNER_RRTCONNECT;
    }
    else if (boost::iequals("SBL", plannerStr))
    {
        planner = PLANNER_SBL;
    }
    else
    {
        std::cout << "Invalid planner string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    return true;
}
