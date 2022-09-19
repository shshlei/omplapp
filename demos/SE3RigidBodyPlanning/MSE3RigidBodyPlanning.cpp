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
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/geometry/detail/ContactStateValidityChecker.h>

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>

// feasible planners
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/bispace/RRTBispace.h>
#include <ompl/geometric/planners/bispace/RRTBispaceD.h>
#include <ompl/geometric/planners/bispace/CellBispace.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/ase/BiASE.h>
#include <ompl/geometric/planners/hsc/BiHSC.h>
#include <ompl/geometric/planners/hsc/BiHSCCell.h>
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
#include <ompl/geometric/planners/bispace/CellBispacestar.h>
#include <ompl/geometric/planners/ase/BiASEstar.h>
#include <ompl/geometric/planners/hsc/BiHSCstar.h>
#include <ompl/geometric/planners/hsc/BiHSCCellstar.h>
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
    PLANNER_BIHSCCELL,
    PLANNER_BIHSCCELLSTAR,
    PLANNER_HSCASE,
    PLANNER_HSCASESTAR,
    PLANNER_RRT,
    PLANNER_BKPIECE1,
    PLANNER_LBKPIECE1,
    PLANNER_RRTBISPACE,
    PLANNER_RRTBISPACESTAR,
    PLANNER_RRTBISPACED,
    PLANNER_RRTBISPACEDSTAR,
    PLANNER_CELLBISPACE,
    PLANNER_CELLBISPACESTAR,
    PLANNER_RRTCONNECT,
    PLANNER_LAZYRRT,
    PLANNER_SBL,
};

ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr si, PlannerType plannerType, double range, double pen_distance, std::shared_ptr<app::ContactStateValidityChecker> svc)
{
    switch (plannerType)
    {
		case PLANNER_BFMTSTAR:
        {
            auto planner = std::make_shared<og::BFMT>(si);
            planner->setNumSamples(500);
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
            planner->setNumSamples(2000);
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
        case PLANNER_BKPIECE1:
        {
            auto planner = std::make_shared<og::BKPIECE1>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_LBKPIECE1:
        {
            auto planner = std::make_shared<og::LBKPIECE1>(si);
//            planner->setMinValidPathFraction(0.5);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
        case PLANNER_RRTBISPACE:
        {
            auto planner = std::make_shared<og::RRTBispace>(si);
            planner->setLazyPath(true);
            planner->setLazyNode(true);
            planner->setRewire(true);
            planner->setRewireSort(true);
            planner->setUseBispace(true);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("pen_distance")))
                params.setParam(std::string("pen_distance"), ompl::toString(pen_distance));

            return planner;
            break;
        }
        case PLANNER_RRTBISPACESTAR:
        {
            auto planner = std::make_shared<og::RRTBispacestar>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("pen_distance")))
                params.setParam(std::string("pen_distance"), ompl::toString(pen_distance));

            return planner;
            break;
        }
        case PLANNER_RRTBISPACED:
        {
            auto planner = std::make_shared<og::RRTBispaceD>(si);
            planner->setLazyPath(true);
            planner->setLazyNode(true);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("pen_distance")))
                params.setParam(std::string("pen_distance"), ompl::toString(pen_distance));

            return planner;
            break;
        }
        case PLANNER_CELLBISPACE:
        {
            auto planner = std::make_shared<og::CellBispace>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("pen_distance")))
                params.setParam(std::string("pen_distance"), ompl::toString(pen_distance));

            return planner;
            break;
        }
        case PLANNER_CELLBISPACESTAR:
        {
            auto planner = std::make_shared<og::CellBispacestar>(si);
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
            planner->setUseCollisionCertificateChecker(true);
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
            planner->setUseCollisionCertificateChecker(true);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("pen_distance")))
                params.setParam(std::string("pen_distance"), ompl::toString(pen_distance));
            return planner;
            break;
        }
        case PLANNER_BIHSCCELL:
        {
            auto planner = std::make_shared<og::BiHSCCell>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("pen_distance")))
                params.setParam(std::string("pen_distance"), ompl::toString(pen_distance));

            return planner;
            break;
        }
        case PLANNER_BIHSCCELLSTAR:
        {
            auto planner = std::make_shared<og::BiHSCCellstar>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
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
            if (params.hasParam(std::string("pen_distance")))
                params.setParam(std::string("pen_distance"), ompl::toString(pen_distance));

            return planner;
            break;
        }
        case PLANNER_HSCASESTAR:
        {
            auto planner = std::make_shared<og::HSCASEstar>(si);
            planner->setSafetyCertificateChecker(svc->getSafetyCertificateChecker());
            planner->setCollisionCertificateChecker(svc->getCollisionCertificateChecker());
            planner->setDistanceCertificate(svc->getDistanceCertificate());
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));
            if (params.hasParam(std::string("pen_distance")))
                params.setParam(std::string("pen_distance"), ompl::toString(pen_distance));

            return planner;
            break;
        }
        */
        case PLANNER_LAZYRRT:
        {
            auto planner = std::make_shared<og::LazyRRT>(si);
            ompl::base::ParamSet& params = planner->params();
            if (params.hasParam(std::string("range")))
                params.setParam(std::string("range"), ompl::toString(range));

            return planner;
            break;
        }
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

void envHome(bool optimal, bool default_param, app::SE3RigidBodyPlanning &setup, double &range, double &pen_distance)
{
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
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
    obj->setCostThreshold(base::Cost(1400.0));
    setup.setOptimizationObjective(obj);
    setup.setup();

    pen_distance = 75.0;
    if (optimal)
        range = 75.0;
    else 
        range = 15.0;

    if (default_param)
    {
        range = 0.0;
        pen_distance = 0.0;
    }
}

void envCubicles(bool optimal, bool default_param, app::SE3RigidBodyPlanning &setup, double &range, double &pen_distance)
{
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
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
    obj->setCostThreshold(base::Cost(2000.0));
    setup.setOptimizationObjective(obj);
    setup.setup();

    pen_distance = 75.0;
    if (optimal)
        range = 75.0;
    else 
        range = 15.0;

    if (default_param)
    {
        range = 0.0;
        pen_distance = 0.0;
    }
}

void envTwistycool(bool optimal, bool default_param, app::SE3RigidBodyPlanning &setup, double &range, double &pen_distance)
{
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
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
    obj->setCostThreshold(base::Cost(600.0));
    setup.setOptimizationObjective(obj);
    setup.setup();

    pen_distance = 75.0;
    if (optimal)
        range = 75.0;
    else 
        range = 15.0;

    if (default_param)
    {
        range = 0.0;
        pen_distance = 0.0;
    }
}

void envTwistycooler(bool optimal, bool default_param, app::SE3RigidBodyPlanning &setup, double &range, double &pen_distance)
{
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
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
    obj->setCostThreshold(base::Cost(1100.0));
    setup.setOptimizationObjective(obj);
    setup.setup();

    pen_distance = 75.0;
    if (optimal)
        range = 75.0;
    else 
        range = 15.0;

    if (default_param)
    {
        range = 0.0;
        pen_distance = 0.0;
    }
}

void envAbstract(bool optimal, bool default_param, app::SE3RigidBodyPlanning &setup, double &range, double &pen_distance)
{
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
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
    obj->setCostThreshold(base::Cost(700.0));
    setup.setOptimizationObjective(obj);
    setup.setup();

    pen_distance = 30.0;
    if (optimal)
        range = 30.0;
    else 
        range = 15.0;

    if (default_param)
    {
        range = 0.0;
        pen_distance = 0.0;
    }
}

void envApartment(bool optimal, bool default_param, app::SE3RigidBodyPlanning &setup, double &range, double &pen_distance)
{
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
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
    obj->setCostThreshold(base::Cost(500.0));
    setup.setOptimizationObjective(obj);
    setup.setup();

    pen_distance = 10.0;
    if (optimal)
        range = 20.0;
    else 
        range = 10.0;

    if (default_param)
    {
        range = 0.0;
        pen_distance = 0.0;
    }
}

void envApartmentReverse(bool optimal, bool default_param, app::SE3RigidBodyPlanning &setup, double &range, double &pen_distance)
{
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
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
    obj->setCostThreshold(base::Cost(500.0));
    setup.setOptimizationObjective(obj);
    setup.setup();

    pen_distance = 10.0;
    if (optimal)
        range = 20.0;
    else 
        range = 10.0;

    if (default_param)
    {
        range = 0.0;
        pen_distance = 0.0;
    }
}

void envPipedream(bool optimal, bool default_param, app::SE3RigidBodyPlanning &setup, double &range, double &pen_distance)
{
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/spirelli_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/pipedream_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(54.59); 
    start->setY(22.21);
    start->setZ(-10.16);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(69.59); 
    goal->setY(22.21);
    goal->setZ(13.84);
    goal->rotation().setAxisAngle(1.0, 0.0, 0.0, 3.14159265359);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
    obj->setCostThreshold(base::Cost(0.0));
    setup.setOptimizationObjective(obj);
    setup.setup();

    pen_distance = 10.0;
    if (optimal)
        range = 20.0;
    else 
        range = 10.0;

    if (default_param)
    {
        range = 0.0;
        pen_distance = 0.0;
    }
}

void envSpirelli(bool optimal, bool default_param, app::SE3RigidBodyPlanning &setup, double &range, double &pen_distance)
{
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/spirelli_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/spirelli_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(2.59); 
    start->setY(23.21);
    start->setZ(-32.16);
    start->rotation().setAxisAngle(0.0, 0.0, 1.0, 3.14159265359);

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(64.59); 
    goal->setY(22.21);
    goal->setZ(1.84);
    goal->rotation().setAxisAngle(0.0, 1.0, 0.0, -1.57079632679);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
    obj->setCostThreshold(base::Cost(0.0));
    setup.setOptimizationObjective(obj);
    setup.setup();

    pen_distance = 10.0;
    if (optimal)
        range = 20.0;
    else 
        range = 10.0;

    if (default_param)
    {
        range = 0.0;
        pen_distance = 0.0;
    }
}

// Parse the command-line arguments
bool argParse(int argc, char** argv, double *runTimePtr, PlannerType *plannerPtr, bool &optimal, std::string &env, bool &default_param, unsigned int &run_cycle);

int main(int argc, char* argv[])
{
    // The parsed arguments
    bool optimal;
    bool default_param;
    double runTime;
    PlannerType plannerType;
    std::string env;
    unsigned int run_cycle;
    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, &runTime, &plannerType, optimal, env, default_param, run_cycle))
    {
        return -1;
    }

    ompl::app::SE3RigidBodyPlanning setup;
    double range, pen_distance;

    if (env == "Home")
        envHome(optimal, default_param, setup, range, pen_distance);
    else if (env == "Cubicles")
        envCubicles(optimal, default_param, setup, range, pen_distance);
    else if (env == "Twistycool")
        envTwistycool(optimal, default_param, setup, range, pen_distance);
    else if (env == "Twistycooler")
        envTwistycooler(optimal, default_param, setup, range, pen_distance);
    else if (env == "Abstract")
        envAbstract(optimal, default_param, setup, range, pen_distance);
    else if (env == "Apartment")
        envApartment(optimal, default_param, setup, range, pen_distance);
    else if (env == "ApartmentReverse")
        envApartmentReverse(optimal, default_param, setup, range, pen_distance);
    else if (env == "Pipedream")
        envPipedream(optimal, default_param, setup, range, pen_distance);
    else if (env == "Spirelli")
        envSpirelli(optimal, default_param, setup, range, pen_distance);
    
    auto si = setup.getSpaceInformation();
    auto svc = std::make_shared<app::ContactStateValidityChecker>(si, setup.getMotionModel(), 0.05, -0.05,
            setup.getGeometricComponentStateSpace(), setup.getGeometricStateExtractor(), setup.getGeometrySpecification());
    setup.setStateValidityChecker(svc);

    // Construct the optimal planner specified by our command line argument.
    // This helper function is simply a switch statement.
    ob::PlannerPtr planner = allocatePlanner(si, plannerType, range, pen_distance, svc);
    setup.setPlanner(planner);

    base::PlannerDataStorage pds;
    for (unsigned int i = 0; i < run_cycle; i++)
    {
        std::cout << "Iteration " << i + 1 << std::endl;

        setup.clear();
        setup.solve(runTime);

        // attempt to solve the problem, and print it to screen if a solution is found
        if (setup.haveSolutionPath())
        {
            std::ofstream out(boost::str(boost::format("se3path_%i.txt") % i).c_str());
            auto path = setup.getSolutionPath();
            path.printAsMatrix(out);
            out.close();
        }

        base::PlannerData pd(si);
        setup.getPlannerData(pd);
        pds.store(pd, boost::str(boost::format("se3pd_%i.txt") % i).c_str());

        /*
        base::PlannerData pd_bias_start(si);
        planner.get()->as<ompl::geometric::RRTBispace>()->getBiasData(pd_bias_start, true);
        pds.store(pd_bias_start, boost::str(boost::format("bias_start_pd_%i.txt") % i).c_str());

        base::PlannerData pd_bias_goal(si);
        planner.get()->as<ompl::geometric::RRTBispace>()->getBiasData(pd_bias_goal, false);
        pds.store(pd_bias_goal, boost::str(boost::format("bias_goal_pd_%i.txt") % i).c_str());
        */
    }
   
    return 0;
}

/** Parse the command line arguments into a string for an output file and the planner/optimization types */
bool argParse(int argc, char** argv, double* runTimePtr, PlannerType *plannerPtr, bool &optimal, std::string &env, bool &default_param, unsigned int &run_cycle)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("env,e", bpo::value<std::string>()->default_value("Home"), "(Optional) Specify the Benchmark environment, defaults to Home if not given. Valid options are Home, Cubicles, Twistycool, Twistycooler, Abstract, Apartment, Apartment-Reverse, Pipedream, Spirelli.")
        ("default_param,d", bpo::value<bool>()->default_value(false), "(Optional) Specify if the planner is set the default params.")
        ("runtime,t", bpo::value<double>()->default_value(1.0), "(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.")
        ("planner,p", bpo::value<std::string>()->default_value("RRTstar"), "(Optional) Specify the optimal planner to use, defaults to RRTstar if not given. Valid options are BFMTstar, BITstar, CForest, FMTstar, InformedRRTstar, PRMstar, RRTstar, SORRTstar, RRT, RRTConnect, BKPIECE1, LBKPIECE1, RRTBispace, RRTBispacestar, RRTBispaceD, RRTBispaceDstar, CellBispace, CellBispacestar, BiASE, BiASEstar, BiHSC, BiHSCstar, BiHSCCell, BiHSCCellstar, HSCASE, HSCASEstar, LazyRRT, SBL.")
        ("run_cycle,r", bpo::value<unsigned int>()->default_value(1), "(Optional) Specify the run cycles.");
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
    *runTimePtr = vm["runtime"].as<double>();

    // Sanity check
    if (*runTimePtr <= 0.0)
    {
        std::cout << "Invalid runtime." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    default_param = vm["default_param"].as<bool>();

    run_cycle = vm["run_cycle"].as<unsigned int>();

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
    else if (boost::iequals("Pipedream", envStr))
    {
        env = "Pipedream";
    }
    else if (boost::iequals("Spirelli", envStr))
    {
        env = "Spirelli";
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
        *plannerPtr = PLANNER_BFMTSTAR;
    }
    else if (boost::iequals("BITstar", plannerStr))
    {
        optimal = true;
        *plannerPtr = PLANNER_BITSTAR;
    }
    else if (boost::iequals("CForest", plannerStr))
    {
        optimal = true;
        *plannerPtr = PLANNER_CFOREST;
    }
    else if (boost::iequals("FMTstar", plannerStr))
    {
        optimal = true;
        *plannerPtr = PLANNER_FMTSTAR;
    }
    else if (boost::iequals("InformedRRTstar", plannerStr))
    {
        optimal = true;
        *plannerPtr = PLANNER_INF_RRTSTAR;
    }
    else if (boost::iequals("PRMstar", plannerStr))
    {
        optimal = true;
        *plannerPtr = PLANNER_PRMSTAR;
    }
    else if (boost::iequals("RRTstar", plannerStr))
    {
        optimal = true;
        *plannerPtr = PLANNER_RRTSTAR;
    }
    else if (boost::iequals("SORRTstar", plannerStr))
    {
        optimal = true;
        *plannerPtr = PLANNER_SORRTSTAR;
    }
    else if (boost::iequals("RRT", plannerStr))
    {
        *plannerPtr = PLANNER_RRT;
    }
    else if (boost::iequals("BKPIECE1", plannerStr))
    {
        *plannerPtr = PLANNER_BKPIECE1;
    }
    else if (boost::iequals("LBKPIECE1", plannerStr))
    {
        *plannerPtr = PLANNER_LBKPIECE1;
    }
    else if (boost::iequals("RRTBispace", plannerStr))
    {
        *plannerPtr = PLANNER_RRTBISPACE;
    }
    else if (boost::iequals("RRTBispacestar", plannerStr))
    {
        optimal = true;
        *plannerPtr = PLANNER_RRTBISPACESTAR;
    }
    else if (boost::iequals("RRTBispaceD", plannerStr))
    {
        *plannerPtr = PLANNER_RRTBISPACED;
    }
    else if (boost::iequals("RRTBispaceDstar", plannerStr))
    {
        optimal = true;
        *plannerPtr = PLANNER_RRTBISPACEDSTAR;
    }
    else if (boost::iequals("CellBispace", plannerStr))
    {
        *plannerPtr = PLANNER_CELLBISPACE;
    }
    else if (boost::iequals("CellBispacestar", plannerStr))
    {
        optimal = true;
        *plannerPtr = PLANNER_CELLBISPACESTAR;
    }
    else if (boost::iequals("BiASE", plannerStr))
    {
        *plannerPtr = PLANNER_BIASE;
    }
    else if (boost::iequals("BiASEstar", plannerStr))
    {
        optimal = true;
        *plannerPtr = PLANNER_BIASESTAR;
    }
    else if (boost::iequals("BiHSC", plannerStr))
    {
        *plannerPtr = PLANNER_BIHSC;
    }
    else if (boost::iequals("BiHSCstar", plannerStr))
    {
        optimal = true;
        *plannerPtr = PLANNER_BIHSCSTAR;
    }
    else if (boost::iequals("BiHSCCell", plannerStr))
    {
        *plannerPtr = PLANNER_BIHSCCELL;
    }
    else if (boost::iequals("BiHSCCellstar", plannerStr))
    {
        optimal = true;
        *plannerPtr = PLANNER_BIHSCCELLSTAR;
    }
    else if (boost::iequals("HSCASE", plannerStr))
    {
        *plannerPtr = PLANNER_HSCASE;
    }
    else if (boost::iequals("HSCASEstar", plannerStr))
    {
        optimal = true;
        *plannerPtr = PLANNER_HSCASESTAR;
    }
    else if (boost::iequals("RRTConnect", plannerStr))
    {
        *plannerPtr = PLANNER_RRTCONNECT;
    }
    else if (boost::iequals("LazyRRT", plannerStr))
    {
        *plannerPtr = PLANNER_LAZYRRT;
    }
    else if (boost::iequals("SBL", plannerStr))
    {
        *plannerPtr = PLANNER_SBL;
    }
    else
    {
        std::cout << "Invalid planner string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    return true;
}
