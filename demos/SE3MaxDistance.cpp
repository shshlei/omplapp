#include <omplapp/config.h>

#include "omplapp/geometry/RigidBodyGeometry.h"
#include "omplapp/geometry/detail/ContactStateValidityChecker.h"

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/State.h>

#include <ompl/util/Console.h>

#include <vector>
#include <iostream>

using namespace ompl;
namespace ob = ompl::base;

const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int /*index*/)
{
    return state;
}

int main(/*int argc, char* argv[]*/)
{
    app::RigidBodyGeometry rbg(app::Motion_3D);

//    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_robot.dae";
//    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_env.dae";

//    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Home_robot.dae";
//    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Home_env.dae";

//    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_robot.dae";
//    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_env.dae";

    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycooler_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycooler_env.dae";

    rbg.setRobotMesh(robot_fname);
    rbg.setEnvironmentMesh(env_fname);

    auto space(std::make_shared<ob::SE3StateSpace>());
    space->setBounds(rbg.inferEnvironmentBounds());

    auto si(std::make_shared<ob::SpaceInformation>(space));

    double ccm = 150.0;
    auto svc = std::make_shared<app::ContactStateValidityChecker>(si, app::Motion_3D, 0.05, ccm, space,
                getGeometricComponentStateInternal, rbg.getGeometrySpecification());

    si->setStateValidityChecker(svc);

    si->setStateValidityCheckingResolution(0.01);

    si->setup();

    si->printProperties();
    std::cout << std::endl;
    si->printSettings();

    double maxdist = 0.0;
    double mindist = ccm;
    double avgdist = 0.0;

    ompl::base::State *state = si->allocState();
    auto sampler = si->allocStateSampler();

    unsigned int count = 0;
    unsigned int validc = 0;
    while (count < 1.e3)
    {
        sampler->sampleUniform(state);

        double dist = svc->clearance(state);

        if (dist > 0.05 && dist < ccm)
        {
            avgdist += dist;
            validc++;
        }

        if (dist > maxdist)
        {
            maxdist = dist;

            std::cout << "max distance " << maxdist << std::endl;

            if (maxdist >= ccm)
                break;
        }

        if (dist > 0.05 && dist < mindist)
        {
            mindist = dist;
            std::cout << "min distance " << mindist << std::endl;
        }

        count++;
    }

    std::cout << "max distance " << maxdist << std::endl;
    std::cout << "min distance " << mindist << std::endl;
    std::cout << "avg distance " << avgdist / (double) validc << std::endl;

    si->freeState(state);
    sampler.reset();

    return 0;
}

