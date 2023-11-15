#include <omplapp/config.h>

#include <omplapp/geometry/RigidBodyGeometry.h>
#include <omplapp/geometry/detail/ContactStateValidityChecker.h>

#include <ompl/base/State.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/SafetyCertificate.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

#include <ompl/util/Console.h>
#include <ompl/util/Time.h>

#include <boost/program_options.hpp>
#include <boost/math/constants/constants.hpp>

#include <Eigen/Geometry>

#include <vector>
#include <iostream>
#include <fstream>

using namespace ompl;
namespace ob = ompl::base;

const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int /*index*/)
{
    return state;
}

namespace Test
{
    using namespace std;

    class LSC 
    {
    public:

        using SafetyCertificateChecker = std::function<bool(const ompl::base::State *state, const ompl::base::SafetyCertificate *sc, std::vector<double> &dist)>;

        using CollisionCertificateChecker = std::function<bool(const ompl::base::State *state, const std::vector<ompl::base::SafetyCertificate *> &ocv)>;

        using DistanceCertificate = std::function<std::vector<double>(const base::State *a, const base::State *b)>;

        /** \brief The constructor needs the instance of the space information */
        LSC(const ompl::base::SpaceInformationPtr &si, const CollisionCertificateChecker &collisionCertificateChecker, const DistanceCertificate &distanceCertificate) :
            si_(std::move(si)), collisionCertificateChecker_(collisionCertificateChecker), distanceCertificate_(distanceCertificate)
        {
        }

        void setup()
        {
            onn_.reset(new ompl::NearestNeighborsGNATNoThreadSafety<ompl::base::SafetyCertificate *>());
            onn_->setDistanceFunction([this](const ompl::base::SafetyCertificate *a, const ompl::base::SafetyCertificate *b)
                                             {
                                                 return distanceFunction(a, b);
                                             });
        }

        ompl::base::State* getState(const ompl::base::State *state)
        {
            ompl::base::SafetyCertificate *sc = new ompl::base::SafetyCertificate(si_);
            si_->copyState(sc->state, state);

            ompl::base::SafetyCertificate *nsc = onn_->nearest(sc);

            si_->freeState(sc->state);
            delete sc->contact;
            delete sc;

            return nsc->state;
        }

        ~LSC()
        {
            std::vector<ompl::base::SafetyCertificate *> safetycertificates;

            if (onn_ != nullptr)
            {
                onn_->list(safetycertificates);
                for (auto &safetycertificate : safetycertificates)
                {
                    if (safetycertificate->state != nullptr)
                        si_->freeState(safetycertificate->state);
                    if (safetycertificate->contact != nullptr)
                        delete safetycertificate->contact;
                    delete safetycertificate;
                }
                safetycertificates.clear();
            }
        }
    
        bool isValid(const ompl::base::State *state, bool &lazy)
        {
            lazy = false;

            if (si_->satisfiesBounds(state))
            {
                ompl::base::SafetyCertificate *sc = new ompl::base::SafetyCertificate(si_);
                si_->copyState(sc->state, state);

                if (onn_->size() > 0)
                {
                    std::vector<ompl::base::SafetyCertificate *> nsc;

                    time::point start = time::now();
                    onn_->nearestK(sc, 5, nsc);
                    nbtime_ += time::seconds(time::now() - start);

                    start = time::now();
                    bool osc = collisionCertificateChecker_(sc->state, nsc);
                    time_ += time::seconds(time::now() - start);

                    if (osc)
                    {	
                        si_->freeState(sc->state);
                        delete sc->contact;
                        delete sc;

                        lazy = true;
                        return false;
                    }
                }

                time::point start = time::now();
                double dist = 0.0;
                bool valid = si_->isValid(sc->state, *sc->contact, dist);
                normaltime_ += time::seconds(time::now() - start);

                if (!valid)
                {
                    onn_->add(sc);
                }
                else 
                {
                    si_->freeState(sc->state);
                    delete sc->contact;
                    delete sc;
                }

                return valid;
            }
            else 
                return false;
        }

    public:

        double distanceFunction(const ompl::base::SafetyCertificate *a, const ompl::base::SafetyCertificate *b) const
        {
            std::vector<double> cdist = distanceCertificate_(a->state, b->state);
            double dist = std::accumulate(cdist.begin(), cdist.end(), 0.0);
            return dist;
        }
        
        ompl::base::SpaceInformationPtr si_;

        std::shared_ptr<ompl::NearestNeighbors<ompl::base::SafetyCertificate *>> onn_;

        CollisionCertificateChecker collisionCertificateChecker_;

        DistanceCertificate distanceCertificate_;

        double time_{0}, nbtime_{0}, normaltime_{0};
    };
}

void envHome(app::RigidBodyGeometry &rbg)
{
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Home_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Home_env.dae";
    rbg.setRobotMesh(robot_fname);
    rbg.setEnvironmentMesh(env_fname);
}

void envCubicles(app::RigidBodyGeometry &rbg)
{
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_env.dae";
    rbg.setRobotMesh(robot_fname);
    rbg.setEnvironmentMesh(env_fname);
}

void envTwistycool(app::RigidBodyGeometry &rbg)
{
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_env.dae";
    rbg.setRobotMesh(robot_fname);
    rbg.setEnvironmentMesh(env_fname);
}

void envTwistycooler(app::RigidBodyGeometry &rbg)
{
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycooler_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycooler_env.dae";
    rbg.setRobotMesh(robot_fname);
    rbg.setEnvironmentMesh(env_fname);
}

void envAbstract(app::RigidBodyGeometry &rbg)
{
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Abstract_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Abstract_env.dae";
    rbg.setRobotMesh(robot_fname);
    rbg.setEnvironmentMesh(env_fname);
}

void envApartment(app::RigidBodyGeometry &rbg)
{
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_env.dae";
    rbg.setRobotMesh(robot_fname);
    rbg.setEnvironmentMesh(env_fname);
}

// Parse the command-line arguments
bool argParse(int argc, char** argv, std::string &env);

int main(int argc, char* argv[])
{
    // The parsed arguments
    std::string env;

    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, env))
    {
        return -1;
    }

    app::RigidBodyGeometry rbg(app::Motion_3D);

    if (env == "Home")
        envHome(rbg);
    else if (env == "Cubicles")
        envCubicles(rbg);
    else if (env == "Twistycool")
        envTwistycool(rbg);
    else if (env == "Twistycooler")
        envTwistycooler(rbg);
    else if (env == "Abstract")
        envAbstract(rbg);
    else if (env == "Apartment")
        envApartment(rbg);

    auto space(std::make_shared<ob::SE3StateSpace>());
    space->setBounds(rbg.inferEnvironmentBounds());

    auto si(std::make_shared<ob::SpaceInformation>(space));

    auto svc = std::make_shared<app::ContactStateValidityChecker>(si, app::Motion_3D, 0.05, -0.05, space, getGeometricComponentStateInternal, rbg.getGeometrySpecification());

    si->setStateValidityChecker(svc);

    si->setStateValidityCheckingResolution(0.01);

    si->setup();

    ompl::base::StateSamplerPtr sampler = si->allocStateSampler();

    auto lsc = std::make_shared<Test::LSC>(si, svc->getCollisionCertificateChecker(), svc->getDistanceCertificate());
    lsc->setup();

    bool failed = false;
    bool lazy = false;

    unsigned int count = 0;
    unsigned int lsccount = 0; //invalid invalid lazy

    unsigned int validcount = 0;
    unsigned int truefailed = 0;

    unsigned int iter = 1.e5;

    ompl::base::State *state = si->allocState();
    ompl::base::State *temps = si->allocState();

    double svctime = 0;
    double lsctime = 0;

    std::ofstream ofs(env + "_CheckTime.txt");

    ofs << 0 << " " << 0 << " " << 0 << std::endl;

    while (count < iter)
    {
        sampler->sampleUniform(state);

        if (failed)
        {
            si->copyState(state, temps);
            failed = false;
        }

        time::point start = time::now();
        bool svcvalid = svc->isValid(state);
        svctime += time::seconds(time::now() - start);

        start = time::now();
        bool lscvalid = lsc->isValid(state, lazy);
        lsctime += time::seconds(time::now() - start);

        if (svcvalid)
            validcount++;

        if (!svcvalid && !lscvalid && lazy) //pass
            lsccount++;
        else if (svcvalid && !lscvalid) //
        {
            truefailed++;
//            si->copyState(temps, state);
//            failed = true;
        }

        count++;

        if (count % 10 == 0)
        {
            ofs << count << " " << svctime << " " << lsctime << std::endl;
        }
    }

    ofs.close();

    si->freeState(state);
    si->freeState(temps);

    OMPL_INFORM("valid number %u, invalid number %u", validcount, iter - validcount);

    OMPL_INFORM("svctime %.5f, lsctime %.5f, lscchecktime %.5f, lscnbtime %.5f, lscnormaltime %.5f", svctime, lsctime, lsc->time_, lsc->nbtime_, lsc->normaltime_);

    OMPL_INFORM("%u certificates ", lsc->onn_->size());

    OMPL_INFORM("lsccount %u, truefailed %u", lsccount, truefailed);

    return 0;
}

bool argParse(int argc, char** argv, std::string &env)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("env,e", bpo::value<std::string>()->default_value("Home"), "(Optional) Specify the Benchmark environment, defaults to Home if not given. Valid options are Home, Cubicles, Twistycool, Twistycooler, Abstract, Apartment.");
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);

    // Check if the help flag has been given:
    if (vm.count("help") != 0u)
    {
        std::cout << desc << std::endl;
        return false;
    }

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
    else
    {
        std::cout << "Invalid environment string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    return true;
}
