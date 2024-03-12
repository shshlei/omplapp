#include <omplapp/config.h>

#include <omplapp/geometry/detail/Box2dStateValidityChecker.h>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/SafetyCertificate.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/datastructures/Grid.h>

#include <ompl/util/Console.h>
#include <ompl/util/Time.h>

#include <boost/program_options.hpp>
#include <boost/math/constants/constants.hpp>

#include <vector>
#include <iostream>
#include <fstream>

using namespace ompl;
namespace ob = ompl::base;

const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int /*index*/)
{
    return state;
}

const base::State* getGeometricComponentStateInternal2(const base::State* state, unsigned int index)
{
    assert(index == 0 || index == 1);
    const auto* st = state->as<base::CompoundStateSpace::StateType>()->as<base::SE2StateSpace::StateType>(index);
    return static_cast<const base::State*>(st);
}

namespace Test
{
    using namespace std;

    template <typename T>
    class SimpleGrid
    {
    public:
        struct CellData
        {
            CellData() = default;
            ~CellData() = default;
            std::vector<T *> datas;
        };

        using SGrid = Grid<CellData *>;
        using Cell = typename SGrid::Cell;
        using Coord = typename SGrid::Coord;

        SimpleGrid() : grid_(0), size_(0)
        {
        }

        ~SimpleGrid()
        {
            freeMemory();
        }

        void setDimension(unsigned int dim)
        {
            grid_.setDimension(dim);
        }

        unsigned int add(T *data, const Coord &coord)
        {
            Cell *cell = grid_.getCell(coord);
            unsigned int created = 0;
            if (cell)
                cell->data->datas.push_back(data);
            else
            {
                cell = grid_.createCell(coord);
                cell->data = new CellData();
                cell->data->datas.push_back(data);
                grid_.add(cell);
            }
            size_++;
            return created;
        }

        Cell *getCell(const Coord &coord)
        {
            return grid_.getCell(coord);
        }

        const Cell *getCell(const Coord &coord) const
        {
            return grid_.getCell(coord);
        }

        const SGrid &getGrid() const
        {
            return grid_;
        }

        SGrid &getGrid()
        {
            return grid_;
        }

        void freeMemory()
        {
            for (auto it = grid_.begin(); it != grid_.end(); ++it)
                freeCellData(it->second->data);
            grid_.clear();
        }

        std::size_t size() const
        {
            return size_;
        }

    private:
        void freeCellData(CellData *cdata)
        {
            delete cdata;
        }

        SGrid grid_;
        std::size_t size_{0};
    };

    class LSC 
    {
    public:

        using CollisionCertificateChecker = std::function<bool(const ompl::base::State *state, const std::vector<ompl::base::SafetyCertificate *> &ocv)>;

        using SimpleGridData = SimpleGrid<ompl::base::SafetyCertificate>;
        using Cell = SimpleGridData::Cell;
        using Coord = SimpleGridData::Coord;

        LSC(const ompl::base::SpaceInformationPtr &si, const CollisionCertificateChecker &collisionCertificateChecker) :
            si_(std::move(si)), collisionCertificateChecker_(collisionCertificateChecker)
        {
            projectionEvaluator_ = si->getStateSpace()->getDefaultProjection();
            //projectionEvaluator_->mulCellSizes(2.0);
        }

        void setup()
        {
            onn_.reset(new SimpleGridData());
            onn_->setDimension(2);
        }

        ~LSC()
        {
            std::vector<Cell *> cells;
            onn_->getGrid().getCells(cells);
            for (auto & cell : cells)
            {
                for (auto &safetycertificate : cell->data->datas)
                {
                    if (safetycertificate->state != nullptr)
                        si_->freeState(safetycertificate->state);
                    if (safetycertificate->contact != nullptr)
                        delete safetycertificate->contact;
                    delete safetycertificate;
                }
            }
        }
    
        bool isValid(const ompl::base::State *state, bool &lazy)
        {
            lazy = false;
            Coord xcoord(projectionEvaluator_->getDimension());
            projectionEvaluator_->computeCoordinates(state, xcoord);
            Cell *cell = onn_->getCell(xcoord);
            if (cell)
            {
                bool osc = collisionCertificateChecker_(state, cell->data->datas);
                if (osc)
                {
                    total_++;
                    sc_++;
                    lazy = true;
                    return false;
                }
            }

            ompl::base::SafetyCertificate *sc = new ompl::base::SafetyCertificate(si_);
            double dist = 0.0;
            bool valid = si_->isValid(state, *sc->contact, dist);

            if (!valid)
            {
                total_++;
                si_->copyState(sc->state, state);
                onn_->add(sc, xcoord);
            }
            else 
            {
                si_->freeState(sc->state);
                delete sc->contact;
                delete sc;
            }
            return valid;
        }

    private:

        ompl::base::SpaceInformationPtr si_;

        base::ProjectionEvaluatorPtr projectionEvaluator_;

        CollisionCertificateChecker collisionCertificateChecker_;

    public:

        std::shared_ptr<SimpleGridData> onn_;

        int total_{0}, sc_{0};
    };
}

bool argParse(int argc, char** argv, std::string &env, int &robot, std::string &spaceType);

int main(int argc, char* argv[])
{
    // The parsed arguments
    std::string env;
    int robot;
    std::string spaceType;
    // Parse the arguments, returns true if successful, false otherwise
    if (!argParse(argc, argv, env, robot, spaceType))
        return -1;

    ob::StateSpacePtr space;
    ob::StateSpacePtr gspace;
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0.0);
    bounds.setHigh(1.0);
    if (spaceType == "RealVector2")
    {
        if (robot == 1)
        {
            auto space1(std::make_shared<ob::RealVectorStateSpace>(2));
            space1->setBounds(bounds);
            space = space1;
            gspace = space;
        }
        else if (robot == 2)
        {
            space = std::make_shared<ob::CompoundStateSpace>();

            auto space1(std::make_shared<ob::RealVectorStateSpace>(2));
            space1->setBounds(bounds);
            auto space2(std::make_shared<ob::RealVectorStateSpace>(2));
            space2->setBounds(bounds);

            space->as<ob::CompoundStateSpace>()->addSubspace(space1, 1.0);
            space->as<ob::CompoundStateSpace>()->addSubspace(space2, 1.0);
            gspace = space1;
        }

    }
    else if (spaceType == "SE2")
    {
        if (robot == 1)
        {
            auto space1(std::make_shared<ob::SE2StateSpace>());
            space1->setBounds(bounds);
            space = space1;
            gspace = space;
        }
        else if (robot == 2)
        {
            space = std::make_shared<ob::CompoundStateSpace>();

            auto space1(std::make_shared<ob::SE2StateSpace>());
            space1->setBounds(bounds);
            auto space2(std::make_shared<ob::SE2StateSpace>());
            space2->setBounds(bounds);

            space->as<ob::CompoundStateSpace>()->addSubspace(space1, 1.0);
            space->as<ob::CompoundStateSpace>()->addSubspace(space2, 1.0);
            gspace = space1;
        }
    } 

    auto si(std::make_shared<ob::SpaceInformation>(space));
    auto svc = std::make_shared<app::Box2dStateValidityChecker>(si, gspace,
                robot == 1 ? getGeometricComponentStateInternal : getGeometricComponentStateInternal2);
    svc->setEnvironmentFile(std::string(OMPLAPP_RESOURCE_DIR) + "/" + env);

    int vcount = 3;
    Eigen::Vector2d vecs[vcount];
//    double x[3] = {5.0 * 0.0086579571682871, -5.0 * 0.02506512753291945, 5.0 * 0.012808997914287135};
//    double y[3] = {5.0 * 0.028723505664735693, 5.0 * 0.01648451945791818, -5.0 * 0.027128021904145316}
    double x[3] = {0.0086579571682871, -0.02506512753291945, 0.012808997914287135};
    double y[3] = {0.028723505664735693, 0.01648451945791818, -0.027128021904145316};
    for (int i = 0; i < vcount; i++)
        vecs[i] = Eigen::Vector2d(x[i], y[i]);

    auto polygon(std::make_shared<app::geometries::Polygon>());
    polygon->set(vecs, vcount);
    svc->addRobotShape(polygon);
    if (robot == 2)
        svc->addRobotShape(polygon);

    si->setStateValidityChecker(svc);
    si->setStateValidityCheckingResolution(0.01);
    si->setup();

    ompl::base::StateSamplerPtr sampler = si->allocStateSampler();

    auto lsc = std::make_shared<Test::LSC>(si, svc->getCollisionCertificateChecker());
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

    double svctime = 0.0;
    double lsctime = 0.0;

    std::ofstream ofs(robot == 1 ? env + "_CheckTime.txt" : "Multi" + env + "_CheckTime.txt");
    std::ofstream ofs2("sc_pure_sampling.txt");
    ofs << 0 << " " << 0 << " " << 0 << std::endl;
    ofs2 << 0 << " " << 0 << std::endl;
    while (count < iter)
    {
        sampler->sampleUniform(state);

        if (failed)
        {
            si->copyState(state, temps);
            failed = false;
        }

        time::point start = time::now();
        bool svcvalid = si->isValid(state);
        svctime += time::seconds(time::now() - start);

        start = time::now();
        bool lscvalid = lsc->isValid(state, lazy);
        lsctime += time::seconds(time::now() - start);

        if (svcvalid)
            validcount++;

        if (!svcvalid && !lscvalid && lazy) //pass
            lsccount++;
        else if (svcvalid && !lscvalid) //todo
        {
            truefailed++;
            //si->copyState(temps, state);
            //failed = true;
        }

        count++;
        if (count % 10 == 0)
        {
            ofs << count << " " << svctime << " " << lsctime << std::endl;
        }
        ofs2 << lsc->total_ << " " << lsc->sc_  << std::endl;
    }

    ofs.close();
    ofs2.close();

    si->freeState(state);
    si->freeState(temps);

    OMPL_INFORM("valid number %u, invalid number %u", validcount, iter - validcount);

    OMPL_INFORM("svctime %.5f, lsctime %.5f", svctime, lsctime);

    OMPL_INFORM("%u certificates ", lsc->onn_->size());

    OMPL_INFORM("lsccount %u, truefailed %u", lsccount, truefailed);
    std::cout << std::endl;

    return 0;
}

bool argParse(int argc, char** argv, std::string &env, int &robot, std::string &spaceType)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("env,e", bpo::value<std::string>()->default_value("random_scenarios.ply"), "(Optional) Specify the polygon and circle environment, defaults to random_scenarios.ply if not given.")
        ("robot,r", bpo::value<int>()->default_value(1), "(Optional) Specify the robot number, defaults to 1 if not given.")
        ("spacetype,s", bpo::value<std::string>()->default_value("RealVector2"), "(Optional) Specify the planning space type, default to RealVector2 if not given. Valid options are RealVector2, SE2.");
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
    robot = vm["robot"].as<int>();

    spaceType = vm["spacetype"].as<std::string>();
    if (boost::iequals("RealVector2", spaceType))
        spaceType = "RealVector2";
    else if (boost::iequals("SE2", spaceType))
        spaceType = "SE2";

    return true;
}
