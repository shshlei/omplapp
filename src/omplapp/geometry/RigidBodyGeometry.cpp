/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan, Shi Shenglei */

#include "omplapp/geometry/RigidBodyGeometry.h"
#if OMPL_HAS_PQP
#include "omplapp/geometry/detail/PQPStateValidityChecker.h"
#endif
#include "omplapp/geometry/detail/FCLStateValidityChecker.h"

#include "omplapp/geometry/detail/assimpUtil.h"
#include "omplapp/geometry/detail/ContactStateValidityChecker.h"

boost::filesystem::path ompl::app::RigidBodyGeometry::findMeshFile(const std::string& fname)
{
    boost::filesystem::path path(fname);
    if (boost::filesystem::exists(path))
        return boost::filesystem::absolute(path);
    if (path.is_absolute())
        return {};
    for (const auto &dir : meshPath_)
    {
        boost::filesystem::path candidate(dir / path);
        if (boost::filesystem::exists(candidate))
            return boost::filesystem::absolute(candidate);
    }
    return {};
}

bool ompl::app::RigidBodyGeometry::setRobotMesh(const std::string &robot)
{
    importerRobot_.clear();
    computeGeometrySpecification();
    return addRobotMesh(robot);
}

bool ompl::app::RigidBodyGeometry::addRobotMesh(const std::string &robot)
{
    assert(!robot.empty());
    std::size_t p = importerRobot_.size();
    importerRobot_.resize(p + 1);
    importerRobot_[p] = std::make_shared<Assimp::Importer>();

    const boost::filesystem::path path = findMeshFile(robot);
    if (path.empty())
        OMPL_ERROR("File '%s' not found in mesh path.", robot.c_str());
    const aiScene* robotScene = importerRobot_[p]->ReadFile(path.string().c_str(),
                                                            aiProcess_GenNormals             |
                                                            aiProcess_Triangulate            |
                                                            aiProcess_JoinIdenticalVertices  |
                                                            aiProcess_SortByPType            |
                                                            aiProcess_OptimizeGraph);
    if (robotScene != nullptr)
    {
        if (!robotScene->HasMeshes())
        {
            OMPL_ERROR("There is no mesh specified in the indicated robot resource: %s", robot.c_str());
            importerRobot_.resize(p);
        }
    }
    else
    {
        OMPL_ERROR("Unable to load robot scene: %s", robot.c_str());
        importerRobot_.resize(p);
    }

    if (p < importerRobot_.size())
    {
        computeGeometrySpecification();
        return true;
    }
    return false;
}

//bool ompl::app::RigidBodyGeometry::addRobotMesh(const std::string &robot)
//{
//    assert(!robot.empty());
//    std::size_t p = importerRobot_.size();
//    importerRobot_.resize(p + 1);
//    importerRobot_[p] = std::make_shared<Assimp::Importer>();
//
//    const boost::filesystem::path path = findMeshFile(robot);
//    if (path.empty())
//        OMPL_ERROR("File '%s' not found in mesh path.", robot.c_str());
//
//    importerRobot_[p]->SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
//                          aiComponent_NORMALS | aiComponent_TANGENTS_AND_BITANGENTS | aiComponent_COLORS |
//                          aiComponent_TEXCOORDS | aiComponent_BONEWEIGHTS | aiComponent_ANIMATIONS |
//                          aiComponent_TEXTURES | aiComponent_LIGHTS | aiComponent_CAMERAS |
//                          aiComponent_MATERIALS);
//
//    const aiScene* robotScene = importerRobot_[p]->ReadFile(path.string().c_str(),
//                                                            aiProcess_GenNormals             |
//                                                            aiProcess_Triangulate            |
//                                                            aiProcess_JoinIdenticalVertices  |
//                                                            aiProcess_SortByPType            |
//                                                            aiProcess_RemoveComponent); 
//
//    if (robotScene != nullptr)
//    {
//        if (!robotScene->HasMeshes())
//        {
//            OMPL_ERROR("There is no mesh specified in the indicated robot resource: %s", robot.c_str());
//            importerRobot_.resize(p);
//        }
//    }
//    else
//    {
//        OMPL_ERROR("Unable to load robot scene: %s", robot.c_str());
//        importerRobot_.resize(p);
//    }
//
//    if (p < importerRobot_.size())
//    {
////        robotScene->mRootNode->mTransformation = aiMatrix4x4();
//
//        importerRobot_[p]->ApplyPostProcessing(aiProcess_OptimizeMeshes | aiProcess_OptimizeGraph);
//
//        computeGeometrySpecification();
//        return true;
//    }
//
//    return false;
//}

bool ompl::app::RigidBodyGeometry::setEnvironmentMesh(const std::string &env)
{
    importerEnv_.clear();
    computeGeometrySpecification();
    return addEnvironmentMesh(env);
}

bool ompl::app::RigidBodyGeometry::addEnvironmentMesh(const std::string &env)
{
    assert(!env.empty());
    std::size_t p = importerEnv_.size();
    importerEnv_.resize(p + 1);
    importerEnv_[p] = std::make_shared<Assimp::Importer>();

    const boost::filesystem::path path = findMeshFile(env);
    if (path.empty())
        OMPL_ERROR("File '%s' not found in mesh path.", env.c_str());
    const aiScene* envScene = importerEnv_[p]->ReadFile(path.string().c_str(),
                                                        aiProcess_GenNormals             |
                                                        aiProcess_Triangulate            |
                                                        aiProcess_JoinIdenticalVertices  |
                                                        aiProcess_SortByPType            |
                                                        aiProcess_OptimizeGraph);

    if (envScene != nullptr)
    {
        if (!envScene->HasMeshes())
        {
            OMPL_ERROR("There is no mesh specified in the indicated environment resource: %s", env.c_str());
            importerEnv_.resize(p);
        }
    }
    else
    {
        OMPL_ERROR("Unable to load environment scene: %s", env.c_str());
        importerEnv_.resize(p);
    }

    if (p < importerEnv_.size())
    {
        computeGeometrySpecification();
        return true;
    }

    return false;
}

//bool ompl::app::RigidBodyGeometry::addEnvironmentMesh(const std::string &env)
//{
//    assert(!env.empty());
//    std::size_t p = importerEnv_.size();
//    importerEnv_.resize(p + 1);
//    importerEnv_[p] = std::make_shared<Assimp::Importer>();
//
//    importerEnv_[p]->SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
//                          aiComponent_NORMALS | aiComponent_TANGENTS_AND_BITANGENTS | aiComponent_COLORS |
//                          aiComponent_TEXCOORDS | aiComponent_BONEWEIGHTS | aiComponent_ANIMATIONS |
//                          aiComponent_TEXTURES | aiComponent_LIGHTS | aiComponent_CAMERAS |
//                          aiComponent_MATERIALS);
//
//    const boost::filesystem::path path = findMeshFile(env);
//    if (path.empty())
//        OMPL_ERROR("File '%s' not found in mesh path.", env.c_str());
//
//    const aiScene* envScene = importerEnv_[p]->ReadFile(path.string().c_str(),
//                                                        aiProcess_GenNormals             |
//                                                        aiProcess_Triangulate            |
//                                                        aiProcess_JoinIdenticalVertices  |
//                                                        aiProcess_SortByPType            |
//                                                        aiProcess_RemoveComponent);
//
//    if (envScene != nullptr)
//    {
//        if (!envScene->HasMeshes())
//        {
//            OMPL_ERROR("There is no mesh specified in the indicated environment resource: %s", env.c_str());
//            importerEnv_.resize(p);
//        }
//    }
//    else
//    {
//        OMPL_ERROR("Unable to load environment scene: %s", env.c_str());
//        importerEnv_.resize(p);
//    }
//
//    if (p < importerEnv_.size())
//    {
//        importerEnv_[p]->ApplyPostProcessing(aiProcess_OptimizeMeshes | aiProcess_OptimizeGraph);
//        computeGeometrySpecification();
//        return true;
//    }
//    
//    return false;
//}

ompl::base::RealVectorBounds ompl::app::RigidBodyGeometry::inferEnvironmentBounds() const
{
    base::RealVectorBounds bounds(3);

    for (const auto & i : importerEnv_)
    {
        std::vector<aiVector3D> vertices;
        scene::extractVertices(i->GetScene(), vertices);
        scene::inferBounds(bounds, vertices, factor_, add_);
    }

    if (mtype_ == Motion_2D)
    {
        bounds.low.resize(2);
        bounds.high.resize(2);
    }

    return bounds;
}

const ompl::app::GeometrySpecification& ompl::app::RigidBodyGeometry::getGeometrySpecification() const
{
    return geom_;
}

void ompl::app::RigidBodyGeometry::computeGeometrySpecification()
{
    validitySvc_.reset();
    geom_.obstacles.clear();
    geom_.obstaclesShift.clear();
    geom_.robot.clear();
    geom_.robotShift.clear();

    for (auto & i : importerEnv_)
        geom_.obstacles.push_back(i->GetScene());

    for (unsigned int i = 0 ; i < importerRobot_.size() ; ++i)
    {
        geom_.robot.push_back(importerRobot_[i]->GetScene());
        aiVector3D c = getRobotCenter(i);
        if (mtype_ == Motion_2D)
            c[2] = 0.0;
        geom_.robotShift.push_back(c);
    }
}

aiVector3D ompl::app::RigidBodyGeometry::getRobotCenter(unsigned int robotIndex) const
{
    aiVector3D s(0.0, 0.0, 0.0);
    if (robotIndex >= importerRobot_.size())
        throw Exception("Robot " + std::to_string(robotIndex) + " not found.");

    scene::sceneCenter(importerRobot_[robotIndex]->GetScene(), s);
    return s;
}

void ompl::app::RigidBodyGeometry::setCollisionChecker(CollisionChecker cchecker)
{
    if (cchecker != cchecker_)
    {
        cchecker_ = cchecker;
        if (validitySvc_)
        {
            validitySvc_.reset();
        }

        assert(!validitySvc_);
    }
}

const ompl::base::StateValidityCheckerPtr& ompl::app::RigidBodyGeometry::allocStateValidityChecker(const base::SpaceInformationPtr &si,
        const base::StateSpacePtr &gspace, const GeometricStateExtractor &se, bool selfCollision)
{
    if (validitySvc_)
        return validitySvc_;

    GeometrySpecification geom = getGeometrySpecification();

    switch (cchecker_)
    {
#if OMPL_HAS_PQP
        case PQP:
            if (mtype_ == Motion_2D)
                validitySvc_ = std::make_shared<PQPStateValidityChecker<Motion_2D>>(si, geom, se, selfCollision);
            else
                validitySvc_ = std::make_shared<PQPStateValidityChecker<Motion_3D>>(si, geom, se, selfCollision);
            break;
#endif
        case FCL:
            if (mtype_ == Motion_2D)
                validitySvc_ = std::make_shared<FCLStateValidityChecker<Motion_2D>>(si, geom, se, selfCollision);
            else
                validitySvc_ = std::make_shared<FCLStateValidityChecker<Motion_3D>>(si, geom, se, selfCollision);
            break;
        case BULLET:
            validitySvc_ = std::make_shared<ContactStateValidityChecker>(si, mtype_, 0.05, -0.05, gspace, se, geom);
            break;

        default:
            OMPL_ERROR("Unexpected collision checker type (%d) encountered", cchecker_);
    };

    return validitySvc_;
}
