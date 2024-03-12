find_package(HDF5 REQUIRED COMPONENTS CXX HL)
include_directories(${HDF5_INCLUDE_DIRS})

set(OMPL_BUILD_DEMOS ON)
if (OMPL_BUILD_DEMOS)
    set(OMPLAPP_DEMO_LIBRARIES ${OMPLAPP_LIBRARIES} ompl ompl_app_base)

    macro(add_omplapp_demo demo_name)
      add_executable(${ARGV})
      target_link_libraries(${demo_name} ${OMPLAPP_DEMO_LIBRARIES}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
        ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endmacro(add_omplapp_demo)

    add_omplapp_demo(demo_CurvesCarPlanning CarControl/CurvesCarPlanning.cpp)
    add_omplapp_demo(demo_KinematicCarPlanning CarControl/KinematicCarPlanning.cpp CarControl/KinematicCarSetup.cpp)

#    add_omplapp_demo(demo_SE3RigidBodyPlanningBenchmark SE3RigidBodyPlanning/SE3RigidBodyPlanningBenchmark.cpp)
#    add_omplapp_demo(demo_SE3RigidBodyPlanning SE3RigidBodyPlanning/SE3RigidBodyPlanning.cpp)
#    add_omplapp_demo(demo_SE3RigidBodyPlanningWithOptimization SE3RigidBodyPlanning/SE3RigidBodyPlanningWithOptimization.cpp)
#    add_omplapp_demo(demo_SE2RigidBodyPlanning SE2RigidBodyPlanning/SE2RigidBodyPlanning.cpp)
#    add_omplapp_demo(demo_KinematicCarPlanning SE2RigidBodyPlanning/KinematicCarPlanning.cpp)
#    add_omplapp_demo(demo_DynamicCarPlanning SE2RigidBodyPlanning/DynamicCarPlanning.cpp)
#    add_omplapp_demo(demo_BlimpPlanning SE3RigidBodyPlanning/BlimpPlanning.cpp)
#    add_omplapp_demo(demo_QuadrotorPlanning SE3RigidBodyPlanning/QuadrotorPlanning.cpp)
#    add_omplapp_demo(demo_SE2MultiRigidBodyPlanning SE2RigidBodyPlanning/SE2MultiRigidBodyPlanning.cpp)
#    add_omplapp_demo(demo_SE3MultiRigidBodyPlanning SE3RigidBodyPlanning/SE3MultiRigidBodyPlanning.cpp)
#    add_omplapp_demo(demo_CollisionCheckers SE3RigidBodyPlanning/CollisionCheckers.cpp)
#    add_omplapp_demo(demo_AnytimePathShortening AnytimePathShorteningDemo.cpp)

    find_package(adolc REQUIRED)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(ipopt REQUIRED IMPORTED_TARGET ipopt)

    add_omplapp_demo(demo_ParkingPureSampling Parking/ParkingPureSampling.cpp)

    add_omplapp_demo(demo_ParkingAC Parking/ParkingAC.cpp)
    target_link_libraries(demo_ParkingAC adolc PkgConfig::ipopt)

    add_omplapp_demo(demo_ParkingMJ2 Parking/ParkingMJ2.cpp)
    target_link_libraries(demo_ParkingMJ2 adolc PkgConfig::ipopt)
    #add_omplapp_demo(demo_ParkingActivePoints Parking/ParkingActivePoints.cpp)

    add_omplapp_demo(demo_ParkingDualDistance Parking/ParkingDualDistance.cpp)
    target_link_libraries(demo_ParkingDualDistance adolc PkgConfig::ipopt)

    add_omplapp_demo(demo_ParkingDualMJ2 Parking/ParkingDualMJ2.cpp)
    target_link_libraries(demo_ParkingDualMJ2 adolc PkgConfig::ipopt)

    add_omplapp_demo(demo_ParkingMJ2Initial Parking/ParkingMJ2Initial.cpp)
    target_link_libraries(demo_ParkingMJ2Initial adolc PkgConfig::ipopt)

    add_omplapp_demo(demo_ParkingMJ2InitialWoDynamics Parking/ParkingMJ2InitialWoDynamics.cpp)
    target_link_libraries(demo_ParkingMJ2InitialWoDynamics adolc PkgConfig::ipopt)

    add_omplapp_demo(demo_ParkingCompare Parking/ParkingCompare.cpp)
    target_link_libraries(demo_ParkingCompare adolc PkgConfig::ipopt)

    add_omplapp_demo(demo_DifferentialDriving DifferentialDriving/DifferentialDriving.cpp)
    target_link_libraries(demo_DifferentialDriving adolc PkgConfig::ipopt)

    add_omplapp_demo(demo_DrivingWoObstacles RandomScenariosDriving/DrivingWoObstacles.cpp)
    target_link_libraries(demo_DrivingWoObstacles adolc PkgConfig::ipopt)
    add_omplapp_demo(demo_DrivingBoxAp RandomScenariosDriving/DrivingBoxAp.cpp)
    target_link_libraries(demo_DrivingBoxAp adolc PkgConfig::ipopt)
    add_omplapp_demo(demo_DrivingBoxApAABB RandomScenariosDriving/DrivingBoxApAABB.cpp)
    target_link_libraries(demo_DrivingBoxApAABB adolc PkgConfig::ipopt)

    add_omplapp_demo(demo_DrivingLocalCollision RandomScenariosDriving/DrivingLocalCollision.cpp)
    target_link_libraries(demo_DrivingLocalCollision adolc PkgConfig::ipopt)
    add_omplapp_demo(demo_DrivingBox RandomScenariosDriving/DrivingBox.cpp)
    target_link_libraries(demo_DrivingBox adolc PkgConfig::ipopt)

    add_omplapp_demo(demo_DrivingBoxCompare RandomScenariosDriving/DrivingBoxCompare.cpp)
    target_link_libraries(demo_DrivingBoxCompare adolc PkgConfig::ipopt)
    add_omplapp_demo(demo_DrivingBoxCompareAABB RandomScenariosDriving/DrivingBoxCompareAABB.cpp)
    target_link_libraries(demo_DrivingBoxCompareAABB adolc PkgConfig::ipopt)
    add_omplapp_demo(demo_DrivingActivePoints RandomScenariosDriving/DrivingActivePoints.cpp)
    add_omplapp_demo(demo_DrivingPureSampling RandomScenariosDriving/DrivingPureSampling.cpp)

    add_omplapp_demo(demo_OvertakingBox Overtaking/OvertakingBox.cpp)
    target_link_libraries(demo_OvertakingBox adolc PkgConfig::ipopt)

    add_omplapp_demo(demo_OvertakingBoxAp Overtaking/OvertakingBoxAp.cpp)
    target_link_libraries(demo_OvertakingBoxAp adolc PkgConfig::ipopt)

    add_omplapp_demo(demo_OvertakingBoxCompare Overtaking/OvertakingBoxCompare.cpp)
    target_link_libraries(demo_OvertakingBoxCompare adolc PkgConfig::ipopt)

    add_omplapp_demo(demo_OvertakingStageOne Overtaking/OvertakingStageOne.cpp)
    target_link_libraries(demo_OvertakingStageOne adolc PkgConfig::ipopt)

    add_omplapp_demo(demo_create_geometries BulletTest/create_geometries_example.cpp)
    add_omplapp_demo(demo_parse_mesh BulletTest/parse_mesh_example.cpp)
    add_omplapp_demo(demo_box_box BulletTest/box_box.cpp)
    add_omplapp_demo(demo_RandomScenarioCreation RandomPolygonCircles/RandomScenarioCreation.cpp)
    #add_omplapp_demo(demo_RandomScenarioAABB RandomPolygonCircles/RandomScenarioAABB.cpp RandomPolygonCircles/RandomScenarioRead.cpp)
    add_omplapp_demo(demo_RandomScenarioReal2CSpace RandomPolygonCircles/RandomScenarioReal2CSpace.cpp RandomPolygonCircles/RandomScenarioRead.cpp)
    add_omplapp_demo(demo_DistanceCalculation Distance/DistanceCalculation.cpp)

    add_omplapp_demo(demo_Exam MessTest/Exam.cpp)
    add_omplapp_demo(demo_TestBinaryHeap MessTest/TestBinaryHeap.cpp)
    add_omplapp_demo(demo_TestGrid MessTest/TestGrid.cpp)
    #add_omplapp_demo(demo_TestDiscretization MessTest/TestDiscretization.cpp)
    add_omplapp_demo(demo_TestVectorErase MessTest/TestVectorErase.cpp)
    add_omplapp_demo(demo_TestGraphRemoveNodeVSClearAdd MessTest/TestGraphRemoveNodeVSClearAdd.cpp)

    #add_omplapp_demo(demo_SimpleConfigurationCertificate
        #SimpleConfigurationCertificate/SimpleConfigurationCertificate.cpp
        #SimpleConfigurationCertificate/SimpleCertificateStateValidityChecker.h
        #SimpleConfigurationCertificate/SimpleCertificateStateValidityChecker.cpp
        #SimpleConfigurationCertificate/CSCRRT.h
        #SimpleConfigurationCertificate/CSCRRT.h
        #SimpleConfigurationCertificate/CSCRRT.cpp
        #SimpleConfigurationCertificate/CSCRRTstar.h
        #SimpleConfigurationCertificate/CSCRRTstar.cpp
        #SimpleConfigurationCertificate/CSCRRTConnect.h
        #SimpleConfigurationCertificate/CSCRRTConnect.cpp)

    #add_omplapp_demo(demo_SimpleConfigurationBenchmark
        #SimpleConfigurationCertificate/SimpleConfigurationBenchmark.cpp
        #SimpleConfigurationCertificate/SimpleCertificateStateValidityChecker.h
        #SimpleConfigurationCertificate/SimpleCertificateStateValidityChecker.cpp
        #SimpleConfigurationCertificate/CSCRRT.h
        #SimpleConfigurationCertificate/CSCRRT.cpp
        #SimpleConfigurationCertificate/CSCRRTstar.h
        #SimpleConfigurationCertificate/CSCRRTstar.cpp
        #SimpleConfigurationCertificate/CSCRRTConnect.h
        #SimpleConfigurationCertificate/CSCRRTConnect.cpp)

    add_omplapp_demo(demo_RandomPolygonCircles RandomPolygonCircles/RandomPolygonCirclesPlanning.cpp RandomPolygonCircles/NaiveConnect.h RandomPolygonCircles/NaiveConnect.cpp)
    add_omplapp_demo(demo_RandomPolygonCirclesBenchmark RandomPolygonCircles/RandomPolygonCirclesBenchmark.cpp)
    add_omplapp_demo(demo_RealVector2RigidBodyPlanning RealVector2RigidBodyPlanning/RealVector2RigidBodyPlanning.cpp)
    add_omplapp_demo(demo_NarrowPassages RealVector2RigidBodyPlanning/NarrowPassages.cpp)
    add_omplapp_demo(demo_NarrowPassagesBenchmark RealVector2RigidBodyPlanning/NarrowPassagesBenchmark.cpp)

    add_omplapp_demo(demo_MaxDistance SE2MaxDistance.cpp)
    add_omplapp_demo(demo_Box2dSafetyCertificateChecker Box2dSafetyCertificateChecker.cpp)  
    add_omplapp_demo(demo_MSE2RigidBodyPlanning SE2RigidBodyPlanning/MSE2RigidBodyPlanning.cpp)
    add_omplapp_demo(demo_MSE2MultiRigidBodyPlanning SE2RigidBodyPlanning/MSE2MultiRigidBodyPlanning.cpp)
    add_omplapp_demo(demo_MSE2Benchmark SE2RigidBodyPlanning/MSE2Benchmark.cpp)
    add_omplapp_demo(demo_MSE2MultiBenchmark SE2RigidBodyPlanning/MSE2MultiBenchmark.cpp)
    add_omplapp_demo(demo_MSE2RRTBenchmark SE2RigidBodyPlanning/MSE2RRTBenchmark.cpp)

    add_omplapp_demo(demo_SE3MaxDistance SE3MaxDistance.cpp)
    add_omplapp_demo(demo_SE3SafetyCertificateChecker SE3SafetyCertificateChecker.cpp)  
    add_omplapp_demo(demo_MSE3RigidBodyPlanning SE3RigidBodyPlanning/MSE3RigidBodyPlanning.cpp)
    add_omplapp_demo(demo_MSE3Benchmark SE3RigidBodyPlanning/MSE3Benchmark.cpp)
    add_omplapp_demo(demo_MSE3RRTBenchmark SE3RigidBodyPlanning/MSE3RRTBenchmark.cpp)

    add_omplapp_demo(demo_CollisionDataCreationPoint DeepCollision/CollisionDataCreationPoint.cpp)
    add_omplapp_demo(demo_CollisionDataCreationShape DeepCollision/CollisionDataCreationShape.cpp)

    add_executable(demo_CollisionDataCreationHybridShape DeepCollision/CollisionDataCreationHybridShape.cpp)
    target_link_libraries(demo_CollisionDataCreationHybridShape ${OMPLAPP_DEMO_LIBRARIES}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
        ${Boost_PROGRAM_OPTIONS_LIBRARY} ${HDF5_CXX_LIBRARIES} ${HDF5_LIBRARIES})

    add_executable(demo_CollisionDataCreationHybridHybridShape DeepCollision/CollisionDataCreationHybridHybridShape.cpp)
    target_link_libraries(demo_CollisionDataCreationHybridHybridShape ${OMPLAPP_DEMO_LIBRARIES}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
        ${Boost_PROGRAM_OPTIONS_LIBRARY} ${HDF5_CXX_LIBRARIES} ${HDF5_LIBRARIES})

    # create executable for planning with OPENDE
    if (ODE_FOUND AND DRAWSTUFF_FOUND AND GLUT_INCLUDE_DIR AND GLUT_LIBRARIES)
        if(APPLE)
            set_package_properties(GLUT PROPERTIES
                URL "https://www.opengl.org/resources/libraries/glut/"
                PURPOSE "Used for one ODE demo program on macOS.")
            find_package(GLUT)
        endif(APPLE)
        aux_source_directory(OpenDEPlanning OpenDEPlanning_SRC)
        add_executable(demo_OpenDEPlanning ${OpenDEPlanning_SRC})
        target_include_directories(demo_OpenDEPlanning PRIVATE ${ODE_INCLUDE_DIRS} ${DRAWSTUFF_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
        target_link_libraries(demo_OpenDEPlanning ${DRAWSTUFF_LIBRARY} ${ODE_LIBRARIES} ${OMPLAPP_DEMO_LIBRARIES} ${GLUT_LIBRARIES})
    endif()
endif(OMPL_BUILD_DEMOS)

file(GLOB OMPLAPP_DEMO_PY_FILES "*/*.py")
install_python(PROGRAMS ${OMPLAPP_DEMO_PY_FILES}
  DESTINATION "${OMPL_DEMO_INSTALL_DIR}"
  COMPONENT omplapp)
file(GLOB OMPLAPP_DEMO_CXX_FILES "*/*.cpp")
install(FILES ${OMPLAPP_DEMO_CXX_FILES}
    DESTINATION "${OMPL_DEMO_INSTALL_DIR}"
    COMPONENT omplapp)
