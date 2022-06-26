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

    add_omplapp_demo(demo_create_geometries create_geometries_example.cpp)
    add_omplapp_demo(demo_parse_mesh parse_mesh_example.cpp)
    add_omplapp_demo(demo_box_box box_box.cpp)
    add_omplapp_demo(demo_Exam Exam.cpp)
    add_omplapp_demo(demo_RandomScenario RandomScenario.cpp)
    add_omplapp_demo(demo_ReadRandomScenario ReadRandomScenario.cpp)

    add_omplapp_demo(demo_TestBinaryHeap TestBinaryHeap.cpp)
    add_omplapp_demo(demo_TestGrid TestGrid.cpp)
    add_omplapp_demo(demo_TestDiscretization TestDiscretization.cpp)
    add_omplapp_demo(demo_TestVectorErase TestVectorErase.cpp)
    add_omplapp_demo(demo_TestGraphRemoveNodeVSClearAdd TestGraphRemoveNodeVSClearAdd.cpp)

    add_omplapp_demo(demo_RandomPolygonCircles RandomPolygonCircles/RandomPolygonCirclesPlanning.cpp)
    add_omplapp_demo(demo_RealVector2RigidBodyPlanning RealVector2RigidBodyPlanning/RealVector2RigidBodyPlanning.cpp)

    add_omplapp_demo(demo_MaxDistance SE2MaxDistance.cpp)
    add_omplapp_demo(demo_Box2dSafetyCertificateChecker Box2dSafetyCertificateChecker.cpp)  
    add_omplapp_demo(demo_Box2dSafetyCertificateChecker2 Box2dSafetyCertificateChecker2.cpp)
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
