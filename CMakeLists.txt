cmake_minimum_required(VERSION 3.5)
project(omplapp VERSION 1.5.2 LANGUAGES CXX)
set(OMPL_ABI_VERSION 16)

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
    # By default, use Release mode
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Type of build" FORCE)

    # On 32bit architectures, use RelWithDebInfo
    if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_SIZEOF_VOID_P EQUAL 4)
        set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Type of build" FORCE)
    endif()
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(POLICY CMP0012)
  cmake_policy(SET CMP0012 NEW)
endif()

set(CMAKE_MODULE_PATH
    "${CMAKE_MODULE_PATH}"
    "${CMAKE_ROOT_DIR}/cmake/Modules"
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules"
    "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules")
include(GNUInstallDirs)
include(FeatureSummary)
include(CompilerSettings)
include(OMPLUtils)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

set(OMPLAPP_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src;${CMAKE_CURRENT_BINARY_DIR}/src")
set(OMPLAPP_RESOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/resources")
set(OMPL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/ompl/src;${CMAKE_CURRENT_BINARY_DIR}/ompl/src")
set(OMPL_DEMO_INSTALL_DIR "${CMAKE_INSTALL_DATAROOTDIR}/ompl/demos"
    CACHE STRING "Relative path to directory where demos will be installed")
set(OMPL_CMAKE_UTIL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules"
    CACHE FILEPATH "Path to directory with auxiliary CMake scripts for OMPL")

if(MSVC)
    add_definitions(-DBOOST_ALL_NO_LIB)
    add_definitions(-DBOOST_PROGRAM_OPTIONS_DYN_LINK)
endif(MSVC)
# Ensure dynamic linking with boost unit_test_framework
add_definitions(-DBOOST_TEST_DYN_LINK)
# Avoid valgrind error due to overflow error, cf. https://github.com/ompl/ompl/issues/664
add_definitions(-DBOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS)

set_package_properties(Boost PROPERTIES
    URL "http://boost.org"
    PURPOSE "Used throughout OMPL for data serialization, graphs, etc.")
set(Boost_USE_MULTITHREADED ON)
set(Boost_NO_BOOST_CMAKE ON)
find_package(Boost 1.58 QUIET REQUIRED COMPONENTS serialization filesystem system program_options)

# on macOS we need to check whether to use libc++ or libstdc++ with clang++
if(CMAKE_CXX_COMPILER_ID MATCHES "^(Apple)?Clang$")
    include(GetPrerequisites)
    get_prerequisites("${Boost_SYSTEM_LIBRARY}" _libs 0 0 "/" "")
    set(CXXSTDLIB "")
    foreach(_lib ${_libs})
        if(_lib MATCHES "libc\\+\\+")
            set(CXXSTDLIB "libc++")
        elseif(_lib MATCHES "libstdc\\+\\+")
            set(CXXSTDLIB "libstdc++")
        endif()
    endforeach()
    if(CXXSTDLIB)
        add_definitions(-stdlib=${CXXSTDLIB})
    endif()
endif()

set_package_properties(Eigen3 PROPERTIES
    URL "http://eigen.tuxfamily.org"
    PURPOSE "A linear algebra library used throughout OMPL.")
find_package(Eigen3 REQUIRED)
set_package_properties(Triangle PROPERTIES
    URL "http://www.cs.cmu.edu/~quake/triangle.html"
    PURPOSE "Used to create triangular decompositions of polygonal 2D environments.")
find_package(Triangle QUIET)
set_package_properties(flann PROPERTIES
    URL "https://github.com/mariusmuja/flann"
    PURPOSE "If detetected, FLANN can be used for nearest neighbor queries by OMPL.")
find_package(flann 1.8.3 QUIET)
set_package_properties(ODE PROPERTIES
    URL "http://ode.org"
    PURPOSE "The Open Dynamics Engine can be used as a \"black box\" for kinodynamic planning.")
find_package(ODE QUIET)
set_package_properties(Drawstuff PROPERTIES
    URL "http://ode.org"
    PURPOSE "Part of the ODE source distribution, used in one demo program.")
find_package(Drawstuff QUIET)
set_package_properties(assimp PROPERTIES
    URL "http://assimp.org"
    PURPOSE "Used in ompl_app for reading meshes representing robots and environments.")
find_package(assimp REQUIRED)
set_package_properties(ccd PROPERTIES
    URL "https://github.com/danfis/libccd"
    PURPOSE "Collision detection library used by fcl.")
find_package(ccd REQUIRED)
set_package_properties(fcl PROPERTIES
    URL "https://github.com/flexible-collision-library/fcl"
    PURPOSE "The default collision checking library.")
find_package(octomap QUIET)
find_package(fcl 0.7 REQUIRED)
#add_definitions(-DBT_USE_DOUBLE_PRECISION)
find_package(Bullet REQUIRED)
find_package(Box2DCollision REQUIRED)
set_package_properties(Threads PROPERTIES
    URL "https://en.wikipedia.org/wiki/POSIX_Threads"
    PURPOSE "Pthreads is sometimes needed, depending on OS / compiler.")
find_package(Threads QUIET)
set_package_properties(Doxygen PROPERTIES
    URL "http://doxygen.org"
    PURPOSE "Used to create the OMPL documentation (i.e., http://ompl.kavrakilab.org).")
find_package(Doxygen)

include_directories(SYSTEM "${Boost_INCLUDE_DIR}")
include_directories(
    "${OMPLAPP_INCLUDE_DIRS}"
    "${OMPL_INCLUDE_DIRS}"
    "${EIGEN3_INCLUDE_DIR}"
    "${FCL_INCLUDE_DIRS}"
    "${BULLET_INCLUDE_DIRS}"
    "${Box2DCollision_INCLUDE_DIR}"
    "${ASSIMP_INCLUDE_DIRS}")

set(OMPLAPP_MODULE_LIBRARIES
    ${ASSIMP_LIBRARY}
    ${FCL_LIBRARIES}
    ${BULLET_LIBRARIES}
    ${Box2DCollision_LIBRARY})
set(OMPLAPP_LIBRARIES
    ${ASSIMP_LIBRARIES}
    ${FCL_LIBRARIES}
    ${Box2DCollision_LIBRARY}
    ${BULLET_LIBRARIES})
link_directories(${ASSIMP_LIBRARY_DIRS} ${CCD_LIBRARY_DIRS} ${OCTOMAP_LIBRARY_DIRS} ${FCL_LIBRARY_DIRS})

set(OMPL_EXTENSION_TRIANGLE ${TRIANGLE_FOUND})
if (OMPL_EXTENSION_TRIANGLE)
  include_directories(${TRIANGLE_INCLUDE_DIR})
endif()

if (flann_FOUND)
    set(OMPL_HAVE_FLANN 1)
    include_directories("${FLANN_INCLUDE_DIRS}")
    link_directories(${FLANN_LIBRARY_DIRS})
endif()

set(OMPL_EXTENSION_ODE ${ODE_FOUND})
if (OMPL_EXTENSION_ODE)
    include_directories("${ODE_INCLUDE_DIRS}")
endif()

set_package_properties(PQP PROPERTIES
    URL "http://gamma.cs.unc.edu/SSV/"
    PURPOSE "Used as an alternative, additional collision checking library (the default is FCL).")
find_package(PQP QUIET)
if(PQP_FOUND)
    include_directories("${PQP_INCLUDE_DIR}")
    set(OMPLAPP_MODULE_LIBRARIES ${OMPLAPP_MODULE_LIBRARIES} ${PQP_LIBRARY})
    set(OMPLAPP_LIBRARIES ${OMPLAPP_LIBRARIES} ${PQP_LIBRARY})
endif()

add_subdirectory(ompl/doc)
add_subdirectory(ompl/src)
#add_subdirectory(ompl/demos)
add_subdirectory(doc)
add_subdirectory(src)
add_subdirectory(demos)
#add_subdirectory(benchmark)

install(DIRECTORY resources
    DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/ompl"
    COMPONENT omplapp
    PATTERN ".DS_Store" EXCLUDE)

include(CMakePackageConfigHelpers)
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})
set(LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
configure_package_config_file(ompl/omplConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/omplConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/ompl/cmake
    PATH_VARS INCLUDE_INSTALL_DIR LIB_INSTALL_DIR
    NO_CHECK_REQUIRED_COMPONENTS_MACRO)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/omplConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/omplConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/omplConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/ompl/cmake
    COMPONENT ompl)

# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)
add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

include(CPackSettings)

set_package_properties(PkgConfig PROPERTIES
    URL "https://www.freedesktop.org/wiki/Software/pkg-config/"
    PURPOSE "Used to find (compilation flags for) dependencies.")
feature_summary(DESCRIPTION INCLUDE_QUIET_PACKAGES WHAT ALL)
