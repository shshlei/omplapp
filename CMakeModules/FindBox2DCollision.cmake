# Find Box2DCollision
# ----------
#
# Finds the Box2DCollision library. This module defines:
#
#  Box2DCollision_FOUND             - True if Box2DCollision library is found
#  Box2DCollision::Box2DCollision   - Box2DCollision imported target
#
# Additionally these variables are defined for internal usage:
#
#  Box2DCollision_LIBRARY        - Box2DCollision library
#  Box2DCollision_INCLUDE_DIR    - Include dir
#

# Library
find_library(Box2DCollision_LIBRARY
    NAMES
        box2d_collision)

# Include dir
find_path(Box2DCollision_INCLUDE_DIR
    NAMES
        box2d_collision/b2_bvh_manager.h)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Box2DCollision DEFAULT_MSG
    Box2DCollision_LIBRARY
    Box2DCollision_INCLUDE_DIR)

mark_as_advanced(FORCE
    Box2DCollision_LIBRARY
    Box2DCollision_INCLUDE_DIR)

if(NOT TARGET Box2DCollision::Box2DCollision)
    add_library(Box2DCollision::Box2DCollision UNKNOWN IMPORTED)
    set_target_properties(Box2DCollision::Box2DCollision PROPERTIES
        IMPORTED_LOCATION ${Box2DCollision_LIBRARY}
        INTERFACE_INCLUDE_DIRECTORIES ${Box2DCollision_INCLUDE_DIR})
endif()
