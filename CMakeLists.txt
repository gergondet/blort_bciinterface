cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE(FindPkgConfig)
pkg_check_modules(BCIINTERFACE bci-interface)
IF(BCIINTERFACE_FOUND)
EXEC_PROGRAM(${PKG_CONFIG_EXECUTABLE} ARGS bci-interface --cflags
  OUTPUT_VARIABLE BCIINTERFACE_CFLAGS)
MESSAGE(STATUS "BCIINTERFACE_CFLAGS: ${BCIINTERFACE_CFLAGS}")
EXEC_PROGRAM(${PKG_CONFIG_EXECUTABLE} ARGS bci-interface --libs
  OUTPUT_VARIABLE BCIINTERFACE_LIBS)
MESSAGE(STATUS "BCIINTERFACE_LIBS: ${BCIINTERFACE_LIBS}")
ELSE(BCIINTERFACE_FOUND)
MESSAGE(STATUS "bci-interface not found")
ENDIF(BCIINTERFACE_FOUND)

SET(CVEP_STIMULUS_FOUND 0)
FIND_PATH(HAS_CVEP_STIMULUS name CVEPStimulus.h PATHS ${BCIINTERFACE_INCLUDE_DIRS}/bci-interface/DisplayObject)
IF(HAS_CVEP_STIMULUS STREQUAL "HAS_CVEP_STIMULUS-NOTFOUND")
MESSAGE("Building without CVEP stimuli support")
ELSE()
MESSAGE(STATUS "Building with CVEP stimuli support")
ADD_DEFINITIONS("-DHAS_CVEP_SUPPORT")
SET(CVEP_STIMULUS_FOUND 1)
ENDIF()
IF(CVEP_STIMULUS_FOUND EQUAL 1)
SET(CVEP_LIB cvep-stimuli)
ELSE()
SET(CVEP_LIB "")
ENDIF()

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

ADD_DEFINITIONS(${BCIINTERFACE_CFLAGS})
LINK_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/lib)
rosbuild_add_boost_directories()

LINK_DIRECTORIES(${BCIINTERFACE_LIBRARY_DIRS})

rosbuild_add_library(blortobject src/BLORTObject.cpp src/BLORTObjectsManager.cpp)
target_link_libraries(blortobject ${CVEP_LIB} bci-interface sfml-oculus sfml-graphics sfml-window sfml-system module_tracker module_tomgine GL)

rosbuild_add_executable(example src/example.cpp)
target_link_libraries(example blortobject rosbackground bci-interface sfml-oculus sfml-graphics sfml-window sfml-system module_tracker module_tomgine)
rosbuild_link_boost(example thread)

rosbuild_add_executable(example-oculus src/example-oculus.cpp)
target_link_libraries(example-oculus blortobject rosbackground bci-interface sfml-oculus sfml-graphics sfml-window sfml-system module_tracker module_tomgine)
rosbuild_link_boost(example-oculus thread)

rosbuild_add_executable(minimal src/minimal.cpp)
target_link_libraries(minimal module_tracker module_tomgine sfml-graphics sfml-window sfml-system GL)
rosbuild_link_boost(minimal thread)

INSTALL(FILES src/BLORTObject.h src/BLORTObjectsManager.h DESTINATION ${BCIINTERFACE_INCLUDE_DIRS}/bci-interface/DisplayObject)
INSTALL(TARGETS blortobject DESTINATION ${BCIINTERFACE_LIBRARY_DIRS})
