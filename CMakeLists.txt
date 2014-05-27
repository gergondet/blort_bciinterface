cmake_minimum_required(VERSION 2.4.6)
project(blort_bciinterface)

find_package(catkin REQUIRED COMPONENTS roscpp blort blort_ros bciinterface_rosbackground)
include_directories(${catkin_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

include(FindPkgConfig)
pkg_check_modules(BCIINTERFACE bci-interface)
if(${BCIINTERFACE_LIBRARY_DIRS})
include_directories(${BCIINTERFACE_INCLUDE_DIRS})
link_directories(${BCIINTERFACE_LIBRARY_DIRS})
else()
set(BCIINTERFACE_INCLUDE_DIRS /usr/include)
set(BCIINTERFACE_LIBRARY_DIRS /usr/lib)
endif()


include_directories(include)

set(CVEP_STIMULUS_FOUND 0)
find_path(HAS_CVEP_STIMULUS NAME CVEPStimulus.h PATHS ${BCIINTERFACE_INCLUDE_DIRS}/bci-interface/DisplayObject)
if(HAS_CVEP_STIMULUS STREQUAL "HAS_CVEP_STIMULUS-NOTFOUND")
message("Building without CVEP stimuli support")
else()
message(STATUS "Building with CVEP stimuli support")
add_definitions("-DHAS_CVEP_SUPPORT")
set(CVEP_STIMULUS_FOUND 1)
endif()
if(CVEP_STIMULUS_FOUND EQUAL 1)
set(CVEP_LIB cvep-stimuli)
else()
set(CVEP_LIB "")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES blortobject
)

add_library(blortobject src/BLORTObject.cpp src/BLORTObjectsManager.cpp)
target_link_libraries(blortobject ${CVEP_LIB} ${BCIINTERFACE_LIBRARIES} ${catkin_LIBRARIES} module_tomgine module_tracker)

add_executable(example src/example.cpp)
target_link_libraries(example blortobject rosbackground ${catkin_LIBRARIES} ${Boost_LIBRARIES} sfml-graphics)

add_executable(example-oculus src/example-oculus.cpp)
target_link_libraries(example-oculus blortobject rosbackground ${catkin_LIBRARIES} ${Boost_LIBRARIES} sfml-graphics)

add_executable(minimal src/minimal.cpp)
target_link_libraries(minimal ${catkin_LIBRARIES} ${Boost_LIBRARIES} sfml-graphics sfml-window sfml-system GL module_tomgine module_tracker)

INSTALL(FILES include/bci-interface/DisplayObject/BLORTObject.h include/bci-interface/DisplayObject/BLORTObjectsManager.h DESTINATION ${BCIINTERFACE_INCLUDE_DIRS}/bci-interface/DisplayObject)
INSTALL(TARGETS blortobject DESTINATION ${BCIINTERFACE_LIBRARY_DIRS})
