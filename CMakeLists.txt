# Replace the replay and replay to build the executables

project(stsSynthesis,)
cmake_minimum_required(VERSION 3.2)

set( PROJ_SOURCES "src/replay.cpp" "src/modelFunsCommon.cpp" "src/modelFunsStanding.cpp" "src/modelFunsSitting.cpp" "src/parameterization.cpp" 
					"src/RegisterTypes_osimPlugin.cpp" "src/seatConstraintHandler.cpp" "src/utility.cpp")

set( PROJ_INC "inc/" )

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

include_directories(
	${PROJ_INC}
	/home/vinay/libs/opensim_dependencies_install/spdlog/include
	/home/vinay/libs/opensim_dependencies_install/simbody/include/simbody
	/home/vinay/libs/opensim-core-4.3-install/include
	/home/vinay/libs/opensim-core-4.3-install/include/OpenSim
	/home/vinay/libs/libcmaes/src	
	/home/vinay/libs/libcmaes/build/src
	/usr/include/eigen3)

link_directories(
	/home/vinay/libs/opensim-core-4.3-install/lib
	/home/vinay/libs/opensim_dependencies_install/spdlog/lib
	/home/vinay/libs/opensim_dependencies_install/simbody/lib
	/home/vinay/libs/libcmaes/build/src)

add_executable( replay ${PROJ_SOURCES} )

target_link_libraries( replay
osimActuators
osimAnalyses
osimCommon
osimSimulation
osimTools
SimTKcommon
SimTKmath
SimTKsimbody
cmaes
pthread
)